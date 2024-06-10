#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import sys
import time
from simple_pid import PID
from geometry_msgs.msg import PointStamped
sys.path.append('/home/ubuntu/depthai_blazepose')

from BlazeposeDepthai import BlazeposeDepthai
import matplotlib
import matplotlib.pyplot as plt

#todo
# 1) DECIDE HOW TO FIX x
# 2) add visulaization if possible
# 3) filter lidar data to reduce jerks

class SkeletalFollower(Node):
    def __init__(self, is_pid):
        super().__init__("skeletal_follower")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist,"/cmd_vel",10)
        #self.subscriber_blazepose = self.create_subscription(PointStamped, "/BLAZEPOSE/pose", self.skeleton_callback,10)
        self.subscriber_blazepose = self.create_timer(0.5,  self.skeleton_callback)
        self.subscriber_lidar = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.controller = self.create_timer(0.5, self.control_loop)
        self.image_width = 1792
        self.twist = Twist()
        self.image_height = 1008
        self.prev_Z = 357
        self.lidar_camera_offset = -np.pi / 2  # 90 degrees offset in radians
        self.lidar_ranges = None
        self.angle_increment = None
        self.angle_min = None
        self.horizontal_fov = 69.0  # HFOV FROM DOCS
        # Initialize PID controllers
        self.pid_linear = PID(-0.5, -0.001, -0.0000, setpoint=3)  # target distance Z = 800
        self.pid_angular = PID(0.001, 0.00001, 0, setpoint=0)  # target horizontal position X = 0
        self.pid_linear.output_limits = (0, 5)  # linear speed limits
        self.pid_angular.output_limits = (-0.5, 0.5)  # angular speed limits
        self.epsilon_linear = 0.1
        self.epsilon_angular = 100
        ##params for filter, timer
        self.knee_joint = None
        self.window = []
        self.window_size = 5 ## to be tune of how many values you wish to take average data
        self.start = None
        self.duration = 5 ##plan is to start searching only when no human seen for 5sec

        # Initialize BlazeposeDepthai
        self.tracker = BlazeposeDepthai(input_src="rgb", lm_model="lite", xyz=False)

        ##decesion tree params
        self.alpha = 0.05
        self.K = 1

        ##PLOTTING FEATURES
        # Real-time plotting setup
        self.linear_errors = []
        self.angular_errors = []
        self.times = []
        self.start_time_plot = time.time()

        self.is_pid = is_pid
    

    ##todo
    #1 implement simple decesion tress following paper
    #2 try out basic P Pd 
    def control_loop(self):
        if self.lidar_ranges is None:
            self.get_logger().warn("waiting for pose")
            self.twist.linear.x = 0.0  # STOP
            self.twist.angular.z = 0.0  
        else :
            #self.get_logger().error(f" knee joint = {self.knee_joint}")
            ##get pose of frame centre
            poseX = self.image_width//2
            if self.knee_joint is not None:

                X_knee = self.knee_joint[0]
                angle = self.calculate_angle(X_knee)
                ##using lidar
                index , z = self.get_lidar_distance(angle)[:]
                
                ##using camera depth
                #z = self.knee_joint[2]

                if not math.isinf(z):
                    Z = z
                else :
                    Z = self.prev_Z   

                ##moving average filter
                Z = self.filter(Z)


                X = X_knee - poseX
                #self.get_logger().info(f"X = {X}  Z = {Z} angle = {angle}rad index = {index}")

                  # Calculate the error
                linear_error = abs(Z - self.pid_linear.setpoint)
                angular_error = abs(X - self.pid_angular.setpoint)

                if self.is_pid:
                    # Check if the error is within the acceptable range
                    if linear_error <= self.epsilon_linear or Z < self.pid_linear.setpoint: ##stop if ;less than desired distance or error converged
                        self.twist.linear.x = 0.0  # STOP
                    else:
                        linear_velocity = self.pid_linear(Z) ##call contis None or self.knee_joint is")
                        self.twist.linear.x =  float(linear_velocity)
                        self.get_logger().info(f"vel = {linear_velocity}")


                    if angular_error <= self.epsilon_angular:
                        self.twist.angular.z = 0.0  # STOP
                    else:
                        angular_velocity = self.pid_angular(X)
                        self.twist.angular.z = float(angular_velocity)
                        self.get_logger().info(f"omega = {angular_velocity}")
                else:
                    self.call_decision_tree(X,Z)

                self.get_logger().info(f"X = {X}, z = {Z}")
                ##update previos
                self.prev_Z = Z   
                self.start = None

                ##plotting stuff
                 # Update errors for plotting
                current_time = time.time() - self.start_time_plot
                self.times.append(current_time)
                self.linear_errors.append(linear_error)
                self.angular_errors.append(angular_error)
                self.update_plot()

            else:  ##plan is to stop for 5sec if no skeleton and then start scanning back
                if self.start is None:
                    self.start = time.time()

                elapsed_time = time.time() - self.start
                if elapsed_time >= self.duration:
                    self.twist.angular.z = 0.3  # Start rotating
                    self.get_logger().warn("scanning for bodies")
                else:
                    self.twist.linear.x = 0.0  # STOP
                    self.twist.angular.z = 0.0  # STOP
                    self.get_logger().warn("No body detected, stopping for 5 seconds")
    
             ##publish
        self.publisher.publish(self.twist)

    def update_plot(self):
        plt.figure(figsize=(10, 6))

        plt.subplot(2, 1, 1)
        plt.plot(self.times, self.linear_errors, label='Linear Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Error')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(self.times, self.angular_errors, label='Angular Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Error')
        plt.legend()

        plt.tight_layout()
        plt.savefig('/home/ubuntu/tests/error_plot.png')
        plt.close()

    ##get rgb image coordinates and red point info
    # def skeleton_callback(self, msg: PointStamped):
    #     if msg is not None:
    #         self.knee_joint = msg.point
    #         self.get_logger().info("I can see your knees")
    #     else:
    #         self.knee_joint = None

    def skeleton_callback(self):
        self.knee_joint = self.detect_body()  #list of x, y z pose

    def detect_body(self):
        # Run Blazepose on the next frame
        frame, body = self.tracker.next_frame()
        if body:
            # Extract the coordinates of landmark 25 (knee)
            landmark_25 = body.landmarks[25]
            landmark_26 = body.landmarks[26]
            ##take a point in middle of knnes
            return (landmark_25 + landmark_26)/2.0
        else:
            self.get_logger().warn("no skeleton here")
            return None
    
    def lidar_callback(self, msg: LaserScan):
        #self.get_logger().info("I am bieng called")
        self.lidar_ranges = msg.ranges  
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min

    def calculate_angle(self, x):
        # Calculate the angle of the centroid from the image center
        center_x = self.image_width // 2
        pixel_offset = x - center_x ##basically X
        ## total angle covered by distance over which angle covered
        angle_per_pixel = (self.horizontal_fov / self.image_width) * (np.pi / 180)  # Convert to radians
        angle = pixel_offset * angle_per_pixel
        return angle

    ##currently limited to height of lidar can't find distance of above points
    def get_lidar_distance(self, angle):
        if self.lidar_ranges is None:
            return None
        # lidar aligned at 90 deg anti clockwise wrt to camera
        adjusted_angle = angle + self.lidar_camera_offset  # Adjust with offset
        # Normalize the adjusted angle to be within [-pi pi]
        adjusted_angle = (adjusted_angle + np.pi) % (2 * np.pi) - np.pi
        # Calculate the index in the lidar_ranges array corresponding to the angle
        index = int((adjusted_angle - self.angle_min) / self.angle_increment)
        if 0 <= index < len(self.lidar_ranges):
            return [index, self.lidar_ranges[index]]
        else:
            return None
        
    def filter(self, new_value):
        if len(self.window) >= self.window_size:
            self.window.pop(0)
        self.window.append(new_value)
        return sum(self.window) / len(self.window)
    
    def call_decision_tree(self, X, Z):
        # ICOMITEE 2019 JEMBER (SIMPLE ALGO FOR PERSON FOLLOWING PAPER)
        if  2 <= Z <= 3.5:
            if X <= -100:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.3  # Turn left
            else:
                if X >= 100:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -0.3  # Turn right
                else:
                    self.twist.linear.x = 0.0  # STOP
                    self.twist.angular.z = 0.0
        else:
            if Z < 2:
                self.twist.linear.x = 0.0  # Move backwards
                self.twist.angular.z = 0.0
            else:
                if X <= -100:
                    pwm_left = (30 + Z / self.alpha) / 100.0
                    pwm_right = max(0.0, (30 + Z / self.alpha - ((X + 896) - 996) * self.K) / 100.0)
                    self.twist.linear.x = (pwm_left + pwm_right) / 2
                    self.twist.angular.z = (pwm_right - pwm_left) / 2  # lft => anti => =+ve
                else:
                    if X >= 100:
                        pwm_right = (30 + Z / self.alpha) / 100.0
                        pwm_left = max(0.0, (30 + Z / self.alpha - (796 - (X + 896)) * self.K) / 100.0)
                        self.twist.linear.x = (pwm_left + pwm_right) / 2
                        self.twist.angular.z = (pwm_right - pwm_left) / 2  # right => clock => -ve
                    else:
                        self.twist.linear.x = (30 + Z / self.alpha) / 100.0  # Move forward
                        self.twist.angular.z = 0.0
    

    

def main(args = None):
    rclpy.init(args = args)
    # Use rclpy's argument parsing
    ros_args = rclpy.utilities.remove_ros_args(args)
    #  Parse the visualization argument
    if '--pid' in ros_args:
        is_pid = True
    else:
        is_pid = False
        
    node = SkeletalFollower(is_pid)
    rclpy.spin(node)
    rclpy.shutdown()




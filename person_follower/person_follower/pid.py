#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from simple_pid import PID
import math

##to do
#3tune KP KD KI
##MAY NEED INTEGRAD WINDUP  BUT NO STEADY STAE ERROR


class PID_LIDAR(Node):
    def __init__(self):
        super().__init__("green_point_follower")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist,"/cmd_vel",10)
        self.subscriber_rgb = self.create_subscription(Image , "/color/preview/image", self.image_callback,10)
        self.subscriber_lidar = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.controller = self.create_timer(0.5, self.control_loop)
        self.image_width = None
        self.twist = Twist()
        self.image_height = None
        self.prev_Z = 357
        self.lidar_camera_offset = -np.pi / 2  # 90 degrees offset in radians
        self.lidar_ranges = None
        self.angle_increment = None
        self.angle_min = None
        self.horizontal_fov = 69.0  # HFOV FROM DOCS
        # Initialize PID controllers
        self.pid_linear = PID(-0.05, -0.0001, -0.0000, setpoint=0.7)  # target distance Z = 800
        self.pid_angular = PID(0.0050, 0.00001, 0, setpoint=0)  # target horizontal position X = 0
        self.pid_linear.output_limits = (0, 0.31)  # linear speed limits
        self.pid_angular.output_limits = (-0.5, 0.5)  # angular speed limits
        self.epsilon_linear = 0.1
        self.epsilon_angular = 20
        self.window = []
        self.window_size = 5 ## to be tune of how many values you wish to take average data

    ##todo
    #1 implement simple decesion tress following paper
    #2 try out basic P Pd 
    def control_loop(self):
        if self.lidar_ranges is None or self.image_width is None:
            self.get_logger().warn("waiting for pose")
            self.twist.linear.x = 0.0  # STOP
            self.twist.angular.z = 0.0  
        else :
            ##get pose of frame centre
            poseX = self.image_width//2
            if self.green_point is not None:

                greenX = self.green_point[0]
                angle = self.calculate_angle(greenX)
                index , z = self.get_lidar_distance(angle)[:]

                if not math.isinf(z):
                    Z = z
                else :
                    Z = self.prev_Z   

                X = greenX - poseX
                #self.get_logger().info(f"X = {X}  Z = {Z} angle = {angle}rad index = {index}")

                  # Calculate the error
                linear_error = abs(Z - self.pid_linear.setpoint)
                angular_error = abs(X - self.pid_angular.setpoint)

                # Check if the error is within the acceptable range
                if linear_error <= self.epsilon_linear or Z < self.pid_linear.setpoint: ##stop if ;less than desired distance or error converged
                    self.twist.linear.x = 0.0  # STOP
                else:
                    linear_velocity = self.pid_linear(Z) ##call controller
                    self.twist.linear.x = float(linear_velocity)
                    self.get_logger().info(f"vel = {linear_velocity}")


                if angular_error <= self.epsilon_angular:
                    self.twist.angular.z = 0.0  # STOP
                else:
                    angular_velocity = self.pid_angular(X)
                    self.twist.angular.z = float(angular_velocity)
                    self.get_logger().info(f"omega = {angular_velocity}")
                self.get_logger().info(f"X = {X} error = {angular_error}, z = {Z}")
                ##update previos
                self.prev_Z = Z   

            else:
                self.get_logger().warn("No green point detected but trying my best")
                self.twist.linear.x = 0.0  # STOP
                self.twist.angular.z = 0.3                
    
             ##publish
        self.publisher.publish(self.twist)


    ##get rgb image coordinates and red point info
    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.green_point = self.detect_green_point(cv_image) ##tuple of x , y coordinates of red point
        self.image_height, self.image_width = cv_image.shape[:2]
        # Draw the blue point on the image for visualization
        if self.green_point:
            cv2.circle(cv_image, self.green_point, 10, (255, 0, 0), -1)
            #self.get_logger().info(f"Blue point detected at: {self.blue_point}")
        else:
            self.get_logger().warn("No green point detected")

        # Display the image with the detected blue point
        cv2.imshow("green Point Detection", cv_image)
        cv2.waitKey(1)
    
    def detect_green_point(self, cv_image):
        ##detect red point via masking and contour detection open Cv
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
        return None
    
    def lidar_callback(self, msg: LaserScan):
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

    

def main(args = None):
    rclpy.init(args = args)
    node = PID_LIDAR()
    rclpy.spin(node)
    rclpy.shutdown()

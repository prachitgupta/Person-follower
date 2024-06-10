#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class LidarFollower(Node):
    def __init__(self):
        super().__init__("green_point_follower")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber_rgb = self.create_subscription(Image, "/color/preview/image", self.image_callback, 10)
        self.subscriber_lidar = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.controller = self.create_timer(0.5, self.control_loop)
        self.lidar_ranges = None
        self.angle_increment = None
        self.angle_min = None
        self.image_width = None
        self.twist = Twist()
        self.image_height = None
        self.prev_Z = 357  # currently stored as minimum value 
        self.K = 1  # turning power
        self.alpha = 4  # speed power inversely proportional
        self.horizontal_fov = 69.0  # HFOV FROM DOCS
        self.green_point = None
        self.lidar_camera_offset = -np.pi / 2  # 90 degrees offset in radians

    def control_loop(self):
        if self.lidar_ranges is None or self.image_width is None:
            self.twist.linear.x = 0.0  # STOP
            self.twist.angular.z = 0.0
            self.get_logger().warn("waiting for Pose")
        else:
            # Get pose of frame center
            poseX = self.image_width // 2
            # Get pose of green point
            if self.green_point is not None:
                # Calculate the angle of the green point
                greenX = self.green_point[0]
                angle = self.calculate_angle(greenX)
                index , z = self.get_lidar_distance(angle)[:]

                if not math.isinf(z):
                    Z = z
                else :
                    Z = self.prev_Z
                   

                X = greenX - poseX
                self.get_logger().info(f"X = {X}  Z = {Z} angle = {angle}rad index = {index}")

                # Simple decision tree
                self.call_decision_tree(X, Z)

                ##update previos
                self.prev_Z = Z 

            else:
                self.get_logger().warn("No green point detected but trying my best")
                self.twist.linear.x = 0.0  # STOP
                self.twist.angular.z = 0.3

        #Publish
        self.publisher.publish(self.twist)

    def call_decision_tree(self, X, Z):
        # ICOMITEE 2019 JEMBER (SIMPLE ALGO FOR PERSON FOLLOWING PAPER)
        if  1 <= Z <= 5:
            if X <= -20:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.3  # Turn left
            else:
                if X >= 20:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -0.3  # Turn right
                else:
                    self.twist.linear.x = 0.0  # STOP
                    self.twist.angular.z = 0.0
        else:
            if Z < 1:
                self.twist.linear.x = 0.0  # Move backwards
                self.twist.angular.z = 0.0
            else:
                if X <= -30:
                    pwm_left = (20 + Z / self.alpha) / 100.0
                    pwm_right = max(0.0, (20 + Z / self.alpha - ((X + 320) - 340) * self.K) / 100.0)
                    self.twist.linear.x = (pwm_left + pwm_right) / 2
                    self.twist.angular.z = (pwm_left - pwm_right) / 2  # lft => anti => =+ve
                else:
                    if X >= 30:
                        pwm_right = (20 + Z / self.alpha) / 100.0
                        pwm_left = max(0.0, (20 + Z / self.alpha - (300 - (X + 320)) * self.K) / 100.0)
                        self.twist.linear.x = (pwm_left + pwm_right) / 2
                        self.twist.angular.z = (pwm_left - pwm_right) / 2  # right => clock => -ve
                    else:
                        self.twist.linear.x = (20 + Z / self.alpha) / 100.0  # Move forward
                        self.twist.angular.z = 0.0
            
    

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.green_point = self.detect_green_point(cv_image)  # tuple of x, y coordinates of green point
        self.image_height, self.image_width = cv_image.shape[:2]
        # Draw the green point on the image for visualization
        if self.green_point:
            cv2.circle(cv_image, self.green_point, 10, (255, 0, 0), -1)
        else:
            self.get_logger().warn("No green point detected")

        # Display the image with the detected green point
        cv2.imshow("Green Point Detection", cv_image)
        cv2.waitKey(1)

    def detect_green_point(self, cv_image):
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
                

def main(args=None):
    rclpy.init(args=args)
    node = LidarFollower()
    rclpy.spin(node)
    rclpy.shutdown()


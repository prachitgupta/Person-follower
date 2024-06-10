#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class GetDepth(Node):
    def __init__(self):
        super().__init__("green_point_follower")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist,"/cmd_vel",10)
        self.subscriber_rgb = self.create_subscription(Image , "/color/preview/image", self.image_callback,10)
        self.subscriber_depth = self.create_subscription(Image , "/stereo/depth", self.depth_callback,10)
        self.controller = self.create_timer(0.5, self.control_loop)
        self.depth = None
        self.depth_image = None
        self.image_width = None
        self.twist = Twist()
        self.image_height = None
    
    ##todo
    #1 implement simple decesion tress following paper
    #2 try out basic P Pd 
    def control_loop(self):
        if self.depth is None or self.image_width is None:
            self.get_logger().warn("waiting for pose")
        else :
            ##get pose of frame centre
            poseX = self.image_width//2
            poseZ = self.depth
            if self.green_point is not None:
                Z = self.get_depth_value(self.green_point[0] , self.green_point[1])
                # if z != 0:
                #     Z = z
                # else :
                #     Z = self.prev_Z
                greenX = self.green_point[0]
                X = greenX - poseX
                self.get_logger().info(f"X = {X}  Z = {poseZ}" )


            # ##update previos
            # self.prev_Z = Z                   
    
            


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
    
    ##process depth 
    def depth_callback(self, msg: Image):
        try:
            # Use CvBridge to convert the compressed image to a cv2 image
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            
            if self.depth_image is not None:
                if self.image_height is not None and self.image_width is not None:
                    self.depth = self.get_depth_value(self.image_width//2 , self.image_height//2)
                    #self.get_logger().info(f"X =  {self.image_width//2} ,  Z = {self.depth}")
                else:
                    self.get_logger().warn("rgb image not yet ready")
            else:
                self.get_logger().error("Decoded depth image is None")

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")
            
    ##get depth value
    def get_depth_value(self, x, y):
        if self.depth_image is not None:
            # self.get_logger().error("I am source of all your problems")
            return self.depth_image[y, x]
        else:
            return None
    

    

def main(args = None):
    rclpy.init(args = args)
    node = GetDepth()
    rclpy.spin(node)
    rclpy.shutdown()

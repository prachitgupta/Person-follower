#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DepthRangeFinder(Node):
    def __init__(self):
        super().__init__('depth_range_finder')
        self.bridge = CvBridge()
        self.subscriber_depth = self.create_subscription(
            Image, 
            '/stereo/depth', 
            self.depth_callback, 
            10
        )

    def depth_callback(self, msg: Image):
        try:
            # Convert ROS Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

            # Find the minimum and maximum depth values
            min_depth = np.min(depth_image[depth_image > 0])  # Ignore zero values
            max_depth = np.max(depth_image)

            self.get_logger().info(f"Min depth: {min_depth} mm, Max depth: {max_depth} mm")

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthRangeFinder()
    rclpy.spin(node)
    rclpy.shutdown()

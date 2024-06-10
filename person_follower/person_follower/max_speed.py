#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SpeedTest(Node):
    def __init__(self):
        super().__init__('speed_test')
        
        # QoS profile for the subscriber
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.max_speed = 2  # initial guess
        self.speed_increment = 0.05
        self.commanded_speeds = []
        self.actual_speeds = []

    def timer_callback(self):
        # Increment the target speed
        self.target_speed += self.speed_increment
        if self.target_speed > self.max_speed:
            self.get_logger().info('Reached maximum target speed. Stopping test.')
            self.log_results()
            rclpy.shutdown()
            return
        
        # Publish the target speed
        twist = Twist()
        twist.linear.x = self.target_speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.commanded_speeds.append(self.target_speed)
        
        # Log the current speeds
        self.get_logger().info(f'Commanded Speed: {self.target_speed:.2f} m/s, Actual Speed: {self.current_speed} m/s')
        self.actual_speeds.append(self.current_speed)
    
    def odom_callback(self, msg: Odometry):
        # Extract the actual linear speed from the odometry message
        linear_velocity = msg.twist.twist.linear
        self.current_speed =  linear_velocity.y
        #self.get_logger().info(f"{linear_velocity.x}")
    
    def log_results(self):
        self.get_logger().info('Commanded Speeds (m/s): ' + ', '.join(f'{speed:.2f}' for speed in self.commanded_speeds))
        self.get_logger().info('Actual Speeds (m/s): ' + ', '.join(f'{speed:.2f}' for speed in self.actual_speeds))

def main(args=None):
    rclpy.init(args=args)
    node = SpeedTest()
    rclpy.spin(node)
    rclpy.shutdown()


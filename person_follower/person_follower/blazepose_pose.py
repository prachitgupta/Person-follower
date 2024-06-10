import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import sys
sys.path.append('/home/ubuntu/depthai_blazepose')
import cv2
from cv_bridge import CvBridge
from BlazeposeDepthai import BlazeposeDepthai

class SkeletalTracker(Node):
    def __init__(self, visualisation):
        super().__init__('skeletal_tracker')
        self.publisher_pose = self.create_publisher(PointStamped, 'BLAZEPOSE/pose', 10)

        # Initialize BlazeposeDepthai
        self.tracker = BlazeposeDepthai(input_src="rgb", lm_model="lite", xyz=False)
        self.visualisation = visualisation

        # Initialize OpenCV window
        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Run Blazepose on the next frame
        frame, body = self.tracker.next_frame()

        if frame is not None:
            # Get the height and width of the frame
            height, width = frame.shape[:2]
            self.get_logger().info(f"Frame dimensions: Width: {width}, Height: {height}")

        if body:
            # Extract the coordinates of landmark 25 (knee)
            landmark_25 = body.landmarks[25]
            landmark_26 = body.landmarks[26]
            ##take a point in middle of knnes
            middle = (landmark_25 + landmark_26)/2.0

            # Publish the pose as PointStamped
            pose_msg = PointStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_frame"  # Change this to the appropriate frame id
            pose_msg.point.x = float(middle[0])
            pose_msg.point.y = float(middle[1])
            pose_msg.point.z = float(middle[2])
            self.publisher_pose.publish(pose_msg)

            # Draw landmarks on the frame
            if self.visualisation:
                for lm in body.landmarks:
                    x, y = int(lm[0] * width), int(lm[1] * height)
                    cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

                # Draw the specific landmark (25) in red
                x_25, y_25 = int(landmark_25[0] * width), int(landmark_25[1] * height)
                cv2.circle(frame, (x_25, y_25), 7, (0, 0, 255), -1)

                # Show the frame
                cv2.imshow('BlazePose', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.destroy_node()
                    cv2.destroyAllWindows()
        else:
            self.get_logger().warn("No body detected")

def main(args=None):
    rclpy.init(args=args)

    # Use rclpy's argument parsing
    ros_args = rclpy.utilities.remove_ros_args(args)
    #  Parse the visualization argument
    if '--v' in ros_args:
        visualisation = True
    else:
        visualisation = False
        
    node = SkeletalTracker(visualisation)
    rclpy.spin(node)
    rclpy.shutdown()


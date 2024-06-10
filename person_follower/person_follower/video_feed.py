import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import depthai as dai
from cv_bridge import CvBridge

class CameraStream(Node):
    def __init__(self):
        super().__init__('camera_stream')
        self.publisher_ = self.create_publisher(Image, 'BLAZEPOSE/image', 10)
        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()

        camRgb = self.pipeline.createColorCamera()
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        xoutRgb = self.pipeline.createXLinkOut()
        xoutRgb.setStreamName("video")
        camRgb.video.link(xoutRgb.input)

        self.device = dai.Device(self.pipeline)
        self.qVideo = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        inFrame = self.qVideo.get()
        frame = inFrame.getCvFrame()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraStream()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


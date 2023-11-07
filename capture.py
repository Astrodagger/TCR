#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class MyFramePublisher(Node):
    def __init__(self):
        super().__init__("my_frame_publisher")
        self.get_logger().info("My Frame Publisher started!")
        self.image_publisher = self.create_publisher(Image, "/my_image_pub", 10)
        self.image_bridge = CvBridge()

    def publishVideo(self):
        camera_capture = cv2.VideoCapture(0)
        camera_capture.set(3, 800)
        camera_capture.set(4, 600)
        while True:
            ret, captured_frame = camera_capture.read()
            if ret:
                ros_image_msg = self.image_bridge.cv2_to_imgmsg(captured_frame, "bgr8")
                self.image_publisher.publish(ros_image_msg)
            else:
                break

def main(args=None):
    try:
        rclpy.init(args=args)
        my_node = MyFramePublisher()
        my_node.publishVideo()
    except rclpy.exceptions.ROSInterruptException:
        rclpy.shutdown()

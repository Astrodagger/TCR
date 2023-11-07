#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

class Tracker(Node):
    def __init__(self):
        super().__init__("tracker")
        self.image_sub = self.create_subscription(Image, "/ball_pub", self.img_callback, 10)
        self.ball_pub = self.create_publisher(PoseStamped, '/ball_location', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Tracker started!")

    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = (70, 150, 49)
        upper = (87, 255, 255)
        mask = cv2.inRange(hsv, lower, upper)
        masked_img = cv2.bitwise_and(img, img, mask=mask)
        x, y = self.find_location(masked_img)
        self.draw_contours(img)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.ball_pub.publish(pose)

    def find_location(self, masked_img):
        gray_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        x, y = 400, 300

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)
            if moments["m00"] != 0:
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])

        return x, y

    def get_center(self, contour):
        moments = cv2.moments(contour)
        cx, cy = -1, -1
        if (moments['m00'] != 0):
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
        return cx, cy

    def draw_contours(self, rgb_img):
        black_img = np.zeros(rgb_img.shape, 'uint8')
        for c in self.contours:
            area = cv2.contourArea(c)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if (area > 5000):
                cv2.drawContours(rgb_img, [c], -1, (255, 0, 255), 2)
                cx, cy = self.get_center(c)
                cv2.circle(rgb_img, (cx, cy), int(radius), (0, 255, 255), 3)
                cv2.circle(black_img, (cx, cy), int(radius), (0, 255, 255), 3)
                cv2.circle(black_img, (cx, cy), 5, (150, 0, 255), -1)
        cv2.imshow("Ball Tracking", rgb_img)
        cv2.imshow("Black Background Tracking", black_img)
        cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


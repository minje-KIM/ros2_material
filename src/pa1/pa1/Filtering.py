#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class FilteringNode(Node): 
    def __init__(self):
        super().__init__("filter_node") 
        self.subscriber_ = self.create_subscription(
                        Image, "/cam0/image_raw", self.image_sub_callback, 10
                                )
        self.publisher_ = self.create_publisher(Image, 'output_image', 10)
        self.bridge = CvBridge()

    def apply_filter(self,image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        sobel_x = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3)
        gradient_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        gradient_magnitude_scaled = (gradient_magnitude / gradient_magnitude.max() * 255).astype(np.uint8)
        return cv2.cvtColor(gradient_magnitude_scaled, cv2.COLOR_GRAY2BGR)

    def image_sub_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding='bgr8')
        filtered_img = self.apply_filter(cv_image)
        filtered_msg = self.bridge.cv2_to_imgmsg(filtered_img, 'bgr8')
        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FilteringNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
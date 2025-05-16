#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime

class NightImageEnhancer(Node):
    def __init__(self):
        super().__init__('night_image_enhancer')

        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',  # Adjust this topic if needed
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Image, '/zed/enhanced_image', 10)
        self.bridge = CvBridge()

        # Directory to save images
        self.save_dir = os.path.expanduser('~/zed_images')
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info("Night Image Enhancer Node Initialized.")

    def image_callback(self, msg):
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Enhance image
            enhanced_image = self.enhance_image(cv_image)

            # Publish the enhanced image
            out_msg = self.bridge.cv2_to_imgmsg(enhanced_image, encoding='bgr8')
            self.publisher.publish(out_msg)

            # Save image to disk
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = f"enhanced_{timestamp}.png"
            filepath = os.path.join(self.save_dir, filename)
            cv2.imwrite(filepath, enhanced_image)

            self.get_logger().info(f"Saved enhanced image to: {filepath}")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def enhance_image(self, img):
        # Convert to YCrCb color space
        ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
        y, cr, cb = cv2.split(ycrcb)

        # Apply CLAHE to the Y channel
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        y_clahe = clahe.apply(y)

        # Merge and convert back to BGR
        enhanced = cv2.merge((y_clahe, cr, cb))
        enhanced_bgr = cv2.cvtColor(enhanced, cv2.COLOR_YCrCb2BGR)

        return enhanced_bgr


def main(args=None):
    rclpy.init(args=args)
    node = NightImageEnhancer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

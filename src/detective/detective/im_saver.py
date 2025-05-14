#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.image_saved = False
        self.get_logger().info("Image saver node started. Waiting for an image...")

    def listener_callback(self, msg):
        if not self.image_saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv2.imwrite('saved_image.png', cv_image)
                self.get_logger().info('Image saved as saved_image.png')
                self.image_saved = True
                # Optional: shut down after saving
                rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f"Failed to convert/save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

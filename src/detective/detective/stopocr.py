import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2, os
import numpy as np
import easyocr

class TextDetectorNode(Node):
    def __init__(self):
        super().__init__('text_detector_node')

        # Declare text to search for
        self.declare_parameter("target_text", "STOP")
        self.target_text = self.get_parameter("target_text").get_parameter_value().string_value

        self.bridge = CvBridge()
        
        self.reader = easyocr.Reader(['en'], gpu=True)

        # Image topics
        self.create_subscription(Image, '/zed_node/rgb/image_rect_color', self.image_callback, 10)
        self.create_subscription(Image, '/zed_node/depth/depth_registered', self.depth_callback, 10)

        self.depth_image = None
        self.publisher = self.create_publisher(PointStamped, '/detected_text_info', 10)

        self.get_logger().info("TextDetectorNode initialized")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    def image_callback(self, msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        results = self.reader.readtext(color_image)

        for (bbox, text, confidence) in results:
            if text.strip().lower() == self.target_text.lower():
                # bbox = [top_left, top_right, bottom_right, bottom_left]
                pts = np.array(bbox).astype(int)
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                depth = None
                if self.depth_image is not None and cy < self.depth_image.shape[0] and cx < self.depth_image.shape[1]:
                    depth = float(self.depth_image[cy, cx])
                else:
                    self.get_logger().warn("No depth info available")

                point_msg = PointStamped()
                point_msg.header = msg.header
                point_msg.point.x = float(cx)
                point_msg.point.y = float(cy)
                point_msg.point.z = depth if depth is not None else -1.0

                self.publisher.publish(point_msg)
                self.get_logger().info(f"Detected '{text}' at ({cx}, {cy}) with depth {depth}")
                break  # Stop after first match

def main(args=None):
    rclpy.init(args=args)
    node = TextDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

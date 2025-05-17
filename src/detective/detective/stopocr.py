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

        self.create_subscription(Image, '/zed_node/stereocamera/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/zed_node/stereocamera/depth/image_raw', self.depth_callback, 10)

        self.depth_image = None
        self.image_pub = self.create_publisher(Image, 'annotated_image', 10)
        self.publisher = self.create_publisher(PointStamped, '/detected_text_info', 10)

        self.get_logger().info("TextDetectorNode initialized")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().info(f"Depth Image recieved:")
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    def image_callback(self, msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().info(f"Color Image received:")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
        results = self.reader.readtext(color_image)
        self.get_logger().info(f"Detected text: {results}")
        for (bbox, text, confidence) in results:
            if text.strip().lower() == self.target_text.lower():
                pts = np.array(bbox).astype(int)

                # Compute bounding rectangle
                x, y, w, h = cv2.boundingRect(pts)

                # Slightly increase the size of the rectangle (padding)
                pad = 10
                x = max(0, x - pad)
                y = max(0, y - pad)
                w = w + 2 * pad
                h = h + 2 * pad

                # Draw a thick rectangle (thickness=3)
                cv2.rectangle(color_image, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=3)

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                depth = None
                if self.depth_image is not None and cy < self.depth_image.shape[0] and cx < self.depth_image.shape[1]:
                    depth = float(self.depth_image[cy, cx])
                else:
                    self.get_logger().warn("No depth info available")

                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = "map"
                point_msg.point.x = float(cx)
                point_msg.point.y = float(cy)
                point_msg.point.z = depth if depth is not None else -1.0
                
                self.publisher.publish(point_msg)
                break  # Stop after fi
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = TextDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

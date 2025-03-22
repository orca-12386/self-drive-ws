import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import torch
import numpy as np
import pytesseract
import os
from ultralytics import YOLO

class Detector(Node):
    def __init__(self, img_pub_topic, dist_pub_topic, inp_img_topic, depth_img_topic):
        super().__init__('Detector')

        # ROS2 Publishers & Subscribers
        self.image_sub = self.create_subscription(Image, inp_img_topic, self.img_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_img_topic, self.depth_callback, 10)
        self.image_pub = self.create_publisher(Image, img_pub_topic, 10)
        self.distance_pub = self.create_publisher(Float32, dist_pub_topic, 10)

        # Load YOLO model
        package_name = "detective"
        package_share_dir = get_package_share_directory(package_name)
        self.stop_sign_model_path = os.path.join(package_share_dir, "Trained_models", "StopSigns.pt")
        self.stop_sign_model = YOLO(self.stop_sign_model_path)

        self.bridge = CvBridge()
        self.box_coords = None  # Stores detected bounding box center coordinates
        self.get_logger().info("Stop Sign Detector Initialized Successfully.")

    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.detect_stop_sign(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error in Image Callback: {e}")

    def depth_callback(self, msg):
        if self.box_coords is not None:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            x, y = map(int, self.box_coords)
            
            if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                depth_value = depth_image[y, x]
                self.get_logger().info(f"Distance from Stop Sign: {depth_value:.2f} meters")
                self.distance_pub.publish(Float32(data=float(depth_value)))
                self.box_coords = None  # Reset after publishing
            else:
                self.get_logger().warn(f"Bounding box coordinates out of depth image bounds: ({x}, {y})")

    def detect_stop_sign(self, image):
        try:
            results = self.stop_sign_model(image, verbose=False)[0]
            boxes = results.boxes.xywh
            confs = results.boxes.conf

            if len(boxes) > 0:
                for box, conf in zip(boxes, confs):
                    if conf > 0.5:
                        x, y, w, h = map(int, box)
                        roi = image[max(y - h // 2, 0): min(y + h // 2, image.shape[0]),
                                    max(x - w // 2, 0): min(x + w // 2, image.shape[1])]
                        print("Detected")
                        if self.run_ocr(roi):
                            self.box_coords = (x, y)  # Store center coordinates for depth lookup
                            cv2.rectangle(image, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 2)
                            cv2.putText(image, "STOP", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="rgb8"))
        except Exception as e:
            self.get_logger().error(f"Detection Error: {e}")

    def run_ocr(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        text = pytesseract.image_to_string(thresh, config="--psm 6").strip().lower()
        return "stop" in text


def main(args=None):
    rclpy.init(args=args)
    node = Detector(img_pub_topic="/detection", dist_pub_topic="/stop_sign_dist",
                     inp_img_topic="/zed_node/stereocamera/image_raw", depth_img_topic="/zed_node/stereocamera/depth/image_raw")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import torch
import numpy as np
import os
from ultralytics import YOLO

class Detector(Node):
    def __init__(self, img_pub_topic, depth_img_topic, dist_pub_topic):
        super().__init__('Detector')

        # ROS2 Publisher & Subscriber
        self.image_subscriber = self.create_subscription(Image, img_pub_topic, self.img_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, depth_img_topic, self.depth_callback, 10)
        self.image_publisher = self.create_publisher(Image, "/detected_pedestrian", 10)
        self.dist_publisher = self.create_publisher(Float32, dist_pub_topic, 10)

        # Load YOLO Model
        package_name = "detective"
        package_share_dir = get_package_share_directory(package_name)
        model_path = os.path.join(package_share_dir, "Trained_models", "Pedestrian.pt")
        self.model = YOLO(model_path)

        # Image Processing
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth_image = None
        self.pedestrian_coords = None

        self.get_logger().info("Detection Model Loaded Successfully.")

    def img_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.detect_pedestrian()
        except Exception as e:
            self.get_logger().error(f"Error in Image Callback: {e}")

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.calculate_distance()
        except Exception as e:
            self.get_logger().error(f"Error in Depth Callback: {e}")

    def detect_pedestrian(self):
        if self.latest_image is None:
            return
        try:
            results = self.model.predict(self.latest_image, verbose=False)[0]
            boxes = results.boxes.xywh
            confs = results.boxes.conf
            
            for box, conf in zip(boxes, confs):
                if conf > 0.5:
                    x, y, w, h = map(int, box)
                    x1, y1, x2, y2 = x - w // 2, y - h // 2, x + w // 2, y + h // 2
                    self.pedestrian_coords = (x, y)
                    
                    # Draw Bounding Box
                    cv2.rectangle(self.latest_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(self.latest_image, f"Conf: {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            self.publish_detected_image()
        except Exception as e:
            self.get_logger().error(f"Detection Error: {e}")

    def calculate_distance(self):
        if self.latest_depth_image is None or self.pedestrian_coords is None:
            return
        try:
            x, y = self.pedestrian_coords
            if 0 <= x < self.latest_depth_image.shape[1] and 0 <= y < self.latest_depth_image.shape[0]:
                depth_value = self.latest_depth_image[y, x]
                distance_msg = Float32()
                distance_msg.data = float(depth_value)
                self.dist_publisher.publish(distance_msg)
                self.get_logger().info(f"Published Pedestrian Distance: {depth_value:.2f} meters")
            else:
                self.get_logger().warn("Pedestrian coordinates out of bounds.")
        except Exception as e:
            self.get_logger().error(f"Error in Distance Calculation: {e}")

    def publish_detected_image(self):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(self.latest_image, encoding="rgb8")
            self.image_publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Error in Publishing Image: {e}")


def main(args=None):
    rclpy.init(args=args)
    detector = Detector("/zed_node/stereocamera/image_raw", "/zed_node/stereocamera/depth/image_raw", "/ped_dist")
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", e)

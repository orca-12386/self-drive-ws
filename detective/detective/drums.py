#!/media/summer/ubuntu 2tb/IGVC-2025/IGVC_ws/src/YOLO/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import torch
import numpy as np
import os
from ultralytics import YOLO

class Detector(Node):
    def __init__(self, img_pub_topic, inp_img_topic, depth_img_topic, info_topic, dist_pub_topic):
        super().__init__('Detector')

        # ROS2 Publisher & Subscriber
        self.subscriber = self.create_subscription(Image, inp_img_topic, self.img_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, depth_img_topic, self.depth_callback, 10)
        self.publisher = self.create_publisher(Image, img_pub_topic, 10)
        self.dist_publisher = self.create_publisher(Float32, dist_pub_topic, 10)

        self.declare_parameter('save_directory', './saved_images')
        self.save_directory = self.get_parameter('save_directory').get_parameter_value().string_value

        # Paths for YOLO models
        package_name = "detective"
        package_share_dir = get_package_share_directory(package_name)
        self.traffic_drums_model_path = os.path.join(package_share_dir, "Trained_models", "Drums.pt")

        self.box_coords = None
        self.traffic_drum_model = YOLO(self.traffic_drums_model_path)
        self.img = None
        self.output = None
        self.bridge = CvBridge()

        self.get_logger().info("Detection Models Loaded Successfully.")

    def depth_callback(self, msg):
        if self.box_coords is not None:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            x, y = self.box_coords
            x, y = int(x.item()), int(y.item())
            if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                depth_value = depth_image[y, x]
                self.get_logger().info(f"Distance from the drum: {depth_value}")
                self.dist_publisher.publish(Float32(data = float(depth_value)))
                self.box_coords = None
            else:
                self.get_logger().warn("Detected object is out of bounds in the depth image.")
    
    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')  # Ensure correct format
            cv_image = cv_image.astype(np.uint8)
            self.img = cv_image
            if self.img is not None:
                self.detect()
        except Exception as e:
            self.get_logger().error(f"Error in Image Callback: {e}")

    def detect(self):
        DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

        try:
            drums = self.traffic_drum_model.predict(self.img, verbose=False)[0]
            boxes = drums.boxes.xywh
            confs = drums.boxes.conf
            if len(boxes) > 0:
                for box, conf in zip(boxes, confs):
                    if conf > 0.5:
                        x, y, w, h = map(int, box)
                        x1, y1 = x - w // 2, y - h // 2
                        x2, y2 = x + w // 2, y + h // 2
                        self.box_coords = box[:2]
                        cv2.rectangle(self.img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"Conf: {conf:.2f}"
                        cv2.putText(self.img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            self.output = self.img
        except Exception as e:
            self.get_logger().error(f"Detection Error: {e}")

        if self.output is not None:
            self.publish_images()

    def publish_images(self):
        img_msg = self.bridge.cv2_to_imgmsg(self.output, encoding="rgb8")
        self.publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    img_pub_topic = "/drums"
    inp_img_topic = "/zed_node/stereocamera/image_raw"
    depth_img_topic = "/zed_node/stereocamera/depth/image_raw"
    dist_pub_topic = "/drum_dist"
    
    detect = Detector(img_pub_topic, inp_img_topic, depth_img_topic, '/zed_node/stereocamera/depth/camera_info', dist_pub_topic)
    rclpy.spin(detect)
    detect.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", e)

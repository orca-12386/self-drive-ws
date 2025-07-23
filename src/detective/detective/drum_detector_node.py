import cv2
from detective.detector_base import BaseDetector
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
import scipy.spatial.transform as Rotation
import numpy as np
from math import sin, cos, pi
import tf2_ros
import tf2_geometry_msgs
import rclpy
import time
from rclpy.time import Time

def euler_to_quat(euler):
    return Rotation.from_euler('xyz', euler).as_quat()

def quat_to_euler(quat):
    return Rotation.from_quat(quat).as_euler('xyz')

class TrafficDrumsDetector(BaseDetector):
    def __init__(self):
        super().__init__('barrels_detector', '/detector/traffic_drum', 'trained_models/Drums.pt')
        
        # Override model path for drums
        self._load_model()  # Reload model with new path
        self.odom_sub = self.create_subscription(Odometry,"/odom", self.odom_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/zed_node/stereocamera/depth/image_raw", self.depth_callback, 10)
        self.location_pub = self.create_publisher(Point, "detector/traffic_drum/coordinates", 10)

        self.declare_parameter("sim", False)
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value

        self.detected_point = None
        self.depth_stamp = None
        self.depth_frame_id = None
        self.latest_depth_image = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform_stamped = None
        self.bot_position = None
        self.bot_orientation = None

    def depth_callback(self, msg):
        self.depth_stamp = msg.header.stamp
        self.depth_frame_id = msg.header.frame_id
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation
        try:
            if not self.sim:
                self.transform_stamped = self.tf_buffer.lookup_transform("odom", self.depth_frame_id, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return
        self.detect_objects()
        self.calculate_object_position()
    
    def detect_objects(self):
        if self.latest_image is None:
            return
        
        try:
            confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
            drums = self.model.predict(self.latest_image, verbose=False)[0]
            boxes = drums.boxes.xywh
            confs = drums.boxes.conf
            
            if len(boxes) > 0:
                for box, conf in zip(boxes, confs):
                    if conf > confidence_threshold:
                        x, y, w, h = map(int, box)
                        x1, y1 = x - w // 2, y - h // 2
                        x2, y2 = x + w // 2, y + h // 2
                        self.object_coords = box[:2]
                        
                        cv2.rectangle(self.latest_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"Conf: {conf:.2f}"
                        cv2.putText(self.latest_image, label, (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish detected image
            img_msg = self.bridge.cv2_to_imgmsg(self.latest_image, encoding="rgb8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Detection Error: {e}")

    
    def calculate_object_position(self):
        if (self.latest_depth_image is None or 
            self.object_coords is None or 
            self.camera_info is None):
            return
        
        try:
            x, y = self.object_coords
            x, y = int(x.item()), int(y.item())
            
            if (0 <= x < self.latest_depth_image.shape[1] and 
                0 <= y < self.latest_depth_image.shape[0]):
                depth_value = self.latest_depth_image[y, x]
                point = self._pixel_to_point(x, y, depth_value)
                
                if point:
                    if self.sim:
                        pitch = float(23.5) * pi / float(180)
                        point.y = float(1.5) - point.z*sin(pitch) - point.y*cos(pitch)
                    
                    point.y, point.z = point.z, point.y  
                    point_in = PointStamped()
                    point_out = PointStamped()
                    point_in.header.stamp = self.depth_stamp
                    point_in.header.frame_id = "base_link"
                    point_in.point.x = point.x
                    point_in.point.y = point.y
                    point_in.point.z = point.z

                    if not self.sim:
                        try:
                            point_out = tf2_geometry_msgs.do_transform_point(point_in, self.transform_stamped)
                            point.x = point_out.point.x
                            point.y = point_out.point.y
                            point.z = point_out.point.z
                        except tf2_ros.TransformException as e:
                            self.get_logger().error(f"Transform error: {e}")
                            return

                    if self.bot_orientation is not None:
                        euler = quat_to_euler([self.bot_orientation.x, self.bot_orientation.y, 
                                             self.bot_orientation.z, self.bot_orientation.w])
                        bot_yaw = euler[2]  
                        self.detected_point = Point()
                        self.detected_point.x = self.bot_position.x + point.x * sin(bot_yaw) + point.y * cos(bot_yaw)
                        self.detected_point.y = self.bot_position.y + point.y * sin(bot_yaw) - point.x * cos(bot_yaw) 
                        self.detected_point.z = - point.z

                        self.location_pub.publish(self.detected_point)
                        self.get_logger().info(f"Detected Drum: {self.detected_point}")
            
            self.object_coords = None
        except Exception as e:
            self.get_logger().error(f"Position calculation error: {e}")



def main(args=None):
    rclpy.init(args=args)
    
    detector = TrafficDrumsDetector()
    
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
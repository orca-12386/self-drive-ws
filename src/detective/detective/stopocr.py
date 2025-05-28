import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2, os
import numpy as np
import easyocr
import tf2_ros
import tf2_geometry_msgs
from math import sin, cos
from scipy.spatial.transform import Rotation

def euler_to_quat(euler):
    return Rotation.from_euler('xyz', euler).as_quat()

def quat_to_euler(quat):
    return Rotation.from_quat(quat).as_euler('xyz') 

class TextDetectorNode(Node):
    def __init__(self):
        super().__init__('text_detector_node')

        self.declare_parameter("target_text", "STOP")
        self.target_text = self.get_parameter("target_text").get_parameter_value().string_value

        self.bridge = CvBridge()
        
        self.reader = easyocr.Reader(['en'], gpu=True)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_subscription(Image, '/zed_node/stereocamera/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/zed_node/stereocamera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/zed_node/stereocamera/camera_info', self.camera_info_callback, 10)

        self.sim = True
        self.depth_image = None
        self.depth_stamp = None
        self.depth_frame_id = None
        self.camera_info = None
        self.transform_stamped = None
        self.detected_point = None
        self.image_pub = self.create_publisher(Image, '/annotated_image', 10)
        self.publisher = self.create_publisher(PointStamped, '/detected_text_info', 10)
        self.position_publisher = self.create_publisher(PointStamped, '/detector/stop_sign/position', 10)

        self.get_logger().info("TextDetectorNode initialized")

    
    def pixel_to_point(self, u, v, depth):
        if self.camera_info is None or depth is None:
            return None
        
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        x = (u-cx) * depth / fx
        y = (v-cy) * depth / fy
        z = depth
            
        return Point(x=float(x), y=float(y), z=float(z))

    def cloud_to_global(self, point):
        if self.bot_position is None or self.bot_orientation is None:
            return None

        euler = quat_to_euler([self.bot_orientation.x, self.bot_orientation.y, self.bot_orientation.z, self.bot_orientation.w])
        bot_yaw = euler[2]

        x = self.bot_position.x + point.x * sin(bot_yaw) + point.y * cos(bot_yaw)
        y = self.bot_position.y + point.y * sin(bot_yaw) - point.x * cos(bot_yaw)
        z = point.z
 
        return Point(x=float(x), y=float(y), z=float(z))

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        try:
            self.depth_stamp = msg.header.stamp
            self.depth_frame_id = msg.header.frame_id
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

        if  not self.sim:
            try:
                self.transform_stamped = tf2_ros.Buffer().lookup_transform("map", self.depth_frame_id, self.depth_stamp)
            except tf2_ros.TransformException as e:
                self.get_logger().error(f"Transform lookup failed: {e}")
                return

        results = self.reader.readtext(color_image)
        self.get_logger().info(f"Detected text: {results}")
        for (bbox, text, confidence) in results:
            if text.strip().lower() == self.target_text.lower():
                pts = np.array(bbox).astype(int)
                x, y, w, h = cv2.boundingRect(pts)

                pad = 10
                x = max(0, x - pad)
                y = max(0, y - pad)
                w = w + 2 * pad
                h = h + 2 * pad

                cv2.rectangle(color_image, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=3)

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                depth = None
                if self.depth_image is not None and cy < self.depth_image.shape[0] and cx < self.depth_image.shape[1]:
                    depth = float(self.depth_image[cy, cx])
                else:
                    self.get_logger().warn("No depth info available")
                
                point = self.pixel_to_point(cx, cy, depth)
                if point:
                    if self.sim:
                        pitch = float(23.5) * np.pi / float(180)
                        point.y = float(1.5) - point.z * np.sin(pitch) - point.y * np.cos(pitch)
                    point.y, point.z = point.z, point.y
                    
                    point_in = PointStamped()
                    point_in.header.stamp = self.depth_stamp
                    point_in.header.frame_id = "base_link"
                    point_in.point.x = point.x
                    point_in.point.y = point.y
                    point_in.point.z = point.z

                    point_out = PointStamped()

                    if not self.sim:
                        try:
                            point_out = tf2_geometry_msgs.do_transform_point(point_in, self.transform_stamped)
                            point.x = point_out.point.x
                            point.y = point_out.point.y
                            point.z = point_out.point.z
                        except tf2_ros.TransformException as e:
                            self.get_logger().error(f"Transform error: {e}")
                            return

                    point = self.cloud_to_global(point)

                    self.detected_point = PointStamped()
                    self.detected_point.header.stamp = self.get_clock().now().to_msg()
                    self.detected_point.header.frame_id = "map"
                    self.detected_point.point.x = point.x
                    self.detected_point.point.y = point.y
                    self.detected_point.point.z = point.z

                    self.position_publisher.publish(self.detected_point)
                    self.get_logger().info(f"Detected Stop Sign: {self.detected_point.point.x}, {self.detected_point.point.y}, {self.detected_point.point.z}")
                break  
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = TextDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

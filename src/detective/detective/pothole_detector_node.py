import cv2
from detective.detector_base import BaseDetector
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation
from math import sin, cos, pi
import tf2_ros
import tf2_geometry_msgs

def euler_to_quat(euler):
    return Rotation.from_euler('xyz', euler).as_quat()

def quat_to_euler(quat):
    return Rotation.from_quat(quat).as_euler('xyz') 

class PotholeDetector(BaseDetector):
    def __init__(self):
        super().__init__('pothole_detector', '/detector/pothole', 'trained_models/StopSigns.pt')
        self.mask_sub = self.create_subscription(Image, "/mask/white", self.mask_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,"/odom", self.odom_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/zed_node/stereocamera/depth/image_raw", self.depth_callback, 10)
        self.pointstamped_pub = self.create_publisher(PointStamped, '/detector/pothole/position', 10)
        self.latest_image = None
        self.object_coords = None
        self.detected_point = None
        self.min_area = 100
        self.max_area = 1500
        self.min_circularity = 0.4
        self.bot_position = None
        self.bot_orientation = None
        self.sim = True
        self.depth_stamp = None
        self.depth_frame_id = None
        self.latest_depth_image = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform_stamped = None

    def depth_callback(self, msg):
        self.depth_stamp = msg.header.stamp
        self.depth_frame_id = msg.header.frame_id
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation

    def mask_callback(self, msg):
        try:
            mask_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert mask image: {e}")
            return
            
        if mask_image is not None:
            self.latest_image = mask_image
            if not self.sim:
                try:
                    self.transform_stamped = self.tf_buffer.lookup_transform('map', self.depth_frame_id, self.depth_stamp)
                except tf2_ros.TransformException as e:
                    self.get_logger().error(f"Transform lookup failed: {e}")
                    return
                    
            self.detect_objects()
            self.calculate_object_position()
        else:
            self.get_logger().warning("Did not receive mask image.")

    def detect_objects(self):
        if self.latest_image is None:
            self.get_logger().warning("No image received yet.")
            return

        try:
            image = self.latest_image.copy()
            annotated = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

            contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_ellipse = None
            best_score = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area or area > self.max_area:
                    continue

                if len(contour) < 5:
                    continue

                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue

                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity < self.min_circularity:
                    continue
                
                ellipse = cv2.fitEllipse(contour)
                center = tuple(map(int, ellipse[0]))
                
                axes = ellipse[1]
                major_axis = max(axes)
                minor_axis = min(axes)
                aspect_ratio = minor_axis / major_axis if major_axis > 0 else 0
                
                x,y = center
                depth = self.latest_depth_image[int(y), int(x)]
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                major_axis_meters = (major_axis / fx) * depth
                self.get_logger().info(f"Ellipse radius: {major_axis_meters} meters")

                if major_axis_meters < 0.5 or major_axis_meters > 0.7:
                    continue

                score = area * circularity * aspect_ratio
                
                if score > best_score:
                    best_score = score
                    best_ellipse = {
                        'ellipse': ellipse,
                        'center': center,
                        'contour': contour,
                    }
            
            if best_ellipse:
                ellipse = best_ellipse['ellipse']
                center = best_ellipse['center']
                self.object_coords = center
            else:
                self.get_logger().info("No ellipses detected within criteria.")
                self.object_coords = None

        except Exception as e:
            pass

    def calculate_object_position(self):
        if (self.latest_depth_image is None or
            self.object_coords is None or
            self.camera_info is None):
            self.get_logger().warning(f"Missing data - depth: {self.latest_depth_image is not None}, "
                                    f"coords: {self.object_coords is not None}, "
                                    f"camera_info: {self.camera_info is not None}")  
            return
        try:
            x, y = self.object_coords
            x, y = int(x), int(y)
            
            if (0 <= x < self.latest_depth_image.shape[1] and
                0 <= y < self.latest_depth_image.shape[0]):
                depth_value = self.latest_depth_image[y, x]
                
                point = self._pixel_to_point(x, y, depth_value)
                print(point)
                
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

                        self.detected_point = PointStamped()
                        self.detected_point.header.stamp = self.get_clock().now().to_msg()
                        self.detected_point.header.frame_id = "map"
                        self.detected_point.point.x = self.bot_position.x + point.x * sin(bot_yaw) + point.y * cos(bot_yaw)
                        self.detected_point.point.y = self.bot_position.y + point.y * sin(bot_yaw) - point.x * cos(bot_yaw) 
                        self.detected_point.point.z = - point.z

                        if self.detected_point.point.z > 1:
                            self.get_logger().info("Detected point height is too high(prolly the sky).")
                            return 
                        
                        self.point_pub.publish(point)
                        self.pointstamped_pub.publish(self.detected_point)
                        self.get_logger().info(f"Detected Pothole: {self.detected_point.point}")

            self.object_coords = None
                    
        except Exception as e:
            self.get_logger().error(f"Position calculation error: {e}")

def main(args=None):
    rclpy.init(args=args)
    detector = PotholeDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import torch
import numpy as np
import os
from ultralytics import YOLO
import time


class BaseDetector(Node):
    def __init__(self, node_name, detector_topic, model_path):
        super().__init__(node_name)
        
        # Declare configurable parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_image_topic', '/zed_node/stereocamera/image_raw'),
                ('depth_image_topic', '/zed_node/stereocamera/depth/image_raw'),
                ('camera_info_topic', '/zed_node/stereocamera/camera_info'),
                ('output_image_topic', detector_topic+'/image'),
                ('output_point_topic', detector_topic+'/coordinates'),
                ('confidence_threshold', 0.5),
                ('model_package', 'detective'),
                ('model_path', model_path)
            ]
        )
        
        # Initialize ROS components
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth_image = None
        self.camera_info = None
        self.object_coords = None
        
        # Setup communication
        self._setup_communication()
        
        # Load model
        self._load_model()
        
        self.get_logger().info(f"{node_name} initialized successfully.")
    
    def _setup_communication(self):
        # Image and depth subscribers
        self.image_sub = self.create_subscription(
            Image, 
            self.get_parameter('input_image_topic').get_parameter_value().string_value, 
            self.img_callback, 
            10
        )
        self.depth_sub = self.create_subscription(
            Image, 
            self.get_parameter('depth_image_topic').get_parameter_value().string_value, 
            self.depth_callback, 
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            self.get_parameter('camera_info_topic').get_parameter_value().string_value, 
            self.camera_info_callback, 
            10
        )
        
        # Publishers
        self.image_pub = self.create_publisher(
            Image, 
            self.get_parameter('output_image_topic').get_parameter_value().string_value, 
            10
        )
        self.point_pub = self.create_publisher(
            Point, 
            self.get_parameter('output_point_topic').get_parameter_value().string_value, 
            10
        )
    
    def _load_model(self):
        package_name = self.get_parameter('model_package').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        full_model_path = os.path.join(
            get_package_share_directory(package_name), 
            model_path
        )
        self.model = YOLO(full_model_path)


    def camera_info_callback(self, msg):
        self.camera_info = msg
    
    def img_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.detect_objects()
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")
    
    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.calculate_object_position()
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")
    
    def detect_objects(self):
        # To be implemented by child classes
        raise NotImplementedError("Subclasses must implement object detection")
    
    def calculate_object_position(self):
        # To be implemented by child classes
        raise NotImplementedError("Subclasses must implement position calculation")
    
    def _pixel_to_point(self, u, v, depth):
        """Convert pixel coordinates to 3D point using camera intrinsics"""
        if self.camera_info is None or depth is None:
            return None
        
        fx = self.camera_info.k[0]  # Focal length x
        fy = self.camera_info.k[4]  # Focal length y
        cx = self.camera_info.k[2]  # Optical center x
        cy = self.camera_info.k[5]  # Optical center y
        
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return Point(x=float(x), y=float(y), z=float(z))
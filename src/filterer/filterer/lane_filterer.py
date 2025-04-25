#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class LaneFilterer(Node):
    def __init__(self):
        super().__init__("lane_filter_node")
        
        # Declare and get parameters
        self.declare_parameter("higher_h", 180)
        self.declare_parameter("higher_s", 255)
        self.declare_parameter("higher_v", 255)
        self.declare_parameter("lower_h", 0)
        self.declare_parameter("lower_s", 0)
        self.declare_parameter("lower_v", 0)
        self.declare_parameter("target_v", 140)
        self.declare_parameter('sim',False)
        self.declare_parameter("mask_topic_pub","/mask/white")

        self.hh = self.get_parameter("higher_h").value
        self.hs = self.get_parameter("higher_s").value
        self.hv = self.get_parameter("higher_v").value
        self.lh = self.get_parameter("lower_h").value
        self.ls = self.get_parameter("lower_s").value
        self.lv = self.get_parameter("lower_v").value
        self.target_v = self.get_parameter("target_v").value
        sim = self.get_parameter("sim").value
        mask_topic_pub = self.get_parameter("mask_topic_pub").value
        # Create publisher and subscriber
        self.image_pub = self.create_publisher(
            Image, 
            mask_topic_pub, 
            1
        )
        img_sub_sim = '/zed_node/stereocamera/image_raw'
        img_sub_real = "/zed/zed_node/rgb/image_rect_color"

        img_sub = None
        if(sim):
            img_sub = img_sub_sim
        else:
            img_sub = img_sub_real

        self.image_sub = self.create_subscription(
            Image,
            img_sub,
            self.image_callback,
            1
        )
        
        self.bridge = CvBridge()
        self.get_logger().info("Lane Filterer Node initialized")
        
    def make_image_brighter(self, image, target_v):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_image)
        avg_v = np.mean(v)
        scale_factor = target_v / avg_v if avg_v > 0 else 1
        v = np.clip(v * scale_factor, 0, 255).astype(np.uint8)
        adjusted_hsv = cv2.merge((h, s, v))
        brighter_image = cv2.cvtColor(adjusted_hsv, cv2.COLOR_HSV2BGR)
        return brighter_image
        
    def image_callback(self, msg):
        # print("Inside Image Callback")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # self.get_logger().info(f"SHape of image: {cv_image.shape}")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return
            
        brighter_bgr_image = self.make_image_brighter(cv_image, self.target_v)
        
        # gray_image = cv2.cvtColor(brighter_bgr_image, cv2.COLOR_BGR2GRAY)
        # # blur_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        # canny_image = cv2.Canny(gray_image, 50, 150)
        #
        # kernel = np.ones((1,1), np.uint8)
        # # canny_image = cv2.dilate(canny_image, kernel=kernel, iterations=1)
        # canny_image = cv2.erode(canny_image, kernel=kernel, iterations=0)
        #
        #
        # contours, * = cv2.findContours(canny*image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # min_area = 50
        # filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        #
        # filtered_contour_mask = np.zeros_like(canny_image)
        # cv2.drawContours(filtered_contour_mask, filtered_contours, -1, 255, thickness=cv2.FILLED) # contours,_ = cv2.findContours(canny_image, )
        
        hsv_image = cv2.cvtColor(brighter_bgr_image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([self.lh, self.ls, self.lv])
        upper_white = np.array([self.hh, self.hs, self.hv])
        mask = cv2.inRange(hsv_image, lower_white, upper_white)
        
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel=kernel, iterations=1)
        mask = cv2.dilate(mask, kernel=kernel, iterations=1)
        
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            # Copy header from input message
            mask_msg.header = msg.header
            self.image_pub.publish(mask_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert mask image: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        lane_filterer = LaneFilterer()
        lane_filterer.get_logger().info("Lane Filterer Node Started")
        rclpy.spin(lane_filterer)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if rclpy.ok():
            lane_filterer.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
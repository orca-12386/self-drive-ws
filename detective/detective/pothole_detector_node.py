import cv2
from detective.detector_base import BaseDetector
import rclpy
import numpy as np

class PotholeDetector(BaseDetector):
    def __init__(self):
        super().__init__('pothole_detector', '/detector/pothole')

    
    def detect_objects(self):
        # Retrieve the latest image from the ROS2 subscriber
        if self.latest_image is None:
            self.get_logger().warning("No image received yet.")
            return

        try:
            image = self.latest_image
            
            # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(image, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Find contours in the edge-detected image
            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # potholes = []

            for contour in contours:
                # Approximate the contour to a polygon
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Check if the approximated contour has an elliptical shape
                if len(approx) >= 5:  # Minimum points required to fit an ellipse
                    moments = cv2.moments(contour)
                if moments["m00"] != 0:
                    center_x = int(moments["m10"] / moments["m00"])
                    center_y = int(moments["m01"] / moments["m00"])
                    # potholes.append((center_x, center_y))
                    self.object_coords = (center_x, center_y)

        except Exception as e:
            self.get_logger().error(f"Detection Error: {e}")

        # print(potholes)
        # return potholes

    def calculate_object_position(self):
        if (self.latest_depth_image is None or self.object_coords is None or self.camera_info is None):
            return
        
        try:
            x, y = self.object_coords
            x, y = int(x.item()), int(y.item())
            
            if (0 <= x < self.latest_depth_image.shape[1] and 
                0 <= y < self.latest_depth_image.shape[0]):
                depth_value = self.latest_depth_image[y, x]
                point = self._pixel_to_point(x, y, depth_value)
                
                if point:
                    self.point_pub.publish(point)
                    self.get_logger().info(f"Drum Distance: {point}")
            
            self.object_coords = None
        except Exception as e:
            self.get_logger().error(f"Position calculation error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Choose the detector you want to use
    # detector = StopSignDetector()
    # detector = PedestrianDetector()
    detector = PotholeDetector()
    
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
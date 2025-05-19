import cv2
import rclpy
import easyocr
from detective.detector_base import BaseDetector


class StopSignDetector(BaseDetector):
    def __init__(self):
        super().__init__('stop_sign_detector', '/detector/stop_sign', 'trained_models/StopSigns.pt')
        self.reader = easyocr.Reader(['en'], gpu=True)
        # Override model path for stop signs
        self._load_model()  # Reload model with new path
        self.cx = -1 # centroid x
        self.xy = -1 # centroid y   
    def detect_objects(self):
        if self.latest_image is None:
            return
        
        try:
            confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
            results = self.reader.readtext(self.latest_image)

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
                    cv2.rectangle(self.latest_image, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=3)

                    self.cx = int(np.mean(pts[:, 0]))
                    self.cy = int(np.mean(pts[:, 1]))

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
            x, y = sellf.cx, self.cy
            
            if (0 <= x < self.latest_depth_image.shape[1] and 
                0 <= y < self.latest_depth_image.shape[0]):
                depth_value = self.latest_depth_image[y, x]
                point = self._pixel_to_point(x, y, depth_value)
                
                if point:
                    self.point_pub.publish(point)
                    self.get_logger().info(f"Stop Sign Distance: {point}")
            
            self.object_coords = None
        except Exception as e:
            self.get_logger().error(f"Position calculation error: {e}")

    

def main(args=None):
    rclpy.init(args=args)
    detector = StopSignDetector()    
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import cv2
import rclpy
import pytesseract
from detective.detector_base import BaseDetector


class StopSignDetector(BaseDetector):
    def __init__(self):
        super().__init__('stop_sign_detector', '/detector/stop_sign', 'trained_models/StopSigns.pt')
        
        # Override model path for stop signs
        self._load_model()  # Reload model with new path
    
    def detect_objects(self):
        if self.latest_image is None:
            return
        
        try:
            confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
            results = self.model.predict(self.latest_image, verbose=False)[0]
            boxes = results.boxes.xywh
            confs = results.boxes.conf

            if len(boxes) > 0:
                for box, conf in zip(boxes, confs):
                    if conf > confidence_threshold:
                        x, y, w, h = map(int, box)
                        roi = self.latest_image[max(y - h // 2, 0): min(y + h // 2, self.latest_image.shape[0]),
                                                max(x - w // 2, 0): min(x + w // 2, self.latest_image.shape[1])]
                        
                        if self._run_ocr(roi):
                            self.object_coords = (x, y)
                            
                            cv2.rectangle(self.latest_image, 
                                          (x - w // 2, y - h // 2), 
                                          (x + w // 2, y + h // 2), 
                                          (0, 255, 0), 2)
                            cv2.putText(self.latest_image, "STOP", 
                                        (x, y - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                                        (0, 255, 0), 2)

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
            x, y = map(int, self.object_coords)
            
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
    
    def _run_ocr(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        text = pytesseract.image_to_string(thresh, config="--psm 6").strip().lower()
        return "stop" in text
    

def main(args=None):
    rclpy.init(args=args)
    detector = StopSignDetector()    
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
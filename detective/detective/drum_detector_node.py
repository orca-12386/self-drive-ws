import cv2
from detective.detector_base import BaseDetector
import rclpy
import time

class TrafficDrumsDetector(BaseDetector):
    def __init__(self):
        super().__init__('traffic_drums_detector', '/detector/traffic_drum', 'trained_models/Drums.pt')
        
        # Override model path for drums
        self._load_model()  # Reload model with new path
    
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
    detector = TrafficDrumsDetector()
    
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
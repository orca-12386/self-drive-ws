import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SingleImageSaver(Node):
    def __init__(self):
        super().__init__('single_image_saver')

        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.image_saved = False

    def image_callback(self, msg):
        if self.image_saved:
            return  # Already saved one image, do nothing

        self.get_logger().info('Received image, saving...')

        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Save the image
        cv2.imwrite('saved_image.png', cv_image)

        self.get_logger().info('Image saved as saved_image.png')

        self.image_saved = True

        # Shutdown the node after saving the image
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SingleImageSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

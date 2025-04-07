import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_odom')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.subscription
        
        self.change = 0.0

    def odom_callback(self, msg):
        """Callback function for the /odom subscriber."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

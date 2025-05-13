import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_odom_transformer')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to modified odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/dlo/odom_node/odom',
            self.odom_callback,
            10
        )

        # Publisher for transformed odometry
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/transformed/odom',
            10
        )

    def odom_callback(self, msg):
        """Callback function for the /dlo/odom_node/odom subscriber."""
        # TransformStamped for TF broadcasting
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot/odom'
        t.child_frame_id = 'robot/base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = -msg.pose.pose.position.y  # Invert Y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation  # Assuming rotation stays same

        self.tf_broadcaster.sendTransform(t)

        # Create and publish transformed odometry
        transformed_odom = Odometry()
        transformed_odom.header = 'robot/odom'
        transformed_odom.child_frame_id = 'robot/base_link'
        transformed_odom.pose = msg.pose
        transformed_odom.twist = msg.twist

        # Apply transformation
        transformed_odom.pose.pose.position.y *= -1  # Invert Y position
        transformed_odom.twist.twist.linear.y *= -1  # Invert Y velocity

        self.odom_publisher.publish(transformed_odom)

def main():
    rclpy.init()
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

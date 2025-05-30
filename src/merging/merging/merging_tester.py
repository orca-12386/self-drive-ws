import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from merging_interface.srv import DetectMerging

class WaitForMergingAndPublish(Node):
    def __init__(self):
        super().__init__('wait_for_merging_and_publish')
        self.cli = self.create_client(DetectMerging, '/detection/merging')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for merging detection service...')
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/merging_pose', 10)
        self.bot_position = None
        self.bot_orientation = None
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.merging_found = False

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation

    def timer_callback(self):
        if self.merging_found:
            return
        req = DetectMerging.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            res = future.result()
            if res.is_merging:
                self.get_logger().info("Merging detected! Publishing pose...")
                self.publish_pose()
                self.merging_found = True
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def publish_pose(self):
        if self.bot_position is None or self.bot_orientation is None:
            self.get_logger().warn("Odometry data not yet received, cannot publish pose.")
            return
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position = self.bot_position
        pose_msg.pose.orientation = self.bot_orientation
        self.pose_pub.publish(pose_msg)
        self.get_logger().info("Published PoseStamped message.")

def main(args=None):
    rclpy.init(args=args)
    node = WaitForMergingAndPublish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
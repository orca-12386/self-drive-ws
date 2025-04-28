import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from interfaces.action import GoalAction as StopAction
import time


class StopServer(Node):
    def __init__(self):
        super().__init__('stop_server')
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.action_server = ActionServer(self, StopAction, 'StopAction', execute_callback=self.execute_callback)
        self.get_logger().info("Stop Server Started")
        self.bot_position = None
        self.bot_orientation = None
        self.goal_pose = None
        self.current_cmd_vel = Twist()
        self.prev_bot_position = None
        self.prev_log_time = None

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation
        
    def check_movement(self):
        log_time = time.time()
        if self.prev_bot_position is None or (log_time - self.prev_log_time) >= 2:
            if not (self.prev_bot_position is None):
                return (self.prev_bot_position.x - self.bot_position.x) + (self.prev_bot_position.y - self.bot_position.y) + (self.prev_bot_position.z - self.bot_position.z) > 0.2
            self.prev_bot_position = self.bot_position
            self.prev_log_time = log_time
        return False

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Stop Action Received")
        
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position = self.bot_position
        self.goal_pose.pose.orientation = self.bot_orientation

        self.goal_publisher.publish(self.goal_pose)

        feedback_msg = StopAction.Feedback()
        rate = self.create_rate(2)  

        while True:
            if not self.check_movement():
                break
            
            self.goal_pose.pose.position = self.bot_position
            self.goal_pose.pose.orientation = self.bot_orientation
            self.goal_publisher.publish(self.goal_pose)
            feedback_msg.feedback = f"Linear: {self.current_cmd_vel.linear.x}, Angular: {self.current_cmd_vel.angular.z}"
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Current Velocity: {feedback_msg.feedback}")
            rate.sleep()

        self.get_logger().info("Stop Action Completed")
        goal_handle.succeed()
        result = StopAction.Result()
        result.success = True
        return result
    
def main():
    rclpy.init()
    node = StopServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
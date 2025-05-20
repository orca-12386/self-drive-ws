import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from interfaces.action import GoalAction as StopAction
from rclpy.executors import MultiThreadedExecutor
from actionlib_msgs.msg import GoalID
import time
import asyncio
import math


class StopServer(Node):
    def __init__(self):
        super().__init__('stop_server')
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cancel_publisher = self.create_publisher(GoalID, "/move_base/cancel",10)
        self.action_server = ActionServer(self, StopAction, 'StopAction', execute_callback=self.execute_callback)
        self.get_logger().info("Stop Server Started")
        self.bot_position = None
        self.bot_orientation = None
        self.goal_pose = None
        self.current_cmd_vel = None
        self.prev_bot_position = None
        self.prev_log_time = None

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation
        
    # def check_movement(self):
    #     log_time = time.time()
    #     if self.prev_bot_position is None or (log_time - self.prev_log_time) > 2:
    #         if not (self.prev_bot_position is None):
    #             self.get_logger().info(f"Current Change in Distance{math.sqrt((self.prev_bot_position.x - self.bot_position.x)**2 + (self.prev_bot_position.y - self.bot_position.y)**2)}")
    #             return math.sqrt((self.prev_bot_position.x - self.bot_position.x)**2 + (self.prev_bot_position.y - self.bot_position.y)**2) > 0.1
    #         self.prev_bot_position = self.bot_position
    #         self.prev_log_time = log_time
    #     return False

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Stop Action Received")
        
        feedback_msg = StopAction.Feedback()

        time.sleep(3)
        self.get_logger().info("Sending cancel action")
        cancel_msg = GoalID()
        cancel_msg.id = ""
        cancel_msg.stamp = self.get_clock().now().to_msg()
        self.cancel_publisher.publish(cancel_msg) 
        self.get_logger().info("Sent cancel action to move_base")
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info("Sent 0,0 cmd_vels")
        self.get_logger().info("Stop Action Completed")
        goal_handle.succeed()
        result = StopAction.Result()
        result.success = True
        return result
    
def main():
    rclpy.init()
    stop_node = StopServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(stop_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
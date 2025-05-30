import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from collections import deque
import math
from interfaces.action import GoalAction as LeftTurn
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN

def euler_to_quat(euler):
    return Rotation.from_euler('xyz', euler).as_quat()

def quat_to_euler(quat):
    return Rotation.from_quat(quat).as_euler('xyz') 

class LeftTurnHardCoded(Node):
    def __init__(self):
        super().__init__('Left_turn_node')
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map/white/local', self.map_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.action_server = ActionServer(self, LeftTurn, 'LeftTurn', execute_callback=self.execute_callback)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = None
        self.bot_position = None
        self.bot_orientation = None
        self.goal_pose = None
        self.lane_direction = None
        
        self.get_logger().info("Left Turn Node Started")

    def metres_to_grid(self, distance):
        if self.map_resolution is None:
            raise ValueError("Map resolution is not set")
        return distance/float(self.map_resolution)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin   

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation

    def world_to_map(self, x, y):
        map_x = int((x - self.map_origin.position.x) / self.map_resolution)
        map_y = int((y - self.map_origin.position.y) / self.map_resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        world_x = map_x * self.map_resolution + self.map_origin.position.x
        world_y = map_y * self.map_resolution + self.map_origin.position.y
        return world_x, world_y

    def get_yaw_from_quaternion(self, quat):
        euler = quat_to_euler([quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def calculate_intermediate_goal(self):
        bot_x, bot_y = self.world_to_map(self.bot_position.x, self.bot_position.y)
        print(bot_x, bot_y)
        bot_yaw = self.get_yaw_from_quaternion(self.bot_orientation)
        offset1_yaw = round(bot_yaw/(math.pi/2)) * (math.pi / 2)
        self.lane_direction = offset1_yaw
        offset1_yaw = offset1_yaw + math.pi/2
        offset1 = 2
        offset1 = self.metres_to_grid(offset1)
        goal_x = bot_x + offset1 * math.cos(offset1_yaw)
        goal_y = bot_y + offset1 * math.sin(offset1_yaw)

        offset2 = 5
        offset2 = self.metres_to_grid(offset2)
        offset2_yaw = offset1_yaw - math.pi/2
        print(offset2_yaw)
        goal_x = goal_x + offset2 * math.cos(offset2_yaw)
        goal_y = goal_y + offset2 * math.sin(offset2_yaw)

        goal_x_world, goal_y_world = self.map_to_world(goal_x, goal_y)

        goal_yaw = offset2_yaw + math.pi/3

        self.goal_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = goal_x_world
        self.goal_pose.pose.position.y = goal_y_world
        self.goal_pose.pose.position.z = 0.0

        quaternion = euler_to_quat([0, 0, goal_yaw])

        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]


    def calculate_final_goal(self):
        offset3 = 6
        offset3 = self.metres_to_grid(offset3)
        offset3_yaw = self.lane_direction + math.pi/2
        goal_x, goal_y = self.world_to_map(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
        goal_x = goal_x + offset3 * math.cos(offset3_yaw)
        goal_y = goal_y + offset3 * math.sin(offset3_yaw)

        offset4 = 1
        offset4 = self.metres_to_grid(offset4)
        offset4_yaw = offset3_yaw - math.pi/2
        goal_x = goal_x + offset4 * math.cos(offset4_yaw)
        goal_y = goal_y + offset4 * math.sin(offset4_yaw)
    
        goal_yaw = offset3_yaw 
        goal_x_world, goal_y_world = self.map_to_world(goal_x, goal_y)

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = goal_x_world
        self.goal_pose.pose.position.y = goal_y_world
        self.goal_pose.pose.position.z = 0.0

        quaternion = euler_to_quat([0, 0, goal_yaw])

        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]

    def publish_goal(self):
        if self.goal_pose is not None:
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_publisher.publish(self.goal_pose)
    
    def goal_reached(self):
        if self.goal_pose is None:
            return False
        distance = math.sqrt((self.goal_pose.pose.position.x - self.bot_position.x)**2 + (self.goal_pose.pose.position.y - self.bot_position.y)**2)
        return distance < 0.2


    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing Left Turn Action")
        try:
            self.calculate_intermediate_goal()
            if self.goal_pose is None:
                    self.get_logger().info("Could Not Calculate Goal")
                    goal_handle.abort()
                    return LeftTurn.Result()

            self.publish_goal()

            rate = self.create_rate(5)

            while rclpy.ok():
                if self.goal_reached():
                    self.get_logger().info("Intermediate Goal Reached")
                    break
                self.publish_goal()
                rate.sleep()

            self.calculate_final_goal()
            
            self.publish_goal()
            while rclpy.ok():
                if self.goal_reached():
                    self.get_logger().info("Final Goal Reached")
                    result = LeftTurn.Result()
                    result.success = True
                    goal_handle.succeed()
                    return result
                self.publish_goal()
                rate.sleep()

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            goal_handle.abort()
            return LeftTurn.Result()


def main(args=None):
    rclpy.init(args=args)
    Left_turn_node = LeftTurnHardCoded()

    executor = MultiThreadedExecutor()
    executor.add_node(Left_turn_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        Left_turn_node.destroy_node()
        rclpy.shutdown()



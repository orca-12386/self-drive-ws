import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from collections import deque
import math
from interfaces.action import GoalAction as RightTurn
import asyncio
from scipy.spatial.transform import Rotation

def euler_to_quat(euler):
    return Rotation.from_euler('xyz', euler).as_quat()

def quat_to_euler(quat):
    return Rotation.from_quat(quat).as_euler('xyz') 

class RightTurnNode(Node):
    def __init__(self):
        super().__init__('right_turn_node')
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map/white', self.map_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.action_server = ActionServer(self, RightTurn, 'RightTurn', execute_callback=self.execute_callback)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = None
        self.bot_position = None
        self.bot_orientation = None
        self.goal_pose = None
        self.right_lane_point = None
        self.farthest_point = None

        self.get_logger().info("Right Turn Node Started")

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

    def find_right_lane_point(self):
        if self.bot_position is None:
            self.get_logger().info("Odometry Not Received")
            return
        if self.map_data is None:
            self.get_logger().info("Map Data Not Received")
            return
        
        self.get_logger().info("Searching for right lane")
        bot_x, bot_y = self.world_to_map(self.bot_position.x, self.bot_position.y)
        start = (bot_x, bot_y)
        queue = deque([start])
        visited = {start}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
        while queue:
            x, y = queue.popleft()
            if self.map_data[y, x] > 0:
                
                is_right = False
                bot_yaw = self.get_yaw_from_quaternion(self.bot_orientation)

                if math.pi / 4 <= bot_yaw<= 3 * math.pi / 4:  
                    is_right = y - 2 < bot_y < y + 2
                elif -3 * math.pi / 4 <= bot_yaw <= -math.pi / 4:  
                    is_right = y - 2 < bot_y < y + 2
                elif (-math.pi <= bot_yaw < -3 * math.pi / 4) or (3 * math.pi / 4 < bot_yaw <= math.pi):  
                    is_right = x - 2 < bot_x < x + 2
                elif -math.pi / 4 < bot_yaw < math.pi / 4:  
                    is_right = x - 2 < bot_x < x + 2

                if is_right:
                    self.get_logger().info(f"Right Lane Found: ({x}, {y})")
                    return (x, y)

            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.map_width and 0 <= ny < self.map_height and (nx, ny) not in visited):
                    queue.append((nx, ny))
                    visited.add((nx, ny))
        return None

    def bfs_farthest_lane_point(self, start):
        if start is None:
            return None

        yaw = self.get_yaw_from_quaternion(self.bot_orientation)
        fx, fy = math.cos(yaw), math.sin(yaw)  

        all_directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        
        directions = []
        for dx, dy in all_directions:
            dot_product = dx * fx + dy * fy  
            if dot_product > 0:  
                directions.append((dx, dy))

        queue = deque([start])
        visited = {start}
        farthest = start

        while queue:
            x, y = queue.popleft()

            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.map_width and 0 <= ny < self.map_height and 
                    self.map_data[ny, nx] > 0 and (nx, ny) not in visited):
                    queue.append((nx, ny))
                    visited.add((nx, ny))
                    farthest = (nx, ny)

        if farthest == start:
            self.get_logger().info("Farthest Lane Point not Found")
            return None

        world_x, world_y = self.map_to_world(farthest[0], farthest[1])
        self.get_logger().info(f"Farthest Lane Point Found: ({world_x}, {world_y})")
        return farthest

    def calculate_goal(self):
        if self.bot_position is None or self.map_data is None:
            return
        self.right_lane_point = self.find_right_lane_point()

        if self.right_lane_point is None:
            return
        self.farthest_point = self.bfs_farthest_lane_point(self.right_lane_point)
        if self.farthest_point is None:
            return
        
        right_map_x, right_map_y = self.right_lane_point
        right_world_x, right_world_y = self.map_to_world(right_map_x, right_map_y)

        direction_x = right_world_x - self.bot_position.x
        direction_y = right_world_y - self.bot_position.y

        goal_yaw = math.atan2(direction_y, direction_x)

        perpendicular_slope = goal_yaw + math.pi / 2
        perpendicular_direction_x = math.cos(perpendicular_slope)
        perpendicular_direction_y = math.sin(perpendicular_slope)

        goal_x, goal_y = self.map_to_world(self.farthest_point[0], self.farthest_point[1])

        right_lane_world_x, right_lane_world_y = self.map_to_world(self.right_lane_point[0], self.  right_lane_point[1])
        offset_distance = math.sqrt((right_lane_world_x - self.bot_position.x)**2 + (right_lane_world_y - self.bot_position.y)**2)

        self.get_logger().info(f"Offset Distance: {offset_distance}")
        
        offset_x = offset_distance * perpendicular_direction_x
        offset_y = offset_distance * perpendicular_direction_y
        goal_x += offset_x
        goal_y += offset_y

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = goal_x
        self.goal_pose.pose.position.y = goal_y
        self.goal_pose.pose.position.z = 0.0

        quaternion = euler_to_quat([0, 0, goal_yaw])
        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]

        self.get_logger().info(f"Goal Published: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")

    def publish_goal(self):
        if self.goal_pose is not None:
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_publisher.publish(self.goal_pose)
    
    def goal_reached(self):
        if self.goal_pose is None:
            return False
        distance = math.sqrt((self.goal_pose.pose.position.x - self.bot_position.x)**2 + (self.goal_pose.pose.position.y - self.bot_position.y)**2)
        return distance < 0.2

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing Right Turn")
        try:
            self.calculate_goal()
            if self.goal_pose is None:
                self.get_logger().info("Could Not Calculate Goal")
                goal_handle.abort()
                return RightTurn.Result()

            self.publish_goal()

            while rclpy.ok():
                if self.goal_reached():
                    self.get_logger().info("Goal Reached")
                    goal_handle.succeed()
                    result = RightTurn.Result()
                    result.success = True
                    return result
                self.publish_goal()
                asyncio.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            goal_handle.abort()
            return RightTurn.Result()
                
def main(args=None):
    rclpy.init(args=args)
    node = RightTurnNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        self.get_logger().info("Keyboard Interrupt")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
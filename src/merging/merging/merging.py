import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import math
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
import time


def quat_to_euler(quat):
    return Rotation.from_quat(quat).as_euler('xyz')

def euler_to_quat(euler):
    return Rotation.from_euler('xyz', euler).as_quat()



class Merging(Node):
    def __init__(self):
        super().__init__('merging_node')

        self.create_subscription(OccupancyGrid, '/map/white/local', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_timer(0.5, self.publish_goal)
        
        self.initial_distance = None
        self.max_offset = 5
        self.map_data = None
        self.goal_pose = None
        self.bot_position = None
        self.bot_orientation = None

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.map_width = msg.info.width
        self.map_height = msg.info.height

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation

    def get_yaw_from_quaternion(self, quat):
        euler = quat_to_euler([quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def world_to_map(self, x, y):
        map_x = int((x - self.map_origin.x) / self.map_resolution)
        map_y = int((y - self.map_origin.y) / self.map_resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        world_x = map_x * self.map_resolution + self.map_origin.x
        world_y = map_y * self.map_resolution + self.map_origin.y
        return world_x, world_y

    def find_merging(self):
        if self.bot_position is None or self.map_data is None:
            self.get_logger().warn("Waiting for map/odom data")
            return False

        # self.get_logger().info("Finding merging point")

        bot_x, bot_y = self.world_to_map(self.bot_position.x, self.bot_position.y)
        # print(f"Bot Position: {self.bot_position.x}, {self.bot_position.y}")
        # print(f"Bot Map Position: {bot_x}, {bot_y}")
        start = (bot_x, bot_y)

        queue = deque([start])
        visited = {start}
        bot_yaw = self.get_yaw_from_quaternion(self.bot_orientation)

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while queue:
            x, y = queue.popleft()
            if not (0 <= x < self.map_width and 0 <= y < self.map_height):
                continue

            if self.map_data[y, x] != 0:
                valid = False

                if math.pi / 4 <= bot_yaw <= 3 * math.pi / 4:
                    valid = (bot_y == y) and (x < bot_x)
                elif -3 * math.pi / 4 <= bot_yaw <= -math.pi / 4:
                    valid = (bot_y == y) and (x > bot_x)
                elif abs(bot_yaw) > 3 * math.pi / 4:
                    valid = (bot_x == x) and (y < bot_y)
                elif -math.pi / 4 < bot_yaw < math.pi / 4:
                    valid = (bot_x == x) and (y > bot_y)

                if valid:
                    # print(f"Obstacle at: {x}, {y}")
                    new_distance = math.sqrt((x - bot_x) ** 2 + (y - bot_y) ** 2)
                    # print(f"Distance: {new_distance}")
                    if self.initial_distance is None:
                        self.initial_distance = new_distance
                    elif abs(new_distance - self.initial_distance) > self.max_offset:
                        # print(f"Distance: {abs(new_distance - self.initial_distance)}")
                        self.get_logger().info(f"Found merging point near: ({x}, {y})")
                        return (x,y)
                    else:
                        return None, None

            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.map_width) and (0 <= ny < self.map_height) and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

    def move_to_next_lane(self, next_lane_x, next_lane_y):
        if next_lane_x is None or next_lane_y is None:
            return None, None

        all_directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        new_directions = list()
        skip_dist = 1.5 # in metres
        skip_dist = skip_dist / self.map_resolution # grid coords
        for i in range(2, int(skip_dist)+1):
            for j in range(len(all_directions)):
                new_directions.append((all_directions[j][0]*i, all_directions[j][1]*i))
        all_directions.extend(new_directions)

        yaw = self.get_yaw_from_quaternion(self.bot_orientation)
        fx, fy = math.cos(yaw), math.sin(yaw)
        
        directions = []
        for dx, dy in all_directions:
            dot_product = dx * fx + dy * fy  
            if dot_product > 0:  
                directions.append((dx, dy))
        
        start = (next_lane_x, next_lane_y)
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
        if self.goal_pose is None and self.bot_position and self.bot_orientation and self.map_data is not None:
            self.get_logger().info("Calculating Goal")

            current_lane_x, current_lane_y = self.find_merging()
            if current_lane_x is None or current_lane_y is None:
                self.get_logger().warn("No merging point found")
                return

            bot_x, bot_y = self.world_to_map(self.bot_position.x, self.bot_position.y)

            offset_distance = math.sqrt((current_lane_x - bot_x) ** 2 + (current_lane_y - bot_y) ** 2)

            next_lane_x, next_lane_y= self.move_to_next_lane(current_lane_x, current_lane_y)

            next_lane_x_world, next_lane_y_world = self.map_to_world(next_lane_x, next_lane_y)

            goal_yaw = math.atan2(next_lane_y_world - self.bot_position.y, next_lane_x_world - self.bot_position.x)
            
            offset_dir = goal_yaw - math.pi/2

            offset_dir_x = math.cos(offset_dir)
            offset_dir_y = math.sin(offset_dir)

            offset_x = offset_distance * offset_dir_x
            offset_y = offset_distance * offset_dir_y
            goal_x = next_lane_x + offset_x
            goal_y = next_lane_y + offset_y

            goal_x_world, goal_y_world= self.map_to_world(goal_x, goal_y)

            quaternion = euler_to_quat([0, 0, goal_yaw])

            self.goal_pose = PoseStamped()
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = goal_x_world
            self.goal_pose.pose.position.y = goal_y_world
            self.goal_pose.pose.position.z = 0.0
            
            self.goal_pose.pose.orientation.x = quaternion[0]
            self.goal_pose.pose.orientation.y = quaternion[1]
            self.goal_pose.pose.orientation.z = quaternion[2]
            self.goal_pose.pose.orientation.w = quaternion[3]

            self.get_logger().info(f"Merging Lane Goal: {self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y}")

    def publish_goal(self):
        if self.goal_pose is None:
            self.calculate_goal()
        else:
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_publisher.publish(self.goal_pose)


def main(args=None):
    rclpy.init(args=args)
    node = Merging()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

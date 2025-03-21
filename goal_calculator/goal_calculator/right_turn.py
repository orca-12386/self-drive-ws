import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
import tf_transformations
from collections import deque
import math

class RightTurnNode(Node):
    def __init__(self):
        super().__init__('right_turn_node')
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.timer = self.create_timer(1.0, self.publish_goal)

        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = None
        self.bot_position = None
        self.bot_orientation = None
        self.goal_pose = None

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation
        if self.goal_pose is None:
            self.calculate_goal()

    def world_to_map(self, x, y):
        map_x = int((x - self.map_origin.position.x) / self.map_resolution)
        map_y = int((y - self.map_origin.position.y) / self.map_resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        world_x = map_x * self.map_resolution + self.map_origin.position.x
        world_y = map_y * self.map_resolution + self.map_origin.position.y
        return world_x, world_y

    def get_yaw_from_quaternion(self, quat):
        euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def find_right_lane_point(self):

        bot_x, bot_y = self.world_to_map(self.bot_position.x, self.bot_position.y)
        yaw = self.get_yaw_from_quaternion(self.bot_orientation)

        right_vec_x = math.cos(yaw - math.pi / 2)
        right_vec_y = math.sin(yaw - math.pi / 2)

        map_height, map_width = self.map_data.shape

        distance = 0
        while True:
            distance += 1
            check_x = int(bot_x + right_vec_x * distance)
            check_y = int(bot_y + right_vec_y * distance)

            if 0 <= check_x < map_width and 0 <= check_y < map_height:
                if self.map_data[check_y, check_x] > 0:  
                    world_x, world_y = self.map_to_world(check_x, check_y)
                    self.get_logger().info(f"Right Lane Point Found: ({world_x}, {world_y})")
                    return check_x, check_y
            else:
                break

        self.get_logger().warn("No valid right lane point found.")
        return None

    def bfs_farthest_lane_point(self, start):
        if start is None:
            return None

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
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

        world_x, world_y = self.map_to_world(farthest[0], farthest[1])
        return farthest

    def calculate_goal(self):
        if self.bot_position is None or self.map_data is None:
            return

        right_lane_point = self.find_right_lane_point()
        if right_lane_point is None:
            return

        farthest_point = self.bfs_farthest_lane_point(right_lane_point)
        if farthest_point is None:
            return

        goal_x, goal_y = self.map_to_world(farthest_point[0], farthest_point[1])

        right_lane_world_x, right_lane_world_y = self.map_to_world(right_lane_point[0], right_lane_point[1])
        offset_distance = math.sqrt((right_lane_world_x - self.bot_position.x)**2 + (right_lane_world_y - self.bot_position.y)**2)

        yaw = self.get_yaw_from_quaternion(self.bot_orientation)
        offset_x = offset_distance * math.cos(yaw)
        offset_y = offset_distance * math.sin(yaw)
        goal_x += offset_x
        goal_y += offset_y

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = goal_x
        self.goal_pose.pose.position.y = goal_y
        self.goal_pose.pose.position.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw - math.pi/2)
        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]

    def publish_goal(self):
        if self.goal_pose is not None:
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_publisher.publish(self.goal_pose)
            self.get_logger().info(f"Goal Published: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")

def main(args=None):
    rclpy.init(args=args)
    node = RightTurnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
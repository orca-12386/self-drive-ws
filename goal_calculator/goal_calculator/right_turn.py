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
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map/white/local', self.map_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.right_lane_publisher = self.create_publisher(PointStamped, '/right_lane_point', 10)
        self.farthest_lane_publisher = self.create_publisher(PointStamped, '/farthest_lane_point', 10)

        self.timer = self.create_timer(0.1, self.publish_goal)

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
        self.get_logger().info("Searching for right lane")
        bot_x, bot_y = self.world_to_map(self.bot_position.x, self.bot_position.y)
        start = (bot_x, bot_y)
        queue = deque([start])
        visited = {start}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
        while queue:
            x, y = queue.popleft()
            if self.map_data[y, x] > 0:
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

        quaternion = tf_transformations.quaternion_from_euler(0, 0, goal_yaw)
        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]

        self.get_logger().info(f"Goal Published: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")

    def publish_goal(self):
        if self.goal_pose is not None:
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_publisher.publish(self.goal_pose)
        
        if self.right_lane_point is not None:
            right_lane_msg = PointStamped()
            right_lane_msg.header.frame_id = 'map'
            right_lane_msg.header.stamp = self.get_clock().now().to_msg()
            right_lane_msg.point.x, right_lane_msg.point.y = self.map_to_world(*self.right_lane_point)
            self.right_lane_publisher.publish(right_lane_msg)

        if self.farthest_point is not None:
            farthest_msg = PointStamped()
            farthest_msg.header.frame_id = 'map'
            farthest_msg.header.stamp = self.get_clock().now().to_msg()
            farthest_msg.point.x, farthest_msg.point.y = self.map_to_world(*self.farthest_point)
            self.farthest_lane_publisher.publish(farthest_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RightTurnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
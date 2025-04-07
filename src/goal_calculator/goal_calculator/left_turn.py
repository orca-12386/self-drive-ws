import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
import os
import numpy as np
import tf_transformations
from sklearn.cluster import DBSCAN
import math
from collections import deque
from scipy.spatial.distance import cdist
from interfaces.action import GoalAction as LeftTurn
import asyncio

class LeftTurnNode(Node):
    def __init__(self):
        super().__init__('left_turn_node')
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map/yellow/local', self.map_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.action_server = ActionServer(self, LeftTurn, 'LeftTurn', execute_callback=self.execute_callback)
        # self.create_timer(0.1, self.publish_goal)
        
        self.Midpoint = None
        self.closest_yellow_lane = None
        self.final_goal = None
        self.mid_x_world = None
        self.mid_y_world = None
        self.perpendicular_direction = None
        self.lane_offset = None
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = None
        self.bot_position = None
        self.bot_orientation = None
        self.goal_pose = None

        self.get_logger().info("Left Turn Server Started")

    def map_callback(self, msg):
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.map_resolution = msg.info.resolution
            self.map_origin = msg.info.origin

            # if self.map_data is not None and self.bot_position is not None and self.final_goal is None:
            #     self.calculate_goal()

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
            euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]


    def find_intersection_direction(self):
            self.get_logger().info("Searching for Intersection Direction")
            bot_x, bot_y = self.world_to_map(self.bot_position.x, self.bot_position.y)
            
            yaw = self.get_yaw_from_quaternion(self.bot_orientation)
            closest_lane_point_x = 0
            closest_lane_point_y = 0

            queue = deque([(bot_x, bot_y)])
            visited = {(bot_x, bot_y)}

            directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

            while queue:
                x, y = queue.popleft()
                if self.map_data[y, x] > 0:
                    self.get_logger().info(f"Found Closest Yellow Lane Point: ({x}, {y})")
                    closest_lane_point_x = x
                    closest_lane_point_y = y
                    break
                
                for dx, dy in directions:
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < self.map_width and 0 <= ny < self.map_height and (nx, ny) not in visited):
                        queue.append((nx, ny))
                        visited.add((nx, ny))
            
            direction_x = bot_x - closest_lane_point_x
            direction_y = bot_y - closest_lane_point_y

            direction = math.atan2(direction_y, direction_x)
            return direction, closest_lane_point_x, closest_lane_point_y

    def find_intersection_midpoint(self):
        if self.Midpoint is not None:
            return
        self.get_logger().info("Calculating Intermediate Goal")
        yellow_lane_points = np.argwhere(self.map_data > 0)
        yellow_clusters = DBSCAN(eps=1, min_samples=4, algorithm='auto', metric='euclidean').fit(yellow_lane_points)
        labels = yellow_clusters.labels_

        unique_clusters = np.unique(labels)
        unique_clusters = unique_clusters[unique_clusters != -1]

        direction, closest_lane_x, closest_lane_y = self.find_intersection_direction()
        closest_point = np.array([closest_lane_y, closest_lane_x])

        cluster_distances = []
        for cluster_label in unique_clusters:
            cluster_points = yellow_lane_points[labels == cluster_label]
            min_distance = np.min(np.linalg.norm(cluster_points - closest_point, axis=1))
            cluster_distances.append((cluster_label, min_distance))

        cluster_distances.sort(key=lambda x: x[1])

        lane1_cluster_label = cluster_distances[0][0]
        lane2_cluster_label = None
        
        bot_yaw = self.get_yaw_from_quaternion(self.bot_orientation)
        for cluster in cluster_distances:
            if cluster == cluster_distances[0]:
                continue
            cluster_label = cluster[0]
            cluster_points = yellow_lane_points[labels == cluster_label]
            cluster_center = np.mean(cluster_points, axis=0)

            cluster_center_x = cluster_center[1]
            cluster_center_y = cluster_center[0]

            invalid_cluster = False

            if math.pi / 4 <= bot_yaw <= 3 * math.pi / 4:  
                invalid_cluster = cluster_center_y < closest_lane_y
            elif -3 * math.pi / 4 <= bot_yaw <= -math.pi / 4:  
                invalid_cluster = cluster_center_y > closest_lane_y
            elif (-math.pi <= bot_yaw -3 * math.pi / 4) or (3 * math.pi / 4 < bot_yaw <= math.pi):  
                invalid_cluster = cluster_center_x > closest_lane_x
            elif -math.pi / 4 < bot_yaw < math.pi / 4:  
                invalid_cluster = cluster_center_x < closest_lane_x

            if invalid_cluster:
                continue
            lane2_cluster_label = cluster_label
            break

        lane1_cluster = yellow_lane_points[labels == lane1_cluster_label]
        lane2_cluster = yellow_lane_points[labels == lane2_cluster_label]

        lane1_point = lane1_cluster[0]
        lane2_point = lane2_cluster[0]

        x1, y1 = lane1_point[0], lane1_point[1]
        x2, y2 = lane2_point[0], lane2_point[1]

        m1 = math.tan(direction)  
        m2 = math.tan(direction + math.pi / 2)  

        A1, B1, C1 = -m1, 1, -m1 * x1 + y1
        A2, B2, C2 = -m2, 1, -m2 * x2 + y2

        det = A1 * B2 - A2 * B1

        mid_x = (C1 * B2 - C2 * B1) / det
        mid_y = (A1 * C2 - A2 * C1) / det   

        bot_pose_x, bot_pose_y = self.world_to_map(self.bot_position.x, self.bot_position.y)

        closest_lane_x_world, closest_lane_y_world = self.map_to_world(closest_lane_x, closest_lane_y)

        self.lane_offset = math.sqrt((bot_pose_x - closest_lane_x)**2 + (bot_pose_y- closest_lane_y)**2)
        self.get_logger().info(f"Lane Offset: {self.lane_offset}")

        mid_x_world, mid_y_world = self.map_to_world(mid_y, mid_x)

        goal_yaw = math.atan2(mid_y_world - closest_lane_y_world, mid_x_world - closest_lane_x_world) + math.pi / 4
        perpendicular_direction = math.atan2(mid_y_world - closest_lane_y_world, mid_x_world - closest_lane_x_world) 
        
        self.Midpoint = PoseStamped()
        self.Midpoint.header.frame_id = 'map'
        self.Midpoint.header.stamp = self.get_clock().now().to_msg()
        self.Midpoint.pose.position.x = mid_x_world
        self.Midpoint.pose.position.y = mid_y_world
        self.Midpoint.pose.position.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0, 0, goal_yaw)
        self.Midpoint.pose.orientation.x = quaternion[0]
        self.Midpoint.pose.orientation.y = quaternion[1]
        self.Midpoint.pose.orientation.z = quaternion[2]
        self.Midpoint.pose.orientation.w = quaternion[3]

        self.get_logger().info(f"Midpoint Published: ({self.Midpoint.pose.position.x}, {self.Midpoint.pose.position.y})")

        self.mid_x_world = mid_x_world
        self.mid_y_world = mid_y_world 
        self.perpendicular_direction = perpendicular_direction
        return

    def calculate_goal(self):
        if self.bot_position is None or self.map_data is None:
            return

        if self.Midpoint is None:
            self.get_logger().info("Calculating Midpoint")
            self.find_intersection_midpoint()
            return

        if self.final_goal is None and self.Midpoint is not None:
            self.get_logger().info("Calculating Final Goal")

            directions = [(1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0)]

            start_x, start_y = self.world_to_map(self.bot_position.x, self.bot_position.y)
            bot_yaw = self.get_yaw_from_quaternion(self.bot_orientation)

            queue = deque([(start_x, start_y)])
            visited = {(start_x, start_y)}

            next_lane_point_x = 0
            next_lane_point_y = 0

            while queue:
                x, y = queue.popleft()
                bot_position_x, bot_position_y = self.world_to_map(self.bot_position.x, self.bot_position.y)

                if self.map_data[y, x] > 0:
                    is_right = False

                    if math.pi / 4 <= self.perpendicular_direction <= 3 * math.pi / 4:  
                        is_right = x < bot_position_x - 10
                    elif -3 * math.pi / 4 <= self.perpendicular_direction <= -math.pi / 4:  
                        is_right = x > bot_position_x + 10
                    elif (-math.pi <= self.perpendicular_direction < -3 * math.pi / 4) or (3 * math.pi / 4 < self.perpendicular_direction <= math.pi):  
                        is_right = y < bot_position_y - 10
                    elif -math.pi / 4 < self.perpendicular_direction < math.pi / 4:  
                        is_right = y > bot_position_y + 10

                    if is_right:
                        next_lane_point_x, next_lane_point_y = x, y
                        break

                for dx, dy in directions:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height and (nx, ny) not in visited:
                        queue.append((nx, ny))
                        visited.add((nx, ny))

            if 0 <= next_lane_point_x < self.map_width and 0 <= next_lane_point_y < self.map_height:

                perpendicular_direction_x = math.cos(self.perpendicular_direction)
                perpendicular_direction_y = math.sin(self.perpendicular_direction)

                offset_x = self.lane_offset * perpendicular_direction_x
                offset_y = self.lane_offset * perpendicular_direction_y

                goal_x = next_lane_point_x + offset_x
                goal_y = next_lane_point_y + offset_y

                goal_x_world, goal_y_world = self.map_to_world(goal_x, goal_y)

                self.final_goal = PoseStamped()
                self.final_goal.header.frame_id = 'map'
                self.final_goal.header.stamp = self.get_clock().now().to_msg()
                self.final_goal.pose.position.x = goal_x_world
                self.final_goal.pose.position.y = goal_y_world
                self.final_goal.pose.position.z = 0.0

                quaternion = tf_transformations.quaternion_from_euler(0, 0, self.perpendicular_direction + math.pi / 2)
                self.final_goal.pose.orientation.x = quaternion[0]
                self.final_goal.pose.orientation.y = quaternion[1]
                self.final_goal.pose.orientation.z = quaternion[2]
                self.final_goal.pose.orientation.w = quaternion[3]

                self.get_logger().info(f"Final Goal: ({self.final_goal.pose.position.x}, {self.final_goal.pose.position.y})")
            else:
                self.get_logger().info(f"Calculated goal is outside the map boundaries")

    def publish_goal(self):
            if self.Midpoint is not None and self.final_goal is None:
                self.Midpoint.header.stamp = self.get_clock().now().to_msg()
                self.goal_publisher.publish(self.Midpoint)
                self.get_logger().info("Midpoint Published")
            
            if self.final_goal is not None:
                self.final_goal.header.stamp = self.get_clock().now().to_msg()
                self.goal_publisher.publish(self.final_goal)
                self.get_logger().info("Final Goal Published")

    def goal_reached(self, goal):
        if goal is None:
            return False
        distance = math.sqrt((goal.pose.position.x - self.bot_position.x)**2 + (goal.pose.position.y - self.bot_position.y)**2)
        return distance < 0.2

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing Left Turn")
        try:
            self.calculate_goal()
            if self.final_goal is None and self.Midpoint is None:
                self.get_logger().info("Could Not Calculate Goal")
                goal_handle.abort()
                return LeftTurn.Result()

            if self.Midpoint is not None and self.final_goal is None:
                self.publish_goal()
                while rclpy.ok():
                    if self.goal_reached(self.Midpoint):
                        self.get_logger().info("Midpoint Reached")
                        break
                    self.publish_goal()
                    asyncio.sleep(0.1)

            self.calculate_goal()
            self.publish_goal()

            while rclpy.ok():
                if self.goal_reached(self.final_goal):
                    self.get_logger().info("Goal Reached")
                    goal_handle.succeed()
                    result = LeftTurn.Result()
                    result.success = True
                    return result
                self.publish_goal()
                asyncio.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            goal_handle.abort()
            return LeftTurn.Result()       
                
def main(args=None):

    rclpy.init(args=args)
    left_turn_node = LeftTurnNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(left_turn_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        self.get_logger().info("Keyboard Interrupt")
    finally:
        executor.shutdown()
        left_turn_node.destroy_node()
        rclpy.shutdown()
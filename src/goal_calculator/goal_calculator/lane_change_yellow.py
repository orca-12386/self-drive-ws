#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
import asyncio
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from interfaces.action import GoalAction as LaneChangeAction
from std_msgs.msg import Bool as LaneChangeStatus

# These are your package-specific imports.
from goal_calculator.ros2_wrapper import Message, Subscription, Publisher, Config, NodeGlobal
import goal_calculator.lane_follow as util
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree
import statistics
from collections import defaultdict


# def yaw_to_quaternion(yaw):
#     """Converts a yaw angle (in radians) to a quaternion."""
#     qx = 0.0
#     qy = 0.0
#     qz = math.sin(yaw / 2)
#     qw = math.cos(yaw / 2)
#     return qx, qy, qz, qw

def extrapolate_points(x1, y1, x2, y2, distance):
    dx, dy = x2-x1, y2-y1
    length = np.hypot(dx, dy)
    dx /= length
    dy /= length

    newx = x2 + dx * distance
    newy = y2 + dy * distance
    
    return newx, newy

class LaneChange(Node):
    def __init__(self):
        super().__init__("lane_change_node")
        NodeGlobal.obj = self
        self.declare_parameter('config_file_path')
        Config.read_config(self.get_parameter('config_file_path').value)
        NodeGlobal.log_info("Config read: "+self.get_parameter('config_file_path').value)


        self._action_server = ActionServer(
            self,
            LaneChangeAction,
            'lane_change',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback_wrapper
        )

        # Create subscriptions.
        subscription_info = {
            "map": ["map/yellow/local", OccupancyGrid],
            "odom": ["odom", Odometry],
            "robot_pose_global": ["map/robot_pose_global", PoseStamped],
            "robot_pose_grid": ["map/robot_pose_grid", PoseStamped],
            "robot_orientation": ["map/robot_orientation", Float64, 10]
        }
        Subscription.create_subscriptions(subscription_info)

        publisher_info = {
            "goal_pose": ["move_base_simple/goal1", PoseStamped],
            "lane_change_status": ["lane_change_status", LaneChangeStatus]
        }
        Publisher.create_publishers(publisher_info)

        self.lane_change_active = False
        self.publish_status(False)
        self.goals = list()
        self.timer = self.create_timer(0.1, self.publish_status_periodically)
        self.timer2 = self.create_timer(2, self.add_goal_pose)
        self.furthest1_pub = self.create_publisher(PoseStamped, '/furthest_point1_global', 10)
        self.furthest2_pub = self.create_publisher(PoseStamped, '/furthest_point2_global', 10)

    def publish_furthest_points(self, point1, point2, orientation_yaw=0.0):
        def make_pose_stamped(x, y, yaw):
            pose = PoseStamped()
            pose.header.frame_id = "odom"  # Use "map" if that's your global frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            return pose

        pose1 = make_pose_stamped(point1[0], point1[1], orientation_yaw)
        pose2 = make_pose_stamped(point2[0], point2[1], orientation_yaw)
        self.furthest1_pub.publish(pose1)
        self.furthest2_pub.publish(pose2)
        self.get_logger().info(f"Published furthest points: {point1}, {point2}")
    
    def calculate_angle(self, candidate_goal, previous_goal, second_previous_goal):
        v1 = np.array(candidate_goal) - np.array(previous_goal)
        v2 = np.array(second_previous_goal) - np.array(previous_goal)
        
        if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
            return 0.0
        
        angle = np.arctan2(np.linalg.norm(np.cross(v1, v2)), np.dot(v1, v2))
        return np.degrees(angle)



    def add_goal_pose(self):
        robot_coords_global = Subscription.subs["robot_pose_global"].get_latest_data()
        self.goals.append(np.array(robot_coords_global))

    def publish_status(self, status_bool):
        status_msg = LaneChangeStatus()
        status_msg.data = status_bool
        Publisher.pubs["lane_change_status"].publish(status_msg)

    def publish_status_periodically(self):
        self.publish_status(self.lane_change_active)

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: lane_change = {goal_request.data}")
        if goal_request.data == 1:
            self.get_logger().info("Lane change action received")
            self.lane_change_active = True
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def execute_callback_wrapper(self, goal_handle):
        return asyncio.run(self.async_execute_callback(goal_handle))

    async def async_execute_callback(self, goal_handle):
        NodeGlobal.log_info("Lane change initializing")
        initial_coords = Subscription.subs["robot_pose_grid"].get_latest_data()
        
        try:
            goal_pose = self.get_goal_pose(initial_coords)
            
            if goal_pose is None:
                NodeGlobal.log_error("Failed to compute goal pose.")
                goal_handle.abort()
                return LaneChangeAction.Result()

            self.publish_goal(goal_pose)

            while rclpy.ok():
                curr_coords = Subscription.subs["robot_pose_global"].get_latest_data()
                if self.goal_reached(curr_coords, goal_pose):
                    NodeGlobal.log_info("Lane change complete.")
                    self.lane_change_active = False
                    self.publish_status(False)
                    goal_handle.succeed()
                    return LaneChangeAction.Result()
                await asyncio.sleep(0.1)
        except Exception as e:
            NodeGlobal.log_error(f"Error during lane change execution: {str(e)}")
            goal_handle.abort()
            return LaneChangeAction.Result()

    def goal_reached(self, curr_robot_coords, goal_pose):
        goal_coords = (goal_pose.pose.position.x, goal_pose.pose.position.y)
        distance = util.calculate_distance(curr_robot_coords, goal_coords)
        # Using a configurable threshold would be better
        distance_threshold = Config.config.get("goal_distance_threshold", 1.0)
        return distance < distance_threshold

    def create_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "odom"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        robot_orientation_data = Subscription.subs["robot_orientation"].get_all_data()
        # NodeGlobal.log_info(robot_orientation_data)
        if not robot_orientation_data:
            # Fallback if no orientation data is available
            avg_orientation = 0.0
            NodeGlobal.log_error("No orientation data available, using default 0.0")
        else:
            avg_orientation = statistics.fmean(robot_orientation_data)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(avg_orientation / 2)
        goal_pose.pose.orientation.w = math.cos(avg_orientation / 2)

        return goal_pose

    def get_goal_pose(self, robot_coords_grid):
        def get_all_clusters(binary_array, eps=Config.config["dbscan_eps"], 
                            min_samples=Config.config["dbscan_min_samples"]):
            points = np.argwhere(binary_array > 0)
            points = points[:, [1, 0]]  # Swap columns for (x,y) format
            
            if len(points) < min_samples:
                return []
            
            dbscan = DBSCAN(eps=eps, min_samples=min_samples)
            clusters = dbscan.fit_predict(points)
            return [points[clusters == label] for label in np.unique(clusters) if label != -1]

        try:
            map_data = Subscription.subs["map"].get_latest_data()
            if map_data is None:
                NodeGlobal.log_error("No map data available")
                return None

            all_clusters = get_all_clusters(map_data)
            NodeGlobal.log_info(f"Found {len(all_clusters)} clusters")

            if not all_clusters:
                NodeGlobal.log_error("No clusters detected")
                return None

            # Find cluster closest to robot
            robot_xy = np.array([robot_coords_grid[0], robot_coords_grid[1]])
            min_distance = float('inf')
            closest_cluster = None

            for cluster in all_clusters:
                if len(cluster) < Config.config["minimum_points"]:
                    continue  # Skip small clusters

                # Find nearest point in cluster to robot
                tree = cKDTree(cluster)
                distance, _ = tree.query(robot_xy, k=1)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_cluster = cluster

            if closest_cluster is None:
                NodeGlobal.log_error("No valid clusters found")
                return None

            tree = cKDTree(closest_cluster)
            _, nearest_index = tree.query(robot_xy, k=1)
            first_point = closest_cluster[nearest_index]
            
            def find_furthest_point(cluster, start, max_search_distance=Config.config["max_search_distance"]):
                visited = set()
                furthest_point = start
                queue = []
                queue.append([start, 0])  # Start with search distance 0
                tree = cKDTree(cluster)

                while queue:
                    node, current_search_distance = queue.pop(0)
                    node_tuple = tuple(node)

                    if current_search_distance > max_search_distance:
                        continue  # Allow other points in the queue to be processed

                    if node_tuple in visited:
                        continue

                    visited.add(node_tuple)
                    furthest_point = node  # Update with the last visited point within limit

                    indices = tree.query_ball_point(node, 3)
                    for i in indices:
                        neighbour = cluster[i]
                        if tuple(neighbour) in visited:
                            continue

                        neighbour_search_distance = current_search_distance + util.calculate_distance(node, neighbour)
                        if neighbour_search_distance <= max_search_distance:
                            queue.append([neighbour, neighbour_search_distance])

                return furthest_point

                    
            last_point = find_furthest_point(closest_cluster, first_point)
            furthest_point1 = extrapolate_points(first_point[0], first_point[1], last_point[0], last_point[1], 30)
            furthest_point2 = extrapolate_points(last_point[0], last_point[1], first_point[0], first_point[1], 30)
            furthest_point1_global = util.convert_to_global_coords(furthest_point1)
            furthest_point2_global = util.convert_to_global_coords(furthest_point2)
            self.publish_furthest_points(furthest_point1_global, furthest_point2_global)
            # Modify the goal selection block in get_goal_pose:
            angle1 = self.calculate_angle(furthest_point1_global, self.goals[-1], self.goals[-2])
            angle2 = self.calculate_angle(furthest_point2_global, self.goals[-1], self.goals[-2])
            furthest_point = furthest_point1 if angle1 > angle2 else furthest_point2
            goal_point = extrapolate_points(robot_coords_grid[0], robot_coords_grid[1], furthest_point[0], furthest_point[1], Config.config["extrapolate_distance"])
            # distance_lane2_bot = util.calculate_distance(robot_coords_grid, first_point2)
            # distance_goal_bot = util.calculate_distance(robot_coords_grid, first_point)
            goal_coords = np.array(util.convert_to_global_coords(goal_point))
            # if distance_goal_bot < 0.001:
            #     NodeGlobal.log_warn("Goal point too close to robot, using direct point")
            #     goal_coords = np.array(util.convert_to_global_coords(first_point))
            # else:
            #     ratio = Config.config["offset_goal_ratio"] * distance_lane2_bot / distance_goal_bot
            #     x = ((1 - ratio) * first_point[0] + (ratio) * robot_coords_grid[0])
            #     y = ((1 - ratio) * first_point[1] + (ratio) * robot_coords_grid[1])
            #     goal_coords = [x, y]
            #     goal_coords = np.array(util.convert_to_global_coords(goal_coords))
                
            return self.create_goal_pose(goal_coords)
            
        except Exception as e:
            NodeGlobal.log_error(f"Error in get_goal_pose: {str(e)}")
            return None

    def publish_goal(self, goal_pose):
        parsed_goal = Message.static_parse_message(goal_pose, PoseStamped)
        Publisher.pubs["goal_pose"].publish(goal_pose)
        if len(NodeGlobal.goals) <= 1 or util.calculate_distance(NodeGlobal.goals[-1], parsed_goal) > 2:
            NodeGlobal.goals.append(parsed_goal)
        NodeGlobal.log_info(f"Published goal: {goal_pose.pose.position.x, goal_pose.pose.position.y}")


def main(args=None):
    rclpy.init(args=args)
    NodeGlobal.goals = list()
    Message.add_parser(LaneChangeStatus, lambda message: message.data)

    lane_change_node = LaneChange()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(lane_change_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        lane_change_node.get_logger().info('Keyboard interrupt detected. Exiting...')
    finally:
        lane_change_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
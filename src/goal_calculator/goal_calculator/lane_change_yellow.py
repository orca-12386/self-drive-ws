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
import goal_calculator.util as util
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree
import statistics
from collections import defaultdict
from scipy.spatial.transform import Rotation as R

def get_yaw_from_odometry(odom_msg: Odometry) -> float:
    q = odom_msg.pose.pose.orientation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    _, _, yaw = r.as_euler('xyz', degrees=False)
    return yaw

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

        # Initialize message storage
        self.latest_map_msg = None
        self.latest_odom_msg = None
        self.odom_history = []
        self.max_odom_history = 20  # Store last 20 odometry messages

        # Create subscriptions with direct callbacks
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map/yellow/local/interp',
            self.map_callback,
            10
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Keep the existing subscription manager for compatibility
        subscription_info = {
            "map": ["map/yellow/local/interp", OccupancyGrid],
            "odom": ["odom", Odometry],
        }
        Subscription.create_subscriptions(subscription_info)

        publisher_info = {
            "goal_pose": ["goal_pose", PoseStamped],
            "lane_change_status": ["lane_change_status", LaneChangeStatus]
        }
        Publisher.create_publishers(publisher_info)

        self.lane_change_active = False
        self.publish_status(False)
        self.goals = list()
        self.timer = self.create_timer(0.1, self.publish_status_periodically)
        self.timer2 = self.create_timer(2, self.add_goal_pose)

    def map_callback(self, msg):
        """Handle new map messages"""
        self.latest_map_msg = msg
        # You could perform immediate analysis here if needed
        NodeGlobal.log_info("Received new map data")

    def odom_callback(self, msg):
        """Handle new odometry messages"""
        self.latest_odom_msg = msg
        self.odom_history.append(msg)
        
        # Keep history at a reasonable size
        if len(self.odom_history) > self.max_odom_history:
            self.odom_history = self.odom_history[-self.max_odom_history:]
            
        # You could trigger immediate actions based on new position if needed
        # For example, check if goal reached when in a lane change operation
        if self.lane_change_active:
            if hasattr(self, 'current_goal_pose') and self.current_goal_pose is not None:
                curr_coords = self.get_robot_coords_global()
                if curr_coords and self.goal_reached(curr_coords, self.current_goal_pose):
                    NodeGlobal.log_info("Goal reached based on new odometry data!")
                    # The actual success handling happens in the execute callback

    def get_robot_coords_global(self):
        if self.latest_odom_msg is None:
            return None
        robot_pose_global = self.latest_odom_msg.pose
        robot_coords_global = (robot_pose_global.pose.position.x, robot_pose_global.pose.position.y)
        return robot_coords_global
    
    def get_robot_coords_grid(self):
        if self.latest_map_msg is None:
            return None
        resolution = self.latest_map_msg.info.resolution
        robot_coords_global = self.get_robot_coords_global()
        if robot_coords_global is None:
            return None
        robot_coords_grid = (
            (robot_coords_global[0] - self.latest_map_msg.info.origin.position.x) / resolution, 
            (robot_coords_global[1] - self.latest_map_msg.info.origin.position.y) / resolution
        )
        return robot_coords_grid
    
    def add_goal_pose(self):
        robot_coords_global = self.get_robot_coords_global()
        if robot_coords_global is not None:
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
            if self.latest_map_msg is None or self.latest_odom_msg is None:
                self.get_logger().error("Cannot accept lane change - missing map or odometry data")
                return GoalResponse.REJECT
                
            self.get_logger().info("Lane change action received")
            self.lane_change_active = True
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def execute_callback_wrapper(self, goal_handle):
        return asyncio.run(self.async_execute_callback(goal_handle))

    async def async_execute_callback(self, goal_handle):
        NodeGlobal.log_info("Lane change initializing")

        initial_coords = self.get_robot_coords_grid()
        
        if initial_coords is None:
            NodeGlobal.log_error("Failed to get robot coordinates")
            goal_handle.abort()
            return LaneChangeAction.Result()
            
        try:
            goal_pose = self.get_goal_pose(initial_coords)
            
            if goal_pose is None:
                NodeGlobal.log_error("Failed to compute goal pose.")
                goal_handle.abort()
                return LaneChangeAction.Result()

            self.current_goal_pose = goal_pose
            self.publish_goal(goal_pose)

            while rclpy.ok():
                curr_coords = self.get_robot_coords_global()
                if curr_coords is None:
                    NodeGlobal.log_warn("No current coordinates available, waiting...")
                    await asyncio.sleep(0.1)
                    continue
                    
                if self.goal_reached(curr_coords, goal_pose):
                    NodeGlobal.log_info("Lane change complete.")
                    self.lane_change_active = False
                    self.publish_status(False)
                    self.current_goal_pose = None
                    goal_handle.succeed()
                    return LaneChangeAction.Result()
                await asyncio.sleep(0.1)
        except Exception as e:
            NodeGlobal.log_error(f"Error during lane change execution: {str(e)}")
            self.lane_change_active = False
            self.publish_status(False)
            self.current_goal_pose = None
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
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        
        # Use the stored odometry history for orientation calculation
        if len(self.odom_history) > 5:
            recent_odom = self.odom_history[:-5]  # Skip the most recent 5 as they might be noisy
            robot_orientation_data = [get_yaw_from_odometry(x) for x in recent_odom]
            if robot_orientation_data:
                avg_orientation = statistics.fmean(robot_orientation_data)
            else:
                avg_orientation = 0.0
                NodeGlobal.log_warn("No orientation data in history, using default 0.0")
        else:
            avg_orientation = 0.0
            NodeGlobal.log_warn("Not enough orientation data in history, using default 0.0")
            
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(avg_orientation / 2)
        goal_pose.pose.orientation.w = math.cos(avg_orientation / 2)

        return goal_pose

    def get_goal_pose(self, robot_coords_grid):
        def get_clusters(binary_array, eps=Config.config["dbscan_eps"], min_samples=Config.config["dbscan_min_samples"]):
            points = np.argwhere(binary_array > 0)
            NodeGlobal.log_info(str(points))
            points = points[:, [1, 0]]
            if len(points) < min_samples:
                return [np.array([]), np.array([])]
            dbscan = DBSCAN(eps=eps, min_samples=min_samples)
            clusters = dbscan.fit_predict(points)
            unique_labels = np.unique(clusters)
            unique_labels = unique_labels[unique_labels != -1]
            if len(unique_labels) == 0:
                return [np.array([]), np.array([])]
            cluster_sizes = [(label, np.sum(clusters == label)) for label in unique_labels]
            cluster_sizes.sort(key=lambda x: x[1], reverse=True)
            result = []
            for i in range(2):
                if i < len(cluster_sizes):
                    label = cluster_sizes[i][0]
                    cluster_points = points[clusters == label]
                    result.append(cluster_points)
                else:
                    result.append(np.array([]))
            return result

        try:
            if self.latest_map_msg is None:
                NodeGlobal.log_error("No map data available")
                return None
                
            # Convert map message to data array for clustering
            map_data = np.array(self.latest_map_msg.data).reshape(
                self.latest_map_msg.info.height, 
                self.latest_map_msg.info.width
            )
                
            clusters = get_clusters(map_data)
            NodeGlobal.log_info("Cluster shapes: " + str([cluster.shape for cluster in clusters]))

            # Ensure we have at least two valid clusters
            if all(cluster.shape[0] < Config.config["minimum_points"] for cluster in clusters):
                NodeGlobal.log_error("Not enough points in cluster detected")
                return None

            cluster = clusters[0]
            tree = cKDTree(cluster)
            _, nearest_index = tree.query(robot_coords_grid, k=1)
            first_point = cluster[nearest_index]
            
            def find_furthest_point(cluster, start, max_search_distance=Config.config["max_search_distance"]):
                if len(self.goals) < 2:
                    NodeGlobal.log_error("Not enough goal history to find furthest point, using nearest point in target lane")
                    return start
                    
                visited = set()
                previous_bot_pose = self.goals[-2]
                NodeGlobal.log_info("previous bot pose: " + str(previous_bot_pose))
                start_global = np.array(util.convert_to_global_coords(start))
                max_distance_global = util.calculate_distance(previous_bot_pose, start_global)
                furthest_point = start
                queue = []
                queue.append([start, max_distance_global, 0])  # Add initial distance from start
                tree = cKDTree(cluster)
                
                while queue:
                    node, distance, current_search_distance = queue.pop(0)
                    node_tuple = tuple(node)
                    
                    # Stop searching if we've exceeded the maximum search distance
                    if current_search_distance > max_search_distance:
                        break
                    
                    if node_tuple in visited:
                        continue  # Skip already visited nodes
                    
                    visited.add(node_tuple)
                    
                    if distance >= max_distance_global:
                        max_distance_global = distance
                        furthest_point = node
                        
                    indices = tree.query_ball_point(node, 3)
                    for i in indices:
                        neighbour = cluster[i]
                        if tuple(neighbour) in visited:
                            continue
                            
                        neighbour_global = np.array(util.convert_to_global_coords(neighbour))
                        step_distance = util.calculate_distance(previous_bot_pose, neighbour_global)
                        
                        # Calculate the search distance from the start point
                        neighbour_search_distance = current_search_distance + util.calculate_distance(node, neighbour)
                        
                        if step_distance >= max_distance_global and neighbour_search_distance <= max_search_distance:
                            queue.append([neighbour, step_distance, neighbour_search_distance])
                        
                return furthest_point
                    
            first_point = find_furthest_point(cluster, first_point)
            goal_point = extrapolate_points(robot_coords_grid[0], robot_coords_grid[1], first_point[0], first_point[1], Config.config["extrapolate_distance"])
            goal_coords = np.array(util.convert_to_global_coords(goal_point))
                
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
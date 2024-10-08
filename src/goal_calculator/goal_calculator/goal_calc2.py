#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
import numpy as np
import math
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
import os
from ament_index_python.packages import get_package_share_directory
import inspect
import yaml
from scipy.spatial import cKDTree
import statistics
import time
from collections import defaultdict

# convention: all coordinates are grid unless specified global in variable

def read_config(filepath):
    def load_yaml_file(file_path):
        try:
            with open(file_path, 'r') as yaml_file:
                return yaml.safe_load(yaml_file)
        except Exception as e:
            GoalCalculator.get_logger().error(f"Failed to load YAML file: {e}")
            return None
    # yaml_file_path = os.path.join(get_package_share_directory('goal_calculator'), 'config','config.yaml')
    yaml_file_path = filepath
    with open(yaml_file_path):
        return load_yaml_file(yaml_file_path)


def calculate_angle(candidate_goal, previous_goal, second_previous_goal):
    # Convert goals to numpy arrays for easier vector operations
    v1 = np.array(candidate_goal) - np.array(previous_goal)
    v2 = np.array(second_previous_goal) - np.array(previous_goal)
    # Check if any of the vectors have zero length
    if np.allclose(v1, 0) or np.allclose(v2, 0):
        return 0.0  # Return 0 degrees if any vector has zero length
    # Calculate the angle between the two vectors using arctan2
    angle = np.arctan2(np.linalg.norm(np.cross(v1, v2)), np.dot(v1, v2))
    # Convert angle from radians to degrees
    angle_degrees = np.degrees(angle)
    return angle_degrees

def calculate_distance(point1, point2):
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

def convert_to_global_coords( coord):
    map_message = GoalCalculator.subscribed["map"].get_latest_message()
    origin = np.array([map_message.info.origin.position.x, map_message.info.origin.position.y])
    newcoord = origin + np.array([x*map_message.info.resolution for x in coord])
    return newcoord

def convert_to_grid_coords(coord):
    map_message = GoalCalculator.subscribed["map"].get_latest_message()
    origin = np.array([map_message.info.origin.position.x, map_message.info.origin.position.y])
    newcoord = [coord[0]-origin[0], coord[1]-origin[1]]
    newcoord = np.array([x/map_message.info.resolution for x in newcoord])
    return newcoord


def parse_message(message, datatype):
    if datatype == PoseStamped:
        return (message.pose.position.x, message.pose.position.y)
    elif datatype == OccupancyGrid:
        return np.array(message.data).reshape((message.info.height, message.info.width))
    elif datatype == Float64:
        return message.data
    else:
        print("Message type cannot be parsed")


class Subscription:
    def __init__(self, topic, datatype, obj, maxlength):
        self.topic = topic
        self.datatype = datatype
        self.obj = obj
        self.maxlength = maxlength
        self.messages = list()

    def set_message(self, msg):
        self.messages.append(msg)
        if len(self.messages)>self.maxlength:
            self.messages.pop(0)
        
    def get_all_messages(self):
        if(len(self.messages)) > 0:
            return self.messages
        else:
            return None
        
    def get_latest_message(self):
        if len(self.messages) > 0:
            return self.messages[-1]
        else:
            return None

    def parse_message(self, message, datatype = None):
        if datatype == None:
            datatype = self.datatype
        parsed_message = parse_message(message, datatype)
        # if self.topic == "map" and type(parsed_message) is np.ndarray:
        #     resolution = self.get_latest_message().info.resolution
        #     data = GoalCalculator.subscribed["robot_pose_grid"].get_latest_data()
        #     local_map_dim = (GoalCalculator.config["local_map_dim"]/resolution)
        #     diff = int(local_map_dim/2)
        #     if not (data is None):
        #         data = [int(d) for d in data]
        #         if len(parsed_message)>local_map_dim:
        #             if len(parsed_message[0])>local_map_dim:
        #                 starty = data[1]-diff
        #                 startx = data[0]-diff
        #                 endy = data[1]+diff
        #                 endx = data[0]+diff
        #                 if starty<0:
        #                     starty = 0
        #                 if startx<0:
        #                     startx = 0
        #                 parsed_message = parsed_message[starty:endy,startx:endx]
        return parsed_message

    def get_all_data(self):
        if len(self.messages) > 0:
            return [self.parse_message(message) for message in self.get_all_messages()]
        else:
            return None

    def get_latest_data(self):
        if len(self.messages) > 0:
            return self.parse_message(self.get_latest_message())
        else:
            return None

class CandidateGoal:

    def calculate_goal_angle(self):
        if len(GoalCalculator.goals)>=2:
            return calculate_angle(self.coords, GoalCalculator.goals[-1], GoalCalculator.goals[-2])
        else:
            return None

    def calculate_parent_distance(self):
        return calculate_distance(self.parent1, self.parent2)

    def calculate_robot_distance(self):
        robot_coords_global = GoalCalculator.subscribed["robot_pose_global"].get_latest_data()
        return calculate_distance(self.coords, robot_coords_global)

    def validate(self):
        condition = True
        condition = condition and self.goal_angle > GoalCalculator.config["goal_angle_threshold"]
        condition = condition and self.robot_distance < GoalCalculator.config["goal_distance_max"]
        condition = condition and self.robot_distance > GoalCalculator.config["goal_distance_min"]
        return condition
        # goal angle threshold
        # distance threshold

    def calculate_heuristic(self):
        if self.validate() == False:
            return False
        return self.goal_angle * self.robot_distance / self.parent_distance
        # farther points are preferred
        # higher angle better 

    def __init__(self, parent1, parent2):
        self.coords = (parent1+parent2)/2 # tuple (x,y)
        self.parent1 = parent1
        self.parent2 = parent2
        self.goal_angle = self.calculate_goal_angle()
        self.parent_distance = self.calculate_parent_distance()
        self.robot_distance = self.calculate_robot_distance()
        self.heuristic = self.calculate_heuristic()


# class NearestPairsIterator:
#     def __init__(self, points1, points2, midpoint_tolerance=1e-6):
#         self.points1 = points1
#         self.points2 = points2
#         self.midpoint_tolerance = midpoint_tolerance
        
#         # Build KD-Tree for efficient nearest neighbor search
#         self.tree2 = cKDTree(points2)
        
#         # Initialize state
#         self.current_idx = 0
#         self.used_midpoints = set()
        
#     def __iter__(self):
#         return self
        
#     def __next__(self):
#         while self.current_idx < len(self.points1):
#             point1 = self.points1[self.current_idx]
            
#             # Find nearest point in points2
#             distance, idx = self.tree2.query(point1, k=1)
#             point2 = self.points2[idx]
            
#             # Calculate midpoint
#             midpoint = tuple((point1 + point2) / 2)
            
#             # Move to next point
#             self.current_idx += 1
            
#             # Check if this midpoint is unique (within tolerance)
#             is_unique = True
#             for used_midpoint in self.used_midpoints:
#                 if np.allclose(midpoint, used_midpoint, rtol=0, atol=self.midpoint_tolerance):
#                     is_unique = False
#                     break
            
#             if is_unique:
#                 self.used_midpoints.add(midpoint)
#                 return (point1, point2)
                
#         raise StopIteration

class NearestPairsIterator:
    """
    Optimized iterator class for finding unique nearest pairs between two sets of points,
    filtering out pairs with similar midpoints.
    """
    def __init__(self, points1, points2, midpoint_tolerance=1e-6):
        """
        Initialize the iterator with two arrays of points.
        
        Parameters:
        points1 (np.ndarray): First array of points, shape (N, 2)
        points2 (np.ndarray): Second array of points, shape (M, 2)
        midpoint_tolerance (float): Tolerance for considering midpoints as different
        """
        self.points1 = points1
        self.points2 = points2
        self.tolerance = midpoint_tolerance
        
        # Pre-compute all nearest neighbors at once for better performance
        self.tree2 = cKDTree(points2)
        self.distances, self.indices = self.tree2.query(points1, k=1)
        
        # Initialize state
        self.current_idx = 0
        
        # Use spatial hashing for faster midpoint comparison
        self.cell_size = midpoint_tolerance * 2
        self.used_cells = defaultdict(set)
        
    def _get_cell_coords(self, point):
        """Convert a point to discrete cell coordinates for spatial hashing."""
        return tuple(np.floor(point / self.cell_size).astype(int))
    
    def _is_midpoint_unique(self, midpoint):
        """Check if a midpoint is unique using spatial hashing."""
        cell = self._get_cell_coords(midpoint)
        
        # Check neighboring cells for nearby midpoints
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor_cell = (cell[0] + dx, cell[1] + dy)
                for used_midpoint in self.used_cells[neighbor_cell]:
                    if np.allclose(midpoint, used_midpoint, rtol=0, atol=self.tolerance):
                        return False
        
        # If unique, add to the appropriate cell
        self.used_cells[cell].add(tuple(midpoint))
        return True
        
    def __iter__(self):
        return self
        
    def __next__(self):
        while self.current_idx < len(self.points1):
            point1 = self.points1[self.current_idx]
            point2 = self.points2[self.indices[self.current_idx]]
            
            # Calculate midpoint
            midpoint = (point1 + point2) / 2
            
            # Move to next point
            self.current_idx += 1
            
            # Check if this midpoint is unique using spatial hashing
            if self._is_midpoint_unique(midpoint):
                return (point1, point2)
                
        raise StopIteration


class GoalCalculator(Node):
    def __init__(self):
        super().__init__("goal_calc_node")
        
        self.get_logger().info("Initialising node")

        self.declare_parameter('config_file_path', 'default_value')
        GoalCalculator.config = read_config(self.get_parameter('config_file_path').value)
        self.get_logger().info("Config read: "+self.get_parameter('config_file_path').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        subscription_info = {
            "map": ["local_map", OccupancyGrid],
            "odom": ["odom", Odometry],
            "robot_pose_global": ["robot_pose_global", PoseStamped],
            "robot_pose_grid": ["robot_pose_grid", PoseStamped],
            "robot_orientation": ["robot_orientation", Float64]
        }
        publisher_info = {
            "goal_pose": ["goal_pose", PoseStamped]
        }
        
        create_subscriber = lambda topic: self.create_subscription(subscription_info[topic][1], subscription_info[topic][0], lambda msg: self.topic_callback(topic, msg), 10)
        create_publisher = lambda topic: self.create_publisher(publisher_info[topic][1], publisher_info[topic][0], 10)
        GoalCalculator.subscribed = dict()
        GoalCalculator.publishing = dict()
        
        for topic in subscription_info:
            if topic == "robot_orientation":
                length = GoalCalculator.config["bot_orientation_messages_maxlength"]
            else:
                length = 1
            GoalCalculator.subscribed[topic] = Subscription(subscription_info[topic][0], subscription_info[topic][1], create_subscriber(topic), length)
            self.get_logger().info("Subscribed to "+topic)
        self.get_logger().info("Subscriptions created")

        for topic in publisher_info:
            GoalCalculator.publishing[topic] = create_publisher(topic)
            self.get_logger().info("Created publisher to "+topic)
        self.get_logger().info("Publishers created")


        self.timer = self.create_timer(GoalCalculator.config["controller_interval"], self.controller)
        self.get_logger().info("Timer created")

        GoalCalculator.goals = list()
        
        self.start = True
        self.shutdown = False

        self.get_logger().info("Node initialized")


    def topic_callback(self, topic, msg):
        GoalCalculator.subscribed[topic].set_message(msg) 

    def controller(self):
        self.get_logger().info("Controller is active")
        robot_coords_grid = GoalCalculator.subscribed["robot_pose_grid"].get_latest_data()
        robot_coords_global = GoalCalculator.subscribed["robot_pose_global"].get_latest_data()
        map_data = GoalCalculator.subscribed["map"].get_latest_data()
        if not (map_data is None):
            self.get_logger().info("Local map shape: "+str(map_data.shape))
        if robot_coords_grid == None or map_data is None:
            self.get_logger().info("Not enough information to search and publish yet.")
            return
        if self.start == True:
            self.get_logger().info("Publishing start goal")
            GoalCalculator.goals.append(np.array(robot_coords_global))
            self.publish_goal(self.create_goal_pose(np.array([robot_coords_global[0]+GoalCalculator.config["start_goal_distance"], robot_coords_global[1]])))
            self.start = False
            return
        self.check_shutdown()
        if self.shutdown == True:
            self.get_logger().info("Goal calculator is shutdown")
            return  
        self.get_logger().info("Calculating goal pose")
        start = time.time()      
        goal_pose = self.get_goal_pose()
        end = time.time()
        if goal_pose != False:
            self.get_logger().info(f"Goal pose calculated (Time taken:{end-start})")
            self.publish_goal(goal_pose)
        else:
            self.get_logger().info(f"No valid goal pose found (Time taken: {end-start})")

    def check_shutdown(self):
        shutdown_coords_global = GoalCalculator.config["shutdown_coords_global"]
        resume_coords_global = GoalCalculator.config["resume_coords_global"]
        shutdown_tolerance_distance = GoalCalculator.config["shutdown_tolerance_distance"]
        resume_tolerance_distance = GoalCalculator.config["resume_tolerance_distance"]
        robot_coords_global = GoalCalculator.subscribed["robot_pose_global"].get_latest_data()
        
        if self.shutdown == False and calculate_distance(robot_coords_global, shutdown_coords_global) < shutdown_tolerance_distance:
            self.shutdown = True
            self.get_logger().info("Goal calculator shutdown")
    
        if self.shutdown == True and calculate_distance(robot_coords_global, resume_coords_global) < resume_tolerance_distance:
            self.shutdown = False
            self.get_logger().info("Publishing resume goal")
            GoalCalculator.goals.clear()
            GoalCalculator.goals.append(resume_coords_global)
            self.publish_goal(self.create_goal_pose(np.array([resume_coords_global[0]-GoalCalculator.config["resume_goal_distance"], resume_coords_global[1]])))

    def create_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        
        # Use the average orientation
        robot_orientation_data = GoalCalculator.subscribed["robot_orientation"].get_all_data()
        # print(robot_orientation_data)
        avg_orientation = statistics.fmean(robot_orientation_data)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(avg_orientation / 2)
        goal_pose.pose.orientation.w = math.cos(avg_orientation / 2)
        return goal_pose

    def get_goal_pose(self):
        """
        DBScan and get 2 largest groups of points
        Create list of pairs of points with distance between them
        Create list of midpoints
        Filter midpoints
        Pick midpoint with best heuristic
        """
        def get_top2_clusters(binary_array, eps=GoalCalculator.config["dbscan_eps"], min_samples=GoalCalculator.config["dbscan_min_samples"]):
            points = np.argwhere(binary_array > 0)
            points = points[:, [1, 0]]  # Swap columns to get (x, y) format
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

        map_data = GoalCalculator.subscribed["map"].get_latest_data()
        clusters = get_top2_clusters(map_data)
        self.get_logger().info("Cluster shapes: "+str([cluster.shape for cluster in clusters]))
        for cluster in clusters:
            if cluster.shape[0] < GoalCalculator.config["minimum_points"]:
                return False
            
        for i, cluster in enumerate(clusters):
            clusters[i] = np.array([convert_to_global_coords(point) for point in cluster])

        iterator = NearestPairsIterator(clusters[0], clusters[1], midpoint_tolerance=GoalCalculator.config["midpoint_tolerance"])

        candidate_goals = list()
        max_h_i = 0
        max_h = None
        for parent1, parent2 in iterator:
            candidate_goal = CandidateGoal(parent1, parent2)
            if candidate_goal.heuristic != False:
                candidate_goals.append(candidate_goal)
                if max_h == None or candidate_goal.heuristic > candidate_goals[max_h_i].heuristic:
                    max_h_i = len(candidate_goals) - 1
                    max_h = candidate_goal.heuristic

        if len(candidate_goals) == 0:
            return False
        
        goal = candidate_goals[max_h_i].coords
        goal_pose = self.create_goal_pose(goal)
        return goal_pose

    def publish_goal(self, goal_pose):
        parsed_goal = parse_message(goal_pose, PoseStamped)
        GoalCalculator.publishing["goal_pose"].publish(goal_pose)
        if len(GoalCalculator.goals)<=1 or calculate_distance(GoalCalculator.goals[-1], parsed_goal)>GoalCalculator.config["goal_logger_in_between_distance"]: 
            GoalCalculator.goals.append(parsed_goal)
        self.get_logger().info(f"Published goal: {goal_pose}")


def main(args = None):
    rclpy.init(args = args)
    goal_calculator = GoalCalculator()
    rclpy.spin(goal_calculator)
    goal_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np
import math
from sklearn.cluster import DBSCAN
import yaml
from scipy.spatial import cKDTree
import statistics
import time
from collections import defaultdict
from std_msgs.msg import Bool as LaneKeepDisable
from goal_calculator.ros2_wrapper import Subscription, Message, Publisher, Config, NodeGlobal


# convention: all coordinates are grid unless specified global in variable

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
    map_message = Subscription.subs["map"].get_latest_message()
    origin = np.array([map_message.info.origin.position.x, map_message.info.origin.position.y])
    newcoord = origin + np.array([x*map_message.info.resolution for x in coord])
    return newcoord

def convert_to_grid_coords(coord):
    map_message = Subscription.subs["map"].get_latest_message()
    origin = np.array([map_message.info.origin.position.x, map_message.info.origin.position.y])
    newcoord = [coord[0]-origin[0], coord[1]-origin[1]]
    newcoord = np.array([x/map_message.info.resolution for x in newcoord])
    return newcoord
        

class CandidateGoal:

    def calculate_goal_angle(self):
        if len(NodeGlobal.goals)>=2:
            return calculate_angle(self.coords, NodeGlobal.goals[-1], NodeGlobal.goals[-2])
        else:
            return None

    def calculate_parent_distance(self):
        return calculate_distance(self.parent1, self.parent2)

    def calculate_robot_distance(self):
        robot_coords_global = Subscription.subs["robot_pose_global"].get_latest_data()
        return calculate_distance(self.coords, robot_coords_global)

    def check_obstacle(self):
        map_data = Subscription.subs["map"].get_latest_data()
        goal_coords_grid = [int(d) for d in convert_to_grid_coords(self.coords)]
        if map_data[goal_coords_grid[1]][goal_coords_grid[0]] > 0:
            return True
        else:
            return False

    def validate(self):
        condition = True
        condition = condition and self.goal_angle > Config.config["goal_angle_threshold"]
        condition = condition and self.robot_distance < Config.config["goal_distance_max"]
        condition = condition and self.robot_distance > Config.config["goal_distance_min"]
        condition = condition and self.check_obstacle() == False
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
        
        NodeGlobal.obj = self

        NodeGlobal.log_info("Initialising node")

        self.declare_parameter('config_file_path', 'default_value')
        Config.read_config(self.get_parameter('config_file_path').value)
        NodeGlobal.log_info("Config read: "+self.get_parameter('config_file_path').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        subscription_info = {
            "map": ["local_map", OccupancyGrid],
            "odom": ["odom", Odometry],
            "robot_pose_global": ["robot_pose_global", PoseStamped],
            "robot_pose_grid": ["robot_pose_grid", PoseStamped],
            "robot_orientation": ["robot_orientation", Float64, 10],
            "lane_keep_disable": ["lane_keep_disable", LaneKeepDisable]
        }

        Subscription.create_subscriptions(subscription_info)
        NodeGlobal.log_info("Subscriptions created")

        publisher_info = {
            "goal_pose": ["goal_pose", PoseStamped]
        }
        Publisher.create_publishers(publisher_info)
        NodeGlobal.log_info("Publishers created")

        self.timer = self.create_timer(Config.config["controller_interval"], self.controller)
        NodeGlobal.log_info("Timer created")

        NodeGlobal.goals = list()
        
        self.start = True
        self.shutdown = False

        NodeGlobal.log_info("Node initialized")


    def controller(self):
        lane_keep_disable = Subscription.subs["lane_keep_disable"].get_latest_data()
        if lane_keep_disable == False:
            NodeGlobal.log_info("Controller is active")
            robot_coords_grid = Subscription.subs["robot_pose_grid"].get_latest_data()
            robot_coords_global = Subscription.subs["robot_pose_global"].get_latest_data()
            map_data = Subscription.subs["map"].get_latest_data()
            if not (map_data is None):
                NodeGlobal.log_info("Local map shape: "+str(map_data.shape))
            if robot_coords_grid == None or map_data is None:
                NodeGlobal.log_info("Not enough information to search and publish yet.")
                return
            if self.start == True:
                NodeGlobal.log_info("Publishing start goal")
                NodeGlobal.goals.append(np.array(robot_coords_global))
                self.publish_goal(self.create_goal_pose(np.array([robot_coords_global[0]+Config.config["start_goal_distance"], robot_coords_global[1]])))
                self.start = False
                return
            # self.check_shutdown()
            # if self.shutdown == True:
            #     NodeGlobal.log_info("Goal calculator is shutdown")
            #     return  
            NodeGlobal.log_info("Calculating goal pose")
            start = time.time()      
            goal_pose = self.get_goal_pose(robot_coords_grid)
            end = time.time()
            if goal_pose != False:
                NodeGlobal.log_info(f"Goal pose calculated (Time taken:{end-start})")
                self.publish_goal(goal_pose)
            else:
                NodeGlobal.log_info(f"No valid goal pose found (Time taken: {end-start})")

    # def check_shutdown(self):
    #     shutdown_coords_global = Config.config["shutdown_coords_global"]
    #     resume_coords_global = Config.config["resume_coords_global"]
    #     shutdown_tolerance_distance = Config.config["shutdown_tolerance_distance"]
    #     resume_tolerance_distance = Config.config["resume_tolerance_distance"]
    #     robot_coords_global = Subscription.subs["robot_pose_global"].get_latest_data()
        
    #     if self.shutdown == False and calculate_distance(robot_coords_global, shutdown_coords_global) < shutdown_tolerance_distance:
    #         self.shutdown = True
    #         NodeGlobal.log_info("Goal calculator shutdown")
    
    #     if self.shutdown == True and calculate_distance(robot_coords_global, resume_coords_global) < resume_tolerance_distance:
    #         self.shutdown = False
    #         NodeGlobal.log_info("Publishing resume goal")
    #         NodeGlobal.goals.clear()
    #         NodeGlobal.goals.append(resume_coords_global)
    #         self.publish_goal(self.create_goal_pose(np.array([resume_coords_global[0]-Config.config["resume_goal_distance"], resume_coords_global[1]])))

    def create_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        
        # Use the average orientation
        robot_orientation_data = Subscription.subs["robot_orientation"].get_all_data()
        # NodeGlobal.log_info(robot_orientation_data)
        avg_orientation = statistics.fmean(robot_orientation_data)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(avg_orientation / 2)
        goal_pose.pose.orientation.w = math.cos(avg_orientation / 2)
        return goal_pose

    def get_goal_pose(self, robot_coords_grid):
        """
        DBScan and get 2 largest groups of points
        Create list of pairs of points with distance between them
        Create list of midpoints
        Filter midpoints
        Pick midpoint with best heuristic
        """
        def get_top2_clusters(binary_array, eps=Config.config["dbscan_eps"], min_samples=Config.config["dbscan_min_samples"]):
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

        map_data = Subscription.subs["map"].get_latest_data()
        clusters_with_distance = []
        
        clusters = get_top2_clusters(map_data)
        NodeGlobal.log_info("Cluster shapes: "+str([cluster.shape for cluster in clusters]))
        for cluster in clusters:
            if cluster.shape[0] < 200:
                return False
            tree = cKDTree(cluster)
            distance, _ = tree.query(robot_coords_grid, k=1)
            clusters_with_distance.append((distance, cluster))
        clusters_with_distance.sort(key=lambda x: x[0])
        closest_cluster_distance, clusters[0] = clusters_with_distance[0]
        closest_cluster_distance, clusters[1] = clusters_with_distance[1]
        for i, cluster in enumerate(clusters):
            clusters[i] = np.array([convert_to_global_coords(point) for point in cluster])

        iterator = NearestPairsIterator(clusters[0], clusters[1], midpoint_tolerance=Config.config["midpoint_tolerance"])

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
        parsed_goal = Message.static_parse_message(goal_pose, PoseStamped)
        Publisher.pubs["goal_pose"].publish(goal_pose)
        if len(NodeGlobal.goals)<=1 or calculate_distance(NodeGlobal.goals[-1], parsed_goal)>Config.config["goal_logger_in_between_distance"]: 
            NodeGlobal.goals.append(parsed_goal)
        NodeGlobal.log_info(f"Published goal: {goal_pose}")


def main(args = None):
    rclpy.init(args = args)
    NodeGlobal.goals = list()
    Message.add_parser(LaneKeepDisable, lambda message: message.data)
    goal_calculator = GoalCalculator()
    rclpy.spin(goal_calculator)
    goal_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
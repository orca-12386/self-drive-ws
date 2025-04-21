#!/usr/bin/env python3

import scipy.spatial
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
from goal_calculator.ros2_wrapper import Subscription, Message, Publisher, Config, NodeGlobal
from interfaces.srv import LaneFollowToggle
import scipy
from collections import deque


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


def get_point_at_distance(point, quaternion, distance):
    r = scipy.spatial.transform.Rotation.from_quat([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])    
    forward_vector = r.apply([1, 0, 0])    
    forward_vector = forward_vector / np.linalg.norm(forward_vector)    
    new_point = point + forward_vector * distance    
    return new_point

def get_nearest_point_bfs(src, grid):
    """
    Find nearest point with occupancy > 0 using BFS
    
    Args:
        src: Source point (x, y)
        grid: OccupancyGrid with data and info
    
    Returns:
        Tuple of (found, destination_point)
    """
    visited_vec = []
    visited = set()
    
    def hash_coords(coords):
        return (coords[0] << 32) | coords[1]
    
    q = deque()
    q.append(src)
    
    while q:
        p = q.popleft()
        
        # Check if current point has occupancy > 0
        if grid.data[p[1] * grid.info.width + p[0]] > 0:
            return True, p
        
        # Check if distance from source exceeds threshold
        if math.sqrt((p[0] - src[0])**2 + (p[1] - src[1])**2) > 60:
            return False, None
        
        # Generate neighbors
        neighbours = [
            (p[0] + 1, p[1]),  # right
            (p[0], p[1] + 1),  # down
            (p[0] - 1, p[1]),  # left
            (p[0], p[1] - 1)   # up
        ]
        
        visited_vec.append(p)
        
        for neighbour in neighbours:
            # Check bounds
            if neighbour[0] >= grid.info.width or neighbour[0] < 0:
                continue
            if neighbour[1] >= grid.info.height or neighbour[1] < 0:
                continue
                
            # Add to queue if not visited
            neighbour_hash = hash_coords(neighbour)
            if neighbour_hash not in visited:
                q.append(neighbour)
                visited.add(neighbour_hash)
    
    return False, None


def get_connected_points_bfs(src, grid):
    """
    Find all connected points with occupancy > 0 using BFS
    
    Args:
        src: Source point (x, y)
        grid: OccupancyGrid with data and info
    
    Returns:
        List of connected points
    """
    visited = set()
    q = deque()
    
    def hash_coords(x, y):
        return (x << 32) | y
    
    width = grid.info.width
    height = grid.info.height
    
    src_hash = hash_coords(src[0], src[1])
    visited.add(src_hash)
    q.append(src)
    
    skip_dist = 7
    
    # Direction arrays
    cdx = [1, 0, -1, 0, 1, -1, 1, -1]
    cdy = [0, 1, 0, -1, 1, -1, -1, 1]
    
    # Generate skipped positions (same as C++ logic)
    dx = []
    dy = []
    for i in range(skip_dist * 8):
        c = (i // 8) + 1
        dx.append(c * cdx[i % 8])
        dy.append(c * cdy[i % 8])
    
    connected = []
    
    while q:
        p = q.popleft()
        connected.append(p)
        
        # Check 8 directions with skip_dist
        for i in range(8):
            nx = p[0] + dx[i]
            ny = p[1] + dy[i]
            
            # Check bounds
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            
            # Check if cell has value > 0
            if grid.data[ny * width + nx] <= 0:
                continue
            
            # Check if already visited
            hash_val = hash_coords(nx, ny)
            if hash_val in visited:
                continue
            
            visited.add(hash_val)
            q.append((nx, ny))
        
        # Early stopping condition (commented out in original)
        # if len(connected) > 10000:
        #     break
    
    return connected

        

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

        self.running = False

        NodeGlobal.log_info("Initialising node")

        self.declare_parameter('config_file_path', 'default_value')
        Config.read_config(self.get_parameter('config_file_path').value)
        NodeGlobal.log_info("Config read: "+self.get_parameter('config_file_path').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        subscription_info = {
            "map": ["/map/current", OccupancyGrid],
            "map_y": ["/map/yellow/local/interp", OccupancyGrid],
            "map_w": ["/map/white/local/near", OccupancyGrid],
            "odom": ["odom", Odometry],
            "robot_pose_global": ["/map/robot_pose_global", PoseStamped],
            "robot_pose_grid": ["/map/robot_pose_grid", PoseStamped],
            "robot_orientation": ["/map/robot_orientation", Float64, 10],
            # "lane_follow_disable": ["lane_follow_disable", LaneFollowDisable]
        }

        Subscription.create_subscriptions(subscription_info)
        NodeGlobal.log_info("Subscriptions created")

        publisher_info = {
            "goal_pose": ["goal_pose", PoseStamped]
        }
        Publisher.create_publishers(publisher_info)
        NodeGlobal.log_info("Publishers created")

        # self.timer = self.create_timer(Config.config["controller_interval"], self.controller)
        # NodeGlobal.log_info("Timer created")

        NodeGlobal.goals = list()
        
        self.start = True
        self.shutdown = False

        self.toggle_lane_follow_srv = self.create_service(LaneFollowToggle, 'toggle_lane_follow', self.LaneFollowToggleCallback)

        NodeGlobal.log_info("Node initialized")

    def LaneFollowToggleCallback(self, request, response):
        if request.toggle:
            if not self.running:
                self.start = True
                NodeGlobal.goals.clear()
                NodeGlobal.log_info("Creating timer")
                self.timer = self.create_timer(Config.config["controller_interval"], self.controller)
                NodeGlobal.log_info("Timer created")
            self.running = True
        else:
            if self.running:
                self.timer.cancel()
                del self.timer
                NodeGlobal.log_info("Destroyed timer")
            self.running = False
        response.success = True
        return response

    def controller(self):
        if self.running == True:
            NodeGlobal.log_info("Controller is active")
            robot_coords_grid = Subscription.subs["robot_pose_grid"].get_latest_data()
            robot_coords_global = Subscription.subs["robot_pose_global"].get_latest_data()
            map_data = Subscription.subs["map"].get_latest_data()
            if not (map_data is None):
                NodeGlobal.log_info("Local map shape: "+str(map_data.shape))
            if robot_coords_grid == None or map_data is None:
                NodeGlobal.log_info("Waiting for subscriptions")
                return
            if self.start == True:
                NodeGlobal.log_info("Publishing start goal")
                NodeGlobal.goals.append(np.array(robot_coords_global))
                
                odometry_msg = Subscription.subs["odom"].get_latest_data()
                pos = np.array([
                    odometry_msg.pose.pose.position.x,
                    odometry_msg.pose.pose.position.y,
                    odometry_msg.pose.pose.position.z
                ])
                quat = np.array([
                    odometry_msg.pose.pose.orientation.x,
                    odometry_msg.pose.pose.orientation.y,
                    odometry_msg.pose.pose.orientation.z,
                    odometry_msg.pose.pose.orientation.w
                ])
                first_goal = get_point_at_distance(pos, quat, Config.config["start_goal_distance"])
                self.publish_goal(self.create_goal_pose(first_goal))
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
            NodeGlobal.log_info(str(NodeGlobal.goals))
            if goal_pose != False:
                NodeGlobal.log_info(f"Goal pose calculated (Time taken:{end-start}s)")
                self.publish_goal(goal_pose)
            else:
                NodeGlobal.log_info(f"No valid goal pose found (Time taken: {end-start}s)")

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
        # def get_top1_cluster(binary_array, eps=Config.config["dbscan_eps"], min_samples=Config.config["dbscan_min_samples"]):
        #     points = np.argwhere(binary_array > 0)
        #     points = points[:, [1, 0]]  # Swap columns to get (x, y) format
        #     if len(points) < min_samples:
        #         return [np.array([]), np.array([])]
        #     dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        #     clusters = dbscan.fit_predict(points)
        #     unique_labels = np.unique(clusters)
        #     unique_labels = unique_labels[unique_labels != -1]
        #     if len(unique_labels) == 0:
        #         return [np.array([]), np.array([])]
        #     cluster_sizes = [(label, np.sum(clusters == label)) for label in unique_labels]
        #     cluster_sizes.sort(key=lambda x: x[1], reverse=True)
        #     result = []
        #     for i in range(2):
        #         if i < len(cluster_sizes):
        #             label = cluster_sizes[i][0]
        #             cluster_points = points[clusters == label]
        #             result.append(cluster_points)
        #         else:
        #             result.append(np.array([]))
        #     return result

        def get_cluster(map_name):
            map_data = Subscription.subs[map_name].get_latest_message()
            src = get_nearest_point_bfs([int(x) for x in Subscription.subs["robot_pose_grid"].get_latest_data()], Subscription.subs[map_name].get_latest_message())
            if not src[0]:
                return False
            return get_connected_points_bfs(src[1], Subscription.subs[map_name].get_latest_message())

        yellow_cluster = get_cluster("map_y")
        if not yellow_cluster:
            NodeGlobal.log_info("No yellow cluster found")

        white_cluster = get_cluster("map_w")
        if not white_cluster:
            NodeGlobal.log_info("No white cluster found")

        if not white_cluster or not yellow_cluster:
            return False

        clusters = [yellow_cluster, white_cluster]
        # map_data = Subscription.subs["map"].get_latest_data()
        # clusters_with_distance = []
        
        # clusters = get_top2_clusters(map_data)
        # NodeGlobal.log_info("Cluster shapes: "+str([cluster.shape for cluster in clusters]))
        # for cluster in clusters:
        #     if cluster.shape[0] < 200 and len(clusters) > 2:
        #         return False
        #     tree = cKDTree(cluster)
        #     distance, _ = tree.query(robot_coords_grid, k=1)
        #     clusters_with_distance.append((distance, cluster))
        # clusters_with_distance.sort(key=lambda x: x[0])
        # closest_cluster_distance, clusters[0] = clusters_with_distance[0]
        # closest_cluster_distance, clusters[1] = clusters_with_distance[1]
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
            NodeGlobal.log_info("No candidate goals found")
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
    # Message.add_parser(LaneFollowDisable, lambda message: message.data)
    goal_calculator = GoalCalculator()
    rclpy.spin(goal_calculator)
    goal_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
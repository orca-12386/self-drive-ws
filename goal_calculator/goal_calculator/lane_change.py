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
from interfaces.action import LaneChangeAction
from std_msgs.msg import Bool as LaneChangeStatus

# These are your package-specific imports.
from goal_calculator.ros2_wrapper import Message, Subscription, Publisher, Config, NodeGlobal
import goal_calculator.goal_calc2 as util
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


class LaneChange(Node):
    def __init__(self):
        super().__init__("lane_change_node")
        NodeGlobal.obj = self
        Config.read_config("/home/tanmay/self-drive-function/src/goal_calculator/config/config.yaml")

        self._action_server = ActionServer(
            self,
            LaneChangeAction,
            'lane_change',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback_wrapper
        )

        # Create subscriptions.
        subscription_info = {
            "map": ["map", OccupancyGrid],
            "odom": ["odom", Odometry],
            "robot_pose_global": ["robot_pose_global", PoseStamped],
            "robot_pose_grid": ["robot_pose_grid", PoseStamped],
            "robot_orientation": ["robot_orientation", Float64, 10]
        }
        Subscription.create_subscriptions(subscription_info)

        publisher_info = {
            "goal_pose": ["goal_pose", PoseStamped],
            "lane_change_status": ["lane_change_status", LaneChangeStatus]
        }
        Publisher.create_publishers(publisher_info)

        self.lane_change_active = False
        self.publish_status(False)

        self.timer = self.create_timer(0.1, self.publish_status_periodically)

    def publish_status(self, status_bool):
        status_msg = LaneChangeStatus()
        status_msg.data = status_bool
        Publisher.pubs["lane_change_status"].publish(status_msg)

    def publish_status_periodically(self):
        self.publish_status(self.lane_change_active)

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: lane_change = {goal_request.lane_change}")
        if goal_request.lane_change == 1:
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

    def goal_reached(self, curr_robot_coords, goal_pose):
        goal_coords = (goal_pose.pose.position.x, goal_pose.pose.position.y)
        distance = util.calculate_distance(curr_robot_coords, goal_coords)
        return distance < 1

    def create_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "odom"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        robot_orientation_data = Subscription.subs["robot_orientation"].get_all_data()
        # NodeGlobal.log_info(robot_orientation_data)
        avg_orientation = statistics.fmean(robot_orientation_data)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(avg_orientation / 2)
        goal_pose.pose.orientation.w = math.cos(avg_orientation / 2)

        return goal_pose

    def get_goal_pose(self, robot_coords_grid):
        def get_top2_clusters(binary_array, eps=3, min_samples=3):
            points = np.argwhere(binary_array > 0)
            points = points[:, [1, 0]]  # Convert to (x,y) format.
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
        clusters = get_top2_clusters(map_data)
        NodeGlobal.log_info("Cluster shapes: " + str([cluster.shape for cluster in clusters]))

        clusters_with_distance = []
        for cluster in clusters:
            if cluster.shape[0] < 200:
                return False
            tree = cKDTree(cluster)
            distance, _ = tree.query(robot_coords_grid, k=1)
            clusters_with_distance.append((distance, cluster))
        
        clusters_with_distance.sort(key=lambda x: x[0], reverse=True)
        furthest_cluster_distance, furthest_cluster = clusters_with_distance[0]
        closest_cluster_distance, closest_cluster = clusters_with_distance[-1]
        tree = cKDTree(furthest_cluster)
        tree2 = cKDTree(closest_cluster)
        _, nearest_index2 = tree2.query(robot_coords_grid, k=1)
        _, nearest_index = tree.query(robot_coords_grid, k=1)
        first_point = furthest_cluster[nearest_index]
        first_point2 = closest_cluster[nearest_index2]
        distance_lane2_bot = util.calculate_distance(first_point2, robot_coords_grid)
        distance_goal_bot = util.calculate_distance(robot_coords_grid, first_point)
        ratio = 0.5 * distance_lane2_bot / distance_goal_bot
        x = ((1 - ratio) * first_point[0] + (ratio) * robot_coords_grid[0])
        y = ((1 - ratio) * first_point[1] + (ratio) * robot_coords_grid[1])
        goal_coords = [x, y]

        goal_global = np.array(util.convert_to_global_coords(goal_coords))
        return self.create_goal_pose(goal_global)

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

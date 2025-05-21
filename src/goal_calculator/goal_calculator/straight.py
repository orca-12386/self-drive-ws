import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import tf_transformations
from collections import deque
import math
import threading
from std_msgs.msg import Bool
from rclpy.action import ActionServer, GoalResponse
from action_msgs.msg import GoalStatus
from interfaces.action import GoalAction as Straight
import time


class StraightActionServer(Node):
    def __init__(self):
        super().__init__("go_straight_action_server")

        self._action_server = ActionServer(
            self,
            Straight,
            'StraightTurn',
            self.execute_callback,
            goal_callback=self.goal_callback
        )

        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.yellow_map_subscription = self.create_subscription(OccupancyGrid, '/map/yellow', self.yellow_map_callback, 10)
        self.white_map_subscription = self.create_subscription(OccupancyGrid, '/map/white', self.white_map_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.yellow_point = None
        self.white_point = None
        self.initial_yellow_lane = None
        self.initial_white_lane = None
        self.center_point = None
        self.goal_yaw = None
        self.bot_position = None
        self.bot_orientation = None
        self.goal_pose = None
        self.offset = 2
        self.yellow_map_data = None
        self.white_map_data = None
        self.yellow_map_received = False
        self.white_map_received = False
        self.max_visited_cells = 500
        self.current_goal_reached = True  
        
        self._goal_handle = None
        self._action_completed = False
        self._action_running = False

        self.get_logger().info("Lane following action server initialized")

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        self._goal_handle = goal_handle
        self._action_completed = False
        self._action_running = True
        
        result = Straight.Result()
        
        self.monitor_thread = threading.Thread(target=self.monitor_lanes, args=(goal_handle, result))
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        self.timer = self.create_timer(0.2, self.publish_goal)

        rate = self.create_rate(5)
        while rclpy.ok() and self._action_running and not self._action_completed:
            rate.sleep()
        
        if not self._action_completed:
            result.success = False
            goal_handle.abort()

        return result

    def map_callback(self, msg):
        if msg is None:
            return
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

    def white_map_callback(self, msg):
        if msg is None:
            return
        self.white_map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.white_map_width = msg.info.width
        self.white_map_height = msg.info.height
        self.white_map_resolution = msg.info.resolution
        self.white_map_origin = msg.info.origin
        self.white_map_received = True

    def yellow_map_callback(self, msg):
        if msg is None:
            self.get_logger().warn("No Yellow Map Received")
            return
        self.yellow_map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.yellow_map_width = msg.info.width
        self.yellow_map_height = msg.info.height
        self.yellow_map_resolution = msg.info.resolution
        self.yellow_map_origin = msg.info.origin
        self.yellow_map_received = True

    def odom_callback(self, msg):
        if msg is None:
            return
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation
        
    def world_to_map(self, x, y, origin, resolution):
        map_x = int((x - origin.position.x) / resolution)
        map_y = int((y - origin.position.y) / resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y, origin, resolution):
        world_x = map_x * resolution + origin.position.x
        world_y = map_y * resolution + origin.position.y
        return world_x, world_y

    def get_yaw_from_quaternion(self, quat):
        euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def find_lane(self, map_data, map_width, map_height, lane_name, origin, resolution):
        if self.bot_position is None:
            self.get_logger().warn("No Bot Pose Info")
            return None

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        start_x, start_y = self.world_to_map(self.bot_position.x, self.bot_position.y, origin, resolution)

        start = (start_x, start_y)
        queue = deque([start])
        visited = {start}

        if lane_name == "Yellow":
            map_data = self.yellow_map_data
        else:
            map_data = self.white_map_data

        while queue:
            x, y = queue.popleft()
            
            if not (0 <= y < map_height and 0 <= x < map_width):
                continue
                
            if map_data[y, x] > 0:
                return (x, y)

            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < map_width and 0 <= ny < map_height and (nx, ny) not in visited:
                    queue.append((nx, ny))
                    visited.add((nx, ny))
                    
                    if len(visited) > self.max_visited_cells:
                        return None

        return None

    def find_yellow_lane(self):
        if not self.yellow_map_received or self.bot_position is None:
            return None
            
        result = self.find_lane(
            self.yellow_map_data, 
            self.yellow_map_width, 
            self.yellow_map_height, 
            "Yellow", 
            self.yellow_map_origin, 
            self.yellow_map_resolution
        )
        
        if result is not None:
            x_map, y_map = result
            x_world, y_world = self.map_to_world(
                x_map, y_map, 
                self.yellow_map_origin, 
                self.yellow_map_resolution
            )
            return x_world, y_world
        return None

    def find_white_lane(self):
        if not self.white_map_received or self.bot_position is None:
            return None
            
        result = self.find_lane(
            self.white_map_data, 
            self.white_map_width, 
            self.white_map_height, 
            "White", 
            self.white_map_origin, 
            self.white_map_resolution
        )
        
        if result is not None:
            x_map, y_map = result
            x_world, y_world = self.map_to_world(
                x_map, y_map, 
                self.white_map_origin, 
                self.white_map_resolution
            )
            return x_world, y_world
        return None

    def process_navigation(self):
        if not self._action_running:
            return
            
        if self.goal_pose is not None:
            if self.goal_reached(self.goal_pose):
                self.current_goal_reached = True
                self.get_logger().info("Goal Reached")
        
        if self.white_point is None:
            self.white_point = self.find_white_lane()
        if self.yellow_point is None:
            self.yellow_point = self.find_yellow_lane()

        if self.white_point is not None and self.yellow_point is not None and self.center_point is None:
            self.center_point = (
                (self.yellow_point[0] + self.white_point[0]) / 2,
                (self.yellow_point[1] + self.white_point[1]) / 2
            )
            self.get_logger().info(f"Center Point = {self.center_point}")

        if self.center_point is not None and self.current_goal_reached:
            self.calculate_next_goal()

    def calculate_next_goal(self):
        if self.center_point is None:
            self.get_logger().warn("Center point is not defined. Cannot calculate goal.")
            return

        self.goal_yaw = math.atan2(self.yellow_point[1] - self.white_point[1], self.yellow_point[0] - self.white_point[0]) - math.pi /2
        self.goal_yaw = round(self.goal_yaw/90) * 90

        bot_yaw = self.get_yaw_from_quaternion(self.bot_orientation)

        orientation_offset = abs(bot_yaw - self.goal_yaw) 
        if abs(orientation_offset-180) > 5:
                self.goal_yaw = math.atan2(self.yellow_point[1] - self.white_point[1], self.yellow_point[0] - self.white_point[0]) + math.pi /2
                self.goal_yaw = round(self.goal_yaw/90) * 90

        goal_x = self.center_point[0] + self.offset * math.cos(self.goal_yaw)
        goal_y = self.center_point[1] + self.offset * math.sin(self.goal_yaw)

        self.center_point = (goal_x, goal_y)

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()

        self.goal_pose.pose.position.x = goal_x
        self.goal_pose.pose.position.y = goal_y
        self.goal_pose.pose.position.z = 0.0

        orientation = tf_transformations.quaternion_from_euler(0, 0, self.goal_yaw)
        self.goal_pose.pose.orientation.x = orientation[0]
        self.goal_pose.pose.orientation.y = orientation[1]
        self.goal_pose.pose.orientation.z = orientation[2]
        self.goal_pose.pose.orientation.w = orientation[3]

        self.get_logger().debug(f"New Goal Pose = ({goal_x}, {goal_y})")
        self.current_goal_reached = False

    def goal_reached(self, goal_pose):
        if self.bot_position is None or goal_pose is None:  
            return False
        bot_x = self.bot_position.x
        bot_y = self.bot_position.y
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        distance = math.hypot(bot_x - goal_x, bot_y - goal_y)
        return distance < 0.5

    def publish_goal(self):
        if not self._action_running:
            return
            
        if self.goal_pose is not None:
            self.goal_publisher.publish(self.goal_pose)
            self.get_logger().debug(f"Publishing Goal Pose: {self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y}")
        self.process_navigation()

    def monitor_lanes(self, goal_handle, result):
        while rclpy.ok() and self._action_running:
            if self.white_point and self.yellow_point and goal_handle.is_active:
                
                new_white_point = self.find_white_lane()
                new_yellow_point = self.find_yellow_lane()

                if new_white_point and new_yellow_point:
                    white_distance = math.hypot(
                        new_white_point[0] - self.white_point[0],
                        new_white_point[1] - self.white_point[1]
                    )
                    yellow_distance = math.hypot(
                        new_yellow_point[0] - self.yellow_point[0],
                        new_yellow_point[1] - self.yellow_point[1]
                    )

                    if white_distance > 3 and yellow_distance > 3:
                        self.get_logger().info("Next Lane Reached.")
                        self._action_completed = True
                        self._action_running = False
                        
                        self.get_logger().info("Lane Keeping Action Completed")
                        result.success = True
                        goal_handle.succeed()                        
                        self.goal_pose = None
                        
                        self.timer.cancel()
                        break
            
            time.sleep(0.1)

def main():
    rclpy.init()
    node = StraightActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import math
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from merging_interface.srv import DetectMerging
from rclpy.executors import MultiThreadedExecutor

def quat_to_euler(quat):
    return Rotation.from_quat(quat).as_euler('xyz')

def euler_to_quat(euler):
    return Rotation.from_euler('xyz', euler).as_quat()

class MergingDetectorNode(Node):
    def __init__(self):
        super().__init__('merging_detector_node')
        
        self.create_subscription(OccupancyGrid, '/map/white/local', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.detection_service = self.create_service(DetectMerging, '/detection/merging', self.handle_request)

        self.initial_distance = None
        self.max_offset = 0.4
        self.map_data = None
        self.goal_pose = None
        self.bot_position = None
        self.bot_orientation = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        
        self.get_logger().info("Merging detector initialized")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.map_width = msg.info.width
        self.map_height = msg.info.height

    def odom_callback(self, msg):
        self.bot_position = msg.pose.pose.position
        self.bot_orientation = msg.pose.pose.orientation
        
    def get_yaw_from_quaternion(self, quat):
        euler = quat_to_euler([quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def world_to_map(self, x, y):
        map_x = int((x - self.map_origin.x) / self.map_resolution)
        map_y = int((y - self.map_origin.y) / self.map_resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        world_x = map_x * self.map_resolution + self.map_origin.x
        world_y = map_y * self.map_resolution + self.map_origin.y
        return world_x, world_y

    def calc_angl(self, pt1, pt2):
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]
        if dx == 0 and dy == 0:
            return 0.0
        else:
            angle = math.atan2(dy, dx)
            return angle

    def find_merging(self):
        if self.bot_position is None or self.map_data is None:
            self.get_logger().warn("Waiting for map/odom data")
            return False
            
        bot_x, bot_y = self.world_to_map(self.bot_position.x, self.bot_position.y)
        start = (bot_x, bot_y)
        queue = deque([start])
        visited = {start}
        bot_yaw = self.get_yaw_from_quaternion(self.bot_orientation)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]
        
        while queue:
            x, y = queue.popleft()
            
            if not (0 <= x < self.map_width and 0 <= y < self.map_height):
                continue
                
            if self.map_data[y, x] != 0:
                valid = False
                
                if math.pi / 4 <= bot_yaw <= 3 * math.pi / 4:
                    valid = (bot_y == y) and (x < bot_x)
                elif -3 * math.pi / 4 <= bot_yaw <= -math.pi / 4:
                    valid = (bot_y == y) and (x > bot_x)
                elif abs(bot_yaw) > 3 * math.pi / 4:
                    valid = (bot_x == x) and (y < bot_y)
                elif -math.pi / 4 < bot_yaw < math.pi / 4:
                    valid = (bot_x == x) and (y > bot_y)
                    
                if valid:
                    
                    x_world, y_world = self.map_to_world(x, y)
                    new_distance = math.sqrt((x_world - self.bot_position.x) ** 2 + 
                                           (y_world - self.bot_position.y) ** 2)
                    if self.initial_distance is not None:
                        self.get_logger().info(f"Initial Distance = {self.initial_distance}, Current Distance = {new_distance}")
                        self.get_logger().info(f"{x_world, y_world, self.bot_position.x, self.bot_position.y}")
                        print(f"Initial Distance = {self.initial_distance}, Current Distance = {new_distance}, Difference = {abs(new_distance - self.initial_distance)}")
                    if self.initial_distance is None:
                        self.initial_distance = new_distance
                    elif new_distance - self.initial_distance > self.max_offset:
                        self.get_logger().info(f"Found merging point near: ({x}, {y})")
                        return True
                    else:
                        return False
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.map_width) and (0 <= ny < self.map_height) and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        
        return False

    def handle_request(self, request, response):
        try:
            response.is_merging = self.find_merging()
            self.get_logger().info(f"Merging detection result: {response.is_merging}")
        except Exception as e:
            self.get_logger().error(f"Error in merging detection: {str(e)}")
            response.is_merging = False
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MergingDetectorNode()
    executor = MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
    finally:
        node.get_logger().info("Shutting down Merging Detector Node")
        executor.shutdown()     
        node.destroy_node()
        rclpy.shutdown()

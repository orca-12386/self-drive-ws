#!/usr/bin/env python3

import scipy.spatial
import numpy as np
import math
from goal_calculator.ros2_wrapper import Subscription, Message, Publisher, Config, NodeGlobal
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

def convert_to_global_coords(coord):
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
        
        if not (0<=p[1]<grid.info.height and 0<=p[0]<grid.info.width):
            continue
    
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
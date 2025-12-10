# Path Planning Component for Navigation Tasks

## Overview

The path planning component is a critical element of the Vision-Language-Action (VLA) system that enables autonomous navigation. This component calculates safe and efficient paths for the robot to navigate through its environment, avoiding obstacles and reaching specified destinations. The component interfaces with the ROS 2 Navigation2 stack and integrates with the broader VLA architecture.

## Architecture

The path planning component follows a modular architecture with the following key components:

1. **Path Planning Interface**: Standardized interface for path planning requests
2. **Global Planner**: Long-term path planning using map data
3. **Local Planner**: Short-term path planning considering dynamic obstacles
4. **Obstacle Avoidance**: Real-time obstacle detection and avoidance
5. **Path Execution**: Interface for following calculated paths
6. **Map Management**: Handling of static and dynamic maps

## Implementation

### Path Planning Node

```python
#!/usr/bin/env python3
"""
Path Planning Component for VLA System

This node handles path planning for navigation using ROS 2 Navigation2 stack.
It receives navigation requests and calculates safe paths for the robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from vla_msgs.srv import GetPath
from vla_msgs.msg import DetectedObjects, Object
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import math
from typing import List, Tuple, Optional
import json


class PathPlanningComponent(Node):
    """
    Path planning component for the VLA system
    """
    
    def __init__(self):
        super().__init__('path_planning_component')
        
        # QoS profile
        self.qos_profile = QoSProfile(depth=10)
        
        # TF2 setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Service for path planning requests
        self.get_path_service = self.create_service(
            GetPath,
            'get_path',
            self.get_path_callback
        )
        
        # Subscribers for map and sensor data
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            self.qos_profile
        )
        
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            self.qos_profile
        )
        
        self.detected_objects_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.detected_objects_callback,
            self.qos_profile
        )
        
        # Publishers
        self.global_plan_publisher = self.create_publisher(
            Path,
            '/plan',
            self.qos_profile
        )
        
        self.local_plan_publisher = self.create_publisher(
            Path,
            '/local_plan',
            self.qos_profile
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/path_planning/status',
            self.qos_profile
        )
        
        # Internal state
        self.current_map = None
        self.robot_pose = None
        self.dynamic_obstacles = []  # Detected objects that are obstacles
        self.static_map_resolution = 0.05  # 5cm per cell
        
        self.get_logger().info('Path Planning Component initialized')

    def get_path_callback(self, request, response):
        """
        Handle path planning service requests
        """
        self.get_logger().info(f'Received path planning request from {request.start} to {request.goal}')
        
        try:
            # Get current robot pose if start is empty
            if request.start.x == 0.0 and request.start.y == 0.0:
                robot_pose = self.get_robot_pose()
                if robot_pose:
                    start_pose = PoseStamped()
                    start_pose.pose.position.x = robot_pose.position.x
                    start_pose.pose.position.y = robot_pose.position.y
                    start_pose.pose.orientation = robot_pose.orientation
                else:
                    self.get_logger().error('Could not get robot pose for path start')
                    response.success = False
                    response.message = 'Could not determine robot pose'
                    return response
            else:
                start_pose = PoseStamped()
                start_pose.pose.position.x = request.start.x
                start_pose.pose.position.y = request.start.y
                # Set orientation to face the goal
                dx = request.goal.x - request.start.x
                dy = request.goal.y - request.start.y
                yaw = math.atan2(dy, dx)
                quat = quaternion_from_euler(0, 0, yaw)
                start_pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = request.goal.x
            goal_pose.pose.position.y = request.goal.y
            goal_pose.pose.orientation = start_pose.pose.orientation  # Keep same orientation
            goal_pose.header.frame_id = 'map'
            
            # Plan path
            path = self.plan_path(start_pose, goal_pose)
            
            if path:
                response.success = True
                response.message = f'Found path with {len(path.poses)} waypoints'
                response.path = path
            else:
                response.success = False
                response.message = 'Could not find path to goal'
        except Exception as e:
            self.get_logger().error(f'Error in path planning: {e}')
            response.success = False
            response.message = f'Path planning error: {str(e)}'
        
        return response

    def plan_path(self, start: PoseStamped, goal: PoseStamped) -> Optional[Path]:
        """
        Plan a path from start to goal position
        """
        # In a real implementation, this would interface with a path planning algorithm
        # like A*, Dijkstra, or RRT. For this example, we'll implement a simple A* algorithm
        
        if not self.current_map:
            self.get_logger().error('No map available for path planning')
            return None
        
        try:
            # Convert start and goal to map coordinates
            start_map_coords = self.world_to_map_coords(start.pose.position.x, start.pose.position.y)
            goal_map_coords = self.world_to_map_coords(goal.pose.position.x, goal.pose.position.y)
            
            if not start_map_coords or not goal_map_coords:
                self.get_logger().error('Could not convert positions to map coordinates')
                return None
            
            # Perform path planning using A* algorithm
            path_coords = self.a_star_plan(start_map_coords, goal_map_coords)
            
            if not path_coords:
                self.get_logger().info('No valid path found')
                return None
            
            # Convert path coordinates back to world coordinates
            path = self.create_path_message(path_coords, start.header.frame_id)
            
            # Publish global plan for visualization
            self.global_plan_publisher.publish(path)
            
            return path
            
        except Exception as e:
            self.get_logger().error(f'Error during path planning: {e}')
            return None

    def a_star_plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        A* path planning algorithm implementation
        """
        # Grid dimensions
        height, width = self.current_map.data.shape
        
        # Create open and closed sets
        open_set = []
        closed_set = set()
        
        # Heuristic function (Manhattan distance)
        def heuristic(pos):
            return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
        
        # Start with the start position
        open_set.append((start, 0 + heuristic(start)))  # (position, f_score)
        g_score = {start: 0}
        came_from = {}
        
        while open_set:
            # Get node with lowest f_score
            open_set.sort(key=lambda x: x[1])
            current, current_f = open_set.pop(0)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            closed_set.add(current)
            
            # Check neighbors (8-directional movement)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue  # Skip current cell
                    
                    neighbor = (current[0] + dx, current[1] + dy)
                    
                    # Check if neighbor is valid
                    if (neighbor[0] < 0 or neighbor[0] >= width or 
                        neighbor[1] < 0 or neighbor[1] >= height or 
                        neighbor in closed_set):
                        continue
                    
                    # Check if neighbor is an obstacle
                    if self.is_occupied(neighbor[0], neighbor[1]):
                        continue
                    
                    # Calculate tentative g_score
                    movement_cost = math.sqrt(dx*dx + dy*dy)  # Euclidean distance
                    tentative_g_score = g_score[current] + movement_cost
                    
                    # Check if this path to neighbor is better
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor)
                        
                        if neighbor not in [item[0] for item in open_set]:
                            open_set.append((neighbor, f_score))
        
        # No path found
        return None

    def is_occupied(self, x: int, y: int) -> bool:
        """
        Check if a cell in the map is occupied
        """
        if not self.current_map:
            return True  # If no map, assume occupied
        
        # Check bounds
        height, width = self.current_map.data.shape
        if x < 0 or x >= width or y < 0 or y >= height:
            return True
        
        # Check occupancy value (typically > 50 is considered occupied)
        value = self.current_map.data[y, x]
        return value > 50

    def world_to_map_coords(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """
        Convert world coordinates to map coordinates
        """
        if not self.current_map:
            return None
        
        try:
            # Calculate map coordinates
            map_x = int((x - self.current_map.info.origin.position.x) / self.current_map.info.resolution)
            map_y = int((y - self.current_map.info.origin.position.y) / self.current_map.info.resolution)
            
            return (map_x, map_y)
        except Exception:
            return None

    def create_path_message(self, path_coords: List[Tuple[int, int]], frame_id: str) -> Path:
        """
        Create a Path message from path coordinates
        """
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in path_coords:
            # Convert map coordinates back to world coordinates
            world_x = x * self.current_map.info.resolution + self.current_map.info.origin.position.x
            world_y = y * self.current_map.info.resolution + self.current_map.info.origin.position.y
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.pose.position.x = world_x
            pose_stamped.pose.position.y = world_y
            pose_stamped.pose.position.z = 0.0
            
            # Set orientation to face the next point
            if path_coords.index((x, y)) < len(path_coords) - 1:
                next_x, next_y = path_coords[path_coords.index((x, y)) + 1]
                next_world_x = next_x * self.current_map.info.resolution + self.current_map.info.origin.position.x
                next_world_y = next_y * self.current_map.info.resolution + self.current_map.info.origin.position.y
                
                # Calculate angle to next point
                dx = next_world_x - world_x
                dy = next_world_y - world_y
                yaw = math.atan2(dy, dx)
                
                quat = quaternion_from_euler(0, 0, yaw)
                pose_stamped.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            else:
                # For the last point, keep the same orientation as the previous point
                # (Or set to a default orientation)
                pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg

    def map_callback(self, msg: OccupancyGrid):
        """
        Handle incoming map data
        """
        try:
            # Convert 1D map data to 2D numpy array
            width = msg.info.width
            height = msg.info.height
            data_1d = np.array(msg.data)
            data_2d = data_1d.reshape((height, width))
            
            self.current_map = msg
            self.get_logger().info(f'Map updated: {width}x{height} cells at {msg.info.resolution}m/cell')
        except Exception as e:
            self.get_logger().error(f'Error processing map: {e}')

    def laser_callback(self, msg: LaserScan):
        """
        Handle laser scan data for dynamic obstacle detection
        """
        # Process laser data to detect dynamic obstacles
        # This is a simplified implementation - in practice, you'd use more sophisticated processing
        try:
            # Get robot position for context
            robot_pose = self.get_robot_pose()
            if not robot_pose:
                return
            
            # Convert laser scan to obstacle positions
            obstacles = []
            for i, range_val in enumerate(msg.ranges):
                if not (msg.range_min <= range_val <= msg.range_max):
                    continue  # Invalid range value
                
                # Calculate angle of this reading
                angle = msg.angle_min + i * msg.angle_increment
                
                # Calculate obstacle position in robot frame
                obs_x = range_val * math.cos(angle)
                obs_y = range_val * math.sin(angle)
                
                # Transform to map frame
                map_x = obs_x + robot_pose.position.x
                map_y = obs_y + robot_pose.position.y
                
                obstacles.append(Point(x=map_x, y=map_y, z=0.0))
            
            self.dynamic_obstacles = obstacles
            
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {e}')

    def detected_objects_callback(self, msg: DetectedObjects):
        """
        Handle detected objects from perception module
        """
        # Update dynamic obstacles based on detected objects
        for obj in msg.objects:
            # If object is in the robot's way, treat as obstacle
            # For now, we'll add all detected objects as potential obstacles
            # In practice, you'd only add objects that are actually obstacles
            self.dynamic_obstacles.append(obj.pose.position)

    def get_robot_pose(self):
        """
        Get the current robot pose from TF
        """
        try:
            # Try to get the robot's transform from odom to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',  # target frame
                'base_link',  # source frame
                rclpy.time.Time(),  # get the latest available
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Extract position and orientation from transform
            pose = transform.transform.translation
            orientation = transform.transform.rotation
            
            # Return as a Pose object
            from geometry_msgs.msg import Pose
            robot_pose = Pose()
            robot_pose.position = pose
            robot_pose.orientation = orientation
            
            return robot_pose
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform robot pose: {ex}')
            return None


class EnhancedPathPlanningComponent(PathPlanningComponent):
    """
    Enhanced path planning component with additional features
    """
    
    def __init__(self):
        super().__init__()
        
        # Additional subscribers for enhanced capabilities
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            self.qos_profile
        )
        
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            self.qos_profile
        )
        
        # Internal state for enhanced features
        self.robot_odom = None
        self.global_costmap = None
        self.planning_timeout = 5.0  # seconds
        self.min_path_length = 0.1  # minimum path length in meters
        
        # Obstacle inflation parameters
        self.obstacle_inflation_radius = 0.3  # meters
    
    def odom_callback(self, msg: Odometry):
        """
        Handle odometry data for current robot state
        """
        self.robot_odom = msg
    
    def costmap_callback(self, msg: OccupancyGrid):
        """
        Handle global costmap data
        """
        self.global_costmap = msg
    
    def plan_path(self, start: PoseStamped, goal: PoseStamped) -> Optional[Path]:
        """
        Enhanced path planning with dynamic obstacle consideration
        """
        # Check if goal is reachable
        goal_map_coords = self.world_to_map_coords(goal.pose.position.x, goal.pose.position.y)
        if goal_map_coords and self.is_occupied(goal_map_coords[0], goal_map_coords[1]):
            self.get_logger().warn('Goal position is in an occupied cell')
            return None
        
        # Use the base path planning implementation
        path = super().plan_path(start, goal)
        
        if path and len(path.poses) > 1:
            # Smooth the path
            smoothed_path = self.smooth_path(path)
            return smoothed_path
        
        return path
    
    def smooth_path(self, path: Path) -> Path:
        """
        Apply path smoothing to reduce number of waypoints
        """
        if len(path.poses) < 3:
            return path  # Nothing to smooth
        
        # Implement a simple smoothing algorithm
        smoothed = Path()
        smoothed.header = path.header
        
        # Start point
        smoothed.poses.append(path.poses[0])
        
        # Intermediate points with smoothing
        i = 0
        while i < len(path.poses) - 1:
            start_point = path.poses[i]
            
            # Try to find a point further along the path that has a clear line of sight
            j = len(path.poses) - 1
            while j > i:
                end_point = path.poses[j]
                
                # Check if there's a clear line of sight between start and end points
                if self.has_line_of_sight(start_point.pose.position, end_point.pose.position):
                    smoothed.poses.append(end_point)
                    i = j  # Move to the point we just added
                    break
                else:
                    j -= 1
            
            # If we couldn't find a point with line of sight, add the next point
            if i == j:
                i += 1
                if i < len(path.poses):
                    smoothed.poses.append(path.poses[i])
        
        return smoothed
    
    def has_line_of_sight(self, start: Point, end: Point) -> bool:
        """
        Check if there's a clear line of sight between two points
        """
        if not self.current_map:
            return False
        
        # Convert points to map coordinates
        start_map = self.world_to_map_coords(start.x, start.y)
        end_map = self.world_to_map_coords(end.x, end.y)
        
        if not start_map or not end_map:
            return False
        
        # Use Bresenham's line algorithm to check cells along the line
        dx = abs(end_map[0] - start_map[0])
        dy = abs(end_map[1] - start_map[1])
        x_step = 1 if start_map[0] < end_map[0] else -1
        y_step = 1 if start_map[1] < end_map[1] else -1
        error = dx - dy
        
        x, y = start_map[0], start_map[1]
        
        while x != end_map[0] or y != end_map[1]:
            if self.is_occupied(x, y):
                return False
            
            error2 = error * 2
            if error2 > -dy:
                error -= dy
                x += x_step
            if error2 < dx:
                error += dx
                y += y_step
            
            if self.is_occupied(x, y):
                return False
        
        return True


def main(args=None):
    """
    Main function to run the path planning component
    """
    rclpy.init(args=args)
    
    path_planner = EnhancedPathPlanningComponent()
    
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        path_planner.get_logger().info('Path Planning Component interrupted by user')
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Path Planning with Navigation2 Integration

```python
# navigation2_integration.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf_transformations import quaternion_from_euler
import math
from typing import Optional


class Navigation2Planner(Node):
    """
    Integration with ROS 2 Navigation2 stack for advanced path planning
    """
    
    def __init__(self):
        super().__init__('navigation2_planner')
        
        # Create action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # QoS profile
        self.qos_profile = QoSProfile(depth=10)
        
        # Publishers
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            self.qos_profile
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/navigation2_planner/status',
            self.qos_profile
        )
        
        self.get_logger().info('Navigation2 Planner initialized')
    
    def send_navigation_goal(self, x: float, y: float, theta: float = 0.0) -> bool:
        """
        Send a navigation goal to Navigation2 stack
        """
        # Wait for the action server to be available
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation2 action server not available')
            return False
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Set orientation
        quat = quaternion_from_euler(0, 0, theta)
        goal_pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        # Create action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send the goal
        self.get_logger().info(f'Sending navigation goal to ({x}, {y})')
        self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
            goal_response_callback=self.goal_response_callback
        )
        
        return True
    
    def feedback_callback(self, feedback_msg):
        """
        Handle feedback during navigation
        """
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Navigation feedback: {feedback.current_pose}')
    
    def goal_response_callback(self, future):
        """
        Handle goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by navigation server')
            return
        
        self.get_logger().info('Goal accepted by navigation server')
        # We could also set up result callback here if needed


class VLAPLusNavigationPlanner(EnhancedPathPlanningComponent):
    """
    VLA-specific navigation planner with integration to Navigation2
    """
    
    def __init__(self):
        super().__init__()
        
        # Navigation2 integration
        self.nav2_planner = Navigation2Planner()
        
        # Service to integrate with VLA orchestrator
        self.navigate_to_object_service = self.create_service(
            NavigateToObject,
            'navigate_to_object',
            self.navigate_to_object_callback
        )
    
    def navigate_to_object_callback(self, request, response):
        """
        Navigate to a specific object in the environment
        """
        object_name = request.object_name
        
        # Find object in detected objects
        target_object = self.find_object_by_name(object_name)
        if not target_object:
            response.success = False
            response.message = f'Object "{object_name}" not detected'
            return response
        
        # Navigate to the object
        success = self.nav2_planner.send_navigation_goal(
            target_object.pose.position.x,
            target_object.pose.position.y
        )
        
        if success:
            response.success = True
            response.message = f'Navigating to {object_name} at ({target_object.pose.position.x}, {target_object.pose.position.y})'
        else:
            response.success = False
            response.message = f'Failed to navigate to {object_name}'
        
        return response
    
    def find_object_by_name(self, name: str) -> Optional[Object]:
        """
        Find an object by name in the detected objects
        """
        # This would check the latest detected objects from perception
        # For now, we'll return None as this would need to interface
        # with the perception module's data
        return None
```

### Path Planning Utilities

```python
# path_planning_utils.py
import numpy as np
import math
from geometry_msgs.msg import Point, Pose
from typing import List, Tuple, Optional
from enum import Enum


class PathPlanningAlgorithm(Enum):
    """Enumeration of available path planning algorithms"""
    A_STAR = "a_star"
    DIJKSTRA = "dijkstra"
    RRT = "rrt"
    RRT_STAR = "rrt_star"
    D_STAR = "d_star"


class PathOptimizer:
    """
    Utility class for path optimization
    """
    
    def __init__(self):
        pass
    
    def optimize_path(self, path: List[Pose], algorithm: PathPlanningAlgorithm = PathPlanningAlgorithm.A_STAR):
        """
        Optimize a given path using the specified algorithm
        """
        if algorithm == PathPlanningAlgorithm.A_STAR:
            return self.optimize_with_a_star_smoothing(path)
        else:
            # For other algorithms, we'll use the same smoothing approach
            return self.optimize_with_a_star_smoothing(path)
    
    def optimize_with_a_star_smoothing(self, path: List[Pose]) -> List[Pose]:
        """
        Apply smoothing to reduce path length
        """
        if len(path) < 3:
            return path
        
        optimized_path = [path[0]]  # Start with first point
        current_idx = 0
        
        while current_idx < len(path) - 1:
            # Look ahead to find the furthest point with a valid direct path
            next_idx = len(path) - 1
            
            while next_idx > current_idx:
                if self.is_line_clear(path[current_idx], path[next_idx]):
                    optimized_path.append(path[next_idx])
                    current_idx = next_idx
                    break
                else:
                    next_idx -= 1
            
            # If no valid direct path found, just move to next point
            if current_idx == next_idx:
                current_idx += 1
                if current_idx < len(path):
                    optimized_path.append(path[current_idx])
        
        return optimized_path
    
    def is_line_clear(self, start: Pose, end: Pose) -> bool:
        """
        Check if the line between two poses is clear of obstacles
        """
        # This would check against the costmap in a real implementation
        # For now, we'll assume the line is clear
        return True


class MapProcessor:
    """
    Utility class for processing map data
    """
    
    def __init__(self):
        self.map_data = None
        self.resolution = 0.05  # Default resolution (5cm)
        self.origin = (0, 0)  # Default origin
    
    def set_map(self, occupancy_grid):
        """
        Set the map data from an OccupancyGrid message
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.array(occupancy_grid.data).reshape((height, width))
        
        self.map_data = data
        self.resolution = occupancy_grid.info.resolution
        self.origin = (
            occupancy_grid.info.origin.position.x,
            occupancy_grid.info.origin.position.y
        )
    
    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """
        Convert world coordinates to map coordinates
        """
        if self.map_data is None:
            return None
        
        map_x = int((x - self.origin[0]) / self.resolution)
        map_y = int((y - self.origin[1]) / self.resolution)
        
        # Check bounds
        height, width = self.map_data.shape
        if 0 <= map_x < width and 0 <= map_y < height:
            return (map_x, map_y)
        else:
            return None
    
    def map_to_world(self, map_x: int, map_y: int) -> Optional[Tuple[float, float]]:
        """
        Convert map coordinates to world coordinates
        """
        if self.map_data is None:
            return None
        
        world_x = map_x * self.resolution + self.origin[0]
        world_y = map_y * self.resolution + self.origin[1]
        
        return (world_x, world_y)
    
    def get_occupancy(self, x: float, y: float) -> Optional[int]:
        """
        Get the occupancy value at world coordinates
        """
        map_coords = self.world_to_map(x, y)
        if map_coords is None:
            return None
        
        map_x, map_y = map_coords
        return self.map_data[map_y, map_x]
    
    def is_free(self, x: float, y: float) -> bool:
        """
        Check if a position is free (not occupied)
        """
        occupancy = self.get_occupancy(x, y)
        if occupancy is None:
            return False  # Unknown area treated as occupied
        
        # Typically values > 50 are considered occupied
        return occupancy < 50
    
    def get_neighbors(self, x: int, y: int, eight_connected: bool = True) -> List[Tuple[int, int]]:
        """
        Get valid neighbor cells for path planning
        """
        neighbors = []
        height, width = self.map_data.shape
        
        # 4-connected neighbors
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and self.map_data[ny, nx] < 50:
                neighbors.append((nx, ny))
        
        if eight_connected:
            # Add diagonal neighbors
            for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and self.map_data[ny, nx] < 50:
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def inflate_obstacles(self, inflation_radius: float):
        """
        Inflate obstacles in the map by the specified radius
        """
        if self.map_data is None:
            return
        
        inflation_cells = int(inflation_radius / self.resolution)
        height, width = self.map_data.shape
        
        # Create a copy of the original map
        original_map = self.map_data.copy()
        
        # Inflate each occupied cell
        for y in range(height):
            for x in range(width):
                if original_map[y, x] > 50:  # If originally occupied
                    # Mark cells within inflation radius as occupied
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        for dx in range(-inflation_cells, inflation_cells + 1):
                            nx, ny = x + dx, y + dy
                            
                            # Check if within map bounds
                            if 0 <= nx < width and 0 <= ny < height:
                                # Calculate distance to original obstacle
                                dist = math.sqrt(dx*dx + dy*dy)
                                if dist <= inflation_cells:
                                    # Mark as occupied
                                    if self.map_data[ny, nx] < 99:  # Don't overwrite definitely occupied cells
                                        self.map_data[ny, nx] = 99  # Mark as definitely occupied
``` 

## Path Planning with Dynamic Obstacle Avoidance

```python
# dynamic_obstacle_avoidance.py
import numpy as np
import math
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from typing import List, Tuple, Dict
from dataclasses import dataclass


@dataclass
class Obstacle:
    """Represents a detected obstacle"""
    position: Point
    velocity: Point  # Optional for moving obstacles
    radius: float = 0.3  # Safety radius around obstacle
    is_dynamic: bool = False


class DynamicObstacleAvoidance:
    """
    Module for handling dynamic obstacles during navigation
    """
    
    def __init__(self):
        self.obstacles: List[Obstacle] = []
        self.robot_position: Point = Point(x=0.0, y=0.0, z=0.0)
        self.robot_velocity: Point = Point(x=0.0, y=0.0, z=0.0)
        self.max_detection_range = 10.0  # meters
        self.avoidance_distance = 0.5  # minimum distance to keep from obstacles
    
    def add_obstacle(self, position: Point, velocity: Point = None, radius: float = 0.3):
        """
        Add a detected obstacle
        """
        obstacle = Obstacle(
            position=position,
            velocity=velocity or Point(x=0.0, y=0.0, z=0.0),
            radius=radius,
            is_dynamic=bool(velocity)
        )
        self.obstacles.append(obstacle)
    
    def remove_obstacle(self, position: Point, tolerance: float = 0.5):
        """
        Remove an obstacle by position
        """
        self.obstacles = [
            obs for obs in self.obstacles
            if math.sqrt((obs.position.x - position.x)**2 + (obs.position.y - position.y)**2) > tolerance
        ]
    
    def get_avoidance_vector(self, target: Point) -> Tuple[float, float]:
        """
        Calculate avoidance vector based on current obstacles
        """
        avoidance_x, avoidance_y = 0.0, 0.0
        
        for obstacle in self.obstacles:
            # Calculate distance to obstacle
            dist_x = self.robot_position.x - obstacle.position.x
            dist_y = self.robot_position.y - obstacle.position.y
            distance = math.sqrt(dist_x**2 + dist_y**2)
            
            # Only consider obstacles that are close enough to be relevant
            if distance < self.max_detection_range:
                # Calculate repulsive force
                if distance < obstacle.radius + self.avoidance_distance:
                    # Very close to obstacle - strong repulsive force
                    force = 1.0 / (distance + 0.1)  # Avoid division by zero
                    avoidance_x += (dist_x / distance) * force
                    avoidance_y += (dist_y / distance) * force
                elif distance < 2 * (obstacle.radius + self.avoidance_distance):
                    # Moderately close - moderate repulsive force
                    force = 0.5 / (distance + 0.1)
                    avoidance_x += (dist_x / distance) * force
                    avoidance_y += (dist_y / distance) * force
        
        return avoidance_x, avoidance_y
    
    def compute_velocity_command(self, target: Point, current_heading: float) -> Twist:
        """
        Compute velocity command considering obstacles
        """
        cmd_vel = Twist()
        
        # Calculate desired direction to target
        dx = target.x - self.robot_position.x
        dy = target.y - self.robot_position.y
        target_distance = math.sqrt(dx*dx + dy*dy)
        
        if target_distance < 0.1:  # Close enough to target
            return cmd_vel  # Stop
        
        # Normalize direction to target
        target_x = dx / target_distance
        target_y = dy / target_distance
        
        # Get avoidance vector
        avoid_x, avoid_y = self.get_avoidance_vector(target)
        
        # Combine target direction with obstacle avoidance
        combined_x = target_x + 0.5 * avoid_x  # Weight for avoidance
        combined_y = target_y + 0.5 * avoid_y
        
        # Normalize the combined direction
        combined_magnitude = math.sqrt(combined_x**2 + combined_y**2)
        if combined_magnitude > 0:
            combined_x /= combined_magnitude
            combined_y /= combined_magnitude
        
        # Calculate linear and angular velocities
        linear_speed = min(0.5, target_distance)  # Slow down as we approach target
        cmd_vel.linear.x = linear_speed
        
        # Calculate angular velocity to face the direction
        target_angle = math.atan2(combined_y, combined_x)
        angle_diff = target_angle - current_heading
        # Normalize angle difference to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        cmd_vel.angular.z = 2.0 * angle_diff  # Proportional controller
        
        return cmd_vel
    
    def process_laser_scan(self, laser_scan: LaserScan):
        """
        Process laser scan data to detect obstacles
        """
        # Clear previous dynamic obstacles
        self.obstacles = [obs for obs in self.obstacles if not obs.is_dynamic]
        
        # Process laser data
        for i, range_val in enumerate(laser_scan.ranges):
            if not (laser_scan.range_min <= range_val <= laser_scan.range_max):
                continue  # Invalid range value
            
            # Calculate angle of this reading
            angle = laser_scan.angle_min + i * laser_scan.angle_increment
            
            # Calculate obstacle position in robot frame
            obs_x = range_val * math.cos(angle)
            obs_y = range_val * math.sin(angle)
            
            # Transform to map frame (assuming we know robot's global position)
            # In practice, this would be done with TF transforms
            map_x = obs_x + self.robot_position.x
            map_y = obs_y + self.robot_position.y
            
            obstacle_position = Point(x=map_x, y=map_y, z=0.0)
            
            # Add obstacle (assuming it's static for now)
            self.add_obstacle(obstacle_position, radius=0.15)


class LocalPathPlanner(DynamicObstacleAvoidance):
    """
    Local path planner that handles immediate obstacles and adjusts navigation
    """
    
    def __init__(self):
        super().__init__()
        self.local_path = []
        self.current_waypoint_index = 0
        self.replan_threshold = 0.5  # Replan if robot deviates by more than this distance
    
    def update_local_path(self, global_path: List[Pose], robot_position: Point):
        """
        Update local path based on global path and robot position
        """
        self.robot_position = robot_position
        
        # Find the closest point on the global path to the robot's current position
        closest_idx = 0
        min_dist = float('inf')
        
        for i, pose in enumerate(global_path):
            dist = math.sqrt(
                (pose.position.x - robot_position.x)**2 + 
                (pose.position.y - robot_position.y)**2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Set the current waypoint to the closest point or the next one
        self.current_waypoint_index = min(closest_idx + 1, len(global_path) - 1)
        
        # Create local path from current position to a point ahead on the global path
        lookahead = min(10, len(global_path) - self.current_waypoint_index)  # Look ahead 10 waypoints max
        self.local_path = global_path[self.current_waypoint_index:self.current_waypoint_index + lookahead]
    
    def get_next_waypoint(self) -> Optional[Pose]:
        """
        Get the next waypoint to navigate to
        """
        if not self.local_path or self.current_waypoint_index >= len(self.local_path):
            return None
        
        return self.local_path[self.current_waypoint_index]
    
    def has_reached_waypoint(self, tolerance: float = 0.3) -> bool:
        """
        Check if robot has reached the current waypoint
        """
        if not self.local_path or self.current_waypoint_index >= len(self.local_path):
            return True
        
        waypoint = self.local_path[self.current_waypoint_index]
        distance = math.sqrt(
            (waypoint.position.x - self.robot_position.x)**2 + 
            (waypoint.position.y - self.robot_position.y)**2
        )
        
        return distance <= tolerance
    
    def advance_waypoint(self):
        """
        Move to the next waypoint in the local path
        """
        if self.current_waypoint_index < len(self.local_path) - 1:
            self.current_waypoint_index += 1
    
    def needs_replanning(self, robot_position: Point, threshold: float = None) -> bool:
        """
        Check if the robot has deviated too far from the path and needs replanning
        """
        if threshold is None:
            threshold = self.replan_threshold
        
        if not self.local_path:
            return False
        
        # Calculate distance to current segment
        if self.current_waypoint_index > 0:
            prev_waypoint = self.local_path[self.current_waypoint_index - 1].position
            curr_waypoint = self.local_path[self.current_waypoint_index].position
            
            # Calculate distance from robot to path segment
            dist_to_segment = self.distance_to_segment(
                robot_position, prev_waypoint, curr_waypoint
            )
            
            return dist_to_segment > threshold
        
        return False
    
    def distance_to_segment(self, point: Point, seg_start: Point, seg_end: Point) -> float:
        """
        Calculate the distance from a point to a line segment
        """
        # Vector from start to end of segment
        seg_vec_x = seg_end.x - seg_start.x
        seg_vec_y = seg_end.y - seg_start.y
        seg_len_sq = seg_vec_x**2 + seg_vec_y**2
        
        if seg_len_sq == 0:
            # Segment is actually a point
            return math.sqrt((point.x - seg_start.x)**2 + (point.y - seg_start.y)**2)
        
        # Parameter for projection of point onto line
        t = max(0, min(1, ((point.x - seg_start.x) * seg_vec_x + (point.y - seg_start.y) * seg_vec_y) / seg_len_sq))
        
        # Closest point on segment
        proj_x = seg_start.x + t * seg_vec_x
        proj_y = seg_start.y + t * seg_vec_y
        
        return math.sqrt((point.x - proj_x)**2 + (point.y - proj_y)**2)


# Integration example
class IntegratedPathPlanner(VLAPLusNavigationPlanner):
    """
    Complete path planning integration for VLA system
    """
    
    def __init__(self):
        super().__init__()
        
        # Local path planner for dynamic obstacle avoidance
        self.local_planner = LocalPathPlanner()
        
        # Subscribe to robot odometry for position updates
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            self.qos_profile
        )
        
        # Subscribe to laser scan for obstacle detection
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            self.qos_profile
        )
    
    def odom_callback(self, msg: Odometry):
        """
        Update robot position from odometry
        """
        self.local_planner.robot_position = msg.pose.pose.position
        self.local_planner.robot_velocity = msg.twist.twist.linear
        
        # Update robot's heading
        from tf_transformations import euler_from_quaternion
        orientation = msg.pose.pose.orientation
        _, _, robot_heading = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # If following a path, compute and publish velocity command
        target = self.local_planner.get_next_waypoint()
        if target:
            cmd_vel = self.local_planner.compute_velocity_command(target.pose.position, robot_heading)
            # In practice, publish to velocity command topic
            # self.cmd_vel_publisher.publish(cmd_vel)
    
    def laser_callback(self, msg: LaserScan):
        """
        Process laser scan for obstacle detection
        """
        self.local_planner.process_laser_scan(msg)
    
    def follow_path(self, path: Path):
        """
        Follow a given path with dynamic obstacle avoidance
        """
        # Set the global path for the local planner
        self.local_planner.local_path = path.poses
        self.local_planner.current_waypoint_index = 0
        
        # Main control loop would run here
        # For now, we just set up the path
        self.get_logger().info(f'Starting to follow path with {len(path.poses)} waypoints')
```

## Testing the Path Planning Component

```python
# test_path_planning.py
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from vla_msgs.srv import GetPath
import time


class TestPathPlanningComponent(unittest.TestCase):
    """
    Test class for the path planning component
    """
    
    def setUp(self):
        rclpy.init()
        self.node = Node('test_path_planning')
        
        # Create service client for path requests
        self.path_client = self.node.create_client(GetPath, 'get_path')
        
        # Wait for service to be available
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            print('Path planning service not available, waiting again...')
    
    def test_path_planning_request(self):
        """
        Test that path planning service responds correctly
        """
        # Create a path planning request
        request = GetPath.Request()
        request.start = Point(x=0.0, y=0.0, z=0.0)
        request.goal = Point(x=5.0, y=5.0, z=0.0)
        
        # Call the service
        future = self.path_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        # Check response
        response = future.result()
        self.assertIsNotNone(response, "Path planning service did not respond")
        self.assertTrue(response.success, f"Path planning failed: {response.message}")
        
        if response.success:
            self.assertIsNotNone(response.path, "Path planning returned None path")
            self.assertGreater(len(response.path.poses), 0, "Path planning returned empty path")
            print(f"Successfully planned path with {len(response.path.poses)} waypoints")
    
    def test_obstacle_avoidance_path(self):
        """
        Test path planning with obstacles
        """
        # For this test, we'd need to simulate a map with obstacles
        # In practice, you'd set up test conditions with known obstacles
        
        # Create a path planning request
        request = GetPath.Request()
        request.start = Point(x=0.0, y=0.0, z=0.0)
        request.goal = Point(x=10.0, y=0.0, z=0.0)
        
        # Call the service
        future = self.path_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        # Check response
        response = future.result()
        self.assertIsNotNone(response, "Path planning service did not respond")
        
        if response.success:
            path = response.path
            print(f"Obstacle avoidance test path: {len(path.poses)} waypoints")
            
            # Verify path is valid (no waypoints are NaN or inf)
            for pose_stamped in path.poses:
                pos = pose_stamped.pose.position
                self.assertTrue(math.isfinite(pos.x), f"Invalid x coordinate: {pos.x}")
                self.assertTrue(math.isfinite(pos.y), f"Invalid y coordinate: {pos.y}")
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
```

## Launch Configuration

```xml
<!-- launch/path_planning_component.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    planner_freq = LaunchConfiguration('planner_freq', default='1.0')
    obstacle_inflation = LaunchConfiguration('obstacle_inflation', default='0.3')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'planner_freq',
            default_value='1.0',
            description='Path planner frequency in Hz'
        ),
        DeclareLaunchArgument(
            'obstacle_inflation',
            default_value='0.3',
            description='Obstacle inflation radius in meters'
        ),
        
        # Path planning component node
        Node(
            package='vla_examples',
            executable='path_planning_component',
            name='path_planning_component',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'planner_freq': planner_freq},
                {'obstacle_inflation': obstacle_inflation}
            ],
            remappings=[
                ('/map', '/map'),
                ('/scan', '/scan'),
                ('/odom', '/odom'),
            ],
            output='screen'
        )
    ])
```

## Conclusion

The path planning component implemented here provides essential navigation capabilities for the VLA system. Key features include:

1. **Global Path Planning**: A* algorithm implementation for finding optimal paths
2. **Dynamic Obstacle Avoidance**: Real-time obstacle detection and avoidance
3. **Path Smoothing**: Reduces number of waypoints for smoother navigation
4. **Integration Ready**: ROS2 service interface for easy integration with other components
5. **Navigation2 Compatibility**: Designed to work with ROS2 Navigation2 stack

The component is designed to be a critical part of the VLA architecture, providing safe and efficient navigation capabilities that enable the robot to move through its environment to perform tasks requested through the voice and cognitive planning components.
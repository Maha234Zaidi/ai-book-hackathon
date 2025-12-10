# Code Examples: Nav2 Configuration with Humanoid Constraints

This document provides practical code examples for configuring and implementing Navigation2 with humanoid-specific constraints and bipedal movement requirements.

## 1. Basic Nav2 Node with Humanoid Parameters

This example demonstrates how to create a Nav2-based navigation node with humanoid-specific parameters:

```python
#!/usr/bin/env python3
"""
Basic Nav2 Node with Humanoid Parameters
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from tf_transformations import quaternion_from_euler
import math

class HumanoidNav2Node(Node):
    def __init__(self):
        super().__init__('humanoid_nav2_node')
        
        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create publisher for velocity commands (for direct control when needed)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Humanoid-specific parameters
        self.declare_parameter('max_linear_vel', 0.3)  # Conservative for balance
        self.declare_parameter('max_angular_vel', 0.4)  # Conservative for stability
        self.declare_parameter('humanoid_safety_margin', 0.8)  # Safety buffer
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        # State variables
        self.current_goal = None
        self.is_navigating = False
        
        self.get_logger().info('Humanoid Nav2 Node initialized with safety constraints')

    def send_navigation_goal(self, x, y, theta):
        """Send navigation goal with humanoid-specific constraints"""
        # Wait for action server
        self.nav_client.wait_for_server()
        
        # Create navigation goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'  # Global map frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set goal position
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        
        # Set goal orientation using quaternion
        quat = quaternion_from_euler(0, 0, theta)
        goal.pose.pose.orientation = Quaternion(
            x=quat[0], 
            y=quat[1], 
            z=quat[2], 
            w=quat[3]
        )
        
        # Send the goal
        self.current_goal = goal
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info(
            f'Sent humanoid navigation goal: ({x}, {y}, {theta:.2f})'
        )

    def goal_response_callback(self, future):
        """Handle response from navigation server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.is_navigating = False
            return
            
        self.get_logger().info('Navigation goal accepted')
        self.is_navigating = True
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 3:  # SUCCEEDED
            self.get_logger().info('Navigation to goal succeeded')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')
        
        self.is_navigating = False

    def get_humanoid_parameters(self):
        """Get current humanoid-specific navigation parameters"""
        params = {
            'max_linear_vel': self.get_parameter('max_linear_vel').value,
            'max_angular_vel': self.get_parameter('max_angular_vel').value,
            'safety_margin': self.get_parameter('humanoid_safety_margin').value,
            'base_frame': self.get_parameter('base_frame').value,
            'odom_frame': self.get_parameter('odom_frame').value
        }
        return params

    def emergency_stop(self):
        """Send emergency stop command"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Emergency stop command sent')

def main(args=None):
    rclpy.init(args=args)
    
    humanoid_nav_node = HumanoidNav2Node()
    
    # Example: Send a navigation goal after initialization
    # In practice, goals would come from a higher-level planner
    humanoid_nav_node.get_logger().info('Sending sample navigation goal...')
    humanoid_nav_node.send_navigation_goal(2.0, 1.0, 0.0)
    
    try:
        rclpy.spin(humanoid_nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. Humanoid-Specific Costmap Configuration

This example shows how to configure costmaps with humanoid-specific parameters:

```python
#!/usr/bin/env python3
"""
Humanoid-Specific Costmap Configuration
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class HumanoidCostmapManager(Node):
    def __init__(self):
        super().__init__('humanoid_costmap_manager')
        
        # Create subscription to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Create publisher for local costmap visualization
        self.costmap_viz_pub = self.create_publisher(
            MarkerArray,
            '/humanoid_costmap_visualization',
            10
        )
        
        # Create publisher for modified costmap
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/humanoid_local_costmap',
            10
        )
        
        # Humanoid-specific parameters
        self.declare_parameter('robot_radius', 0.4)  # Larger for humanoid
        self.declare_parameter('safety_inflation', 0.8)  # Extra safety for stability
        self.declare_parameter('step_height_limit', 0.1)  # Humanoid step constraints
        
        # Costmap parameters
        self.resolution = 0.05  # 5cm per cell
        self.width = 100  # 5m wide (100 * 0.05)
        self.height = 100  # 5m high (100 * 0.05)
        
        # Initialize costmap
        self.costmap = np.zeros((self.height, self.width), dtype=np.int8)
        self.costmap_origin = (-2.5, -2.5)  # Center the map
        
        # Timer to periodically update costmap visualization
        self.viz_timer = self.create_timer(0.5, self.publish_visualization)
        
        self.get_logger().info('Humanoid Costmap Manager initialized')

    def scan_callback(self, msg):
        """Process laser scan to update humanoid-specific costmap"""
        # Reset costmap
        self.costmap.fill(0)  # Clear with free space (0)
        
        # Get robot radius and safety parameters
        robot_radius = self.get_parameter('robot_radius').value
        safety_inflation = self.get_parameter('safety_inflation').value
        
        # Convert laser scan points to costmap coordinates
        angle = msg.angle_min
        for range_val in msg.ranges:
            if not (msg.range_min <= range_val <= msg.range_max):
                angle += msg.angle_increment
                continue
                
            # Calculate Cartesian coordinates
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            
            # Convert to costmap cell coordinates
            cell_x = int((x - self.costmap_origin[0]) / self.resolution)
            cell_y = int((y - self.costmap_origin[1]) / self.resolution)
            
            # Check bounds
            if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
                # Mark as obstacle
                self.costmap[cell_y, cell_x] = 100  # Occupied
                
                # Apply humanoid-specific inflation
                self.apply_humanoid_inflation(cell_x, cell_y, robot_radius + safety_inflation)
            
            angle += msg.angle_increment

    def apply_humanoid_inflation(self, obs_x, obs_y, inflation_radius):
        """Apply humanoid-specific inflation to the costmap"""
        inflation_cells = int(inflation_radius / self.resolution)
        
        for dx in range(-inflation_cells, inflation_cells + 1):
            for dy in range(-inflation_cells, inflation_cells + 1):
                cell_x = obs_x + dx
                cell_y = obs_y + dy
                
                # Check bounds
                if not (0 <= cell_x < self.width and 0 <= cell_y < self.height):
                    continue
                
                # Calculate distance from obstacle center
                dist = math.sqrt(dx*dx + dy*dy) * self.resolution
                
                if dist <= inflation_radius:
                    # Apply distance-based cost (higher cost closer to obstacle)
                    # For humanoid, use a more aggressive inflation
                    cost = int(100 * (1 - dist / inflation_radius))
                    self.costmap[cell_y, cell_x] = max(self.costmap[cell_y, cell_x], cost)

    def publish_visualization(self):
        """Publish visualization markers for the costmap"""
        marker_array = MarkerArray()
        
        # Create grid-based visualization
        marker = Marker()
        marker.header.frame_id = "map"  # or appropriate frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "humanoid_costmap"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set scale (point size)
        marker.scale.x = self.resolution * 0.8  # Slightly smaller than cell size
        marker.scale.y = self.resolution * 0.8
        
        # Color points based on cost value
        for y in range(self.height):
            for x in range(self.width):
                if self.costmap[y, x] > 20:  # Only show cells with significant cost
                    # Convert cell coordinates to world coordinates
                    world_x = self.costmap_origin[0] + x * self.resolution
                    world_y = self.costmap_origin[1] + y * self.resolution
                    
                    point = Point()
                    point.x = world_x
                    point.y = world_y
                    point.z = 0.0
                    marker.points.append(point)
                    
                    # Color based on cost (red for obstacles, yellow for inflated areas)
                    if self.costmap[y, x] >= 90:
                        marker.colors.append(self.create_color(1.0, 0.0, 0.0, 0.7))  # Red
                    elif self.costmap[y, x] >= 50:
                        marker.colors.append(self.create_color(1.0, 1.0, 0.0, 0.5))  # Yellow
                    else:
                        marker.colors.append(self.create_color(1.0, 0.5, 0.0, 0.3))  # Orange

        marker_array.markers.append(marker)
        
        # Publish the markers
        self.costmap_viz_pub.publish(marker_array)

    def create_color(self, r, g, b, a):
        """Create a color for visualization"""
        from std_msgs.msg import ColorRGBA
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color

def main(args=None):
    rclpy.init(args=args)
    
    costmap_manager = HumanoidCostmapManager()
    
    try:
        rclpy.spin(costmap_manager)
    except KeyboardInterrupt:
        pass
    finally:
        costmap_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Humanoid Path Planner with Stability Constraints

This example demonstrates a path planner that considers humanoid stability constraints:

```python
#!/usr/bin/env python3
"""
Humanoid Path Planner with Stability Constraints
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np
import math

class HumanoidPathPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_path_planner')
        
        # Create subscription to initial pose and goal
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )
        
        # Create publisher for planned path
        self.path_pub = self.create_publisher(
            Path,
            '/humanoid_planned_path',
            10
        )
        
        # Create publisher for path visualization
        self.path_viz_pub = self.create_publisher(
            Marker,
            '/humanoid_path_visualization',
            10
        )
        
        # Humanoid-specific parameters
        self.declare_parameter('min_turning_radius', 0.5)  # Minimum turning radius for stability
        self.declare_parameter('max_step_length', 0.4)     # Maximum step length
        self.declare_parameter('max_step_width', 0.3)      # Maximum step width for stability
        
        # State variables
        self.start_pose = None
        self.goal_pose = None
        self.path = []
        
        self.get_logger().info('Humanoid Path Planner initialized')

    def initial_pose_callback(self, msg):
        """Receive initial pose for the robot"""
        self.start_pose = msg.pose.pose
        self.get_logger().info(
            f'Start pose set to: ({self.start_pose.position.x:.2f}, '
            f'{self.start_pose.position.y:.2f})'
        )

    def set_goal_pose(self, goal_pose):
        """Set goal pose and plan path"""
        self.goal_pose = goal_pose
        self.get_logger().info(
            f'Goal pose set to: ({goal_pose.position.x:.2f}, '
            f'{goal_pose.position.y:.2f})'
        )
        
        # Plan path with humanoid constraints
        self.plan_humanoid_path()

    def plan_humanoid_path(self):
        """Plan path considering humanoid stability constraints"""
        if not self.start_pose or not self.goal_pose:
            return
        
        # Get humanoid parameters
        min_turning_radius = self.get_parameter('min_turning_radius').value
        max_step_length = self.get_parameter('max_step_length').value
        
        start_x = self.start_pose.position.x
        start_y = self.start_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        # Calculate direction to goal
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate number of steps needed
        num_steps = max(1, int(distance / max_step_length))
        
        # Generate path with smooth, stable transitions
        path_poses = []
        for i in range(num_steps + 1):
            # Interpolate position
            t = i / num_steps if num_steps > 0 else 0
            x = start_x + t * dx
            y = start_y + t * dy
            
            # Create pose
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Calculate orientation towards next point (or goal)
            if i < num_steps:  # Not the last point
                next_x = start_x + ((i+1)/num_steps) * dx
                next_y = start_y + ((i+1)/num_steps) * dy
                yaw = math.atan2(next_y - y, next_x - x)
            else:  # Last point - use goal orientation
                goal_yaw = 2 * math.atan2(
                    self.goal_pose.orientation.z, 
                    self.goal_pose.orientation.w
                )
                yaw = goal_yaw
            
            # Convert yaw to quaternion
            from tf_transformations import quaternion_from_euler
            quat = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            path_poses.append(pose)
        
        # Apply path smoothing to ensure stable turning
        smoothed_path = self.smooth_humanoid_path(path_poses)
        
        # Publish the path
        self.publish_path(smoothed_path)
        
        # Publish visualization
        self.publish_path_visualization(smoothed_path)

    def smooth_humanoid_path(self, path_poses):
        """Apply smoothing that considers humanoid stability"""
        if len(path_poses) < 3:
            return path_poses
        
        # Get humanoid parameters
        min_turning_radius = self.get_parameter('min_turning_radius').value
        
        smoothed_poses = [path_poses[0]]  # Start with first pose
        
        for i in range(1, len(path_poses) - 1):
            prev_pose = path_poses[i-1]
            curr_pose = path_poses[i]
            next_pose = path_poses[i+1]
            
            # Calculate direction vectors
            prev_to_curr = np.array([
                curr_pose.pose.position.x - prev_pose.pose.position.x,
                curr_pose.pose.position.y - prev_pose.pose.position.y
            ])
            curr_to_next = np.array([
                next_pose.pose.position.x - curr_pose.pose.position.x,
                next_pose.pose.position.y - curr_pose.pose.position.y
            ])
            
            # Calculate angle between segments
            angle = self.calculate_angle_between_vectors(prev_to_curr, curr_to_next)
            
            # If angle is too sharp for humanoid, add intermediate points
            if abs(angle) > math.radians(30):  # Too sharp for stable turning
                # Add an intermediate pose to make turning smoother
                intermediate_pose = PoseStamped()
                intermediate_pose.header = curr_pose.header
                intermediate_pose.pose.position.x = curr_pose.pose.position.x
                intermediate_pose.pose.position.y = curr_pose.pose.position.y
                intermediate_pose.pose.position.z = curr_pose.pose.position.z
                
                # Calculate smoothed orientation
                avg_direction = (prev_to_curr + curr_to_next) / 2
                smooth_yaw = math.atan2(avg_direction[1], avg_direction[0])
                
                from tf_transformations import quaternion_from_euler
                quat = quaternion_from_euler(0, 0, smooth_yaw)
                intermediate_pose.pose.orientation.x = quat[0]
                intermediate_pose.pose.orientation.y = quat[1]
                intermediate_pose.pose.orientation.z = quat[2]
                intermediate_pose.pose.orientation.w = quat[3]
                
                smoothed_poses.append(intermediate_pose)
            
            # Add the original pose
            smoothed_poses.append(curr_pose)
        
        # Add the last pose
        if len(path_poses) > 1:
            smoothed_poses.append(path_poses[-1])
        
        return smoothed_poses

    def calculate_angle_between_vectors(self, v1, v2):
        """Calculate the angle between two 2D vectors"""
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        
        if norm_v1 == 0 or norm_v2 == 0:
            return 0.0
        
        v1_norm = v1 / norm_v1
        v2_norm = v2 / norm_v2
        
        dot_product = np.dot(v1_norm, v2_norm)
        # Clamp to valid range for arccos
        dot_product = max(-1.0, min(1.0, dot_product))
        
        angle = math.acos(dot_product)
        
        # Determine sign of angle using cross product
        cross_product = v1_norm[0]*v2_norm[1] - v1_norm[1]*v2_norm[0]
        if cross_product < 0:
            angle = -angle
            
        return angle

    def publish_path(self, path_poses):
        """Publish the planned path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        path_msg.poses = path_poses
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_poses)} waypoints')

    def publish_path_visualization(self, path_poses):
        """Publish visualization marker for the path"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "humanoid_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set pose orientation to identity
        marker.pose.orientation.w = 1.0
        
        # Set scale (line width)
        marker.scale.x = 0.05  # Line width
        
        # Set color (green for humanoid path)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (opacity)
        
        # Add path points
        for pose_stamped in path_poses:
            marker.points.append(pose_stamped.pose.position)
        
        self.path_viz_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    
    path_planner = HumanoidPathPlanner()
    
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Integration with Isaac ROS Navigation

This example demonstrates how to integrate humanoid navigation with Isaac ROS:

```python
#!/usr/bin/env python3
"""
Isaac ROS Humanoid Navigation Integration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
import numpy as np

class IsaacHumanoidNavigator(Node):
    def __init__(self):
        super().__init__('isaac_humanoid_navigator')
        
        # Create subscriptions for Isaac Sim sensors
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Create publisher for humanoid velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create publisher for visualization
        self.vis_pub = self.create_publisher(
            MarkerArray,
            '/isaac_humanoid_nav_visualization',
            10
        )
        
        # Humanoid-specific parameters
        self.declare_parameter('max_linear_speed', 0.3)   # Conservative for stability
        self.declare_parameter('max_angular_speed', 0.4)  # Conservative for balance
        self.declare_parameter('safety_distance', 0.6)    # Safety margin
        self.declare_parameter('step_size', 0.3)          # Humanoid step constraints
        
        # State variables
        self.current_pose = None
        self.current_odom = None
        self.scan_data = None
        self.is_obstacle_close = False
        self.last_cmd_time = self.get_clock().now()
        
        # Timer for navigation control
        self.nav_timer = self.create_timer(0.1, self.navigation_control)
        
        self.get_logger().info('Isaac Humanoid Navigator initialized')

    def odom_callback(self, msg):
        """Receive odometry data"""
        self.current_odom = msg

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.scan_data = msg
        
        # Check for close obstacles within humanoid safety distance
        safety_dist = self.get_parameter('safety_distance').value
        self.is_obstacle_close = False
        
        for range_val in msg.ranges:
            if msg.range_min <= range_val <= safety_dist:
                self.is_obstacle_close = True
                break

    def pose_callback(self, msg):
        """Receive pose estimate"""
        self.current_pose = msg.pose.pose

    def navigation_control(self):
        """Main navigation control loop with humanoid constraints"""
        if not self.current_pose or not self.scan_data:
            return
        
        # Get humanoid parameters
        max_linear = self.get_parameter('max_linear_speed').value
        max_angular = self.get_parameter('max_angular_speed').value
        
        cmd_vel = Twist()
        
        if self.is_obstacle_close:
            # Humanoid-specific obstacle avoidance
            cmd_vel = self.humanoid_avoid_obstacles(max_linear, max_angular)
        else:
            # Normal navigation - move toward goal
            # For this example, we'll simulate goal-seeking behavior
            cmd_vel.linear.x = max_linear * 0.7  # Move forward conservatively
            cmd_vel.angular.z = 0.0  # No turning unless needed
        
        # Apply humanoid-specific constraints
        cmd_vel = self.apply_humanoid_constraints(cmd_vel, max_linear, max_angular)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info(
            f'Humanoid Nav Command: Linear={cmd_vel.linear.x:.3f}, '
            f'Angular={cmd_vel.angular.z:.3f}, '
            f'Obstacle Close={self.is_obstacle_close}'
        )

    def humanoid_avoid_obstacles(self, max_linear, max_angular):
        """Humanoid-specific obstacle avoidance behavior"""
        if not self.scan_data:
            return Twist()
        
        # Analyze scan data to find the safest direction
        ranges = np.array(self.scan_data.ranges)
        angles = np.linspace(
            self.scan_data.angle_min,
            self.scan_data.angle_max,
            len(ranges)
        )
        
        # Filter out invalid ranges
        valid_mask = (ranges >= self.scan_data.range_min) & (ranges <= self.scan_data.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            # No valid readings, stop
            return Twist()
        
        # Find the direction with maximum clearance
        max_gap_idx = np.argmax(valid_ranges)
        preferred_angle = valid_angles[max_gap_idx]
        
        # Calculate avoidance command
        cmd_vel = Twist()
        
        # If the preferred direction is forward, continue forward
        if abs(preferred_angle) < 0.5:  # ~30 degrees
            cmd_vel.linear.x = max_linear * 0.3  # Slow forward movement
            cmd_vel.angular.z = 0.0
        elif preferred_angle > 0:
            # Turn right (positive angular)
            cmd_vel.linear.x = max_linear * 0.1  # Slow forward
            cmd_vel.angular.z = max_angular * 0.5  # Moderate turn rate
        else:
            # Turn left (negative angular)
            cmd_vel.linear.x = max_linear * 0.1  # Slow forward
            cmd_vel.angular.z = -max_angular * 0.5  # Moderate turn rate
        
        return cmd_vel

    def apply_humanoid_constraints(self, cmd_vel, max_linear, max_angular):
        """Apply humanoid-specific constraints to velocity commands"""
        # Limit linear velocity
        cmd_vel.linear.x = max(min(cmd_vel.linear.x, max_linear), -max_linear)
        cmd_vel.linear.y = max(min(cmd_vel.linear.y, max_linear * 0.5), -max_linear * 0.5)  # More conservative for sideways
        
        # Limit angular velocity
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, max_angular), -max_angular)
        
        # Humanoid-specific: Reduce sharp changes in commands for stability
        cmd_vel.linear.x = self.smooth_command(cmd_vel.linear.x, 0.8, max_linear * 0.1)
        cmd_vel.angular.z = self.smooth_command(cmd_vel.angular.z, 0.8, max_angular * 0.1)
        
        return cmd_vel

    def smooth_command(self, new_cmd, smoothing_factor, max_change):
        """Apply smoothing to reduce abrupt changes in commands"""
        # This is a simplified smoothing approach
        # In practice, you'd use the last command value stored in a class variable
        return new_cmd  # Placeholder - implement actual smoothing as needed

def main(args=None):
    rclpy.init(args=args)
    
    navigator = IsaacHumanoidNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Humanoid Navigation Parameter Tuning Tool

This example creates a tool for tuning humanoid navigation parameters:

```python
#!/usr/bin/env python3
"""
Humanoid Navigation Parameter Tuning Tool
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

class HumanoidParamTuner(Node):
    def __init__(self):
        super().__init__('humanoid_param_tuner')
        
        # Declare humanoid-specific parameters with descriptors
        # Velocity parameters
        self.declare_parameter(
            'humanoid_max_linear_vel',
            0.3,
            ParameterDescriptor(
                name='humanoid_max_linear_vel',
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum linear velocity for humanoid stability',
                floating_point_range=[ParameterDescriptor.FloatingPointRange(from_value=0.0, to_value=2.0, step=0.1)]
            )
        )
        
        # Safety parameters
        self.declare_parameter(
            'humanoid_safety_margin',
            0.6,
            ParameterDescriptor(
                name='humanoid_safety_margin',
                type=ParameterType.PARAMETER_DOUBLE,
                description='Safety margin for humanoid obstacle avoidance',
                floating_point_range=[ParameterDescriptor.FloatingPointRange(from_value=0.1, to_value=2.0, step=0.1)]
            )
        )
        
        # Turning parameters
        self.declare_parameter(
            'humanoid_max_angular_vel',
            0.4,
            ParameterDescriptor(
                name='humanoid_max_angular_vel',
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum angular velocity for humanoid balance',
                floating_point_range=[ParameterDescriptor.FloatingPointRange(from_value=0.0, to_value=2.0, step=0.1)]
            )
        )
        
        # Gait parameters
        self.declare_parameter(
            'humanoid_step_size',
            0.3,
            ParameterDescriptor(
                name='humanoid_step_size',
                type=ParameterType.PARAMETER_DOUBLE,
                description='Nominal step size for humanoid gait',
                floating_point_range=[ParameterDescriptor.FloatingPointRange(from_value=0.1, to_value=1.0, step=0.05)]
            )
        )
        
        # Performance monitoring publishers
        self.performance_pub = self.create_publisher(Float32, '/humanoid/performance/score', 10)
        self.stability_pub = self.create_publisher(Float32, '/humanoid/stability/score', 10)
        
        # Timer to periodically evaluate and report performance
        self.eval_timer = self.create_timer(2.0, self.evaluate_performance)
        
        # Timer to adjust parameters based on performance
        self.tune_timer = self.create_timer(5.0, self.auto_tune_parameters)
        
        self.get_logger().info('Humanoid Parameter Tuner initialized')

    def get_humanoid_params(self):
        """Get current humanoid navigation parameters"""
        params = {
            'max_linear_vel': self.get_parameter('humanoid_max_linear_vel').value,
            'safety_margin': self.get_parameter('humanoid_safety_margin').value, 
            'max_angular_vel': self.get_parameter('humanoid_max_angular_vel').value,
            'step_size': self.get_parameter('humanoid_step_size').value
        }
        return params

    def evaluate_performance(self):
        """Evaluate and report humanoid navigation performance"""
        # In a real implementation, this would evaluate actual navigation performance
        # For this example, we'll simulate performance metrics
        
        # Simulate performance scores (0.0-1.0, higher is better)
        import random
        stability_score = random.uniform(0.6, 0.9)
        efficiency_score = random.uniform(0.4, 0.8)
        
        # Publish performance metrics
        stability_msg = Float32()
        stability_msg.data = float(stability_score)
        self.stability_pub.publish(stability_msg)
        
        performance_msg = Float32()
        performance_msg.data = float(efficiency_score)
        self.performance_pub.publish(performance_msg)
        
        self.get_logger().info(
            f'Humanoid Performance - Stability: {stability_score:.2f}, '
            f'Efficiency: {efficiency_score:.2f}'
        )

    def auto_tune_parameters(self):
        """Automatically adjust parameters based on performance"""
        current_params = self.get_humanoid_params()
        
        # Example: Adjust parameters based on performance feedback
        stability_msg = Float32()
        stability_msg.data = 0.7  # Simulated current stability score
        self.stability_pub.publish(stability_msg)
        
        # If stability is poor (below threshold), reduce speeds for more stable movement
        if stability_msg.data < 0.65:
            # Reduce linear velocity for better stability
            new_linear_vel = max(0.1, current_params['max_linear_vel'] * 0.9)
            
            # Reduce angular velocity for better stability
            new_angular_vel = max(0.1, current_params['max_angular_vel'] * 0.9)
            
            # Update parameters
            self.set_parameter(Parameter('humanoid_max_linear_vel', Parameter.Type.DOUBLE, new_linear_vel))
            self.set_parameter(Parameter('humanoid_max_angular_vel', Parameter.Type.DOUBLE, new_angular_vel))
            
            self.get_logger().info(
                f'Adjusted for stability: Linear={new_linear_vel:.2f}, '
                f'Angular={new_angular_vel:.2f}'
            )
        elif stability_msg.data > 0.85:
            # If stability is good, can potentially increase speeds
            new_linear_vel = min(0.5, current_params['max_linear_vel'] * 1.1)
            self.set_parameter(Parameter('humanoid_max_linear_vel', Parameter.Type.DOUBLE, new_linear_vel))
            
            self.get_logger().info(f'Increased speed for efficiency: Linear={new_linear_vel:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    param_tuner = HumanoidParamTuner()
    
    try:
        rclpy.spin(param_tuner)
    except KeyboardInterrupt:
        pass
    finally:
        param_tuner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

These code examples demonstrate various aspects of implementing Nav2 with humanoid-specific constraints and bipedal movement requirements. Each example includes comments explaining the purpose and can be adapted to specific use cases in humanoid robotics applications.
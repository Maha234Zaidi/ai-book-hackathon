# Comprehensive Code Example: Isaac Components Integration

This document provides a comprehensive, integrated code example that demonstrates how to connect all Isaac ecosystem components into a complete AI-driven robotic system. The example illustrates best practices for integrating perception, navigation, simulation, and AI decision-making components.

## Overview of the Complete System

This integrated example shows how to build an AI-driven robot that:
1. Processes sensor data using Isaac ROS perception packages
2. Makes decisions based on perception results using AI logic
3. Executes navigation goals using Navigation2
4. Operates in Isaac Sim simulation environment
5. Maintains safety standards throughout operation

## Complete System Architecture

```
Isaac Sim (Sensors) → ROS Bridge → Perception (Isaac ROS) → AI Decision → Navigation (Nav2) → Robot Control
                                        ↓
                                    RViz Visualization
```

## Complete System Implementation

### 1. Main System Orchestrator Node

```python
#!/usr/bin/env python3
"""
Complete Isaac Ecosystem Integration Example
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Float32, Bool, String
from builtin_interfaces.msg import Time
import time
import threading
import numpy as np
from typing import Dict, List, Optional, Tuple
import json

class IsaacIntegratedSystem(Node):
    """
    Complete system orchestrator that integrates perception, navigation, 
    and AI decision-making components using the Isaac ecosystem.
    """
    
    def __init__(self):
        super().__init__('isaac_integrated_system')
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # QoS profile for commands and critical data
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Create subscriptions for all sensor inputs
        self.image_sub = self.create_subscription(
            Image, '/camera/image_rect_color', self.image_callback, sensor_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, sensor_qos
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detectnet/detections', self.detection_callback, sensor_qos
        )
        
        # Create publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', cmd_qos)
        
        # Create publisher for system status
        self.system_status_pub = self.create_publisher(String, '/system/status', cmd_qos)
        self.safety_status_pub = self.create_publisher(Bool, '/safety/emergency_stop', cmd_qos)
        
        # Create navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initialize safety system
        self.safety_system = SafetyMonitor(self)
        
        # State management
        self.system_state = "INITIALIZING"  # INITIALIZING, OPERATIONAL, EMERGENCY, SHUTDOWN
        self.latest_image = None
        self.latest_scan = None
        self.latest_camera_info = None
        self.latest_detections = []
        self.target_object = None
        self.is_navigating = False
        self.emergency_stop_active = False
        
        # Performance tracking
        self.cycle_count = 0
        self.cycle_times = []
        self.detection_confidences = []
        
        # AI decision making parameters
        self.declare_parameter('min_detection_confidence', 0.7)
        self.declare_parameter('min_safe_distance', 0.5)
        self.declare_parameter('max_navigation_attempts', 5)
        
        # Create main processing timer
        self.processing_timer = self.create_timer(0.2, self.process_cycle)
        
        self.get_logger().info('Isaac Integrated System initialized')
        self.update_system_status("SYSTEM_READY")

    def image_callback(self, msg):
        """Process incoming camera images"""
        self.latest_image = msg
        self.get_logger().debug(f'Processed image: {msg.width}x{msg.height}')

    def scan_callback(self, msg):
        """Process incoming LIDAR scans"""
        self.latest_scan = msg
        
        # Check for safety hazards
        if self.safety_system.check_for_hazards(msg):
            self.trigger_emergency_stop("Obstacle safety hazard detected")
        
        self.get_logger().debug(f'Processed scan: {len(msg.ranges)} ranges')

    def camera_info_callback(self, msg):
        """Process camera calibration information"""
        self.latest_camera_info = msg
        self.get_logger().debug(f'Updated camera info: {msg.width}x{msg.height}')

    def detection_callback(self, msg):
        """Process object detections from Isaac ROS perception"""
        if msg.detections:
            # Filter detections by confidence
            min_conf = self.get_parameter('min_detection_confidence').value
            filtered_detections = [
                detection for detection in msg.detections
                if any(result.score >= min_conf for result in detection.results)
            ]
            
            self.latest_detections = filtered_detections
            self.get_logger().info(f'Processed {len(filtered_detections)} detections')
            
            # Track confidence metrics
            for detection in filtered_detections:
                for result in detection.results:
                    self.detection_confidences.append(result.score)

    def process_cycle(self):
        """Main processing cycle - the heart of the integrated system"""
        cycle_start_time = time.time()
        
        # Update system state based on safety status
        if self.emergency_stop_active:
            self.system_state = "EMERGENCY"
            return
        else:
            self.system_state = "OPERATIONAL"
        
        # Execute AI decision-making cycle
        decision = self.ai_decision_cycle()
        
        # Execute the decision
        self.execute_decision(decision)
        
        # Track performance
        cycle_time = time.time() - cycle_start_time
        self.cycle_times.append(cycle_time)
        self.cycle_count += 1
        
        # Log performance periodically
        if self.cycle_count % 20 == 0:  # Every 20 cycles (~4 seconds)
            avg_cycle_time = sum(self.cycle_times[-20:]) / 20
            avg_confidence = sum(self.detection_confidences[-20:]) / max(1, len(self.detection_confidences[-20:]))
            
            self.get_logger().info(
                f'System Stats - Cycle time: {avg_cycle_time:.3f}s, '
                f'Detection confidence: {avg_confidence:.2f}, '
                f'Cycle count: {self.cycle_count}'
            )
        
        # Update system status
        self.update_system_status(f"OPERATIONAL - Cycle {self.cycle_count}")

    def ai_decision_cycle(self):
        """AI decision-making cycle"""
        # Evaluate current situation
        situation_analysis = self.analyze_situation()
        
        # Make decision based on analysis
        if situation_analysis['has_dangerous_obstacles']:
            return {"action": "EMERGENCY_STOP", "reason": "Dangerous obstacles detected"}
        elif situation_analysis['has_interesting_objects']:
            return {
                "action": "NAVIGATE_TO_OBJECT", 
                "target": self.select_target_object(situation_analysis['objects']),
                "reason": "Interesting object detected"
            }
        elif situation_analysis['needs_exploration']:
            return {
                "action": "EXPLORE",
                "reason": "No specific targets, exploring environment"
            }
        else:
            return {
                "action": "MONITOR", 
                "reason": "Environment is safe, monitoring"
            }

    def analyze_situation(self):
        """Analyze current situation based on sensor data"""
        analysis = {
            "has_dangerous_obstacles": False,
            "has_interesting_objects": len(self.latest_detections) > 0,
            "needs_exploration": False,
            "objects": self.latest_detections,
            "closest_obstacle_distance": float('inf'),
            "environment_complexity": 0
        }
        
        # Analyze scan data to find closest obstacle
        if self.latest_scan:
            valid_ranges = [r for r in self.latest_scan.ranges 
                           if 0.05 < r < 10.0]  # Filter valid ranges
            if valid_ranges:
                analysis["closest_obstacle_distance"] = min(valid_ranges)
                
                safe_distance = self.get_parameter('min_safe_distance').value
                analysis["has_dangerous_obstacles"] = analysis["closest_obstacle_distance"] < safe_distance
        
        # Determine if exploration is needed
        if not self.latest_detections and analysis["closest_obstacle_distance"] > 2.0:
            analysis["needs_exploration"] = True
        
        return analysis

    def select_target_object(self, objects):
        """Select the most interesting object to navigate to"""
        if not objects:
            return None
        
        # For this example, select the most confident detection
        # In a real system, you would use more sophisticated criteria
        best_detection = max(
            objects, 
            key=lambda obj: max([r.score for r in obj.results] + [0.0])
        )
        
        # Estimate position from detection (in a real system, this would use depth info)
        # For simulation, we'll just return a placeholder position
        target = {
            "detection": best_detection,
            "estimated_distance": 2.0,  # Placeholder
            "estimated_angle": 0.0,     # Placeholder
            "confidence": max([r.score for r in best_detection.results])
        }
        
        return target

    def execute_decision(self, decision):
        """Execute the AI decision"""
        action = decision["action"]
        
        if action == "EMERGENCY_STOP":
            self.trigger_emergency_stop(decision["reason"])
        elif action == "NAVIGATE_TO_OBJECT":
            target = decision.get("target")
            if target:
                self.request_navigation_to_object(target)
        elif action == "EXPLORE":
            self.request_exploration()
        elif action == "MONITOR":
            # Continue monitoring, ensure robot is stable if moving
            self.maintain_safe_motion()
        else:
            self.get_logger().warn(f"Unknown decision action: {action}")

    def request_navigation_to_object(self, target):
        """Request navigation to a specific detected object"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation server not available')
            return False
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # For this example, we'll use estimated position
        # In a real system, this would be calculated from perception
        goal.pose.pose.position.x = target["estimated_distance"] * np.cos(target["estimated_angle"])
        goal.pose.pose.position.y = target["estimated_distance"] * np.sin(target["estimated_angle"])
        goal.pose.pose.position.z = 0.0
        
        # Simple orientation (facing forward)
        goal.pose.pose.orientation.w = 1.0
        
        try:
            goal_future = self.nav_client.send_goal_async(goal)
            goal_future.add_done_callback(self.navigation_goal_sent)
            
            self.is_navigating = True
            self.get_logger().info(
                f'Navigation requested to object with confidence {target["confidence"]:.2f}'
            )
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending navigation goal: {e}')
            return False

    def navigation_goal_sent(self, future):
        """Handle navigation goal sending result"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning('Navigation goal was rejected')
                self.is_navigating = False
                return
            
            self.get_logger().info('Navigation goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_result)
            
        except Exception as e:
            self.get_logger().error(f'Error in navigation goal callback: {e}')
            self.is_navigating = False

    def navigation_result(self, future):
        """Handle navigation result"""
        try:
            result = future.result().result
            status = future.result().status
            
            if status == 3:  # SUCCEEDED
                self.get_logger().info('Navigation to object succeeded')
            else:
                self.get_logger().info(f'Navigation failed with status: {status}')
            
        except Exception as e:
            self.get_logger().error(f'Error in navigation result callback: {e}')
        finally:
            self.is_navigating = False

    def request_exploration(self):
        """Request exploration behavior"""
        # For this example, we'll just move forward slowly
        cmd = Twist()
        cmd.linear.x = 0.2  # Slow forward movement
        cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)

    def maintain_safe_motion(self):
        """Maintain safe motion during monitoring"""
        if not self.is_navigating and not self.emergency_stop_active:
            # Publish gentle movement to keep system active
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

    def trigger_emergency_stop(self, reason="No reason provided"):
        """Trigger emergency stop for safety"""
        self.get_logger().error(f"EMERGENCY STOP TRIGGERED: {reason}")
        
        # Stop all movement immediately
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Update safety status
        safety_msg = Bool()
        safety_msg.data = True
        self.safety_status_pub.publish(safety_msg)
        
        self.emergency_stop_active = True
        self.system_state = "EMERGENCY"

    def update_system_status(self, status_message):
        """Update and publish system status"""
        status_msg = String()
        status_msg.data = f"{status_message} | State: {self.system_state} | Detections: {len(self.latest_detections)}"
        self.system_status_pub.publish(status_msg)


class SafetyMonitor:
    """Safety monitoring system for the integrated robot"""
    
    def __init__(self, parent_node):
        self.parent_node = parent_node
        self.safe_distance = parent_node.get_parameter('min_safe_distance').value
        self.is_emergency = False
        
        # Emergency stop publisher (if not already available)
        self.emergency_pub = parent_node.create_publisher(
            Bool, '/safety/emergency_stop', 10
        )
        
        self.parent_node.get_logger().info('Safety Monitor initialized')

    def check_for_hazards(self, scan_data):
        """Check sensor data for safety hazards"""
        if not scan_data or not scan_data.ranges:
            return False
            
        # Find minimum distance in forward arc
        forward_start = len(scan_data.ranges) // 2 - 30
        forward_end = len(scan_data.ranges) // 2 + 30
        forward_ranges = scan_data.ranges[forward_start:forward_end]
        
        min_distance = min([r for r in forward_ranges if 0.05 < r < 10.0], default=float('inf'))
        
        is_hazard = min_distance < self.safe_distance
        
        if is_hazard:
            self.parent_node.get_logger().warning(
                f'Safety hazard detected! Distance: {min_distance:.2f}m < {self.safe_distance:.2f}m'
            )
        
        return is_hazard

    def trigger_emergency(self, reason="Safety hazard detected"):
        """Trigger emergency procedures"""
        if not self.is_emergency:
            self.is_emergency = True
            
            # Publish emergency signal
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_pub.publish(emergency_msg)


def main(args=None):
    """Main function to run the complete integrated system"""
    rclpy.init(args=args)
    
    # Create the integrated system node
    integrated_system = IsaacIntegratedSystem()
    
    try:
        # Log startup information
        integrated_system.get_logger().info(
            'Starting Isaac Integrated System - '
            'Connecting perception, navigation, and AI components...'
        )
        
        # Spin the node
        rclpy.spin(integrated_system)
        
    except KeyboardInterrupt:
        integrated_system.get_logger().info('Interrupted by user')
    except Exception as e:
        integrated_system.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Cleanup
        integrated_system.get_logger().info('Shutting down Isaac Integrated System')
        integrated_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Supporting Configuration Files

```yaml
# complete_system_config.yaml
# Configuration for the complete integrated system

# Perception parameters
perception_node:
  ros__parameters:
    # Isaac ROS specific configurations
    acceleration_mode: "gpu"
    max_features: 2000
    min_distance_between_features: 5
    
    # Detection thresholds
    min_detection_confidence: 0.7
    max_detections_per_frame: 20

# Navigation parameters
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 16
      control_frequency: 20.0
      nonholonomic: true
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.1
      stateful: true
      horizon_duration: 1.5
      control_horizon: 6
      cmd_angle_instead_rotvel: false
      v_linear_min: -0.3
      v_linear_max: 0.3
      v_angular_min: -0.6
      v_angular_max: 0.6

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 8.0  # Higher for safety
        inflation_radius: 0.8
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 6.0
        inflation_radius: 1.0
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

safety_monitor:
  ros__parameters:
    min_safe_distance: 0.5
    emergency_deceleration: 2.0
    collision_check_frequency: 10.0
```

### 3. Launch File for Complete System

```python
# complete_system_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # Default parameters file
    default_params_file = os.path.join(
        get_package_share_directory('your_robot_package'),
        'config',
        'complete_system_config.yaml'
    )
    
    # Isaac Integrated System node
    integrated_system_node = Node(
        package='your_robot_package',
        executable='isaac_integrated_system',
        name='isaac_integrated_system',
        parameters=[
            {'use_sim_time': use_sim_time},
            params_file
        ],
        remappings=[
            ('/camera/image_rect_color', '/front_camera/image_rect_color'),
            ('/scan', '/laser_scan'),
            ('/cmd_vel', '/robot/cmd_vel'),
        ],
        output='screen'
    )
    
    # Navigation system
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    navigation_node = Node(
        package='nav2_bringup',
        executable='launch_scripts',
        name='navigation_launch',
        parameters=[
            {'use_sim_time': use_sim_time},
            default_params_file
        ],
        output='screen'
    )
    
    # Isaac ROS perception components
    perception_nodes = [
        Node(
            package='isaac_ros_image_proc',
            executable='isaac_ros_rectify',
            name='rectify_node',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('image', '/front_camera/image_raw'),
                ('camera_info', '/front_camera/camera_info'),
                ('image_rect', '/front_camera/image_rect_color'),
            ]
        ),
        Node(
            package='isaac_ros_detectnet',
            executable='isaac_ros_detectnet',
            name='detectnet_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'model_name': 'ssd_mobilenet_v2_coco'},
                {'input_topic': '/front_camera/image_rect_color'},
                {'output_topic': '/detectnet/detections'}
            ]
        )
    ]
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the parameters file to use'),
        integrated_system_node,
        navigation_node,
    ] + perception_nodes)
```

## Usage Instructions

### 1. System Startup
1. Launch Isaac Sim with your environment
2. Source ROS 2 and Isaac ROS:
   ```bash
   source /opt/ros/humble/setup.bash
   source /opt/isaac_ros/setup.sh
   ```
3. Launch the complete system:
   ```bash
   ros2 launch your_robot_package complete_system_launch.py
   ```

### 2. System Operation
- The integrated system will process sensor data automatically
- Perception results will drive AI decision-making
- Navigation goals will be generated and executed based on AI decisions
- Safety systems will monitor for hazards and trigger emergency stops if needed

### 3. Monitoring and Visualization
1. Launch RViz to visualize the complete system:
   ```bash
   rviz2
   ```
2. Add displays for:
   - Local costmap (`/local_costmap/costmap`)
   - Global path (`/plan`)
   - Robot pose (`/amcl_pose`)
   - Laser scan (`/scan`)
   - Detections (`/detectnet/detections`)

## Best Practices Demonstrated

1. **Modular Design**: Clear separation of concerns between components
2. **Safety First**: Comprehensive safety monitoring and emergency procedures
3. **Performance Optimization**: Efficient data processing and resource management
4. **Error Handling**: Robust error handling and recovery behaviors
5. **System Integration**: Seamless connection between Isaac ecosystem components
6. **Monitoring and Logging**: Comprehensive system monitoring and logging
7. **Configuration Management**: Parameterized system configuration
8. **State Management**: Proper system state tracking and management

This comprehensive example demonstrates how to integrate all Isaac ecosystem components into an AI-driven robotic system while maintaining safety, performance, and reliability throughout the system.
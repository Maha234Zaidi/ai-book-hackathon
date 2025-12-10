# Code Examples: Isaac ROS VSLAM Packages

This document provides practical code examples for working with Isaac ROS VSLAM packages, demonstrating how to configure, launch, and integrate hardware-accelerated visual SLAM systems.

## 1. Basic Isaac ROS VSLAM Node Configuration

This example demonstrates how to configure and launch the Isaac ROS VSLAM node:

```python
#!/usr/bin/env python3
"""
Basic Isaac ROS VSLAM Node Configuration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import message_filters

class IsaacVSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_vslam_node')
        
        # Declare parameters for VSLAM configuration
        self.declare_parameter('enable_rectification', True)
        self.declare_parameter('enable_fisheye', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        
        # Create subscriptions for stereo camera data
        self.left_image_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/camera/image_left'
        )
        self.right_image_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/camera/image_right'
        )
        self.left_info_sub = message_filters.Subscriber(
            self, 
            CameraInfo, 
            '/camera/camera_info_left'
        )
        self.right_info_sub = message_filters.Subscriber(
            self, 
            CameraInfo, 
            '/camera/camera_info_right'
        )
        
        # Synchronize stereo inputs
        self.stereo_sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_image_sub, self.right_image_sub, 
             self.left_info_sub, self.right_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.stereo_sync.registerCallback(self.stereo_callback)
        
        # Create publisher for VSLAM pose
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/visual_slam/pose',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/visual_slam/odometry',
            10
        )
        
        self.get_logger().info('Isaac VSLAM Node initialized')

    def stereo_callback(self, left_msg, right_msg, left_info_msg, right_info_msg):
        """Process synchronized stereo camera data"""
        # In a real implementation, this would interface with Isaac ROS VSLAM
        # For demonstration, we'll simulate processing
        
        self.get_logger().info(
            f'Processing stereo pair: {left_msg.header.stamp.sec}.{left_msg.header.stamp.nanosec}'
        )
        
        # Simulate pose estimation
        self.publish_pose_estimate(left_msg.header)

    def publish_pose_estimate(self, header):
        """Publish estimated pose from VSLAM"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = 'map'
        
        # Simulate position estimate (in a real system, this would come from VSLAM)
        pose_msg.pose.pose.position.x = 1.0  # This would be from actual VSLAM
        pose_msg.pose.pose.position.y = 0.5
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        
        # Simulate covariance (representing uncertainty in estimate)
        pose_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published pose estimate: ({pose_msg.pose.pose.position.x}, {pose_msg.pose.pose.position.y})')

def main(args=None):
    rclpy.init(args=args)
    
    vslam_node = IsaacVSLAMNode()
    
    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. Isaac ROS VSLAM Launch File

This example shows how to create a launch file for Isaac ROS VSLAM:

```python
# launch_vslam.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rectification = LaunchConfiguration('enable_rectification')
    enable_fisheye = LaunchConfiguration('enable_fisheye')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')
    
    # Isaac ROS VSLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        name='visual_slam',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_rectification': enable_rectification,
            'enable_fisheye': enable_fisheye,
            'base_frame': base_frame,
            'map_frame': map_frame,
            'enable_occupancy_map': True,
            'enable_slam_visualization': True,
            # Performance parameters
            'acceleration_mode': 'gpu',
            'max_features': 2000,
            'min_distance_between_features': 10,
            'max_num_points': 10000,
        }],
        remappings=[
            ('/stereo_camera/left/image', '/camera/image_left'),
            ('/stereo_camera/right/image', '/camera/image_right'),
            ('/stereo_camera/left/camera_info', '/camera/camera_info_left'),
            ('/stereo_camera/right/camera_info', '/camera/camera_info_right'),
            ('/visual_slam/pose', '/visual_slam/pose'),
            ('/visual_slam/odometry', '/visual_slam/odometry'),
            ('/visual_slam/feature_cloud', '/visual_slam/feature_cloud'),
        ]
    )
    
    # Optional: Visual SLAM visualization node
    visual_slam_viz_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_viz_node',
        name='visual_slam_viz',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/visual_slam/feature_cloud', '/visual_slam/feature_cloud'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_visualization', default='true'))
    )
    
    # Static transform publisher for camera frame (if needed)
    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'enable_rectification',
            default_value='True',
            description='Enable camera rectification'),
        DeclareLaunchArgument(
            'enable_fisheye',
            default_value='False',
            description='Enable fisheye camera model'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame of the robot'),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame for SLAM'),
        visual_slam_node,
        visual_slam_viz_node,
        camera_tf_publisher,
    ])
```

## 3. VSLAM Parameter Tuning Example

This example demonstrates how to tune VSLAM parameters for different scenarios:

```python
#!/usr/bin/env python3
"""
VSLAM Parameter Tuning Example
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

class VSLAMParameterTuner(Node):
    def __init__(self):
        super().__init__('vslam_parameter_tuner')
        
        # Define parameter descriptors for VSLAM
        # Feature detection parameters
        self.declare_parameter(
            'max_features', 
            2000,
            ParameterDescriptor(
                name='max_features',
                type=ParameterType.PARAMETER_INTEGER,
                description='Maximum number of features to track',
                integer_range=[ParameterDescriptor.INTEGER_RANGE(min_value=100, max_value=5000, step=100)]
            )
        )
        
        # Tracking parameters
        self.declare_parameter(
            'min_distance_between_features',
            5,
            ParameterDescriptor(
                name='min_distance_between_features',
                type=ParameterType.PARAMETER_INTEGER,
                description='Minimum distance between tracked features',
                integer_range=[ParameterDescriptor.INTEGER_RANGE(min_value=1, max_value=20, step=1)]
            )
        )
        
        # Mapping parameters
        self.declare_parameter(
            'max_num_points',
            10000,
            ParameterDescriptor(
                name='max_num_points',
                type=ParameterType.PARAMETER_INTEGER,
                description='Maximum number of points in the map',
                integer_range=[ParameterDescriptor.INTEGER_RANGE(min_value=1000, max_value=50000, step=1000)]
            )
        )
        
        # Performance parameters
        self.declare_parameter(
            'acceleration_mode',
            'gpu',
            ParameterDescriptor(
                name='acceleration_mode',
                type=ParameterType.PARAMETER_STRING,
                description='Hardware acceleration mode (gpu/cuda/cpu)',
                additional_constraints=['Allowed values: gpu, cuda, cpu']
            )
        )
        
        # Timer to periodically check and adjust parameters
        self.timer = self.create_timer(10.0, self.parameter_adjustment_callback)
        
        # Service client for reconfiguring VSLAM parameters (if available)
        self.set_params_client = self.create_client(
            SetBool, 
            '/visual_slam/set_parameters'
        )
        
        self.get_logger().info('VSLAM Parameter Tuner initialized')

    def parameter_adjustment_callback(self):
        """Adjust VSLAM parameters based on performance metrics"""
        # Get current parameters
        max_features = self.get_parameter('max_features').value
        min_distance = self.get_parameter('min_distance_between_features').value
        max_points = self.get_parameter('max_num_points').value
        accel_mode = self.get_parameter('acceleration_mode').value
        
        self.get_logger().info(
            f'Current VSLAM parameters: '
            f'Max Features={max_features}, '
            f'Min Distance={min_distance}, '
            f'Max Points={max_points}, '
            f'Accel Mode={accel_mode}'
        )
        
        # Example: Adjust based on performance feedback
        # (In practice, you'd get this from VSLAM performance metrics)
        feature_tracking_efficiency = self.estimate_feature_efficiency()
        
        if feature_tracking_efficiency < 0.7:  # Low efficiency
            # Increase max features to improve tracking
            new_max_features = min(max_features + 200, 5000)
            self.set_parameter(Parameter('max_features', Parameter.Type.INTEGER, new_max_features))
            self.get_logger().info(f'Increased max features to {new_max_features}')
        
        elif feature_tracking_efficiency > 0.9:  # Too efficient, maybe too many features
            # Decrease max features to save computation
            new_max_features = max(max_features - 200, 500)
            self.set_parameter(Parameter('max_features', Parameter.Type.INTEGER, new_max_features))
            self.get_logger().info(f'Decreased max features to {new_max_features}')

    def estimate_feature_efficiency(self):
        """Estimate feature tracking efficiency (simulated)"""
        # In a real implementation, this would come from VSLAM metrics
        import random
        return random.uniform(0.6, 0.95)  # Simulated efficiency value

def main(args=None):
    rclpy.init(args=args)
    
    param_tuner = VSLAMParameterTuner()
    
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

## 4. VSLAM Mapping and Localization Service

This example demonstrates how to create services for controlling VSLAM operations:

```python
#!/usr/bin/env python3
"""
VSLAM Mapping and Localization Service
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class VSLAMService(Node):
    def __init__(self):
        super().__init__('vslam_service')
        
        # Create services
        self.start_mapping_srv = self.create_service(
            Trigger,
            'start_mapping',
            self.start_mapping_callback
        )
        
        self.stop_mapping_srv = self.create_service(
            Trigger, 
            'stop_mapping', 
            self.stop_mapping_callback
        )
        
        self.start_localization_srv = self.create_service(
            Trigger,
            'start_localization',
            self.start_localization_callback
        )
        
        self.reset_map_srv = self.create_service(
            Trigger,
            'reset_map',
            self.reset_map_callback
        )
        
        # Create subscription to VSLAM pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/visual_slam/pose',
            self.pose_callback,
            10
        )
        
        # Create publisher for map visualization
        self.map_viz_pub = self.create_publisher(
            MarkerArray,
            '/vslam_map_visualization',
            10
        )
        
        # State variables
        self.mapping_active = False
        self.localization_active = False
        self.map_points = []
        self.robot_path = []
        
        self.get_logger().info('VSLAM Service initialized')

    def start_mapping_callback(self, request, response):
        """Begin mapping operation"""
        self.mapping_active = True
        self.localization_active = True  # Mapping includes localization
        self.map_points.clear()
        self.robot_path.clear()
        
        response.success = True
        response.message = 'Mapping started'
        self.get_logger().info('VSLAM: Mapping started')
        
        return response

    def stop_mapping_callback(self, request, response):
        """Stop mapping operation"""
        self.mapping_active = False
        response.success = True
        response.message = 'Mapping stopped'
        self.get_logger().info('VSLAM: Mapping stopped')
        
        return response

    def start_localization_callback(self, request, response):
        """Begin localization-only operation"""
        self.localization_active = True
        self.mapping_active = False  # Only localization, no map building
        
        response.success = True
        response.message = 'Localization started'
        self.get_logger().info('VSLAM: Localization started')
        
        return response

    def reset_map_callback(self, request, response):
        """Reset the current map"""
        self.map_points.clear()
        self.robot_path.clear()
        
        response.success = True
        response.message = 'Map reset'
        self.get_logger().info('VSLAM: Map reset')
        
        return response

    def pose_callback(self, msg):
        """Process VSLAM pose estimates"""
        if self.localization_active:
            # Store the robot's position for path visualization
            pos = msg.pose.pose.position
            self.robot_path.append((pos.x, pos.y, pos.z))
            
            # If mapping is active, also store features or map points
            if self.mapping_active:
                # In a real implementation, you'd extract map points from the VSLAM system
                # For this example, we'll simulate adding points based on robot position
                self.add_simulated_map_point(pos)

            # Publish visualization
            self.publish_visualization()

    def add_simulated_map_point(self, position):
        """Add a simulated map point (in real VSLAM this would be from feature triangulation)"""
        # Add some noise to make it realistic
        noise = np.random.normal(0, 0.1, 3)
        point = (position.x + noise[0], position.y + noise[1], position.z + noise[2])
        self.map_points.append(point)

    def publish_visualization(self):
        """Publish visualization markers for the map and path"""
        marker_array = MarkerArray()
        
        # Path marker
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "vslam_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.05  # Line width
        path_marker.color.a = 1.0
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        
        # Add path points
        for x, y, z in self.robot_path:
            point = path_marker.points.add()
            point.x = x
            point.y = y
            point.z = z
        
        marker_array.markers.append(path_marker)
        
        # Map points marker
        map_marker = Marker()
        map_marker.header.frame_id = "map"
        map_marker.header.stamp = self.get_clock().now().to_msg()
        map_marker.ns = "vslam_map_points"
        map_marker.id = 1
        map_marker.type = Marker.POINTS
        map_marker.action = Marker.ADD
        map_marker.pose.orientation.w = 1.0
        map_marker.scale.x = 0.1  # Point size
        map_marker.scale.y = 0.1
        map_marker.color.a = 1.0
        map_marker.color.r = 1.0
        map_marker.color.g = 0.0
        map_marker.color.b = 0.0
        
        # Add map points
        for x, y, z in self.map_points:
            point = map_marker.points.add()
            point.x = x
            point.y = y
            point.z = z
        
        marker_array.markers.append(map_marker)
        
        # Publish the markers
        self.map_viz_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    vslam_service = VSLAMService()
    
    try:
        rclpy.spin(vslam_service)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Integration with Navigation Stack

This example shows how to integrate VSLAM with ROS 2 Navigation2:

```python
#!/usr/bin/env python3
"""
VSLAM Integration with Navigation2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
import numpy as np

class VSLAMNavigationIntegrator(Node):
    def __init__(self):
        super().__init__('vslam_navigation_integrator')
        
        # Create subscription to VSLAM pose
        self.vslam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/visual_slam/pose',
            self.vslam_pose_callback,
            10
        )
        
        # Create action client for Navigation2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for visualization
        self.goal_viz_pub = self.create_publisher(
            Marker,
            '/vslam_navigation_goal',
            10
        )
        
        # Timer to periodically check VSLAM localization quality
        self.quality_check_timer = self.create_timer(1.0, self.check_localization_quality)
        
        # State variables
        self.current_vslam_pose = None
        self.localization_quality_score = 1.0  # 0.0 (poor) to 1.0 (excellent)
        self.navigation_goal = None
        
        self.get_logger().info('VSLAM Navigation Integrator initialized')

    def vslam_pose_callback(self, msg):
        """Process VSLAM pose estimates"""
        self.current_vslam_pose = msg.pose.pose
        
        # Estimate localization quality based on covariance
        # Smaller covariance indicates better localization
        covariance = np.array(msg.pose.covariance).reshape(6, 6)
        pos_cov = covariance[0:3, 0:3]  # Position covariance
        trace = np.trace(pos_cov)
        
        # Inverse relationship: smaller trace = better localization
        # Normalize to [0, 1] range (1.0 being best)
        max_expected_trace = 1.0  # Adjust based on your system
        self.localization_quality_score = max(0.0, min(1.0, (max_expected_trace - trace) / max_expected_trace))
        
        self.get_logger().info(
            f'VSLAM Pose: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}), '
            f'Quality: {self.localization_quality_score:.2f}'
        )

    def send_navigation_goal(self, x, y, theta):
        """Send navigation goal to Navigation2"""
        # Wait for action server
        self.nav_client.wait_for_server()
        
        # Create navigation goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'  # VSLAM map frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        
        # Convert angle to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal.pose.pose.orientation.x = quat[0]
        goal.pose.pose.orientation.y = quat[1]
        goal.pose.pose.orientation.z = quat[2]
        goal.pose.pose.orientation.w = quat[3]
        
        # Send goal
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        
        # Store goal for visualization
        self.navigation_goal = (x, y)
        
        # Visualize the goal
        self.visualize_goal(x, y)
        
        self.get_logger().info(f'Navigation goal sent: ({x}, {y})')

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return
            
        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 3:  # SUCCEEDED (ActionServer status)
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

    def check_localization_quality(self):
        """Periodically check localization quality and trigger relocalization if needed"""
        if self.localization_quality_score < 0.3:  # Poor quality threshold
            self.get_logger().warning(
                f'Poor localization quality ({self.localization_quality_score:.2f}), '
                'considering relocalization strategies'
            )
            # In a real system, you might trigger a relocalization process here
            # This could involve searching for known features or fiducial markers

    def visualize_goal(self, x, y):
        """Publish visualization marker for navigation goal"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "navigation_goals"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2  # Slightly above ground
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.g = 1.0  # Green color for goal
        
        self.goal_viz_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    
    integrator = VSLAMNavigationIntegrator()
    
    # Example: Send a navigation goal after initialization
    # In practice, goals would come from a higher-level planner
    integrator.get_logger().info('Waiting for VSLAM pose...')
    import time
    time.sleep(2)  # Wait for VSLAM to initialize
    
    # Send a sample navigation goal
    integrator.send_navigation_goal(2.0, 1.5, 0.0)
    
    try:
        rclpy.spin(integrator)
    except KeyboardInterrupt:
        pass
    finally:
        integrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6. Performance Monitoring for VSLAM

This example shows how to monitor the performance of VSLAM systems:

```python
#!/usr/bin/env python3
"""
VSLAM Performance Monitoring
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
import time
import statistics

class VSLAMPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('vslam_performance_monitor')
        
        # Create subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10
        )
        
        self.vslam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/visual_slam/pose',
            self.vslam_pose_callback,
            10
        )
        
        # Create publishers for performance metrics
        self.fps_pub = self.create_publisher(Float32, '/vslam/performance/fps', 10)
        self.latency_pub = self.create_publisher(Float32, '/vslam/performance/latency', 10)
        self.tracking_quality_pub = self.create_publisher(Float32, '/vslam/performance/tracking_quality', 10)
        
        # Performance tracking variables
        self.frame_times = []
        self.pose_processing_times = []
        self.feature_count_history = []
        self.last_image_time = None
        self.last_pose_time = None
        
        # Timer to publish performance metrics periodically
        self.metrics_timer = self.create_timer(1.0, self.publish_performance_metrics)
        
        self.get_logger().info('VSLAM Performance Monitor initialized')

    def image_callback(self, msg):
        """Process image messages to measure FPS"""
        current_time = time.time()
        
        if self.last_image_time is not None:
            # Calculate time between frames
            dt = current_time - self.last_image_time
            fps = 1.0 / dt if dt > 0 else 0.0
            
            # Store for statistics
            self.frame_times.append(fps)
            if len(self.frame_times) > 10:  # Keep last 10 measurements
                self.frame_times.pop(0)
        
        self.last_image_time = current_time

    def vslam_pose_callback(self, msg):
        """Process VSLAM pose messages to measure latency and tracking quality"""
        current_time = time.time()
        
        # Calculate processing latency if we have a timestamp
        if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
            msg_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
            latency = current_time - msg_time
            
            self.pose_processing_times.append(latency)
            if len(self.pose_processing_times) > 10:
                self.pose_processing_times.pop(0)
        
        # Estimate tracking quality from covariance (lower covariance = better tracking)
        covariance = msg.pose.covariance
        pos_covariance = covariance[0] + covariance[7] + covariance[14]  # Diagonal elements for position
        tracking_quality = max(0.0, min(1.0, 1.0 / (1.0 + pos_covariance)))  # Normalize to [0,1]
        
        self.feature_count_history.append(tracking_quality)
        if len(self.feature_count_history) > 10:
            self.feature_count_history.pop(0)

    def publish_performance_metrics(self):
        """Publish collected performance metrics"""
        # FPS metric
        if self.frame_times:
            avg_fps = statistics.mean(self.frame_times)
            fps_msg = Float32()
            fps_msg.data = float(avg_fps)
            self.fps_pub.publish(fps_msg)
            
            self.get_logger().info(f'VSLAM FPS: {avg_fps:.2f}')

        # Latency metric
        if self.pose_processing_times:
            avg_latency = statistics.mean(self.pose_processing_times)
            latency_msg = Float32()
            latency_msg.data = float(avg_latency)
            self.latency_pub.publish(latency_msg)
            
            self.get_logger().info(f'VSLAM Latency: {avg_latency:.3f}s')

        # Tracking quality metric
        if self.feature_count_history:
            avg_quality = statistics.mean(self.feature_count_history)
            quality_msg = Float32()
            quality_msg.data = float(avg_quality)
            self.tracking_quality_pub.publish(quality_msg)
            
            self.get_logger().info(f'VSLAM Tracking Quality: {avg_quality:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    monitor = VSLAMPerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

These code examples demonstrate various aspects of working with Isaac ROS VSLAM packages, including basic configuration, parameter tuning, integration with navigation systems, and performance monitoring. Each example includes comments explaining the purpose and can be adapted to specific use cases in robotics applications.
# Isaac Sim and ROS Integration Code Examples

This document provides practical code examples for integrating Isaac Sim with ROS 2, demonstrating how to connect simulated environments with the Isaac ROS hardware-accelerated packages.

## 1. Basic Isaac Sim ROS Bridge Setup

This example demonstrates how to set up the basic ROS bridge between Isaac Sim and ROS 2:

```python
#!/usr/bin/env python3
"""
Basic Isaac Sim to ROS2 Bridge Example
This script demonstrates connecting Isaac Sim sensors to ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class IsaacSimRosBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_ros_bridge')
        
        # Create subscribers for Isaac Sim commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publishers for Isaac Sim sensors
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera_info',
            10
        )
        
        self.get_logger().info('Isaac Sim ROS Bridge initialized')

    def cmd_vel_callback(self, msg):
        """Process velocity commands from ROS2 and send to simulated robot"""
        self.get_logger().info(f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')
        # In a real implementation, this would interface with Isaac Sim
        # to control the simulated robot based on ROS2 commands

def main(args=None):
    rclpy.init(args=args)
    
    bridge = IsaacSimRosBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. Isaac ROS Perception Pipeline

This example shows how to implement a perception pipeline using Isaac ROS packages:

```python
#!/usr/bin/env python3
"""
Isaac ROS Perception Pipeline Example
This script demonstrates how to process data using Isaac ROS accelerated packages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from vision_msgs.msg import Detection2DArray
import message_filters

class IsaacPerceptionPipeline(Node):
    def __init__(self):
        super().__init__('isaac_perception_pipeline')
        
        # Subscribe to camera image data
        self.image_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/camera/image_raw'
        )
        
        # Subscribe to camera info
        self.camera_info_sub = message_filters.Subscriber(
            self, 
            CameraInfo, 
            '/camera/camera_info'
        )
        
        # Synchronize image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.image_callback)
        
        # Publish AprilTag detections
        self.detection_pub = self.create_publisher(
            AprilTagDetectionArray,
            '/tag_detections',
            10
        )
        
        # Publish general object detections
        self.object_pub = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )
        
        self.get_logger().info('Isaac Perception Pipeline initialized')

    def image_callback(self, image_msg, camera_info_msg):
        """Process synchronized image and camera info"""
        self.get_logger().info(f'Processing image: {image_msg.height}x{image_msg.width}')
        
        # In a real implementation, this would interface with Isaac ROS nodes
        # to perform accelerated perception tasks
        
        # Create mock detection results
        detection_array = AprilTagDetectionArray()
        detection_array.header = image_msg.header
        # Process image using Isaac ROS accelerated algorithms
        # (actual processing would happen in Isaac ROS nodes)

def main(args=None):
    rclpy.init(args=args)
    
    perception_pipeline = IsaacPerceptionPipeline()
    
    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Isaac Sim Python API Integration

This example shows how to use Isaac Sim's Python API to create a simulation with ROS integration:

```python
#!/usr/bin/env python3
"""
Isaac Sim Python API with ROS Integration Example
This script demonstrates programmatic control of Isaac Sim with ROS communication.
"""

from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.viewports import set_active_viewport_camera_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni
from pxr import Gf
from omni.isaac.core.utils.nucleus import get_assets_root_path
import carb

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import threading
import time

# Initialize Isaac Sim application
config = {"headless": False}
simulation_app = SimulationApp(config)

# ROS Node for communication
class IsaacSimController(Node):
    def __init__(self):
        super().__init__('isaac_sim_controller')
        
        # Create subscriber for robot commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.cmd_vel_data = None

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = msg
        self.get_logger().info(f'CmdVel received: {msg.linear.x}, {msg.angular.z}')

def ros_thread():
    """Thread function to run ROS node"""
    rclpy.init()
    
    controller = IsaacSimController()
    
    while simulation_app.is_running():
        rclpy.spin_once(controller, timeout_sec=0.1)
        time.sleep(0.01)  # 100Hz update rate
    
    controller.destroy_node()
    rclpy.shutdown()

def main():
    # Start ROS thread
    ros_thread_instance = threading.Thread(target=ros_thread)
    ros_thread_instance.start()
    
    # Setup Isaac Sim World
    world = World(stage_units_in_meters=1.0)
    
    # Add ground plane
    world.scene.add_default_ground_plane()
    
    # Add a simple robot (in this example, a cube)
    from omni.isaac.core.objects import DynamicCuboid
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/cube",
            name="cube",
            position=Gf.Vec3f(0.0, 0.0, 1.0),
            size=0.5,
            color=Gf.Vec3f(0.8, 0.2, 0.2)
        )
    )
    
    # Reset world to apply changes
    world.reset()
    
    # Play simulation
    world.play()
    
    # Main simulation loop
    while simulation_app.is_running():
        # Update Isaac Sim
        simulation_app.update()
        
        # Here you would implement logic to use ROS commands to control the simulated robot
        # For example, reading from the controller.cmd_vel_data and applying forces to the cube
        
        # Simulate for a while
        if world.current_time_step_index >= 1000:
            break
    
    # Shutdown
    world.stop()
    simulation_app.close()
    ros_thread_instance.join()

if __name__ == "__main__":
    main()
```

## 4. Isaac ROS Launch File Example

Create a launch file to orchestrate Isaac ROS nodes:

```xml
<!-- isaac_sim_perception.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Isaac ROS AprilTag detection node
    apriltag_node = Node(
        package='isaac_ros_apriltag',
        executable='isaac_ros_apriltag',
        name='apriltag',
        parameters=[{
            'family': 'tag36h11',
            'size': 0.166,
            'max_tags': 64,
            'tile_size': 1,
            'decimate': 1.0,
            'blur': 0.0,
            'refine_edges': True,
            'refine_decode': True,
            'refine_pose': True,
            'debug': False,
            'timing': False,
        }],
        remappings=[
            ('image', '/camera/image_rect'),
            ('camera_info', '/camera/camera_info'),
            ('detections', 'tag_detections'),
        ]
    )
    
    # Isaac ROS Depth Segmentation node (if needed)
    depth_segmentation_node = Node(
        package='isaac_ros_depth_segmentation',
        executable='isaac_ros_depth_segmentation',
        name='depth_segmentation',
        parameters=[{
            'engine_file_path': '/path/to/trt/engine',
            'input_tensor_names': ['input_tensor'],
            'output_tensor_names': ['output_tensor'],
            'input_binding_names': ['input'],
            'output_binding_names': ['output'],
            'verbose': True,
        }],
        remappings=[
            ('image', '/camera/image_rect'),
            ('depth_segmentation', 'depth_segmentation_result'),
        ]
    )
    
    # Isaac ROS VSLAM node (if needed)
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        name='visual_slam',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'enable_occupancy_map': True,
        }],
        remappings=[
            ('/stereo_camera/left/image', '/camera/image_left'),
            ('/stereo_camera/right/image', '/camera/image_right'),
            ('/stereo_camera/left/camera_info', '/camera/camera_info_left'),
            ('/stereo_camera/right/camera_info', '/camera/camera_info_right'),
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock if true'),
        apriltag_node,
        # Add depth_segmentation_node if needed
        # depth_segmentation_node,
        # Add visual_slam_node if needed
        # visual_slam_node,
    ])
```

## 5. Isaac Sim Configuration for ROS Integration

Example configuration for connecting Isaac Sim to ROS:

```python
# isaac_sim_robot_config.py
"""
Configuration for Isaac Sim robot with ROS integration
"""

# Robot properties
ROBOT_CONFIG = {
    "name": "isaac_carter",
    "usd_path": "/Isaac/Robots/Carter/carter_navigable.usd",
    "position": [0.0, 0.0, 0.0],
    "orientation": [0.0, 0.0, 0.0, 1.0],
    "scale": [1.0, 1.0, 1.0],
    
    # ROS integration
    "ros_bridge": {
        "enabled": True,
        "prefix": "carter",
    },
    
    # Sensors
    "sensors": [
        {
            "name": "camera",
            "type": "rgb",
            "position": [0.0, 0.0, 0.5],
            "orientation": [0.0, 0.0, 0.0],
            "focal_length": 2.0,
            "resolution": [640, 480],
            "topic": "/carter/camera/image_raw"
        },
        {
            "name": "lidar",
            "type": "lidar",
            "position": [0.0, 0.0, 1.0],
            "topic": "/carter/lidar/scan"
        }
    ],
    
    # Differential drive controller
    "drive_controller": {
        "type": "differential",
        "wheel_separation": 0.5,
        "wheel_radius": 0.1,
        "topic": "/carter/cmd_vel"
    }
}

# Environment properties
ENVIRONMENT_CONFIG = {
    "name": "simple_room",
    "usd_path": "/Isaac/Environments/Simple_Room/simple_room.usd",
    "physics": {
        "gravity": [0.0, 0.0, -9.81],
    }
}
```

## 6. Performance Comparison Example

Example to demonstrate the performance benefits of hardware acceleration:

```python
#!/usr/bin/env python3
"""
Performance comparison between hardware-accelerated and CPU-based processing
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class PerformanceComparison(Node):
    def __init__(self):
        super().__init__('perf_comparison')
        
        # Subscribe to raw camera feed
        self.raw_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.raw_image_callback,
            10
        )
        
        # Publish timing information
        self.accel_time_pub = self.create_publisher(
            Float32,
            '/timing/accelerated',
            10
        )
        
        self.cpu_time_pub = self.create_publisher(
            Float32,
            '/timing/cpu_only',
            10
        )
        
        self.get_logger().info('Performance comparison node initialized')

    def raw_image_callback(self, msg):
        """Process image with both accelerated and CPU-only methods"""
        # Simulate accelerated processing time (Isaac ROS)
        start_time = time.time()
        # In real implementation, this would use Isaac ROS accelerated packages
        time.sleep(0.005)  # Simulate accelerated processing time (5ms)
        accel_time = time.time() - start_time
        
        # Simulate CPU-only processing time (standard ROS)
        start_time = time.time()
        # Simulate what CPU-only processing might take
        time.sleep(0.020)  # Simulate CPU processing time (20ms)
        cpu_time = time.time() - start_time
        
        # Publish timing results
        accel_msg = Float32()
        accel_msg.data = accel_time
        self.accel_time_pub.publish(accel_msg)
        
        cpu_msg = Float32()
        cpu_msg.data = cpu_time
        self.cpu_time_pub.publish(cpu_msg)
        
        self.get_logger().info(f'Accelerated: {accel_time:.4f}s, CPU: {cpu_time:.4f}s, Speedup: {cpu_time/accel_time:.2f}x')

def main(args=None):
    rclpy.init(args=args)
    
    perf_node = PerformanceComparison()
    
    try:
        rclpy.spin(perf_node)
    except KeyboardInterrupt:
        pass
    finally:
        perf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

These code examples demonstrate various aspects of Isaac Sim and ROS integration, showcasing how to set up communication channels, implement perception pipelines, and leverage hardware acceleration for improved performance. Each example includes comments explaining the purpose and can be adapted to specific use cases.
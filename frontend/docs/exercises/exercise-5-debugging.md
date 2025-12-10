---
title: Exercise 5 - Debugging ROS 2 Systems
description: Hands-on exercise for debugging ROS 2 applications
sidebar_position: 5
---

# Exercise 5: Debugging ROS 2 Systems

## Objective

- Learn to use ROS 2 debugging tools effectively
- Apply systematic debugging approaches to ROS 2 systems
- Identify and fix common ROS 2 issues
- Use logging and monitoring tools for troubleshooting

## Prerequisites

- Understanding of ROS 2 concepts from all previous sections
- Experience with creating ROS 2 nodes
- Basic understanding of ROS 2 tools

## Setup Requirements

- ROS 2 Humble Hawksbill installed and sourced
- Python 3.10 or newer
- Basic ROS 2 workspace setup

## Steps

### 1. Set Up a Debugging Workspace

1. Create a new workspace specifically for debugging exercises:
   ```bash
   mkdir -p ~/debug_ws/src
   cd ~/debug_ws
   ```

2. Create a package for debugging exercises:
   ```bash
   cd src
   ros2 pkg create --build-type ament_python debugging_ex --dependencies rclpy std_msgs geometry_msgs sensor_msgs example_interfaces
   ```

### 2. Create a Node with Common Issues

Create the problematic node at `debugging_ex/debugging_ex/problematic_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time

class ProblematicNode(Node):
    def __init__(self):
        super().__init__('problematic_node')
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'problematic_topic', 10)
        
        # Intentionally create a subscription to the same topic we publish to
        # This can cause feedback loops
        self.subscription = self.create_subscription(
            String,
            'problematic_topic',  # Same as publishing topic
            self.feedback_callback,
            10
        )
        
        # Create timer that publishes very frequently
        self.timer = self.create_timer(0.001, self.problematic_timer)  # Very fast timer
        
        # Intentional memory leak - keep appending to list without clearing
        self.data_buffer = []
        
        self.counter = 0
        self.get_logger().info('Problematic node initialized with issues')

    def feedback_callback(self, msg):
        # This callback will be triggered by our own publications
        self.get_logger().info(f'Received feedback: {msg.data}')
        # Adding to the buffer will create a memory leak
        self.data_buffer.append(f'Feedback data: {msg.data}')

    def problematic_timer(self):
        # Publish at very high frequency
        msg = String()
        msg.data = f'High frequency message {self.counter}'
        self.publisher.publish(msg)
        
        # Add to buffer, causing memory leak
        self.data_buffer.append(msg.data)
        
        self.counter += 1
        if self.counter % 100 == 0:
            self.get_logger().info(f'Published {self.counter} messages, buffer size: {len(self.data_buffer)}')


def main(args=None):
    rclpy.init(args=args)
    node = ProblematicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Create a Node with Configuration Issues

Create a node with parameter issues at `debugging_ex/debugging_ex/config_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ConfigNode(Node):
    def __init__(self):
        super().__init__('config_node')
        
        # Declare parameters with problematic default values
        self.declare_parameter('update_rate', -1.0)  # Negative rate is invalid
        self.declare_parameter('multiplier', 'not_a_number')  # Wrong type
        self.declare_parameter('topic_name', '/invalid/topic/name/with/slashes')
        
        # Get parameters (some will be invalid)
        self.update_rate = self.get_parameter('update_rate').value
        self.multiplier = self.get_parameter('multiplier').value
        self.topic_name = self.get_parameter('topic_name').value
        
        # Check for issues and warn
        if self.update_rate <= 0:
            self.get_logger().warn(f'Invalid update rate: {self.update_rate}. Using 1.0 instead.')
            self.update_rate = 1.0
        
        # Try to convert multiplier to float
        try:
            self.multiplier = float(self.multiplier)
        except (ValueError, TypeError):
            self.get_logger().error(f'Cannot convert multiplier to float: {self.multiplier}. Using 1.0.')
            self.multiplier = 1.0
        
        if self.topic_name.startswith('/'):
            self.get_logger().warn(f'Topic name should not start with /: {self.topic_name}')
        
        # Create publisher with potentially invalid topic
        self.publisher = self.create_publisher(Float64, self.topic_name, 10)
        
        # Timer based on potentially invalid rate
        self.timer = self.create_timer(1.0 / self.update_rate, self.pub_callback)
        
        self.counter = 0
        self.get_logger().info(f'Config node initialized with update_rate={self.update_rate}, multiplier={self.multiplier}, topic={self.topic_name}')

    def pub_callback(self):
        msg = Float64()
        msg.data = float(self.counter) * self.multiplier
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published: {msg.data}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ConfigNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Update Package Configuration

Update the `setup.py` file in the debugging_ex package:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'debugging_ex'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Examples of debugging in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'problematic_node = debugging_ex.problematic_node:main',
            'config_node = debugging_ex.config_node:main',
        ],
    },
)
```

### 5. Build and Run the Problematic Nodes

1. Navigate back to the workspace root:
   ```bash
   cd ~/debug_ws
   ```

2. Build the debugging package:
   ```bash
   colcon build --packages-select debugging_ex
   ```

3. Source the newly built package:
   ```bash
   source install/setup.bash
   ```

4. Run the problematic node in one terminal:
   ```bash
   ros2 run debugging_ex problematic_node
   ```

### 6. Use Debugging Tools to Identify Issues

1. In a new terminal, with the environment sourced, check for issues:

   List all running nodes:
   ```bash
   ros2 node list
   ```

   Check topics:
   ```bash
   ros2 topic list
   ros2 topic info /problematic_topic
   ```

   Monitor the topic:
   ```bash
   ros2 topic echo /problematic_topic
   ```

2. Use rqt tools to visualize the system:
   ```bash
   rqt
   ```
   Or use specific rqt plugins:
   ```bash
   rqt_graph  # Shows node connections
   rqt_console  # Shows log messages
   ```

3. Check system resources to identify the memory leak:
   ```bash
   # In a new terminal, monitor the process
   htop  # Or use top and look for the ROS 2 process
   ```

### 7. Fix the Issues Identified

1. Create a corrected version of the problematic node at `debugging_ex/debugging_ex/corrected_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64  # Different topic for feedback
import time

class CorrectedNode(Node):
    def __init__(self):
        super().__init__('corrected_node')
        
        # Create publisher for original topic
        self.publisher = self.create_publisher(String, 'corrected_topic', 10)
        
        # Create subscription for a different topic to avoid feedback
        self.subscription = self.create_subscription(
            Int64,  # Different message type
            'feedback_topic',  # Different topic name
            self.feedback_callback,
            10
        )
        
        # Reasonable timer frequency (10 Hz instead of 1000 Hz)
        self.timer = self.create_timer(0.1, self.corrected_timer)
        
        # Properly managed buffer with size limit
        self.data_buffer = []
        self.max_buffer_size = 100
        
        self.counter = 0
        self.get_logger().info('Corrected node initialized without issues')

    def feedback_callback(self, msg):
        # Process external feedback (not from our own publications)
        self.get_logger().info(f'Received external feedback: {msg.data}')
        # Add to buffer with size management
        self.data_buffer.append(f'Feedback data: {msg.data}')
        
        # Manage buffer size to prevent memory leak
        if len(self.data_buffer) > self.max_buffer_size:
            self.data_buffer.pop(0)

    def corrected_timer(self):
        # Publish at reasonable frequency
        msg = String()
        msg.data = f'Reasonable frequency message {self.counter}'
        self.publisher.publish(msg)
        
        # Add to buffer with size management
        self.data_buffer.append(msg.data)
        
        # Manage buffer size to prevent memory leak
        if len(self.data_buffer) > self.max_buffer_size:
            self.data_buffer.pop(0)
        
        self.counter += 1
        if self.counter % 10 == 0:  # Less frequent logging
            self.get_logger().info(f'Published {self.counter} messages, buffer size: {len(self.data_buffer)}')


def main(args=None):
    rclpy.init(args=args)
    node = CorrectedNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 8. Use Quality of Service (QoS) Debugging

Create a QoS debugging example at `debugging_ex/debugging_ex/qos_node.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

class QoSDebugNode(Node):
    def __init__(self):
        super().__init__('qos_debug_node')
        
        # Create a publisher with incompatible QoS settings for debugging
        # (This will cause issues when communicating with nodes expecting reliable delivery)
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # This may cause issues
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.publisher = self.create_publisher(String, 'qos_test_topic', qos_profile)
        
        # Create timer to publish messages
        self.timer = self.create_timer(1.0, self.qos_timer_callback)
        self.counter = 0
        
        self.get_logger().info('QoS debug node initialized')

    def qos_timer_callback(self):
        msg = String()
        msg.data = f'QoS test message {self.counter}'
        self.publisher.publish(msg)
        self.counter += 1
        self.get_logger().info(f'Published QoS message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = QoSDebugNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 9. Create a Launch File with Multiple Nodes for Debugging

Create the launch file at `debugging_ex/launch/debugging_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for debugging examples."""
    
    # Declare launch arguments
    debug_level = LaunchConfiguration('debug_level')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'debug_level',
            default_value='info',
            description='Logging level for the nodes'
        ),
        
        # Corrected node
        Node(
            package='debugging_ex',
            executable='corrected_node',
            name='corrected_node',
            parameters=[],
            output='screen',
            arguments=['--ros-args', '--log-level', debug_level]
        ),
        
        # Node to test QoS issues
        Node(
            package='debugging_ex',
            executable='qos_node',
            name='qos_debug_node',
            output='screen'
        )
    ])
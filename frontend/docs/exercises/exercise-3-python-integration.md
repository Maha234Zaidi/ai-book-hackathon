---
title: Exercise 3 - Python Integration with rclpy
description: Hands-on exercise for integrating Python with ROS 2 using rclpy
sidebar_position: 3
---

# Exercise 3: Python Integration with rclpy

## Objective

- Understand how to create ROS 2 nodes using Python and rclpy
- Learn to work with parameters in ROS 2 nodes
- Implement advanced patterns like timers and service clients
- Create a launch file to run multiple nodes together

## Prerequisites

- Completed previous exercises (Environment Setup, Nodes and Topics, Services)
- Python 3.10 or newer
- Understanding of basic ROS 2 concepts
- Basic Python programming skills

## Setup Requirements

- ROS 2 Humble Hawksbill installed and sourced
- Python development environment
- Basic ROS 2 workspace setup

## Steps

### 1. Review the Code Examples

First, review the following Python examples that demonstrate advanced rclpy usage:

1. **Parameter Node**: Demonstrates how to declare and use parameters in a ROS 2 node
2. **Timer Node**: Shows how to use timers and callback groups
3. **Complex Agent**: A robot agent with multiple behaviors

### 2. Create a New Package for Python Integration

1. Navigate to your workspace source directory:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new package for Python integration examples:
   ```bash
   ros2 pkg create --build-type ament_python python_integration_ex --dependencies rclpy std_msgs geometry_msgs sensor_msgs example_interfaces
   ```

### 3. Create the Parameter Node

Create the parameter node at `python_integration_ex/python_integration_ex/parameter_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('publish_frequency', 1.0)
        
        # Access parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        
        # Create timer based on parameter frequency
        self.timer = self.create_timer(
            1.0 / self.publish_frequency, 
            self.status_callback
        )
        
        self.get_logger().info(
            f'ParameterNode initialized with: '
            f'robot_name={self.robot_name}, '
            f'max_velocity={self.max_velocity}, '
            f'safety_distance={self.safety_distance}, '
            f'publish_frequency={self.publish_frequency}'
        )

    def status_callback(self):
        msg = String()
        msg.data = f'{self.robot_name} - Velocity: {self.max_velocity}, Distance: {self.safety_distance}'
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published status: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    
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

### 4. Create the Timer Node with Callback Groups

Create the timer node at `python_integration_ex/python_integration_ex/timer_node.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

class TimerCallbackNode(Node):
    def __init__(self):
        super().__init__('timer_callback_node')
        
        # Create a mutually exclusive callback group
        cb_group = MutuallyExclusiveCallbackGroup()
        
        # Create a timer that publishes a message periodically
        self.timer_counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Create a service server in the same callback group
        self.srv = self.create_service(
            AddTwoInts, 
            'calculate_velocity', 
            self.service_callback
        )
        
        # Create publisher for timer messages
        self.timer_publisher = self.create_publisher(String, 'timer_messages', 10)
        
        self.get_logger().info('Timer and Callback Group node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = f'Timer callback executed {self.timer_counter} times'
        self.timer_publisher.publish(msg)
        self.timer_counter += 1
        self.get_logger().info(f'Timer: {msg.data}')

    def service_callback(self, request, response):
        # Simulate a calculation based on request values
        velocity = (request.a + request.b) / 10.0
        response.sum = int(velocity * 10)  # Convert back to int
        self.get_logger().info(f'Calculated velocity: {velocity} m/s')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TimerCallbackNode()
    
    # Use a multi-threaded executor to handle both timer and service callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5. Update Package Configuration

Update the `setup.py` file in the python_integration_ex package:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'python_integration_ex'

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
    description='Examples of Python integration with ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'param_node = python_integration_ex.parameter_node:main',
            'timer_node = python_integration_ex.timer_node:main',
        ],
    },
)
```

### 6. Build and Run the Nodes

1. Navigate back to the workspace root:
   ```bash
   cd ~/ros2_ws
   ```

2. Build the package:
   ```bash
   colcon build --packages-select python_integration_ex
   ```

3. Source the newly built package:
   ```bash
   source install/setup.bash
   ```

4. Run the parameter node with custom parameters:
   ```bash
   ros2 run python_integration_ex param_node --ros-args -p robot_name:=my_robot -p max_velocity:=1.0
   ```

5. In another terminal, run the timer node:
   ```bash
   ros2 run python_integration_ex timer_node
   ```

6. In a third terminal, test the service provided by the timer node:
   ```bash
   ros2 service call /calculate_velocity example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
   ```

### 7. Create and Use a Launch File

1. Create a launch directory in your package:
   ```bash
   mkdir -p python_integration_ex/launch
   ```

2. Create the launch file at `python_integration_ex/launch/python_integration_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for Python integration examples."""
    
    # Declare launch arguments
    robot_name = LaunchConfiguration('robot_name')
    max_velocity = LaunchConfiguration('max_velocity')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'robot_name',
            default_value='turtlebot',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'max_velocity',
            default_value='0.5',
            description='Maximum velocity for the robot'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Parameter node with custom parameters
        Node(
            package='python_integration_ex',
            executable='param_node',
            name='parameter_example_node',
            parameters=[
                {'robot_name': robot_name},
                {'max_velocity': max_velocity},
                {'safety_distance': 0.5},
                {'publish_frequency': 2.0}
            ],
            output='screen'
        ),
        
        # Timer node
        Node(
            package='python_integration_ex',
            executable='timer_node',
            name='timer_example_node',
            output='screen'
        )
    ])
```

3. Build again to include the launch file:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select python_integration_ex
   source install/setup.bash
   ```

4. Run the launch file:
   ```bash
   ros2 launch python_integration_ex python_integration_launch.py robot_name:=launch_robot max_velocity:=0.8
   ```

## Expected Result

- Parameter node runs with specified parameters and publishes status messages
- Timer node runs with timer and service functionality
- Service call returns calculated values
- Launch file successfully starts both nodes with specified parameters

## Verification

1. Check that nodes are running:
   ```bash
   ros2 node list
   ```
   You should see the nodes from your package.

2. Check parameters of the parameter node:
   ```bash
   ros2 param list
   ros2 param get /parameter_example_node robot_name
   ```

3. Verify topics are being published:
   ```bash
   ros2 topic list
   ros2 topic echo /robot_status
   ```

4. Verify service is available:
   ```bash
   ros2 service list
   ros2 service call /calculate_velocity example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"
   ```

## Troubleshooting

- **Issue**: Parameter node doesn't accept parameters from command line
  - **Solution**: Ensure you're using the correct parameter syntax with `--ros-args -p`

- **Issue**: Launch file doesn't run
  - **Solution**: Check that the launch file is in the correct location and properly formatted

- **Issue**: Service call fails
  - **Solution**: Ensure the service node is running before making the call

- **Issue**: Nodes crash or don't behave as expected
  - **Solution**: Check the logs for error messages

## Learning Outcomes

- Understanding of parameter declaration and usage in ROS 2 nodes
- Knowledge of callback groups and executors
- Ability to create and use launch files
- Experience with complex node behaviors

## Further Exploration

1. Add more parameters to control different aspects of the robot behavior
2. Implement a subscriber in one of the nodes to react to messages
3. Create a custom service definition and use it in your nodes
4. Add error handling and logging in multiple places
5. Experiment with different QoS settings for your publishers and subscribers
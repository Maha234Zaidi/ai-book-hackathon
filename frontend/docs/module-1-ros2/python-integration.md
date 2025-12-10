---
title: Python Integration with rclpy
description: Using Python to interface with ROS 2 systems
sidebar_position: 3
---

# Python Integration with rclpy

## Learning Objectives

- Understand the rclpy client library for Python
- Create ROS 2 nodes in Python with advanced features
- Manage parameters in ROS 2 nodes using Python
- Implement advanced patterns for Python-based ROS 2 applications
- Create launch files and use them with Python nodes

## Introduction to rclpy

The Robot Operating System 2 (ROS 2) provides client libraries that enable developers to write ROS 2 code in different programming languages. For Python, ROS 2 offers rclpy, which is a Python client library that provides Python APIs for ROS 2 concepts.

### Why Python with ROS 2?

Python is a popular choice for robotics development due to its:
- Simple syntax that allows for rapid prototyping
- Rich ecosystem of libraries for machine learning and AI
- Ease of use for scripting and automation tasks
- Strong community support

## Getting Started with rclpy

### Installation

rclpy is part of the ROS 2 installation, but you need to ensure you have the necessary packages installed:

```bash
# Install rclpy along with other Python ROS 2 packages
sudo apt install python3-ros-interfaces  # On Ubuntu
# Or use pip to install directly
pip3 install rclpy
```

### Basic Node Structure

A minimal ROS 2 node in Python with rclpy follows this structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize your node components here
        
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Node Creation with rclpy

### Using Timers

Timers are useful for executing code at regular intervals:

```python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback: {self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Using Callback Groups

Callback groups allow you to manage how callbacks are executed:

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class CallbackGroupNode(Node):
    def __init__(self):
        super().__init__('callback_group_node')
        
        # Create a mutually exclusive callback group
        cb_group = MutuallyExclusiveCallbackGroup()
        
        # Create a timer that uses the callback group
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=cb_group)
        
        # Create a service that also uses the same callback group
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.service_callback, 
            callback_group=cb_group
        )

    def timer_callback(self):
        self.get_logger().info('Timer callback executing')

    def service_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CallbackGroupNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameter Management

### Defining Parameters

Nodes can have parameters that can be configured at runtime:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_string_param', 'default_value')
        self.declare_parameter('my_int_param', 42)
        self.declare_parameter('my_double_param', 3.14)
        self.declare_parameter('my_bool_param', True)
        
        # Access parameter values
        self.string_param = self.get_parameter('my_string_param').value
        self.int_param = self.get_parameter('my_int_param').value
        self.double_param = self.get_parameter('my_double_param').value
        self.bool_param = self.get_parameter('my_bool_param').value

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameter Callbacks

You can also set up callbacks to respond when parameters change:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterCallbackNode(Node):
    def __init__(self):
        super().__init__('parameter_callback_node')
        
        # Declare parameters
        self.declare_parameter('my_parameter', 'default_value')
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'my_parameter' and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterCallbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Patterns and Techniques

### Using Actions

Actions are another communication pattern that allow for goal-oriented, long-running tasks:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time

# Assuming you have an action definition
# from my_package.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Use actual action type
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        # Accept or reject a goal
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accept or reject a cancel request
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Simulate execution
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            self.get_logger().info(f'Sending feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        
        self.get_logger().info(f'Result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    
    action_server = FibonacciActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    
    try:
        executor.spin()
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Lifecycle Nodes

Lifecycle nodes provide a structured way to manage the state of a node:

```python
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor

class LifecycleNodeExample(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_node_example')

    def on_configure(self, state):
        self.get_logger().info(f'Configuring node: {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info(f'Activating node: {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info(f'Deactivating node: {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info(f'Cleaning up node: {state.label}')
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNodeExample()
    
    # Spin the node to process events
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Debugging

### Exception Handling in Callbacks

Always handle potential exceptions in your ROS 2 node callbacks:

```python
import rclpy
from rclpy.node import Node
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.error_count = 0
        
    def robust_callback(self, msg):
        try:
            # Your processing logic here
            processed_data = self.process_message(msg)
            self.publish_result(processed_data)
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Error in callback: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            # Implement appropriate error recovery strategy
</`

### Debugging Techniques

- Use `rclpy.logging` to add informative log messages
- Monitor topics with `ros2 topic echo`
- Check node status with `ros2 node list` and `ros2 node info`
- Use `rclpy.create_timer` to add health checks and monitoring

## Launch Files

Launch files allow you to start multiple nodes and configure them at once:

Example launch file (`my_launch_file.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='minimal_publisher',
            name='publisher_node',
            parameters=[
                {'my_param': 'value'},
                {'another_param': 42}
            ]
        ),
        Node(
            package='my_package',
            executable='minimal_subscriber',
            name='subscriber_node'
        )
    ])
```

To run the launch file:
```bash
ros2 launch my_package my_launch_file.py
```

## Best Practices for rclpy

1. Always call `rclpy.init()` before creating nodes and `rclpy.shutdown()` when done
2. Use `rclpy.spin()` to keep nodes running and processing callbacks
3. Implement proper error handling and logging in all callbacks
4. Use appropriate callback groups to manage execution order
5. Leverage parameters for configuration rather than hardcoded values
6. Use launch files to manage complex multi-node systems
7. Follow Python best practices like PEP 8 for code style

## Summary

This section explored the integration of Python with ROS 2 using the rclpy client library. You learned about creating advanced nodes with timers and callback groups, managing parameters, implementing actions and lifecycle nodes, and best practices for robust Python-based ROS 2 development.

The next section will cover URDF (Unified Robot Description Format), which is used to describe robot models in ROS 2.
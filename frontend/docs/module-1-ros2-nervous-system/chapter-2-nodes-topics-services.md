---
title: ROS 2 Nodes, Topics & Services
sidebar_position: 3
---

# ROS 2 Nodes, Topics & Services

## Overview
This chapter delves into the practical aspects of creating and managing distributed robot components in ROS 2. We'll explore how to create nodes in Python using rclpy, implement publisher/subscriber patterns, and contrast services with topics for different communication needs. The focus will be on practical examples for humanoid control scenarios.

## Learning Objectives
After completing this chapter, students will be able to:
- Create and run basic ROS 2 nodes using Python and rclpy
- Implement publisher and subscriber patterns in their own nodes
- Contrast services vs. topics and choose the appropriate communication method
- Understand practical applications of these patterns for humanoid robots

## Creating Nodes in Python with rclpy

### Understanding rclpy
rclpy is the Python library for ROS 2 that allows Python programs to interact with ROS 2. It provides the standard API for most ROS concepts including nodes, publishers, subscribers, services, and parameters. rclpy serves as a Python wrapper around the ROS Client Library (rcl), providing Python developers with access to ROS 2's powerful features [ROS 2 Python Client Library, 2023].

### Basic Node Structure
Every ROS 2 node created with rclpy follows a similar structure:

1. Import necessary rclpy modules and message types
2. Create a class that inherits from `rclpy.node.Node`
3. Initialize the node in the class constructor
4. Set up publishers, subscribers, services, or clients
5. Implement callback functions for subscriptions and services
6. Use rclpy.spin() to keep the node running

### Example: Basic Node Creation
Here's a minimal example of creating a node:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    
    node = MinimalNode()
    
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

### Node Lifecycle
When creating a ROS 2 node, it's important to understand its lifecycle:

1. **Initialization**: The node is created and registered with the ROS graph
2. **Active**: The node runs and can communicate with other nodes
3. **Shutdown**: The node is properly destroyed and unregistered

The `destroy_node()` method should always be called to properly clean up resources.

## Publisher/Subscriber Patterns

### Understanding Topics and Messages
In ROS 2, topics are named buses over which nodes exchange messages. A message is a simple data structure that consists of a set of typed fields. Messages are defined in special definition files and are used for communication between nodes [ROS 2 Message Types, 2023].

### Publisher Implementation
A publisher node creates and sends messages to a topic. Here's how to implement a publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1
```

### Subscriber Implementation
A subscriber node receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### Publisher/Subscriber Communication Pattern
The publisher-subscriber pattern works as follows:

1. A publisher node sends messages to a topic by calling `publish()`
2. The DDS infrastructure handles message delivery to all subscribers
3. Subscribers receive messages in their callback functions
4. Communication is asynchronous and decoupled

This pattern is ideal for sensor data, system status, and other continuous information streams.

### Complete Publisher Node Example
Here's a more complete example of a publisher node that could be used in a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        self.i = 0

    def publish_joint_states(self):
        # Create message
        msg = JointState()
        msg.name = ['head_pan_joint', 'head_tilt_joint', 'left_elbow_joint', 'right_elbow_joint']

        # Example: simple oscillating motion
        head_pan_pos = 0.5 * math.sin(self.i * 0.1)
        head_tilt_pos = 0.3 * math.sin(self.i * 0.08)
        left_elbow_pos = -1.0 + 0.2 * math.sin(self.i * 0.12)
        right_elbow_pos = -1.0 + 0.2 * math.cos(self.i * 0.12)

        msg.position = [head_pan_pos, head_tilt_pos, left_elbow_pos, right_elbow_pos]
        msg.velocity = [0.0] * len(msg.position)  # zero velocity for simplicity
        msg.effort = [0.0] * len(msg.position)    # zero effort for simplicity

        # Publish message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint states: Head Pan={head_pan_pos:.2f}, Left Elbow={left_elbow_pos:.2f}')
        self.i += 1
```

### Complete Subscriber Node Example
And here's a more complete example of a subscriber node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Publisher for safety status
        self.safety_publisher = self.create_publisher(Bool, 'safety_status', 10)
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        # Process joint states
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')

        # Check for safety conditions (example: if elbow is too bent)
        for i, joint_name in enumerate(msg.name):
            if 'elbow' in joint_name and abs(msg.position[i]) > 2.5:
                self.get_logger().warn(f'Safety issue: {joint_name} position {msg.position[i]} exceeds safe limit!')

                # Publish safety alert
                safety_msg = Bool()
                safety_msg.data = True
                self.safety_publisher.publish(safety_msg)
                break
        else:
            # Publish safety clear if no issues
            safety_msg = Bool()
            safety_msg.data = False
            self.safety_publisher.publish(safety_msg)
```

## Services vs. Topics: Request-Response vs. Streaming

### Understanding Services
While topics are great for continuous data streams, services are designed for request-response interactions. A service has a specific request message type and a specific response message type, and clients request services to send a request message and get a response message in return [ROS 2 Services, 2023].

### Service Implementation
Here's how to implement a service server:

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

### Client Implementation
A client calls the service:

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future
```

### Comparing Communication Patterns

| Aspect | Topics (Publish/Subscribe) | Services (Request/Response) |
|--------|-----------------------------|------------------------------|
| **Communication Type** | Asynchronous, one-way | Synchronous, two-way |
| **Use Case** | Continuous data (sensors, status) | Specific requests (calculations, commands) |
| **Coupling** | Decoupled (publisher doesn't know subscribers) | More coupled (client knows server) |
| **Reliability** | Best effort or reliable | Reliable (request always gets response) |
| **Timing** | Data-driven | Event-driven |

## Practical Examples for Humanoid Control

### Joint Control Publisher
For humanoid control, a node might publish joint position commands:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_commands', 10)
        # Command robot joints at 10Hz
        self.timer = self.create_timer(0.1, self.publish_joint_commands)

    def publish_joint_commands(self):
        msg = JointState()
        msg.name = ['head_pan_joint', 'head_tilt_joint', 'left_elbow_joint']
        msg.position = [0.5, 0.3, -1.2]  # radians
        self.publisher.publish(msg)
```

### Sensor Data Subscriber
A node might subscribe to sensor data to make decisions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

    def imu_callback(self, msg):
        # Check if robot is tilting too much
        if abs(msg.linear_acceleration.x) > 9.0:
            self.get_logger().warn('Robot is tilting too much!')
            # Implement recovery behavior
```

### Service for Complex Actions
For complex humanoid actions that require parameters and return results:

```python
# Example service interface: GetJointRange.srv
# float64 joint_position
# ---
# bool in_safe_range
# float64 min_position
# float64 max_position

from your_interfaces.srv import GetJointRange
import rclpy
from rclpy.node import Node

class JointSafetyService(Node):
    def __init__(self):
        super().__init__('joint_safety_service')
        self.srv = self.create_service(
            GetJointRange,
            'check_joint_safety',
            self.safety_check_callback)

    def safety_check_callback(self, request, response):
        # Check if joint position is within safe range
        response.in_safe_range = -2.0 <= request.joint_position <= 2.0
        response.min_position = -2.0
        response.max_position = 2.0
        return response
```

## Communication Flow Diagrams

### Publisher-Subscriber Communication Flow

```
                    Publisher-Subscriber Pattern
                    ============================

    Publisher Node                  DDS Layer                  Subscriber Node(s)
    ==============                  =========                  ================

    1. Creates publisher            Handles message            1. Creates subscription
       for topic "joint_states"    routing, QoS,              for topic "joint_states"
       using create_publisher()    discovery                  using create_subscription()

    2. Creates message              3. Message routed         2. Message delivered to
       with JointState data            to subscribers             callback function
       using msg = JointState()        by DDS infrastructure      using listener_callback()

    3. Publishes message            4. Multiple subscribers    3. Each subscriber
       using publisher.publish()       can receive same           processes message
                                     message simultaneously     independently
```

### Service-Client Communication Flow

```
                    Service-Client Pattern
                    ======================

    Client Node                    Service Server Node          DDS Layer
    ===========                    ===============          ===========

    1. Creates client              1. Creates service         1. Handles service
       for "add_two_ints"             using create_service()      discovery and
       using create_client()                                    request routing

    2. Calls service with          2. Service callback        2. Routes request
       request data (a=2, b=3)      receives request           to service server
       using call_async()           and processes it

    3. Waits for response         3. Service returns         3. Routes response
       containing result             processed result            to client node
```

## Detailed Communication Pattern Comparison

| Aspect | Publisher-Subscriber | Service-Client | Action-Client/Server |
|--------|----------------------|----------------|----------------------|
| **Communication Type** | Asynchronous, one-way | Synchronous, two-way | Asynchronous with feedback |
| **Data Flow** | Unidirectional (pub→sub) | Bidirectional (req↔res) | Bidirectional (goal+feedback↔result) |
| **Timing** | Continuous/periodic | On-demand | Long-running with updates |
| **Coupling** | Loose (pub doesn't know subs) | Moderate (client knows server) | Moderate (client knows server) |
| **Reliability** | Best effort or reliable | Reliable | Reliable with feedback |
| **Use Case** | Sensor data, status updates | Calculations, validations | Complex tasks with progress |
| **Humanoid Example** | Joint states, IMU data | Inverse kinematics calc | Walking pattern execution |
| **ROS Message** | Single message type | Request + Response types | Goal + Feedback + Result |
| **Cancelation** | No | No | Yes |
| **Feedback** | No | No | Continuous feedback |

## Summary
This chapter covered the essential ROS 2 communication patterns: nodes with their publishers and subscribers for asynchronous data streaming, and services for synchronous request-response interactions. We explored practical implementations in Python using rclpy and examined how these patterns apply to humanoid robot control scenarios. Understanding when to use topics versus services is crucial for designing effective robotic systems.

Recalling concepts from [Chapter 1](./chapter-1-ros2-intro.md), we've now implemented the publisher-subscriber and service patterns that form the core of the "robotic nervous system" architecture we discussed. The DDS infrastructure we learned about provides the underlying communication layer that enables these patterns to work reliably.

In the next chapter, we'll explore how AI agents can interface with these ROS 2 components to control humanoid robots.

## Key Terms
- **Node**: An instance of a computational process that uses ROS for communication
- **Topic**: A named bus over which messages are sent
- **Publisher**: A node that sends messages to topics
- **Subscriber**: A node that receives messages from topics
- **Service**: A communication pattern that allows nodes to request information or actions from other nodes
- **Client**: A node that makes requests to a service
- **Message**: A data structure passed between nodes
- **rclpy**: The Python library for ROS 2

## Further Reading
- ROS 2 Tutorials: Writing a Simple Publisher and Subscriber [ROS 2 Tutorials, 2023]
- ROS 2 Tutorials: Writing a Simple Service and Client [ROS 2 Tutorials, 2023]
- Understanding ROS 2 QoS Settings for Real-time Performance [ROS 2 QoS Guide, 2023]

## Exercises (Optional)
1. Design a node structure for a walking humanoid robot that includes at least 3 publishers, 3 subscribers, and 1 service.
2. Identify scenarios where you would use a service instead of a topic for humanoid control.
3. Create a conceptual message definition for humanoid joint commands that includes position, velocity, and effort for multiple joints.
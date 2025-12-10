---
title: Nodes, Topics, and Services
description: Core communication mechanisms in ROS 2
sidebar_position: 2
---

# Nodes, Topics, and Services

## Learning Objectives

- Understand the concept of nodes in ROS 2
- Learn how topics enable asynchronous communication
- Master the use of services for synchronous communication
- Implement publisher and subscriber patterns
- Implement client and server service patterns

## Introduction to ROS 2 Communication

In ROS 2, communication between different parts of your robot system happens through a set of core mechanisms: nodes, topics, and services. Understanding these concepts is essential for creating any complex robotic system that involves multiple interacting components.

## Nodes

### What is a Node?

A node is an executable process that works as part of a ROS 2 system. Nodes are the fundamental building blocks of ROS 2 applications. Each node performs a specific function within the larger system and communicates with other nodes to achieve complex robotic behaviors.

### Creating Nodes

In ROS 2, nodes are implemented as classes that inherit from the rclpy.Node base class. A minimal node implementation includes:

1. Inheritance from rclpy.Node
2. Initialization of the node with a name
3. Proper shutdown handling

### Node Lifecycle

Nodes go through different states in their lifecycle:
- Unconfigured → Inactive → Active
- Active → Inactive → Unconfigured → Finalized

This lifecycle management allows for more robust robot systems with clear state transitions.

## Topics and Message Passing

### Publish-Subscribe Pattern

Topics implement the publish-subscribe pattern in ROS 2. Publishers send messages to topics, and subscribers receive messages from topics. This enables asynchronous, decoupled communication between nodes.

### Topics Characteristics

- Unidirectional data flow from publisher to subscriber
- Multiple publishers and subscribers can communicate on the same topic
- No direct relationship between publisher and subscriber
- Quality of Service (QoS) settings allow for fine-tuning communication behavior

### Message Types

Messages are the data structures sent over topics. ROS 2 provides standard message types in packages like std_msgs, sensor_msgs, and geometry_msgs. Users can also define custom message types.

## Services

### Request-Response Pattern

Unlike topics which provide asynchronous communication, services implement a synchronous request-response pattern. A service client sends a request to a service server, which processes the request and returns a response.

### Service Characteristics

- Synchronous communication model
- Request must be processed before response is returned
- Direct relationship between client and server
- Useful for operations that require confirmation or computation

### Service Types

Like messages, services have defined types that specify the structure of the request and response. ROS 2 provides standard services, but custom service types can also be defined.

## Practical Examples

### Publisher Example

Here's a Python example implementing a simple publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

Here's a corresponding subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Implementation

Here's an example of a service implementation:

Service definition file (`AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

Service server:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Service client:
```python
from example_interfaces.srv import AddTwoInts
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
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info('Result of add_two_ints: %d' % response.sum)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS)

Quality of Service settings allow you to fine-tune the behavior of topic communication. You can control properties like:

- Reliability: Best effort or reliable
- Durability: Volatile or transient local
- History: Keep last N messages or keep all
- Deadline: Maximum time between messages
- Lifespan: How long messages persist after publication

These settings help ensure your communication patterns meet the needs of your robotic system.

## Best Practices for Communication

1. Use appropriate naming conventions for nodes, topics, and services
2. Choose appropriate QoS settings for your communication needs
3. Implement proper error handling in all communication patterns
4. Use appropriate message types for your data
5. Separate publisher/subscriber logic when possible for maintainability

## Summary

This section covered the fundamental communication mechanisms in ROS 2: nodes, topics, and services. You learned about the publish-subscribe pattern for asynchronous communication and the request-response pattern for synchronous communication. You also saw practical examples of implementing these patterns in Python using rclpy.

In the next section, we'll explore how to integrate Python programming with ROS 2 using the rclpy library in greater detail.
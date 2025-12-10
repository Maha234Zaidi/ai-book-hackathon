---
title: Exercise 2 - Nodes and Topics
description: Hands-on exercise for working with ROS 2 nodes and topics
sidebar_position: 2
---

# Exercise 2: Nodes and Topics

## Objective

- Understand the fundamental concepts of nodes and topics in ROS 2
- Create a publisher node that publishes messages to a topic
- Create a subscriber node that receives messages from a topic
- Run multiple nodes and observe communication between them

## Prerequisites

- Completed Exercise 1 (ROS 2 Environment Setup)
- Basic Python programming knowledge
- Understanding of ROS 2 concepts from the introduction section

## Setup Requirements

- ROS 2 Humble Hawksbill installed and sourced
- Python 3.10 or newer
- Basic ROS 2 workspace setup

## Steps

### 1. Create a New Package

1. Create a new workspace directory if you don't have one:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. Create a new package for this exercise:
   ```bash
   cd src
   ros2 pkg create --build-type ament_python nodes_topics_ex --dependencies rclpy std_msgs
   ```

### 2. Create the Publisher Node

Create the publisher script at `nodes_topics_ex/nodes_topics_ex/publisher_member_function.py`:

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

### 3. Create the Subscriber Node

Create the subscriber script at `nodes_topics_ex/nodes_topics_ex/subscriber_member_function.py`:

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

### 4. Update Package Configuration

Update the `setup.py` file in the package root to include entry points:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'nodes_topics_ex'

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
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = nodes_topics_ex.publisher_member_function:main',
            'listener = nodes_topics_ex.subscriber_member_function:main',
        ],
    },
)
```

### 5. Build the Package

1. Navigate back to the workspace root:
   ```bash
   cd ~/ros2_ws
   ```

2. Build the package:
   ```bash
   colcon build --packages-select nodes_topics_ex
   ```

3. Source the newly built package:
   ```bash
   source install/setup.bash
   ```

### 6. Run the Publisher and Subscriber

1. Open a new terminal, navigate to your workspace, and source the setup:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ```

2. Run the publisher node:
   ```bash
   ros2 run nodes_topics_ex talker
   ```

3. In another terminal, source the setup and run the subscriber:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 run nodes_topics_ex listener
   ```

4. Observe the communication between the publisher and subscriber nodes.

### 7. Explore Topic Commands

In a third terminal (with the environment sourced), explore the following commands:

1. List active topics:
   ```bash
   ros2 topic list
   ```

2. Get information about the topic:
   ```bash
   ros2 topic info /topic
   ```

3. Echo messages from the topic without running the subscriber:
   ```bash
   ros2 topic echo /topic
   ```

4. Publish to the topic directly from the command line:
   ```bash
   ros2 topic pub /topic std_msgs/msg/String "data: 'Hello from command line'"
   ```

## Expected Result

- Publisher terminal displays "Publishing: 'Hello World: X'" messages approximately every 0.5 seconds
- Subscriber terminal displays "I heard: 'Hello World: X'" messages at the same rate
- Messages sent by the publisher are received by the subscriber
- Topic commands work as expected

## Verification

1. Check that both nodes are running:
   ```bash
   ros2 node list
   ```
   You should see `minimal_publisher` and `minimal_subscriber`.

2. Verify the topic exists and has the correct type:
   ```bash
   ros2 topic list -t
   ```
   You should see `/topic` with type `std_msgs/msg/String`.

3. Verify messages are being published:
   ```bash
   ros2 topic hz /topic
   ```
   You should see a message rate around 2 Hz (since publishing every 0.5 seconds).

## Troubleshooting

- **Issue**: Publisher/subscriber terminals show "command not found"
  - **Solution**: Ensure you've sourced the workspace setup: `source install/setup.bash`

- **Issue**: Nodes start but don't communicate
  - **Solution**: Make sure both terminals have the ROS 2 environment and workspace sourced

- **Issue**: Package build fails
  - **Solution**: Check that package names and file paths match exactly between the code and setup.py

- **Issue**: Topic commands don't show the expected results
  - **Solution**: Make sure both nodes are running before executing topic commands

## Learning Outcomes

- Understanding of the basic publisher-subscriber pattern in ROS 2
- Ability to create and run simple ROS 2 nodes
- Familiarity with basic ROS 2 command-line tools
- Knowledge of how topics enable communication between nodes

## Further Exploration

1. Modify the message content in the publisher
2. Change the publishing frequency by modifying `timer_period`
3. Create a second subscriber node to the same topic
4. Add parameters to control the message content or publishing rate
5. Create a custom message type instead of using String
# Template for Hands-on Exercises

## Exercise Template

Use this template as a starting point for creating hands-on exercises for the ROS 2 module:

```markdown
---
title: Exercise [Number] - [Exercise Title]
description: [Brief summary of the exercise]
sidebar_position: [Number]
---

# Exercise: [Exercise Title]

## Objective

[What the student will learn/practice]
- [Learning objective 1]
- [Learning objective 2]

## Prerequisites

[What students need to know before starting]
- [Prerequisite 1]
- [Prerequisite 2]

## Setup Requirements

[What needs to be installed/configured before starting]
- [Setup requirement 1]
- [Setup requirement 2]

## Steps

1. [Step-by-step instruction]
   - Include code snippets where necessary
   - Include expected outputs at key steps
   
2. [Continue with additional steps]

3. [More steps as needed]

## Expected Result

[What success looks like]
- [Expected outcome 1]
- [Expected outcome 2]

## Verification

[How to verify the exercise was completed successfully]
- [Verification step 1]
- [Verification step 2]

## Troubleshooting

[Common issues and solutions]
- **Issue 1**: [Description of common issue]
  - **Solution**: [How to resolve the issue]
- **Issue 2**: [Description of common issue]  
  - **Solution**: [How to resolve the issue]

## Learning Outcomes

[What students should understand after completing this exercise]
- [Outcome 1]
- [Outcome 2]

## Further Exploration

[Optional challenges or extensions for advanced students]
- [Challenge 1]
- [Challenge 2]

## References

[Links to related content or external resources]
- [Related section in the module]
- [Official ROS 2 documentation]
```

## Example Exercise

Here's an example using the template:

```markdown
---
title: Exercise 1 - Basic Publisher and Subscriber
description: Create and run a simple publisher-subscriber pair in ROS 2
sidebar_position: 1
---

# Exercise 1: Basic Publisher and Subscriber

## Objective

- Understand the basic structure of ROS 2 nodes
- Create a publisher node that sends messages
- Create a subscriber node that receives messages
- Observe the communication between nodes

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Basic understanding of Python
- Basic command line knowledge

## Setup Requirements

- ROS 2 environment sourced in your terminal
- Python 3.10 or newer
- `rclpy` package installed (normally included with ROS 2)

## Steps

1. Create a new package for the exercise:
   ```bash
   mkdir -p ~/ros2_exercises/src
   cd ~/ros2_exercises/src
   ros2 pkg create --build-type ament_python publisher_subscriber_ex --dependencies rclpy std_msgs
   ```

2. Create the publisher script at `publisher_subscriber_ex/publisher_subscriber_ex/publisher.py`:
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

3. Create the subscriber script at `publisher_subscriber_ex/publisher_subscriber_ex/subscriber.py`:
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

4. Update the `setup.py` file to include entry points:
   ```python
   import os
   from glob import glob
   from setuptools import setup

   package_name = 'publisher_subscriber_ex'

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
       description='Basic publisher subscriber example',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'talker = publisher_subscriber_ex.publisher:main',
               'listener = publisher_subscriber_ex.subscriber:main',
           ],
       },
   )
   ```

5. Build the package:
   ```bash
   cd ~/ros2_exercises
   colcon build --packages-select publisher_subscriber_ex
   source install/setup.bash
   ```

6. Open two terminal windows and source the environment in both:
   ```bash
   cd ~/ros2_exercises
   source install/setup.bash
   ```

7. In the first terminal, run the publisher:
   ```bash
   ros2 run publisher_subscriber_ex talker
   ```

8. In the second terminal, run the subscriber:
   ```bash
   ros2 run publisher_subscriber_ex listener
   ```

## Expected Result

- The publisher terminal displays "Publishing: 'Hello World: X'" messages approximately every 0.5 seconds
- The subscriber terminal displays "I heard: 'Hello World: X'" messages at the same rate
- Messages sent by the publisher are received by the subscriber

## Verification

- Check that both nodes are running using `ros2 node list`
- Verify the topic is active using `ros2 topic list`
- Use `ros2 topic echo /topic` in another terminal to see the messages

## Troubleshooting

- **Issue**: Command 'ros2' not found
  - **Solution**: Ensure ROS 2 environment is sourced in your terminal
- **Issue**: Nodes don't communicate
  - **Solution**: Verify both terminals have the ROS 2 environment sourced and are on the same domain
- **Issue**: Package build fails
  - **Solution**: Verify all code has been entered correctly and check for Python syntax errors

## Learning Outcomes

- Understanding of the basic ROS 2 node structure
- Knowledge of publisher and subscriber pattern
- Experience with ROS 2 command-line tools

## Further Exploration

- Modify the message content in the publisher
- Add parameters to control the publishing rate
- Create multiple subscribers to the same topic
```

## Key Elements to Remember

When creating exercises, ensure you include:

1. **Clear objectives** that align with learning goals
2. **Appropriate prerequisites** to set student expectations
3. **Detailed, step-by-step instructions** with no missing steps
4. **Expected results** so students know when they've succeeded
5. **Troubleshooting** to help students overcome common issues
6. **Verification steps** to confirm completion
7. **Learning outcomes** to reinforce key concepts
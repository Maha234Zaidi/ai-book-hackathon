---
title: Exercise 2.2 - Services
description: Hands-on exercise for working with ROS 2 services
sidebar_position: 2.2
---

# Exercise 2.2: Services

## Objective

- Understand the concept of services in ROS 2
- Create a service server that responds to requests
- Create a service client that sends requests to the server
- Compare service communication with topic communication

## Prerequisites

- Completed Exercise 1 (ROS 2 Environment Setup)
- Completed Exercise 2 (Nodes and Topics)
- Understanding of ROS 2 concepts from the module sections

## Setup Requirements

- ROS 2 Humble Hawksbill installed and sourced
- Python 3.10 or newer
- Basic ROS 2 workspace setup

## Steps

### 1. Create the Service Definition

1. In your workspace (`~/ros2_ws/src/nodes_topics_ex`), create a directory for services:
   ```bash
   mkdir -p srv
   ```

2. Create the service definition file `srv/AddTwoInts.srv`:
   ```
   int64 a
   int64 b
   ---
   int64 sum
   ```

3. Update the `package.xml` file to include the service:
   ```xml
   <buildtool_depend>ament_python</buildtool_depend>
   
   <depend>rclpy</depend>
   <depend>std_msgs</depend>
   
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

4. Update the `setup.py` file to include the service:
   ```python
   from setuptools import setup
   from glob import glob
   import os
   
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
       description='Examples of publisher/subscriber/services using rclpy',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'talker = nodes_topics_ex.publisher_member_function:main',
               'listener = nodes_topics_ex.subscriber_member_function:main',
           ],
       },
       # Add the service definition
       packages=[package_name],
       package_dir={'': 'nodes_topics_ex'},
       package_data={package_name: ['srv/*.srv']},
   )
   ```

   Actually, we need to modify the setup.py to properly include services. Let me create a new package specifically for services:

### 1 (revised). Create a New Package for Services

1. Navigate to your workspace source directory:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new package for services with the proper dependencies:
   ```bash
   ros2 pkg create --build-type ament_python services_ex --dependencies rclpy std_msgs example_interfaces
   ```

3. Create the server script at `services_ex/services_ex/service_server.py`:

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
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Create the Service Client

Create the client script at `services_ex/services_ex/service_client.py`:

```python
from example_interfaces.srv import AddTwoInts
import sys
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
        return self.future

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    rclpy.spin_until_future_complete(minimal_client, future)

    response = future.result()
    minimal_client.get_logger().info(
        'Result of add_two_ints: %d' % response.sum)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Update Package Configuration for Services

Update the `setup.py` file in the services_ex package:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'services_ex'

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
    description='Examples of services using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_server = services_ex.service_server:main',
            'add_client = services_ex.service_client:main',
        ],
    },
)
```

### 4. Build the Services Package

1. Navigate back to the workspace root:
   ```bash
   cd ~/ros2_ws
   ```

2. Build the services package:
   ```bash
   colcon build --packages-select services_ex
   ```

3. Source the newly built package:
   ```bash
   source install/setup.bash
   ```

### 5. Run the Service Server and Client

1. Open a new terminal, navigate to your workspace, and source the setup:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ```

2. Run the service server:
   ```bash
   ros2 run services_ex add_server
   ```

3. In another terminal, source the setup and run the service client with two numbers:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 run services_ex add_client 2 3
   ```

4. The client should output "Result of add_two_ints: 5" and the server should log the request.

### 6. Explore Service Commands

With the server running, in a third terminal (with environment sourced):

1. List active services:
   ```bash
   ros2 service list
   ```

2. Get information about the service:
   ```bash
   ros2 service info /add_two_ints
   ```

3. Call the service directly from the command line:
   ```bash
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
   ```

## Expected Result

- Service server runs and waits for requests
- Service client sends request and receives response with the sum
- Server logs the incoming request details
- Command-line service call works successfully

## Verification

1. Check that the service is available:
   ```bash
   ros2 service list
   ```
   You should see `/add_two_ints`.

2. Verify the service type:
   ```bash
   ros2 service type /add_two_ints
   ```
   You should see `example_interfaces/srv/AddTwoInts`.

3. Test the service from command line:
   ```bash
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
   ```
   You should get a response with sum=12.

## Troubleshooting

- **Issue**: Service commands are not found
  - **Solution**: Ensure the service package has been built and sourced

- **Issue**: Client gets timeout error
  - **Solution**: Make sure the server is running before starting the client

- **Issue**: Service call from command line fails
  - **Solution**: Verify service name and type are correct, and server is running

- **Issue**: Module or package import errors
  - **Solution**: Check that all packages are properly built and sourced

## Learning Outcomes

- Understanding of the service-server pattern in ROS 2
- Ability to create and run service servers and clients
- Familiarity with ROS 2 service command-line tools
- Knowledge of how services differ from topics (request-response vs. publish-subscribe)
- Understanding when to use services vs. topics for communication

## Further Exploration

1. Create a service that performs a different mathematical operation
2. Create a service that returns multiple values
3. Implement error handling in the service server
4. Create a client that calls the service multiple times in a loop
5. Compare the latency of services vs. topics for communication
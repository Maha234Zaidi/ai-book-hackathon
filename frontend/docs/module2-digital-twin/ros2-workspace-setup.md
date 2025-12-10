# Setting up ROS 2 Workspace for Digital Twin Examples

This guide covers setting up a basic ROS 2 workspace for the digital twin examples used in this module.

## Prerequisites

Before creating your workspace, ensure you have:
- ROS 2 Humble Hawksbill installed
- Properly sourced ROS 2 environment (`source /opt/ros/humble/setup.bash`)
- Python 3.10 installed

## Creating the Workspace

### 1. Set up the workspace directory structure:

```bash
# Create the workspace directory
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws
```

### 2. Create a basic ROS 2 package for digital twin examples:

```bash
cd ~/digital_twin_ws/src
ros2 pkg create --build-type ament_python digital_twin_examples
```

This creates a basic package structure with the following:

```
digital_twin_examples/
├── package.xml
├── setup.cfg
├── setup.py
└── digital_twin_examples/
    └── __init__.py
```

### 3. Update the package.xml to include dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>digital_twin_examples</name>
  <version>0.0.0</version>
  <description>Examples for digital twin implementations using Gazebo and Unity</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>urdf</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 4. Create a launch file for the digital twin simulation:

Create `~/digital_twin_ws/src/digital_twin_examples/digital_twin_examples/launch/digital_twin_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Robot description parameter
    urdf_file = os.path.join(
        get_package_share_directory('digital_twin_examples'),
        'urdf',
        'digital_twin_robot.urdf'
    )
    
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        
        # Robot State Publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_description': robot_description}
            ],
        ),
        
        # Joint State Publisher node (for simulation)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
        
        # Digital Twin State Publisher node
        Node(
            package='digital_twin_examples',
            executable='digital_twin_publisher',
            name='digital_twin_publisher',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
    ])
```

### 5. Create a simple publisher node:

Create `~/digital_twin_ws/src/digital_twin_examples/digital_twin_examples/digital_twin_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class DigitalTwinPublisher(Node):
    def __init__(self):
        super().__init__('digital_twin_publisher')
        
        # Publisher for general digital twin state
        self.state_publisher = self.create_publisher(String, 'digital_twin_state', 10)
        
        # Publisher for joint states
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # TF broadcaster for transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer to periodically publish data
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # Initialize joint states message
        self.joint_state = JointState()
        self.joint_state.name = ['base_to_wheel_left', 'base_to_wheel_right']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0]

    def timer_callback(self):
        # Publish general state message
        msg = String()
        msg.data = f'Digital twin state: {self.i}'
        self.state_publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Update and publish joint states
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.header.frame_id = 'base_link'
        
        # Simulate wheel movement
        self.joint_state.position[0] = float(self.i) * 0.1  # Left wheel
        self.joint_state.position[1] = float(self.i) * 0.1  # Right wheel
        
        self.joint_publisher.publish(self.joint_state)
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Set transform (this would come from localization in a real system)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    digital_twin_publisher = DigitalTwinPublisher()
    
    try:
        rclpy.spin(digital_twin_publisher)
    except KeyboardInterrupt:
        pass
    
    digital_twin_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6. Update setup.py to include the executable:

Update `~/digital_twin_ws/src/digital_twin_examples/setup.py`:

```python
from setuptools import setup

package_name = 'digital_twin_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/digital_twin_simulation.launch.py'
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/digital_twin_robot.urdf'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Examples for digital twin implementations using Gazebo and Unity',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'digital_twin_publisher = digital_twin_examples.digital_twin_publisher:main',
        ],
    },
)
```

### 7. Create a URDF directory and copy the robot URDF:

```bash
mkdir -p ~/digital_twin_ws/src/digital_twin_examples/urdf
```

Then copy the URDF file we created earlier to this location:

```bash
cp /path/to/digital_twin_robot.urdf ~/digital_twin_ws/src/digital_twin_examples/urdf/
```

### 8. Build the workspace:

```bash
cd ~/digital_twin_ws
colcon build --packages-select digital_twin_examples
```

### 9. Source the workspace:

```bash
source ~/digital_twin_ws/install/setup.bash
```

## Running the Examples

### To run the digital twin publisher:

```bash
# Make sure the workspace is sourced
source ~/digital_twin_ws/install/setup.bash

# Run the publisher node
ros2 run digital_twin_examples digital_twin_publisher
```

### To run with the launch file:

```bash
# Make sure the workspace is sourced
source ~/digital_twin_ws/install/setup.bash

# Run the launch file
ros2 launch digital_twin_examples digital_twin_simulation.launch.py
```

## Testing the Setup

To verify your workspace is working correctly:

1. Check that the package is found:
```bash
ros2 pkg list | grep digital_twin
```

2. Check that the executable is available:
```bash
ros2 run digital_twin_examples digital_twin_publisher --ros-args --help
```

3. Run the publisher and monitor topics:
```bash
# Terminal 1: Run the publisher
ros2 run digital_twin_examples digital_twin_publisher

# Terminal 2: Monitor topics
source ~/digital_twin_ws/install/setup.bash
ros2 topic echo /joint_states sensor_msgs/msg/JointState
```

## Adding to Your Shell Profile

To automatically source your workspace when opening a terminal:

```bash
echo "source ~/digital_twin_ws/install/setup.bash" >> ~/.bashrc
```

## Next Steps

This workspace provides a foundation for the digital twin examples in this module. You can:

1. Add more nodes for specific digital twin functionality
2. Integrate with Gazebo using the ROS 2 Gazebo plugins
3. Add Unity communication using the ROS-TCP-Endpoint
4. Expand the URDF with additional sensors and components

The workspace is now ready for the digital twin examples in this module!
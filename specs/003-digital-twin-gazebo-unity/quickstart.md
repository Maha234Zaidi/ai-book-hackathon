# Quickstart: Digital Twin Module (Gazebo & Unity)

## Prerequisites

- Ubuntu 22.04 LTS (or equivalent ROS 2 environment)
- ROS 2 Humble Hawksbill installed
- Gazebo Garden installed
- Unity 2022.3 LTS installed
- Git
- Python 3.10

## Environment Setup

### 1. ROS 2 Environment
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -gpg-agent
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-argcomplete
sudo apt install python3-colcon-common-extensions

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### 2. Gazebo Installation
```bash
# Install Gazebo Garden
sudo apt install software-properties-common lsb-release
sudo add-apt-repository "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main"
sudo apt update
sudo apt install gazebo
```

### 3. Unity Setup
- Download Unity Hub from the Unity website
- Install Unity 2022.3 LTS through Unity Hub
- Install the Unity Robotics SDK (Unity-Robotics-Hub)

## Creating Your First Digital Twin

### 1. URDF Model Preparation
Create a simple robot model in URDF format:

```xml
<!-- example_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.083"/>
    </inertial>
  </link>

  <link name="wheel_1">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="base_to_wheel_1" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_1"/>
    <origin xyz="0.2 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### 2. Setting up Gazebo Simulation
```bash
# Create a new Gazebo world
mkdir -p ~/digital_twin_ws/worlds
# Add your world file here

# Launch Gazebo with your robot
export GAZEBO_MODEL_PATH=~/digital_twin_ws/models:$GAZEBO_MODEL_PATH
gazebo --verbose ~/digital_twin_ws/worlds/your_world.world
```

### 3. Creating ROS 2 Package
```bash
# Create a new workspace
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws
colcon build --packages-select digital_twin_examples
source install/setup.bash

# Create a new package
cd ~/digital_twin_ws/src
ros2 pkg create --build-type ament_python digital_twin_examples
```

### 4. Unity Integration
1. Open Unity Hub and create a new 3D project
2. Import the Unity Robotics SDK
3. Set up a TCP connection to receive data from ROS 2
4. Create a robot model that mirrors the URDF model

### 5. Example Publisher Node (Python)
```python
# ~/digital_twin_ws/src/digital_twin_examples/digital_twin_examples/state_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

class StatePublisher(Node):
    def __init__(self):
        super().__init__('digital_twin_state_publisher')
        self.publisher_ = self.create_publisher(String, 'digital_twin_state', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Digital Twin State: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    state_publisher = StatePublisher()
    rclpy.spin(state_publisher)
    state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Complete Setup

1. Source your ROS 2 environment:
   ```bash
   source ~/digital_twin_ws/install/setup.bash
   ```

2. Start the ROS 2 publisher:
   ```bash
   ros2 run digital_twin_examples state_publisher
   ```

3. Launch Gazebo with your world:
   ```bash
   gazebo --verbose ~/digital_twin_ws/worlds/your_world.world
   ```

4. Start your Unity visualization (with TCP connection to ROS 2)

## Validation

Check that all components are communicating properly:
```bash
# Check active ROS 2 topics
ros2 topic list

# Echo the digital twin state topic
ros2 topic echo /digital_twin_state std_msgs/msg/String
```

## Troubleshooting

- If Gazebo doesn't launch properly, check that the model files are in the correct location and that GAZEBO_MODEL_PATH is set.
- If Unity can't connect, verify the TCP connection settings and firewall configuration.
- If ROS 2 nodes can't communicate, ensure all terminals have sourced the correct setup.bash file.
---
title: Exercise 4 - URDF Robot Modeling
description: Hands-on exercise for creating robot models with URDF
sidebar_position: 4
---

# Exercise 4: URDF Robot Modeling

## Objective

- Understand the structure of URDF (Unified Robot Description Format) files
- Create a simple robot model with links and joints
- Validate URDF files and visualize them
- Add sensors to a robot model

## Prerequisites

- Completed previous exercises
- Basic understanding of 3D geometry
- Understanding of ROS 2 concepts from the module sections

## Setup Requirements

- ROS 2 Humble Hawksbill installed and sourced
- RViz2 for visualization
- URDF tools (should be included with ROS 2 installation)

## Steps

### 1. Explore the Simple Robot URDF

First, let's look at a simple robot model with base, wheels, and a caster:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
      <material name="blue"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </visual>
    
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    
    <inertial>
      <mass value="1.0"/>
      <inertia 
        ixx="0.01" ixy="0.0" ixz="0.0"
        iyy="0.01" iyz="0.0"
        izz="0.02"/>
    </inertial>
  </link>
  
  <!-- Left Wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black"/>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </visual>
    
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </collision>
    
    <inertial>
      <mass value="0.2"/>
      <inertia 
        ixx="0.001" ixy="0.0" ixz="0.0"
        iyy="0.001" iyz="0.0"
        izz="0.002"/>
    </inertial>
  </link>
  
  <!-- Similar definitions for right wheel and caster -->
</robot>
```

### 2. Create Your Own Robot Model

1. Create a new directory for your URDF files:
   ```bash
   mkdir -p ~/robot_models/urdf
   cd ~/robot_models/urdf
   ```

2. Create a simple robot model file called `my_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Materials -->
  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 1.0 0.0 1.0"/>
  </material>
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <material name="orange"/>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>
  
  <!-- Sensor Mount -->
  <joint name="sensor_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_mount"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>
  
  <link name="sensor_mount">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Arm Base -->
  <joint name="arm_base_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_base"/>
    <origin xyz="-0.1 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
  
  <link name="arm_base">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="orange"/>
    </visual>
    
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Arm Link -->
  <joint name="arm_joint" type="revolute">
    <parent link="arm_base"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
  
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.2 0.05 0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.2 0.05 0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

### 3. Validate Your URDF

1. Validate the syntax of your URDF file:
   ```bash
   check_urdf my_robot.urdf
   ```

2. If you don't have the check_urdf command, install the urdfdom package:
   ```bash
   sudo apt install liburdfdom-tools
   ```

3. Generate a graphical representation of your robot:
   ```bash
   urdf_to_graphiz my_robot.urdf
   ```

### 4. Visualize Your Robot in RViz

1. Create a launch file to visualize the robot:
   ```bash
   mkdir -p ~/robot_models/launch
   cd ~/robot_models/launch
   ```

2. Create `view_robot.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the urdf file path
    urdf_file_path = os.path.join(
        get_package_share_directory('robot_state_publisher'),
        'urdf',
        'test.urdf'  # Replace with your URDF file
    )
    
    # Or use the file directly from the path
    robot_desc = Command(['xacro ', '~/robot_models/urdf/my_robot.urdf'])
    
    # Robot State Publisher node
    params = {'robot_description': robot_desc}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        node_robot_state_publisher,
        rviz_node
    ])
```

Actually, for a simpler approach, let's run the robot description directly:

1. Use the robot_state_publisher to visualize (create a temporary file for this example):
   ```bash
   # First, make sure robot_state_publisher is installed
   sudo apt install ros-humble-robot-state-publisher
   ```

2. Start the robot state publisher with your URDF:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(xacro ~/robot_models/urdf/my_robot.urdf)'
   ```

   If xacro isn't available, install it:
   ```bash
   sudo apt install ros-humble-xacro
   ```

   For a direct approach without xacro:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat ~/robot_models/urdf/my_robot.urdf)'
   ```

3. In another terminal, start RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```

4. In RViz, add a RobotModel display and set the Robot Description parameter to "robot_description".

### 5. Add a Sensor to Your Robot

1. Modify your URDF file to add a camera sensor:

```xml
  <!-- Camera Mount -->
  <joint name="camera_joint" type="fixed">
    <parent link="sensor_mount"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Gazebo plugin for the camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <visualize>true</visualize>
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

### 6. Create an Xacro Version (Optional)

1. Create `my_robot.xacro` for a more modular approach:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_xacro_robot">

  <!-- Properties -->
  <xacro:property name="base_size" value="0.3 0.3 0.1" />
  <xacro:property name="wheel_radius" value="0.05" />
  <xacro:property name="wheel_width" value="0.02" />
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Materials -->
  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 1.0 0.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_size}"/>
      </geometry>
      <material name="orange"/>
    </visual>
    
    <collision>
      <geometry>
        <box size="${base_size}"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Macro for creating wheels -->
  <xacro:macro name="simple_wheel" params="prefix parent x_pos y_pos z_pos">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_pos} ${y_pos} ${z_pos}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="green"/>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </visual>
      
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </collision>
      
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create wheels -->
  <xacro:simple_wheel prefix="front_left" parent="base_link" x_pos="0.1" y_pos="0.1" z_pos="-0.05"/>
  <xacro:simple_wheel prefix="front_right" parent="base_link" x_pos="0.1" y_pos="-0.1" z_pos="-0.05"/>
  <xacro:simple_wheel prefix="back_left" parent="base_link" x_pos="-0.1" y_pos="0.1" z_pos="-0.05"/>
  <xacro:simple_wheel prefix="back_right" parent="base_link" x_pos="-0.1" y_pos="-0.1" z_pos="-0.05"/>

</robot>
```

2. Convert Xacro to URDF:
   ```bash
   xacro my_robot.xacro > my_robot_converted.urdf
   ```

## Expected Result

- Your URDF file passes validation with `check_urdf`
- You can visualize your robot in RViz
- Your robot model includes links connected by joints
- Your robot includes visual, collision, and inertial properties
- If applicable, sensors are properly defined in your model

## Verification

1. Check URDF syntax:
   ```bash
   check_urdf ~/robot_models/urdf/my_robot.urdf
   ```

2. Generate a graph to visualize the kinematic tree:
   ```bash
   urdf_to_graphiz ~/robot_models/urdf/my_robot.urdf
   # This creates output files like my_robot.png, my_robot.gv, etc.
   ```

3. Check the number of links and joints:
   ```bash
   grep -c "<link" ~/robot_models/urdf/my_robot.urdf
   grep -c "<joint" ~/robot_models/urdf/my_robot.urdf
   ```

## Troubleshooting

- **Issue**: `check_urdf` command not found
  - **Solution**: Install the required package: `sudo apt install liburdfdom-tools`

- **Issue**: RViz shows no robot model
  - **Solution**: Ensure the robot_state_publisher is running and the topic names match

- **Issue**: URDF validation errors
  - **Solution**: Check for proper XML syntax, required attributes, and correct element structure

- **Issue**: Robot appears distorted in RViz
  - **Solution**: Check that geometry dimensions and origins are properly specified

## Learning Outcomes

- Understanding of URDF structure and components
- Ability to create robot models with proper links and joints
- Knowledge of visual, collision, and inertial properties
- Experience with both URDF and Xacro formats
- Understanding of how to add sensors to robot models

## Further Exploration

1. Create a more complex robot with additional links and joints
2. Add different joint types (prismatic, fixed, etc.)
3. Experiment with different geometric shapes in your robot
4. Create a URDF for a simple humanoid robot
5. Add transmissions to your robot for simulation purposes
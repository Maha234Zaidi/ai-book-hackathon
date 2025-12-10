---
title: URDF for Humanoid Robot Modeling
sidebar_position: 5
---

# Understanding URDF for Humanoid Robots

## Overview
This chapter introduces the Unified Robot Description Format (URDF), which is essential for modeling humanoid robots in ROS 2. We'll cover URDF fundamentals including links, joints, and sensors, how to design a minimal humanoid URDF, and how URDF integrates with ROS 2 and simulation tools. URDF is the standard for representing robot models and provides the geometric and kinematic descriptions that AI agents need to understand and control robot configurations.

## Learning Objectives
After completing this chapter, students will be able to:
- Explain URDF fundamentals: links, joints, and sensors
- Design a minimal humanoid URDF
- Understand visual and collision models in URDF
- Describe how URDF integrates with ROS 2 and simulation tools
- Model basic humanoid robot structures using URDF

## URDF Fundamentals: Links, Joints, and Sensors

### What is URDF?
URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. Its primary function is to provide a complete description of robot structure, including physical dimensions, inertial properties, visual appearance, and kinematic relationships. URDF enables ROS tools to understand the robot's structure and simulate its behavior [ROS URDF Documentation, 2023].

### Links: The Building Blocks
In URDF, a **link** represents a rigid body part of the robot. Each link has:
- A name identifier
- Visual properties (appearance)
- Collision properties (collision detection)
- Inertial properties (mass, center of mass, inertia matrix)

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder radius="0.2" length="0.6"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.2" length="0.6"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joints: Connecting the Links
Joints define the connection between two links and specify the allowable motion. URDF supports several joint types:
- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Continuous**: Rotational movement without limits
- **Prismatic**: Linear sliding movement
- **Floating**: 6DOF movement (for floating objects)

```xml
<joint name="base_to_camera" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0.0 0.3" rpy="0 0 0"/>
</joint>

<joint name="arm_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="upper_arm_link"/>
  <origin xyz="0.0 0.1 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- rotation around Y-axis -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Materials and Appearance
Materials define how links appear visually:

```xml
<material name="red">
  <color rgba="0.8 0.0 0.0 1.0"/>
</material>

<material name="green">
  <color rgba="0.0 0.8 0.0 1.0"/>
</material>
```

### Sensors in URDF
Although not as commonly used as links and joints, URDF can optionally represent sensors:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
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
  </sensor>
</gazebo>
```

## Designing a Minimal Humanoid URDF

### Humanoid Robot Structure
A basic humanoid robot structure includes:
1. **Torso**: Central body connecting limbs
2. **Head**: Sensor mounting point
3. **Arms**: With shoulder, elbow, and wrist joints
4. **Legs**: With hip, knee, and ankle joints
5. **Feet**: For balance and support

### Minimal Humanoid URDF Example
Here's a simplified URDF for a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Material Definitions -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Base Link (Pelvis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.5"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0.0 0.0 0.3"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="2"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.125"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.0" effort="40" velocity="3"/>
  </joint>

  <!-- Similar definitions for left arm, legs, etc. -->
</robot>
```

### URDF Best Practices
1. **Base Link**: Always define a base_link as the root of the kinematic chain
2. **Kinematic Chain**: Ensure all links are connected through joints
3. **Units**: Use meters for lengths, kilograms for masses
4. **Masses**: Include realistic mass values for proper dynamics
5. **Visual vs Collision**: Use simple shapes for collision models to improve simulation speed
6. **Joint Limits**: Always define appropriate limits to prevent damage

## Visual and Collision Models

### Visual Elements
Visual elements define how the robot appears in simulation and visualization tools. Key aspects include:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://my_robot/meshes/part.dae" scale="1.0 1.0 1.0"/>
    <!-- or -->
    <box size="0.1 0.2 0.3"/>
    <!-- or -->
    <cylinder radius="0.05" length="0.2"/>
    <!-- or -->
    <sphere radius="0.1"/>
  </geometry>
  <material name="red"/>
</visual>
```

### Collision Elements
Collision models define the shape used for collision detection. These can differ from visual models to optimize performance:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Often using simpler shapes than visual models -->
    <!-- e.g., using boxes instead of complex meshes -->
    <box size="0.1 0.2 0.3"/>
  </geometry>
</collision>
```

### Using Mesh Files
For complex geometries, URDF supports mesh files:

```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/complex_part.stl" scale="1.0 1.0 1.0"/>
  </geometry>
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
</visual>
```

For real robot applications, meshes are typically stored in a `meshes` subdirectory of a ROS package and referenced using the `package://` format.

## How URDF Integrates with ROS 2 and Simulation Tools

### URDF and ROS 2 Ecosystem
URDF integrates with ROS 2 through several key mechanisms:

1. **Robot State Publisher**: Converts joint positions to transforms
2. **TF2**: Provides transformation trees for spatial relationships
3. **RViz**: Visualizes robot models in real-time
4. **Simulation**: Used by Gazebo, Ignition, and other simulators

### Robot State Publisher
The robot_state_publisher package takes joint position data and calculates the 3D poses of each link using the kinematic chain defined in the URDF. It publishes transform data to the `/tf` and `/tf_static` topics, which can be consumed by other ROS 2 nodes [Robot State Publisher, 2023].

### Example Launch Configuration
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command([
        'xacro ',
        get_package_share_directory('my_robot_description'),
        '/urdf/my_robot.urdf.xacro'
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])
```

### URDF with Controllers
In real applications, URDF works with joint state controllers to provide feedback:

```yaml
# Control configuration for a humanoid robot
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    use_sim_time: true

joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster

effort_controllers:
  type: effort_controllers/JointGroupEffortController
  joints:
    - neck_joint
    - right_shoulder_joint
    - right_elbow_joint
    # ... other joints
```

### URDF and AI Agents
As discussed in [Chapter 3](./chapter-3-ai-agents-control.md), AI agents use URDF information to understand robot structure and kinematics, enabling them to:
- Plan valid joint movements
- Avoid self-collisions
- Understand reachability constraints
- Coordinate complex multi-limb actions

## URDF Tools and Utilities

### Checking URDF Validity
To verify a URDF is correctly formatted:
```bash
check_urdf /path/to/robot.urdf
```

### Visualizing URDF
```bash
urdf_to_graphiz /path/to/robot.urdf
```

### Converting Xacro to URDF
Many robots use Xacro (XML Macros) to simplify URDF creation:
```bash
xacro input_file.xacro > output_file.urdf
```

Xacro allows parameterization and reuse of common structures, which is especially useful for symmetric limbs on humanoid robots.

## Summary
This chapter introduced URDF (Unified Robot Description Format), the standard for representing robot models in ROS 2. We covered the fundamental components of URDF: links for rigid bodies, joints for connections, and the importance of visual and collision models. We demonstrated how to design a minimal humanoid URDF and explained how URDF integrates with the broader ROS 2 ecosystem, including the robot state publisher, TF2, and visualizers.

Building on concepts from [Chapter 1](./chapter-1-ros2-intro.md), URDF provides the geometric and kinematic descriptions that enable the "robotic nervous system" to have a structural understanding of the physical robot. Following [Chapter 2](./chapter-2-nodes-topics-services.md)'s communication patterns, URDF provides the model that AI agents in [Chapter 3](./chapter-3-ai-agents-control.md) need to understand how their commands will affect the robot's physical configuration.

In the next chapter, we'll tie these concepts together in a mini-project that demonstrates the complete integration of nodes, communication systems, AI agents, and robot models in a practical humanoid control pipeline.

## Key Terms
- **URDF**: Unified Robot Description Format, an XML format for representing robot models
- **Link**: A rigid body part of the robot in URDF
- **Joint**: Defines the connection and allowed motion between two links
- **Visual Model**: The appearance of a robot link in visualization tools
- **Collision Model**: The shape used for collision detection in simulation
- **Robot State Publisher**: ROS 2 package that publishes transform data based on joint positions
- **TF2**: Transform library that manages coordinate frames in ROS 2
- **Xacro**: XML macro language for simplifying URDF creation

## Further Reading
- URDF/XML Format Reference [ROS URDF Format, 2023]
- Robot Modeling Best Practices [Robotic Modeling Guidelines, 2023]
- Integrating URDF with Control Systems [ROS Control URDF Integration, 2023]

## Exercises (Optional)
1. Modify the minimal humanoid URDF example to include a left arm with the same joint structure as the right arm.
2. Design a URDF that includes a simple gripper mechanism with two fingers.
3. Identify the minimum set of joints required to achieve a basic walking gait in a humanoid robot.
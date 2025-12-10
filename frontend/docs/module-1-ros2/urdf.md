---
title: Understanding URDF
description: Defining robot models with Unified Robot Description Format
sidebar_position: 4
---

# Understanding URDF

## Learning Objectives

- Understand the purpose and structure of URDF (Unified Robot Description Format)
- Define robot links with proper physical properties
- Create joints to connect links with specific degrees of freedom
- Integrate sensors into your robot model
- Validate URDF files and troubleshoot common issues

## Introduction to URDF

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. URDF allows you to define the physical structure of a robot, including its links (rigid parts), joints (connections between links), and other properties like visual and collision geometry.

### Why URDF Matters

URDF is essential for:
- Robot simulation in Gazebo and other physics engines
- Robot visualization in RViz
- Inverse kinematics and motion planning
- Collision detection
- Robot calibration and control

## URDF Structure

A URDF file is an XML document that describes a robot as a tree structure of connected links. Each link is connected to its parent via a joint.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Links are the rigid parts of the robot -->
  <link name="base_link">
    <!-- Visual properties for display -->
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <!-- Collision properties for physics simulation -->
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <!-- Physical properties like mass and inertia -->
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
  
  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.5 0" rpy="0 0 0"/>
  </joint>
  
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

## Defining Links

### Link Structure

A link represents a rigid part of the robot and can have multiple child elements:

1. **Visual**: How the link appears in visualizations
2. **Collision**: How the link interacts in physics simulation
3. **Inertial**: Physical properties for dynamics calculations

### Visual Properties

The visual element defines how the link appears in visualization tools:

```xml
<link name="visual_example">
  <visual>
    <!-- Origin specifies position and orientation relative to link frame -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- Geometry defines the shape -->
    <geometry>
      <!-- Options: box, cylinder, sphere, or mesh -->
      <box size="0.1 0.2 0.3"/>
    </geometry>
    <!-- Material defines color and texture -->
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
</link>
```

### Collision Properties

The collision element defines how the link interacts in physics simulations:

```xml
<link name="collision_example">
  <collision>
    <!-- Typically simpler geometry than visual for performance -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.2 0.3"/>
    </geometry>
  </collision>
</link>
```

### Inertial Properties

The inertial element defines the physical properties of the link:

```xml
<link name="inertial_example">
  <inertial>
    <!-- Mass in kilograms -->
    <mass value="0.1"/>
    <!-- Inertia matrix -->
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

## Defining Joints

Joints connect links and define how they can move relative to each other.

### Joint Types

1. **Revolute**: Rotational joint with limited range
2. **Continuous**: Rotational joint without limits
3. **Prismatic**: Linear sliding joint with limits
4. **Fixed**: No movement between links
5. **Floating**: 6 DOF movement (rarely used)
6. **Planar**: Movement on a plane

### Joint Structure

```xml
<joint name="joint_example" type="revolute">
  <!-- Parent link in the kinematic tree -->
  <parent link="parent_link"/>
  <!-- Child link in the kinematic tree -->
  <child link="child_link"/>
  <!-- Position and orientation of joint frame -->
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <!-- Axis of rotation or translation -->
  <axis xyz="0 0 1"/>
  <!-- Joint limits (for revolute and prismatic) -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

## Advanced URDF Features

### Materials

Define materials for visual appearance:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="white">
  <color rgba="1 1 1 1"/>
</material>
```

### Transmission Elements

Define how actuators interact with joints:

```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-Specific Elements

For simulation in Gazebo:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

## Sensors in URDF

Sensors are typically implemented as fixed joints attached to existing links:

```xml
<!-- Joint for sensor attachment -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Sensor link -->
<link name="camera_link">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Gazebo plugin for the sensor -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <pose>0 0 0 0 0 0</pose>
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
  </sensor>
</gazebo>
```

## URDF for Humanoid Robots

Humanoid robots require special attention to create realistic models:

### Key Considerations for Humanoid Models

1. Proper joint limits to match human range of motion
2. Appropriate inertial properties for each body segment
3. Kinematic chain structure (legs, arms, torso, head)
4. Balance and stability considerations

### Example Humanoid Fragment

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Upper Arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>
</robot>
```

## URDF Validation and Tools

### Validating URDF

Several tools help validate and visualize URDF:

1. `check_urdf <urdf_file>`: Checks for XML syntax and structure
2. `urdf_to_graphiz <urdf_file>`: Creates kinematic graph
3. RViz: Visualizes the robot model
4. Gazebo: Tests physics simulation

### Common URDF Issues and Solutions

1. **Floating point precision**: Use consistent precision (typically 3-4 decimal places)
2. **Inertia calculations**: Use proper formulas for different geometries
3. **Joint limits**: Set appropriate limits to match real hardware
4. **Mass properties**: Ensure physical plausibility of mass and inertia values

## Best Practices for URDF

1. **Start Simple**: Begin with a basic skeleton and add complexity gradually
2. **Use Standard Units**: Stick to SI units (meters, kilograms, seconds)
3. **Validate Regularly**: Check your URDF frequently during development
4. **Modular Design**: Use xacro macros for complex, repetitive structures
5. **Documentation**: Comment your URDF files to explain design decisions
6. **Realistic Parameters**: Use actual robot dimensions and properties when available

## Xacro: XML Macros for URDF

Xacro allows you to create parameterized URDF models:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_example">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  
  <!-- Macro for a wheel -->
  <xacro:macro name="wheel" params="prefix parent xyz">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="0 ${M_PI/2} 0"/>
      <axis xyz="0 0 1"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create wheels -->
  <link name="chassis">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <xacro:wheel prefix="front_left" parent="chassis" xyz="0.15 0.15 0"/>
  <xacro:wheel prefix="front_right" parent="chassis" xyz="0.15 -0.15 0"/>
  <xacro:wheel prefix="back_left" parent="chassis" xyz="-0.15 0.15 0"/>
  <xacro:wheel prefix="back_right" parent="chassis" xyz="-0.15 -0.15 0"/>
</robot>
```

## Summary

This section covered the fundamentals of URDF (Unified Robot Description Format), the standard for describing robot models in ROS. You learned about the structure of URDF files, how to define links and joints, how to integrate sensors, and best practices for creating robot models, particularly for humanoid robots. 

You also learned about advanced topics like Xacro for creating parameterized robot models and tools for validating URDF files. Understanding URDF is crucial for robot simulation, visualization, and control in ROS-based systems.

In the next section, we'll cover best practices and preparation for capstone projects that combine all the concepts learned in this module.
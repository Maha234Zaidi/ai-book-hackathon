# URDF Examples: Different Physical Properties

This section provides comprehensive URDF examples demonstrating how to define different physical properties such as mass, friction, and damping to create realistic robot models for your digital twin system.

## Understanding Physical Properties in URDF

Physical properties in URDF define how objects behave in the physics simulation. These properties include:

- **Mass**: How much matter the object contains (affects response to forces)
- **Inertia**: How mass is distributed (affects rotational behavior)
- **Friction**: Resistance to sliding motion
- **Damping**: Resistance to motion (linear and angular)

## Basic Physical Properties Setup

### Simple Link with Mass and Inertia
```xml
<link name="basic_link">
  <!-- Inertial properties define how the object responds to forces -->
  <inertial>
    <!-- Mass in kilograms -->
    <mass value="1.0"/>
    
    <!-- Inertia tensor (how mass is distributed) -->
    <inertia 
      ixx="0.0833" ixy="0.0" ixz="0.0"
      iyy="0.0833" iyz="0.0"
      izz="0.1667" />
  </inertial>
  
  <!-- Visual representation -->
  <visual>
    <geometry>
      <box size="0.5 0.5 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  
  <!-- Collision representation -->
  <collision>
    <geometry>
      <box size="0.5 0.5 0.1"/>
    </geometry>
  </collision>
</link>
```

### Calculating Inertia Values

For common shapes, you can calculate inertia values:

**Box** (width w, depth d, height h, mass m):
- ixx = 1/12 * m * (h² + d²)
- iyy = 1/12 * m * (w² + h²) 
- izz = 1/12 * m * (w² + d²)

**Cylinder** (radius r, height h, mass m), axis-aligned with Y:
- ixx = 1/12 * m * (3*r² + h²)
- iyy = 1/2 * m * r²
- izz = 1/12 * m * (3*r² + h²)

**Sphere** (radius r, mass m):
- ixx = iyy = izz = 2/5 * m * r²

## Mass Examples

### Low Mass Object (e.g., Ping Pong Ball)
```xml
<link name="ping_pong_ball">
  <inertial>
    <mass value="0.0027"/>  <!-- 2.7 grams -->
    <inertia 
      ixx="0.00000162" ixy="0" ixz="0"
      iyy="0.00000162" iyz="0"
      izz="0.00000162" />
  </inertial>
  
  <visual>
    <geometry>
      <sphere radius="0.02"/>
    </geometry>
    <material name="white">
      <color rgba="1 1 1 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <sphere radius="0.02"/>
    </geometry>
  </collision>
</link>
```

### Medium Mass Object (e.g., Robot Wheel)
```xml
<link name="robot_wheel">
  <inertial>
    <mass value="0.5"/>  <!-- 500 grams -->
    <inertia 
      ixx="0.00156" ixy="0" ixz="0"   <!-- Cylinder formula -->
      iyy="0.00125" iyz="0"
      izz="0.00156" />
  </inertial>
  
  <visual>
    <geometry>
      <cylinder length="0.05" radius="0.1"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <cylinder length="0.05" radius="0.1"/>
    </geometry>
  </collision>
</link>
```

### High Mass Object (e.g., Robot Base)
```xml
<link name="robot_base">
  <inertial>
    <mass value="10.0"/>  <!-- 10 kg -->
    <inertia 
      ixx="0.416" ixy="0" ixz="0"     <!-- Box formula for 0.5x0.5x0.2m -->
      iyy="0.416" iyz="0"
      izz="0.625" />
  </inertial>
  
  <visual>
    <geometry>
      <box size="0.5 0.5 0.2"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <box size="0.5 0.5 0.2"/>
    </geometry>
  </collision>
</link>
```

## Friction Examples

Friction is defined in the collision surface parameters:

### High Friction (e.g., Rubber Tire)
```xml
<link name="high_friction_wheel">
  <inertial>
    <mass value="0.8"/>
    <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.0015" iyz="0" izz="0.002"/>
  </inertial>
  
  <visual>
    <geometry>
      <cylinder length="0.05" radius="0.1"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <cylinder length="0.05" radius="0.1"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.2</mu>      <!-- High primary friction (0.8-1.2 for rubber) -->
          <mu2>1.2</mu2>    <!-- High secondary friction -->
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

### Medium Friction (e.g., Plastic Surface)
```xml
<link name="medium_friction_surface">
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
  </inertial>
  
  <visual>
    <geometry>
      <box size="0.4 0.4 0.02"/>
    </geometry>
    <material name="plastic">
      <color rgba="0.5 0.7 0.9 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <box size="0.4 0.4 0.02"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>0.5</mu>      <!-- Medium friction (0.3-0.7 for plastic) -->
          <mu2>0.4</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

### Low Friction (e.g., Ice or Teflon)
```xml
<link name="low_friction_surface">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.167"/>
  </inertial>
  
  <visual>
    <geometry>
      <box size="0.5 0.5 0.1"/>
    </geometry>
    <material name="ice">
      <color rgba="0.8 0.9 1 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <box size="0.5 0.5 0.1"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>0.05</mu>     <!-- Very low friction (0.01-0.1 for ice/teflon) -->
          <mu2>0.04</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

## Damping Examples

Damping represents resistance to motion, simulating effects like air resistance or viscous damping:

### High Damping (e.g., Movement in Thick Fluid)
```xml
<link name="high_damping_link">
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
  </inertial>
  
  <visual>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
    <material name="orange">
      <color rgba="1 0.5 0 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
    <surface>
      <damping>
        <linear>20.0</linear>     <!-- High linear damping -->
        <angular>5.0</angular>    <!-- High angular damping -->
      </damping>
    </surface>
  </collision>
</link>
```

### Low Damping (e.g., Movement in Air)
```xml
<link name="low_damping_link">
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005"/>
  </inertial>
  
  <visual>
    <geometry>
      <sphere radius="0.03"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <sphere radius="0.03"/>
    </geometry>
    <surface>
      <damping>
        <linear>0.1</linear>      <!-- Low linear damping -->
        <angular>0.1</angular>    <!-- Low angular damping -->
      </damping>
    </surface>
  </collision>
</link>
```

## Complete Robot Example with Various Physical Properties

Here's a comprehensive robot model demonstrating different physical properties:

```xml
<?xml version="1.0"?>
<robot name="physics_demo_robot">
  <!-- Robot base with standard properties -->
  <link name="base_link">
    <inertial>
      <mass value="8.0"/>
      <inertia 
        ixx="0.333" ixy="0" ixz="0"
        iyy="0.333" iyz="0"
        izz="0.5" />
    </inertial>
    
    <visual>
      <geometry>
        <box size="0.4 0.4 0.15"/>
      </geometry>
      <material name="robot_blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.4 0.4 0.15"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.3</mu>      <!-- Moderate friction for base -->
            <mu2>0.3</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- High-friction drive wheels -->
  <link name="left_wheel">
    <inertial>
      <mass value="0.8"/>
      <inertia 
        ixx="0.0021" ixy="0" ixz="0"
        iyy="0.0016" iyz="0"
        izz="0.0021" />
    </inertial>
    
    <visual>
      <geometry>
        <cylinder length="0.04" radius="0.08"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <cylinder length="0.04" radius="0.08"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>      <!-- High friction for traction -->
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <link name="right_wheel">
    <inertial>
      <mass value="0.8"/>
      <inertia 
        ixx="0.0021" ixy="0" ixz="0"
        iyy="0.0016" iyz="0"
        izz="0.0021" />
    </inertial>
    
    <visual>
      <geometry>
        <cylinder length="0.04" radius="0.08"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <cylinder length="0.04" radius="0.08"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- Low-friction caster wheel -->
  <link name="caster_wheel">
    <inertial>
      <mass value="0.1"/>
      <inertia 
        ixx="0.00008" ixy="0" ixz="0"
        iyy="0.00008" iyz="0"
        izz="0.00008" />
    </inertial>
    
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.1</mu>      <!-- Low friction for free rotation -->
            <mu2>0.1</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- High-damping sensor component -->
  <link name="lidar_sensor">
    <inertial>
      <mass value="0.3"/>
      <inertia 
        ixx="0.0003" ixy="0" ixz="0"
        iyy="0.0003" iyz="0"
        izz="0.0003" />
    </inertial>
    
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="silver">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <surface>
        <damping>
          <linear>5.0</linear>    <!-- Higher damping for sensor stability -->
          <angular>2.0</angular>
        </damping>
      </surface>
    </collision>
  </link>

  <!-- Joints connecting components -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.22 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.22 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.15 0 -0.075" rpy="0 0 0"/>
  </joint>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_sensor"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
```

## Advanced Physical Property Configurations

### Variable Friction Surfaces (Gazebo-specific)
```xml
<link name="variable_friction_link">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.167"/>
  </inertial>
  
  <visual>
    <geometry>
      <box size="0.2 0.2 0.1"/>
    </geometry>
  </visual>
  
  <collision>
    <geometry>
      <box size="0.2 0.2 0.1"/>
    </geometry>
  </collision>
  
  <!-- Gazebo-specific surface properties -->
  <gazebo reference="variable_friction_link">
    <collision>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.6</mu2>
            <fdir1>1 0 0</fdir1>  <!-- Friction direction -->
          </ode>
          <torsional>
            <coefficient>1.0</coefficient>  <!-- Torsional friction -->
            <patch_radius>0.01</patch_radius>
          </torsional>
        </friction>
      </surface>
    </collision>
  </gazebo>
</link>
```

### Bounce and Restitution Properties
```xml
<link name="bouncy_object">
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0004" ixy="0" ixz="0" iyy="0.0004" iyz="0" izz="0.0004"/>
  </inertial>
  
  <visual>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  
  <collision>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
    <surface>
      <bounce>
        <restitution_coefficient>0.9</restitution_coefficient>  <!-- Very bouncy -->
        <threshold>1.0</threshold>  <!-- Velocity threshold for bounce effect -->
      </bounce>
    </surface>
  </collision>
</link>
```

## Tips for Setting Realistic Physical Properties

### 1. Research Real-World Values
- Use material property databases for accurate friction coefficients
- Reference physics literature for moment of inertia calculations
- Consider environmental conditions (temperature, humidity, etc.)

### 2. Balance Accuracy with Performance
- Use simplified geometries for collision when possible
- Choose appropriate mass values that don't make simulation unstable
- Test with your specific physics engine configuration

### 3. Validation Approach
- Start with approximate values and refine through testing
- Compare simulation behavior with real-world expectations
- Use multiple test scenarios to validate properties

### 4. Common Material Friction Coefficients
- **Rubber on concrete**: 0.8-1.0
- **Steel on steel (dry)**: 0.6-0.8
- **Aluminum on aluminum**: 1.05-1.35
- **Teflon on Teflon**: 0.04
- **Ice on ice**: 0.02-0.09
- **Wood on wood**: 0.25-0.5

With these examples, you can create URDF models with realistic physical properties that accurately represent the behavior of your real-world robots in the digital twin system.
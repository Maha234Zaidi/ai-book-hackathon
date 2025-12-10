# Hands-on Exercise 2: Configuring Physics Parameters for Realistic Simulation

In this exercise, you'll learn how to configure physics parameters in Gazebo to create realistic simulations that accurately model real-world robot behavior. You'll work with mass, friction, damping, and other parameters to create a robot that behaves realistically in various scenarios.

## Learning Objectives

By completing this exercise, you will:
1. Configure realistic physical properties for different robot components
2. Adjust physics engine parameters for optimal simulation
3. Validate simulation behavior against real-world expectations
4. Troubleshoot common physics simulation issues

## Prerequisites

Before starting this exercise, ensure you have:
- Gazebo installed (Garden or Harmonic version)
- ROS 2 Humble Hawksbill installed
- Basic understanding of URDF and SDF formats
- Completed Exercise 1 on basic digital twin setup

## Part 1: Setting Up the Exercise Environment

### Step 1: Create Exercise Directory
```bash
mkdir -p ~/gazebo_physics_exercise/models
mkdir -p ~/gazebo_physics_exercise/worlds
cd ~/gazebo_physics_exercise
```

### Step 2: Create Robot Model - Differential Drive Robot with Physical Properties

Create `~/gazebo_physics_exercise/models/physics_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="physics_robot">
  <!-- Robot base with realistic mass and inertia -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>  <!-- 5kg base -->
      <inertia 
        ixx="0.2083" ixy="0.0" ixz="0.0"
        iyy="0.2083" iyz="0.0"
        izz="0.3125" />
    </inertial>
    
    <visual>
      <geometry>
        <box size="0.3 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.3 0.3 0.15"/>
      </geometry>
    </collision>
  </link>

  <!-- Left wheel with rubber properties for traction -->
  <link name="wheel_left">
    <inertial>
      <mass value="0.3"/>
      <inertia 
        ixx="0.00075" ixy="0.0" ixz="0.0"
        iyy="0.001125" iyz="0.0"
        izz="0.00075" />
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
            <mu>1.0</mu>    <!-- High friction for traction -->
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- Right wheel with rubber properties for traction -->
  <link name="wheel_right">
    <inertial>
      <mass value="0.3"/>
      <inertia 
        ixx="0.00075" ixy="0.0" ixz="0.0"
        iyy="0.001125" iyz="0.0"
        izz="0.00075" />
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
            <mu>1.0</mu>    <!-- High friction for traction -->
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- Caster wheel with low friction -->
  <link name="caster_wheel">
    <inertial>
      <mass value="0.1"/>
      <inertia 
        ixx="0.00004" ixy="0.0" ixz="0.0"
        iyy="0.00004" iyz="0.0"
        izz="0.00004" />
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
            <mu>0.1</mu>    <!-- Low friction for free rotation -->
            <mu2>0.1</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- Sensor (LiDAR) on top -->
  <link name="lidar_sensor">
    <inertial>
      <mass value="0.2"/>
      <inertia 
        ixx="0.0001" ixy="0.0" ixz="0.0"
        iyy="0.0001" iyz="0.0"
        izz="0.0001" />
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
    </collision>
  </link>

  <!-- Joints -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.17 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.17 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.12 0 -0.075" rpy="0 0 0"/>
  </joint>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_sensor"/>
    <origin xyz="0.0 0.0 0.125" rpy="0 0 0"/>
  </joint>
</robot>
```

### Step 3: Create a Test World with Different Surfaces

Create `~/gazebo_physics_exercise/worlds/physics_test_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_test_world">
    <!-- Earth-like gravity -->
    <gravity>0 0 -9.8</gravity>
    
    <!-- Physics engine configuration -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    
    <!-- Light sources -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- High friction surface (rubber-like) -->
    <model name="high_friction_surface">
      <pose>2 0 0.01 0 0 0</pose>
      <link name="surface_link">
        <visual name="visual">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <material>
            <ambient>1 0.5 0 1</ambient>
            <diffuse>1 0.5 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>    <!-- High friction -->
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Low friction surface (ice-like) -->
    <model name="low_friction_surface">
      <pose>-2 0 0.01 0 0 0</pose>
      <link name="surface_link">
        <visual name="visual">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <material>
            <ambient>0.8 0.9 1 1</ambient>
            <diffuse>0.8 0.9 1 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.05</mu>   <!-- Low friction -->
                <mu2>0.05</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Medium friction surface (wood-like) -->
    <model name="medium_friction_surface">
      <pose>0 2 0.01 0 0 0</pose>
      <link name="surface_link">
        <visual name="visual">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.4</mu>    <!-- Medium friction -->
                <mu2>0.4</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Sloped surface -->
    <model name="sloped_surface">
      <pose>0 -2 0 0 0.2 0</pose>
      <link name="slope_link">
        <visual name="visual">
          <geometry>
            <box size="2 1 0.02"/>
          </geometry>
          <material>
            <ambient>0.5 1 0.5 1</ambient>
            <diffuse>0.5 1 0.5 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="2 1 0.02"/>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>    <!-- Moderate friction for slope -->
                <mu2>0.6</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <inertial>
          <mass>20</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Place our robot in the center -->
    <include>
      <uri>model://physics_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## Part 2: Running the Basic Simulation

### Step 4: Launch the Simulation

First, set up the GAZEBO_MODEL_PATH:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_physics_exercise/models
```

Then launch Gazebo with your world:

```bash
gazebo ~/gazebo_physics_exercise/worlds/physics_test_world.sdf
```

### Step 5: Observe Initial Behavior

Observe the robot's behavior:
1. Does it sit stably on the ground?
2. Do the wheels stay in contact with the surface?
3. Does it respond appropriately to the environment?

## Part 3: Adjusting Physics Parameters

### Step 6: Modify the Physics Engine Configuration

Create a new world file with more precise physics settings: `~/gazebo_physics_exercise/worlds/physics_test_world_precise.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_test_world_precise">
    <gravity>0 0 -9.8</gravity>
    
    <!-- More precise physics engine configuration -->
    <physics name="precise_physics" type="ode">
      <max_step_size>0.0005</max_step_size>    <!-- Smaller time step for accuracy -->
      <real_time_factor>0.5</real_time_factor> <!-- Allow slower than real-time -->
      <real_time_update_rate>2000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>20</iters>      <!-- More iterations for accuracy -->
          <sor>1.0</sor>          <!-- Conservative SOR value -->
        </solver>
        <constraints>
          <cfm>0.0001</cfm>       <!-- Small CFM for stability -->
          <erp>0.1</erp>          <!-- Lower ERP for stability -->
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.0005</contact_surface_layer> <!-- Thinner contact layer -->
        </constraints>
      </ode>
    </physics>
    
    <!-- Include same models as before -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- High friction surface (rubber-like) -->
    <model name="high_friction_surface">
      <pose>2 0 0.01 0 0 0</pose>
      <link name="surface_link">
        <visual name="visual">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <material>
            <ambient>1 0.5 0 1</ambient>
            <diffuse>1 0.5 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="1 1 0.02"/>
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
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Low friction surface (ice-like) -->
    <model name="low_friction_surface">
      <pose>-2 0 0.01 0 0 0</pose>
      <link name="surface_link">
        <visual name="visual">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <material>
            <ambient>0.8 0.9 1 1</ambient>
            <diffuse>0.8 0.9 1 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.05</mu>
                <mu2>0.05</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Medium friction surface (wood-like) -->
    <model name="medium_friction_surface">
      <pose>0 2 0.01 0 0 0</pose>
      <link name="surface_link">
        <visual name="visual">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="1 1 0.02"/>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.4</mu>
                <mu2>0.4</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Sloped surface -->
    <model name="sloped_surface">
      <pose>0 -2 0 0 0.2 0</pose>
      <link name="slope_link">
        <visual name="visual">
          <geometry>
            <box size="2 1 0.02"/>
          </geometry>
          <material>
            <ambient>0.5 1 0.5 1</ambient>
            <diffuse>0.5 1 0.5 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="2 1 0.02"/>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <inertial>
          <mass>20</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Place our robot in the center -->
    <include>
      <uri>model://physics_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Step 7: Compare Different Physics Configurations

Run both configurations and observe the differences:
- Launch the original world: `gazebo ~/gazebo_physics_exercise/worlds/physics_test_world.sdf`
- Launch the precise world: `gazebo ~/gazebo_physics_exercise/worlds/physics_test_world_precise.sdf`

Compare the behaviors:
1. Stability of the robot
2. Response to collisions
3. Smoothness of motion
4. Performance (framerate)

## Part 4: Experimenting with Physical Properties

### Step 8: Create a Physics Property Testing Robot

Create a new robot model with adjustable parameters: `~/gazebo_physics_exercise/models/test_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="test_robot">
  <!-- Base with configurable physical properties -->
  <link name="base_link">
    <inertial>
      <mass value="2.0"/>  <!-- Adjust this value to test different masses -->
      <inertia 
        ixx="0.083" ixy="0.0" ixz="0.0"
        iyy="0.083" iyz="0.0"
        izz="0.167" />
    </inertial>
    
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.5</mu>    <!-- Adjust this for testing -->
            <mu2>0.5</mu2>
          </ode>
        </friction>
        <damping>
          <linear>0.1</linear>   <!-- Adjust this for testing -->
          <angular>0.1</angular>
        </damping>
      </surface>
    </collision>
  </link>
  
  <!-- Additional test objects with different properties -->
  <link name="light_object">
    <inertial>
      <mass value="0.1"/>  <!-- Very light -->
      <inertia ixx="0.000083" ixy="0.0" ixz="0.0" iyy="0.000083" iyz="0.0" izz="0.000167"/>
    </inertial>
    
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
  </link>
  
  <link name="heavy_object">
    <inertial>
      <mass value="5.0"/>  <!-- Heavy -->
      <inertia ixx="0.0083" ixy="0.0" ixz="0.0" iyy="0.0083" iyz="0.0" izz="0.0167"/>
    </inertial>
    
    <visual>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
      <material name="purple">
        <color rgba="1 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
    </collision>
  </link>
  
  <link name="bouncy_object">
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.00016" ixy="0.0" ixz="0.0" iyy="0.00016" iyz="0.0" izz="0.00032"/>
    </inertial>
    
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="cyan">
        <color rgba="0 1 1 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <surface>
        <bounce>
          <restitution_coefficient>0.8</restitution_coefficient>  <!-- Bouncy -->
          <threshold>1.0</threshold>
        </bounce>
      </surface>
    </collision>
  </link>
  
  <!-- Position them for testing -->
  <joint name="light_joint" type="fixed">
    <parent link="base_link"/>
    <child link="light_object"/>
    <origin xyz="0.2 0 0.1"/>
  </joint>
  
  <joint name="heavy_joint" type="fixed">
    <parent link="base_link"/>
    <child link="heavy_object"/>
    <origin xyz="-0.2 0 0.1"/>
  </joint>
  
  <joint name="bouncy_joint" type="fixed">
    <parent link="base_link"/>
    <child link="bouncy_object"/>
    <origin xyz="0 0.2 0.1"/>
  </joint>
</robot>
```

### Step 9: Create a Testing World

Create `~/gazebo_physics_exercise/worlds/test_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="test_world">
    <gravity>0 0 -9.8</gravity>
    
    <!-- Variable physics configuration for testing -->
    <physics name="test_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Ramps to test different behaviors -->
    <model name="ramp_15">
      <pose>2 0 0.5 0 0.26 0</pose>  <!-- ~15 degrees -->
      <link name="ramp_link">
        <visual name="visual">
          <geometry>
            <box size="1 1 0.05"/>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="1 1 0.05"/>
          </geometry>
        </collision>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <model name="ramp_30">
      <pose>-2 0 0.5 0 0.52 0</pose>  <!-- ~30 degrees -->
      <link name="ramp_link">
        <visual name="visual">
          <geometry>
            <box size="1 1 0.05"/>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="1 1 0.05"/>
          </geometry>
        </collision>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Place the test robot -->
    <include>
      <uri>model://test_robot</uri>
      <pose>0 0 0.2 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## Part 5: Testing and Validation

### Step 10: Run Physical Property Tests

Launch your test world:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_physics_exercise/models
gazebo ~/gazebo_physics_exercise/worlds/test_world.sdf
```

### Step 11: Test Different Parameters

Create a Python script to help test and validate different parameters: `~/gazebo_physics_exercise/test_params.py`:

```python
#!/usr/bin/env python3
# test_params.py
# Script to test and validate physics parameters in Gazebo

import sys
import os
import math
import time

def calculate_fall_time(height, gravity=9.81):
    """
    Calculate theoretical fall time for an object dropped from a height
    Formula: t = sqrt(2h/g)
    """
    if height <= 0:
        return 0
    return math.sqrt(2 * height / gravity)

def calculate_impact_velocity(height, gravity=9.81):
    """
    Calculate the velocity at impact for an object dropped from height
    Formula: v = sqrt(2gh)
    """
    if height <= 0:
        return 0
    return math.sqrt(2 * gravity * height)

def test_mass_impact(mass, height, observed_time, gravity=9.81):
    """
    Test if mass affects fall time (it shouldn't in vacuum)
    """
    theoretical_time = calculate_fall_time(height, gravity)
    print(f"Mass: {mass}kg, Height: {height}m")
    print(f"Theoretical fall time: {theoretical_time:.3f}s")
    print(f"Observed fall time: {observed_time:.3f}s")
    
    # In a perfect vacuum, mass doesn't affect fall time
    # In simulation with air resistance, small differences may occur
    tolerance = 0.1  # 10% tolerance for simulation approximations
    relative_diff = abs(observed_time - theoretical_time) / theoretical_time
    
    if relative_diff <= tolerance:
        print("✓ Mass does not significantly affect fall time (as expected)")
        return True
    else:
        print(f"✗ Unexpected difference: {relative_diff*100:.1f}%")
        print("  This could be due to simulation precision or air resistance")
        return False

def test_friction_effect(ground_friction, object_velocity, time_to_stop):
    """
    Test how friction affects stopping distance
    """
    print(f"Ground friction: {ground_friction}, Initial velocity: {object_velocity:.3f}m/s")
    print(f"Time to stop: {time_to_stop:.3f}s")
    
    # Estimate acceleration due to friction
    # a = g * μ (where g is gravity and μ is friction coefficient)
    estimated_acceleration = 9.81 * ground_friction
    print(f"Estimated deceleration: {estimated_acceleration:.3f} m/s²")
    
    # Calculate stopping distance
    stopping_distance = (object_velocity ** 2) / (2 * estimated_acceleration)
    print(f"Estimated stopping distance: {stopping_distance:.3f}m")

def validate_simulation():
    """
    Perform comprehensive validation of the physics simulation
    """
    print("Physics Simulation Validation")
    print("="*50)
    
    # Test 1: Verify gravity with ball drop
    print("\nTest 1: Gravity Verification")
    height = 1.0  # Drop from 1m
    theoretical_time = calculate_fall_time(height)
    impact_velocity = calculate_impact_velocity(height)
    
    print(f"Object dropped from {height}m:")
    print(f"  Theoretical fall time: {theoretical_time:.3f}s")
    print(f"  Theoretical impact velocity: {impact_velocity:.3f}m/s")
    print("  To validate: Drop an object from 1m and measure time to impact")
    
    # Test 2: Mass independence
    print("\nTest 2: Mass Independence")
    print("  Drop objects of different masses from same height")
    print("  All should fall in approximately the same time")
    
    # Test 3: Friction validation
    print("\nTest 3: Friction Validation")
    print("  Objects on high-friction surface should stop quickly")
    print("  Objects on low-friction surface should slide longer")
    
    print("\n" + "="*50)
    print("Perform these tests in Gazebo and compare results")
    print("Adjust physics parameters if simulation doesn't match expectations")

if __name__ == "__main__":
    validate_simulation()
    
    # Instructions for user
    print("\n\nHow to use this validation:")
    print("1. Run the Gazebo simulation with test_world.sdf")
    print("2. Use the GUI to drop objects from known heights")
    print("3. Use the test_params.py script to calculate expected values")
    print("4. Compare simulation behavior with theoretical values")
    print("5. Adjust physical properties in URDF files as needed")
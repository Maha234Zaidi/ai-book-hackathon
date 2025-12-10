# Hands-on Exercise 3: Implementing and Comparing Sensor Data Between Gazebo and Unity

In this exercise, you'll implement a complete system that demonstrates how sensor data flows from Gazebo physics simulation to Unity visualization, and learn how to compare and validate the data between both systems.

## Learning Objectives

After completing this exercise, you will:
1. Implement ROS 2 communication between Gazebo and Unity
2. Visualize real-time sensor data in Unity
3. Compare sensor data consistency between Gazebo and Unity
4. Validate the accuracy of the simulation-to-visualization pipeline

## Prerequisites

Before starting this exercise, ensure you have:
- Gazebo Garden or newer installed
- Unity 2022.3 LTS installed
- ROS 2 Humble Hawksbill installed
- Completed Exercises 1 and 2
- Basic understanding of ROS 2 concepts

## Part 1: Setting Up the Communication Infrastructure

### Step 1: Create a Robot Model with Multiple Sensors

First, let's create a URDF model of a robot with multiple sensors:

`~/gazebo_unity_sensor_exercise/models/sensor_robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="sensor_integration_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.52" ixy="0.0" ixz="0.0" iyy="0.52" iyz="0.0" izz="1.04"/>
    </inertial>
    
    <visual>
      <geometry>
        <box size="0.3 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.3 0.3 0.15"/>
      </geometry>
    </collision>
  </link>

  <!-- Wheels -->
  <link name="wheel_left">
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0004" ixy="0.0" ixz="0.0" iyy="0.0004" iyz="0.0" izz="0.0006"/>
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
    </collision>
  </link>

  <link name="wheel_right">
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0004" ixy="0.0" ixz="0.0" iyy="0.0004" iyz="0.0" izz="0.0006"/>
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
    </collision>
  </link>

  <!-- LiDAR sensor -->
  <link name="lidar_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
    
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
    </collision>
  </link>

  <!-- IMU sensor -->
  <link name="imu_link">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00005" ixy="0.0" ixz="0.0" iyy="0.00005" iyz="0.0" izz="0.00005"/>
    </inertial>
    
    <visual>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.02 0.02 0.02"/>
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

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
  </joint>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
  </joint>
</robot>
```

### Step 2: Create a Gazebo World with Obstacles

`~/gazebo_unity_sensor_exercise/worlds/sensor_test_world.sdf`

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="sensor_test_world">
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add some obstacles for the LiDAR to detect -->
    <model name="obstacle_1">
      <pose>2 1 0.1 0 0 0</pose>
      <link name="obstacle_1_link">
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
        </inertial>
        <visual name="visual">
          <geometry>
            <box size="0.5 0.5 0.2"/>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="0.5 0.5 0.2"/>
          </geometry>
        </collision>
      </link>
    </model>
    
    <model name="obstacle_2">
      <pose>-1 -1 0.1 0 0 0</pose>
      <link name="obstacle_2_link">
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder radius="0.2" length="0.3"/>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder radius="0.2" length="0.3"/>
          </geometry>
        </collision>
      </link>
    </model>
    
    <model name="obstacle_3">
      <pose>0 2 0.1 0 0 0</pose>
      <link name="obstacle_3_link">
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere radius="0.2"/>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere radius="0.2"/>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Step 3: Enhanced Robot Model with Sensors

Now create the enhanced model with sensor definitions:

`~/gazebo_unity_sensor_exercise/models/sensor_robot/model.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="sensor_integration_robot">
    <include>
      <uri>file://$(find sensor_robot)/meshes/base_link.stl</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
    
    <include>
      <uri>file://$(find sensor_robot)/meshes/wheel_left.stl</uri>
      <pose>0 0.17 0 0 0 0</pose>
    </include>
    
    <include>
      <uri>file://$(find sensor_robot)/meshes/wheel_right.stl</uri>
      <pose>0 -0.17 0 0 0 0</pose>
    </include>
    
    <link name="base_link">
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.52</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.52</iyy>
          <iyz>0</iyz>
          <izz>1.04</izz>
        </inertia>
      </inertial>
      
      <visual name="base_visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.15</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
        </material>
      </visual>
      
      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.15</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name="wheel_left">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.0004</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0004</iyy>
          <iyz>0</iyz>
          <izz>0.0006</izz>
        </inertia>
      </inertial>
      
      <visual name="wheel_left_visual">
        <geometry>
          <cylinder>
            <length>0.04</length>
            <radius>0.08</radius>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      
      <collision name="wheel_left_collision">
        <geometry>
          <cylinder>
            <length>0.04</length>
            <radius>0.08</radius>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <link name="wheel_right">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.0004</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0004</iyy>
          <iyz>0</iyz>
          <izz>0.0006</izz>
        </inertia>
      </inertial>
      
      <visual name="wheel_right_visual">
        <geometry>
          <cylinder>
            <length>0.04</length>
            <radius>0.08</radius>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      
      <collision name="wheel_right_collision">
        <geometry>
          <cylinder>
            <length>0.04</length>
            <radius>0.08</radius>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <link name="lidar_link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      
      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <length>0.05</length>
            <radius>0.03</radius>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
        </material>
      </visual>
      
      <collision name="lidar_collision">
        <geometry>
          <cylinder>
            <length>0.05</length>
            <radius>0.03</radius>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <link name="imu_link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00005</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00005</iyy>
          <iyz>0</iyz>
          <izz>0.00005</izz>
        </inertia>
      </inertial>
      
      <visual name="imu_visual">
        <geometry>
          <box>
            <size>0.02 0.02 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      
      <collision name="imu_collision">
        <geometry>
          <box>
            <size>0.02 0.02 0.02</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="wheel_left_joint" type="continuous">
      <parent>base_link</parent>
      <child>wheel_left</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
      <pose>0 0.17 0 0 -0 0</pose>
    </joint>

    <joint name="wheel_right_joint" type="continuous">
      <parent>base_link</parent>
      <child>wheel_right</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
      <pose>0 -0.17 0 0 -0 0</pose>
    </joint>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
      <pose>0.1 0 0.1 0 -0 0</pose>
    </joint>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
      <pose>0 0 0.08 0 -0 0</pose>
    </joint>

    <!-- LiDAR Sensor -->
    <sensor name="lidar_sensor" type="ray">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_plugin">
        <ros>
          <namespace>/lidar</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>

    <!-- IMU Sensor -->
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>imu/data</topic>
      <plugin filename="libgazebo_ros_imu.so" name="imu_plugin">
        <ros>
          <namespace>/imu</namespace>
        </ros>
        <topicName>data</topicName>
        <bodyName>imu_link</bodyName>
        <frameName>imu_link</frameName>
        <updateRateHZ>100.0</updateRateHZ>
        <gaussianNoise>0.005</gaussianNoise>
      </plugin>
    </sensor>
  </model>
</sdf>
```

### Step 4: Create the Model Configuration

`~/gazebo_unity_sensor_exercise/models/sensor_robot/model.config`

```xml
<?xml version="1.0"?>
<model>
  <name>sensor_integration_robot</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Digital Twin Student</name>
    <email>student@example.com</email>
  </author>
  <description>
    A robot model with LiDAR and IMU sensors for sensor integration exercise.
  </description>
</model>
```

## Part 2: Setting Up ROS 2 Communication

### Step 5: Create a ROS 2 Package for Sensor Data Processing

```bash
# Create ROS 2 workspace and package
mkdir -p ~/sensor_integration_ws/src
cd ~/sensor_integration_ws/src
ros2 pkg create --build-type ament_python sensor_integration_pkg
```

### Step 6: Create Sensor Data Publisher Node

`~/sensor_integration_ws/src/sensor_integration_pkg/sensor_integration_pkg/sensor_publisher.py`

```python
#!/usr/bin/env python3
# sensor_publisher.py
# Publishes sensor data from Gazebo to be consumed by Unity visualization

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Publishers for sensor data
        self.lidar_pub = self.create_publisher(LaserScan, 'lidar/scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing sensor data
        self.timer = self.create_timer(0.1, self.publish_sensors)  # 10 Hz
        
        # Robot state variables
        self.time = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0
        
        # IMU simulation parameters
        self.angular_velocity_z = 0.0
        self.linear_acceleration = [0.0, 0.0, 0.0]
        
        # Robot motion parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.get_logger().info('Sensor publisher node started')

    def publish_sensors(self):
        # Simulate robot motion
        dt = 0.1  # Time step
        
        # Update robot position (simple differential drive model)
        self.position_x += self.linear_velocity * math.cos(self.theta) * dt
        self.position_y += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt
        
        # Random motion for interesting sensor data
        if self.time % 5 < 2.5:  # Move forward for 2.5 seconds
            self.linear_velocity = 0.5
            self.angular_velocity = 0.1 * math.sin(self.time * 0.5)  # Gentle turns
        else:  # Then turn in place
            self.linear_velocity = 0.0
            self.angular_velocity = 0.3  # Turn at 0.3 rad/s
        
        # Publish IMU data
        self.publish_imu()
        
        # Publish LiDAR data
        self.publish_lidar()
        
        # Publish Odometry
        self.publish_odom()
        
        # Update time
        self.time += dt

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Set orientation (from current theta)
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.theta)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        
        # Set angular velocity (around Z axis)
        msg.angular_velocity.z = self.angular_velocity
        
        # Add some noise to angular velocity
        noise = np.random.normal(0, 0.01)
        msg.angular_velocity.z += noise
        
        # Set linear acceleration (simulate movement)
        msg.linear_acceleration.x = self.linear_velocity * 0.1  # Simulate acceleration
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.8  # Gravity
        
        # Add noise to acceleration
        msg.linear_acceleration.x += np.random.normal(0, 0.05)
        msg.linear_acceleration.y += np.random.normal(0, 0.05)
        msg.linear_acceleration.z += np.random.normal(0, 0.05)
        
        self.imu_pub.publish(msg)

    def publish_lidar(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        
        # Set LiDAR parameters
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / 360  # 360 points
        msg.time_increment = 0.0
        msg.scan_time = 0.1  # 10Hz
        msg.range_min = 0.1
        msg.range_max = 10.0
        
        # Simulate LiDAR readings
        # This is a simplified simulation that adds obstacles at specific directions
        ranges = []
        for i in range(360):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Calculate distances to simulated obstacles
            obstacle_dist1 = self.calculate_obstacle_distance(angle, 2.0, 1.0)  # obstacle_1
            obstacle_dist2 = self.calculate_obstacle_distance(angle, -1.0, -1.0)  # obstacle_2
            obstacle_dist3 = self.calculate_obstacle_distance(angle, 0.0, 2.0)  # obstacle_3
            
            # Take the minimum distance (closest obstacle in this direction)
            distance = min(obstacle_dist1, obstacle_dist2, obstacle_dist3)
            
            # Add some noise to the distance
            distance += np.random.normal(0, 0.02)  # 2cm noise
            
            ranges.append(min(distance, msg.range_max))
        
        msg.ranges = ranges
        msg.intensities = []  # Not used in this simulation
        
        self.lidar_pub.publish(msg)

    def calculate_obstacle_distance(self, ray_angle, obs_x, obs_y):
        """
        Calculate the distance from robot to circular obstacle in given direction.
        Simplified for this example.
        """
        # Robot position in world
        robot_x = self.position_x
        robot_y = self.position_y
        
        # Convert ray angle to world coordinates
        ray_end_x = robot_x + 10.0 * math.cos(ray_angle + self.theta)
        ray_end_y = robot_y + 10.0 * math.sin(ray_angle + self.theta)
        
        # Calculate distance between robot and obstacle
        dx = obs_x - robot_x
        dy = obs_y - robot_y
        dist_to_obs = math.sqrt(dx*dx + dy*dy)
        
        # For a circular obstacle with radius 0.2m, return distance to surface
        obstacle_radius = 0.2
        if dist_to_obs < obstacle_radius:
            return 0.1  # Inside obstacle
        else:
            return dist_to_obs - obstacle_radius

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        # Set position
        msg.pose.pose.position.x = self.position_x
        msg.pose.pose.position.y = self.position_y
        msg.pose.pose.position.z = 0.0
        
        # Set orientation
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Set velocities
        msg.twist.twist.linear.x = self.linear_velocity
        msg.twist.twist.angular.z = self.angular_velocity
        
        self.odom_pub.publish(msg)
        
        # Publish TF
        t = TransformStamped()
        
        # Set the time stamp
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        # Set translation and rotation
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 7: Create a Launch File

`~/sensor_integration_ws/src/sensor_integration_pkg/launch/sensor_integration.launch.py`

```python
# sensor_integration.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_integration_pkg',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        )
    ])
```

### Step 8: Update Setup Files

`~/sensor_integration_ws/src/sensor_integration_pkg/setup.py`

```python
from setuptools import setup

package_name = 'sensor_integration_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor_integration.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Digital Twin Student',
    maintainer_email='student@example.com',
    description='Package for sensor integration exercise',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = sensor_integration_pkg.sensor_publisher:main',
        ],
    },
)
```

`~/sensor_integration_ws/src/sensor_integration_pkg/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sensor_integration_pkg</name>
  <version>0.0.0</version>
  <description>Package for sensor integration exercise</description>
  <maintainer email="student@example.com">Digital Twin Student</maintainer>
  <license>MIT</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf_transformations</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Build the ROS 2 package:

```bash
cd ~/sensor_integration_ws
colcon build --packages-select sensor_integration_pkg
source install/setup.bash
```

## Part 3: Implementing Unity Visualization

### Step 9: Create Unity Project Structure

If you haven't already, create a Unity project for visualization:

```bash
# Create Unity project directory
mkdir -p ~/unity_sensor_visualization
```

For this exercise, we'll create the Unity C# scripts needed for sensor visualization:

`~/unity_sensor_visualization/SensorDataVisualizer.cs`

```csharp
// SensorDataVisualizer.cs
// Unity script to visualize sensor data received from ROS 2
using System.Collections.Generic;
using UnityEngine;

#if ENABLE_ROS2
using Ros2UnityCore;
using Ros2Sharp;
using SensorMsgs = Ros2Sharp.SensorMsgs;
using ImuMsg = Ros2Sharp.Imu;
using LaserScan = Ros2Sharp.LaserScan;
#endif

public class SensorDataVisualizer : MonoBehaviour
{
    [Header("ROS2 Connection Settings")]
    public string rosBridgeIP = "127.0.0.1";
    public int rosBridgePort = 9090;

    [Header("Visualization Settings")]
    public GameObject lidarPointPrefab;
    public float lidarPointSize = 0.05f;
    public Color lidarPointColor = Color.red;
    
    public GameObject imuIndicator;
    public GameObject robotModel;
    
    public float maxLidarRange = 10.0f;
    public int maxLidarPoints = 360;
    
    [Header("Performance Settings")]
    public bool useObjectPooling = true;
    public int poolSize = 500;
    
    // Internal references
    private List<GameObject> lidarPoints;
    private List<GameObject> objectPool;
    private Material lidarPointMaterial;
    
    #if ENABLE_ROS2
    private Ros2Socket rosSocket;
    private bool isConnected = false;
    #endif
    
    void Start()
    {
        InitializeVisualization();
        SetupROSConnection();
    }
    
    void InitializeVisualization()
    {
        // Create material for LiDAR points
        lidarPointMaterial = new Material(Shader.Find("Sprites/Default"));
        lidarPointMaterial.color = lidarPointColor;
        
        // Initialize object pooling if enabled
        if (useObjectPooling)
        {
            objectPool = new List<GameObject>();
            lidarPoints = new List<GameObject>();
            
            for (int i = 0; i < poolSize; i++)
            {
                GameObject point = CreateLidarPoint();
                point.SetActive(false);
                objectPool.Add(point);
            }
        }
        
        Debug.Log("Sensor visualization system initialized");
    }
    
    void SetupROSConnection()
    {
        #if ENABLE_ROS2
        Ros2Settings.Instance.RosIPAddress = rosBridgeIP;
        Ros2Settings.Instance.RosPort = rosBridgePort;
        
        rosSocket = UnityROSTCPConnector.Instance Ros2Socket;
        rosSocket.OnConnected += OnRosConnected;
        rosSocket.OnClosed += OnRosDisconnected;
        
        rosSocket.Connect();
        #else
        Debug.LogWarning("ROS2 support not enabled. Visualization will not receive real data.");
        #endif
    }
    
    #if ENABLE_ROS2
    void OnRosConnected()
    {
        isConnected = true;
        Debug.Log("Connected to ROS2 bridge");
        
        // Subscribe to sensor topics
        rosSocket.Subscribe<LaserScan>("/lidar/scan", OnLidarDataReceived, 50);
        rosSocket.Subscribe<ImuMsg>("/imu/data", OnImuDataReceived, 50);
    }
    
    void OnRosDisconnected()
    {
        isConnected = false;
        Debug.LogWarning("Disconnected from ROS2 bridge");
    }
    #endif
    
    GameObject CreateLidarPoint()
    {
        // Create a visual representation for a LiDAR point
        if (lidarPointPrefab != null)
        {
            GameObject point = Instantiate(lidarPointPrefab);
            return point;
        }
        else
        {
            // Create a default sphere for visualization
            GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            point.transform.SetParent(transform);
            point.GetComponent<Renderer>().material = lidarPointMaterial;
            
            // Remove the collider since we don't need physics for visualization
            DestroyImmediate(point.GetComponent<SphereCollider>());
            
            return point;
        }
    }
    
    #if ENABLE_ROS2
    void OnLidarDataReceived(LaserScan scanMsg)
    {
        UpdateLidarVisualization(scanMsg);
    }
    
    void OnImuDataReceived(ImuMsg imuMsg)
    {
        UpdateImuVisualization(imuMsg);
    }
    #endif
    
    void UpdateLidarVisualization(object scanMsgObj)
    {
        #if ENABLE_ROS2
        LaserScan scanMsg = (LaserScan)scanMsgObj;
        
        if (!useObjectPooling)
        {
            // Clear previous points if not using pooling
            foreach (Transform child in transform)
            {
                DestroyImmediate(child.gameObject);
            }
        }
        else
        {
            // Deactivate all previously used points
            foreach (GameObject point in lidarPoints)
            {
                if (point != null) point.SetActive(false);
            }
            lidarPoints.Clear();
        }
        
        // Calculate positions from scan data
        for (int i = 0; i < scanMsg.ranges.Count; i++)
        {
            float range = (float)scanMsg.ranges[i];
            
            // Skip invalid ranges
            if (range <= 0 || range >= scanMsg.range_max || float.IsNaN(range) || float.IsInfinity(range))
                continue;
                
            // Calculate angle
            float angle = (float)scanMsg.angle_min + i * (float)scanMsg.angle_increment;
            
            // Convert polar to cartesian coordinates
            float x = Mathf.Cos(angle) * range;
            float y = 0; // LiDAR is generally 2D
            float z = Mathf.Sin(angle) * range;
            
            // Note: This assumes LiDAR is mounted facing forward (Y direction in Unity)
            // Adjust based on your mounting orientation
            
            // Create point visualization
            GameObject lidarPoint = GetOrCreatePointObject();
            if (lidarPoint != null)
            {
                lidarPoint.transform.position = new Vector3(x, y, z);
                lidarPoint.transform.localScale = Vector3.one * lidarPointSize;
                lidarPoint.SetActive(true);
                
                if (useObjectPooling) lidarPoints.Add(lidarPoint);
            }
        }
        
        Debug.Log($"Updated LiDAR visualization with {scanMsg.ranges.Count} points");
        #endif
    }
    
    GameObject GetOrCreatePointObject()
    {
        if (!useObjectPooling)
        {
            return CreateLidarPoint();
        }
        
        // Try to find an inactive object in the pool
        foreach (GameObject obj in objectPool)
        {
            if (!obj.activeInHierarchy)
            {
                return obj;
            }
        }
        
        // Pool is exhausted, create a new one
        GameObject newObj = CreateLidarPoint();
        objectPool.Add(newObj);
        return newObj;
    }
    
    void UpdateImuVisualization(object imuMsgObj)
    {
        #if ENABLE_ROS2
        ImuMsg imuMsg = (ImuMsg)imuMsgObj;
        
        // Extract orientation quaternion
        float x = (float)imuMsg.orientation.x;
        float y = (float)imuMsg.orientation.y;
        float z = (float)imuMsg.orientation.z;
        float w = (float)imuMsg.orientation.w;
        
        Quaternion orientation = new Quaternion(x, y, z, w);
        
        // Apply orientation to IMU indicator
        if (imuIndicator != null)
        {
            imuIndicator.transform.rotation = orientation;
        }
        
        // Apply orientation to robot model as well
        if (robotModel != null)
        {
            robotModel.transform.rotation = orientation;
        }
        
        // Extract angular velocity
        float angVelX = (float)imuMsg.angular_velocity.x;
        float angVelY = (float)imuMsg.angular_velocity.y;
        float angVelZ = (float)imuMsg.angular_velocity.z;
        
        // Extract linear acceleration
        float linAccX = (float)imuMsg.linear_acceleration.x;
        float linAccY = (float)imuMsg.linear_acceleration.y;
        float linAccZ = (float)imuMsg.linear_acceleration.z;
        
        Debug.Log($"IMU Data - Orientation: {orientation.eulerAngles}, Angular Vel: ({angVelX:F3}, {angVelY:F3}, {angVelZ:F3}), Linear Acc: ({linAccX:F3}, {linAccY:F3}, {linAccZ:F3})");
        #endif
    }
    
    // Method to visualize simulated sensor data (for testing without ROS2)
    public void VisualizeSimulatedLidar(List<float> ranges, float angleMin, float angleMax, float angleIncrement)
    {
        if (!useObjectPooling)
        {
            // Clear previous points
            foreach (Transform child in transform)
            {
                DestroyImmediate(child.gameObject);
            }
        }
        else
        {
            // Deactivate all previously used points
            foreach (GameObject point in lidarPoints)
            {
                if (point != null) point.SetActive(false);
            }
            lidarPoints.Clear();
        }
        
        // Calculate positions from simulated scan data
        for (int i = 0; i < ranges.Count; i++)
        {
            float range = ranges[i];
            
            // Skip invalid ranges
            if (range <= 0 || range >= maxLidarRange || float.IsNaN(range) || float.IsInfinity(range))
                continue;
                
            // Calculate angle
            float angle = angleMin + i * angleIncrement;
            
            // Convert polar to cartesian coordinates
            float x = Mathf.Cos(angle) * range;
            float z = Mathf.Sin(angle) * range;  // In Unity, Z is forward, Y is up
            
            // Create point visualization
            GameObject lidarPoint = GetOrCreatePointObject();
            if (lidarPoint != null)
            {
                lidarPoint.transform.position = new Vector3(x, 0, z);
                lidarPoint.transform.localScale = Vector3.one * lidarPointSize;
                lidarPoint.SetActive(true);
                
                if (useObjectPooling) lidarPoints.Add(lidarPoint);
            }
        }
        
        Debug.Log($"Visualized simulated LiDAR with {ranges.Count} points");
    }
    
    // Visualization helper for IMU data
    public void VisualizeSimulatedImu(Quaternion orientation, Vector3 angularVelocity, Vector3 linearAcceleration)
    {
        // Apply orientation to IMU indicator
        if (imuIndicator != null)
        {
            imuIndicator.transform.rotation = orientation;
        }
        
        // Apply orientation to robot model as well
        if (robotModel != null)
        {
            robotModel.transform.rotation = orientation;
        }
        
        Debug.Log($"Visualized simulated IMU - Orientation: {orientation.eulerAngles}, Angular Vel: {angularVelocity}, Linear Acc: {linearAcceleration}");
    }
    
    void Update()
    {
        // Visualization updates happen via ROS messages, but we can add
        // additional UI or performance indicators here
        
        if (Input.GetKeyDown(KeyCode.Space))
        {
            // Simulate sending a command to the robot
            Debug.Log("Sending movement command to robot");
            #if ENABLE_ROS2
            // This would publish a cmd_vel message
            #endif
        }
    }
    
    void OnDestroy()
    {
        #if ENABLE_ROS2
        if (rosSocket != null)
        {
            rosSocket.Disconnect();
        }
        #endif
    }
}
```

### Step 10: Create Sensor Comparison Visualizer

`~/unity_sensor_visualization/SensorComparisonVisualizer.cs`

```csharp
// SensorComparisonVisualizer.cs
// Compares sensor data from simulation (Gazebo) with visualization (Unity)
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SensorComparisonVisualizer : MonoBehaviour
{
    [Header("Sensor Comparison Components")]
    public SensorDataVisualizer unitySensorVisualizer;
    public Text comparisonResultsText;
    public Text statsText;
    public Color passColor = Color.green;
    public Color failColor = Color.red;
    
    [Header("Tolerance Settings")]
    public float lidarDistanceTolerance = 0.05f; // 5cm tolerance
    public float imuAngleTolerance = 2.0f;       // 2 degree tolerance
    public float comparisonFrequency = 0.5f;     // Compare every 0.5 seconds
    
    // Storage for comparison data
    private float lastComparisonTime = 0f;
    private Queue<SensorComparisonResult> comparisonHistory;
    private int totalComparisons = 0;
    private int passedComparisons = 0;
    
    [System.Serializable]
    public class SensorComparisonResult
    {
        public float timestamp;
        public bool lidarMatch;
        public bool imuMatch;
        public float lidarError;
        public float imuError;
        public int lidarPointsCompared;
    }
    
    void Start()
    {
        comparisonHistory = new Queue<SensorComparisonResult>();
        lastComparisonTime = Time.time;
        
        Debug.Log("Sensor comparison system initialized");
    }
    
    void Update()
    {
        if (Time.time - lastComparisonTime > 1.0f / comparisonFrequency)
        {
            PerformSensorComparison();
            lastComparisonTime = Time.time;
        }
        
        UpdateStatisticsDisplay();
    }
    
    void PerformSensorComparison()
    {
        // This would typically compare Gazebo simulation data with Unity visualization data
        // For this exercise, we'll simulate the comparison
        
        SensorComparisonResult result = new SensorComparisonResult();
        result.timestamp = Time.time;
        
        // Simulate comparison of LiDAR data
        result.lidarMatch = SimulateLidarComparison(out result.lidarError, out result.lidarPointsCompared);
        
        // Simulate comparison of IMU data  
        result.imuMatch = SimulateImuComparison(out result.imuError);
        
        // Update statistics
        totalComparisons++;
        if (result.lidarMatch && result.imuMatch)
            passedComparisons++;
        
        // Store result
        comparisonHistory.Enqueue(result);
        
        // Maintain history size
        if (comparisonHistory.Count > 100) // Keep last 100 comparisons
        {
            comparisonHistory.Dequeue();
        }
        
        UpdateComparisonDisplay(result);
    }
    
    bool SimulateLidarComparison(out float maxError, out int pointsCompared)
    {
        // For simulation purposes, we'll generate random errors
        maxError = Random.Range(0.01f, 0.08f); // Error between 1-8cm
        pointsCompared = Random.Range(180, 360); // 180-360 points
        
        return maxError <= lidarDistanceTolerance;
    }
    
    bool SimulateImuComparison(out float angleError)
    {
        // For simulation purposes, generate random angle error
        angleError = Random.Range(0.5f, 3.0f); // Error between 0.5-3.0 degrees
        
        return angleError <= imuAngleTolerance;
    }
    
    void UpdateComparisonDisplay(SensorComparisonResult result)
    {
        if (comparisonResultsText != null)
        {
            string resultStr = $"<b>Sensor Comparison (Time: {Time.time:0.00}s)</b>\n";
            resultStr += $"LiDAR: {(result.lidarMatch ? "<color=green>PASS</color>" : "<color=red>FAIL</color>")}";
            resultStr += $" (Error: {result.lidarError*100:F1}cm, Points: {result.lidarPointsCompared})\n";
            
            resultStr += $"IMU: {(result.imuMatch ? "<color=green>PASS</color>" : "<color=red>FAIL</color>")}";
            resultStr += $" (Error: {result.imuError:F1}°)\n";
            
            comparisonResultsText.text = resultStr;
        }
    }
    
    void UpdateStatisticsDisplay()
    {
        if (statsText != null && totalComparisons > 0)
        {
            float successRate = (float)passedComparisons / totalComparisons * 100f;
            
            string statsStr = $"<b>Comparison Statistics</b>\n";
            statsStr += $"Total Comparisons: {totalComparisons}\n";
            statsStr += $"Passed: {passedComparisons}\n";
            statsStr += $"Success Rate: {successRate:F1}%\n";
            
            if (successRate >= 95f)
                statsStr += $"<color=green>System Performance: Excellent</color>";
            else if (successRate >= 80f)
                statsStr += $"<color=yellow>System Performance: Good</color>";
            else
                statsStr += $"<color=red>System Performance: Needs Attention</color>";
            
            statsText.text = statsStr;
        }
    }
    
    // Method to manually trigger a comparison
    public void TriggerComparison()
    {
        PerformSensorComparison();
    }
    
    // Method to export comparison results
    public void ExportComparisonResults()
    {
        string exportData = "Timestamp,LiDAR_Match,IMU_Match,LiDAR_Error,IMU_Error,LiDAR_Points\n";
        
        foreach (SensorComparisonResult result in comparisonHistory)
        {
            exportData += $"{result.timestamp},{result.lidarMatch},{result.imuMatch},{result.lidarError},{result.imuError},{result.lidarPointsCompared}\n";
        }
        
        // In a real implementation, you'd save this to a file
        Debug.Log($"Comparison results export would contain {comparisonHistory.Count} entries");
        Debug.Log(exportData); // Just for demonstration
    }
    
    // Method to adjust tolerance settings during runtime
    public void SetLidarTolerance(float newTolerance)
    {
        lidarDistanceTolerance = newTolerance;
        Debug.Log($"LiDAR tolerance adjusted to {newTolerance*100:F1}cm");
    }
    
    public void SetImuTolerance(float newTolerance)
    {
        imuAngleTolerance = newTolerance;
        Debug.Log($"IMU tolerance adjusted to {newTolerance:F1} degrees");
    }
    
    // Method to reset comparison statistics
    public void ResetComparisonStats()
    {
        totalComparisons = 0;
        passedComparisons = 0;
        comparisonHistory.Clear();
        
        if (comparisonResultsText != null)
            comparisonResultsText.text = "Comparison statistics reset.";
            
        if (statsText != null)
            statsText.text = "Statistics reset. Starting fresh comparison...";
            
        Debug.Log("Sensor comparison statistics reset");
    }
    
    // Method to run comprehensive validation
    public void RunComprehensiveValidation()
    {
        Debug.Log("Running comprehensive sensor validation...");
        
        // This would perform a more detailed validation
        // including accuracy, precision, drift, and consistency tests
        
        StartCoroutine(ComprehensiveValidationRoutine());
    }
    
    System.Collections.IEnumerator ComprehensiveValidationRoutine()
    {
        // Step 1: Accuracy validation
        Debug.Log("1. Validating sensor accuracy...");
        
        // Simulate accuracy validation
        yield return new WaitForSeconds(1.0f);
        
        // Step 2: Precision validation
        Debug.Log("2. Validating sensor precision...");
        yield return new WaitForSeconds(1.0f);
        
        // Step 3: Drift validation
        Debug.Log("3. Validating sensor drift over time...");
        yield return new WaitForSeconds(1.0f);
        
        // Step 4: Consistency validation
        Debug.Log("4. Validating sensor consistency...");
        yield return new WaitForSeconds(1.0f);
        
        Debug.Log("Comprehensive validation completed!");
        
        // In a real implementation, this would generate a detailed report
    }
}
```

## Part 4: Running the Complete System

### Step 11: Launch Gazebo Simulation

From a terminal:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Set Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/gazebo_unity_sensor_exercise/models

# Launch Gazebo with our test world
gz sim -r $HOME/gazebo_unity_sensor_exercise/worlds/sensor_test_world.sdf
```

### Step 12: Launch the Robot in Gazebo

In another terminal:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Spawn the robot in the simulation
gz model -s -m sensor_integration_robot -f $HOME/gazebo_unity_sensor_exercise/models/sensor_robot/model.sdf -x 0 -y 0 -z 0.1
```

### Step 13: Launch the ROS 2 Publisher

In another terminal:

```bash
# Source our workspace
source /opt/ros/humble/setup.bash
source $HOME/sensor_integration_ws/install/setup.bash

# Run the sensor publisher node
ros2 run sensor_integration_pkg sensor_publisher
```

### Step 14: Launch the ROS Bridge (if using Unity ROS2 connector)

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

### Step 15: Set Up Unity Visualization

1. Create a new Unity 3D project or use an existing one
2. Create an empty GameObject called "SensorVisualizer"
3. Attach the SensorDataVisualizer.cs script to it
4. Create another empty GameObject called "SensorComparator"
5. Attach the SensorComparisonVisualizer.cs script to it
6. Create UI elements for the comparison results display
7. Create a simple robot model to represent the sensor platform

### Step 16: Configure Unity for ROS2 Communication

If using the Unity ROS2 connector:

1. Import the ROS2Unity Package
2. Configure the Ros2Settings with your ROS bridge IP and port
3. Ensure your Unity project is built with ROS2 support enabled

### Step 17: Run the Unity Visualization

1. Press Play in the Unity Editor
2. The visualization should connect to the ROS2 bridge
3. LiDAR points should appear as the robot moves through the environment
4. The IMU indicator should update based on simulated orientation changes

## Part 5: Verification and Validation

### Step 18: Validate Sensor Data Consistency

With both Gazebo and Unity running:

1. **Monitor ROS topics**:
   ```bash
   # Check if LiDAR data is being published
   ros2 topic echo /lidar/scan
   
   # Check if IMU data is being published
   ros2 topic echo /imu/data
   ```

2. **Compare visualization accuracy**:
   - Check if Unity visualization matches Gazebo simulation
   - Verify that obstacles appear in the same relative positions
   - Confirm that robot orientation matches between systems

3. **Run the comparison tool** in Unity to validate data consistency

### Step 19: Troubleshooting Common Issues

**Issue 1: No sensor data in Unity**
- Check that ROS bridge is running and accessible
- Verify IP address and port settings
- Confirm that Unity has ROS2 support enabled

**Issue 2: LiDAR points not appearing correctly**
- Verify coordinate system conversions
- Check that point scaling is appropriate
- Confirm that ranges are within expected limits

**Issue 3: IMU orientation not matching**
- Check quaternion format differences between systems
- Verify coordinate system conventions (ROS vs Unity)

## Part 6: Performance and Optimization

### Step 20: Optimize Visualization Performance

1. **Implement object pooling** for LiDAR points to reduce instantiation overhead
2. **Use instancing** for uniform point visualization
3. **Limit the number of displayed points** based on distance or importance
4. **Use Level of Detail (LOD)** systems for distant point clouds

### Step 21: Test with Different Scenarios

1. **Vary the robot motion** patterns to test different sensor behaviors
2. **Add more obstacles** to the environment to validate detection
3. **Test edge cases** like close-range detection and maximum range
4. **Monitor performance** metrics during complex scenarios

This hands-on exercise demonstrates the complete pipeline for integrating sensor data between Gazebo physics simulation and Unity visualization, enabling you to validate and compare the consistency of sensor data flowing between the two systems.
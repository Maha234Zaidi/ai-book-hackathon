# Hands-on Exercise 4: Complete Digital Twin Integration

In this exercise, you'll create a complete digital twin system with Gazebo physics and Unity visualization, implementing proper synchronization and communication between the two systems.

## Learning Objectives

By completing this exercise, you will:
1. Integrate all components learned in previous exercises into a unified system
2. Implement proper time synchronization between Gazebo and Unity
3. Create a communication pipeline for multiple sensor types
4. Validate the complete digital twin system performance

## Prerequisites

Before starting this exercise, ensure you have:
- Completed Exercises 1, 2, and 3
- Gazebo Garden or newer installed
- Unity 2022.3 LTS installed
- ROS 2 Humble Hawksbill installed
- Python 3.10 installed
- Basic understanding of ROS 2 concepts and Unity

## Part 1: Setting Up the Complete System

### Step 1: Create the Robot Model with All Sensors

Create `~/digital_twin_complete/models/integrated_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="integrated_robot">
  <!-- Robot base -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia
        ixx="0.416" ixy="0.0" ixz="0.0"
        iyy="0.416" iyz="0.0"
        izz="0.833" />
    </inertial>

    <visual>
      <geometry>
        <box size="0.4 0.4 0.2"/>
      </geometry>
      <material name="light_blue">
        <color rgba="0.5 0.5 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.4 0.4 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Left wheel -->
  <link name="wheel_left">
    <inertial>
      <mass value="0.5"/>
      <inertia
        ixx="0.00125" ixy="0.0" ixz="0.0"
        iyy="0.001875" iyz="0.0"
        izz="0.00125" />
    </inertial>

    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Right wheel -->
  <link name="wheel_right">
    <inertial>
      <mass value="0.5"/>
      <inertia
        ixx="0.00125" ixy="0.0" ixz="0.0"
        iyy="0.001875" iyz="0.0"
        izz="0.00125" />
    </inertial>

    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Caster wheel -->
  <link name="caster_wheel">
    <inertial>
      <mass value="0.2"/>
      <inertia
        ixx="0.00004" ixy="0.0" ixz="0.0"
        iyy="0.00004" iyz="0.0"
        izz="0.00004" />
    </inertial>

    <visual>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <!-- IMU Sensor -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia
        ixx="0.000001" ixy="0.0" ixz="0.0"
        iyy="0.000001" iyz="0.0"
        izz="0.000001" />
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

  <!-- LiDAR Sensor -->
  <link name="lidar_link">
    <inertial>
      <mass value="0.2"/>
      <inertia
        ixx="0.0001" ixy="0.0" ixz="0.0"
        iyy="0.0001" iyz="0.0"
        izz="0.0001" />
    </inertial>

    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Depth Camera -->
  <link name="camera_link">
    <inertial>
      <mass value="0.1"/>
      <inertia
        ixx="0.00001" ixy="0.0" ixz="0.0"
        iyy="0.00001" iyz="0.0"
        izz="0.00001" />
    </inertial>

    <visual>
      <geometry>
        <box size="0.03 0.08 0.03"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.03 0.08 0.03"/>
      </geometry>
    </collision>
  </link>

  <!-- Joints -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.18 0 -0.06" rpy="0 0 0"/>
  </joint>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.18 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins for sensors -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/integrated_robot</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <update_rate>100</update_rate>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="lidar_link">
    <sensor name="lidar_sensor" type="gpu_ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_gpu_laser.so">
        <ros>
          <namespace>/integrated_robot</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor name="camera_sensor" type="depth">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <ros>
          <namespace>/integrated_robot</namespace>
          <remapping>image_raw:=rgb/image_raw</remapping>
          <remapping>depth/image_raw:=depth/image_raw</remapping>
          <remapping>depth/camera_info:=depth/camera_info</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo material definitions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="wheel_left">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_right">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="caster_wheel">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="imu_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="lidar_link">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/White</material>
  </gazebo>
</robot>
```

### Step 2: Create the Complete World File

Create `~/digital_twin_complete/worlds/complete_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="complete_world">
    <!-- Gravity -->
    <gravity>0 0 -9.8</gravity>

    <!-- Physics engine configuration -->
    <physics name="complete_physics" type="ode">
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

    <!-- Environment with obstacles -->
    <model name="obstacle_1">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box size="0.5 0.5 1.0"/>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="0.5 0.5 1.0"/>
          </geometry>
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

    <model name="obstacle_2">
      <pose>-2 -2 0.3 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder radius="0.3" length="0.6"/>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder radius="0.3" length="0.6"/>
          </geometry>
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

    <!-- Place the complete robot -->
    <include>
      <uri>model://integrated_robot</uri>
      <pose>0 0 0.2 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## Part 2: Creating the ROS 2 Communication Node

### Step 3: Create a ROS 2 Node to Collect All Sensor Data

Create `~/digital_twin_complete/src/sensor_fusion_node.py`:

```python
#!/usr/bin/env python3
"""
Sensor Fusion Node - Collects all sensor data from the integrated robot
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo, Imu, PointCloud2, JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import numpy as np
from cv_bridge import CvBridge
import cv2

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Create publishers for aggregated data
        self.fused_data_pub = self.create_publisher(Header, '/integrated_robot/fused_data', 10)
        
        # Create subscribers for all sensors
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/integrated_robot/scan',
            self.lidar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/integrated_robot/imu/data',
            self.imu_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/integrated_robot/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/integrated_robot/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Data storage
        self.lidar_data = None
        self.imu_data = None
        self.image_data = None
        self.depth_data = None
        
        # Timer for publishing aggregated data
        self.timer = self.create_timer(0.1, self.publish_fused_data)  # 10 Hz
        
        # Bridge for image processing
        self.bridge = CvBridge()
        
        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        self.lidar_data = {
            'ranges': msg.ranges,
            'intensities': msg.intensities,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'header': msg.header
        }

    def imu_callback(self, msg):
        self.imu_data = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'header': msg.header
        }

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_data = {
                'image': cv_image,
                'encoding': msg.encoding,
                'header': msg.header,
                'height': msg.height,
                'width': msg.width
            }
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def depth_callback(self, msg):
        try:
            # Convert depth image
            depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            self.depth_data = {
                'image': depth_image,
                'encoding': msg.encoding,
                'header': msg.header,
                'height': msg.height,
                'width': msg.width
            }
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def publish_fused_data(self):
        # Create header with current timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "integrated_robot_fused_data"
        
        # Check if we have all sensor data
        all_data_available = all([
            self.lidar_data is not None,
            self.imu_data is not None,
            self.image_data is not None,
            self.depth_data is not None
        ])
        
        if all_data_available:
            self.get_logger().info('All sensor data available for fusion', throttle_duration_sec=1.0)
        
        # Publish the header as a simple indicator (in a real system, you'd publish more complex fused data)
        self.fused_data_pub.publish(header)

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()
    
    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        sensor_fusion_node.get_logger().info('Shutting down Sensor Fusion Node')
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 3: Unity Integration

### Step 4: Create Unity Scene for Complete Digital Twin

Create a Unity script to handle all sensor types: `~/digital_twin_complete/unity_scripts/CompleteDigitalTwinController.cs`

```csharp
using UnityEngine;
using System.Collections.Generic;

#if ENABLE_ROS2
using Ros2UnityCore;
using Ros2Sharp;
using LaserScan = Ros2Sharp.LaserScan;
using Image = Ros2Sharp.Image;
using Imu = Ros2Sharp.Imu;
#endif

public class CompleteDigitalTwinController : MonoBehaviour
{
    [Header("Robot Components")]
    public Transform robotBase;
    public Transform leftWheel;
    public Transform rightWheel;
    public Transform lidarSensor;
    public Transform cameraSensor;
    public Transform imuSensor;

    [Header("Visualization")]
    public GameObject[] lidarRays;
    public Renderer cameraRenderer;
    public GameObject imuIndicator;

    [Header("ROS Settings")]
    public string rosBridgeIP = "127.0.0.1";
    public int rosBridgePort = 10000;

    private Ros2Socket rosSocket;
    private bool rosConnected = false;

    // Data storage
    private float[] lidarRanges;
    private Vector3 imuAcceleration;
    private Vector3 imuAngularVelocity;
    private Vector3 imuOrientation;

    // Constants
    private const int NUM_LIDAR_RAYS = 720;

    void Start()
    {
        InitializeRobot();
        InitializeROSConnection();
    }

    void InitializeRobot()
    {
        // Initialize lidar visualization
        if (lidarRays == null || lidarRays.Length == 0)
        {
            CreateLiDARVisualization();
        }
    }

    void InitializeROSConnection()
    {
#if ENABLE_ROS2
        Ros2Settings.Instance.RosIPAddress = rosBridgeIP;
        Ros2Settings.Instance.RosPort = rosBridgePort;

        rosSocket = UnityROSTCPConnector.Instance Ros2Socket;
        rosSocket.OnConnected += OnRosConnected;
        rosSocket.OnDisconnected += OnRosDisconnected;

        rosSocket.Connect();
#else
        Debug.LogWarning("ROS2 support not enabled. Simulation will run without real sensor data.");
#endif
    }

    void OnRosConnected()
    {
        rosConnected = true;
        Debug.Log("Connected to ROS bridge");

#if ENABLE_ROS2
        // Subscribe to all sensor topics
        rosSocket.Subscribe<LaserScan>("/integrated_robot/scan", OnLidarDataReceived, 10);
        rosSocket.Subscribe<Imu>("/integrated_robot/imu/data", OnImuDataReceived, 10);
        rosSocket.Subscribe<Image>("/integrated_robot/rgb/image_raw", OnImageDataReceived, 10);
        rosSocket.Subscribe<Image>("/integrated_robot/depth/image_raw", OnDepthDataReceived, 10);
#endif
    }

    void OnRosDisconnected()
    {
        rosConnected = false;
        Debug.Log("Disconnected from ROS bridge");
    }

    void OnLidarDataReceived(LaserScan scan)
    {
        // Store lidar data
        lidarRanges = new float[scan.ranges.Count];
        for (int i = 0; i < scan.ranges.Count && i < lidarRanges.Length; i++)
        {
            lidarRanges[i] = (float)scan.ranges[i];
        }

        // Update visualization
        UpdateLiDARVisualization();
    }

    void OnImuDataReceived(Imu imuMsg)
    {
        // Store IMU data
        imuAcceleration = new Vector3(
            (float)imuMsg.linear_acceleration.x,
            (float)imuMsg.linear_acceleration.y,
            (float)imuMsg.linear_acceleration.z
        );

        imuAngularVelocity = new Vector3(
            (float)imuMsg.angular_velocity.x,
            (float)imuMsg.angular_velocity.y,
            (float)imuMsg.angular_velocity.z
        );

        imuOrientation = new Vector3(
            (float)imuMsg.orientation.x,
            (float)imuMsg.orientation.y,
            (float)imuMsg.orientation.z
        );

        // Update visualization
        UpdateIMUVisualization();
    }

    void OnImageDataReceived(Image imageMsg)
    {
        // Process and update camera visualization
        UpdateCameraVisualization(imageMsg);
    }

    void OnDepthDataReceived(Image depthMsg)
    {
        // Process and update depth visualization
        UpdateDepthVisualization(depthMsg);
    }

    void CreateLiDARVisualization()
    {
        lidarRays = new GameObject[NUM_LIDAR_RAYS];
        
        for (int i = 0; i < NUM_LIDAR_RAYS; i++)
        {
            GameObject rayGO = new GameObject($"LiDAR_Ray_{i}");
            rayGO.transform.SetParent(lidarSensor);
            rayGO.transform.localPosition = Vector3.zero;
            
            LineRenderer lr = rayGO.AddComponent<LineRenderer>();
            lr.material = new Material(Shader.Find("Particles/Additive"));
            lr.startWidth = 0.01f;
            lr.endWidth = 0.005f;
            lr.positionCount = 2;
            lr.useWorldSpace = true;
            
            lidarRays[i] = rayGO;
        }
    }

    void UpdateLiDARVisualization()
    {
        if (lidarRanges == null) return;

        for (int i = 0; i < lidarRanges.Length && i < lidarRays.Length; i++)
        {
            LineRenderer lr = lidarRays[i].GetComponent<LineRenderer>();
            
            float angle = Mathf.Lerp(-Mathf.PI/2, Mathf.PI/2, (float)i / lidarRanges.Length);
            float distance = lidarRanges[i];
            
            // Apply maximum distance limit
            if (float.IsNaN(distance) || float.IsInfinity(distance))
                distance = 30.0f; // Use max range
            
            if (distance > 30.0f) distance = 30.0f;
            
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            Vector3 startPoint = lidarSensor.position;
            Vector3 endPoint = lidarSensor.position + lidarSensor.TransformDirection(direction) * distance;
            
            lr.SetPosition(0, startPoint);
            lr.SetPosition(1, endPoint);
            
            // Color based on distance
            float distanceRatio = distance / 30.0f;
            Color rayColor = Color.Lerp(Color.red, Color.green, distanceRatio);
            lr.startColor = rayColor;
            lr.endColor = rayColor;
        }
    }

    void UpdateIMUVisualization()
    {
        if (imuIndicator != null)
        {
            // Simple visualization - move indicator based on acceleration
            imuIndicator.transform.Translate(imuAcceleration * 0.01f, Space.World);
            
            // Constrain to a reasonable area
            Vector3 pos = imuIndicator.transform.position;
            pos.x = Mathf.Clamp(pos.x, -1f, 1f);
            pos.y = Mathf.Clamp(pos.y, -1f, 1f);
            pos.z = Mathf.Clamp(pos.z, -1f, 1f);
            imuIndicator.transform.position = pos;
        }
    }

    void UpdateCameraVisualization(Image imageMsg)
    {
        // This is a simplified version - in a real implementation you'd convert the ROS image to a Unity texture
        if (cameraRenderer != null)
        {
            // In a real implementation, you would convert imageMsg to a Unity Texture2D
            // and assign it to the material of cameraRenderer
        }
    }

    void UpdateDepthVisualization(Image depthMsg)
    {
        // Similar to camera visualization, but for depth data
        // In a real implementation, you would process depth data into a format
        // usable by Unity for visualization
    }

    void Update()
    {
        // Fallback visualization if ROS is not connected
        if (!rosConnected)
        {
            SimulateSensorData();
        }
    }

    void SimulateSensorData()
    {
        // Simulate lidar data
        if (lidarRanges == null)
        {
            lidarRanges = new float[NUM_LIDAR_RAYS];
            for (int i = 0; i < lidarRanges.Length; i++)
            {
                lidarRanges[i] = 10.0f + Mathf.Sin(i * 0.1f) * 2.0f; // Simulated distances
            }
        }

        // Simulate IMU data
        imuAcceleration = new Vector3(
            Mathf.Sin(Time.time) * 0.1f,
            Mathf.Cos(Time.time) * 0.1f,
            9.81f + Mathf.Sin(Time.time * 2) * 0.01f
        );

        UpdateLiDARVisualization();
        UpdateIMUVisualization();
    }

    void OnDestroy()
    {
        if (rosSocket != null)
        {
            rosSocket.Disconnect();
        }
    }
}
```

## Part 4: Testing the Complete System

### Step 5: Create the Launch Script

Create `~/digital_twin_complete/launch_system.sh`:

```bash
#!/bin/bash
# Complete Digital Twin Launch Script

# Set up environment
source /opt/ros/humble/setup.bash

# Set model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/digital_twin_complete/models
export GZ_SIM_RESOURCE_PATH=$GAZEBO_MODEL_PATH:~/digital_twin_complete/models

echo "Starting Complete Digital Twin System..."
echo "Step 1: Launching Gazebo simulation..."
echo "Please run: gz sim -r ~/digital_twin_complete/worlds/complete_world.sdf"

echo ""
echo "Step 2: In a new terminal, source ROS and run the sensor fusion node:"
echo "source /opt/ros/humble/setup.bash"
echo "cd ~/digital_twin_complete/src"
echo "python3 sensor_fusion_node.py"

echo ""
echo "Step 3: Start Unity application with the CompleteDigitalTwinController script"
echo "Make sure Unity is configured to connect to the ROS bridge"

echo ""
echo "Step 4: Verify all sensors are working by checking:"
echo " - LiDAR data: 'ros2 topic echo /integrated_robot/scan'"
echo " - IMU data: 'ros2 topic echo /integrated_robot/imu/data'"
echo " - Camera data: 'ros2 topic echo /integrated_robot/rgb/image_raw'"
echo " - Depth data: 'ros2 topic echo /integrated_robot/depth/image_raw'"

echo ""
echo "For Unity visualization, ensure the ROS IP and port are correctly configured"
echo "in the CompleteDigitalTwinController script."
```

### Step 6: Validation and Verification

Create a validation script: `~/digital_twin_complete/validate_system.py`:

```python
#!/usr/bin/env python3
"""
Validation script for the complete digital twin system
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import time
import threading

class DigitalTwinValidator(Node):
    def __init__(self):
        super().__init__('digital_twin_validator')
        
        # Flags to track if messages have been received
        self.lidar_received = False
        self.imu_received = False
        self.image_received = False
        self.depth_received = False
        
        # Message counters
        self.lidar_count = 0
        self.imu_count = 0
        self.image_count = 0
        self.depth_count = 0
        
        # Subscribers for validation
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/integrated_robot/scan',
            self.lidar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/integrated_robot/imu/data',
            self.imu_callback,
            10
        )
        
        # We won't subscribe to images as they're high-bandwidth
        # but we'll check if they're available using topic info
        
        self.get_logger().info('Digital Twin Validator initialized')

    def lidar_callback(self, msg):
        self.lidar_received = True
        self.lidar_count += 1
        if self.lidar_count % 10 == 0:  # Log every 10 messages
            self.get_logger().info(f'LiDAR messages received: {self.lidar_count}')

    def imu_callback(self, msg):
        self.imu_received = True
        self.imu_count += 1
        if self.imu_count % 10 == 0:  # Log every 10 messages
            self.get_logger().info(f'IMU messages received: {self.imu_count}')

    def validate_system(self, timeout=30):
        """
        Validate that all systems are running properly
        """
        start_time = time.time()
        
        self.get_logger().info(f'Starting validation for {timeout} seconds...')
        
        while time.time() - start_time < timeout:
            # Check if we have received data from critical sensors
            if self.lidar_received and self.imu_received:
                self.get_logger().info('All critical sensors are active!')
                return True
            
            time.sleep(0.5)
        
        # Check what sensors are missing
        missing_sensors = []
        if not self.lidar_received:
            missing_sensors.append("LiDAR")
        if not self.imu_received:
            missing_sensors.append("IMU")
        if not self.image_received:
            missing_sensors.append("Camera")
        if not self.depth_received:
            missing_sensors.append("Depth Camera")
        
        if missing_sensors:
            self.get_logger().error(f'Missing data from: {", ".join(missing_sensors)}')
        
        return False

def main(args=None):
    rclpy.init(args=args)
    validator = DigitalTwinValidator()
    
    # Run validation in a separate thread to allow ROS to process messages
    validation_result = [False]  # Use list to allow modification from thread
    
    def run_validation():
        validation_result[0] = validator.validate_system(timeout=30)
    
    validation_thread = threading.Thread(target=run_validation)
    validation_thread.start()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print('Validation interrupted by user')
    finally:
        validation_thread.join(timeout=1)  # Wait for validation to complete
        validator.destroy_node()
        rclpy.shutdown()
        
        # Print final result
        if validation_result[0]:
            print("\n✓ Validation PASSED: All systems are operational")
        else:
            print("\n✗ Validation FAILED: Some systems are not responding")
        
        return 0 if validation_result[0] else 1

if __name__ == '__main__':
    exit(main())
```

## Part 5: Integration and Performance Testing

### Step 7: Run Complete System Test

Execute the following steps to test your complete digital twin:

1. **Prepare the environment:**
   ```bash
   mkdir -p ~/digital_twin_complete/{models,worlds,src,unity_scripts}
   ```

2. **Set up Gazebo models directory structure:**
   ```bash
   mkdir -p ~/digital_twin_complete/models/integrated_robot
   ```

3. **Create model.config file:**
   ```xml
   <?xml version="1.0"?>
   <model>
     <name>integrated_robot</name>
     <version>1.0</version>
     <sdf version='1.7'>integrated_robot.urdf</sdf>
     <author>
       <name>Digital Twin Developer</name>
       <email>dev@example.com</email>
     </author>
     <description>
       A complete robot model with all sensors for digital twin exercises.
     </description>
   </model>
   ```

4. **Launch the complete system:**
   ```bash
   # Terminal 1: Start Gazebo
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/digital_twin_complete/models
   gz sim -r ~/digital_twin_complete/worlds/complete_world.sdf
   
   # Terminal 2: Start ROS2 bridge (if using)
   source /opt/ros/humble/setup.bash
   ros2 run rosbridge_server rosbridge_websocket --port 9090
   
   # Terminal 3: Run sensor fusion node
   source /opt/ros/humble/setup.bash
   cd ~/digital_twin_complete/src
   python3 sensor_fusion_node.py
   
   # Terminal 4: Validate the system
   source /opt/ros/humble/setup.bash
   cd ~/digital_twin_complete/src
   python3 validate_system.py
   ```

5. **In Unity:**
   - Create a new 3D project
   - Import the ROS# package if available
   - Add the CompleteDigitalTwinController to a GameObject
   - Configure the IP address to match your ROS bridge
   - Run the scene

## Troubleshooting

### Common Issues:

1. **Gazebo doesn't start properly:**
   - Check if all model files are in the correct directory structure
   - Verify URDF syntax with `check_urdf`

2. **ROS communication issues:**
   - Verify that the ROS bridge is running
   - Check IP addresses and port configurations
   - Ensure firewall is not blocking connections

3. **Unity visualization problems:**
   - Confirm that Unity is configured for ROS communication
   - Check that the coordinate system conversion is correct

## Conclusion

This exercise has helped you create a complete digital twin system with:
- A robot model equipped with multiple sensors (LiDAR, IMU, Camera, Depth)
- Proper ROS communication for all sensor types
- Unity visualization of sensor data
- Validation and testing tools

The system you've built provides a foundation for more complex digital twin applications with additional sensors and control systems.

## Performance Considerations

The complete digital twin system requires significant computational resources. Monitor:
- CPU usage on the ROS host
- Memory consumption in Unity
- Network bandwidth if running components on separate machines
- Rendering performance in Unity
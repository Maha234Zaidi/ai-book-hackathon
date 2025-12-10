# Hands-on Exercise 1: Setting up a Basic Digital Twin with Simple Robot Model

In this exercise, you'll create a basic digital twin system using Gazebo for physics simulation and Unity for visualization. By the end of this exercise, you'll have a complete system with a simple robot model that synchronizes between both environments.

## Learning Objectives

By completing this exercise, you will:
1. Create a simple robot model in URDF format
2. Set up a basic Gazebo simulation environment
3. Create a matching visualization in Unity
4. Implement basic communication between Gazebo and Unity
5. Validate the synchronization between both systems

## Prerequisites

Before starting this exercise, ensure you have:
- ROS 2 Humble Hawksbill installed
- Gazebo Garden installed
- Unity 2022.3 LTS installed
- Python 3.10 installed
- Git installed

## Part 1: Creating the Robot Model (URDF)

### Step 1: Create the URDF file

Create a new file called `simple_robot.urdf` with the following content:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
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
      <mass value="10.0"/>
      <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="0.416" iyz="0.0" izz="0.625"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.002916" ixy="0.0" ixz="0.0" iyy="0.002916" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.002916" ixy="0.0" ixz="0.0" iyy="0.002916" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Caster Wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Base to Left Wheel Joint -->
  <joint name="base_to_wheel_left" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0.0 0.25 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Base to Right Wheel Joint -->
  <joint name="base_to_wheel_right" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0.0 -0.25 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Base to Caster Wheel Joint -->
  <joint name="base_to_caster" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.2 0.0 -0.15" rpy="0 0 0"/>
  </joint>

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
    <material>Gazebo/Gray</material>
  </gazebo>
</robot>
```

### Step 2: Validate the URDF file

Create a Python script to validate your URDF file:

```python
# validate_urdf.py
import xml.etree.ElementTree as ET
from urdf_parser_py.urdf import URDF

def validate_urdf(urdf_path):
    try:
        # Parse with XML parser first
        tree = ET.parse(urdf_path)
        print("✓ XML parsing successful")
        
        # Parse with URDF parser
        robot = URDF.from_xml_file(urdf_path)
        print(f"✓ URDF parsing successful: {robot.name}")
        print(f"  Links: {len(robot.links)}, Joints: {len(robot.joints)}")
        
        return True
    except Exception as e:
        print(f"✗ Validation failed: {e}")
        return False

if __name__ == "__main__":
    urdf_file = "simple_robot.urdf"
    success = validate_urdf(urdf_file)
    if success:
        print(f"\n✓ URDF validation passed: {urdf_file}")
    else:
        print(f"\n✗ URDF validation failed: {urdf_file}")
```

Run the validation:
```bash
python3 validate_urdf.py
```

## Part 2: Setting up Gazebo Simulation

### Step 3: Create a Gazebo world file

Create a file called `simple_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include our robot model -->
    <include>
      <uri>model://simple_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>

    <!-- Physics engine -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### Step 4: Launch the Gazebo simulation

Create a launch script to start Gazebo with your robot:

```bash
#!/bin/bash
# To run this script, make it executable: chmod +x launch_gazebo.sh

# Source ROS 2
source /opt/ros/humble/setup.bash

# Set the GAZEBO_MODEL_PATH to include our robot
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)

# Launch Gazebo with the world file
gazebo --verbose simple_world.sdf
```

Make the script executable and run it:
```bash
chmod +x launch_gazebo.sh
./launch_gazebo.sh
```

**Note**: You'll need to place your URDF file in the appropriate Gazebo models directory structure. Create:
```
~/.gazebo/models/simple_robot/
├── model.config
└── meshes/
    └── (any mesh files, though we're using primitive shapes)
```

The `model.config` file should contain:
```xml
<?xml version="1.0"?>
<model>
  <name>simple_robot</name>
  <version>1.0</version>
  <sdf version='1.7'>model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>
    A simple robot model for digital twin exercises.
  </description>
</model>
```

And create a `model.sdf` file that includes your URDF:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_robot">
    <include>
      <uri>file://$(find simple_robot)/simple_robot.urdf</uri>
    </include>
  </model>
</sdf>
```

## Part 3: Creating the Unity Visualization

### Step 5: Set up Unity scene for the robot

1. Open Unity Hub and create a new 3D project named "DigitalTwinRobot"

2. Create a C# script called `SimpleRobotController.cs`:

```csharp
using UnityEngine;

public class SimpleRobotController : MonoBehaviour
{
    [Header("Robot Dimensions")]
    public float bodyLength = 0.5f;
    public float bodyWidth = 0.5f;
    public float bodyHeight = 0.2f;
    public float wheelRadius = 0.1f;
    public float wheelWidth = 0.05f;
    public float wheelOffset = 0.25f;
    
    [Header("Materials")]
    public Material bodyMaterial;
    public Material wheelMaterial;
    public Material casterMaterial;
    
    [Header("Wheels")]
    public Transform leftWheel;
    public Transform rightWheel;
    public Transform casterWheel;
    
    void Start()
    {
        CreateRobot();
    }

    void CreateRobot()
    {
        // Create body
        GameObject body = GameObject.CreatePrimitive(PrimitiveType.Cube);
        body.name = "RobotBody";
        body.transform.position = Vector3.zero;
        body.transform.localScale = new Vector3(bodyLength, bodyHeight, bodyWidth);
        body.GetComponent<Renderer>().material = bodyMaterial;
        body.transform.parent = transform;
        
        // Create left wheel
        leftWheel = CreateWheel("LeftWheel", new Vector3(0, 0, wheelOffset));
        
        // Create right wheel  
        rightWheel = CreateWheel("RightWheel", new Vector3(0, 0, -wheelOffset));
        
        // Create caster wheel
        casterWheel = CreateSphere("CasterWheel", new Vector3(bodyLength/2, -0.1f, 0), 0.05f);
        casterWheel.transform.parent = transform;
    }
    
    Transform CreateWheel(string name, Vector3 offset)
    {
        GameObject wheel = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        wheel.name = name;
        wheel.transform.position = new Vector3(0, 0, 0) + offset;
        wheel.transform.localScale = new Vector3(wheelRadius * 2, wheelWidth / 2, wheelRadius * 2); // Unity cylinder is 2 units tall
        wheel.transform.Rotate(0, 0, 90); // Rotate so it's horizontal
        wheel.GetComponent<Renderer>().material = wheelMaterial;
        wheel.transform.parent = transform;
        return wheel.transform;
    }
    
    Transform CreateSphere(string name, Vector3 position, float radius)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.name = name;
        sphere.transform.position = position;
        sphere.transform.localScale = new Vector3(radius * 2, radius * 2, radius * 2);
        sphere.GetComponent<Renderer>().material = casterMaterial;
        return sphere.transform;
    }
    
    // Method to update wheel positions based on ROS data
    public void UpdateWheelPosition(string wheelName, float angle)
    {
        Transform wheel = null;
        if (wheelName == "left") wheel = leftWheel;
        else if (wheelName == "right") wheel = rightWheel;
        
        if (wheel != null)
        {
            // Update wheel rotation based on joint angle
            wheel.Rotate(Vector3.right, angle * Mathf.Rad2Deg);
        }
    }
}
```

3. In your Unity scene:
   - Create an empty GameObject named "Robot"
   - Create materials for blue (body), black (wheels), and gray (caster)
   - Attach the `SimpleRobotController.cs` script to the Robot object
   - Assign the materials to the script in the inspector

### Step 6: Add coordinate system conversion

Create a script to handle ROS/Unity coordinate system differences:

```csharp
// ROSUnityCoordinateConverter.cs
using UnityEngine;

public static class ROSUnityCoordinateConverter
{
    // Convert position from ROS to Unity coordinate system
    public static Vector3 ROS2UnityPosition(Vector3 rosPosition)
    {
        // Swap X and Z, negate Y for some transformation scenarios
        // Note: This is simplified - actual conversion depends on your specific setup
        return new Vector3(rosPosition.z, rosPosition.y, rosPosition.x);
    }
    
    // Convert orientation from ROS to Unity coordinate system
    public static Quaternion ROS2UnityOrientation(Quaternion rosOrientation)
    {
        // Convert quaternion from ROS to Unity coordinate system
        return new Quaternion(rosOrientation.z, rosOrientation.y, rosOrientation.x, -rosOrientation.w);
    }
    
    // Convert position from Unity to ROS coordinate system
    public static Vector3 Unity2ROSPose(Vector3 unityPosition)
    {
        // Swap X and Z, negate Y
        return new Vector3(unityPosition.z, unityPosition.y, unityPosition.x);
    }
    
    // Convert orientation from Unity to ROS coordinate system
    public static Quaternion Unity2ROSOrientation(Quaternion unityOrientation)
    {
        // Convert quaternion from Unity to ROS coordinate system
        return new Quaternion(unityOrientation.z, unityOrientation.y, unityOrientation.x, -unityOrientation.w);
    }
}
```

## Part 4: Implementing Gazebo-Unity Communication

### Step 7: Create ROS publisher for joint states

Create a ROS 2 node that publishes joint states from Gazebo (we'll use the one from the previous section):

```python
# robot_joint_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class RobotJointPublisher(Node):
    def __init__(self):
        super().__init__('robot_joint_publisher')
        
        # Create publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish messages at regular intervals
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize joint positions
        self.joint_positions = [0.0, 0.0]  # Left and right wheels
        self.i = 0

    def timer_callback(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Define joint names
        msg.name = ['base_to_wheel_left', 'base_to_wheel_right']
        
        # Update joint positions (example: oscillating motion)
        self.joint_positions[0] = math.sin(self.i * 0.1) * 0.5  # Left wheel
        self.joint_positions[1] = math.sin(self.i * 0.1 + 1.0) * 0.5  # Right wheel
        
        # Add velocities and efforts if needed
        msg.position = self.joint_positions
        msg.velocity = [0.0, 0.0]  # Example velocities
        msg.effort = [0.0, 0.0]    # Example efforts
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint positions: {self.joint_positions[0]:.3f}, {self.joint_positions[1]:.3f}')
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    robot_joint_publisher = RobotJointPublisher()
    
    try:
        rclpy.spin(robot_joint_publisher)
    except KeyboardInterrupt:
        print("Shutting down robot joint publisher...")
    
    robot_joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 8: Update Unity to receive joint states

Create a Unity script to receive joint states and update the robot:

```csharp
// JointStateReceiver.cs
using System.Collections.Generic;
using UnityEngine;

#if ENABLE_ROS2
using Ros2UnityCore;
using Ros2Sharp;
using JointState = Ros2Sharp.JointState;

public class JointStateReceiver : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosBridgeIP = "127.0.0.1";
    public int rosBridgePort = 10000;
    
    [Header("Robot Reference")]
    public SimpleRobotController robotController;
    
    private Ros2Socket rosSocket;
    private string jointStatesTopic = "/joint_states";
    private bool rosInitialized = false;

    void Start()
    {
        if (robotController == null)
        {
            robotController = GetComponent<SimpleRobotController>();
        }
        
        InitializeROSConnection();
    }

    void InitializeROSConnection()
    {
        Ros2Settings.Instance.RosIPAddress = rosBridgeIP;
        Ros2Settings.Instance.RosPort = rosBridgePort;
        
        rosSocket = UnityROSTCPConnector.Instance Ros2Socket;
        rosSocket.OnConnected += () => {
            Debug.Log("Connected to ROS bridge");
            rosSocket.Subscribe<JointState>(jointStatesTopic, OnJointStateReceived, 50);
            rosInitialized = true;
        };
        
        rosSocket.Connect();
    }

    void OnJointStateReceived(JointState jointState)
    {
        if (jointState.name.Count != jointState.position.Count)
        {
            Debug.LogWarning("Joint names and positions count mismatch");
            return;
        }
        
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float position = (float)jointState.position[i];
            
            if (jointName.Contains("left"))
            {
                robotController.UpdateWheelPosition("left", position);
            }
            else if (jointName.Contains("right"))
            {
                robotController.UpdateWheelPosition("right", position);
            }
        }
    }

    void OnDestroy()
    {
        if (rosSocket != null)
        {
            rosSocket.Disconnect();
        }
    }
}
#endif
```

## Part 5: Running the Complete System

### Step 9: Integration Test

1. **Start the ROS 2 bridge** (if using):
```bash
source /opt/ros/humble/setup.bash
# If using rosbridge
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

2. **Run the joint state publisher**:
```bash
source /opt/ros/humble/setup.bash
python3 robot_joint_publisher.py
```

3. **Launch Gazebo**:
```bash
./launch_gazebo.sh
```

4. **Start Unity** and run the scene with the robot controller and joint state receiver

5. **Verify synchronization**: The wheels in Unity should rotate in sync with the values published by your ROS node, which would match the Gazebo simulation if you had a real simulation running.

### Step 10: Validation

To verify your digital twin setup:

1. Check that joint states are being published:
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states
```

2. Confirm that the Unity visualization updates in response to joint state messages.

3. Verify that the coordinate systems are properly aligned between Gazebo and Unity.

## Troubleshooting

### Common Issues:

1. **Gazebo doesn't load the robot model**: Ensure the URDF file is in the correct Gazebo models directory structure.

2. **Unity not receiving messages**: Check ROS IP address and port settings, and ensure the ROS bridge is running.

3. **Coordinate system mismatch**: Verify the conversion between ROS/Gazebo and Unity coordinate systems.

4. **Joints not updating**: Check the joint names in your URDF match those in your ROS publisher.

## Conclusion

You've now created a basic digital twin system with:
- A simple robot model defined in URDF
- Gazebo physics simulation environment
- Unity visualization matching the Gazebo model
- Communication between both systems via ROS 2

This foundation can be expanded with additional sensors, more complex environments, and advanced control systems as you progress through the module.

## References

[1] Open Source Robotics Foundation, "Gazebo Robotics Simulator," 2025. [Online]. Available: https://gazebosim.org/. [Accessed: Dec. 9, 2025].

[2] Unity Technologies, "Unity 2022.3 LTS," 2025. [Online]. Available: https://unity.com/. [Accessed: Dec. 9, 2025].

[3] Open Source Robotics Foundation, "ROS 2 Documentation," 2025. [Online]. Available: https://docs.ros.org/. [Accessed: Dec. 9, 2025].
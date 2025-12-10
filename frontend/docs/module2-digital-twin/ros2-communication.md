# ROS 2 Communication Between Gazebo and Unity

One of the critical components of any digital twin system is the communication layer that synchronizes data between the physics simulation (Gazebo) and the visualization (Unity). This section provides actionable code snippets for implementing ROS 2 communication between these systems.

## Overview of Communication Requirements

Digital twin systems require several types of communication:

1. **State Synchronization**: Sending robot joint states from Gazebo to Unity
2. **Sensor Data**: Transmitting simulated sensor readings to Unity for visualization
3. **Control Commands**: Sending commands from Unity to control the simulation
4. **TF Transforms**: Broadcasting coordinate transforms between frames

## Setting up the ROS 2 Environment

Before implementing communication code, ensure your ROS 2 environment is properly set up:

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source your workspace if you created one
source ~/digital_twin_ws/install/setup.bash
```

## Python: Basic Publisher Node

Here's a basic ROS 2 publisher node that publishes joint states from simulation:

```python
# joint_state_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import random

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish messages at regular intervals
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize joint positions
        self.joint_positions = [0.0, 0.0, 0.0]  # For our example robot
        self.i = 0

    def timer_callback(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Define joint names
        msg.name = ['base_to_wheel_left', 'base_to_wheel_right', 'caster_joint']
        
        # Update joint positions (example: oscillating motion)
        self.joint_positions[0] = math.sin(self.i * 0.1) * 0.5
        self.joint_positions[1] = math.sin(self.i * 0.1 + 1.0) * 0.5
        self.joint_positions[2] = 0.0  # Fixed for caster
        
        # Add velocities and efforts if needed
        msg.position = self.joint_positions
        msg.velocity = [0.0, 0.0, 0.0]  # Example velocities
        msg.effort = [0.0, 0.0, 0.0]    # Example efforts
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint states: {self.joint_positions}')
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        print("Shutting down joint state publisher...")
    
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Python: Subscribing to Joint States

Here's how to subscribe to joint states in another node:

```python
# joint_state_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        
        # Create subscription to joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states:')
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.get_logger().info(f'  {name}: {msg.position[i]:.3f}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        print("Shutting down joint state subscriber...")
    
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Python: Publisher for TF Transforms

Coordinate transforms are essential for proper visualization:

```python
# tf_publisher.py
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to broadcast transforms
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.broadcast_transform)
        
        # Initialize some transform values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def broadcast_transform(self):
        # Create transform
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Set transform values
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        from math import sin, cos
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin(self.theta / 2)
        t.transform.rotation.w = cos(self.theta / 2)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Update position for demonstration
        self.x += 0.01
        self.y += 0.005
        self.theta += 0.01

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisher()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        print("Shutting down TF publisher...")
    
    tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Unity: Receiving ROS 2 Messages

For Unity to receive ROS 2 messages, we'll use the ROS-TCP-Connector. Here's a Unity C# script to receive joint state messages:

```csharp
// ROSJointStateReceiver.cs
using System.Collections.Generic;
using UnityEngine;
using Ros2UnityCore;
using Ros2Sharp;
using JointState = Ros2Sharp.JointState;

public class ROSJointStateReceiver : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosBridgeIP = "127.0.0.1";
    public int rosBridgePort = 10000;
    
    [Header("Joint Mapping")]
    public Transform leftWheel;
    public Transform rightWheel;
    public Transform casterWheel;
    
    private Ros2Socket rosSocket;
    private string jointStatesTopic = "/joint_states";
    private bool rosInitialized = false;

    void Start()
    {
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
        // Update Unity objects based on received joint states
        if (jointState.name.Count != jointState.position.Count)
        {
            Debug.LogWarning("Joint names and positions count mismatch");
            return;
        }
        
        // Update each joint
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float position = (float)jointState.position[i];
            
            // Update the appropriate Unity transform
            if (jointName.Contains("left") && leftWheel != null)
            {
                UpdateWheelPosition(leftWheel, position);
            }
            else if (jointName.Contains("right") && rightWheel != null)
            {
                UpdateWheelPosition(rightWheel, position);
            }
            else if (jointName.Contains("caster") && casterWheel != null)
            {
                // Update caster wheel position
                casterWheel.Rotate(Vector3.up, position * Mathf.Rad2Deg);
            }
        }
    }

    void UpdateWheelPosition(Transform wheel, float angleRadians)
    {
        if (wheel != null)
        {
            // Rotate the wheel around its axis
            wheel.Rotate(Vector3.right, angleRadians * Mathf.Rad2Deg);
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
```

## Unity: Publishing Commands to ROS 2

Here's how to publish commands from Unity back to ROS 2:

```csharp
// ROSCommandPublisher.cs
using UnityEngine;
using Ros2UnityCore;
using Ros2Sharp;
using Twist = Ros2Sharp.Twist;

public class ROSCommandPublisher : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosBridgeIP = "127.0.0.1";
    public int rosBridgePort = 10000;
    
    [Header("Robot Control")]
    public float linearSpeed = 1.0f;
    public float angularSpeed = 1.0f;
    
    private Ros2Socket rosSocket;
    private string cmdVelTopic = "/cmd_vel";
    private bool rosInitialized = false;
    private Publisher<Twist> cmdVelPublisher;

    void Start()
    {
        InitializeROSConnection();
    }

    void InitializeROSConnection()
    {
        Ros2Settings.Instance.RosIPAddress = rosBridgeIP;
        Ros2Settings.Instance.RosPort = rosBridgePort;
        
        rosSocket = UnityROSTCPConnector.Instance Ros2Socket;
        rosSocket.OnConnected += () => {
            Debug.Log("Connected to ROS bridge");
            
            // Create publisher for velocity commands
            cmdVelPublisher = rosSocket.Advertise<Twist>(cmdVelTopic);
            rosInitialized = true;
            
            Debug.Log("Ready to send commands");
        };
        
        rosSocket.Connect();
    }

    void Update()
    {
        // Get input from user
        float forwardInput = Input.GetAxis("Vertical");  // Up/Down arrows
        float turnInput = Input.GetAxis("Horizontal");   // Left/Right arrows
        
        // Create and send velocity command if connected
        if (rosInitialized && cmdVelPublisher != null)
        {
            Twist cmdVel = new Twist();
            cmdVel.linear.x = forwardInput * linearSpeed;
            cmdVel.linear.y = 0.0f;
            cmdVel.linear.z = 0.0f;
            cmdVel.angular.x = 0.0f;
            cmdVel.angular.y = 0.0f;
            cmdVel.angular.z = turnInput * angularSpeed;
            
            cmdVelPublisher.Publish(cmdVel);
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
```

## Testing the Communication

To test the ROS 2 communication between Gazebo and Unity:

1. **Start ROS 2 Bridge** (if using):
```bash
# Terminal 1: Start the ROS 2 bridge
source /opt/ros/humble/setup.bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

2. **Run the publisher**:
```bash
# Terminal 2: Run the joint state publisher
source /opt/ros/humble/setup.bash
python3 joint_state_publisher.py
```

3. **Monitor the topic**:
```bash
# Terminal 3: Monitor joint states
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states sensor_msgs/msg/JointState
```

4. **Verify in Unity**: Check that the Unity visualization updates based on the published joint states.

## Best Practices for ROS 2 Communication

1. **Message Frequency**: Balance between responsiveness and bandwidth. 20-50 Hz is typically sufficient for visual updates.

2. **Error Handling**: Always implement proper error handling for network disconnections and message parsing.

3. **Data Validation**: Validate received data to prevent unexpected behavior from malformed messages.

4. **Synchronization**: Consider time synchronization between systems, especially for real-time applications.

5. **Efficiency**: Use appropriate message types and compression for your specific data needs.

These code snippets provide the foundation for implementing communication between Gazebo and Unity in your digital twin systems. They can be adapted and expanded based on your specific requirements.
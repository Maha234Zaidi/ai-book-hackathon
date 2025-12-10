# Creating Unity Scene for Digital Twin

This guide explains how to create a Unity scene that mirrors the Gazebo environment for the digital twin system.

## Setting up the Unity Project

### 1. Create a New Unity Project
1. Open Unity Hub
2. Click "New Project"
3. Select the 3D (Built-in Render Pipeline) template
4. Name your project "DigitalTwinVisualization"
5. Choose a location and click "Create"

### 2. Import Required Packages
- Install the Unity Robotics package via Package Manager
- Install the ROS-TCP-Connector package for communication

## Creating the Environment

### 1. Basic Scene Setup

First, let's create a basic scene that mirrors our Gazebo world:

```csharp
// DigitalTwinEnvironment.cs
using UnityEngine;

public class DigitalTwinEnvironment : MonoBehaviour
{
    [Header("Environment Dimensions")]
    public float width = 10f;
    public float height = 10f;
    public float wallHeight = 2f;
    public float wallThickness = 0.2f;
    
    [Header("Materials")]
    public Material groundMaterial;
    public Material wallMaterial;
    
    void Start()
    {
        CreateEnvironment();
    }

    void CreateEnvironment()
    {
        // Create ground plane
        GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
        ground.name = "Ground";
        ground.transform.position = Vector3.zero;
        ground.transform.localScale = new Vector3(width / 10f, 1, height / 10f); // Plane is 10x10 units by default
        if (groundMaterial != null)
        {
            ground.GetComponent<Renderer>().material = groundMaterial;
        }
        
        // Create walls
        CreateWall(new Vector3(0, wallHeight/2, height/2), new Vector3(width, wallThickness, wallHeight)); // North wall
        CreateWall(new Vector3(0, wallHeight/2, -height/2), new Vector3(width, wallThickness, wallHeight)); // South wall
        CreateWall(new Vector3(width/2, wallHeight/2, 0), new Vector3(wallThickness, wallThickness, wallHeight), 90f); // East wall
        CreateWall(new Vector3(-width/2, wallHeight/2, 0), new Vector3(wallThickness, wallThickness, wallHeight), 90f); // West wall
        
        // Add lighting
        CreateLighting();
    }
    
    GameObject CreateWall(Vector3 position, Vector3 size, float rotationY = 0f)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = "Wall";
        wall.transform.position = position;
        wall.transform.localScale = size;
        wall.transform.Rotate(0, rotationY, 0);
        wall.GetComponent<Renderer>().material = wallMaterial;
        
        // Make it static for better performance
        wall.isStatic = true;
        
        return wall;
    }
    
    void CreateLighting()
    {
        // Create directional light (sun)
        GameObject sunLight = new GameObject("Sun Light");
        sunLight.transform.position = new Vector3(0, 10, 0);
        sunLight.transform.rotation = Quaternion.Euler(50, -30, 0);
        
        Light lightComponent = sunLight.AddComponent<Light>();
        lightComponent.type = LightType.Directional;
        lightComponent.color = Color.white;
        lightComponent.intensity = 1.0f;
        lightComponent.shadows = LightShadows.Soft;
    }
}
```

### 2. Robot Model Setup

To match the URDF model we created, we need to create a similar robot in Unity:

```csharp
// DigitalTwinRobot.cs
using UnityEngine;

public class DigitalTwinRobot : MonoBehaviour
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

### 3. Coordinate System Conversion

Since ROS/Gazebo uses a right-handed coordinate system (X forward, Y left, Z up) and Unity uses a left-handed system (X right, Y up, Z forward), we need conversion utilities:

```csharp
// ROSUnityCoordinateConverter.cs
using UnityEngine;

public static class ROSUnityCoordinateConverter
{
    // Convert position from ROS to Unity coordinate system
    public static Vector3 ROS2UnityPosition(Vector3 rosPosition)
    {
        // Swap X and Z, negate Y
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

### 4. Scene Setup Instructions

1. Create an empty GameObject named "DigitalTwinEnvironment"
2. Attach the `DigitalTwinEnvironment.cs` script to it
3. Create another empty GameObject named "Robot"
4. Attach the `DigitalTwinRobot.cs` script to it
5. Create materials for ground, walls, robot body, wheels, and caster wheel
6. Assign the materials to the respective fields in the inspector

### 5. Connecting to ROS

To receive data from Gazebo/ROS, you'll need to implement ROS communication:

```csharp
// ROSConnectionHandler.cs
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class ROSConnectionHandler : MonoBehaviour
{
    ROSConnection ros;
    public DigitalTwinRobot robot;
    
    // ROS topic names
    const string JOINT_STATES_TOPIC = "/joint_states";
    
    void Start()
    {
        // Get the ROS connection
        ros = ROSConnection.instance;
        
        // Subscribe to joint states topic
        ros.Subscribe<sensor_msgs.JointState>(JOINT_STATES_TOPIC, JointStateCallback);
    }
    
    void JointStateCallback(sensor_msgs.JointState jointState)
    {
        // Update robot based on joint states
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float position = jointState.position[i];
            
            // Update the appropriate wheel
            if (jointName.Contains("left"))
            {
                robot.UpdateWheelPosition("left", position);
            }
            else if (jointName.Contains("right"))
            {
                robot.UpdateWheelPosition("right", position);
            }
        }
    }
}
```

## Best Practices for Unity Scene

1. **Performance Optimization**: Use occlusion culling and level of detail (LOD) systems for complex scenes
2. **Lighting**: Use baked lighting for static elements to improve performance
3. **Materials**: Use physically based materials for realistic rendering
4. **Coordinate System**: Always account for the coordinate system differences between ROS and Unity
5. **Communication**: Use efficient data transfer methods between ROS and Unity

This Unity scene provides a visual counterpart to the Gazebo simulation, completing the digital twin system where physics simulation happens in Gazebo and visualization occurs in Unity.
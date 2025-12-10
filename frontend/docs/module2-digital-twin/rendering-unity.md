# High-Fidelity Rendering in Unity

Unity provides the visualization component of our digital twin system, offering high-fidelity rendering that can accurately represent the physical system being simulated. This section covers how to create compelling visualizations that complement the physics simulation in Gazebo.

## Introduction to Unity Visualization

Unity is a powerful real-time 3D development platform that excels at creating high-quality visualizations. When combined with Gazebo's physics simulation, we can create digital twins that are both physically accurate and visually compelling.

### Key Rendering Components

1. **Lighting**: Realistic illumination and shadows
2. **Materials & Textures**: Surface properties and appearance
3. **Camera Systems**: Perspectives and viewport management
4. **Human-Robot Interaction**: UI elements and visualization tools

## Lighting in Unity

Lighting is crucial for creating realistic and informative visualizations in Unity. Proper lighting can help users understand spatial relationships and identify important elements in the digital twin. Unity provides multiple lighting approaches that can be configured for your digital twin application.

### Types of Lighting

Unity supports several types of lights:
- Directional Lights (for simulating sun/moon)
- Point Lights (for bulbs, explosions)
- Spot Lights (for flashlights, car headlights)
- Area Lights (for soft lighting)

Unity also supports both Real-time and Baked Global Illumination (GI) systems:
- **Real-time GI** updates lighting based on dynamic changes in the scene
- **Baked GI** pre-calculates lighting for static objects, providing better performance

### Setting Up Lighting

```csharp
// Example: Creating and positioning a directional light in Unity with advanced settings
using UnityEngine;

public class LightingSetup : MonoBehaviour
{
    void Start()
    {
        // Create a directional light
        GameObject lightGameObject = new GameObject("Main Directional Light");
        Light lightComponent = lightGameObject.AddComponent<Light>();
        lightComponent.type = LightType.Directional;
        lightComponent.color = Color.white;
        lightComponent.intensity = 1.0f;
        lightComponent.shadows = LightShadows.Soft;  // Soft shadows for realism
        lightComponent.shadowStrength = 0.8f;       // Shadow strength

        // Position and rotate the light
        lightGameObject.transform.position = new Vector3(0, 10, 0);
        lightGameObject.transform.rotation = Quaternion.Euler(50, -120, 0);
    }
}
```

### Lighting Configuration for Consistency with Gazebo

To ensure visual consistency between the physics simulation in Gazebo and the rendering in Unity, consider matching lighting parameters:

1. **Direction**: Align the main directional light in Unity with Gazebo's ambient light direction
2. **Intensity**: Match lighting intensities between the two environments
3. **Color Temperature**: Use similar color temperatures for consistent appearance

## Materials and Textures

Materials and textures give objects their visual appearance in Unity, defining how they interact with light and what they look like. Proper material setup is essential for maintaining consistency with the physical properties defined in your URDF models.

### Material Properties

Key properties of Unity materials include:
- Albedo (base color)
- Metallic (how metallic the surface appears)
- Smoothness (how smooth or rough the surface is)
- Normal maps (surface detail without geometry)
- Emission (self-illuminating properties)
- Occlusion (ambient light occlusion)
- Detail maps (fine surface details)

### Creating Materials from URDF Properties

When importing models from Gazebo, materials should reflect the visual properties defined in the URDF:

```csharp
// Example: Creating a Unity material that matches URDF visual properties
using UnityEngine;

public class MaterialFromURDF : MonoBehaviour
{
    public Material CreateMaterialFromURDF(string colorName, float metallic, float smoothness)
    {
        Material material = new Material(Shader.Find("Standard"));

        // Set albedo color based on URDF specification
        switch(colorName.ToLower())
        {
            case "red":
                material.SetColor("_Color", Color.red);
                break;
            case "blue":
                material.SetColor("_Color", Color.blue);
                break;
            case "green":
                material.SetColor("_Color", Color.green);
                break;
            case "black":
                material.SetColor("_Color", Color.black);
                break;
            case "white":
                material.SetColor("_Color", Color.white);
                break;
            default:
                material.SetColor("_Color", Color.gray);
                break;
        }

        // Apply metallic and smoothness properties from URDF
        material.SetFloat("_Metallic", metallic);
        material.SetFloat("_Smoothness", smoothness);

        return material;
    }
}
```

## Importing Models from Gazebo

To maintain consistency between Gazebo physics and Unity visualization, you need to import models from Gazebo into Unity. This involves converting URDF models to formats Unity can understand.

### Conversion Process

1. Export Gazebo models as standard 3D formats (FBX, OBJ)
2. Import into Unity with appropriate scaling
3. Apply materials that match the visual properties defined in URDF
4. Verify that the physical and visual representations align

### Maintaining Consistency

When importing models, ensure that:
- Scaling is consistent between Gazebo and Unity (meters in both environments)
- Materials match those specified in the URDF visual properties
- Coordinate systems are properly converted (Gazebo: X-forward, Y-left, Z-up; Unity: Z-forward, X-right, Y-up)

## Camera Systems

A well-designed camera system is essential for navigating and observing the digital twin. Unity provides various camera options and techniques.

### Common Camera Setups

- **Orbit cameras** for inspection and detailed viewing
- **Fixed cameras** for monitoring specific areas
- **Dynamic cameras** that follow the robot
- **VR cameras** for immersive experiences

### Camera Configuration Example

```csharp
// Example: Setting up a follow camera that tracks a robot
using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform target;            // The robot to follow
    public float smoothSpeed = 0.125f;  // Smoothing factor
    public Vector3 offset = new Vector3(0, 5, -10); // Camera offset

    void LateUpdate()
    {
        Vector3 desiredPosition = target.position + offset;
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
        transform.position = smoothedPosition;

        transform.LookAt(target);
    }
}
```

## Human-Robot Interaction (HRI) in Unity

Creating effective human-robot interaction interfaces in Unity is important for digital twin applications. These interfaces should allow users to interact with and control the digital twin effectively.

### UI Elements for HRI

1. **Control Panels**: Allow users to send commands to the robot
2. **Sensor Data Visualization**: Display sensor readings in easy-to-understand formats
3. **Status Indicators**: Show the robot's current state and health
4. **Animation Controls**: Enable users to trigger specific robot behaviors

### Example HRI Interface

```csharp
// Example: Simple HRI panel for controlling a robot
using UnityEngine;
using UnityEngine.UI;

public class RobotControlPanel : MonoBehaviour
{
    public Button moveForwardButton;
    public Button turnLeftButton;
    public Button turnRightButton;
    public Button stopButton;
    public Text statusText;

    private void Start()
    {
        // Assign button click events
        moveForwardButton.onClick.AddListener(MoveForward);
        turnLeftButton.onClick.AddListener(TurnLeft);
        turnRightButton.onClick.AddListener(TurnRight);
        stopButton.onClick.AddListener(StopRobot);
    }

    void MoveForward()
    {
        // Send command to robot via ROS bridge
        statusText.text = "Moving Forward...";
        // Implementation would send command via ROS TCP endpoint
    }

    void TurnLeft()
    {
        // Send command to robot via ROS bridge
        statusText.text = "Turning Left...";
    }

    void TurnRight()
    {
        // Send command to robot via ROS bridge
        statusText.text = "Turning Right...";
    }

    void StopRobot()
    {
        // Send command to robot via ROS bridge
        statusText.text = "Stopped";
    }
}
```

## Performance Optimization

For digital twin applications with complex scenes, performance optimization is crucial to maintain a smooth user experience.

### Rendering Optimization Techniques

1. **Level of Detail (LOD)**: Reduce geometry complexity for distant objects
2. **Occlusion Culling**: Don't render objects hidden from view
3. **Texture Streaming**: Load textures at appropriate resolutions based on distance
4. **Light Baking**: Precompute lighting for static objects
5. **Shader Optimization**: Use efficient shaders that don't overly tax the GPU

## Hands-on Exercise: Creating a Visualization Scene

In this exercise, you'll create a Unity scene that visualizes a simple robot model with appropriate lighting and materials that match its physical properties defined in the URDF.

### Exercise Steps:

1. Create a new Unity project
2. Import the robot model exported from Gazebo
3. Configure lighting to match the Gazebo environment
4. Apply materials that match the URDF visual properties
5. Set up a camera system for viewing the robot
6. Create a basic UI for human-robot interaction
# Coordinate System Consistency in Digital Twin Systems

This document defines the consistent coordinate systems used between Gazebo physics simulation and Unity visualization in digital twin implementations. Proper coordinate system alignment is critical for accurate mapping between physical simulation and visual representation.

## Overview

Coordinate system differences between simulation and visualization environments are a common source of errors in digital twin implementations. This document establishes clear conventions and conversion methods between Gazebo and Unity coordinate systems.

## ROS/Gazebo Coordinate System

ROS uses a right-handed coordinate system (adhering to the Right Hand Rule):
- **X-axis**: Forward (positive X points forward)
- **Y-axis**: Left (positive Y points to the left)
- **Z-axis**: Up (positive Z points up)

This is consistent with the REP-103 standard for ROS coordinate frames.

## Unity Coordinate System

Unity uses a left-handed coordinate system:
- **X-axis**: Right (positive X points to the right)
- **Y-axis**: Up (positive Y points up)
- **Z-axis**: Forward (positive Z points forward)

## Converting Between Systems

### Position Conversion

To convert a position from ROS/Gazebo to Unity:
```
Unity_X = Gazebo_Z
Unity_Y = Gazebo_Y
Unity_Z = Gazebo_X
```

In code:

```csharp
// C# Implementation for Unity
public static Vector3 ROS2UnityPosition(Vector3 rosPosition)
{
    return new Vector3(rosPosition.z, rosPosition.y, rosPosition.x);
}

public static Vector3 Unity2ROSPose(Vector3 unityPosition)
{
    return new Vector3(unityPosition.z, unityPosition.y, unityPosition.x);
}
```

```python
# Python Implementation for ROS
def ros_to_unity_position(ros_pos):
    return [ros_pos.z, ros_pos.y, ros_pos.x]

def unity_to_ros_position(unity_pos):
    return [unity_pos.z, unity_pos.y, unity_pos.x]
```

### Orientation/Quaternion Conversion

For rotations, convert the quaternion from ROS to Unity:

```csharp
// C# Implementation for Unity
public static Quaternion ROS2UnityOrientation(Quaternion rosOrientation)
{
    // Convert quaternion from ROS to Unity coordinate system
    return new Quaternion(rosOrientation.z, rosOrientation.y, rosOrientation.x, -rosOrientation.w);
}

public static Quaternion Unity2ROSOrientation(Quaternion unityOrientation)
{
    // Convert quaternion from Unity to ROS coordinate system
    return new Quaternion(unityOrientation.z, unityOrientation.y, unityOrientation.x, -unityOrientation.w);
}
```

### Verification Examples

Here are common transformations to verify your implementation:

| ROS/Gazebo | Unity Equivalent |
|------------|------------------|
| Position (1, 0, 0) | Position (0, 0, 1) |
| Position (0, 1, 0) | Position (0, 1, 0) |
| Position (0, 0, 1) | Position (1, 0, 0) |
| Rotation ROS (0, 0, 0, 1) | Rotation Unity (0, 0, 0, -1) |

## Practical Implementation

### In URDF Definitions

When creating URDF models, keep in mind that they'll be converted to Unity coordinates:

```xml
<!-- Example: Robot positioned in Gazebo world -->
<model name="digital_twin_robot">
  <!-- In Gazebo: position (5, 2, 0) means 5m forward, 2m left -->
  <pose>5 2 0 0 0 0</pose>
  <!-- This will appear in Unity at (0, 2, 5): 0m right, 2m up, 5m forward -->
  ...
</model>
```

### In Unity Code

When reading positions from ROS topics in Unity:

```csharp
// Example: Processing Odometry message from ROS
void ProcessOdometry(nav_msgs.Odometry rosOdom)
{
    // Convert the pose to Unity coordinate system
    Vector3 unityPosition = ROS2UnityPosition(rosOdom.pose.pose.position);
    Quaternion unityRotation = ROS2UnityOrientation(rosOdom.pose.pose.orientation);
    
    // Apply the transformation to our Unity robot object
    transform.position = unityPosition;
    transform.rotation = unityRotation;
}
```

## Best Practices

1. **Consistent Transform Functions**: Create utility functions for coordinate conversion and use them throughout your codebase.

2. **Documentation**: Clearly document in your code when coordinate system conversions occur.

3. **Testing**: Test conversions with known values to verify accuracy.

4. **Visual Verification**: Use visual debugging tools to verify that objects appear in the expected positions.

5. **Reference Frames**: Always specify which coordinate frame your coordinates are in when writing documentation or comments.

## Common Issues and Solutions

### Issue 1: Robot facing wrong direction
**Cause**: Incorrect orientation conversion.
**Solution**: Verify quaternion conversion functions and check for sign errors.

### Issue 2: Robot appears in wrong position
**Cause**: Incorrect position conversion.
**Solution**: Check the mapping of X, Y, Z axes between systems.

### Issue 3: Rotations behave unexpectedly
**Cause**: Euler angle conversion issues.
**Solution**: Use quaternions for all rotation calculations and convert only when necessary.

## Validation

To verify coordinate system consistency:

1. Place an object at a known position in Gazebo (e.g., (1, 0, 0))
2. Check that it appears at the expected position in Unity (e.g., (0, 0, 1))
3. Rotate the object in Gazebo and verify the rotation appears correctly in Unity
4. Move the object and verify the movement direction is consistent in both systems

By following these conventions, you'll ensure that your digital twin system maintains accurate spatial relationships between the physical simulation in Gazebo and the visualization in Unity.
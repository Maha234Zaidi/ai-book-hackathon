# Detailed Examples: Configuring Gravity in Gazebo

Gravity is a fundamental aspect of physics simulation in Gazebo. This section provides detailed examples of how to configure gravity for different scenarios and environments in your digital twin system.

## Understanding Gravity in Gazebo

In Gazebo, gravity is represented as a 3D vector in meters per second squared (m/s²) that applies a constant acceleration to all objects in the simulation. The default Earth gravity vector is [0, 0, -9.8], indicating acceleration in the negative Z direction (downward) at 9.8 m/s².

## Basic Gravity Configuration

### Setting Gravity in World Files

The primary way to configure gravity is through the SDF world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_gravity_world">
    <!-- Default Earth gravity -->
    <gravity>0 0 -9.8</gravity>
    
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### Direction and Magnitude

The gravity vector has three components [X, Y, Z] that determine the direction and magnitude of gravitational acceleration:

- **X**: Acceleration in the X direction (positive is right)
- **Y**: Acceleration in the Y direction (positive is forward in Gazebo)  
- **Z**: Acceleration in the Z direction (positive is up, negative is down)

## Earth-Like Gravity Examples

### Standard Earth Gravity
```xml
<gravity>0 0 -9.8</gravity>
```
Standard Earth gravity with 9.8 m/s² downward acceleration.

### Adjusted Earth Gravity (Different Units)
```xml
<gravity>0 0 -32.174</gravity>  <!-- Imperial units (ft/s²) -->
```

### Polar Gravity
Earth's gravitational acceleration varies slightly at different locations:
```xml
<gravity>0 0 -9.832</gravity>  <!-- At the poles -->
<gravity>0 0 -9.780</gravity>  <!-- At the equator -->
```

## Non-Earth Gravity Examples

### Moon Gravity
```xml
<gravity>0 0 -1.62</gravity>
```
The moon has approximately 1/6th of Earth's gravity.

### Mars Gravity
```xml
<gravity>0 0 -3.71</gravity>
```
Mars has about 38% of Earth's gravity.

### Asteroid Gravity
```xml
<gravity>0 0 -0.01</gravity>
```
A small asteroid might have very weak gravity.

### Zero Gravity (Space)
```xml
<gravity>0 0 0</gravity>
```
Useful for simulating space or zero-gravity environments.

### Lateral Gravity
```xml
<gravity>-9.8 0 0</gravity>  <!-- Gravity pulls in -X direction -->
<gravity>0 9.8 0</gravity>   <!-- Gravity pulls in +Y direction -->
<gravity>5 5 -3</gravity>    <!-- Diagonal gravity vector -->
```

## Advanced Gravity Configuration

### Changing Gravity During Simulation

You can modify gravity during runtime using Gazebo services. Here are examples in different languages:

**Command Line (Gazebo Garden/Harmonic):**
```bash
# Change to moon gravity
gz service -s /world/demo_world/set_physics_engine_config \
  --reqtype gz.msgs.Physics \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'gravity: {x: 0.0, y: 0.0, z: -1.62}'
```

**Python:**
```python
#!/usr/bin/env python3
# change_gravity.py
import rclpy
from rclpy.node import Node
from gz.msgs10 import Physics
from gz.transport10 import Node as GzNode
import time

class GravityChanger(Node):
    def __init__(self):
        super().__init__('gravity_changer')
        self.gz_node = GzNode()
        
        # Change gravity to moon level
        self.change_gravity(0.0, 0.0, -1.62)
        
    def change_gravity(self, x, y, z):
        """Change the gravity in the simulation"""
        physics_msg = Physics()
        physics_msg.gravity.x = x
        physics_msg.gravity.y = y
        physics_msg.gravity.z = z
        
        # Publish the change
        self.gz_node.request(
            '/world/default/set_physics_engine_config',
            physics_msg
        )
        
        self.get_logger().info(f'Changed gravity to: [{x}, {y}, {z}]')

def main():
    rclpy.init()
    node = GravityChanger()
    
    # Give some time for the change to take effect
    time.sleep(1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**C++:**
```cpp
// change_gravity.cpp
#include <gz/msgs10/physics.pb.h>
#include <gz/transport10/Node.hh>
#include <iostream>

int main() {
    gz::transport::Node node;
    
    // Create a physics message with new gravity
    gz::msgs::Physics physics_msg;
    auto* gravity = physics_msg.mutable_gravity();
    gravity->set_x(0.0);
    gravity->set_y(0.0);
    gravity->set_z(-1.62);  // Moon gravity
    
    // Send the request to change physics settings
    bool result;
    bool success = node.Request("/world/default/set_physics_engine_config", 
                                physics_msg, 5000, result);
    
    if (success && result) {
        std::cout << "Gravity successfully changed to moon level" << std::endl;
    } else {
        std::cout << "Failed to change gravity" << std::endl;
    }
    
    return 0;
}
```

## Physics Engine-Specific Gravity Configuration

### For ODE Physics Engine
```xml
<physics name="ode_physics" type="ode">
  <gravity>0 0 -9.8</gravity>
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
```

### For DART Physics Engine
```xml
<physics name="dart_physics" type="dart">
  <gravity>0 0 -9.8</gravity>
  <dart>
    <solver>
      <type>PGS</type>  <!-- PGS or CCD -->
      <solver_iterations>10</solver_iterations>
    </solver>
  </dart>
</physics>
```

## Practical Example: Multi-Gravity World

Here's a complete example of a world file with different gravity zones simulated using plugins:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="multi_gravity_world">
    <!-- Default gravity (Earth) -->
    <gravity>0 0 -9.8</gravity>
    
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Moon gravity plugin (using a custom plugin) -->
    <model name="moon_gravity_zone">
      <link name="moon_zone_link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>5.0</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 0.2</ambient>
            <diffuse>0.5 0.5 0.5 0.2</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>5.0</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
      
      <!-- Custom plugin to simulate moon gravity in this area -->
      <plugin name="moon_gravity_plugin" filename="libMoonGravityPlugin.so">
        <gravity>0 0 -1.62</gravity>
        <radius>5.0</radius>
        <position>5 5 2</position>
      </plugin>
    </model>
    
    <!-- Physics engine -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Spawn some test objects -->
    <model name="earth_ball">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0002</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0002</iyy>
            <iyz>0</iyz>
            <izz>0.0002</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <model name="moon_ball">
      <pose>5 5 2 0 0 0</pose>  <!-- In the moon gravity zone -->
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0002</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0002</iyy>
            <iyz>0</iyz>
            <izz>0.0002</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Testing Gravity Effects

### Basic Test Model
Create a simple model to test different gravity settings:

```xml
<?xml version="1.0"?>
<robot name="gravity_tester">
  <link name="gravity_test_base">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
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
    </collision>
  </link>
</robot>
```

### Validation Test Script
```python
#!/usr/bin/env python3
# validate_gravity.py
import math

def calculate_fall_time(height, gravity_magnitude):
    """
    Calculate time to fall from given height with given gravity
    Formula: t = sqrt(2h/g)
    """
    if gravity_magnitude == 0:
        return float('inf')  # Takes infinite time in zero gravity
    
    return math.sqrt(2 * height / abs(gravity_magnitude))

def validate_gravity_setting(expected_gravity, measured_time, fall_height):
    """
    Validate if the measured fall time matches the expected gravity
    """
    calculated_time = calculate_fall_time(fall_height, abs(expected_gravity))
    
    # Allow 10% tolerance for simulation inaccuracies
    tolerance = 0.10
    
    if abs(measured_time - calculated_time) / calculated_time <= tolerance:
        print(f"✓ Gravity validation passed: Expected {expected_gravity} m/s², measured time {measured_time:.3f}s, calculated time {calculated_time:.3f}s")
        return True
    else:
        print(f"✗ Gravity validation failed: Expected {expected_gravity} m/s², measured time {measured_time:.3f}s, calculated time {calculated_time:.3f}s")
        return False

# Example usage
if __name__ == "__main__":
    height = 1.0  # 1 meter drop
    earth_gravity = -9.8
    measured_time = 0.45  # Measured from simulation
    
    validate_gravity_setting(earth_gravity, measured_time, height)
```

## Best Practices for Gravity Configuration

1. **Use appropriate values**: Match the gravity value to your intended environment (Earth, Moon, Mars, etc.)

2. **Consider computational efficiency**: Very small gravity values might require smaller time steps for stability

3. **Document changes**: Clearly document any non-standard gravity values used in your simulations

4. **Test with simple objects first**: Verify gravity settings with simple objects before applying to complex robots

5. **Account for coordinate systems**: Remember that Gazebo uses Z-down convention where negative Z values mean "down"

## Troubleshooting Gravity Issues

### Objects Don't Fall
- Check if gravity is set to [0 0 0] (zero gravity)
- Verify that objects have proper mass and inertia values
- Check that collision geometry is defined

### Unstable Simulations
- Try reducing the max step size in physics configuration
- Adjust ERP and CFM parameters
- Ensure gravity and other physics parameters are reasonable

### Objects Fall Too Fast/Slow
- Verify the gravity vector is correctly set
- Check units (m/s² vs ft/s²)
- Confirm coordinate system conventions (Z-down in Gazebo)

With these examples and techniques, you can accurately configure gravity in your Gazebo simulations to match real-world conditions or create special environments for your digital twin applications.
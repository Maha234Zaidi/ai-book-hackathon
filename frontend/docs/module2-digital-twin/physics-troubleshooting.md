# Common Physics Simulation Issues and Troubleshooting

Physics simulations in Gazebo can encounter various issues that affect the realism and stability of your digital twin system. This section documents the most common problems and provides troubleshooting approaches to resolve them.

## Overview of Common Issues

Physics simulation issues typically fall into several categories:
1. **Stability problems**: Simulation becoming unstable or exploding
2. **Accuracy problems**: Behavior not matching real-world expectations
3. **Performance problems**: Simulation running too slowly
4. **Interaction problems**: Objects not interacting as expected

## Stability Issues

### Issue 1: Objects Falling Through Each Other (Phantom Pass-Through)

**Symptoms:**
- Objects pass through surfaces or other objects
- Robot wheels fall through the ground
- Collision detection fails

**Causes:**
- Collision geometry not properly defined
- Time step too large
- Objects moving too fast in a single step
- Penetration depth settings too small

**Solutions:**
1. **Verify collision geometry:**
```xml
<!-- Make sure collision geometry is properly defined -->
<collision>
  <geometry>
    <box size="0.2 0.2 0.2"/>  <!-- Ensure geometry exists -->
  </geometry>
</collision>
```

2. **Reduce time step:**
```xml
<physics name="fixed_step" type="ode">
  <max_step_size>0.0005</max_step_size>  <!-- Reduce from 0.001 -->
  <!-- ... -->
</physics>
```

3. **Increase ERP (Error Reduction Parameter):**
```xml
<constraints>
  <erp>0.8</erp>  <!-- Increase from 0.2 -->
  <contact_surface_layer>0.002</contact_surface_layer>  <!-- Increase from 0.001 -->
</constraints>
```

4. **Add more collision points** (for complex shapes, use multiple simpler shapes)

### Issue 2: Simulation Explosion (Objects Flying Apart)

**Symptoms:**
- Objects suddenly shoot away at high speed
- Robot joints break apart
- Constraints violated dramatically

**Causes:**
- Physics parameters too aggressive
- Mass/inertia values unrealistic
- Solver parameters inappropriate
- High-velocity collisions

**Solutions:**
1. **Verify mass and inertia:**
```xml
<inertial>
  <mass value="1.0"/>  <!-- Ensure reasonable mass -->
  <inertia 
    ixx="0.083" ixy="0.0" ixz="0.0"  <!-- Verify these values are reasonable -->
    iyy="0.083" iyz="0.0"
    izz="0.167" />
</inertial>
```

2. **Use more conservative solver parameters:**
```xml
<solver>
  <type>quick</type>
  <iters>50</iters>  <!-- Increase iterations -->
  <sor>1.0</sor>    <!-- Lower SOR value -->
</solver>
<constraints>
  <cfm>0.001</cfm>  <!-- Small but non-zero CFM -->
  <erp>0.2</erp>    <!-- Conservative ERP -->
</constraints>
```

3. **Use lower update rates if not time-critical:**
```xml
<max_step_size>0.001</max_step_size>
<real_time_update_rate>1000</real_time_update_rate>  <!-- Adjust as needed -->
```

### Issue 3: Jittery or Unstable Movement

**Symptoms:**
- Objects shake or vibrate when at rest
- Robot wheels wobble without external input
- Joint oscillations

**Causes:**
- Time step too large relative to constraints
- ERP/CFM values not optimized
- Constraints too tight

**Solutions:**
1. **Fine-tune ERP and CFM:**
```xml
<constraints>
  <erp>0.1 - 0.4</erp>  <!-- Experiment with values in this range -->
  <cfm>0.0001 - 0.01</cfm>  <!-- Add some compliance -->
</constraints>
```

2. **Add damping to links:**
```xml
<collision>
  <geometry>
    <box size="0.2 0.2 0.2"/>
  </geometry>
  <surface>
    <damping>
      <linear>0.1</linear>    <!-- Add small damping -->
      <angular>0.1</angular>
    </damping>
  </surface>
</collision>
```

3. **Verify joint physical properties:**
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel"/>
  <axis xyz="0 1 0"/>
  <!-- Add joint limits if needed -->
  <dynamics damping="0.1" friction="0.0"/>  <!-- Add damping/friction -->
</joint>
```

## Accuracy Issues

### Issue 4: Incorrect Physical Response

**Symptoms:**
- Robot doesn't move forward despite appropriate wheel rotation
- Object bounce doesn't match expectations
- Sliding behavior incorrect

**Causes:**
- Mass distribution incorrect
- Friction parameters unrealistic
- Inertia values wrong

**Solutions:**
1. **Calculate proper mass and inertia:**
```xml
<!-- For a cylindrical wheel with radius r, height h, mass m -->
<!-- ixx = izz = 1/12 * m * (3*r² + h²), iyy = 1/2 * m * r² -->
<inertial>
  <mass value="0.5"/>
  <inertia 
    ixx="0.001875" ixy="0" ixz="0"  <!-- For r=0.1, h=0.05, m=0.5 -->
    iyy="0.0025" iyz="0"
    izz="0.001875" />
</inertial>
```

2. **Verify friction coefficients:**
```xml
<collision>
  <geometry>
    <cylinder length="0.05" radius="0.1"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>   <!-- Rubber tire: 0.8-1.2 -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

3. **Adjust bounciness (restitution coefficient):**
```xml
<surface>
  <bounce>
    <restitution_coefficient>0.2</restitution_coefficient>  <!-- Low for non-bouncy -->
    <threshold>1.0</threshold>
  </bounce>
</surface>
```

### Issue 5: Robot Cannot Climb Slopes or Overcome Small Obstacles

**Symptoms:**
- Robot gets stuck on small bumps
- Cannot climb gentle slopes
- Wheels slip without traction

**Causes:**
- Insufficient friction
- Wrong center of mass
- Inappropriate wheel configuration

**Solutions:**
1. **Increase friction for drive wheels:**
```xml
<collision>
  <!-- On drive wheels -->
  <surface>
    <friction>
      <ode>
        <mu>1.2</mu>    <!-- High friction for traction -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

2. **Verify center of mass and weight distribution:**
```xml
<link name="base_link">
  <inertial>
    <mass value="5.0"/>
    <origin xyz="0 0 0"/>  <!-- Ensure COM is reasonable -->
    <inertia ixx="..." ... />
  </inertial>
</link>
```

## Performance Issues

### Issue 6: Slow Simulation Performance

**Symptoms:**
- Low frame rate in Gazebo
- Simulation running slower than real-time
- High CPU usage

**Causes:**
- Physics step size too small
- Solver iterations too high
- Visualization quality too high

**Solutions:**
1. **Optimize physics parameters:**
```xml
<physics name="fast_physics" type="ode">
  <max_step_size>0.002</max_step_size>     <!-- Increase step size (0.001-0.01) -->
  <real_time_update_rate>500</real_time_update_rate>  <!-- Reduce update rate -->
  <ode>
    <solver>
      <iters>10</iters>     <!-- Reduce iterations -->
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

2. **Simplify collision geometry:**
```xml
<!-- Instead of complex meshes, use simple primitives -->
<collision>
  <!-- Use boxes/cylinders/spheres instead of complex meshes -->
  <geometry>
    <box size="0.3 0.3 0.15"/>  <!-- Simpler than a detailed mesh -->
  </geometry>
</collision>
```

3. **Reduce visual quality settings in Gazebo GUI**

### Issue 7: High CPU Usage During Simulation

**Symptoms:**
- High CPU usage even when nothing is moving
- Simulation becomes unresponsive

**Causes:**
- Physics engine continuously solving minor constraints
- Too many contact points
- Excessive precision requirements

**Solutions:**
1. **Add damping to reduce oscillations:**
```xml
<collision>
  <surface>
    <damping>
      <linear>1.0</linear>    <!-- Higher damping for stability -->
      <angular>1.0</angular>
    </damping>
  </surface>
</collision>
```

2. **Use slightly compliant constraints:**
```xml
<constraints>
  <cfm>0.001</cfm>    <!-- Small CFM for more compliant constraints -->
  <erp>0.3</erp>      <!-- Moderate ERP -->
</constraints>
```

## Interaction Issues

### Issue 8: Joint Limits Not Respected

**Symptoms:**
- Robot joints exceeding their intended range
- Robot "breaking" when hitting limits

**Causes:**
- Joint limits not properly specified
- Soft limits not set
- High forces overcoming limits

**Solutions:**
```xml
<joint name="arm_joint" type="revolute">
  <parent link="base"/>
  <child link="arm"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>  <!-- Set proper limits -->
  <dynamics damping="1.0" friction="0.1"/>  <!-- Add damping -->
</joint>
```

### Issue 9: Robot Falling Through Ground on Spawn

**Symptoms:**
- Robot falls through the ground immediately on simulation start
- Objects sink into surfaces

**Causes:**
- Collision geometry not aligned with visual geometry
- Incorrect initial position
- Penetration depth too small

**Solutions:**
1. **Ensure proper pose and collision alignment:**
```xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.3 0.15"/>
    </geometry>
  </visual>
  
  <collision>
    <!-- Align collision with visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.3 0.15"/>
    </geometry>
  </collision>
</link>
```

2. **Spawn at appropriate height:**
```xml
<!-- In SDF world file -->
<include>
  <uri>model://robot</uri>
  <pose>0 0 0.2 0 0 0</pose>  <!-- Start slightly above ground -->
</include>
```

## Systematic Troubleshooting Approach

### Step 1: Identify the Issue Category
Determine whether the issue is related to stability, accuracy, performance, or interaction.

### Step 2: Isolate the Problem
- Test with a minimal model (single object)
- Gradually add complexity to identify the problematic component
- Use the Gazebo GUI to visually inspect behaviors

### Step 3: Check Basic Configuration
- Verify URDF/SDF syntax
- Ensure all mass, inertia, and collision elements are defined
- Check that units are consistent (metric system)

### Step 4: Adjust Parameters Systematically
- Start with conservative values
- Change one parameter at a time
- Document what works and what doesn't

### Step 5: Validate with Simple Tests
- Drop a box from a known height and measure fall time
- Test that objects of different masses fall at the same rate
- Verify that friction is causing appropriate sliding behavior

## Debugging Tools and Techniques

### Visual Debugging
- Enable contact visualization in Gazebo GUI (View → Contacts)
- Use the transforms visualization (View → Transparent)
- Enable center of mass visualization (View → COM)

### Logging and Monitoring
- Use `gz topic -e` to monitor physics topics for unusual values
- Monitor simulation statistics: `gz stats`
- Check for model contact forces

### Validation Scripts
Create validation tests to automatically check for common issues:

```bash
#!/bin/bash
# validate_physics.sh - Basic physics validation script

# Check if Gazebo is running
if ! pgrep gz > /dev/null; then
    echo "Gazebo is not running"
    exit 1
fi

echo "Basic Physics Validation:"
echo "- Check if objects fall at approximately 9.8 m/s²"
echo "- Verify that high-friction objects don't slide freely"
echo "- Confirm that low-friction objects move easily"
echo "- Test that different masses fall at same rate"
```

## When to Seek Additional Help

If you've tried the troubleshooting steps and still experience issues:

1. **Check Gazebo version compatibility** with your URDF/SDF files
2. **Verify your physics engine choice** is appropriate for your use case
3. **Consider alternative modeling approaches** (different joint types, etc.)
4. **Consult the Gazebo community forums** for similar issues
5. **Validate with known working examples** from Gazebo tutorials

By following these troubleshooting approaches, you should be able to resolve most physics simulation issues and create stable, realistic digital twin systems in Gazebo.
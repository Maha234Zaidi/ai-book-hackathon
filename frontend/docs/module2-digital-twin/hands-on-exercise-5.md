# Hands-on Exercise 5: Optimization, Troubleshooting, and Reproducibility

In this final exercise, you'll learn to optimize your digital twin system for better performance, troubleshoot common issues, and ensure reproducibility across different environments.

## Learning Objectives

By completing this exercise, you will:
1. Identify and fix performance bottlenecks in Gazebo and Unity
2. Troubleshoot common integration issues between Gazebo and Unity
3. Create reproducible environments using configuration files
4. Implement best practices for long-term maintainability

## Prerequisites

Before starting this exercise, ensure you have:
- Completed Exercises 1-4
- Access to both Gazebo and Unity environments
- Basic understanding of system optimization
- Docker installed (for reproducibility section)

## Part 1: Performance Optimization

### Step 1: Identify Performance Bottlenecks

First, let's create a performance monitoring script to identify bottlenecks:

`~/digital_twin_optimization/performance_monitor.py`

```python
#!/usr/bin/env python3
"""
Performance monitoring script for digital twin systems
"""
import psutil
import time
import rospy
from std_msgs.msg import Float32
import subprocess
import json

class PerformanceMonitor:
    def __init__(self):
        # Initialize ROS if available
        try:
            rospy.init_node('performance_monitor', anonymous=True)
            self.cpu_pub = rospy.Publisher('/system/cpu_usage', Float32, queue_size=10)
            self.mem_pub = rospy.Publisher('/system/mem_usage', Float32, queue_size=10)
            self.gazebo_pub = rospy.Publisher('/system/gazebo_fps', Float32, queue_size=10)
            self.rate = rospy.Rate(1)  # 1 Hz
            self.ros_available = True
        except:
            print("ROS not available, running in standalone mode")
            self.ros_available = False

    def get_gazebo_fps(self):
        """Get Gazebo simulation FPS using gazebo topic"""
        try:
            # Try to get Gazebo stats
            result = subprocess.run(['gz', 'stats'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                # Parse FPS from gz stats output (simplified)
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'SimTime' in line and 'RealTime' in line:
                        # Extract FPS information (this is a simplified example)
                        # In practice, you'd have a more robust parsing method
                        return 100.0  # Placeholder value
            return 0.0
        except:
            return 0.0

    def monitor_system(self):
        """Monitor system resources"""
        while not rospy.is_shutdown() if self.ros_available else True:
            # Get CPU usage
            cpu_percent = psutil.cpu_percent(interval=1)
            memory_percent = psutil.virtual_memory().percent
            
            # Get Gazebo FPS if available
            gazebo_fps = self.get_gazebo_fps()
            
            # Print system stats
            print(f"CPU: {cpu_percent}%, Memory: {memory_percent}%, Gazebo FPS: {gazebo_fps}")
            
            # Publish to ROS topics if available
            if self.ros_available:
                self.cpu_pub.publish(cpu_percent)
                self.mem_pub.publish(memory_percent)
                self.gazebo_pub.publish(gazebo_fps)
                self.rate.sleep()
            else:
                time.sleep(1)

    def get_optimization_recommendations(self, cpu_usage, mem_usage, gazebo_fps):
        """Provide optimization recommendations based on system stats"""
        recommendations = []
        
        if cpu_usage > 80:
            recommendations.append("High CPU usage detected. Consider reducing simulation complexity.")
            recommendations.append("Reduce number of entities in simulation or decrease update rates.")
            recommendations.append("Use simpler collision geometries (boxes instead of meshes).")
        
        if mem_usage > 80:
            recommendations.append("High memory usage detected. Optimize asset sizes.")
            recommendations.append("Use texture compression and LOD systems in Unity.")
            recommendations.append("Consider using occlusion culling in Unity.")
        
        if gazebo_fps < 100:  # Assuming target is 1000Hz physics
            recommendations.append(f"Low Gazebo FPS ({gazebo_fps}). Optimize physics parameters.")
            recommendations.append("Increase max_step_size or reduce solver iterations if accuracy allows.")
            recommendations.append("Reduce number of contacts and complex geometries.")
        
        return recommendations

if __name__ == '__main__':
    monitor = PerformanceMonitor()
    
    try:
        print("Starting performance monitoring...")
        print("Press Ctrl+C to stop and see optimization recommendations")
        
        start_time = time.time()
        snapshots = []
        
        # Collect data for 30 seconds
        for i in range(30):
            cpu_percent = psutil.cpu_percent(interval=1)
            memory_percent = psutil.virtual_memory().percent
            gazebo_fps = monitor.get_gazebo_fps()
            
            snapshots.append({
                'timestamp': time.time() - start_time,
                'cpu': cpu_percent,
                'memory': memory_percent,
                'gazebo_fps': gazebo_fps
            })
            
            print(f"Time: {snapshots[-1]['timestamp']:.1f}s, CPU: {cpu_percent}%, Mem: {memory_percent}%, FPS: {gazebo_fps}")
        
        # Calculate averages
        avg_cpu = sum(s['cpu'] for s in snapshots) / len(snapshots)
        avg_mem = sum(s['memory'] for s in snapshots) / len(snapshots)
        avg_fps = sum(s['gazebo_fps'] for s in snapshots) / len(snapshots) if snapshots else 0
        
        print(f"\nAverage stats: CPU={avg_cpu:.1f}%, Memory={avg_mem:.1f}%, Gazebo FPS={avg_fps:.1f}")
        
        # Generate recommendations
        recommendations = monitor.get_optimization_recommendations(avg_cpu, avg_mem, avg_fps)
        
        print("\nOptimization Recommendations:")
        for rec in recommendations:
            print(f"  - {rec}")
            
        if not recommendations:
            print("  - System performance looks good!")
            
    except KeyboardInterrupt:
        print("\nPerformance monitoring stopped by user.")
```

### Step 2: Optimize Gazebo Configuration

Create an optimized world configuration: `~/digital_twin_optimization/optimized_world.sdf`

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="optimized_world">
    <gravity>0 0 -9.8</gravity>

    <!-- Optimized physics configuration -->
    <physics name="optimized_physics" type="ode">
      <max_step_size>0.01</max_step_size>         <!-- Larger step for performance -->
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>  <!-- Lower update rate for performance -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>20</iters>        <!-- Balance between accuracy and performance -->
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>1e-5</cfm>          <!-- Constraint Force Mixing -->
          <erp>0.2</erp>           <!-- Error Reduction Parameter -->
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

    <!-- Simplified robot with optimized collision meshes -->
    <model name="optimized_robot">
      <pose>0 0 0.2 0 0 0</pose>
      <link name="chassis">
        <inertial>
          <mass>5.0</mass>
          <inertia ixx="0.416" ixy="0" ixz="0" iyy="0.416" iyz="0" izz="0.833"/>
        </inertial>
        
        <visual name="chassis_visual">
          <geometry>
            <box size="0.3 0.3 0.15"/>
          </geometry>
          <material>
            <ambient>0.2 0.2 1 1</ambient>
            <diffuse>0.2 0.2 1 1</diffuse>
          </material>
        </visual>
        
        <!-- Use simple collision geometry for performance -->
        <collision name="chassis_collision">
          <geometry>
            <box size="0.3 0.3 0.15"/>
          </geometry>
        </collision>
      </link>
      
      <!-- Simplified wheels -->
      <link name="wheel_left">
        <inertial>
          <mass>0.5</mass>
          <inertia ixx="0.00125" ixy="0" ixz="0" iyy="0.001875" iyz="0" izz="0.00125"/>
        </inertial>
        
        <visual name="wheel_visual">
          <geometry>
            <cylinder radius="0.1" length="0.04"/>
          </geometry>
        </visual>
        
        <collision name="wheel_collision">
          <geometry>
            <cylinder radius="0.1" length="0.04"/>
          </geometry>
        </collision>
      </link>
      
      <link name="wheel_right">
        <inertial>
          <mass>0.5</mass>
          <inertia ixx="0.00125" ixy="0" ixz="0" iyy="0.001875" iyz="0" izz="0.00125"/>
        </inertial>
        
        <visual name="wheel_visual">
          <geometry>
            <cylinder radius="0.1" length="0.04"/>
          </geometry>
        </visual>
        
        <collision name="wheel_collision">
          <geometry>
            <cylinder radius="0.1" length="0.04"/>
          </geometry>
        </collision>
      </link>
      
      <!-- Fixed joints for wheels -->
      <joint name="left_wheel_joint" type="continuous">
        <parent>chassis</parent>
        <child>wheel_left</child>
        <origin xyz="0 0.17 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
      </joint>
      
      <joint name="right_wheel_joint" type="continuous">
        <parent>chassis</parent>
        <child>wheel_right</child>
        <origin xyz="0 -0.17 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
      </joint>
    </model>
  </world>
</sdf>
```

### Step 3: Create Unity Performance Optimization Scripts

Create `~/digital_twin_optimization/unity_scripts/OptimizedRobotController.cs`:

```csharp
using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
public class OptimizedRobotController : MonoBehaviour
{
    [Header("Performance Settings")]
    public float updateInterval = 0.1f;  // Update less frequently for performance
    public int lodDistance = 20;         // Distance to switch to low-detail models
    
    [Header("Movement")]
    public float maxSpeed = 5f;
    public float maxAngularSpeed = 180f; // degrees per second
    
    [Header("Optimization")]
    public bool useLOD = true;
    public bool dynamicBatching = true;
    
    private float lastUpdate = 0f;
    private Transform leftWheel;
    private Transform rightWheel;
    
    // Components that can be disabled at distance
    private Renderer[] detailedRenderers;
    private Collider[] colliders;
    
    void Start()
    {
        // Find robot components
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach(Transform child in allChildren)
        {
            if(child.name.Contains("Wheel") || child.name.Contains("wheel"))
            {
                if(child.name.Contains("Left") || child.name.Contains("left"))
                    leftWheel = child;
                else if(child.name.Contains("Right") || child.name.Contains("right"))
                    rightWheel = child;
            }
        }
        
        // Cache components for optimization
        detailedRenderers = GetComponentsInChildren<Renderer>();
        colliders = GetComponentsInChildren<Collider>();
    }
    
    void Update()
    {
        // Only update at specific intervals to save performance
        if (Time.time - lastUpdate > updateInterval)
        {
            ApplyOptimizations();
            lastUpdate = Time.time;
        }
        
        // Update wheel rotation smoothly between performance updates
        UpdateWheelRotation();
    }
    
    void ApplyOptimizations()
    {
        if (Camera.main == null) return;
        
        float distanceToCamera = Vector3.Distance(transform.position, Camera.main.transform.position);
        
        // Apply Level of Detail
        if (useLOD)
        {
            ApplyLOD(distanceToCamera);
        }
        
        // Apply other optimizations based on distance
        ApplyDistanceBasedOptimizations(distanceToCamera);
    }
    
    void ApplyLOD(float distance)
    {
        // Disable detailed rendering when far away
        foreach(Renderer renderer in detailedRenderers)
        {
            if(renderer != null)
            {
                renderer.enabled = distance < lodDistance;
            }
        }
        
        // Potentially switch to a simplified mesh at distance
        // This would require multiple mesh LODs in a real implementation
    }
    
    void ApplyDistanceBasedOptimizations(float distance)
    {
        // Disable physics for distant robots
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.detectCollisions = distance < 50f;  // Only detect collisions when close
        }
        
        // Disable colliders for distant robots
        foreach(Collider col in colliders)
        {
            if(col != null)
            {
                col.enabled = distance < 30f;
            }
        }
    }
    
    void UpdateWheelRotation()
    {
        // Smooth animation independent of performance update intervals
        // This would be driven by received data in a real implementation
        
        // Example: Rotate wheels at a constant speed for demonstration
        if (leftWheel != null)
        {
            leftWheel.Rotate(Vector3.right, maxAngularSpeed * Time.deltaTime);
        }
        
        if (rightWheel != null)
        {
            rightWheel.Rotate(Vector3.right, maxAngularSpeed * Time.deltaTime);
        }
    }
    
    // Method to update with real data from Gazebo/ROS
    public void UpdateWithRealData(float leftWheelVelocity, float rightWheelVelocity)
    {
        // Apply real velocities to wheels
        if (leftWheel != null)
        {
            leftWheel.Rotate(Vector3.right, leftWheelVelocity * Time.deltaTime * Mathf.Rad2Deg);
        }
        
        if (rightWheel != null)
        {
            rightWheel.Rotate(Vector3.right, rightWheelVelocity * Time.deltaTime * Mathf.Rad2Deg);
        }
    }
}
```

## Part 2: Troubleshooting Common Issues

### Step 4: Create Troubleshooting Guide and Scripts

Create `~/digital_twin_optimization/troubleshooting_guide.md`:

```markdown
# Digital Twin Troubleshooting Guide

## Common Issues and Solutions

### Gazebo Issues

**Problem**: Gazebo crashes or runs very slowly
- Solution: Check system resources, reduce model complexity, adjust physics parameters
- Check: `nvidia-smi` (for GPU) or `htop` (for CPU/Memory)

**Problem**: Robot falls through ground or doesn't respond to physics
- Solution: Verify mass and inertia parameters in URDF
- Check: `rosrun xacro xacro your_robot.urdf.xacro > output.urdf && check_urdf output.urdf`

**Problem**: Sensors don't publish data
- Solution: Verify plugin configurations and namespace settings
- Check: `ros2 topic list` and `ros2 topic echo` on sensor topics

### Unity Issues

**Problem**: Unity doesn't receive ROS messages
- Solution: Verify IP addresses, firewall settings, and ROS bridge status
- Check: `ros2 run tf2_tools view_frames` and network connectivity

**Problem**: High latency between Gazebo and Unity
- Solution: Optimize network settings, reduce message frequency, use compression
- Check: Network bandwidth and ROS bridge logs

### Communication Issues

**Problem**: Data synchronization problems
- Solution: Implement proper timestamp handling and interpolation
- Check: Use `ros2 bag` to record and analyze timing issues

## System Diagnostics Script

Run the following script to diagnose common issues:

```bash
#!/bin/bash
echo "=== Digital Twin System Diagnostic ==="

# Check if ROS is installed and sourced
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 is installed"
    echo "ROS_DISTRO: $ROS_DISTRO"
else
    echo "✗ ROS 2 is not installed or not in PATH"
fi

# Check if Gazebo is installed
if command -v gz &> /dev/null; then
    echo "✓ Gazebo is installed"
    gz --version
else
    echo "✗ Gazebo is not installed"
fi

# Check Unity (check if Unity Hub exists)
if command -v Unity &> /dev/null || [ -d "/Applications/Unity Hub.app" ] || [ -d "$HOME/UnityHub" ]; then
    echo "✓ Unity is available"
else
    echo "⚠ Unity installation not detected"
fi

# Check network connectivity (try to ping localhost)
if ping -c 1 127.0.0.1 &> /dev/null; then
    echo "✓ Network connectivity OK"
else
    echo "✗ Network connectivity issue"
fi

# Check common ports used by ROS bridge
if netstat -tuln | grep -q ':9090'; then
    echo "✓ ROS bridge port (9090) is in use"
else
    echo "⚠ ROS bridge port (9090) is not in use"
fi

echo "=== Diagnostic Complete ==="
```
"""

### Step 5: Create Advanced Troubleshooting Script

Create `~/digital_twin_optimization/diagnostic_runner.py`:

```python
#!/usr/bin/env python3
"""
Advanced diagnostic tool for digital twin systems
"""
import subprocess
import sys
import os
import time
import socket
from urllib.request import urlopen
import psutil

def check_ros():
    """Check ROS installation and basic functionality"""
    print("Checking ROS installation...")
    try:
        result = subprocess.run(['ros2', 'topic', '--help'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✓ ROS 2 installed and accessible")
            return True
        else:
            print("✗ ROS 2 not accessible")
            return False
    except FileNotFoundError:
        print("✗ ROS 2 not installed or not in PATH")
        return False
    except subprocess.TimeoutExpired:
        print("✗ ROS 2 command timed out")
        return False

def check_gazebo():
    """Check Gazebo installation"""
    print("Checking Gazebo installation...")
    try:
        result = subprocess.run(['gz', '--version'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✓ Gazebo installed: {result.stdout.strip()}")
            return True
        else:
            print("✗ Gazebo not accessible")
            return False
    except FileNotFoundError:
        print("✗ Gazebo not installed")
        return False
    except subprocess.TimeoutExpired:
        print("✗ Gazebo command timed out")
        return False

def check_network_port(port):
    """Check if a network port is available"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(('localhost', port))
            s.close()
            return True
        except OSError:
            return False

def check_ros_bridge_connection():
    """Check if ROS bridge is accessible"""
    print("Checking ROS bridge connection...")
    try:
        # Try to access ROS bridge default port
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = sock.connect_ex(('localhost', 9090))
        sock.close()
        
        if result == 0:
            print("✓ ROS bridge appears to be running on port 9090")
            return True
        else:
            print("⚠ ROS bridge not responding on port 9090")
            # Check if it might be running on a different port
            for port in [9091, 9092, 11311]:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                result = sock.connect_ex(('localhost', port))
                sock.close()
                if result == 0:
                    print(f"✓ ROS bridge found on port {port}")
                    return True
            return False
    except Exception as e:
        print(f"✗ Error checking ROS bridge: {e}")
        return False

def check_system_resources():
    """Check system resources"""
    print("Checking system resources...")
    
    cpu_percent = psutil.cpu_percent(interval=1)
    memory_percent = psutil.virtual_memory().percent
    disk_percent = psutil.disk_usage('/').percent
    
    print(f"  CPU usage: {cpu_percent}%")
    print(f"  Memory usage: {memory_percent}%")
    print(f"  Disk usage: {disk_percent}%")
    
    issues = []
    if cpu_percent > 90:
        issues.append("High CPU usage (>90%)")
    if memory_percent > 90:
        issues.append("High memory usage (>90%)")
    if disk_percent > 95:
        issues.append("Low disk space (<5%)")
    
    if issues:
        print("⚠ Issues found:")
        for issue in issues:
            print(f"  - {issue}")
        return False
    else:
        print("✓ System resources look OK")
        return True

def run_diagnostics():
    """Run all diagnostic checks"""
    print("Starting Digital Twin Diagnostic Tool")
    print("="*50)
    
    results = {
        'ros': check_ros(),
        'gazebo': check_gazebo(),
        'ros_bridge': check_ros_bridge_connection(),
        'resources': check_system_resources()
    }
    
    print("="*50)
    print("Diagnostic Summary:")
    print(f"  ROS: {'✓' if results['ros'] else '✗'}")
    print(f"  Gazebo: {'✓' if results['gazebo'] else '✗'}")
    print(f"  ROS Bridge: {'✓' if results['ros_bridge'] else '✗'}")
    print(f"  System Resources: {'✓' if results['resources'] else '✗'}")
    
    if all(results.values()):
        print("\n✓ All systems appear to be functioning correctly!")
        return True
    else:
        print("\n⚠ Issues detected. Please address the above problems before continuing.")
        return False

if __name__ == '__main__':
    success = run_diagnostics()
    sys.exit(0 if success else 1)
```

## Part 3: Ensuring Reproducibility

### Step 6: Create Docker Configuration for Reproducibility

Create `~/digital_twin_optimization/Dockerfile`:

```dockerfile
# Use ROS 2 Humble with Ubuntu 22.04
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV GAZEBO_VERSION=garden

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    wget \
    curl \
    git \
    vim \
    nano \
    htop \
    net-tools \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Garden
RUN apt-get update && \
    apt install -y \
    software-properties-common \
    lsb-release \
    && rm -rf /var/lib/apt/lists/* && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo.list > /dev/null && \
    apt-get update && \
    apt-get install -y gazebo \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /workspace/ros2_ws/src
WORKDIR /workspace/ros2_ws

# Copy example packages (if any)
# COPY . src/

# Source ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Build workspace if needed
# RUN . /opt/ros/humble/setup.sh && \
#     colcon build --packages-select <your_packages>

# Set up proper entrypoint
CMD ["bash"]
```

### Step 7: Create Docker Compose for Multi-Container Setup

Create `~/digital_twin_optimization/docker-compose.yml`:

```yaml
version: '3.8'

services:
  ros-core:
    build: .
    container_name: ros_humble_core
    privileged: true
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./ros2_ws:/workspace/ros2_ws
    network_mode: host
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        cd /workspace/ros2_ws &&
        bash
      "
    
  gazebo:
    build: .
    container_name: gazebo_server
    privileged: true
    environment:
      - DISPLAY=${DISPLAY}
      - GAZEBO_MODEL_PATH=/models:/usr/share/gazebo-8/models
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./models:/models
    network_mode: host
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        gazebo --verbose
      "

  rosbridge:
    build: .
    container_name: rosbridge_server
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    network_mode: host
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        ros2 run rosbridge_server rosbridge_websocket --port 9090
      "
```

### Step 8: Create Environment Setup Script

Create `~/digital_twin_optimization/setup_environment.sh`:

```bash
#!/bin/bash
# Complete environment setup for digital twin reproducibility

set -e  # Exit on any error

echo "Setting up Digital Twin Environment..."

# Create directory structure
mkdir -p ~/digital_twin_env/{models,worlds,src,config,logs}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check for required tools
echo "Checking for required tools..."

if command_exists ros2; then
    echo "✓ ROS 2 is installed"
else
    echo "✗ ROS 2 is not installed. Please install ROS 2 Humble Hawksbill."
    exit 1
fi

if command_exists gz; then
    echo "✓ Gazebo is installed"
else
    echo "✗ Gazebo is not installed. Please install Gazebo Garden."
    exit 1
fi

# Create a python virtual environment
echo "Creating Python virtual environment..."
python3 -m venv ~/digital_twin_env/venv
source ~/digital_twin_env/venv/bin/activate
pip install --upgrade pip
pip install numpy scipy matplotlib opencv-python cv-bridge

# Create a requirements file for Python dependencies
cat > ~/digital_twin_env/requirements.txt << EOF
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.5.0
opencv-python>=4.5.0
psutil>=5.8.0
# ROS Bridge will be installed through ROS packages
EOF

echo "Installing Python dependencies..."
pip install -r ~/digital_twin_env/requirements.txt

# Create environment config file
cat > ~/digital_twin_env/digital_twin_config.env << EOF
# Digital Twin Environment Configuration
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/digital_twin_env/models
export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:~/digital_twin_env/models
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export ROS_BRIDGE_PORT=9090
export UNITY_ROS_IP=127.0.0.1
export UNITY_ROS_PORT=10000
EOF

# Source the configuration
echo "source ~/digital_twin_env/digital_twin_config.env" >> ~/.bashrc

echo "Environment setup complete!"
echo "To activate your environment, run: source ~/digital_twin_env/venv/bin/activate"
echo "To load the configuration, run: source ~/digital_twin_env/digital_twin_config.env"

# Create a quick start script
cat > ~/digital_twin_env/quick_start.sh << 'EOF'
#!/bin/bash
# Quick start script for Digital Twin Environment

# Source ROS
source /opt/ros/humble/setup.bash

# Source our environment
source ~/digital_twin_env/digital_twin_config.env

# Activate Python environment
source ~/digital_twin_env/venv/bin/activate

echo "Digital Twin environment loaded!"
echo "You can now run:"
echo "  - gazebo (for Gazebo simulation)"
echo "  - ros2 run ... (for ROS nodes)"
echo "  - python3 ... (for Python scripts with digital twin dependencies)"
EOF

chmod +x ~/digital_twin_env/quick_start.sh

echo "Quick start script created: ~/digital_twin_env/quick_start.sh"
echo "Run this script to load your digital twin environment in new terminals."

# Create a README with usage instructions
cat > ~/digital_twin_env/README.md << 'EOF'
# Digital Twin Environment

This directory contains the complete environment setup for the digital twin system.

## Getting Started

1. Activate the Python environment:
   ```bash
   source ~/digital_twin_env/venv/bin/activate
   ```

2. Source the environment configuration:
   ```bash
   source ~/digital_twin_env/digital_twin_config.env
   ```

3. Or use the quick start script:
   ```bash
   ~/digital_twin_env/quick_start.sh
   ```

## Running the System

### Method 1: Native
1. Terminal 1 - Start Gazebo:
   ```bash
   source ~/digital_twin_env/digital_twin_config.env
   gz sim -r /path/to/your/world.sdf
   ```

2. Terminal 2 - Start ROS bridge:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/digital_twin_env/digital_twin_config.env
   ros2 run rosbridge_server rosbridge_websocket --port 9090
   ```

3. Terminal 3 - Run ROS nodes:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/digital_twin_env/digital_twin_config.env
   cd ~/digital_twin_env/src
   python3 your_ros_node.py
   ```

### Method 2: Docker
1. Build and run containers:
   ```bash
   cd ~/digital_twin_optimization
   docker-compose up -d
   ```

## Troubleshooting

- If Gazebo models are not found, verify GAZEBO_MODEL_PATH is set correctly
- If ROS nodes can't communicate, check ROS_DOMAIN_ID and network settings
- For Unity communication issues, verify ROS bridge IP and port settings
EOF

echo "Setup complete! See ~/digital_twin_env/README.md for detailed instructions."
```

## Part 4: Testing Reproducibility

### Step 9: Create System Validation Script

Create `~/digital_twin_optimization/validate_environment.py`:

```python
#!/usr/bin/env python3
"""
Validation script to ensure the digital twin environment is properly configured
"""
import os
import sys
import subprocess
import platform
import psutil
from pathlib import Path

def validate_ros():
    """Validate ROS installation and basic functionality"""
    print("Validating ROS installation...")
    
    try:
        result = subprocess.run(['ros2', '--version'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"  ✓ ROS 2 version: {result.stdout.strip()}")
            return True
        else:
            print(f"  ✗ ROS 2 check failed: {result.stderr}")
            return False
    except Exception as e:
        print(f"  ✗ ROS 2 validation error: {e}")
        return False

def validate_gazebo():
    """Validate Gazebo installation"""
    print("Validating Gazebo installation...")
    
    try:
        result = subprocess.run(['gz', '--version'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"  ✓ Gazebo version: {result.stdout.strip()}")
            return True
        else:
            print(f"  ✗ Gazebo check failed: {result.stderr}")
            return False
    except Exception as e:
        print(f"  ✗ Gazebo validation error: {e}")
        return False

def validate_environment_vars():
    """Validate required environment variables"""
    print("Validating environment variables...")
    
    required_vars = [
        'GAZEBO_MODEL_PATH',
        'ROS_DOMAIN_ID',
        'RMW_IMPLEMENTATION'
    ]
    
    all_valid = True
    for var in required_vars:
        if os.environ.get(var):
            print(f"  ✓ {var} = {os.environ[var]}")
        else:
            print(f"  ✗ {var} is not set")
            all_valid = False
    
    return all_valid

def validate_directories():
    """Validate required directories exist"""
    print("Validating required directories...")
    
    required_dirs = [
        Path.home() / 'digital_twin_env',
        Path.home() / 'digital_twin_env' / 'models',
        Path.home() / 'digital_twin_env' / 'worlds',
        Path.home() / 'digital_twin_env' / 'src',
        Path.home() / 'digital_twin_env' / 'venv'
    ]
    
    all_exist = True
    for directory in required_dirs:
        if directory.exists():
            print(f"  ✓ {directory}")
        else:
            print(f"  ✗ Missing: {directory}")
            all_exist = False
    
    return all_exist

def validate_python_packages():
    """Validate required Python packages are installed"""
    print("Validating Python packages...")
    
    required_packages = [
        ('numpy', '1.21.0'),
        ('scipy', '1.7.0'),
        ('matplotlib', '3.5.0'),
        ('opencv-python', '4.5.0'),
        ('psutil', '5.8.0')
    ]
    
    all_installed = True
    for package, min_version in required_packages:
        try:
            __import__(package)
            print(f"  ✓ {package}")
        except ImportError:
            print(f"  ✗ {package} not installed")
            all_installed = False
    
    return all_installed

def validate_system_resources():
    """Validate system has sufficient resources"""
    print("Validating system resources...")
    
    # Check available memory (at least 2GB free recommended)
    memory = psutil.virtual_memory()
    free_memory_gb = memory.available / (1024**3)
    
    if free_memory_gb >= 2.0:
        print(f"  ✓ Memory available: {free_memory_gb:.1f} GB")
    else:
        print(f"  ⚠ Low memory available: {free_memory_gb:.1f} GB (recommended >2GB)")
    
    # Check disk space (at least 5GB free recommended)
    disk = psutil.disk_usage('/')
    free_disk_gb = disk.free / (1024**3)
    
    if free_disk_gb >= 5.0:
        print(f"  ✓ Disk space available: {free_disk_gb:.1f} GB")
    else:
        print(f"  ⚠ Low disk space: {free_disk_gb:.1f} GB available")
    
    # Check CPU cores (at least 2 recommended)
    cpu_count = os.cpu_count()
    if cpu_count >= 2:
        print(f"  ✓ CPU cores available: {cpu_count}")
    else:
        print(f"  ⚠ Low CPU cores: {cpu_count} (recommended >=2)")
    
    return free_memory_gb >= 1.0 and free_disk_gb >= 1.0 and cpu_count >= 1

def validate_network():
    """Validate network connectivity for ROS bridge"""
    print("Validating network connectivity...")
    
    import socket
    
    # Check if we can bind to common ROS ports
    test_ports = [9090, 11311, 10000]
    available_ports = []
    
    for port in test_ports:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('localhost', port))
                available_ports.append(port)
        except OSError:
            # Port might be in use, which is fine if it's by a needed service
            pass  # We'll just note that it's taken
    
    print(f"  ✓ Network connectivity test passed")
    print(f"    Available ports: {available_ports}")
    
    return True

def run_complete_validation():
    """Run all validation checks"""
    print("Running Digital Twin Environment Validation")
    print("=" * 50)
    
    checks = [
        ("ROS Installation", validate_ros),
        ("Gazebo Installation", validate_gazebo),
        ("Environment Variables", validate_environment_vars),
        ("Required Directories", validate_directories),
        ("Python Packages", validate_python_packages),
        ("System Resources", validate_system_resources),
        ("Network Connectivity", validate_network)
    ]
    
    results = {}
    for name, check_func in checks:
        print(f"\n{name}:")
        results[name] = check_func()
    
    print("\n" + "=" * 50)
    print("Validation Summary:")
    
    passed = sum(results.values())
    total = len(results)
    
    for name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {name}: {status}")
    
    print(f"\nOverall: {passed}/{total} checks passed")
    
    if passed == total:
        print("🎉 Environment is properly configured for digital twin development!")
        return True
    else:
        print("⚠ Some issues were found. Please address them before proceeding.")
        return False

if __name__ == '__main__':
    success = run_complete_validation()
    sys.exit(0 if success else 1)
```

## Part 5: Running the Complete Exercise

### Step 10: Execute the Optimization and Validation Process

Follow these steps to complete the exercise:

1. **Set up the environment:**
   ```bash
   cd ~/digital_twin_optimization
   chmod +x setup_environment.sh
   ./setup_environment.sh
   ```

2. **Run the performance monitor:**
   ```bash
   source ~/digital_twin_env/quick_start.sh
   python3 performance_monitor.py
   ```

3. **Run the diagnostic tool:**
   ```bash
   source ~/digital_twin_env/quick_start.sh
   python3 diagnostic_runner.py
   ```

4. **Validate the complete environment:**
   ```bash
   source ~/digital_twin_env/quick_start.sh
   python3 validate_environment.py
   ```

5. **Test the optimized system with Gazebo:**
   ```bash
   # In one terminal
   source ~/digital_twin_env/digital_twin_config.env
   gz sim -r optimized_world.sdf
   
   # In another terminal
   source /opt/ros/humble/setup.bash
   source ~/digital_twin_env/digital_twin_config.env
   python3 performance_monitor.py
   ```

## Conclusion

In this final exercise, you've learned essential skills for maintaining and deploying digital twin systems:

1. **Performance Optimization**: Techniques to improve both Gazebo simulation and Unity rendering performance
2. **Troubleshooting**: Methods to diagnose and fix common issues in digital twin systems
3. **Reproducibility**: Creating consistent environments that work across different systems

These skills are crucial for deploying digital twin systems in production environments where reliability and performance are critical.

## Best Practices Summary

- Always validate system configurations before deployment
- Monitor system resources during operation
- Use containerization for consistent environments
- Implement proper error handling and logging
- Optimize assets for real-time performance
- Test across different hardware configurations

This completes the 5th and final hands-on exercise for the Digital Twin module. You now have a comprehensive understanding of creating, optimizing, and maintaining digital twin systems using Gazebo and Unity.
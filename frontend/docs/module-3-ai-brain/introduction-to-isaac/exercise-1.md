# Exercise 1: Isaac Ecosystem Exploration

**Estimated Completion Time**: 45 minutes

**Learning Objectives**:
- Understand the main components of the NVIDIA Isaac ecosystem
- Explore Isaac Sim interface and basic functionality
- Run Isaac ROS packages in a simulated environment
- Observe hardware acceleration benefits

**Prerequisites**:
- NVIDIA GPU with CUDA support
- Isaac Sim installed and configured
- Isaac ROS packages installed
- Basic understanding of ROS 2 concepts

**Environment Requirements**:
- Ubuntu 22.04 LTS
- NVIDIA driver 495 or newer
- Isaac Sim 2023.1 or later
- ROS 2 Humble Hawksbill
- Isaac ROS navigation packages

## Introduction

This exercise introduces you to the NVIDIA Isaac ecosystem by exploring its main components and observing their integration. You'll launch Isaac Sim, run Isaac ROS packages in simulation, and observe how hardware acceleration improves performance compared to CPU-only implementations.

## Step-by-Step Instructions

### Step 1: Launch Isaac Sim
1. Open a terminal and navigate to your Isaac Sim directory:
```bash
cd ~/isaac-sim
```

2. Launch Isaac Sim with the default settings:
```bash
./isaac-sim.py
```

3. Wait for Isaac Sim to fully load. You should see the Omniverse interface with a default scene.

**Expected Result**: Isaac Sim GUI opens and loads the default environment without errors.

### Step 2: Load a Basic Robot Scene
1. In Isaac Sim, go to the "Quick Access" panel
2. Select "Isaac Examples" > "Robot Systems" > "Simple Robot"
3. This will load a basic wheeled robot in a simple environment

**Expected Result**: A wheeled robot appears in the simulation environment with basic physics properties.

### Step 3: Set Up Isaac ROS Bridge
1. In Isaac Sim, go to "Window" > "Extensions" to open the Extensions window
2. Find and enable "ROS2 Bridge" extension if not already enabled
3. In a new terminal, source ROS 2 and Isaac ROS:
```bash
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh
```

4. Use the ROS Bridge to publish the robot's state:
```bash
# In Isaac Sim, create a ROS publisher for robot state
# Go to the robot prim, right-click, and select "Add" > "ROS Component" > "Publish Robot State"
```

### Step 4: Launch Isaac ROS Perception Pipeline
1. In a new terminal, create a workspace for our perception example:
```bash
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

2. Launch the Isaac ROS AprilTag detector:
```bash
# This will run on the simulated camera data
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

**Expected Result**: The AprilTag detection node starts and waits for camera data.

### Step 5: Create Fiducial Markers in Simulation
1. In Isaac Sim, go to the Stage panel
2. Right-click in the hierarchy and select "Isaac Examples" > "Sensors Suite" > "Fiducials" > "AprilTag Board"
3. Position the AprilTag board in view of the robot's camera

### Step 6: Run Simulation and Observe Detection
1. In Isaac Sim, press the "Play" button to start the simulation
2. Watch the Isaac Sim viewport as the robot begins to process the camera feed
3. Check the ROS2 terminal where you launched the AprilTag node to see detection results

**Expected Result**: AprilTag detections should appear in the terminal output, showing the IDs and poses of detected tags.

### Step 7: Compare Performance Metrics
1. Note the detection rate and frame processing time in the terminal
2. In Isaac Sim, you can adjust the rendering quality to observe how this affects performance
3. Compare the results with what you might expect from a CPU-only implementation

**Expected Result**: Hardware acceleration should allow for higher frame rates and more complex processing than CPU-only alternatives.

## Validation Checkpoints

### Checkpoint 1: After launching Isaac Sim
- Expected output: Simulation environment loads without errors
- Troubleshooting: If Isaac Sim fails to launch, verify GPU drivers and CUDA installation

### Checkpoint 2: After loading the robot scene
- Expected output: Robot appears in the simulation with physics properties
- Troubleshooting: If the scene doesn't load, check Isaac Sim installation integrity

### Checkpoint 3: After AprilTag detection launches
- Expected output: AprilTag detection node starts without errors
- Troubleshooting: If the node fails to launch, verify Isaac ROS packages installation

### Checkpoint 4: After simulation begins
- Expected output: AprilTag detections are published when markers are in camera view
- Troubleshooting: If no detections occur, verify camera is pointing at AprilTag and lighting conditions

## Expected Outcome

By the end of this exercise, you should have successfully explored the main components of the Isaac ecosystem. You'll have launched Isaac Sim, integrated it with Isaac ROS packages, and observed hardware-accelerated perception in action. This demonstrates the integration between simulation and perception components of the Isaac ecosystem.

## Assessment Criteria

- **Success**: Successfully complete all steps, observe AprilTag detections, and understand Isaac ecosystem components
- **Partial Success**: Complete most steps but experience issues with detection or observation
- **Requires Review**: Complete setup but unable to observe expected results

## Troubleshooting Section

### Issue 1: Isaac Sim fails to launch
- **Description**: Isaac Sim doesn't start or crashes immediately
- **Solution**: Verify NVIDIA GPU drivers and CUDA installation; check system requirements
- **Prevention**: Ensure GPU meets minimum requirements before installation

### Issue 2: AprilTag node fails to launch
- **Description**: ROS2 launch command fails with import or dependency errors
- **Solution**: Check Isaac ROS packages installation: `dpkg -l | grep isaac-ros`
- **Prevention**: Verify package installation before running exercises

### Issue 3: No AprilTag detections
- **Description**: No detection messages appear even when markers are in view
- **Solution**: Verify camera is properly configured and pointing at AprilTag; check lighting conditions
- **Prevention**: Ensure proper camera settings and marker placement

### Issue 4: Poor performance
- **Description**: Simulation runs slowly or detection takes too long
- **Solution**: Reduce rendering quality in Isaac Sim settings; close other GPU-intensive applications
- **Prevention**: Verify sufficient GPU VRAM and system RAM before starting exercise

## Extension Activities (Optional)

1. Try different Isaac ROS packages (e.g., depth segmentation) with the simulation
2. Add more complex environments to see how performance scales
3. Experiment with different robot models available in Isaac Sim
# VSLAM Implementation Validation Exercise

This exercise validates that users can successfully follow the VSLAM section and implement a working visual SLAM system on their robot or simulator.

## Exercise: Visual SLAM Implementation with Isaac ROS

**Estimated Completion Time**: 75 minutes

**Learning Objectives**:
- Set up Isaac ROS VSLAM components in simulation
- Configure camera sensors for visual SLAM
- Execute VSLAM algorithm and generate a map
- Navigate using the generated map

**Prerequisites**:
- Completion of Introduction to Isaac and Photorealistic Simulation sections
- Isaac Sim properly installed and configured
- Isaac ROS VSLAM packages installed
- Basic understanding of ROS 2 and navigation concepts

**Environment Requirements**:
- NVIDIA GPU with CUDA support
- Isaac Sim 2023.1 or later
- ROS 2 Humble Hawksbill
- Isaac ROS VSLAM packages installed

## Introduction

This validation exercise confirms your ability to implement a working visual SLAM system using Isaac ROS packages. You'll set up a robot with camera sensors in Isaac Sim, launch the VSLAM pipeline, generate a map of the environment, and then use that map for navigation.

## Step-by-Step Instructions

### Step 1: Verify Isaac ROS VSLAM Installation
1. Open a terminal and source ROS 2 and Isaac ROS:
```bash
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh
```

2. Verify Isaac ROS VSLAM packages are installed:
```bash
dpkg -l | grep isaac-ros-visual-slam
```

3. Check for required dependencies:
```bash
ros2 pkg list | grep vslam
```

**Expected Result**: Isaac ROS VSLAM packages are installed and accessible.

### Step 2: Launch Isaac Sim with a Suitable Environment
1. Open Isaac Sim and load an environment suitable for VSLAM:
```bash
cd ~/isaac-sim
./isaac-sim.sh
```

2. In Isaac Sim, load a textured environment:
   - Go to "Quick Access" > "Isaac Examples" > "Environments" > "3D Furniture"
   - This provides rich visual features for SLAM algorithms

3. Add a robot with camera sensors:
   - Go to "Quick Access" > "Isaac Examples" > "Robot Systems" > "Carter"
   - Verify the robot has RGB and depth sensors

**Expected Result**: Isaac Sim running with a textured environment and robot equipped with cameras.

### Step 3: Configure ROS Bridge for Sensor Data
1. In Isaac Sim, ensure the ROS2 Bridge extension is enabled:
   - Go to "Window" > "Extensions" > search for "ROS2 Bridge"
   - Enable it if not already active

2. Add ROS components to the robot's cameras:
   - In the Stage panel, find your robot's camera
   - Right-click the camera and select "Add" > "ROS Component" > "Publish ROS Camera"
   - Set the topic name to `/camera/image_rect_color` for RGB
   - Set the topic name to `/camera/depth/image_rect_raw` for depth
   - Set the camera info topic to `/camera/camera_info`

**Expected Result**: Camera data publishing to ROS topics.

### Step 4: Launch Isaac ROS VSLAM Pipeline
1. Open a new terminal and source the necessary environments:
```bash
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh
```

2. Launch the Isaac ROS VSLAM node:
```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
  use_sim_time:=True \
  enable_rectification:=True \
  enable_fisheye:=False
```

3. Verify the node is running and listening to the correct topics:
```bash
# Check topics
ros2 topic list | grep camera
ros2 topic list | grep slam

# Check node status
ros2 node list | grep visual_slam
```

**Expected Result**: Isaac ROS VSLAM node is running and receiving camera data.

### Step 5: Drive the Robot to Generate a Map
1. In Isaac Sim, start the simulation:
   - Press the "Play" button to start physics simulation

2. Command the robot to move through the environment:
```bash
# Open another terminal
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh

# Send velocity commands to the robot
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2" -1
```

3. Allow the robot to move for 30-60 seconds to build the map:
   - The VSLAM node should be processing the visual input
   - Check ROS2 terminal for VSLAM processing messages

**Expected Result**: The robot moves through the environment while the VSLAM algorithm builds a map.

### Step 6: Monitor the VSLAM Output
1. Visualize the SLAM results using RViz:
```bash
# In a new terminal
source /opt/ros/humble/setup.bash
rviz2
```

2. In RViz, add the following displays:
   - Add a "TF" display and set "Fixed Frame" to "map"
   - Add a "PointCloud2" display and set topic to `/visual_slam/feature_cloud`
   - Add a "Pose" display and set topic to `/visual_slam/pose`
   - Add a "Map" display if available (from `/slam_graph`)

3. Verify that the map is being built as the robot moves:
   - Look for the pose track of the robot
   - Observe feature points being mapped

**Expected Result**: RViz shows the robot's pose and the feature map being built in real-time.

### Step 7: Save and Validate the Map
1. After mapping, stop the robot:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1
```

2. Save the map using the Isaac ROS VSLAM services (if available):
```bash
# Check available services
ros2 service list | grep slam

# If a save service is available, call it
# ros2 service call /slam_toolbox/save_map slam_toolbox/SaveMap
```

3. Alternatively, record the necessary topics:
```bash
# Record SLAM-related topics
cd ~/slam_output
ros2 bag record /visual_slam/pose /visual_slam/feature_cloud /visual_slam/traj_pose /tf
```

**Expected Result**: A map of the environment has been generated and can be saved for later use.

### Step 8: Navigation Using the Generated Map
1. If navigation capabilities are included in your setup, test navigation:
```bash
# Launch navigation stack with the generated map
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh

# Launch navigation with the generated map
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

2. Send a navigation goal:
```bash
# In another terminal
source /opt/ros/humble/setup.bash

# Send a navigation goal (if using Nav2)
ros2 run nav2_msgs SendGoal -f map [x] [y] [theta] --frame_id map
```

**Expected Result**: The robot can navigate to specified goals using the map generated by the VSLAM system.

## Validation Checkpoints

### Checkpoint 1: After verifying VSLAM installation
- Expected output: Isaac ROS VSLAM packages are installed and accessible
- Troubleshooting: If packages aren't found, reinstall Isaac ROS VSLAM packages

### Checkpoint 2: After launching Isaac Sim
- Expected output: Isaac Sim running with textured environment and robot
- Troubleshooting: If environment doesn't load, check Isaac Sim installation

### Checkpoint 3: After configuring ROS bridge
- Expected output: Camera data publishing to ROS topics
- Troubleshooting: If no data appears, verify Isaac Sim ROS bridge configuration

### Checkpoint 4: After launching VSLAM pipeline
- Expected output: VSLAM node running and receiving camera data
- Troubleshooting: If node fails, check camera topic configuration

### Checkpoint 5: After robot movement
- Expected output: VSLAM processing messages in terminal
- Troubleshooting: If no processing messages, check sensor configuration and lighting

### Checkpoint 6: After visualization in RViz
- Expected output: Robot pose and map features visible in RViz
- Troubleshooting: If nothing appears, check TF frames and topic connections

### Checkpoint 7: After map generation
- Expected output: Map of the environment created and saved
- Troubleshooting: If map is sparse or poor quality, ensure adequate visual features in environment

### Checkpoint 8: After navigation test
- Expected output: Robot successfully navigates to goal using VSLAM map
- Troubleshooting: If navigation fails, verify map quality and localization

## Expected Outcome

By the end of this exercise, you should have successfully:
- Set up Isaac ROS VSLAM in Isaac Sim
- Generated a visual map of the environment
- Demonstrated navigation using the generated map

This validates your understanding of VSLAM concepts and implementation using Isaac ROS hardware acceleration.

## Assessment Criteria

- **Success**: Complete all steps, generate a good quality map, demonstrate navigation
- **Partial Success**: Complete most steps but experience minor issues with map quality or navigation
- **Requires Review**: Unable to complete core VSLAM implementation or map generation

## Troubleshooting Section

### Issue 1: Isaac ROS VSLAM packages not found
- **Description**: Cannot find or launch Isaac ROS VSLAM nodes
- **Solution**: Verify Isaac ROS installation; reinstall missing packages
- **Prevention**: Confirm complete Isaac ROS installation before starting

### Issue 2: No camera data in ROS topics
- **Description**: Camera topics are empty or not publishing
- **Solution**: Check Isaac Sim ROS bridge configuration and camera settings
- **Prevention**: Verify ROS bridge setup before proceeding

### Issue 3: VSLAM node crashes or fails to process data
- **Description**: Isaac ROS VSLAM node terminates unexpectedly
- **Solution**: Check camera data format, resolution, and frame rate compatibility
- **Prevention**: Verify camera configuration matches VSLAM requirements

### Issue 4: Poor or sparse map generation
- **Description**: Generated map lacks detail or features
- **Solution**: Ensure environment has sufficient visual features; improve lighting
- **Prevention**: Select environments with rich visual features

### Issue 5: Navigation fails with VSLAM map
- **Description**: Robot cannot localize or navigate using VSLAM map
- **Solution**: Check localization node, verify map quality, confirm TF relationships
- **Prevention**: Validate map and localization before attempting navigation

## Extension Activities (Optional)

1. Test VSLAM with different environments (outdoor, repetitive textures)
2. Evaluate performance on different hardware configurations
3. Compare Isaac ROS VSLAM results with other SLAM implementations
4. Implement loop closure and relocalization scenarios
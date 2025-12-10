# Exercise 3: VSLAM Implementation

**Estimated Completion Time**: 90 minutes

**Learning Objectives**:
- Configure Isaac ROS VSLAM components in simulation
- Execute visual SLAM to generate a map of the environment
- Implement navigation using the generated visual map
- Validate the VSLAM and navigation performance

**Prerequisites**:
- Completion of Introduction to Isaac, Photorealistic Simulation, and VSLAM Theory sections
- Isaac Sim properly installed with RTX rendering
- Isaac ROS VSLAM packages installed
- Basic understanding of ROS 2 navigation concepts

**Environment Requirements**:
- NVIDIA GPU with CUDA support and 8GB+ VRAM
- Isaac Sim 2023.1 or later
- ROS 2 Humble Hawksbill
- Isaac ROS VSLAM and navigation packages installed

## Introduction

This hands-on exercise guides you through implementing a complete VSLAM and navigation system using Isaac ROS packages. You'll create a mapping run to build a visual map of an environment, then use that map for autonomous navigation tasks.

## Step-by-Step Instructions

### Step 1: Verify Isaac ROS VSLAM Installation
1. Open a terminal and verify Isaac ROS VSLAM packages are installed:
```bash
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh

# Check if Isaac ROS VSLAM packages are available
dpkg -l | grep isaac-ros-visual-slam
```

2. Check for related packages:
```bash
ros2 pkg list | grep visual_slam
ros2 pkg list | grep apriltag
ros2 pkg list | grep image_proc
```

3. Check your GPU compatibility:
```bash
nvidia-smi
# Should show your GPU and driver information
```

**Expected Result**: Isaac ROS VSLAM packages are installed and GPU is detected.

### Step 2: Launch Isaac Sim with VSLAM-Suitable Environment
1. Launch Isaac Sim:
```bash
cd ~/isaac-sim
./isaac-sim.sh
```

2. In Isaac Sim, load a textured environment suitable for VSLAM:
   - Go to "Quick Access" > "Isaac Examples" > "Environments" > "3D Room"
   - This environment has plenty of visual features for SLAM algorithms

3. Add a robot with stereo cameras:
   - Go to "Quick Access" > "Isaac Examples" > "Robot Systems" > "Stereo Carter"
   - This robot comes with stereo cameras pre-configured for VSLAM

4. Position the robot at a suitable starting location:
   - Use the transform gizmo to position the robot near the center of the room
   - Ensure the cameras have a clear view of the environment

**Expected Result**: Isaac Sim running with a textured environment and robot equipped with stereo cameras.

### Step 3: Configure ROS Bridge for Camera Data
1. In Isaac Sim, ensure the ROS2 Bridge extension is enabled:
   - Go to "Window" > "Extensions" > search for "ROS2 Bridge"
   - Enable it if not already active

2. Add ROS components to the stereo cameras:
   - In the Stage panel, find the left stereo camera
   - Right-click and select "Add" > "ROS Component" > "Publish ROS Camera"
   - Set the topic to `/camera/image_left` and info to `/camera/camera_info_left`
   - Repeat for the right camera, using `/camera/image_right` and `/camera/camera_info_right`

**Expected Result**: Stereo camera data publishing to ROS topics.

### Step 4: Launch Isaac ROS VSLAM Pipeline
1. Open a new terminal and source ROS environments:
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

3. Verify the node is receiving camera data:
```bash
# Check if topics are being published
ros2 topic echo /visual_slam/pose --field pose.position.x -1
# Should show changing x position values as robot moves
```

**Expected Result**: Isaac ROS VSLAM node is running and processing camera data.

### Step 5: Generate a Map with VSLAM
1. In Isaac Sim, start the simulation:
   - Press the "Play" button to start physics simulation

2. Move the robot through the environment manually:
   - You can use Isaac Sim's joint control or publish commands
```bash
# In another terminal
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh

# Send a command to move the robot forward and turn
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2" -1
```

3. Allow the robot to traverse most of the environment:
   - Move the robot to cover as much of the environment as possible
   - Try to move at a reasonable speed to allow feature tracking
   - Spend 2-3 minutes to build a comprehensive map

**Expected Result**: The VSLAM system builds a map of the environment as the robot moves.

### Step 6: Monitor VSLAM Performance in RViz
1. Open RViz in a new terminal:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

2. In RViz, set up the display:
   - Set "Fixed Frame" to "map" (this should be published by VSLAM)
   - Add a "Pose" display for `/visual_slam/pose` topic
   - Add a "PointCloud2" display for `/visual_slam/feature_cloud` topic
   - Add a "TF" display to visualize coordinate frames

3. Observe the map building in real-time:
   - You should see the robot's trajectory being plotted
   - Feature points should appear in the environment
   - The map should grow as the robot explores

**Expected Result**: Real-time visualization of map building and robot localization.

### Step 7: Evaluate Map Quality
1. Stop the robot motion:
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

2. Assess the quality of the generated map:
   - Check if the map shows the general structure of the environment
   - Look for consistent feature distribution
   - Verify that the robot's trajectory appears reasonable (no major drift)

3. Record the map quality metrics:
   - Note the number of features in the map
   - Observe the consistency of the robot's path
   - Check for loop closures if the robot passed the same area multiple times

**Expected Result**: A reasonably complete map of the environment with consistent robot trajectory.

### Step 8: Launch Navigation Stack with the Generated Map
1. If Navigation2 is available, launch it with the VSLAM map:
```bash
# In a new terminal
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/path/to/nav2_params.yaml
```

2. Set an initial pose in RViz:
   - In RViz, use the "2D Pose Estimate" tool to set the robot's initial position
   - Click on the map at the robot's actual location and orient the arrow correctly

3. Send a navigation goal:
   - Use the "2D Nav Goal" tool in RViz to set a destination
   - Click on the map where you want the robot to navigate

**Expected Result**: Robot autonomously navigates to the goal using the VSLAM map.

### Step 9: Validate Navigation Performance
1. Observe the navigation execution:
   - Watch the robot's path planning and execution in RViz
   - Verify the robot follows the planned path safely

2. Record navigation metrics:
   - Time to reach the goal
   - Path length compared to straight-line distance
   - Number of recovery behaviors triggered (if any)

3. Test multiple goals:
   - Set 2-3 different navigation goals to validate performance
   - Try goals in different areas of the environment

**Expected Result**: Successful navigation to multiple goals using the VSLAM map.

## Validation Checkpoints

### Checkpoint 1: After verifying VSLAM installation
- Expected output: Isaac ROS VSLAM packages are accessible
- Troubleshooting: If packages aren't found, reinstall Isaac ROS packages

### Checkpoint 2: After launching Isaac Sim with environment
- Expected output: Isaac Sim running with textured environment and robot
- Troubleshooting: If environment doesn't load, check Isaac Sim installation

### Checkpoint 3: After configuring ROS bridge for cameras
- Expected output: Stereo camera data published to ROS topics
- Troubleshooting: If no data appears, check Isaac Sim ROS bridge configuration

### Checkpoint 4: After launching VSLAM pipeline
- Expected output: VSLAM node running and processing camera data
- Troubleshooting: If node fails, verify camera topic configuration

### Checkpoint 5: After generating map
- Expected output: VSLAM processing messages and pose updates
- Troubleshooting: If no processing occurs, check camera data and lighting

### Checkpoint 6: After RViz visualization
- Expected output: Real-time visualization of trajectory and features
- Troubleshooting: If visualization is empty, check TF tree and topics

### Checkpoint 7: After map evaluation
- Expected output: Consistent map showing environment structure
- Troubleshooting: If map is sparse, ensure environment has visual features

### Checkpoint 8: After launching navigation
- Expected output: Navigation stack running with VSLAM map
- Troubleshooting: If navigation fails, verify map quality and localization

### Checkpoint 9: After validation
- Expected output: Successful navigation to multiple goals
- Troubleshooting: If navigation fails, adjust parameters or check maps

## Expected Outcome

By the end of this exercise, you should have successfully:
- Set up Isaac ROS VSLAM with stereo cameras in Isaac Sim
- Generated a 3D map of the environment using visual SLAM
- Used the map for autonomous navigation tasks
- Validated the performance of both mapping and navigation

This demonstrates complete implementation of Visual SLAM and navigation using Isaac ROS hardware acceleration.

## Assessment Criteria

- **Success**: Complete all steps, generate good quality map, demonstrate successful navigation
- **Partial Success**: Complete most steps but experience minor issues with map quality or navigation
- **Requires Review**: Unable to complete core VSLAM implementation or navigation tasks

## Troubleshooting Section

### Issue 1: Isaac ROS VSLAM node fails to launch
- **Description**: VSLAM node terminates with errors on startup
- **Solution**: Check camera topic configuration, verify Isaac ROS installation
- **Prevention**: Confirm all prerequisites before starting exercise

### Issue 2: No VSLAM pose output
- **Description**: VSLAM node runs but doesn't publish pose information
- **Solution**: Check camera calibration, verify camera image quality
- **Prevention**: Validate camera data before launching VSLAM

### Issue 3: Poor map quality
- **Description**: Generated map is sparse or inaccurate
- **Solution**: Ensure environment has visual features, check camera settings
- **Prevention**: Use textured environments for VSLAM exercises

### Issue 4: Navigation fails with VSLAM map
- **Description**: Robot cannot navigate correctly using VSLAM map
- **Solution**: Verify localization in map, check navigation parameters
- **Prevention**: Validate map quality before attempting navigation

### Issue 5: Feature tracking failures
- **Description**: VSLAM loses track frequently during mapping
- **Solution**: Reduce robot speed, improve lighting, check camera calibration
- **Prevention**: Plan mapping path to maximize visual features

## Extension Activities (Optional)

1. Test VSLAM on different environments (outdoor, low-texture)
2. Evaluate the impact of motion blur on VSLAM performance
3. Compare Isaac ROS VSLAM with other SLAM implementations
4. Implement multi-session mapping with map merging
5. Test visual navigation in dynamic environments with moving objects
# Nav2 Path Planning Validation Exercise

This exercise validates that users can successfully implement path planning for bipedal humanoid movement with obstacle avoidance using Nav2 to enable complex locomotion behaviors.

## Exercise: Nav2 Path Planning for Bipedal Navigation

**Estimated Completion Time**: 80 minutes

**Learning Objectives**:
- Configure Nav2 for bipedal humanoid movement with specific constraints
- Implement obstacle avoidance for humanoid robots
- Plan and execute complex paths with humanoid-specific considerations
- Validate path planning performance for bipedal locomotion

**Prerequisites**:
- Completion of Introduction to Isaac, Photorealistic Simulation, and VSLAM sections
- Isaac Sim properly installed with RTX rendering
- Isaac ROS packages including navigation components installed
- Basic understanding of ROS 2 navigation concepts and Nav2

**Environment Requirements**:
- NVIDIA GPU with CUDA support
- Isaac Sim 2023.1 or later
- ROS 2 Humble Hawksbill
- Isaac ROS navigation packages installed
- Nav2 packages installed

## Introduction

This validation exercise confirms your ability to implement Nav2 path planning specifically for bipedal humanoid movement with obstacle avoidance. You'll configure Nav2 with parameters tailored for humanoid dynamics, set up a simulation environment with obstacles, and execute path planning and navigation tasks.

## Step-by-Step Instructions

### Step 1: Verify Nav2 Installation and Isaac ROS Integration
1. Open a terminal and verify Nav2 packages are installed:
```bash
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh

# Check Nav2 packages
ros2 pkg list | grep nav2
```

2. Verify Isaac ROS navigation packages:
```bash
dpkg -l | grep isaac-ros-nav
```

3. Check for required dependencies:
```bash
# Check for Isaac Sim navigation assets
ls ~/isaac-sim/isaac_sim_assets/Isaac/Robots/
```

**Expected Result**: Nav2 and Isaac ROS navigation packages are installed and accessible.

### Step 2: Launch Isaac Sim with Navigation Environment
1. Launch Isaac Sim:
```bash
cd ~/isaac-sim
./isaac-sim.sh
```

2. In Isaac Sim, load an environment suitable for navigation with obstacles:
   - Go to "Quick Access" > "Isaac Examples" > "Environments" > "Tennis Court"
   - This environment provides a large, open space with some obstacles

3. Add a humanoid robot model:
   - Go to "Quick Access" > "Isaac Examples" > "Robot Systems" > "Carter" (or appropriate humanoid model)
   - Position the robot in an open area of the environment

4. Add obstacles for navigation challenges:
   - Use the "Create" menu to add static objects (cubes, cylinders) as obstacles
   - Place obstacles to create navigation challenges (narrow passages, detours)

**Expected Result**: Isaac Sim running with a navigation environment and humanoid robot with obstacles.

### Step 3: Configure ROS Bridge for Navigation
1. In Isaac Sim, ensure the ROS2 Bridge extension is enabled:
   - Go to "Window" > "Extensions" > search for "ROS2 Bridge"
   - Enable it if not already active

2. Add navigation-specific ROS components to the robot:
   - Add "Publish Robot State" to publish TF and joint states
   - Add "Subscribe Twist" to receive velocity commands
   - Add any necessary sensor components (LIDAR, camera) for navigation

**Expected Result**: Robot state publishing and velocity command subscription configured.

### Step 4: Configure Humanoid-Specific Nav2 Parameters
1. Create a custom Nav2 configuration file for humanoid navigation:

```yaml
# ~/nav2_humanoid_config.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_thresh: 0.5
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    set_initial_pose: true
    set_initial_pose_: false
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the BT XML files are located
    behavior_tree_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # Path to our custom BT for humanoid navigation
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    # Humanoid-specific controller parameters
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    
    # Humanoid-specific velocity limits (bipedal constraints)
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 24
      control_frequency: 20.0
      nonholonomic: true  # Humanoid typically nonholonomic
      xy_goal_tolerance: 0.25  # Larger for humanoid balance
      yaw_goal_tolerance: 0.2
      stateful: true
      horizon_duration: 1.5
      control_horizon: 6
      cmd_angle_instead_rotvel: false
      # Humanoid-specific acceleration limits for stable walking
      v_linear_min: -0.5
      v_linear_max: 0.5
      v_linear_thresh: 0.1
      v_angular_min: -0.6
      v_angular_max: 0.6
      v_angular_thresh: 0.05

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Larger for humanoid safety margin
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True
  local_footprint_publisher:
    ros__parameters:
      use_sim_time: True
  footprint_info:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3  # Larger for humanoid
      resolution: 0.05
      track_unknown_space: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 1.0
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      # Use GridBased planner with humanoid-specific parameters
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5  # Larger tolerance for humanoid path planning
      use_astar: false
      allow_unknown: true
```

2. Save the configuration file as `~/nav2_humanoid_config.yaml`

**Expected Result**: Custom Nav2 configuration file created with humanoid-specific parameters.

### Step 5: Launch Navigation Stack with Humanoid Configuration
1. In a new terminal, source the environments:
```bash
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh
```

2. Launch Nav2 with your humanoid configuration:
```bash
# Create a simple launch script for Nav2 with humanoid config
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=~/nav2_humanoid_config.yaml
```

3. In another terminal, launch RViz for visualization:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

4. In RViz, add displays for:
   - Local costmap (`/local_costmap/costmap`)
   - Global costmap (`/global_costmap/costmap`)
   - Global path (`/plan`)
   - Robot pose (`/amcl_pose`)
   - Laser scan (`/scan`)

**Expected Result**: Nav2 stack running with humanoid-specific parameters and visualization in RViz.

### Step 6: Set Initial Pose and Plan Paths
1. In RViz, use the "2D Pose Estimate" tool to set the robot's initial position:
   - Click on the map where the robot is located in Isaac Sim
   - Set the orientation arrow to match the robot's heading

2. Use the "2D Nav Goal" tool to set navigation destinations:
   - Try goals that require navigating around obstacles
   - Select goals that test bipedal movement (avoid sharp turns if possible)

3. Observe the path planning and execution:
   - Watch the global path being calculated
   - Observe the robot following the path while avoiding obstacles

**Expected Result**: Successful path planning and navigation around obstacles with humanoid-specific movements.

### Step 7: Test Obstacle Avoidance Scenarios
1. Create challenging scenarios with obstacles:
   - Narrow passages that test humanoid width constraints
   - Multiple obstacles requiring complex path planning

2. Test different navigation goals:
   - Short distances to test precision
   - Long distances to test path efficiency
   - Goals behind obstacles to test planning capability

3. Monitor the safety of the navigation:
   - Ensure the robot maintains safe distances from obstacles
   - Check that the robot doesn't collide with obstacles

**Expected Result**: Successful navigation in complex obstacle scenarios with humanoid-specific constraints considered.

### Step 8: Validate Humanoid-Specific Performance
1. Assess path planning quality:
   - Path smoothness (important for bipedal stability)
   - Compliance with humanoid kinematic constraints
   - Time to reach goals

2. Evaluate obstacle avoidance:
   - Minimum safe distance maintained
   - Smooth transitions around obstacles
   - Ability to handle dead-end situations

3. Test recovery behaviors:
   - If the robot gets stuck, does it execute recovery behaviors?
   - How does it handle local minima situations?

**Expected Result**: Navigation performance that considers humanoid dynamics and constraints.

## Validation Checkpoints

### Checkpoint 1: After verifying installations
- Expected output: Nav2 and Isaac ROS navigation packages accessible
- Troubleshooting: If packages aren't found, install missing navigation packages

### Checkpoint 2: After launching Isaac Sim environment
- Expected output: Isaac Sim with navigation environment and humanoid robot
- Troubleshooting: If environment doesn't load, check Isaac Sim installation

### Checkpoint 3: After configuring ROS bridge
- Expected output: Robot state publishing and command subscription working
- Troubleshooting: If communication fails, verify ROS bridge setup

### Checkpoint 4: After creating humanoid config
- Expected output: Valid YAML configuration file with humanoid parameters
- Troubleshooting: Validate YAML syntax if Nav2 fails to load config

### Checkpoint 5: After launching Nav2 stack
- Expected output: Nav2 nodes running and communicating properly
- Troubleshooting: Check parameter file path and format

### Checkpoint 6: After setting initial pose
- Expected output: Robot pose correctly set in Nav2 localization
- Troubleshooting: If localization fails, ensure map and robot pose alignment

### Checkpoint 7: After path planning
- Expected output: Valid path generated around obstacles
- Troubleshooting: If no path generated, check costmap settings and obstacle detection

### Checkpoint 8: After navigation execution
- Expected output: Robot successfully navigating to goal while avoiding obstacles
- Troubleshooting: If navigation fails, check controller and planner parameters

## Expected Outcome

By the end of this exercise, you should have successfully:
- Configured Nav2 with humanoid-specific parameters
- Set up a navigation environment in Isaac Sim with obstacles
- Executed path planning and navigation with obstacle avoidance
- Validated that the navigation system considers humanoid dynamics

This demonstrates the implementation of path planning for bipedal humanoid movement with obstacle avoidance using Nav2.

## Assessment Criteria

- **Success**: Complete all steps, demonstrate successful navigation with obstacle avoidance, validate humanoid-specific constraints
- **Partial Success**: Complete most steps but experience minor issues with path planning or obstacle avoidance
- **Requires Review**: Unable to complete core navigation implementation or parameter configuration

## Troubleshooting Section

### Issue 1: Nav2 fails to launch with custom configuration
- **Description**: Nav2 nodes crash or fail to start with humanoid config
- **Solution**: Validate YAML syntax, check parameter compatibility with Nav2 version
- **Prevention**: Test with default config first, then modify incrementally

### Issue 2: Robot fails to navigate around obstacles
- **Description**: Robot collides with obstacles or fails to find paths
- **Solution**: Adjust costmap inflation parameters, verify sensor data
- **Prevention**: Test obstacle detection in isolation before navigation

### Issue 3: Poor path quality for humanoid
- **Description**: Generated paths have sharp turns unsuitable for bipedal movement
- **Solution**: Adjust planner and controller parameters for smoother paths
- **Prevention**: Consider humanoid kinematics in parameter selection

### Issue 4: Localization fails during navigation
- **Description**: Robot loses track of position during movement
- **Solution**: Improve map quality, adjust AMCL parameters, verify odometry
- **Prevention**: Validate localization before starting navigation

## Extension Activities (Optional)

1. Implement custom behaviors for bipedal-specific movements (e.g., climbing over small obstacles)
2. Add semantic navigation capabilities using visual perception
3. Test formation navigation with multiple humanoid robots
4. Implement terrain-aware navigation for different ground types suitable for bipedal locomotion
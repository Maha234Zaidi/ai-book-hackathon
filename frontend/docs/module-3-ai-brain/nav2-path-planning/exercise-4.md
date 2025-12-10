# Exercise 4: Bipedal Navigation

**Estimated Completion Time**: 100 minutes

**Learning Objectives**:
- Configure Nav2 specifically for bipedal humanoid movement
- Implement obstacle avoidance tailored for humanoid robots
- Execute navigation with humanoid-specific constraints
- Validate bipedal locomotion safety and efficiency

**Prerequisites**:
- Completion of Introduction to Isaac, Photorealistic Simulation, VSLAM, and Nav2 sections
- Isaac Sim properly installed with RTX rendering
- Isaac ROS navigation packages installed
- Basic understanding of ROS 2 navigation and humanoid robotics concepts

**Environment Requirements**:
- NVIDIA GPU with CUDA support
- Isaac Sim 2023.1 or later
- ROS 2 Humble Hawksbill
- Isaac ROS navigation and Nav2 packages installed
- Access to humanoid robot models in Isaac Sim

## Introduction

This hands-on exercise guides you through implementing navigation specifically for bipedal humanoid robots using Nav2. You'll configure navigation parameters for humanoid constraints, set up a simulation environment with obstacles, and execute navigation tasks that consider bipedal locomotion requirements.

## Step-by-Step Instructions

### Step 1: Verify Prerequisites and Installations
1. Open a terminal and verify required packages are installed:
```bash
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh

# Check for navigation packages
ros2 pkg list | grep nav2
dpkg -l | grep isaac-ros-nav

# Check for humanoid robot models
ls ~/isaac-sim/isaac_sim_assets/Isaac/Robots/
```

2. Verify Isaac Sim has humanoid robot assets:
```bash
# Look for humanoid robot models
ls ~/isaac_sim_assets/Isaac/Robots/*human*
```

3. Check for necessary dependencies:
```bash
# Check if custom controllers (if needed) are available
ros2 control list_controllers
```

**Expected Result**: All required packages and humanoid robot assets are available.

### Step 2: Launch Isaac Sim with Humanoid-Friendly Environment
1. Launch Isaac Sim with a suitable environment:
```bash
cd ~/isaac-sim
./isaac-sim.sh
```

2. In Isaac Sim, load an environment appropriate for humanoid navigation:
   - Go to "Quick Access" > "Isaac Examples" > "Environments" > "Simple Room" or "Apartment"
   - This environment provides clear paths with some obstacles for navigation

3. Add a humanoid robot model (if available) or use a differential robot as proxy for testing:
   - Go to "Quick Access" > "Isaac Examples" > "Robot Systems" > "Carter" (or appropriate humanoid if available)
   - Position the robot at a central location in the environment
   - Ensure the robot has appropriate sensors (LIDAR, camera) for navigation

4. Configure the environment for humanoid navigation:
   - Add obstacles that would require a bipedal robot to navigate around
   - Ensure clear paths of appropriate width for humanoid movement
   - Add gentle slopes or stairs if available (for advanced testing)

**Expected Result**: Isaac Sim running with humanoid-appropriate environment and robot.

### Step 3: Configure Humanoid-Specific Navigation Parameters
1. Create a custom Nav2 configuration file for humanoid navigation:

```yaml
# ~/nav2_humanoid_bipedal.yaml
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
    behavior_tree_xml_filename: "navigate_w_replanning_and_recovery.xml"
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
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    
    # Humanoid-specific controller parameters
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 24
      control_frequency: 20.0
      nonholonomic: true  # Humanoid typically nonholonomic
      xy_goal_tolerance: 0.4  # Larger for humanoid balance
      yaw_goal_tolerance: 0.3
      stateful: true
      horizon_duration: 1.5
      control_horizon: 6
      cmd_angle_instead_rotvel: false
      # Humanoid-specific velocity limits for stable walking
      v_linear_min: -0.3
      v_linear_max: 0.3  # Conservative speed for balance
      v_linear_thresh: 0.1
      v_angular_min: -0.4
      v_angular_max: 0.4  # Conservative turning for balance
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
      width: 8     # Larger for humanoid safety margin
      height: 8
      resolution: 0.05
      robot_radius: 0.4  # Account for humanoid width + safety
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0  # Higher for humanoid safety
        inflation_radius: 0.8
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

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.4  # Larger for humanoid
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
        cost_scaling_factor: 5.0  # Higher for humanoid safety
        inflation_radius: 1.0
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.7  # Larger tolerance for humanoid path planning
      use_astar: false
      allow_unknown: true

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    action_server_result_tolerance: 1.0
    action_server_goal_tolerance: 1.0
```

2. Save the configuration file as `~/nav2_humanoid_bipedal.yaml`

**Expected Result**: Custom Nav2 configuration file created with humanoid-specific parameters.

### Step 4: Configure ROS Bridge for Humanoid Navigation
1. In Isaac Sim, ensure the ROS2 Bridge extension is enabled:
   - Go to "Window" > "Extensions" > search for "ROS2 Bridge"
   - Enable it if not already active

2. Add necessary ROS components to the robot:
   - Add "Publish Robot State" to publish TF and joint states
   - Add "Subscribe Twist" to receive velocity commands
   - Add "Publish Laser Scan" or other sensor components for navigation
   - Ensure proper frame relationships (base_link, odom, map)

**Expected Result**: Robot properly configured for ROS communication.

### Step 5: Launch Navigation Stack with Humanoid Configuration
1. In a new terminal, source the environments:
```bash
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh
```

2. Launch Nav2 with your humanoid configuration:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=~/nav2_humanoid_bipedal.yaml
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

**Expected Result**: Nav2 stack running with humanoid-specific parameters.

### Step 6: Configure Humanoid-Specific Navigation Behavior
1. In RViz, set the initial pose for the robot:
   - Use the "2D Pose Estimate" tool
   - Click on the map where the robot is located in Isaac Sim
   - Align the arrow with the robot's orientation

2. Test the costmap visualization:
   - Verify obstacles are properly detected and inflated
   - Check that the robot footprint is appropriately sized for humanoid navigation
   - Validate that safety margins are visible

3. Adjust parameters if needed based on visualization:
   - Modify inflation radius if obstacles are too close
   - Adjust robot radius if navigation paths are too conservative
   - Validate that the robot's size is properly represented

**Expected Result**: Navigation system properly initialized with humanoid parameters.

### Step 7: Execute Basic Navigation with Humanoid Constraints
1. Use the "2D Nav Goal" tool in RViz to set a navigation goal:
   - Choose a goal that requires navigating through a corridor or around an obstacle
   - Keep the path simple for the first attempt
   - Avoid sharp turns or narrow passages initially

2. Observe the navigation execution:
   - Watch the global path being computed
   - Monitor the local path following and obstacle avoidance
   - Verify the robot maintains appropriate safety distances

3. Monitor for humanoid-specific behaviors:
   - Slower, more careful navigation compared to wheeled robots
   - Larger safety margins maintained from obstacles
   - Smooth path following without sudden direction changes

**Expected Result**: Successful navigation with humanoid-specific constraints.

### Step 8: Test Complex Obstacle Avoidance
1. Create more challenging navigation scenarios:
   - Set goals that require complex maneuvering
   - Position obstacles to test the robot's ability to navigate around them
   - Test narrow passages that require careful navigation

2. Observe the robot's behavior in challenging situations:
   - How does it handle tight spaces?
   - Does it maintain balance during complex maneuvers?
   - How does it react to unexpected obstacles?

3. Test recovery behaviors if obstacles block the path:
   - Does the robot execute appropriate recovery behaviors?
   - How does it handle local minima situations?

**Expected Result**: Successful navigation around complex obstacles with humanoid-specific safety considerations.

### Step 9: Validate Humanoid Navigation Performance
1. Assess navigation quality with humanoid-specific metrics:
   - Path smoothness (important for balance)
   - Safety margins maintained from obstacles
   - Navigation time vs. wheeled robots (should be slower but safer)
   - Stability during navigation (no oscillations)

2. Test multiple navigation goals:
   - Short distances for precision
   - Long distances for path efficiency
   - Complex routes with multiple obstacles
   - Goals in different areas of the environment

3. Evaluate the robot's ability to handle humanoid-specific challenges:
   - Turning without losing balance
   - Maintaining straight paths
   - Handling sudden obstacle appearance
   - Recovering from navigation challenges

**Expected Result**: Navigation system that properly considers humanoid dynamics and constraints.

## Validation Checkpoints

### Checkpoint 1: After verifying prerequisites
- Expected output: All necessary packages and assets available
- Troubleshooting: Install missing packages or check Isaac Sim installation

### Checkpoint 2: After launching Isaac Sim
- Expected output: Isaac Sim with humanoid-appropriate environment
- Troubleshooting: If environment doesn't load, check Isaac Sim installation

### Checkpoint 3: After creating humanoid config
- Expected output: Valid YAML configuration for humanoid navigation
- Troubleshooting: Validate YAML syntax and parameter compatibility

### Checkpoint 4: After configuring ROS bridge
- Expected output: Robot communication properly established
- Troubleshooting: Check ROS bridge configuration in Isaac Sim

### Checkpoint 5: After launching Nav2 stack
- Expected output: Nav2 nodes running with humanoid parameters
- Troubleshooting: Verify parameter file path and content

### Checkpoint 6: After setting initial pose
- Expected output: Robot pose correctly initialized in navigation system
- Troubleshooting: Check coordinate frame alignment

### Checkpoint 7: After basic navigation
- Expected output: Successful navigation with humanoid-specific behaviors
- Troubleshooting: Check velocity and acceleration parameters

### Checkpoint 8: After complex obstacle tests
- Expected output: Successful navigation around complex obstacles
- Troubleshooting: Adjust costmap parameters if needed

### Checkpoint 9: After performance validation
- Expected output: Validated navigation performance with humanoid constraints
- Troubleshooting: Fine-tune parameters based on performance

## Expected Outcome

By the end of this exercise, you should have successfully:
- Configured Nav2 with humanoid-specific parameters for bipedal movement
- Executed navigation tasks with appropriate safety margins for humanoid robots
- Demonstrated obstacle avoidance with considerations for balance and stability
- Validated navigation performance with humanoid-specific constraints

This demonstrates the implementation of path planning for bipedal humanoid movement with obstacle avoidance using Nav2.

## Assessment Criteria

- **Success**: Complete all steps, demonstrate successful navigation with humanoid constraints, validate safety and performance
- **Partial Success**: Complete most steps but experience minor issues with navigation or parameter tuning
- **Requires Review**: Unable to complete core navigation implementation or parameter configuration

## Troubleshooting Section

### Issue 1: Nav2 fails to launch with custom configuration
- **Description**: Nav2 nodes crash or fail to start with humanoid config
- **Solution**: Validate YAML syntax, check parameter compatibility with Nav2 version
- **Prevention**: Test with default config first, then modify incrementally

### Issue 2: Robot moves too slowly or cautiously
- **Description**: Navigation speeds are too conservative for practical use
- **Solution**: Adjust velocity limits and safety margins appropriately
- **Prevention**: Consider the balance between safety and efficiency

### Issue 3: Poor obstacle avoidance behavior
- **Description**: Robot collides with obstacles or fails to find paths
- **Solution**: Adjust costmap inflation, resolution, and controller parameters
- **Prevention**: Validate sensor data quality and range

### Issue 4: Navigation failures during complex maneuvers
- **Description**: Robot fails when navigation requires complex movements
- **Solution**: Adjust path planning tolerances and recovery behavior parameters
- **Prevention**: Test simpler navigation tasks before complex ones

### Issue 5: Localization drift during navigation
- **Description**: Robot loses track of position during navigation
- **Solution**: Improve map quality, adjust AMCL parameters, verify sensor quality
- **Prevention**: Validate localization before starting navigation tasks

## Extension Activities (Optional)

1. Implement terrain-specific gait adjustments for different surface types
2. Test navigation with dynamic obstacles that move during navigation
3. Implement coordinated multi-humanoid navigation in the same environment
4. Add semantic navigation capabilities using visual perception for identifying safe walking areas
5. Test navigation with upper-body manipulation tasks while maintaining balance
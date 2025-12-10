# Local Planner: Dynamic Obstacle Avoidance and Path Following

The Local Planner is a critical component of the Navigation2 stack that handles real-time navigation tasks such as dynamic obstacle avoidance and local path following. While the Global Planner computes a path from start to goal, the Local Planner ensures the robot successfully follows this path while avoiding unexpected obstacles and handling real-time motion constraints.

## Overview of Local Path Planning

Local path planning operates in the robot's immediate vicinity, typically within a "local costmap" that updates frequently based on sensor data. The local planner's responsibilities include:

1. **Path Tracking**: Following the global plan while maintaining the robot's stability
2. **Obstacle Avoidance**: Avoiding obstacles not present in the global map
3. **Trajectory Generation**: Creating executable motion commands for the robot
4. **Dynamic Control**: Adjusting motion based on real-time conditions

Unlike the global planner, which operates less frequently and considers the entire environment, the local planner runs at high frequency (typically 10-20 Hz) to respond to dynamic changes in the robot's immediate environment.

## Local Planner Algorithms

### 1. Trajectory Rollout Methods

Trajectory rollout methods generate and evaluate multiple possible future trajectories, selecting the best one based on a cost function.

#### Dynamic Window Approach (DWA)
DWA limits the search space to kinematically feasible velocities based on the robot's physical capabilities and current state.

**Algorithm:**
1. Define a velocity space based on robot limits
2. Generate trajectories for valid velocity samples
3. Evaluate each trajectory using cost functions:
   - Goal distance cost
   - Obstacle clearance cost  
   - Heading alignment cost
4. Select the trajectory with the lowest combined cost

```cpp
// Simplified DWA implementation
Trajectory selectBestTrajectory(RobotState current_state, 
                               Path global_path,
                               Costmap2D local_costmap) {
    double best_cost = std::numeric_limits<double>::max();
    Trajectory best_traj;
    
    // Sample velocities in the dynamic window
    for (double v = v_min; v <= v_max; v += v_resolution) {
        for (double w = w_min; w <= w_max; w += w_resolution) {
            Trajectory traj = generateTrajectory(current_state, v, w);
            
            if (isValidTrajectory(traj, local_costmap)) {
                double cost = calculateTrajectoryCost(traj, global_path);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_traj = traj;
                }
            }
        }
    }
    
    return best_traj;
}
```

#### Timed Elastic Band (TEB)
TEB represents the trajectory as an elastic band of poses that can be optimized considering various constraints and costs.

**Advantages:**
- Produces smooth, time-optimal trajectories
- Handles differential drive and car-like robots well
- Considers dynamic constraints explicitly

### 2. Vector Field Histogram (VFH)

VFH uses a polar histogram to represent the robot's environment and find safe directions to move.

**Process:**
1. Build a histogram of obstacles around the robot
2. Find safe directions that avoid obstacles
3. Select direction toward the goal when possible

### 3. Receding Horizon Approach

A Model Predictive Control (MPC) approach that solves an optimization problem for a short time horizon, then repeats the process as the robot moves.

## Navigation2 Local Planners

Navigation2 provides several local planner plugins:

### 1. DWA Local Planner

The DWA Local Planner is widely used and well-suited for differential and omni-directional robots.

```yaml
# DWA Local Planner configuration
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "nav2_dwb_controller::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_vel_theta: -1.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.05
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.1
      goal_check_type: "odom"
```

### 2. MPPI (Model Predictive Path Integral) Controller

MPPI is a sampling-based control approach that can handle noise and uncertainty better than traditional optimization-based controllers.

```yaml
# MPPI Controller configuration
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 24
      control_frequency: 20.0
      nonholonomic: true
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.2
      stateful: true
      horizon_duration: 1.5
      control_horizon: 6
      cmd_angle_instead_rotvel: false
      v_linear_min: -0.5
      v_linear_max: 0.5
      v_angular_min: -0.6
      v_angular_max: 0.6
```

### 3. TEB (Timed Elastic Band) Local Planner

Provides optimized trajectories with explicit handling of kinematic constraints.

## Local Planner Parameters

### Key Configuration Parameters

1. **Velocity Limits**
   ```yaml
   max_vel_x: 0.5      # Maximum forward velocity (m/s)
   min_vel_x: 0.1      # Minimum forward velocity (m/s)
   max_vel_theta: 1.0  # Maximum angular velocity (rad/s)
   min_vel_theta: -1.0 # Minimum angular velocity (rad/s)
   ```

2. **Acceleration Limits**
   ```yaml
   acc_lim_x: 2.5      # Linear acceleration limit
   acc_lim_theta: 3.2  # Angular acceleration limit
   decel_lim_x: -2.5   # Linear deceleration limit
   ```

3. **Goal Tolerances**
   ```yaml
   xy_goal_tolerance: 0.25   # Position tolerance (m)
   yaw_goal_tolerance: 0.1   # Orientation tolerance (rad)
   ```

4. **Sampling Parameters** (for sampling-based planners)
   ```yaml
   vx_samples: 20      # Velocity samples in x direction
   vtheta_samples: 40  # Angular velocity samples
   ```

5. **Simulation Parameters** (for trajectory evaluation)
   ```yaml
   sim_time: 1.7       # Time to simulate trajectories (s)
   linear_granularity: 0.05  # Step size for path discretization (m)
   ```

### Humanoid-Specific Parameters

When configuring local planners for humanoid robots, additional considerations apply:

1. **Balance Constraints**
   ```yaml
   # More conservative velocity limits for balance
   max_vel_x: 0.3      # Slower for bipedal stability
   max_vel_theta: 0.4  # Slower turning for balance
   ```

2. **Stability Regions**
   ```yaml
   # Larger safety margins for fall prevention
   xy_goal_tolerance: 0.4  # Larger tolerance for stability
   ```

3. **Dynamic Response**
   ```yaml
   # Conservative acceleration for stable walking
   acc_lim_x: 0.5      # Lower acceleration for stable steps
   acc_lim_theta: 0.8  # Lower angular acceleration
   ```

## Local Planning Process

### 1. Costmap Update
The local planner receives updated costmap data at high frequency (typically 5-10 Hz), incorporating:
- Sensor readings from LIDAR, cameras, etc.
- Dynamic obstacles in the environment
- Robot position and orientation updates

### 2. Trajectory Generation
Based on current robot state, the local planner generates potential trajectories:
- Sample velocity combinations within constraints
- Simulate robot motion for each velocity
- Consider kinematic constraints (especially important for humanoid robots)

### 3. Trajectory Evaluation
Each trajectory is evaluated using cost functions:
- **Obstacle Cost**: Penalties for trajectories that might hit obstacles
- **Goal Cost**: Rewards for trajectories that move toward the goal
- **Path Following Cost**: Penalties for deviating from the global path
- **Kinematic Cost**: Penalties for trajectories that violate robot constraints

### 4. Command Generation
The best trajectory is converted to velocity commands for the robot:
- Forward velocity
- Angular velocity (for rotation)
- For humanoid robots, this may involve more complex gait commands

## Integration with Other Navigation Components

### Global Planner Coordination
The local planner must work closely with the global planner:
- Receive updated global path periodically
- Report progress along the global path
- Signal when global replanning is needed (e.g., if path is blocked)

### Costmap Integration
The local planner heavily depends on the local costmap:
- Uses costmap to detect obstacles in the immediate vicinity
- Updates costmap with robot's current position
- Incorporates sensor data to detect dynamic obstacles

```yaml
# Local costmap configuration for humanoid
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6     # Larger window for humanoid safety
      height: 6
      resolution: 0.05  # Fine resolution for precision
      robot_radius: 0.4  # Humanoid-specific size
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0  # Higher for humanoid safety
        inflation_radius: 0.8
```

### Controller Integration
The local planner generates commands that are implemented by lower-level controllers:
- Velocity commands for differential drive robots
- Joint commands for more complex robots
- Gait patterns for humanoid robots

## Humanoid-Specific Adaptations

### 1. Gait Pattern Considerations
Humanoid robots require special consideration for their walking patterns:
- Maintain stable center of mass during movement
- Coordinate leg movements for stable locomotion
- Plan for step-by-step movement rather than continuous motion

### 2. Balance Constraints
The local planner must account for humanoid balance:
- Limit turning rates to prevent falls
- Maintain appropriate velocity for stable walking
- Use larger safety margins in path planning

### 3. Step Planning Integration
For fully humanoid robots, the local planner might need to interface with step planning:
- Generate footstep plans based on local trajectory
- Coordinate with footstep controller
- Handle transitions between different types of terrain

## Challenges in Local Path Planning

### 1. Real-Time Performance
The local planner must operate at high frequency while evaluating many potential trajectories. For humanoid robots, this challenge is amplified by the complexity of bipedal locomotion.

### 2. Dynamic Obstacle Handling
Dealing with moving obstacles in the robot's path, especially when they appear suddenly.

### 3. Local Minima
Scenarios where the local planner cannot find a valid path forward, even though one exists globally.

### 4. Oscillation
When the robot repeatedly moves back and forth without making progress.

## Recovery Behaviors

When the local planner cannot find a valid path, Navigation2 includes recovery behaviors:

### 1. Oscillation Recovery
Detects when the robot is oscillating and tries to break out of the pattern.

### 2. Rotate Recovery
A simple rotation in place to potentially clear a local obstacle.

### 3. Back Up Recovery
Moves robot backward to a more navigable position.

```yaml
# Recovery behavior configuration
bt_navigator:
  ros__parameters:
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node  # Recovery behavior
    - nav2_spin_action_bt_node      # Recovery behavior
    - nav2_wait_action_bt_node
    # ... other plugins
```

## Best Practices

### 1. Parameter Tuning
- Start with conservative parameters and gradually increase
- Test extensively in simulation before real deployment
- Consider the robot's physical limitations

### 2. Sensor Integration
- Ensure adequate and accurate sensor coverage
- Handle sensor noise and limitations appropriately
- Verify sensor data is current and reliable

### 3. Frequency Optimization
- Balance planning frequency with computational load
- Ensure sensor data is fresh when planning
- Consider using different frequencies for different situations

### 4. Safety Considerations
- Implement appropriate safety margins
- Include emergency stop mechanisms
- Monitor robot stability during navigation

## Performance Metrics

Evaluate local planner performance with metrics such as:
- **Execution Success Rate**: Percentage of paths successfully followed
- **Obstacle Avoidance Success**: Rate of successful obstacle avoidance
- **Path Deviation**: How closely the robot follows the planned path
- **Smoothness**: Quality of robot motion (especially important for humanoids)
- **Collision Rate**: Frequency of collisions with obstacles
- **Time to Goal**: Actual time vs. planned time to reach goal

## Future Developments

### 1. AI-Enhanced Local Planning
Integration of machine learning for improved obstacle avoidance and path following.

### 2. Predictive Local Planning
Anticipating dynamic obstacle movements to plan more effective paths.

### 3. Humanoid-Specific Algorithms
More sophisticated algorithms specifically designed for bipedal locomotion.

## Troubleshooting Local Planning

### 1. Local Minima Issues
- **Issue**: Robot gets stuck in local minima, unable to find path around obstacles
- **Description**: Local planner cannot find valid path forward despite global path being achievable
- **Solution**: Implement appropriate recovery behaviors
  - Execute rotating in place to find alternative path
  - Implement backup and strafing motions
  - Increase inflation radius in costmap to avoid getting too close to obstacles
- **Prevention**: Optimize costmap parameters for environment
  - Tune obstacle inflation to avoid close passages
  - Adjust robot footprint to include safety margins
  - Configure appropriate sensor ranges and update rates

### 2. Oscillation Problems
- **Issue**: Robot oscillates back and forth without making progress
- **Description**: Robot repeatedly moves toward obstacles and backs away
- **Solution**: Implement oscillation detection and recovery
  - Add hysteresis to obstacle avoidance behavior
  - Use progress checker to detect lack of forward progress
  - Implement wait or rotate behaviors when oscillation detected
- **Prevention**: Configure velocity and acceleration limits appropriately
  - Use conservative acceleration parameters for smooth motion
  - Implement appropriate look-ahead distances
  - Adjust controller parameters to reduce aggressive corrections

### 3. Trajectory Execution Failures
- **Issue**: Robot unable to follow computed trajectories
- **Description**: Robot commands computed by local planner cannot be executed
- **Solution**: Validate trajectory feasibility
  - Check that trajectories respect robot kinematic constraints
  - Verify that velocity commands are within robot capabilities
  - Implement trajectory smoothing to reduce sharp changes
- **Prevention**: Design trajectories for robot capabilities
  - Ensure computed paths respect velocity and acceleration limits
  - Implement appropriate trajectory validation before execution
  - Tune controller parameters for stable path following

### 4. Dynamic Obstacle Response
- **Issue**: Local planner fails to respond adequately to dynamic obstacles
- **Description**: Robot collides with or becomes stuck behind moving obstacles
- **Solution**: Improve dynamic obstacle handling
  - Ensure costmap updates frequently to capture dynamic obstacles
  - Tune costmap parameters for appropriate obstacle detection
  - Adjust controller look-ahead distance based on obstacle speed
- **Prevention**: Optimize for dynamic environments
  - Configure appropriate sensor fusion for dynamic obstacle detection
  - Implement prediction models for anticipatory planning
  - Tune update frequencies for environment dynamics

### 5. Humanoid-Specific Navigation Issues
- **Issue**: Navigation behaviors unsafe for humanoid balance
- **Description**: Local planner generates commands that compromise humanoid stability
- **Solution**: Implement humanoid-aware navigation
  - Use conservative velocity limits for balance maintenance
  - Implement larger safety margins for fall prevention
  - Adjust turning rates to prevent loss of balance
- **Prevention**: Design humanoid-aware parameter sets
  - Configure velocity and acceleration limits for stability
  - Implement balance-aware trajectory generation
  - Use larger safety margins for humanoid-specific navigation

The local planner is essential for safe, effective navigation, handling the real-time challenges of moving through dynamic environments while following the global path plan. For humanoid robots, the local planner must balance path following with the special requirements of bipedal locomotion and balance.
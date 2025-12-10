# Global Planner: Path Planning from Start to Goal

The Global Planner is a critical component of the Navigation2 stack that computes a path from the robot's current location to a specified goal. This section covers the theory, implementation, and configuration of global path planning for robotics applications, with special attention to humanoid robot constraints.

## Overview of Global Path Planning

Global path planning is the process of finding an optimal or near-optimal path from a start point to a goal point in a known or partially known environment. The global planner takes into account static obstacles, map information, and other environmental constraints to compute a path that the robot can follow.

### Key Responsibilities

The global planner is responsible for:

1. **Path Computation**: Finding a valid path from start to goal that minimizes a specific cost function
2. **Obstacle Avoidance**: Ensuring the computed path doesn't intersect with known obstacles
3. **Optimality**: Creating paths that are optimal with respect to certain criteria (distance, time, safety)
4. **Topological Considerations**: Handling connectivity in the environment and avoiding topological dead-ends

### Cost Function Considerations

The global planner uses a cost function to evaluate and compare different potential paths. Common cost factors include:

- **Path Length**: Minimizing the total distance traveled
- **Safety Margin**: Maintaining safe distances from obstacles
- **Traversability**: Favoring easier-to-navigate terrain
- **Energy Efficiency**: Minimizing power consumption
- **Time**: Reducing travel time to the goal

## Global Planner Algorithms

### 1. Dijkstra's Algorithm

Dijkstra's algorithm systematically explores all possible paths from the start location, guaranteeing the shortest path. It works well in grid-based environments but can be computationally expensive for large maps.

**Pros:**
- Guaranteed optimal solution
- Works with any cost function
- Simple to implement

**Cons:**
- Expensive computational complexity
- Explores many unnecessary nodes in large environments

### 2. A* (A-Star) Algorithm

A* improves upon Dijkstra's algorithm by using a heuristic function to guide the search toward the goal, making it more efficient for goal-directed pathfinding.

**Pros:**
- More efficient than Dijkstra's
- Still guarantees optimal solution
- Works well in practice

**Cons:**
- Heuristic must be admissible (never overestimate)
- Performance depends on heuristic quality

### 3. D* (Dynamic A*) Algorithm

D* is designed for dynamic environments where obstacles may appear or move after the initial path is computed.

**Pros:**
- Handles dynamic obstacles
- Repairs path incrementally without full replanning
- Efficient for changing environments

**Cons:**
- Complex implementation
- Higher initial computational cost

### 4. Theta* Algorithm

Theta* allows for any-angle paths, not constrained to grid edges, resulting in more natural-looking paths.

**Pros:**
- Produces more direct paths
- Better than grid-constrained algorithms
- Maintains optimality properties

**Cons:**
- Line-of-sight checks can be expensive
- Complexity increases with map size

### 5. Probabilistic Roadmaps (PRM)

PRM pre-computes a roadmap of possible paths in the environment, making query-time path planning very fast.

**Pros:**
- Fast query time
- Good for static environments
- Handles complex kinematics

**Cons:**
- Requires pre-processing
- Not ideal for dynamic environments
- Memory-intensive

## Navigation2 Global Planners

Navigation2 implements several global planner plugins that can be dynamically loaded:

### 1. Navfn

Navfn (Navigation Function) is based on Dijkstra's algorithm and works on grid maps. It computes a navigation function over the entire map, representing the shortest distance to the goal from each cell.

```yaml
# Example Navfn configuration
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### 2. Generic Gradient Path Planner

A more recent implementation that allows for custom gradient functions and path generation techniques.

### 3. CARL (Coverage and RL Navigation) Planner

Designed for exploration and coverage tasks, not just point-to-point navigation.

### 4. Behavior Tree Integration

Modern Navigation2 uses Behavior Trees that can intelligently switch between different planning algorithms based on the situation:

```xml
<!-- Example of BT-based planning -->
<BehaviorTree>
  <PipelineSequence>
    <RecoveryNode number_of_retries="3">
      <ComputePathToPose goal_updater_node="goal_updater"/>
      <ReactiveFallback>
        <IsGoalReached goal_checker_node="goal_checker"/>
        <IsPathValid path_checker_node="path_checker"/>
      </ReactiveFallback>
    </RecoveryNode>
  </PipelineSequence>
</BehaviorTree>
```

## Configuration and Parameters

### Critical Parameters

1. **Tolerance**: How close the robot needs to get to the goal to be considered successful
   ```yaml
   tolerance: 0.5  # meters
   ```

2. **Use A* vs Dijkstra**: Whether to use the greedy A* algorithm or systematic Dijkstra
   ```yaml
   use_astar: true
   ```

3. **Allow Unknown Space**: Whether to plan through unknown areas of the map
   ```yaml
   allow_unknown: true
   ```

4. **Planner Frequency**: How often to replan when the path needs updating
   ```yaml
   planner_frequency: 0.5  # Hz
   ```

### Humanoid-Specific Parameters

When configuring global planners for humanoid robots, special considerations apply:

1. **Safety Distance**: Humanoid robots typically need wider safety margins due to balance considerations
   ```yaml
   # Inflation layer for costmap
   inflation_radius: 0.8  # Larger than typical wheeled robots
   ```

2. **Path Smoothness**: Humanoid robots benefit from smoother paths to maintain balance
   ```yaml
   # Prefer algorithms that produce smooth paths
   tolerance: 0.7  # Slightly larger for smoother path following
   ```

3. **Kinematic Constraints**: Account for humanoid kinematic limitations
   ```yaml
   # Custom parameters for humanoid kinematics
   angular_penality: 1.5  # Higher cost for turning
   ```

## Global Path Planning Process

### 1. Map Representation

The global planner operates on a map representation of the environment, typically:
- **Occupancy Grid**: 2D grid with occupied/free/unknown state for each cell
- **Topological Map**: Graph-based representation with locations as nodes
- **Semantic Map**: Includes object-level information about the environment

### 2. Path Validation

After computing a path, the global planner (or associated components) validate it:
- Check for collisions with known obstacles
- Verify that path segments are within the robot's kinematic capabilities
- Ensure the path is traversable by the specific robot type

### 3. Path Smoothing

Computed paths often require smoothing to:
- Remove redundant waypoints
- Create smoother curves for better robot motion
- Reduce the number of path points to follow

```cpp
// Example path smoothing operation
std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& original_path) {
    
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
    
    // Add first and last points
    if (!original_path.empty()) {
        smoothed_path.push_back(original_path.front());
        
        // Apply smoothing algorithm (e.g., cubic spline interpolation)
        for (size_t i = 1; i < original_path.size() - 1; ++i) {
            // Humanoid-specific smoothing for balance
            geometry_msgs::msg::PoseStamped smooth_pose = original_path[i];
            // Apply smoothing logic here
            smoothed_path.push_back(smooth_pose);
        }
        
        if (original_path.size() > 1) {
            smoothed_path.push_back(original_path.back());
        }
    }
    
    return smoothed_path;
}
```

## Integration with Other Navigation Components

### Costmap Integration

The global planner heavily relies on the global costmap to:
- Identify obstacles and free space
- Apply inflation to create safety margins
- Consider terrain costs and preferences

```yaml
# Costmap integration parameters
global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.4  # Humanoid-specific radius
      resolution: 0.05   # Fine resolution for humanoid
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      inflation_layer:
        cost_scaling_factor: 5.0  # High factor for humanoid safety
        inflation_radius: 0.8
```

### Local Planner Coordination

The global planner must coordinate with the local planner:
- Providing a feasible path to track
- Updating the path when necessary
- Handling cases where the local planner cannot follow the global path

## Challenges in Global Path Planning

### 1. Computational Complexity
Large environments require significant computational resources. Solutions include:
- Hierarchical planning approaches
- Precomputed path caches
- Efficient data structures (e.g., octrees)

### 2. Dynamic Environments
Changing obstacles require replanning. Approaches include:
- Incremental planning algorithms
- Hybrid approaches combining global and local planning
- Predictive models of dynamic obstacles

### 3. Kinematic Constraints
Humanoid robots have complex kinematic constraints that the path must respect:
- Turning radius limitations
- Balance constraints during movement
- Step height restrictions

### 4. Optimality vs. Efficiency Trade-offs
Finding the optimal path often conflicts with computational efficiency. Navigation2 addresses this with:
- Timeout mechanisms
- Any-time algorithms
- Multi-resolution approaches

## Humanoid-Specific Adaptations

### 1. Balance-Aware Path Planning

Humanoid robots must maintain balance while following paths:
- Prefer paths with minimal sharp turns
- Avoid paths with obstacles requiring complex maneuvering
- Consider the robot's center of mass during planning

### 2. Footstep Planning Integration

The global path may need to be converted to footstep plans:
- Path points represent foot positions
- Consider step size and spacing constraints
- Plan for stable stepping sequences

### 3. Stability Regions

Consider the humanoid's stability requirements:
- Plan paths through stable terrain
- Maintain center of mass within support polygon
- Plan for recovery steps if needed

## Best Practices

### 1. Parameter Tuning
- Start with default parameters and tune based on robot characteristics
- Consider the robot's size, speed, and kinematics
- Test extensively in simulation before real-world deployment

### 2. Planning Frequency
- Balance between fresh path information and computational load
- Adjust based on environment dynamics and robot speed
- Consider using different frequencies for different situations

### 3. Error Handling
- Implement proper timeout mechanisms
- Provide fallback strategies when planning fails
- Include recovery behaviors for exceptional cases

### 4. Validation
- Verify path feasibility before sending to controller
- Check that paths account for robot dimensions
- Test with edge cases and challenging scenarios

## Performance Metrics

Evaluate global planner performance with metrics such as:
- **Path Optimality**: How close the path is to the theoretical optimum
- **Computation Time**: How long it takes to generate a path
- **Success Rate**: Percentage of planning requests that succeed
- **Path Smoothness**: Quality of the generated path for robot execution
- **Safety**: How well the path maintains safe distances from obstacles

## Future Developments

### 1. Learning-Based Planners
Integration of machine learning techniques for improved path planning in complex environments.

### 2. Multi-Robot Coordination
Advanced planners that consider multiple robots navigating simultaneously.

### 3. Predictive Planning
Planners that predict future changes in the environment to plan more effectively.

The global planner forms the foundation of Navigation2's path planning capabilities, providing the strategic route that enables robots to navigate from start to goal while avoiding obstacles. For humanoid robots, careful consideration of balance, kinematics, and safety is essential for effective global planning.
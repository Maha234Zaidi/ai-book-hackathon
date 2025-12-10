# ROS 2 Terminology and Naming Conventions

## Core ROS 2 Concepts

### Nodes
- **Definition**: A process that performs computation in a ROS 2 system
- **Naming Convention**: Use snake_case with descriptive names
  - Examples: `camera_driver`, `path_planner`, `velocity_controller`
- **Best Practice**: Name should reflect the node's primary function

### Topics
- **Definition**: Named buses over which nodes exchange messages
- **Naming Convention**: Use forward slash hierarchy with descriptive names
  - Examples: `/sensor_data`, `/cmd_vel`, `/robot_state`
- **Best Practice**: Use forward slashes to create logical hierarchies (e.g., `/arm/joint_states`)

### Services
- **Definition**: Synchronous request/response communication
- **Naming Convention**: Use action-oriented names in snake_case
  - Examples: `get_path`, `add_two_ints`, `set_parameters`
- **Best Practice**: Name should describe the action performed

### Actions
- **Definition**: Asynchronous goal-oriented communication
- **Naming Convention**: Use action-oriented names in snake_case
  - Examples: `navigate_to_pose`, `follow_joint_trajectory`
- **Best Practice**: Name should describe the goal being achieved

### Parameters
- **Definition**: Configuration values that can be set at runtime
- **Naming Convention**: Use descriptive names, often hierarchical
  - Examples: `robot_name`, `update_frequency`, `safety.timeout`
- **Best Practice**: Use consistent prefixes for related parameters

## ROS 2 Message Types

### Standard Types
- `std_msgs`: Basic data types (String, Int64, Float64, etc.)
- `geometry_msgs`: Geometric primitives (Point, Pose, Twist, etc.)
- `sensor_msgs`: Sensor data (LaserScan, Image, JointState, etc.)
- `nav_msgs`: Navigation-specific messages (Odometry, Path, etc.)

## Python (rclpy) Conventions

### Node Class Names
- Use PascalCase with descriptive names
- Example: `class CameraDriverNode(Node):`

### Function and Method Names
- Use snake_case consistently
- Examples: `publish_scan()`, `process_laser_data()`

### Variable Names
- Use snake_case for local variables
- Prefix private attributes with underscore: `_internal_state`

## URDF Conventions

### Link Names
- Use descriptive names in snake_case
- Examples: `base_link`, `wheel_front_left`, `camera_optical_frame`

### Joint Names
- Use descriptive names indicating parent and child links
- Examples: `base_to_wheel`, `torso_to_head`, `shoulder_pitch_joint`

### Frame Conventions
- `base_link`: Main reference frame of the robot
- `<sensor>_frame`: Reference frame for sensor data
- `<actuator>_frame`: Reference frame for actuator commands

## Code Documentation

### Comments
- Use clear, concise comments explaining the 'why' not just the 'what'
- Comment complex algorithms or non-obvious code decisions

### Docstrings
- Use docstrings for all public methods and classes
- Follow Google Python style for docstrings

## Error Handling

### Logging
- Use appropriate log levels:
  - DEBUG: Detailed diagnostic information
  - INFO: General information about program execution
  - WARN: Indication of potential issues
  - ERROR: Error events that allow the program to continue
  - FATAL: Very severe error events that cause premature termination

### Exception Handling
- Catch specific exceptions rather than using bare `except:`
- Always log exceptions with sufficient detail for debugging
- Implement appropriate recovery mechanisms when possible
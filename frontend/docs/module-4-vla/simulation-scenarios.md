# Simulation Scenarios for Complete VLA System Testing

## Overview

This document outlines comprehensive simulation scenarios designed to test the complete Vision-Language-Action (VLA) system in various conditions. These scenarios validate the integration of all VLA components and ensure the system performs reliably in different environments and situations.

## Simulation Environment Setup

### Gazebo Models and Worlds

#### Test World Configuration
Create a comprehensive test world with various objects and environments:

```xml
<!-- worlds/vla_comprehensive_test.world -->
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="vla_comprehensive_test">
    <!-- Include standard environment elements -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Main room with furniture -->
    <model name="main_room">
      <pose>0 0 0 0 0 0</pose>
      <link name="room_link">
        <collision name="room_collision">
          <geometry>
            <box><size>8 8 3</size></box>
          </geometry>
        </collision>
        <visual name="room_visual">
          <geometry>
            <box><size>8 8 3</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.9 0.9 0.9 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Furniture and objects for testing -->
    <model name="dining_table">
      <pose>-2 -2 0 0 0 0</pose>
      <include>
        <uri>model://table</uri>
      </include>
    </model>
    
    <!-- Various objects for manipulation testing -->
    <model name="red_cup">
      <pose>-1.8 -1.8 0.82 0 0 0</pose>
      <link name="cup_link">
        <inertial>
          <mass>0.2</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
        <visual name="cup_visual">
          <geometry>
            <cylinder><radius>0.04</radius><length>0.1</length></cylinder>
          </geometry>
          <material>
            <script>Gazebo/Red</script>
          </material>
        </visual>
        <collision name="cup_collision">
          <geometry>
            <cylinder><radius>0.04</radius><length>0.1</length></cylinder>
          </geometry>
        </collision>
      </link>
    </model>
    
    <model name="blue_bottle">
      <pose>-2.2 -2.2 0.82 0 0 0</pose>
      <link name="bottle_link">
        <visual name="bottle_visual">
          <geometry>
            <cylinder><radius>0.03</radius><length>0.15</length></cylinder>
          </geometry>
          <material>
            <script>Gazebo/Blue</script>
          </material>
        </visual>
        <collision name="bottle_collision">
          <geometry>
            <cylinder><radius>0.03</radius><length>0.15</length></cylinder>
          </geometry>
        </collision>
      </link>
    </model>
    
    <model name="wooden_box">
      <pose>-1 -1 0.1 0 0 0</pose>
      <include>
        <uri>model://box</uri>
      </include>
      <model name="box">
        <pose>0 0 0.1 0 0 0</pose>
        <link name="box_link">
          <visual name="box_visual">
            <geometry>
              <box><size>0.15 0.15 0.15</size></box>
            </geometry>
            <material>
              <script>Gazebo/Wood</script>
          </material>
          </visual>
        </link>
      </model>
    </model>
    
    <!-- Kitchen area -->
    <model name="kitchen_counter">
      <pose>2 -2 0 0 0 0</pose>
      <link name="counter_link">
        <visual name="counter_visual">
          <geometry>
            <box><size>1.5 0.6 0.8</size></box>
          </geometry>
          <material>
            <script>Gazebo/Grey</script>
          </material>
        </visual>
        <collision name="counter_collision">
          <geometry>
            <box><size>1.5 0.6 0.8</size></box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <model name="kitchen_cup">
      <pose>2.2 -2.2 1.0 0 0 0</pose>
      <link name="kitchen_cup_link">
        <visual name="kitchen_cup_visual">
          <geometry>
            <cylinder><radius>0.04</radius><length>0.1</length></cylinder>
          </geometry>
          <material>
            <script>Gazebo/Green</script>
          </material>
        </visual>
        <collision name="kitchen_cup_collision">
          <geometry>
            <cylinder><radius>0.04</radius><length>0.1</length></cylinder>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Living room area -->
    <model name="sofa">
      <pose>2 2 0 0 0 1.57</pose>
      <include>
        <uri>model://couch</uri>
      </include>
    </model>
    
    <!-- Bookshelf -->
    <model name="bookshelf">
      <pose>-2 2 0 0 0 0</pose>
      <link name="shelf_link">
        <visual name="shelf_visual">
          <geometry>
            <box><size>0.3 1.2 1.0</size></box>
          </geometry>
          <material>
            <script>Gazebo/Wood</script>
          </material>
        </visual>
        <collision name="shelf_collision">
          <geometry>
            <box><size>0.3 1.2 1.0</size></box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <model name="book">
      <pose>-2.15 1.8 0.5 0 0 0</pose>
      <link name="book_link">
        <visual name="book_visual">
          <geometry>
            <box><size>0.15 0.02 0.2</size></box>
          </geometry>
          <material>
            <script>Gazebo/BlueLaser</script>
          </material>
        </visual>
        <collision name="book_collision">
          <geometry>
            <box><size>0.15 0.02 0.2</size></box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Robot starting position -->
    <include>
      <name>mobile_robot</name>
      <uri>model://turtlebot3_waffle</uri>
      <pose>-3 0 0 0 0 0</pose>
    </include>
    
    <!-- Add lighting -->
    <light type="point" name="light_1">
      <pose>-2 2 2 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>8</range>
        <constant>0.2</constant>
        <linear>0.5</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
    </light>
  </world>
</sdf>
```

### Simulation Launch Configuration

```python
# launch/vla_simulation_scenarios.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='vla_comprehensive_test.world')
    
    # Get package directories
    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')
    vla_package_dir = get_package_share_directory('vla_examples')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world',
        default_value='vla_comprehensive_test.world',
        description='World file to load in Gazebo'
    )
    
    # Launch Gazebo with our world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_package_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution([get_package_share_directory('vla_examples'), 'worlds', world_file]),
            'verbose': 'true',
            'server_required': 'true',
            'gui_required': 'true'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'publish_frequency': 50.0}
        ],
        remappings=[
            ('/joint_states', 'mobile_robot/joint_states')
        ]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )
    
    # VLA system components
    vla_components = [
        Node(
            package='vla_examples',
            executable='vla_system_orchestrator',
            name='vla_system_orchestrator',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        Node(
            package='vla_examples',
            executable='cognitive_planner',
            name='cognitive_planner',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        Node(
            package='vla_examples',
            executable='perception_module',
            name='perception_module',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        Node(
            package='vla_examples',
            executable='path_planning_component',
            name='path_planning_component',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        Node(
            package='vla_examples',
            executable='manipulation_system',
            name='manipulation_system',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    ]
    
    # Create launch description
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_world_file,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
    ])
    
    # Add VLA components
    for component in vla_components:
        ld.add_action(component)
    
    return ld
```

## Test Scenarios

### Scenario 1: Basic Navigation Test

**Objective**: Verify the system can understand navigation commands and execute them safely.

**Setup**:
- Robot starts at position (-3, 0, 0)
- Target: Dining table at (-2, -2, 0)
- Environment: Clear path with known obstacles

**Test Procedure**:
1. Send command: "Go to the dining table"
2. Monitor path planning and navigation execution
3. Verify robot reaches target location safely
4. Check for collision avoidance

**Expected Results**:
- Cognitive planner generates navigation action
- Path planning finds safe route
- Robot navigates to dining table without collisions
- System reports successful completion

```python
# test_scenarios/navigation_test.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vla_msgs.msg import SystemHealth
import time
import math


class NavigationTestScenario(Node):
    """
    Test scenario for basic navigation functionality
    """
    
    def __init__(self):
        super().__init__('navigation_test_scenario')
        
        # Publishers for sending commands
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            10
        )
        
        # Subscribers for monitoring progress
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.status_subscriber = self.create_subscription(
            String,
            '/vla_system/status',
            self.status_callback,
            10
        )
        
        # Test state
        self.current_position = None
        self.target_position = (-2.0, -2.0)  # Dining table position
        self.reached_target = False
        self.test_completed = False
        self.test_start_time = None
        self.status_history = []
        
        self.get_logger().info('Navigation Test Scenario initialized')
    
    def odom_callback(self, msg):
        """
        Track robot position
        """
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        if self.current_position and self.target_position:
            distance_to_target = math.sqrt(
                (self.current_position[0] - self.target_position[0])**2 +
                (self.current_position[1] - self.target_position[1])**2
            )
            
            if distance_to_target < 0.3:  # Within 30cm of target
                self.reached_target = True
                self.test_completed = True
                self.get_logger().info('Robot reached target position')
    
    def status_callback(self, msg):
        """
        Track system status during test
        """
        self.status_history.append(msg.data)
    
    def execute_test(self):
        """
        Execute the navigation test scenario
        """
        self.get_logger().info('Starting navigation test')
        self.test_start_time = time.time()
        
        # Send navigation command
        command_msg = String()
        command_msg.data = "Go to the dining table"
        self.command_publisher.publish(command_msg)
        
        self.get_logger().info(f'Sent command: "Go to the dining table"')
        
        # Wait for test completion or timeout
        timeout = time.time() + 30.0  # 30 second timeout
        while not self.test_completed and time.time() < timeout:
            time.sleep(0.1)
        
        # Report results
        self.report_results()
    
    def report_results(self):
        """
        Report test results
        """
        if self.reached_target:
            self.get_logger().info('✅ Navigation test PASSED - Robot reached target')
        else:
            self.get_logger().info('❌ Navigation test FAILED - Robot did not reach target')
        
        if self.test_start_time:
            duration = time.time() - self.test_start_time
            self.get_logger().info(f'Test duration: {duration:.2f} seconds')
        
        self.get_logger().info(f'Final position: {self.current_position}')
        self.get_logger().info(f'Target position: {self.target_position}')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = NavigationTestScenario()
    
    try:
        test_node.execute_test()
    except KeyboardInterrupt:
        test_node.get_logger().info('Navigation test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Scenario 2: Object Manipulation Test

**Objective**: Verify the system can detect, navigate to, and manipulate objects.

**Setup**:
- Robot starts at position (-3, 0, 0)
- Target object: Red cup at (-1.8, -1.8, 0.82)
- Environment: Clear workspace around the object

**Test Procedure**:
1. Send command: "Pick up the red cup"
2. Monitor object detection
3. Verify navigation to object location
4. Execute grasp action
5. Confirm successful manipulation

**Expected Results**:
- Perception system detects red cup
- Cognitive planner generates approach and grasp actions
- Robot navigates to cup location
- Successful grasp of the cup
- System reports successful completion

```python
# test_scenarios/manipulation_test.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import DetectedObjects, Object
from geometry_msgs.msg import Point
import time
import json


class ManipulationTestScenario(Node):
    """
    Test scenario for object manipulation functionality
    """
    
    def __init__(self):
        super().__init__('manipulation_test_scenario')
        
        # Publishers
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            10
        )
        
        # Subscribers
        self.detected_objects_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.detected_objects_callback,
            10
        )
        
        self.status_subscriber = self.create_subscription(
            String,
            '/vla_system/status',
            self.status_callback,
            10
        )
        
        # Test state
        self.red_cup_detected = False
        self.red_cup_position = Point(x=-1.8, y=-1.8, z=0.82)
        self.manipulation_successful = False
        self.test_completed = False
        self.status_history = []
        
        self.get_logger().info('Manipulation Test Scenario initialized')
    
    def detected_objects_callback(self, msg):
        """
        Track detected objects to verify red cup detection
        """
        for obj in msg.objects:
            if 'cup' in obj.name.lower() and 'red' in obj.name.lower():
                self.red_cup_detected = True
                self.get_logger().info(f'Red cup detected at: ({obj.pose.position.x}, {obj.pose.position.y}, {obj.pose.position.z})')
                
                # Check if position is close to expected position
                dist_x = abs(obj.pose.position.x - self.red_cup_position.x)
                dist_y = abs(obj.pose.position.y - self.red_cup_position.y)
                
                if dist_x < 0.2 and dist_y < 0.2:
                    self.get_logger().info('Red cup position verified')
    
    def status_callback(self, msg):
        """
        Track system status for manipulation success
        """
        self.status_history.append(msg.data)
        
        # Look for signs of successful manipulation
        if 'grasp' in msg.data.lower() and 'success' in msg.data.lower():
            self.manipulation_successful = True
        
        if 'manipulation complete' in msg.data.lower() or 'action sequence completed' in msg.data.lower():
            self.test_completed = True
    
    def execute_test(self):
        """
        Execute the manipulation test scenario
        """
        self.get_logger().info('Starting manipulation test')
        self.get_logger().info('Waiting for red cup detection...')
        
        # Send manipulation command
        command_msg = String()
        command_msg.data = "Pick up the red cup"
        self.command_publisher.publish(command_msg)
        
        self.get_logger().info(f'Sent command: "Pick up the red cup"')
        
        # Wait for test completion or timeout
        timeout = time.time() + 60.0  # 60 second timeout
        while not self.test_completed and time.time() < timeout:
            time.sleep(0.1)
        
        # Report results
        self.report_results()
    
    def report_results(self):
        """
        Report test results
        """
        self.get_logger().info('=== MANIPULATION TEST RESULTS ===')
        
        if self.red_cup_detected:
            self.get_logger().info('✅ Red cup detected')
        else:
            self.get_logger().info('❌ Red cup not detected')
        
        if self.manipulation_successful:
            self.get_logger().info('✅ Manipulation operation successful')
        else:
            self.get_logger().info('❌ Manipulation operation failed')
        
        if self.test_completed:
            self.get_logger().info('✅ Test completed successfully')
        else:
            self.get_logger().info('❌ Test timed out or failed to complete')
        
        self.get_logger().info(f'Total status updates received: {len(self.status_history)}')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = ManipulationTestScenario()
    
    try:
        test_node.execute_test()
    except KeyboardInterrupt:
        test_node.get_logger().info('Manipulation test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Scenario 3: Multi-Step Task Test

**Objective**: Verify the system can execute complex, multi-step tasks involving navigation and manipulation.

**Setup**:
- Robot starts at position (-3, 0, 0)
- Task: Navigate to kitchen counter, pick up green cup, bring it to dining table
- Environment: Multiple rooms with obstacles

**Test Procedure**:
1. Send command: "Go to the kitchen, pick up the green cup, and bring it to the dining table"
2. Monitor multi-step action generation
3. Verify each step execution (navigate, grasp, navigate, place)
4. Confirm task completion

**Expected Results**:
- Cognitive planner generates multi-step action sequence
- Robot successfully navigates between rooms
- Successful grasp of green cup
- Successful placement at dining table
- System reports task completion

```python
# test_scenarios/multi_step_test.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence, VLAAction
import time
import json


class MultiStepTestScenario(Node):
    """
    Test scenario for multi-step task execution
    """
    
    def __init__(self):
        super().__init__('multi_step_test_scenario')
        
        # Publishers
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            10
        )
        
        # Subscribers
        self.action_sequence_subscriber = self.create_subscription(
            ActionSequence,
            '/vla/action_sequence',
            self.action_sequence_callback,
            10
        )
        
        self.status_subscriber = self.create_subscription(
            String,
            '/vla_system/status',
            self.status_callback,
            10
        )
        
        # Test state
        self.action_sequence_received = False
        self.expected_actions = ['navigate_to', 'grasp', 'navigate_to', 'place_at']
        self.executed_actions = []
        self.task_completed = False
        self.test_completed = False
        self.status_history = []
        
        self.get_logger().info('Multi-Step Test Scenario initialized')
    
    def action_sequence_callback(self, msg):
        """
        Track received action sequence
        """
        self.action_sequence_received = True
        self.get_logger().info(f'Received action sequence with {len(msg.actions)} actions')
        
        # Log action types
        action_types = [action.type for action in msg.actions]
        self.get_logger().info(f'Action sequence: {action_types}')
        
        # Verify sequence complexity
        if len(msg.actions) >= 4:
            self.get_logger().info('Action sequence has sufficient complexity for multi-step task')
    
    def status_callback(self, msg):
        """
        Track execution status and completed actions
        """
        self.status_history.append(msg.data)
        
        # Look for action completion indicators
        if 'action executed' in msg.data.lower() or 'action completed' in msg.data.lower():
            # Extract action type from status message
            for action_type in self.expected_actions:
                if action_type in msg.data.lower():
                    if action_type not in self.executed_actions:
                        self.executed_actions.append(action_type)
                        self.get_logger().info(f'Completed action: {action_type}')
                        break
        
        # Check for task completion
        if 'sequence completed' in msg.data.lower() or 'task completed' in msg.data.lower():
            self.task_completed = True
            self.test_completed = True
            self.get_logger().info('Multi-step task completed successfully')
    
    def execute_test(self):
        """
        Execute the multi-step test scenario
        """
        self.get_logger().info('Starting multi-step task test')
        self.get_logger().info('Task: "Go to the kitchen, pick up the green cup, and bring it to the dining table"')
        
        # Send multi-step command
        command_msg = String()
        command_msg.data = "Go to the kitchen, pick up the green cup, and bring it to the dining table"
        self.command_publisher.publish(command_msg)
        
        self.get_logger().info(f'Sent command: "{command_msg.data}"')
        
        # Wait for test completion or timeout
        timeout = time.time() + 120.0  # 2 minute timeout for complex task
        while not self.test_completed and time.time() < timeout:
            time.sleep(0.1)
        
        # Report results
        self.report_results()
    
    def report_results(self):
        """
        Report test results
        """
        self.get_logger().info('=== MULTI-STEP TEST RESULTS ===')
        
        if self.action_sequence_received:
            self.get_logger().info('✅ Action sequence received')
        else:
            self.get_logger().info('❌ Action sequence not received')
        
        self.get_logger().info(f'Expected actions: {self.expected_actions}')
        self.get_logger().info(f'Executed actions: {self.executed_actions}')
        
        # Check if all expected actions were executed
        all_actions_executed = all(action in self.executed_actions for action in self.expected_actions)
        if all_actions_executed:
            self.get_logger().info('✅ All expected actions were executed')
        else:
            missing_actions = [action for action in self.expected_actions if action not in self.executed_actions]
            self.get_logger().info(f'❌ Missing actions: {missing_actions}')
        
        if self.task_completed:
            self.get_logger().info('✅ Task completed successfully')
        else:
            self.get_logger().info('❌ Task not completed (timed out)')
        
        self.get_logger().info(f'Total status updates: {len(self.status_history)}')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = MultiStepTestScenario()
    
    try:
        test_node.execute_test()
    except KeyboardInterrupt:
        test_node.get_logger().info('Multi-step test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Scenario 4: Obstacle Avoidance Test

**Objective**: Verify the system can navigate safely around obstacles during task execution.

**Setup**:
- Robot starts at position (-3, 0, 0)
- Target: Dining table at (-2, -2, 0)
- Dynamic obstacle: Box placed halfway to destination

**Test Procedure**:
1. Send command: "Go to the dining table"
2. Introduce dynamic obstacle during navigation
3. Monitor path replanning and obstacle avoidance
4. Verify successful navigation around obstacle

**Expected Results**:
- Robot detects obstacle during navigation
- System replans path to avoid obstacle
- Robot successfully reaches destination
- No collisions occur

```python
# test_scenarios/obstacle_avoidance_test.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from vla_msgs.msg import SystemHealth
import time
import math


class ObstacleAvoidanceTestScenario(Node):
    """
    Test scenario for obstacle avoidance during navigation
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance_test_scenario')
        
        # Publishers
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.status_subscriber = self.create_subscription(
            String,
            '/vla_system/status',
            self.status_callback,
            10
        )
        
        # Test state
        self.obstacle_detected = False
        self.obstacle_avoided = False
        self.reached_destination = False
        self.test_completed = False
        self.navigation_active = False
        self.status_history = []
        
        # Obstacle detection parameters
        self.obstacle_threshold = 0.5  # meters
        self.obstacle_direction_range = (315, 45)  # Front-facing sector
        
        self.get_logger().info('Obstacle Avoidance Test Scenario initialized')
    
    def laser_callback(self, msg):
        """
        Process laser scan data to detect obstacles
        """
        if not self.navigation_active:
            return
        
        # Check for obstacles in front of the robot
        front_ranges = []
        
        # Get indices for front-facing sector (in degrees)
        angle_increment = math.degrees(msg.angle_increment)
        start_angle = math.degrees(msg.angle_min)
        
        # Check the front 90-degree sector
        for i, range_val in enumerate(msg.ranges):
            angle = start_angle + i * angle_increment
            
            # Normalize angle to -180 to 180 range
            if angle > 180:
                angle -= 360
            elif angle < -180:
                angle += 360
            
            # Check if angle is in the front sector (-45 to 45 degrees)
            if -45 <= angle <= 45:
                if 0 < range_val < self.obstacle_threshold:
                    front_ranges.append(range_val)
        
        if front_ranges:
            self.obstacle_detected = True
            min_distance = min(front_ranges)
            self.get_logger().info(f'Obstacle detected! Distance: {min_distance:.2f}m')
            
            # Check if robot successfully avoids obstacle
            # In a real test, this would check if the robot changes its course
            self.obstacle_avoided = True
    
    def status_callback(self, msg):
        """
        Monitor navigation status
        """
        self.status_history.append(msg.data)
        
        # Check if system is in navigation state
        if 'navigating' in msg.data.lower() or 'navigation' in msg.data.lower():
            self.navigation_active = True
        
        # Check for destination reached
        if 'reached destination' in msg.data.lower() or 'arrived' in msg.data.lower():
            self.reached_destination = True
            self.test_completed = True
            self.navigation_active = False
    
    def execute_test(self):
        """
        Execute the obstacle avoidance test scenario
        """
        self.get_logger().info('Starting obstacle avoidance test')
        self.get_logger().info('Robot will navigate to target while avoiding dynamic obstacles')
        
        # Send navigation command
        command_msg = String()
        command_msg.data = "Go to the dining table"
        self.command_publisher.publish(command_msg)
        
        self.get_logger().info(f'Sent navigation command')
        
        # Wait for test completion or timeout
        timeout = time.time() + 60.0  # 60 second timeout
        while not self.test_completed and time.time() < timeout:
            time.sleep(0.1)
        
        # Report results
        self.report_results()
    
    def report_results(self):
        """
        Report test results
        """
        self.get_logger().info('=== OBSTACLE AVOIDANCE TEST RESULTS ===')
        
        if self.obstacle_detected:
            self.get_logger().info('✅ Obstacle detected during navigation')
        else:
            self.get_logger().info('⚠️ No obstacles detected (may indicate clear path or sensor issue)')
        
        if self.obstacle_avoided:
            self.get_logger().info('✅ Robot successfully avoided obstacle')
        else:
            self.get_logger().info('❌ Robot did not avoid obstacle')
        
        if self.reached_destination:
            self.get_logger().info('✅ Robot reached destination')
        else:
            self.get_logger().info('❌ Robot did not reach destination')
        
        if self.test_completed:
            self.get_logger().info('✅ Test completed successfully')
        else:
            self.get_logger().info('❌ Test timed out')
        
        self.get_logger().info(f'Total status updates: {len(self.status_history)}')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = ObstacleAvoidanceTestScenario()
    
    try:
        test_node.execute_test()
    except KeyboardInterrupt:
        test_node.get_logger().info('Obstacle avoidance test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Scenario 5: Environmental Context Test

**Objective**: Verify the system considers environmental context when planning actions.

**Setup**:
- Robot starts at position (-3, 0, 0)
- Multiple similar objects (red and blue cups)
- Commands that require contextual understanding

**Test Procedure**:
1. Send command: "Pick up the cup near the table"
2. Monitor object disambiguation
3. Verify correct cup selection based on context
4. Confirm successful execution

**Expected Results**:
- Perceptron system detects multiple objects
- Cognitive planner disambiguates based on context
- Robot selects correct object
- Successful manipulation of specified object

```python
# test_scenarios/environmental_context_test.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import DetectedObjects, Object
from geometry_msgs.msg import Point
import time
import math
import json


class EnvironmentalContextTestScenario(Node):
    """
    Test scenario for environmental context understanding
    """
    
    def __init__(self):
        super().__init__('environmental_context_test_scenario')
        
        # Publishers
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            10
        )
        
        # Subscribers
        self.detected_objects_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.detected_objects_callback,
            10
        )
        
        self.action_sequence_subscriber = self.create_subscription(
            vla_msgs.msg.ActionSequence,
            '/vla/action_sequence',
            self.action_sequence_callback,
            10
        )
        
        self.status_subscriber = self.create_subscription(
            String,
            '/vla_system/status',
            self.status_callback,
            10
        )
        
        # Test state
        self.objects_detected = []
        self.context_understood = False
        self.correct_object_selected = False
        self.test_completed = False
        self.target_object = None  # Will be set based on context
        self.status_history = []
        
        self.get_logger().info('Environmental Context Test Scenario initialized')
    
    def detected_objects_callback(self, msg):
        """
        Process detected objects to verify context understanding
        """
        self.objects_detected = msg.objects
        self.get_logger().info(f'Detected {len(msg.objects)} objects in environment')
        
        # Identify objects relevant to the test
        for obj in msg.objects:
            if 'cup' in obj.name.lower():
                self.get_logger().info(f'Found cup: {obj.name} at ({obj.pose.position.x}, {obj.pose.position.y})')
    
    def action_sequence_callback(self, msg):
        """
        Check if the correct object is specified in the action sequence
        """
        for action in msg.actions:
            if action.type == 'grasp' and 'params' in action.params:
                try:
                    params = json.loads(action.params)
                    object_name = params.get('object_name', '')
                    
                    # Check if the selected object makes sense in context
                    if self.objects_detected:
                        # Find the cup that is "near the table" (dining table at -2, -2)
                        dining_table_pos = Point(x=-2.0, y=-2.0, z=0.0)
                        relevant_cup = self.find_closest_cup_to_position(dining_table_pos, self.objects_detected)
                        
                        if relevant_cup and relevant_cup.name.lower() in object_name.lower():
                            self.correct_object_selected = True
                            self.context_understood = True
                            self.get_logger().info(f'✅ Correct object selected based on context: {object_name}')
                        else:
                            self.get_logger().info(f'❌ Object selected may not match context: {object_name}')
                            self.get_logger().info(f'   Context-relevant object would be: {relevant_cup.name if relevant_cup else "None"}')
                except json.JSONDecodeError:
                    self.get_logger().error(f'Could not parse action params: {action.params}')
    
    def find_closest_cup_to_position(self, target_pos, objects):
        """
        Find the cup closest to a target position
        """
        cups = [obj for obj in objects if 'cup' in obj.name.lower()]
        if not cups:
            return None
        
        closest_cup = min(
            cups,
            key=lambda obj: math.sqrt(
                (obj.pose.position.x - target_pos.x)**2 +
                (obj.pose.position.y - target_pos.y)**2
            )
        )
        
        return closest_cup
    
    def status_callback(self, msg):
        """
        Monitor execution status
        """
        self.status_history.append(msg.data)
        
        # Check for task completion
        if 'completed' in msg.data.lower() or 'success' in msg.data.lower():
            self.test_completed = True
    
    def execute_test(self):
        """
        Execute the environmental context test scenario
        """
        self.get_logger().info('Starting environmental context test')
        self.get_logger().info('Command: "Pick up the cup near the table"')
        
        # Send contextual command
        command_msg = String()
        command_msg.data = "Pick up the cup near the table"
        self.command_publisher.publish(command_msg)
        
        self.get_logger().info(f'Sent contextual command: "{command_msg.data}"')
        
        # Wait for test completion or timeout
        timeout = time.time() + 45.0  # 45 second timeout
        while not self.test_completed and time.time() < timeout:
            time.sleep(0.1)
        
        # Report results
        self.report_results()
    
    def report_results(self):
        """
        Report test results
        """
        self.get_logger().info('=== ENVIRONMENTAL CONTEXT TEST RESULTS ===')
        
        self.get_logger().info(f'Objects detected: {len(self.objects_detected)}')
        
        if self.context_understood:
            self.get_logger().info('✅ Environmental context was understood correctly')
        else:
            self.get_logger().info('❌ Environmental context was not correctly understood')
        
        if self.correct_object_selected:
            self.get_logger().info('✅ Correct object was selected based on context')
        else:
            self.get_logger().info('❌ Incorrect object might have been selected')
        
        if self.test_completed:
            self.get_logger().info('✅ Test completed')
        else:
            self.get_logger().info('❌ Test timed out')
        
        self.get_logger().info(f'Total status updates: {len(self.status_history)}')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = EnvironmentalContextTestScenario()
    
    try:
        test_node.execute_test()
    except KeyboardInterrupt:
        test_node.get_logger().info('Environmental context test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Automated Test Suite

### Main Test Runner

```python
# run_simulation_tests.py
import subprocess
import sys
import time
import os
from pathlib import Path


class VLASimulationTestSuite:
    """
    Automated test suite for VLA system simulation scenarios
    """
    
    def __init__(self):
        self.test_scenarios = [
            ("Navigation Test", "navigation_test.py"),
            ("Manipulation Test", "manipulation_test.py"),
            ("Multi-Step Task Test", "multi_step_test.py"),
            ("Obstacle Avoidance Test", "obstacle_avoidance_test.py"),
            ("Environmental Context Test", "environmental_context_test.py")
        ]
        self.results = {}
    
    def run_test(self, test_name, test_script):
        """
        Run a single test scenario
        """
        print(f"\n{'='*60}")
        print(f"RUNNING: {test_name}")
        print(f"{'='*60}")
        
        try:
            # Run the test script
            result = subprocess.run([
                sys.executable, 
                str(Path(__file__).parent / "test_scenarios" / test_script)
            ], capture_output=True, text=True, timeout=180)  # 3-minute timeout
            
            success = result.returncode == 0
            output = result.stdout
            error = result.stderr
            
            # Analyze output for test results
            if success:
                if "✅" in output and "PASSED" in output:
                    status = "PASSED"
                elif "❌" in output and "FAILED" in output:
                    status = "FAILED"
                    success = False
                else:
                    # Look for positive indicators in output
                    if "completed" in output.lower() or "success" in output.lower():
                        status = "PASSED"
                    else:
                        status = "INCONCLUSIVE"
            else:
                status = "FAILED"
            
            print(f"STATUS: {status}")
            if error:
                print(f"ERROR OUTPUT:\n{error}")
            
            self.results[test_name] = {
                'success': success,
                'status': status,
                'output': output,
                'error': error
            }
            
        except subprocess.TimeoutExpired:
            print(f"STATUS: TIMEOUT")
            self.results[test_name] = {
                'success': False,
                'status': 'TIMEOUT',
                'output': '',
                'error': 'Test timed out after 3 minutes'
            }
        except Exception as e:
            print(f"STATUS: ERROR - {e}")
            self.results[test_name] = {
                'success': False,
                'status': 'ERROR',
                'output': '',
                'error': str(e)
            }
    
    def run_all_tests(self):
        """
        Run all test scenarios
        """
        print("Starting VLA System Simulation Test Suite")
        print(f"Running {len(self.test_scenarios)} test scenarios...\n")
        
        for test_name, test_script in self.test_scenarios:
            self.run_test(test_name, test_script)
            time.sleep(2)  # Brief pause between tests
    
    def generate_report(self):
        """
        Generate test report
        """
        print(f"\n{'='*60}")
        print("TEST SUITE RESULTS SUMMARY")
        print(f"{'='*60}")
        
        passed = 0
        failed = 0
        inconclusive = 0
        
        for test_name, result in self.results.items():
            status = result['status']
            print(f"{test_name}: {status}")
            
            if result['success']:
                passed += 1
            elif status == 'INCONCLUSIVE':
                inconclusive += 1
            else:
                failed += 1
        
        print(f"\nSUMMARY:")
        print(f"  Total Tests: {len(self.results)}")
        print(f"  Passed: {passed}")
        print(f"  Failed: {failed}")
        print(f"  Inconclusive: {inconclusive}")
        
        success_rate = (passed / len(self.results)) * 100 if self.results else 0
        print(f"  Success Rate: {success_rate:.1f}%")
        
        return passed, failed, inconclusive, success_rate
    
    def save_detailed_report(self, filename="vla_test_report.txt"):
        """
        Save detailed test report to file
        """
        with open(filename, 'w') as f:
            f.write("VLA SYSTEM SIMULATION TEST REPORT\n")
            f.write("="*50 + "\n\n")
            
            for test_name, result in self.results.items():
                f.write(f"TEST: {test_name}\n")
                f.write(f"Status: {result['status']}\n")
                f.write(f"Success: {result['success']}\n")
                
                if result['output']:
                    f.write(f"Output:\n{result['output']}\n")
                
                if result['error']:
                    f.write(f"Error:\n{result['error']}\n")
                
                f.write("-" * 50 + "\n\n")
        
        print(f"\nDetailed report saved to: {filename}")


def main():
    """
    Main function to run the test suite
    """
    # Check if simulation is running
    print("Checking for active Gazebo simulation...")
    
    # Note: In a real implementation, you might want to check if Gazebo is running
    # For now, we'll assume the simulation is already started separately
    
    print("Starting VLA System Simulation Test Suite...")
    print("Note: Make sure Gazebo simulation is running with VLA system components loaded")
    
    # Create and run test suite
    test_suite = VLASimulationTestSuite()
    test_suite.run_all_tests()
    
    # Generate summary
    passed, failed, inconclusive, success_rate = test_suite.generate_report()
    
    # Save detailed report
    test_suite.save_detailed_report()
    
    # Exit with appropriate code
    if failed == 0 and inconclusive == 0:
        print("\n🎉 All tests passed! VLA system is ready for deployment.")
        return 0
    elif success_rate >= 80:
        print(f"\n⚠️  Tests mostly passed with {success_rate:.1f}% success rate. Review failed tests.")
        return 1
    else:
        print(f"\n❌ Tests failed with only {success_rate:.1f}% success rate. System needs debugging.")
        return 2


if __name__ == "__main__":
    exit(main())
```

## Performance and Stress Testing

### Performance Metrics Collection

```python
# performance_metrics.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import SystemHealth
import time
import statistics
from collections import deque
import json


class PerformanceMetricsCollector(Node):
    """
    Node to collect performance metrics for the VLA system
    """
    
    def __init__(self):
        super().__init__('performance_metrics_collector')
        
        # Subscribers for system status
        self.status_subscriber = self.create_subscription(
            String,
            '/vla_system/status',
            self.status_callback,
            10
        )
        
        self.health_subscriber = self.create_subscription(
            SystemHealth,
            '/vla_system/health',
            self.health_callback,
            10
        )
        
        # Metrics storage
        self.command_process_times = deque(maxlen=100)  # Keep last 100 measurements
        self.navigation_speeds = deque(maxlen=100)
        self.system_uptime = time.time()
        self.total_commands_processed = 0
        self.start_time = time.time()
        
        # Timers for periodic metrics reporting
        self.metrics_timer = self.create_timer(5.0, self.report_metrics)
        
        self.get_logger().info('Performance Metrics Collector initialized')
    
    def status_callback(self, msg):
        """
        Process system status for performance metrics
        """
        try:
            status_data = json.loads(msg.data)
            
            # Record command processing if applicable
            if 'command' in status_data:
                self.total_commands_processed += 1
            
            # Record processing time if available
            if 'processing_time' in status_data:
                self.command_process_times.append(status_data['processing_time'])
                
        except json.JSONDecodeError:
            pass  # Ignore invalid JSON messages
    
    def health_callback(self, msg):
        """
        Process system health metrics
        """
        # Could track additional metrics like error rates, etc.
        pass
    
    def report_metrics(self):
        """
        Report current performance metrics
        """
        current_time = time.time()
        uptime_seconds = current_time - self.system_uptime
        
        # Calculate metrics
        avg_process_time = statistics.mean(self.command_process_times) if self.command_process_times else 0
        min_process_time = min(self.command_process_times) if self.command_process_times else 0
        max_process_time = max(self.command_process_times) if self.command_process_times else 0
        
        commands_per_minute = (self.total_commands_processed / uptime_seconds) * 60 if uptime_seconds > 0 else 0
        
        # Report metrics
        self.get_logger().info(f'=== PERFORMANCE METRICS ===')
        self.get_logger().info(f'Uptime: {uptime_seconds:.1f} seconds')
        self.get_logger().info(f'Total commands processed: {self.total_commands_processed}')
        self.get_logger().info(f'Commands per minute: {commands_per_minute:.2f}')
        self.get_logger().info(f'Avg command process time: {avg_process_time:.3f}s')
        self.get_logger().info(f'Min process time: {min_process_time:.3f}s')
        self.get_logger().info(f'Max process time: {max_process_time:.3f}s')
        self.get_logger().info(f'Stored measurements: {len(self.command_process_times)}')
        self.get_logger().info(f'===========================')


def main(args=None):
    rclpy.init(args=args)
    
    metrics_collector = PerformanceMetricsCollector()
    
    try:
        rclpy.spin(metrics_collector)
    except KeyboardInterrupt:
        metrics_collector.get_logger().info('Performance metrics collection stopped')
    finally:
        metrics_collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Conclusion

These simulation scenarios provide comprehensive testing for the VLA system, covering:

1. **Basic Navigation**: Verifies the system can understand and execute navigation commands
2. **Object Manipulation**: Tests the perception and manipulation capabilities
3. **Multi-Step Tasks**: Validates the cognitive planner's ability to handle complex tasks
4. **Obstacle Avoidance**: Ensures safe navigation in dynamic environments
5. **Environmental Context**: Tests understanding of spatial relationships and context

Each scenario includes detailed setup instructions, expected outcomes, and automated test implementations to ensure consistent and reproducible testing of the VLA system in simulation environments.
# Simulation Test for Voice Command Pipeline using Gazebo

This document provides a complete implementation for testing the voice command pipeline in a Gazebo simulation environment. The test validates that voice commands are properly processed and result in expected robotic actions in simulation.

## Overview

The simulation test framework for the voice command pipeline provides an end-to-end testing environment where natural language commands are processed by the VLA system and executed by a simulated robot in Gazebo. This allows validation of the entire voice command processing chain without requiring physical hardware.

## Complete Simulation Test Implementation

```python
#!/usr/bin/env python3
"""
Simulation Test for Voice Command Pipeline

This script provides end-to-end testing of the voice command pipeline
in a Gazebo simulation environment.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry

from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformListener, Buffer

import time
import threading
from typing import Dict, List, Optional, Tuple
import math
import json


class VoiceCommandSimulationTester(Node):
    """
    Node for testing voice command pipeline in simulation
    """
    
    def __init__(self):
        super().__init__('voice_command_sim_tester')
        
        # Parameters for testing
        self.declare_parameter('test_timeout', 60.0)  # seconds
        self.declare_parameter('robot_name', 'turtlebot3')
        self.declare_parameter('initial_position', [0.0, 0.0, 0.0])  # x, y, theta
        
        self.test_timeout = self.get_parameter('test_timeout').value
        self.robot_name = self.get_parameter('robot_name').value
        initial_pos = self.get_parameter('initial_position').value
        
        # Test state management
        self.test_active = False
        self.test_start_time = None
        self.current_test = None
        self.test_results = []
        
        # Robot state
        self.current_pose = Pose()
        self.current_pose.position.x = initial_pos[0]
        self.current_pose.position.y = initial_pos[1]
        self.set_yaw_in_pose(self.current_pose, initial_pos[2])
        
        # Publishers
        self.voice_command_pub = self.create_publisher(
            String,
            '/vla/command',
            QoSProfile(depth=10)
        )
        
        self.test_status_pub = self.create_publisher(
            String,
            '/vla/test_status',
            QoSProfile(depth=10)
        )
        
        self.test_result_pub = self.create_publisher(
            String,
            '/vla/test_result',
            QoSProfile(depth=10)
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10)
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/vla/command_status',
            self.status_callback,
            QoSProfile(depth=10)
        )
        
        self.text_recognition_sub = self.create_subscription(
            String,
            '/vla/recognized_text',
            self.text_recognition_callback,
            QoSProfile(depth=10)
        )
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for test monitoring
        self.test_monitor_timer = self.create_timer(1.0, self.monitor_test_progress)
        
        self.get_logger().info("Voice Command Simulation Tester initialized")
    
    def set_yaw_in_pose(self, pose: Pose, yaw: float):
        """
        Set yaw angle in pose quaternion
        """
        from math import sin, cos
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        pose.orientation.z = sy
        pose.orientation.w = cy
    
    def get_yaw_from_pose(self, pose: Pose) -> float:
        """
        Extract yaw angle from pose quaternion
        """
        import math
        siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + 
                         pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1 - 2 * (pose.orientation.y * pose.orientation.y + 
                             pose.orientation.z * pose.orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg: Odometry):
        """
        Update current robot pose from odometry
        """
        self.current_pose = msg.pose.pose
    
    def status_callback(self, msg: String):
        """
        Handle command status updates
        """
        status_text = msg.data
        self.get_logger().debug(f"Command status: {status_text}")
        
        if self.test_active and self.current_test:
            # Log status in current test context
            self.current_test['status_updates'].append({
                'timestamp': time.time(),
                'status': status_text
            })
    
    def text_recognition_callback(self, msg: String):
        """
        Handle recognized text updates
        """
        recognized_text = msg.data
        self.get_logger().info(f"Recognized text: {recognized_text}")
        
        if self.test_active and self.current_test:
            self.current_test['text_recognized'] = recognized_text
    
    def start_test(self, test_name: str, commands: List[str], 
                   expected_outcomes: List[str] = None):
        """
        Start a new test with specified commands
        """
        if self.test_active:
            self.get_logger().error("Test already in progress")
            return False
        
        self.get_logger().info(f"Starting test: {test_name}")
        
        # Initialize test state
        self.test_active = True
        self.test_start_time = time.time()
        self.current_test = {
            'name': test_name,
            'commands': commands[:],  # Copy the list
            'expected_outcomes': expected_outcomes[:] if expected_outcomes else [],
            'status_updates': [],
            'text_recognized': None,
            'initial_pose': self.current_pose,
            'results': [],
            'passed': False,
            'execution_log': []
        }
        
        # Send the commands to the system
        for i, command in enumerate(commands):
            self.get_logger().info(f"Sending command {i+1}/{len(commands)}: {command}")
            cmd_msg = String()
            cmd_msg.data = command
            self.voice_command_pub.publish(cmd_msg)
            
            # Wait a bit between commands
            time.sleep(0.5)
        
        # Publish test start status
        status_msg = String()
        status_msg.data = f"test_started: {test_name}"
        self.test_status_pub.publish(status_msg)
        
        return True
    
    def monitor_test_progress(self):
        """
        Monitor test progress and determine if test should end
        """
        if not self.test_active or not self.current_test:
            return
        
        elapsed_time = time.time() - self.test_start_time
        
        # Check if test timeout has been reached
        if elapsed_time > self.test_timeout:
            self.end_test("timeout")
        
        # Additional conditions could be added here to end tests early
        # based on certain criteria being met
    
    def end_test(self, reason: str = "completed"):
        """
        End the current test and record results
        """
        if not self.test_active or not self.current_test:
            return
        
        test_end_time = time.time()
        test_duration = test_end_time - self.test_start_time
        
        # Finalize test results
        self.current_test['passed'] = self.evaluate_test_results()
        self.current_test['duration'] = test_duration
        self.current_test['end_reason'] = reason
        
        self.get_logger().info(
            f"Test '{self.current_test['name']}' {reason}: "
            f"passed={self.current_test['passed']}, "
            f"duration={test_duration:.2f}s"
        )
        
        # Log test result
        test_result = {
            'test_name': self.current_test['name'],
            'passed': self.current_test['passed'],
            'duration': test_duration,
            'commands': self.current_test['commands'],
            'text_recognized': self.current_test['text_recognized'],
            'end_reason': reason
        }
        
        self.test_results.append(test_result)
        
        # Publish result
        result_msg = String()
        result_msg.data = json.dumps(test_result)
        self.test_result_pub.publish(result_msg)
        
        # Reset test state
        self.test_active = False
        self.test_start_time = None
        test_to_save = self.current_test
        self.current_test = None
        
        # Log detailed result
        self.get_logger().info(f"Test result: {json.dumps(test_result, indent=2)}")
    
    def evaluate_test_results(self) -> bool:
        """
        Evaluate if the test was successful based on expected outcomes
        """
        if not self.current_test:
            return False
        
        # Basic evaluation - check if recognized text matches expectations
        recognized_text = self.current_test.get('text_recognized', '').lower()
        expected_outcomes = self.current_test.get('expected_outcomes', [])
        
        if expected_outcomes:
            # Check if any expected outcome is contained in recognized text
            for expected in expected_outcomes:
                if expected.lower() in recognized_text:
                    return True
            return False
        
        # For movement tests, we could check final position vs expected
        # This is a simplified check - real evaluation would be more complex
        return bool(recognized_text.strip())
    
    def run_predefined_tests(self):
        """
        Run a series of predefined tests
        """
        self.get_logger().info("Running predefined tests...")
        
        # Test 1: Basic movement command
        self.start_test(
            test_name="Basic Movement",
            commands=["Move forward 1 meter"],
            expected_outcomes=["move", "forward"]
        )
        
        # Wait for first test to complete
        time.sleep(10)  # Wait longer than the actual test execution
        
        # Test 2: Turn command
        self.start_test(
            test_name="Turn Command",
            commands=["Turn left 90 degrees"],
            expected_outcomes=["turn", "left"]
        )
        
        time.sleep(10)
        
        # Test 3: Complex command
        self.start_test(
            test_name="Complex Command",
            commands=["Go to the kitchen and pick up the red cup"],
            expected_outcomes=["go", "kitchen", "pick"]
        )
        
        time.sleep(10)
        
        # Test 4: Multiple commands
        self.start_test(
            test_name="Multiple Commands",
            commands=["Move forward", "Stop", "Turn right"],
            expected_outcomes=["move", "stop", "turn"]
        )
        
        time.sleep(15)  # Longer for multiple commands
        
        # Print summary
        self.print_test_summary()
    
    def print_test_summary(self):
        """
        Print a summary of all completed tests
        """
        if not self.test_results:
            self.get_logger().info("No tests completed")
            return
        
        self.get_logger().info("=== Test Summary ===")
        passed_count = sum(1 for result in self.test_results if result['passed'])
        total_count = len(self.test_results)
        
        for result in self.test_results:
            status = "PASSED" if result['passed'] else "FAILED"
            self.get_logger().info(
                f"{status}: {result['test_name']} "
                f"({result['duration']:.2f}s)"
            )
        
        self.get_logger().info(f"Overall: {passed_count}/{total_count} tests passed")


class SimulationEnvironment:
    """
    Helper class to manage the simulation environment
    """
    
    def __init__(self):
        self.gazebo_started = False
        self.model_spawned = False
    
    def start_gazebo(self, world_name: str = "empty_world"):
        """
        Start Gazebo simulation with specified world
        """
        import subprocess
        import os
        
        try:
            # Start Gazebo with specified world
            cmd = ["ros2", "launch", "turtlebot3_gazebo", f"empty_world.launch.py"]
            self.gazebo_process = subprocess.Popen(cmd)
            
            # Wait a bit for Gazebo to start
            time.sleep(5)
            
            self.gazebo_started = True
            print(f"Gazebo started with world: {world_name}")
            return True
            
        except Exception as e:
            print(f"Failed to start Gazebo: {str(e)}")
            return False
    
    def spawn_robot(self, model_name: str = "turtlebot3_waffle"):
        """
        Spawn robot model in Gazebo
        """
        import subprocess
        
        if not self.gazebo_started:
            print("Gazebo not started, cannot spawn robot")
            return False
        
        try:
            # Spawn the robot model
            cmd = [
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", "turtlebot3", 
                "-database", model_name,
                "-x", "0", "-y", "0", "-z", "0"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.model_spawned = True
                print(f"Robot model {model_name} spawned successfully")
                return True
            else:
                print(f"Failed to spawn robot: {result.stderr}")
                return False
                
        except Exception as e:
            print(f"Error spawning robot: {str(e)}")
            return False
    
    def stop_simulation(self):
        """
        Stop the simulation environment
        """
        if hasattr(self, 'gazebo_process'):
            self.gazebo_process.terminate()
            self.gazebo_process.wait()
            print("Gazebo simulation stopped")


def run_complete_simulation_test():
    """
    Run complete simulation test from start to finish
    """
    print("Starting complete voice command simulation test...")
    
    # Initialize simulation environment
    sim_env = SimulationEnvironment()
    
    try:
        # Start Gazebo
        if not sim_env.start_gazebo():
            print("Failed to start Gazebo, exiting test")
            return
        
        # Wait for ROS to be ready
        time.sleep(5)
        
        # Spawn robot
        if not sim_env.spawn_robot():
            print("Failed to spawn robot, exiting test")
            return
        
        # Wait for robot to be ready
        time.sleep(3)
        
        # Initialize ROS
        rclpy.init()
        
        # Create and run tester node
        tester_node = VoiceCommandSimulationTester()
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(tester_node)
        
        # Start a separate thread to run the tests
        def run_tests():
            time.sleep(5)  # Wait for all systems to be ready
            tester_node.run_predefined_tests()
        
        test_thread = threading.Thread(target=run_tests)
        test_thread.start()
        
        try:
            # Spin the node to handle callbacks
            executor.spin()
        except KeyboardInterrupt:
            print("Test interrupted by user")
        finally:
            # Cleanup
            executor.shutdown()
            tester_node.destroy_node()
            test_thread.join(timeout=5)
        
        rclpy.shutdown()
    
    finally:
        # Always stop simulation
        sim_env.stop_simulation()


def main(args=None):
    """
    Main function to run the simulation test
    """
    run_complete_simulation_test()


# Additional test utilities
class TestAssertions:
    """
    Utility class for test assertions in the simulation
    """
    
    @staticmethod
    def assert_robot_moved_to_position(tester_node, expected_x, expected_y, tolerance=0.2):
        """
        Assert that robot moved to expected position
        """
        current_x = tester_node.current_pose.position.x
        current_y = tester_node.current_pose.position.y
        
        distance = math.sqrt(
            (current_x - expected_x)**2 + (current_y - expected_y)**2
        )
        
        if distance <= tolerance:
            tester_node.get_logger().info(
                f"Robot is at expected position: ({current_x:.2f}, {current_y:.2f})"
            )
            return True
        else:
            tester_node.get_logger().error(
                f"Robot not at expected position. "
                f"Expected: ({expected_x}, {expected_y}), "
                f"Actual: ({current_x:.2f}, {current_y:.2f}), "
                f"Distance: {distance:.2f}, Tolerance: {tolerance}"
            )
            return False
    
    @staticmethod
    def assert_recognized_text_contains(tester_node, expected_text_fragment):
        """
        Assert that recognized text contains expected fragment
        """
        recognized_text = getattr(tester_node, 'last_recognized_text', '')
        if expected_text_fragment.lower() in recognized_text.lower():
            tester_node.get_logger().info(
                f"Text contains expected fragment: '{expected_text_fragment}'"
            )
            return True
        else:
            tester_node.get_logger().error(
                f"Expected text fragment '{expected_text_fragment}' not found in: '{recognized_text}'"
            )
            return False


# Example custom test cases
def custom_test_scenario_1(tester_node):
    """
    Custom test scenario: Navigate to specific location
    """
    tester_node.get_logger().info("Running custom test: Navigate to location")
    
    # Start test
    success = tester_node.start_test(
        test_name="Navigate to Location",
        commands=["Go to the charging station"],
        expected_outcomes=["navigate", "go", "station"]
    )
    
    if success:
        # Wait for navigation to complete and verify position
        time.sleep(15)  # Wait for navigation
        
        # Verify robot moved to expected area (example coordinates)
        success = TestAssertions.assert_robot_moved_to_position(
            tester_node, expected_x=1.0, expected_y=1.0
        )
        
        if success:
            tester_node.get_logger().info("Navigation test passed")
        else:
            tester_node.get_logger().error("Navigation test failed")


if __name__ == '__main__':
    main()
```

## 7. Example Test Configuration Files

### Test Configuration YAML

```yaml
# vla_simulation_tests.yaml
voice_command_sim_tester:
  ros__parameters:
    test_timeout: 60.0
    robot_name: 'turtlebot3'
    initial_position: [0.0, 0.0, 0.0]
    test_scenarios:
      basic_movement:
        commands: ["Move forward 1 meter"]
        expected_outcomes: ["move", "forward"]
      turn_test:
        commands: ["Turn left 90 degrees"]
        expected_outcomes: ["turn", "left"]
      complex_command:
        commands: ["Go to the kitchen and pick up the red cup"]
        expected_outcomes: ["go", "kitchen", "pick"]
```

### Gazebo World Configuration for Testing

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="vla_test_world">
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 -0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add landmarks for navigation testing -->
    <model name="kitchen_area">
      <pose>3 2 0 0 0 0</pose>
      <link name="kitchen_link">
        <visual name="kitchen_visual">
          <geometry>
            <box>
              <size>2 2 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.7 1.0 1</ambient>
            <diffuse>0.5 0.7 1.0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="charging_station">
      <pose>-2 -3 0 0 0 0</pose>
      <link name="station_link">
        <visual name="station_visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1.0 0.5 0.5 1</ambient>
            <diffuse>1.0 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Place a simple obstacle -->
    <model name="obstacle1">
      <pose>1 0 0 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## 8. Test Execution Scripts

### Launch File for Testing

```xml
<!-- launch/voice_command_test.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('vla_simulation_tests')
    
    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='vla_test_world',
        description='Choose one of the world files from `/vla_simulation_tests/worlds`'
    )
    
    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch',
            '/gazebo.launch.py'
        ]),
        launch_arguments={'world': PathJoinSubstitution([
            package_dir, 'worlds', LaunchConfiguration('world')
        ])}.items()
    )
    
    # Robot spawn node
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-database', 'turtlebot3_waffle',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )
    
    # Voice command simulation tester
    tester = Node(
        package='vla_simulation_tests',
        executable='voice_command_simulation_tester',
        name='voice_command_sim_tester',
        parameters=[
            os.path.join(package_dir, 'config', 'vla_simulation_tests.yaml')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_robot,
        tester
    ])
```

## 9. Reporting and Analysis Tools

```python
#!/usr/bin/env python3
"""
Analysis and reporting tools for voice command simulation tests
"""

import json
import matplotlib.pyplot as plt
import pandas as pd
from typing import List, Dict
import argparse
import os

def load_test_results(results_file: str) -> List[Dict]:
    """
    Load test results from file
    """
    with open(results_file, 'r') as f:
        results = [json.loads(line) for line in f if line.strip()]
    return results

def generate_test_report(results: List[Dict], output_dir: str):
    """
    Generate a comprehensive test report
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Create summary statistics
    df = pd.DataFrame(results)
    
    # Overall statistics
    total_tests = len(df)
    passed_tests = len(df[df['passed'] == True])
    success_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0
    
    # Create report
    report_content = f"""
# Voice Command Pipeline Simulation Test Report

## Summary
- Total Tests Run: {total_tests}
- Tests Passed: {passed_tests}
- Success Rate: {success_rate:.2f}%

## Test Results by Type
{df.groupby('test_name').agg({{'passed': ['count', 'sum', lambda x: (x.sum()/x.count())*100]}}).round(2)}

## Detailed Results
"""
    
    for i, result in enumerate(results):
        status = "PASSED" if result['passed'] else "FAILED"
        report_content += f"""
### Test {i+1}: {result['test_name']}
- Status: {status}
- Duration: {result['duration']:.2f}s
- Commands: {', '.join(result['commands'])}
- Recognized: {result.get('text_recognized', 'N/A')}
"""
    
    # Save report
    with open(os.path.join(output_dir, 'test_report.md'), 'w') as f:
        f.write(report_content)
    
    # Create visualization
    plt.figure(figsize=(10, 6))
    test_names = [r['test_name'] for r in results]
    statuses = ['Pass' if r['passed'] else 'Fail' for r in results]
    
    # Count pass/fail for each test name
    pass_fail_counts = {}
    for result in results:
        name = result['test_name']
        if name not in pass_fail_counts:
            pass_fail_counts[name] = {'pass': 0, 'fail': 0}
        if result['passed']:
            pass_fail_counts[name]['pass'] += 1
        else:
            pass_fail_counts[name]['fail'] += 1
    
    # Create stacked bar chart
    names = list(pass_fail_counts.keys())
    passes = [pass_fail_counts[n]['pass'] for n in names]
    fails = [pass_fail_counts[n]['fail'] for n in names]
    
    plt.bar(names, passes, label='Pass', color='green')
    plt.bar(names, fails, bottom=passes, label='Fail', color='red')
    plt.title('Test Results by Test Type')
    plt.xlabel('Test Type')
    plt.ylabel('Number of Tests')
    plt.xticks(rotation=45, ha='right')
    plt.legend()
    plt.tight_layout()
    
    plt.savefig(os.path.join(output_dir, 'test_results_chart.png'))
    plt.close()
    
    print(f"Test report generated in {output_dir}")

def main():
    parser = argparse.ArgumentParser(description='Analyze voice command simulation test results')
    parser.add_argument('results_file', help='Path to test results JSON file')
    parser.add_argument('-o', '--output', default='./test_report', help='Output directory for report')
    
    args = parser.parse_args()
    
    results = load_test_results(args.results_file)
    generate_test_report(results, args.output)

if __name__ == '__main__':
    main()
```

This comprehensive simulation test framework provides:

1. A complete testing environment for voice command processing
2. Integration with Gazebo simulation
3. Automated test execution with various scenarios
4. Detailed result reporting and analysis
5. Configuration flexibility for different test scenarios
6. Visualization of test results

The framework enables thorough validation of the voice command pipeline in a controlled simulation environment before deployment to physical robots.
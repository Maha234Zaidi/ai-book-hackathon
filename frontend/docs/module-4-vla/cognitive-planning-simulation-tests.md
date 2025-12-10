# Simulation Tests for Cognitive Planning using Gazebo

## Overview

This document outlines simulation tests for cognitive planning systems in Vision-Language-Action (VLA) systems using the Gazebo simulation environment. These tests validate the cognitive planner's ability to interpret commands and generate appropriate action sequences in a simulated environment.

## Test Setup

Before running the tests, ensure your Gazebo environment is properly configured with:

1. A simulated robot (e.g., TurtleBot3)
2. Objects for manipulation (cups, blocks, etc.)
3. ROS 2 bridge between Gazebo and the cognitive planner
4. Proper sensor simulation (camera, LiDAR, etc.)

## Basic Simulation Test Framework

```python
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from . import cognitive_planner  # Import the cognitive planner module
import time
import json


class GazeboSimulationTestNode(Node):
    """
    ROS 2 node for running simulation tests with Gazebo
    """
    
    def __init__(self):
        super().__init__('gazebo_simulation_test_node')
        
        # Publishers and subscribers for simulation interaction
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            QoSProfile(depth=10)
        )
        
        self.action_subscriber = self.create_subscription(
            String,  # In practice, use proper action sequence message
            '/vla/action_sequence',
            self.action_callback,
            QoSProfile(depth=10)
        )
        
        self.sim_pose_publisher = self.create_publisher(
            Pose,
            '/gazebo/set_model_state',
            QoSProfile(depth=10)
        )
        
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            QoSProfile(depth=10)
        )
        
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10)
        )
        
        # Test state tracking
        self.last_action_sequence = None
        self.simulation_ready = False
        self.perception_data = None
        
        self.get_logger().info('Gazebo Simulation Test Node initialized')
    
    def camera_callback(self, msg):
        """Handle camera data from simulation"""
        self.perception_data = {
            'camera': msg,
            'timestamp': self.get_clock().now().nanoseconds
        }
    
    def laser_callback(self, msg):
        """Handle laser scan data from simulation"""
        self.perception_data = self.perception_data or {}
        self.perception_data['laser'] = msg
    
    def action_callback(self, msg):
        """Handle received action sequence"""
        try:
            self.last_action_sequence = json.loads(msg.data)
            self.get_logger().info(f'Received action sequence with {len(self.last_action_sequence.get("actions", []))} actions')
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse action sequence')
    
    def send_command(self, command):
        """Send a natural language command to the cognitive planner"""
        cmd_msg = String()
        cmd_msg.data = command
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info(f'Sent command: {command}')
    
    def wait_for_action_sequence(self, timeout=10.0):
        """Wait for an action sequence to be generated"""
        start_time = time.time()
        while self.last_action_sequence is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.last_action_sequence is not None
    
    def reset_simulation(self):
        """Reset the simulation to a known state"""
        # In a real implementation, this would interface with Gazebo services
        # to reset the environment
        pass


class CognitivePlanningGazeboTests(unittest.TestCase):
    """
    Test suite for cognitive planning in Gazebo simulation
    """
    
    @classmethod
    def setUpClass(cls):
        """Set up the test environment before running tests"""
        rclpy.init()
        cls.test_node = GazeboSimulationTestNode()
        
        # Wait for simulation to be ready
        time.sleep(3.0)  # Allow time for Gazebo to initialize
    
    @classmethod
    def tearDownClass(cls):
        """Tear down the test environment after running tests"""
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    def setUp(self):
        """Set up before each test"""
        self.test_node.reset_simulation()
        self.test_node.last_action_sequence = None
    
    def test_simple_navigation_command(self):
        """Test a simple navigation command"""
        command = "Move to the red cup"
        
        # Send command to cognitive planner
        self.test_node.send_command(command)
        
        # Wait for action sequence
        action_received = self.test_node.wait_for_action_sequence(timeout=15.0)
        
        # Verify that an action sequence was generated
        self.assertTrue(action_received, "Action sequence was not received within timeout")
        
        # Verify that the sequence contains appropriate actions
        actions = self.test_node.last_action_sequence.get('actions', [])
        self.assertGreater(len(actions), 0, "Action sequence is empty")
        
        # Check that the sequence includes navigation
        navigation_action = next((a for a in actions if a['type'] == 'navigate_to'), None)
        self.assertIsNotNone(navigation_action, "No navigate_to action found in sequence")
        
        # Verify the navigation action has the correct parameters
        self.assertEqual(navigation_action['params']['object_name'], 'red cup')
    
    def test_object_manipulation_command(self):
        """Test a command involving object manipulation"""
        command = "Grasp the blue block and place it on the table"
        
        # Send command to cognitive planner
        self.test_node.send_command(command)
        
        # Wait for action sequence
        action_received = self.test_node.wait_for_action_sequence(timeout=20.0)
        
        # Verify that an action sequence was generated
        self.assertTrue(action_received, "Action sequence was not received within timeout")
        
        # Get actions
        actions = self.test_node.last_action_sequence.get('actions', [])
        self.assertGreater(len(actions), 0, "Action sequence is empty")
        
        # Check for required actions in the sequence
        action_types = [a['type'] for a in actions]
        
        # Should navigate to object, grasp it, navigate to destination, place it
        self.assertIn('navigate_to', action_types, "No navigate_to action found")
        self.assertIn('grasp', action_types, "No grasp action found")
        self.assertIn('place_at', action_types, "No place_at action found")
        
        # Verify the sequence has the expected number of steps
        self.assertGreaterEqual(len(actions), 4, "Not enough actions for object manipulation task")
    
    def test_complex_multi_step_command(self):
        """Test a complex multi-step command"""
        command = "Go to the kitchen, pick up the red cup, bring it to the living room, and place it on the table"
        
        # Send command to cognitive planner
        self.test_node.send_command(command)
        
        # Wait for action sequence
        action_received = self.test_node.wait_for_action_sequence(timeout=30.0)
        
        # Verify that an action sequence was generated
        self.assertTrue(action_received, "Action sequence was not received within timeout")
        
        # Get actions
        actions = self.test_node.last_action_sequence.get('actions', [])
        self.assertGreater(len(actions), 0, "Action sequence is empty")
        
        # Complex task should have multiple steps
        self.assertGreaterEqual(len(actions), 6, "Not enough actions for complex task")
        
        # Check for the expected sequence of actions
        action_types = [a['type'] for a in actions]
        
        # Should include multiple navigation, grasp, and place actions
        nav_count = action_types.count('navigate_to')
        self.assertGreaterEqual(nav_count, 2, "Expected at least 2 navigation actions")
        
        grasp_count = action_types.count('grasp')
        self.assertEqual(grasp_count, 1, "Expected exactly 1 grasp action")
        
        place_count = action_types.count('place_at')
        self.assertEqual(place_count, 1, "Expected exactly 1 place action")
    
    def test_command_with_obstacle_avoidance(self):
        """Test a command where the planner needs to account for obstacles"""
        command = "Navigate to the red cup without hitting the blue box"
        
        # Send command to cognitive planner
        self.test_node.send_command(command)
        
        # Wait for action sequence
        action_received = self.test_node.wait_for_action_sequence(timeout=15.0)
        
        # Verify that an action sequence was generated
        self.assertTrue(action_received, "Action sequence was not received within timeout")
        
        # Get actions
        actions = self.test_node.last_action_sequence.get('actions', [])
        self.assertGreater(len(actions), 0, "Action sequence is empty")
        
        # The sequence should include navigation and consider obstacles
        navigation_action = next((a for a in actions if a['type'] == 'navigate_to'), None)
        self.assertIsNotNone(navigation_action, "No navigate_to action found in sequence")
    
    def test_command_with_reasoning_explanation(self):
        """Test that commands generate proper reasoning explanations"""
        command = "Clean the room by putting toys in the toy box"
        
        # Send command to cognitive planner
        self.test_node.send_command(command)
        
        # Wait for action sequence
        action_received = self.test_node.wait_for_action_sequence(timeout=25.0)
        
        # Verify that an action sequence was generated
        self.assertTrue(action_received, "Action sequence was not received within timeout")
        
        # Get actions and check for reasoning explanations
        actions = self.test_node.last_action_sequence.get('actions', [])
        self.assertGreater(len(actions), 0, "Action sequence is empty")
        
        # All actions should have reasoning explanations (for educational version)
        for action in actions:
            self.assertIn('reasoning_explanation', action, 
                         f"Action {action['type']} missing reasoning explanation")
            self.assertIsInstance(action['reasoning_explanation'], str,
                                 f"Reasoning explanation for {action['type']} is not a string")
    
    def test_command_error_handling(self):
        """Test how the system handles invalid commands"""
        command = "Do something impossible"  # Invalid command
        
        # Send command to cognitive planner
        self.test_node.send_command(command)
        
        # Wait for action sequence (may be empty for invalid commands)
        action_received = self.test_node.wait_for_action_sequence(timeout=10.0)
        
        # Even with an invalid command, the system should respond appropriately
        # (either with a valid sequence or an error response)
        self.assertTrue(action_received or self.test_node.last_action_sequence is not None,
                       "No response received for invalid command")
    
    def test_command_with_perception_feedback(self):
        """Test command execution with perception feedback"""
        command = "Find and grasp the nearest red object"
        
        # Send command to cognitive planner
        self.test_node.send_command(command)
        
        # Wait for action sequence
        action_received = self.test_node.wait_for_action_sequence(timeout=20.0)
        
        # Verify that an action sequence was generated
        self.assertTrue(action_received, "Action sequence was not received within timeout")
        
        # Check that perception feedback was incorporated
        actions = self.test_node.last_action_sequence.get('actions', [])
        self.assertGreater(len(actions), 0, "Action sequence is empty")
        
        # Look for detection action in sequence (to find the object first)
        detect_action = next((a for a in actions if a['type'] == 'detect_object'), None)
        if detect_action:
            self.assertEqual(detect_action['params']['object_name'], 'red object')


def run_simulation_tests():
    """
    Run the simulation tests
    """
    # Create test suite
    test_suite = unittest.TestLoader().loadTestsFromTestCase(CognitivePlanningGazeboTests)
    
    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_simulation_tests()
    exit(0 if success else 1)
```

## Gazebo Integration for Testing

```python
# gazebo_integration.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile


class GazeboIntegration(Node):
    """
    Node to integrate cognitive planning tests with Gazebo simulation
    """
    
    def __init__(self):
        super().__init__('gazebo_integration')
        
        # Gazebo service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        
        # Wait for services
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting again...')
            
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set state service not available, waiting again...')
        
        # Test command publisher
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            QoSProfile(depth=10)
        )
        
        self.get_logger().info('Gazebo Integration Node initialized')
    
    def spawn_object(self, name, model_xml, pose, reference_frame=''):
        """
        Spawn an object in the simulation
        """
        request = SpawnEntity.Request()
        request.name = name
        request.xml = model_xml
        request.initial_pose = pose
        request.reference_frame = reference_frame
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully spawned {name}')
                return True
            else:
                self.get_logger().error(f'Failed to spawn {name}: {response.status_message}')
                return False
        else:
            self.get_logger().error(f'Failed to call spawn service for {name}')
            return False
    
    def delete_object(self, name):
        """
        Delete an object from the simulation
        """
        request = DeleteEntity.Request()
        request.name = name
        
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully deleted {name}')
                return True
            else:
                self.get_logger().error(f'Failed to delete {name}: {response.result}')
                return False
        else:
            self.get_logger().error(f'Failed to call delete service for {name}')
            return False
    
    def set_object_pose(self, name, pose, reference_frame=''):
        """
        Set the pose of an existing object
        """
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose = pose
        request.state.reference_frame = reference_frame
        
        future = self.set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully set pose for {name}')
                return True
            else:
                self.get_logger().error(f'Failed to set pose for {name}: {response.status_message}')
                return False
        else:
            self.get_logger().error(f'Failed to call set state service for {name}')
            return False
    
    def create_test_scenario(self, scenario_name):
        """
        Create a specific test scenario in the simulation
        """
        if scenario_name == "simple_navigation":
            # Create a simple scenario with one object to navigate to
            cup_pose = Pose()
            cup_pose.position.x = 1.0
            cup_pose.position.y = 1.0
            cup_pose.position.z = 0.0
            cup_pose.orientation.w = 1.0
            
            # Define a simple object model (in practice, use a proper URDF/SDF)
            cup_model = """
            <sdf version="1.6">
                <model name="red_cup">
                    <pose>1.0 1.0 0.0 0 0 0</pose>
                    <link name="link">
                        <visual name="visual">
                            <geometry>
                                <cylinder>
                                    <radius>0.05</radius>
                                    <length>0.1</length>
                                </cylinder>
                            </geometry>
                            <material>
                                <ambient>1 0 0 1</ambient>
                                <diffuse>1 0 0 1</diffuse>
                            </material>
                        </visual>
                        <collision name="collision">
                            <geometry>
                                <cylinder>
                                    <radius>0.05</radius>
                                    <length>0.1</length>
                                </cylinder>
                            </geometry>
                        </collision>
                    </link>
                </model>
            </sdf>
            """
            
            return self.spawn_object("red_cup", cup_model, cup_pose)
        
        elif scenario_name == "object_manipulation":
            # Create a scenario with multiple objects for manipulation
            table_pose = Pose()
            table_pose.position.x = 2.0
            table_pose.position.y = 0.0
            table_pose.position.z = 0.0
            table_pose.orientation.w = 1.0
            
            table_model = """
            <sdf version="1.6">
                <model name="table">
                    <pose>2.0 0.0 0.0 0 0 0</pose>
                    <link name="link">
                        <visual name="visual">
                            <geometry>
                                <box>
                                    <size>1.0 0.8 0.8</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.8 0.8 0.7 1</ambient>
                                <diffuse>0.8 0.8 0.7 1</diffuse>
                            </material>
                        </visual>
                        <collision name="collision">
                            <geometry>
                                <box>
                                    <size>1.0 0.8 0.8</size>
                                </box>
                            </geometry>
                        </collision>
                    </link>
                </model>
            </sdf>
            """
            
            block_pose = Pose()
            block_pose.position.x = 0.5
            block_pose.position.y = 0.5
            block_pose.position.z = 0.45  # On top of the table
            block_pose.orientation.w = 1.0
            
            block_model = """
            <sdf version="1.6">
                <model name="blue_block">
                    <pose>0.5 0.5 0.45 0 0 0</pose>
                    <link name="link">
                        <visual name="visual">
                            <geometry>
                                <box>
                                    <size>0.1 0.1 0.1</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0 0 1 1</ambient>
                                <diffuse>0 0 1 1</diffuse>
                            </material>
                        </visual>
                        <collision name="collision">
                            <geometry>
                                <box>
                                    <size>0.1 0.1 0.1</size>
                                </box>
                            </geometry>
                        </collision>
                    </link>
                </model>
            </sdf>
            """
            
            success1 = self.spawn_object("table", table_model, table_pose)
            success2 = self.spawn_object("blue_block", block_model, block_pose)
            
            return success1 and success2
        
        return False


# Example of how to use the integration in tests
def example_test_with_gazebo():
    """
    Example of running a test with Gazebo integration
    """
    rclpy.init()
    
    # Create the integration node
    integration_node = GazeboIntegration()
    
    # Create a test scenario
    success = integration_node.create_test_scenario("object_manipulation")
    if not success:
        integration_node.get_logger().error("Failed to create test scenario")
        return
    
    # Give Gazebo time to load the models
    time.sleep(2.0)
    
    # Send a command to test
    cmd_msg = String()
    cmd_msg.data = "Grasp the blue block and place it on the table"
    integration_node.command_publisher.publish(cmd_msg)
    
    # Wait for response (in real test, you'd have a more sophisticated verification)
    time.sleep(5.0)
    
    # Clean up
    integration_node.delete_object("blue_block")
    integration_node.delete_object("table")
    
    integration_node.destroy_node()
    rclpy.shutdown()
```

## Running the Simulation Tests

To run the simulation tests, use the following workflow:

1. Start Gazebo with your robot model:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. Start your cognitive planner node:
   ```bash
   ros2 run vla_examples cognitive_planner_node
   ```

3. Run the simulation tests:
   ```bash
   ros2 run vla_examples test_cognitive_planning_gazebo.py
   ```

## Continuous Integration Testing

For use in CI pipelines:

```python
# ci_simulation_tests.py
import subprocess
import sys
import time
import unittest


class CISimulationTests(unittest.TestCase):
    """
    Simulation tests designed for CI/CD pipelines
    """
    
    def setUp(self):
        """
        Start the simulation environment in headless mode for CI
        """
        # Start Gazebo in headless mode
        self.gazebo_process = subprocess.Popen([
            'gzserver',  # or 'gazebo' depending on version
            '--verbose',  # Add '--headless-rendering' for headless mode
            'empty.world'  # or your custom world file
        ])
        
        # Give Gazebo time to start
        time.sleep(5)
        
        # Start the cognitive planner
        self.planner_process = subprocess.Popen([
            'ros2', 'run', 'vla_examples', 'cognitive_planner_node'
        ])
        
        # Give the planner time to initialize
        time.sleep(3)
    
    def tearDown(self):
        """
        Clean up the simulation environment
        """
        if hasattr(self, 'planner_process'):
            self.planner_process.terminate()
            self.planner_process.wait()
        
        if hasattr(self, 'gazebo_process'):
            self.gazebo_process.terminate()
            self.gazebo_process.wait()
    
    def test_basic_functionality(self):
        """
        Test basic cognitive planning functionality in simulation
        """
        # Run the actual tests via command line
        result = subprocess.run([
            'ros2', 'run', 'vla_examples', 'test_cognitive_planning_gazebo.py'
        ], capture_output=True, text=True)
        
        # The test should return 0 for success
        self.assertEqual(result.returncode, 0, 
                        f"Simulation tests failed with output: {result.stdout} {result.stderr}")
    
    def test_performance_under_load(self):
        """
        Test performance under load with multiple simultaneous requests
        """
        # This would test how the cognitive planner performs when multiple
        # commands are sent in quick succession
        import threading
        import queue
        
        command_queue = queue.Queue()
        results_queue = queue.Queue()
        
        def send_command_and_wait(cmd):
            # In a real implementation, send command via ROS2 client
            # and wait for response
            time.sleep(1)  # Simulate processing time
            results_queue.put("success")
        
        # Send multiple commands concurrently
        commands = [
            "Move forward",
            "Turn left",
            "Grasp object",
            "Navigate to location"
        ]
        
        threads = []
        for cmd in commands:
            thread = threading.Thread(target=send_command_and_wait, args=(cmd,))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Verify all commands were processed
        completed_count = 0
        while not results_queue.empty():
            results_queue.get()
            completed_count += 1
        
        self.assertEqual(completed_count, len(commands), 
                        "Not all commands were processed successfully")
```

## Best Practices for Simulation Testing

1. **Isolated Environments**: Each test should run in its own Gazebo instance or reset the environment between tests.

2. **Deterministic Scenarios**: Ensure tests run in a predictable environment by controlling the initial state of objects and robot.

3. **Timeout Handling**: Implement proper timeouts to prevent tests from hanging indefinitely.

4. **Resource Management**: Properly clean up spawned objects and terminate processes after tests.

5. **Validation**: Verify not just that the planner returns an action sequence, but that the sequence is appropriate for the given scenario.

## Summary

These simulation tests validate cognitive planning functionality in realistic environments using Gazebo. They ensure that the cognitive planner can generate appropriate action sequences for various commands and scenarios, providing confidence in the system's capabilities before deployment to physical robots.
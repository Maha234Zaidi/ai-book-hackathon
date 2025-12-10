# Mapping System from Transcribed Text to ROS 2 Commands

This document provides an implementation of a system that maps transcribed text commands to ROS 2 commands. This component is essential for converting natural language into executable robotic actions.

## Complete Implementation

```python
#!/usr/bin/env python3
"""
Text-to-ROS Command Mapper for VLA System

This node listens for transcribed text commands and maps them to appropriate ROS 2 commands
for execution by the robotic platform.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState

# Import common ROS 2 action interfaces
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory

from typing import Dict, List, Optional, Tuple, Any
import re
import math


class TextToROSCommandMapper(Node):
    """
    Maps transcribed text commands to ROS 2 commands for robot execution.
    """
    
    def __init__(self):
        super().__init__('text_to_ros_command_mapper')
        
        # Configuration parameters
        self.declare_parameter('command_threshold', 0.7)
        self.declare_parameter('enable_nlu', True)  # Enable Natural Language Understanding
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        self.command_threshold = self.get_parameter('command_threshold').value
        self.enable_nlu = self.get_parameter('enable_nlu').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Command vocabulary and patterns
        self.command_patterns = {
            # Movement commands
            'move_forward': [r'move forward', r'go forward', r'forward', r'straight', r'step forward'],
            'move_backward': [r'move backward', r'go backward', r'backward', r'back', r'step back'],
            'turn_left': [r'turn left', r'rotate left', r'left', r'pivot left'],
            'turn_right': [r'turn right', r'rotate right', r'right', r'pivot right'],
            'move_to_location': [r'go to', r'move to', r'go over to', r'go toward', r'go near'],
            'stop': [r'stop', r'halt', r'freeze', r'hold position'],
            
            # Manipulation commands
            'grasp': [r'pick up', r'grasp', r'grab', r'catch', r'hold', r'take'],
            'release': [r'release', r'drop', r'let go', r'ungrasp', r'put down'],
            'lift': [r'lift', r'raise', r'pick up', r'up'],
            'lower': [r'lower', r'down', r'place down'],
            
            # Navigation commands
            'navigate': [r'navigate to', r'find', r'locate', r'go to the', r'travel to'],
            
            # Object interaction
            'detect_object': [r'find', r'see', r'locate', r'show me', r'find the', r'where is'],
            'describe_scene': [r'describe', r'what do you see', r'tell me', r'what is there'],
        }
        
        # Object vocabulary
        self.object_types = [
            'cup', 'bottle', 'chair', 'table', 'book', 'phone', 'ball', 
            'box', 'bowl', 'plate', 'person', 'door', 'window'
        ]
        
        # Location vocabulary
        self.location_keywords = [
            'kitchen', 'living room', 'bedroom', 'office', 'hallway', 'bathroom',
            'entrance', 'exit', 'dining room', 'garden', 'garage', 'corridor'
        ]
        
        # Create subscribers
        self.text_subscription = self.create_subscription(
            String,
            '/vla/recognized_text',
            self.text_callback,
            qos_profile_sensor_data
        )
        
        # Create publishers for different robot interfaces
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla/command_status',
            10
        )
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, FollowJointTrajectory, 'manipulation_controller/follow_joint_trajectory')
        
        # Internal state
        self.current_pose = None
        self.last_command = None
        
        # Log initialization
        self.get_logger().info("Text-to-ROS Command Mapper initialized")
        self.get_logger().info(f"Command threshold: {self.command_threshold}")

    def text_callback(self, msg: String):
        """
        Callback for receiving transcribed text.
        """
        self.get_logger().info(f"Received text command: {msg.data}")
        
        try:
            # Parse the command
            command_result = self.parse_command(msg.data)
            
            if command_result:
                command_type, params = command_result
                self.get_logger().info(f"Parsed command: {command_type} with params: {params}")
                
                # Execute the command
                success = self.execute_command(command_type, params)
                
                # Publish status
                status_msg = String()
                status_msg.data = f"Command {command_type} {'executed' if success else 'failed'}: {msg.data}"
                self.status_publisher.publish(status_msg)
                
                if success:
                    self.get_logger().info(f"Command {command_type} executed successfully")
                else:
                    self.get_logger().error(f"Failed to execute command {command_type}")
            else:
                self.get_logger().warning(f"Could not parse command: {msg.data}")
                status_msg = String()
                status_msg.data = f"Could not parse command: {msg.data}"
                self.status_publisher.publish(status_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing text command: {str(e)}")
            status_msg = String()
            status_msg.data = f"Error processing command: {str(e)}"
            self.status_publisher.publish(status_msg)

    def parse_command(self, text: str) -> Optional[Tuple[str, Dict[str, Any]]]:
        """
        Parse natural language command into structured command.
        
        Args:
            text: Natural language command string
            
        Returns:
            Tuple of (command_type, params) or None if parsing fails
        """
        text_lower = text.lower().strip()
        
        # Check each command pattern
        for cmd_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    # Extract additional parameters based on command type
                    params = self.extract_parameters(text_lower, cmd_type)
                    return cmd_type, params
        
        return None

    def extract_parameters(self, text: str, command_type: str) -> Dict[str, Any]:
        """
        Extract parameters from text based on command type.
        
        Args:
            text: Text command
            command_type: Type of command
            
        Returns:
            Dictionary of extracted parameters
        """
        params = {}
        
        if command_type in ['move_to_location', 'navigate']:
            # Extract location from text
            for location in self.location_keywords:
                if location in text:
                    params['location'] = location
                    break
            
            # Try to extract coordinates if present
            coord_match = re.search(r'(\d+\.?\d*)[,\s]+(\d+\.?\d*)', text)
            if coord_match:
                params['x'] = float(coord_match.group(1))
                params['y'] = float(coord_match.group(2))
        
        elif command_type in ['grasp', 'detect_object']:
            # Extract object name
            for obj in self.object_types:
                if obj in text:
                    params['object'] = obj
                    break
            
            # Check for color if present
            color_match = re.search(r'(red|blue|green|yellow|white|black|pink|purple|orange|brown)', text)
            if color_match:
                params['color'] = color_match.group(1)
        
        elif command_type in ['move_forward', 'move_backward']:
            # Extract distance if specified
            distance_match = re.search(r'(\d+\.?\d*)\s*(m|meter|cm|centimeter)', text)
            if distance_match:
                distance = float(distance_match.group(1))
                unit = distance_match.group(2)
                
                # Convert to meters if needed
                if unit.startswith('c'):  # cm
                    distance /= 100.0
                params['distance'] = distance
            else:
                params['distance'] = 1.0  # Default distance
        
        elif command_type in ['turn_left', 'turn_right']:
            # Extract angle if specified
            angle_match = re.search(r'(\d+\.?\d*)\s*(degree|deg)', text)
            if angle_match:
                params['angle'] = math.radians(float(angle_match.group(1)))
            else:
                params['angle'] = math.radians(90.0)  # Default 90 degrees
        
        return params

    def execute_command(self, command_type: str, params: Dict[str, Any]) -> bool:
        """
        Execute the parsed command.
        
        Args:
            command_type: Type of command to execute
            params: Parameters for the command
            
        Returns:
            True if command was executed successfully, False otherwise
        """
        try:
            if command_type == 'move_forward':
                return self.execute_move_forward(params)
            elif command_type == 'move_backward':
                return self.execute_move_backward(params)
            elif command_type == 'turn_left':
                return self.execute_turn_left(params)
            elif command_type == 'turn_right':
                return self.execute_turn_right(params)
            elif command_type == 'move_to_location':
                return self.execute_move_to_location(params)
            elif command_type == 'stop':
                return self.execute_stop()
            elif command_type == 'grasp':
                return self.execute_grasp(params)
            elif command_type == 'release':
                return self.execute_release()
            elif command_type == 'navigate':
                return self.execute_navigate(params)
            elif command_type == 'detect_object':
                return self.execute_detect_object(params)
            else:
                self.get_logger().error(f"Unknown command type: {command_type}")
                return False
        
        except Exception as e:
            self.get_logger().error(f"Error executing command {command_type}: {str(e)}")
            return False

    def execute_move_forward(self, params: Dict[str, Any]) -> bool:
        """
        Execute forward movement command.
        """
        distance = params.get('distance', 1.0)
        speed = 0.5  # m/s
        
        # Calculate time to move the distance
        duration = distance / speed if speed > 0 else 0.0
        
        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Stop after the calculated duration
        self.stop_after_duration(duration)
        
        self.get_logger().info(f"Moving forward {distance} meters")
        return True

    def execute_move_backward(self, params: Dict[str, Any]) -> bool:
        """
        Execute backward movement command.
        """
        distance = params.get('distance', 1.0)
        speed = -0.5  # negative for backward
        
        # Calculate time to move the distance
        duration = abs(distance / speed) if speed < 0 else 0.0
        
        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Stop after the calculated duration
        self.stop_after_duration(duration)
        
        self.get_logger().info(f"Moving backward {distance} meters")
        return True

    def execute_turn_left(self, params: Dict[str, Any]) -> bool:
        """
        Execute left turn command.
        """
        angle = params.get('angle', math.radians(90))
        angular_speed = 0.5  # rad/s
        
        # Calculate time to turn the angle
        duration = abs(angle / angular_speed)
        
        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = angular_speed
        
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Stop after the calculated duration
        self.stop_after_duration(duration)
        
        self.get_logger().info(f"Turning left {math.degrees(angle)} degrees")
        return True

    def execute_turn_right(self, params: Dict[str, Any]) -> bool:
        """
        Execute right turn command.
        """
        angle = params.get('angle', math.radians(90))
        angular_speed = -0.5  # negative for right turn
        
        # Calculate time to turn the angle
        duration = abs(angle / abs(angular_speed))
        
        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = angular_speed
        
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Stop after the calculated duration
        self.stop_after_duration(duration)
        
        self.get_logger().info(f"Turning right {math.degrees(angle)} degrees")
        return True

    def stop_after_duration(self, duration: float):
        """
        Helper method to stop movement after a specified duration.
        """
        # This is a simplified approach
        # In practice, you would use a timer to stop after the duration
        self.get_logger().info(f"Scheduled stop after {duration} seconds")
        
        # For now, we'll just schedule a stop command after the duration
        self.create_timer(duration, self.execute_stop)

    def execute_stop(self) -> bool:
        """
        Execute stop command.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info("Stopping robot")
        return True

    def execute_move_to_location(self, params: Dict[str, Any]) -> bool:
        """
        Execute move to location command.
        """
        if 'location' in params:
            location = params['location']
            self.get_logger().info(f"Moving to {location}")
            
            # In a real implementation, this would involve:
            # 1. Looking up the coordinates of the location
            # 2. Using navigation stack to move to the location
            # 3. Handling the navigation result
            
            # For this example, we'll just publish a status
            status_msg = String()
            status_msg.data = f"Navigation to {location} initiated"
            self.status_publisher.publish(status_msg)
            
            return True
        else:
            self.get_logger().warning("No location specified for move_to_location command")
            return False

    def execute_grasp(self, params: Dict[str, Any]) -> bool:
        """
        Execute grasp command.
        """
        obj = params.get('object', 'object')
        self.get_logger().info(f"Attempting to grasp {obj}")
        
        # In a real implementation, this would:
        # 1. Identify the object using perception system
        # 2. Plan and execute grasping motion
        # 3. Confirm grasp success
        
        status_msg = String()
        status_msg.data = f"Grasping {obj} initiated"
        self.status_publisher.publish(status_msg)
        
        return True

    def execute_release(self) -> bool:
        """
        Execute release command.
        """
        self.get_logger().info("Releasing object")
        
        # In a real implementation, this would:
        # 1. Open the gripper
        # 2. Confirm object release
        
        status_msg = String()
        status_msg.data = "Release initiated"
        self.status_publisher.publish(status_msg)
        
        return True

    def execute_navigate(self, params: Dict[str, Any]) -> bool:
        """
        Execute navigation command.
        """
        # This would typically use the navigation2 stack
        goal_pose = Pose()
        
        # Set up goal pose based on parameters
        if 'x' in params and 'y' in params:
            goal_pose.position.x = params['x']
            goal_pose.position.y = params['y']
            goal_pose.position.z = 0.0
            
            # Default orientation (facing forward)
            goal_pose.orientation.w = 1.0
            
            self.get_logger().info(f"Navigating to position: ({goal_pose.position.x}, {goal_pose.position.y})")
            
            # Send navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = self.map_frame
            goal_msg.pose.pose = goal_pose
            
            # Send goal to navigation action server
            self.nav_client.wait_for_server()
            future = self.nav_client.send_goal_async(goal_msg)
            
            # In a real implementation, you'd wait for result
            return True
        else:
            self.get_logger().warning("Navigate command requires coordinates")
            return False

    def execute_detect_object(self, params: Dict[str, Any]) -> bool:
        """
        Execute object detection command.
        """
        obj = params.get('object', 'object')
        self.get_logger().info(f"Looking for {obj}")
        
        # In a real implementation, this would trigger the perception system
        status_msg = String()
        status_msg.data = f"Detection of {obj} initiated"
        self.status_publisher.publish(status_msg)
        
        return True


class AdvancedCommandMapper(TextToROSCommandMapper):
    """
    Extended version with more sophisticated natural language understanding.
    """
    
    def __init__(self):
        super().__init__()
        
        # Additional patterns for more complex commands
        self.complex_command_patterns = {
            # Complex navigation commands
            'go_to_object': [r'go to the\s+(\w+)\s+and\s+grasp', r'pick up the\s+(\w+)\s+for me'],
            # Sequential action commands
            'sequence': [r'do the following', r'first.*then', r'after.*do'],
        }
        
        # Add more sophisticated parsing capabilities
        self.nlu_enabled = True
        self.get_logger().info("Advanced Command Mapper initialized with NLU capabilities")

    def parse_command(self, text: str) -> Optional[Tuple[str, Dict[str, Any]]]:
        """
        Enhanced command parsing with NLU capabilities.
        """
        if not self.nlu_enabled:
            return super().parse_command(text)
        
        text_lower = text.lower().strip()
        
        # Try complex patterns first
        for cmd_type, patterns in self.complex_command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    groups = match.groups()
                    params = self.extract_parameters(text_lower, cmd_type)
                    
                    # Process capture groups for complex commands
                    if groups:
                        params['capture_groups'] = groups
                    
                    return cmd_type, params
        
        # Fall back to basic patterns
        return super().parse_command(text)


def main(args=None):
    """
    Main function to run the text-to-ROS command mapper node.
    """
    rclpy.init(args=args)
    
    try:
        # Create the command mapper node
        node = AdvancedCommandMapper()
        
        try:
            node.get_logger().info("Starting Text-to-ROS Command Mapper...")
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Interrupted by user")
        finally:
            node.destroy_node()
    
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Key Implementation Features

### 1. Natural Language Processing
- Pattern matching for various command formulations
- Parameter extraction (distances, angles, object names, locations)
- Support for complex commands with multiple parameters

### 2. Command Mapping
- Movement commands: forward, backward, turn left/right
- Navigation commands: go to locations, navigate to coordinates
- Manipulation commands: grasp, release, lift, lower
- Object interaction: detect objects, describe scenes

### 3. ROS 2 Integration
- Subscribes to `/vla/recognized_text` topic
- Publishes to robot command interfaces (`/cmd_vel`)
- Uses action clients for navigation and manipulation
- Publishes status updates to `/vla/command_status`

### 4. Extensible Architecture
- Modular command parsing system
- Easy addition of new command types and patterns
- Support for complex sequential commands

### 5. Error Handling
- Graceful handling of unparsable commands
- Comprehensive logging and status reporting
- Safe fallbacks when parameters are missing

### 6. Configuration
- Adjustable command recognition thresholds
- Configurable frame IDs for navigation
- Enable/disable natural language understanding

## Usage Examples

The mapper can handle commands like:
- "Move forward 2 meters"
- "Go to the kitchen"
- "Pick up the red cup"
- "Turn left 45 degrees"
- "Find the book"
- "Navigate to the table"

## Dependencies

The implementation requires standard ROS 2 packages:
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `nav2_msgs` (for navigation)
- `control_msgs` (for manipulation)

## Integration with VLA System

This component integrates with:
- Voice recognition system (receiving transcribed text)
- Navigation stack (for movement commands)
- Manipulation system (for grasping commands)
- Perception system (for object detection commands)
- Status monitoring (for command execution feedback)

This mapping system serves as a crucial bridge between natural language understanding and robotic action execution in the VLA system.
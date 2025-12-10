# Exercises: Voice Command Implementation

This section provides exercises to practice implementing and working with voice command systems in the VLA framework, with detailed solutions for each exercise.

## Exercise 1: Basic Voice Recognition Node

### Objective
Create a basic ROS 2 node that subscribes to audio input and publishes recognized text.

### Task
Implement a ROS 2 node that:
1. Subscribes to an audio input topic
2. Processes the audio to extract text using a placeholder function
3. Publishes the extracted text to a text output topic
4. Implements basic error handling

### Starter Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData

class BasicVoiceRecognizer(Node):
    def __init__(self):
        super().__init__('basic_voice_recognizer')
        
        # TODO: Create subscriber for audio input
        # TODO: Create publisher for text output
        # TODO: Implement audio processing callback
        
    def audio_callback(self, msg):
        """
        Process incoming audio and publish recognized text
        """
        # TODO: Extract text from audio (use mock function for now)
        # TODO: Publish recognized text
        pass

def main(args=None):
    rclpy.init(args=args)
    # TODO: Create and run the node
    rclpy.shutdown()
```

### Solution
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData

class BasicVoiceRecognizer(Node):
    def __init__(self):
        super().__init__('basic_voice_recognizer')
        
        # Create subscriber for audio input
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )
        
        # Create publisher for text output
        self.text_pub = self.create_publisher(
            String,
            '/recognized_text',
            10
        )
        
        self.get_logger().info("Basic Voice Recognizer Node Started")
    
    def audio_callback(self, msg):
        """
        Process incoming audio and publish recognized text
        """
        try:
            # For this exercise, we'll simulate recognition with a placeholder
            # In real implementation, this would call an ASR service
            recognized_text = self.simulate_recognition(msg)
            
            if recognized_text:
                # Publish recognized text
                text_msg = String()
                text_msg.data = recognized_text
                self.text_pub.publish(text_msg)
                self.get_logger().info(f"Recognized: {recognized_text}")
            else:
                self.get_logger().warning("No text recognized from audio")
                
        except Exception as e:
            self.get_logger().error(f"Error in audio processing: {str(e)}")
    
    def simulate_recognition(self, audio_msg):
        """
        Simulate audio recognition (placeholder function)
        """
        # In a real implementation, this would:
        # 1. Convert audio_msg.data to a format for ASR
        # 2. Call ASR service (like OpenAI Whisper)
        # 3. Return recognized text
        
        # For simulation, return a mock text if audio has data
        if len(audio_msg.data) > 0:
            # Return a mock recognition based on audio data length
            # (simulating that longer audio might contain more words)
            word_count = min(len(audio_msg.data) // 1000, 5)  # Max 5 words
            mock_words = ["hello", "world", "robot", "move", "forward"]
            return " ".join(mock_words[:word_count]) if word_count > 0 else "no speech detected"
        
        return "no audio data"

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BasicVoiceRecognizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Exercise 2: Command Mapping Implementation

### Objective
Implement a command mapping system that converts recognized text to robot commands.

### Task
Create a command mapper that:
1. Takes recognized text as input
2. Maps natural language to specific robot commands
3. Extracts parameters from the text
4. Returns structured command objects

### Starter Code
```python
from typing import Dict, Any, Optional

class CommandMapper:
    def __init__(self):
        # TODO: Define command patterns
        pass
    
    def map_command(self, text: str) -> Optional[Dict[str, Any]]:
        """
        Map text command to structured command
        """
        # TODO: Implement command mapping logic
        pass
    
    def extract_parameters(self, text: str, command_type: str) -> Dict[str, Any]:
        """
        Extract parameters from text based on command type
        """
        # TODO: Implement parameter extraction
        pass

# Example usage
mapper = CommandMapper()
result = mapper.map_command("move forward 2 meters")
print(result)  # Should return something like {'action': 'move_forward', 'params': {'distance': 2.0}}
```

### Solution
```python
import re
from typing import Dict, Any, Optional

class CommandMapper:
    def __init__(self):
        # Define command patterns with regular expressions
        self.command_patterns = {
            'move_forward': [r'move forward', r'go forward', r'straight', r'forward'],
            'move_backward': [r'move backward', r'go backward', r'backward', r'back'],
            'turn_left': [r'turn left', r'rotate left', r'left'],
            'turn_right': [r'turn right', r'rotate right', r'right'],
            'grasp': [r'pick up', r'grasp', r'grab', r'take'],
            'release': [r'release', r'drop', r'let go'],
            'stop': [r'stop', r'hold', r'freeze']
        }
        
        # Unit conversion factors
        self.unit_factors = {
            'm': 1.0, 'meter': 1.0, 'meters': 1.0,
            'cm': 0.01, 'centimeter': 0.01, 'centimeters': 0.01,
            'mm': 0.001, 'millimeter': 0.001, 'millimeters': 0.001,
            'in': 0.0254, 'inch': 0.0254, 'inches': 0.0254,
            'ft': 0.3048, 'foot': 0.3048, 'feet': 0.3048
        }
    
    def map_command(self, text: str) -> Optional[Dict[str, Any]]:
        """
        Map text command to structured command
        """
        text_lower = text.lower().strip()
        
        # Check each command type
        for action_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    # Extract parameters for this command
                    params = self.extract_parameters(text_lower, action_type)
                    
                    return {
                        'action': action_type,
                        'parameters': params
                    }
        
        # No match found
        return None
    
    def extract_parameters(self, text: str, command_type: str) -> Dict[str, Any]:
        """
        Extract parameters from text based on command type
        """
        params = {}
        
        # Extract numeric values with units
        if command_type in ['move_forward', 'move_backward']:
            # Look for distance patterns like "2 meters" or "50 cm"
            distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(\w+)', text)
            if distance_match:
                value = float(distance_match.group(1))
                unit = distance_match.group(2)
                
                # Convert to meters if unit is recognized
                if unit in self.unit_factors:
                    params['distance_meters'] = value * self.unit_factors[unit]
                    params['distance_original'] = f"{value}{unit}"
                else:
                    # Assume meters if no unit specified
                    params['distance_meters'] = value
        
        elif command_type in ['turn_left', 'turn_right']:
            # Look for angle patterns like "90 degrees"
            angle_match = re.search(r'(\d+(?:\.\d+)?)\s*deg(?:ree)?s?', text, re.IGNORECASE)
            if angle_match:
                angle_deg = float(angle_match.group(1))
                params['angle_degrees'] = angle_deg
                params['angle_radians'] = angle_deg * 3.14159 / 180.0
        
        elif command_type in ['grasp', 'release']:
            # Extract object information
            # Look for color and object patterns like "red cup" or "blue ball"
            color_obj_match = re.search(r'(red|blue|green|yellow|white|black|pink|purple|orange|brown)\s+(\w+)', text)
            if color_obj_match:
                params['color'] = color_obj_match.group(1)
                params['object'] = color_obj_match.group(2)
            else:
                # Try to get just the object
                obj_match = re.search(r'(cup|ball|book|box|chair|table|phone|bottle)', text)
                if obj_match:
                    params['object'] = obj_match.group(1)
        
        return params

# Example usage and testing
def test_command_mapper():
    mapper = CommandMapper()
    
    test_cases = [
        "Move forward 2 meters",
        "Turn left 90 degrees", 
        "Pick up the red cup",
        "Go backward 50 centimeters",
        "Stop immediately",
        "Grasp the blue ball"
    ]
    
    print("Command Mapping Test Results:")
    print("=" * 40)
    
    for text in test_cases:
        result = mapper.map_command(text)
        print(f"Input: {text}")
        print(f"Output: {result}")
        print("-" * 30)

if __name__ == "__main__":
    test_command_mapper()
```

---

## Exercise 3: Voice Command Pipeline Integration

### Objective
Integrate the voice recognition and command mapping components into a single pipeline node.

### Task
Create a ROS 2 node that:
1. Subscribes to recognized text
2. Maps the text to robot commands
3. Publishes the commands to appropriate robot interfaces
4. Implements a simple state machine for command processing

### Starter Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VoiceCommandPipeline(Node):
    def __init__(self):
        super().__init__('voice_command_pipeline')
        
        # TODO: Create subscriber for recognized text
        # TODO: Create publishers for robot commands
        # TODO: Initialize command mapper
        # TODO: Implement state management
        
    def text_callback(self, msg):
        """
        Process recognized text and execute corresponding command
        """
        # TODO: Map text to command
        # TODO: Execute command
        pass
    
    def execute_command(self, command):
        """
        Execute the mapped command
        """
        # TODO: Implement command execution based on action type
        pass

def main(args=None):
    rclpy.init(args=args)
    # TODO: Create and run the pipeline node
    rclpy.shutdown()
```

### Solution
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import time

class VoiceCommandPipeline(Node):
    def __init__(self):
        super().__init__('voice_command_pipeline')
        
        # Create subscriber for recognized text
        self.text_sub = self.create_subscription(
            String,
            '/vla/recognized_text',
            self.text_callback,
            10
        )
        
        # Create publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create publisher for system status
        self.status_pub = self.create_publisher(
            String,
            '/vla/status',
            10
        )
        
        # Initialize command mapper
        self.command_mapper = CommandMapper()
        
        # Initialize state
        self.is_executing = False
        self.last_command_time = time.time()
        
        self.get_logger().info("Voice Command Pipeline Node Started")
    
    def text_callback(self, msg):
        """
        Process recognized text and execute corresponding command
        """
        if self.is_executing:
            self.get_logger().warning("Command still executing, ignoring new command")
            return
        
        text = msg.data
        self.get_logger().info(f"Processing command: {text}")
        
        # Map text to command
        command = self.command_mapper.map_command(text)
        
        if command:
            self.get_logger().info(f"Mapped to: {command['action']} with {command['parameters']}")
            
            # Execute the command
            success = self.execute_command(command)
            
            if success:
                self.get_logger().info(f"Command executed: {text}")
                self.status_pub.publish(String(data=f"command_executed: {text}"))
            else:
                self.get_logger().error(f"Command failed: {text}")
                self.status_pub.publish(String(data=f"command_failed: {text}"))
        else:
            self.get_logger().warning(f"Unrecognized command: {text}")
            self.status_pub.publish(String(data=f"command_unrecognized: {text}"))
    
    def execute_command(self, command):
        """
        Execute the mapped command based on action type
        """
        action = command['action']
        params = command['parameters']
        
        try:
            if action == 'move_forward':
                return self.execute_move_forward(params)
            elif action == 'move_backward':
                return self.execute_move_backward(params)
            elif action == 'turn_left':
                return self.execute_turn_left(params)
            elif action == 'turn_right':
                return self.execute_turn_right(params)
            elif action == 'stop':
                return self.execute_stop()
            elif action in ['grasp', 'release']:
                # These would require a manipulator interface in real hardware
                self.get_logger().info(f"Manipulation command: {action}, params: {params}")
                return True
            else:
                self.get_logger().error(f"Unknown action: {action}")
                return False
        
        except Exception as e:
            self.get_logger().error(f"Error executing command {action}: {str(e)}")
            return False
    
    def execute_move_forward(self, params):
        """
        Execute forward movement
        """
        distance = params.get('distance_meters', 1.0)  # Default 1 meter
        speed = 0.5  # m/s
        
        # Calculate duration to move the specified distance
        duration = distance / speed if speed > 0 else 0
        
        return self.move_with_duration(speed, 0, duration)
    
    def execute_move_backward(self, params):
        """
        Execute backward movement
        """
        distance = params.get('distance_meters', 1.0)  # Default 1 meter
        speed = -0.5  # Negative for backward
        
        # Calculate duration to move the specified distance
        duration = abs(distance / speed) if speed != 0 else 0
        
        return self.move_with_duration(speed, 0, duration)
    
    def execute_turn_left(self, params):
        """
        Execute left turn
        """
        angle_rad = params.get('angle_radians', 1.57)  # Default 90 degrees
        angular_speed = 0.5  # rad/s
        
        # Calculate duration to turn the specified angle
        duration = abs(angle_rad / angular_speed) if angular_speed != 0 else 0
        
        return self.move_with_duration(0, angular_speed, duration)
    
    def execute_turn_right(self, params):
        """
        Execute right turn
        """
        angle_rad = params.get('angle_radians', 1.57)  # Default 90 degrees
        angular_speed = -0.5  # Negative for right turn
        
        # Calculate duration to turn the specified angle
        duration = abs(angle_rad / abs(angular_speed)) if angular_speed != 0 else 0
        
        return self.move_with_duration(0, angular_speed, duration)
    
    def execute_stop(self):
        """
        Stop the robot immediately
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped")
        return True
    
    def move_with_duration(self, linear_speed, angular_speed, duration):
        """
        Move robot with specified speeds for given duration
        """
        if duration <= 0:
            return True
        
        self.is_executing = True
        self.get_logger().info(f"Moving with linear:{linear_speed}, angular:{angular_speed} for {duration}s")
        
        # Publish movement command for the specified duration
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # 10 Hz update rate
        
        # Stop the robot after movement
        self.execute_stop()
        
        # Reset execution state
        self.is_executing = False
        self.last_command_time = time.time()
        
        return True

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VoiceCommandPipeline()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Voice Command Pipeline")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Exercise 4: Error Handling and Robustness

### Objective
Enhance the voice command pipeline with comprehensive error handling and robustness features.

### Task
Add the following features to your pipeline:
1. Graceful handling of missing or invalid commands
2. Recovery mechanisms for failed commands
3. Logging and monitoring of errors
4. Timeout and retry logic

### Starter Code
```python
# Extend the VoiceCommandPipeline from Exercise 3 with error handling
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobustVoiceCommandPipeline(Node):
    def __init__(self):
        super().__init__('robust_voice_command_pipeline')
        
        # TODO: Add error tracking variables
        # TODO: Add retry mechanism
        # TODO: Add timeout handling
        
    def text_callback(self, msg):
        """
        Process text with enhanced error handling
        """
        # TODO: Implement with error handling
        pass
    
    def execute_command_with_retry(self, command, max_retries=3):
        """
        Execute command with retry mechanism
        """
        # TODO: Implement retry logic
        pass
    
    def log_error(self, error_msg, command):
        """
        Log errors with context
        """
        # TODO: Implement error logging
        pass
```

### Solution
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import time
import json
from collections import deque

class RobustVoiceCommandPipeline(Node):
    def __init__(self):
        super().__init__('robust_voice_command_pipeline')
        
        # Create subscriber for recognized text
        self.text_sub = self.create_subscription(
            String,
            '/vla/recognized_text',
            self.text_callback,
            10
        )
        
        # Create publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create publisher for system status
        self.status_pub = self.create_publisher(
            String,
            '/vla/status',
            10
        )
        
        # Create publisher for error reports
        self.error_pub = self.create_publisher(
            String,
            '/vla/error_report',
            10
        )
        
        # Initialize command mapper
        self.command_mapper = CommandMapper()
        
        # Initialize state
        self.is_executing = False
        self.current_command = None
        self.command_start_time = None
        self.command_timeout = 30.0  # seconds
        self.max_retries = 3
        
        # Error tracking
        self.error_count = 0
        self.error_history = deque(maxlen=20)  # Keep last 20 errors
        
        # Command retry tracking
        self.retry_count = 0
        self.failed_command = None
        
        # Status timer to monitor command execution
        self.status_timer = self.create_timer(1.0, self.monitor_execution)
        
        self.get_logger().info("Robust Voice Command Pipeline Node Started")
    
    def text_callback(self, msg):
        """
        Process text with enhanced error handling
        """
        if self.is_executing:
            self.get_logger().warning("Command still executing, queuing new command")
            # In a more sophisticated system, you might queue commands
            # For this exercise, we'll just warn and return
            return
        
        text = msg.data
        self.get_logger().info(f"Processing command: {text}")
        
        try:
            # Map text to command
            command = self.command_mapper.map_command(text)
            
            if command:
                self.get_logger().info(f"Mapped to: {command['action']} with {command['parameters']}")
                
                # Execute the command with retry mechanism
                success = self.execute_command_with_retry(command)
                
                if success:
                    self.get_logger().info(f"Command executed successfully: {text}")
                    self.status_pub.publish(String(data=f"command_executed: {text}"))
                    # Reset retry counter on success
                    self.retry_count = 0
                    self.failed_command = None
                else:
                    self.get_logger().error(f"Command execution failed: {text}")
                    self.status_pub.publish(String(data=f"command_failed: {text}"))
                    self.handle_command_failure(command, text)
            else:
                self.get_logger().warning(f"Unrecognized command: {text}")
                error_report = {
                    'type': 'unrecognized_command',
                    'command': text,
                    'timestamp': time.time()
                }
                self.publish_error(error_report)
                self.status_pub.publish(String(data=f"command_unrecognized: {text}"))
        
        except Exception as e:
            self.get_logger().error(f"Error processing command '{text}': {str(e)}")
            error_report = {
                'type': 'processing_error',
                'command': text,
                'error': str(e),
                'timestamp': time.time()
            }
            self.publish_error(error_report)
            self.status_pub.publish(String(data=f"processing_error: {str(e)}"))
    
    def execute_command_with_retry(self, command, max_retries=None):
        """
        Execute command with retry mechanism
        """
        if max_retries is None:
            max_retries = self.max_retries
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"Execution attempt {attempt + 1}/{max_retries}")
                
                # Set execution state
                self.is_executing = True
                self.current_command = command
                self.command_start_time = time.time()
                
                # Execute the command
                success = self.execute_command(command)
                
                # Reset execution state
                self.is_executing = False
                self.current_command = None
                self.command_start_time = None
                
                if success:
                    self.get_logger().info(f"Command executed successfully on attempt {attempt + 1}")
                    return True
                
                self.get_logger().warning(f"Command failed on attempt {attempt + 1}")
                
                if attempt < max_retries - 1:
                    # Wait before retry
                    time.sleep(1.0 * (attempt + 1))  # Exponential backoff
            
            except Exception as e:
                self.get_logger().error(f"Exception on attempt {attempt + 1}: {str(e)}")
                
                # Reset execution state
                self.is_executing = False
                self.current_command = None
                self.command_start_time = None
                
                if attempt == max_retries - 1:
                    self.log_error(f"Command failed after {max_retries} attempts: {str(e)}", command)
        
        # All retries failed
        return False
    
    def execute_command(self, command):
        """
        Execute the mapped command with error handling
        """
        action = command['action']
        params = command['parameters']
        
        try:
            if action == 'move_forward':
                return self.execute_move_forward(params)
            elif action == 'move_backward':
                return self.execute_move_backward(params)
            elif action == 'turn_left':
                return self.execute_turn_left(params)
            elif action == 'turn_right':
                return self.execute_turn_right(params)
            elif action == 'stop':
                return self.execute_stop()
            elif action in ['grasp', 'release']:
                # These would require a manipulator interface in real hardware
                self.get_logger().info(f"Manipulation command: {action}, params: {params}")
                return True
            else:
                self.get_logger().error(f"Unknown action: {action}")
                return False
        
        except Exception as e:
            self.get_logger().error(f"Error executing action {action}: {str(e)}")
            return False
    
    def execute_move_forward(self, params):
        """
        Execute forward movement with timeout
        """
        distance = params.get('distance_meters', 1.0)  # Default 1 meter
        speed = 0.5  # m/s
        
        # Calculate duration to move the specified distance
        duration = distance / speed if speed > 0 else 0
        return self.move_with_duration_and_timeout(speed, 0, duration)
    
    def execute_move_backward(self, params):
        """
        Execute backward movement with timeout
        """
        distance = params.get('distance_meters', 1.0)  # Default 1 meter
        speed = -0.5  # Negative for backward
        
        # Calculate duration to move the specified distance
        duration = abs(distance / speed) if speed != 0 else 0
        return self.move_with_duration_and_timeout(speed, 0, duration)
    
    def execute_turn_left(self, params):
        """
        Execute left turn with timeout
        """
        angle_rad = params.get('angle_radians', 1.57)  # Default 90 degrees
        angular_speed = 0.5  # rad/s
        
        # Calculate duration to turn the specified angle
        duration = abs(angle_rad / angular_speed) if angular_speed != 0 else 0
        return self.move_with_duration_and_timeout(0, angular_speed, duration)
    
    def execute_turn_right(self, params):
        """
        Execute right turn with timeout
        """
        angle_rad = params.get('angle_radians', 1.57)  # Default 90 degrees
        angular_speed = -0.5  # Negative for right turn
        
        # Calculate duration to turn the specified angle
        duration = abs(angle_rad / abs(angular_speed)) if angular_speed != 0 else 0
        return self.move_with_duration_and_timeout(0, angular_speed, duration)
    
    def execute_stop(self):
        """
        Stop the robot immediately
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped")
        return True
    
    def move_with_duration_and_timeout(self, linear_speed, angular_speed, duration):
        """
        Move robot with specified speeds for given duration, with timeout
        """
        if duration <= 0:
            return True
        
        self.is_executing = True
        self.get_logger().info(f"Moving with linear:{linear_speed}, angular:{angular_speed} for {duration}s")
        
        # Set timeout based on duration with a safety margin
        timeout = duration + 5.0  # Add 5 seconds safety margin
        
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            # Check for timeout
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Movement command timed out after {timeout}s")
                self.execute_stop()
                return False
            
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # 10 Hz update rate
        
        # Stop the robot after movement
        self.execute_stop()
        self.is_executing = False
        
        return True
    
    def monitor_execution(self):
        """
        Monitor command execution for timeouts
        """
        if self.is_executing and self.command_start_time:
            elapsed_time = time.time() - self.command_start_time
            
            if elapsed_time > self.command_timeout:
                self.get_logger().error(f"Command timed out: {self.current_command}")
                
                # Stop execution and report timeout
                self.execute_stop()
                self.is_executing = False
                
                error_report = {
                    'type': 'timeout',
                    'command': str(self.current_command),
                    'elapsed_time': elapsed_time,
                    'timestamp': time.time()
                }
                self.publish_error(error_report)
    
    def handle_command_failure(self, command, original_text):
        """
        Handle failed command execution
        """
        self.error_count += 1
        
        error_report = {
            'type': 'command_failure',
            'command': original_text,
            'mapped_command': command,
            'attempt': self.retry_count + 1,
            'timestamp': time.time()
        }
        self.publish_error(error_report)
        
        # Store in history
        self.error_history.append(error_report)
    
    def log_error(self, error_msg, command):
        """
        Log errors with context
        """
        error_report = {
            'type': 'execution_error',
            'error': error_msg,
            'command': str(command),
            'timestamp': time.time()
        }
        self.publish_error(error_report)
        
        # Store in history
        self.error_history.append(error_report)
    
    def publish_error(self, error_report):
        """
        Publish error report to the error topic
        """
        error_msg = String()
        error_msg.data = json.dumps(error_report)
        self.error_pub.publish(error_msg)
        
        # Log to console as well
        self.get_logger().error(f"Error report: {error_report}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobustVoiceCommandPipeline()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Robust Voice Command Pipeline")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Exercise 5: Performance Monitoring and Optimization

### Objective
Add performance monitoring to your voice command system to track metrics like response time, accuracy, and error rates.

### Task
Implement a monitoring system that:
1. Tracks command processing times
2. Monitors system performance metrics
3. Provides statistics and reporting
4. Identifies performance bottlenecks

### Solution
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import statistics
from collections import deque
import json

class MonitoredVoiceCommandPipeline(Node):
    def __init__(self):
        super().__init__('monitored_voice_command_pipeline')
        
        # Create subscriber for recognized text
        self.text_sub = self.create_subscription(
            String,
            '/vla/recognized_text',
            self.text_callback,
            10
        )
        
        # Create publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create publisher for system status
        self.status_pub = self.create_publisher(
            String,
            '/vla/status',
            10
        )
        
        # Create publisher for performance metrics
        self.metrics_pub = self.create_publisher(
            String,
            '/vla/performance_metrics',
            10
        )
        
        # Initialize command mapper
        self.command_mapper = CommandMapper()
        
        # Initialize state
        self.is_executing = False
        self.current_command = None
        
        # Performance monitoring
        self.command_times = deque(maxlen=100)  # Last 100 command times
        self.error_count = 0
        self.success_count = 0
        self.metrics_update_interval = 10.0  # seconds
        self.last_metrics_update = time.time()
        
        # Create timer for periodic metrics reporting
        self.metrics_timer = self.create_timer(5.0, self.report_metrics)
        
        self.get_logger().info("Monitored Voice Command Pipeline Node Started")
    
    def text_callback(self, msg):
        """
        Process text with performance tracking
        """
        start_time = time.time()
        
        if self.is_executing:
            self.get_logger().warning("Command still executing, ignoring new command")
            return
        
        text = msg.data
        self.get_logger().info(f"Processing command: {text}")
        
        try:
            # Map text to command
            map_start = time.time()
            command = self.command_mapper.map_command(text)
            map_time = time.time() - map_start
            
            if command:
                self.get_logger().info(f"Mapped to: {command['action']} with {command['parameters']}")
                
                # Execute the command
                exec_start = time.time()
                success = self.execute_command(command)
                exec_time = time.time() - exec_start
                
                total_time = time.time() - start_time
                
                if success:
                    self.get_logger().info(f"Command executed successfully: {text}")
                    self.success_count += 1
                    self.status_pub.publish(String(data=f"command_executed: {text}"))
                else:
                    self.get_logger().error(f"Command execution failed: {text}")
                    self.error_count += 1
                    self.status_pub.publish(String(data=f"command_failed: {text}"))
                
                # Record performance metrics
                self.record_performance_metrics(total_time, map_time, exec_time, success)
                
            else:
                self.get_logger().warning(f"Unrecognized command: {text}")
                self.error_count += 1
                self.status_pub.publish(String(data=f"command_unrecognized: {text}"))
        
        except Exception as e:
            total_time = time.time() - start_time
            self.get_logger().error(f"Error processing command '{text}': {str(e)}")
            self.error_count += 1
            # Record error in metrics
            self.record_performance_metrics(total_time, 0, 0, False)
    
    def execute_command(self, command):
        """
        Execute the mapped command
        """
        # Simplified execution for this example
        # In a real system, this would contain actual command execution logic
        action = command['action']
        params = command['parameters']
        
        try:
            if action in ['move_forward', 'move_backward', 'turn_left', 'turn_right']:
                # Simulate movement execution
                time.sleep(0.5)  # Simulate execution time
                return True
            elif action == 'stop':
                return True
            elif action in ['grasp', 'release']:
                # Simulate manipulation
                time.sleep(0.3)
                return True
            else:
                self.get_logger().error(f"Unknown action: {action}")
                return False
        
        except Exception as e:
            self.get_logger().error(f"Error executing action {action}: {str(e)}")
            return False
    
    def record_performance_metrics(self, total_time, map_time, exec_time, success):
        """
        Record performance metrics for the command
        """
        self.command_times.append({
            'total_time': total_time,
            'map_time': map_time,
            'exec_time': exec_time,
            'success': success,
            'timestamp': time.time()
        })
    
    def report_metrics(self):
        """
        Report performance metrics
        """
        if not self.command_times:
            return  # No data to report
        
        # Calculate metrics
        total_times = [cmd['total_time'] for cmd in self.command_times]
        map_times = [cmd['map_time'] for cmd in self.command_times]
        exec_times = [cmd['exec_time'] for cmd in self.command_times]
        
        total_count = len(self.command_times)
        successful_count = sum(1 for cmd in self.command_times if cmd['success'])
        success_rate = successful_count / total_count if total_count > 0 else 0
        
        metrics = {
            'timestamp': time.time(),
            'total_commands': total_count,
            'successful_commands': successful_count,
            'success_rate': success_rate,
            'total_errors': self.error_count,
            'avg_total_time': statistics.mean(total_times) if total_times else 0,
            'max_total_time': max(total_times) if total_times else 0,
            'min_total_time': min(total_times) if total_times else 0,
            'std_total_time': statistics.stdev(total_times) if len(total_times) > 1 else 0,
            'avg_mapping_time': statistics.mean(map_times) if map_times else 0,
            'avg_execution_time': statistics.mean(exec_times) if exec_times else 0
        }
        
        # Publish metrics
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics, indent=2)
        self.metrics_pub.publish(metrics_msg)
        
        # Log summary
        self.get_logger().info(
            f"Performance: {success_rate*100:.1f}% success, "
            f"avg time: {metrics['avg_total_time']:.3f}s, "
            f"max time: {metrics['max_total_time']:.3f}s"
        )
        
        # Check for performance issues
        if metrics['avg_total_time'] > 3.0:  # Threshold for concern
            self.get_logger().warning(f"High average command time: {metrics['avg_total_time']:.3f}s")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MonitoredVoiceCommandPipeline()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Monitored Voice Command Pipeline")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

These exercises provide hands-on practice with implementing voice command systems in the VLA framework, covering basic recognition, command mapping, pipeline integration, error handling, and performance monitoring. Each exercise builds on the previous one, creating a comprehensive understanding of voice command implementation in robotics applications.
# Examples and Code Snippets for Voice Command Processing

This document provides practical examples and code snippets demonstrating various aspects of voice command processing in the VLA system.

## 1. Basic Voice Command Processing Example

### Complete Minimal Example

```python
#!/usr/bin/env python3
"""
Minimal example of voice command processing in VLA system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class MinimalVoiceProcessor(Node):
    """
    Minimal voice processing node demonstrating core concepts
    """
    
    def __init__(self):
        super().__init__('minimal_voice_processor')
        
        # Initialize OpenAI
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Create subscribers and publishers
        self.subscription = self.create_subscription(
            String,
            'user_commands',
            self.command_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            String,
            'robot_commands',
            10
        )
        
        self.get_logger().info("Minimal Voice Processor started")
    
    def command_callback(self, msg):
        """
        Process incoming user command
        """
        user_input = msg.data
        self.get_logger().info(f"Received: {user_input}")
        
        # Simple command mapping
        robot_cmd = self.map_command(user_input)
        
        if robot_cmd:
            cmd_msg = String()
            cmd_msg.data = robot_cmd
            self.publisher.publish(cmd_msg)
            self.get_logger().info(f"Published: {robot_cmd}")
    
    def map_command(self, text):
        """
        Simple command mapping function
        """
        text_lower = text.lower()
        
        if "move forward" in text_lower:
            return "MOVE_FORWARD_1M"
        elif "turn left" in text_lower:
            return "TURN_LEFT_90DEG"
        elif "turn right" in text_lower:
            return "TURN_RIGHT_90DEG"
        elif "stop" in text_lower:
            return "STOP"
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = MinimalVoiceProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. Advanced Voice Recognition with Whisper API

### Complete Implementation with Error Handling

```python
#!/usr/bin/env python3
"""
Advanced voice recognition with Whisper API
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import openai
import os
import tempfile
import wave
import threading
from typing import Optional
import time

class AdvancedVoiceRecognizer(Node):
    """
    Advanced voice recognition node with Whisper API integration
    """
    
    def __init__(self):
        super().__init__('advanced_voice_recognizer')
        
        # Initialize OpenAI API
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")
        openai.api_key = api_key
        
        # Configuration
        self.declare_parameter('whisper_model', 'whisper-1')
        self.declare_parameter('confidence_threshold', 0.7)
        
        self.whisper_model = self.get_parameter('whisper_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # Audio buffer for processing
        self.audio_buffer = []
        self.is_listening = False
        
        # Publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/vla/audio_input',
            self.audio_callback,
            10
        )
        
        self.text_pub = self.create_publisher(
            String,
            '/vla/recognized_text',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/vla/recognition_status',
            10
        )
        
        # Timer for processing accumulated audio
        self.process_timer = self.create_timer(2.0, self.process_audio_buffer)
        
        self.get_logger().info("Advanced Voice Recognizer initialized")
    
    def audio_callback(self, msg):
        """
        Handle incoming audio data
        """
        if not self.is_listening:
            return
            
        # Add audio data to buffer
        self.audio_buffer.extend(msg.data)
        self.get_logger().debug(f"Added {len(msg.data)} bytes to audio buffer")
    
    def process_audio_buffer(self):
        """
        Process accumulated audio buffer through Whisper API
        """
        if not self.audio_buffer:
            return
        
        try:
            # Convert audio buffer to WAV format
            wav_path = self.create_wav_file(self.audio_buffer)
            if not wav_path:
                return
            
            # Transcribe using Whisper
            result = self.transcribe_audio(wav_path)
            
            # Clean up temporary file
            os.unlink(wav_path)
            
            if result and result.get("text"):
                text = result["text"].strip()
                
                # Check confidence if available
                confidence = self.calculate_confidence(result)
                
                if confidence >= self.confidence_threshold:
                    # Publish recognized text
                    text_msg = String()
                    text_msg.data = text
                    self.text_pub.publish(text_msg)
                    self.get_logger().info(f"Recognized: {text} (confidence: {confidence:.2f})")
                else:
                    self.get_logger().warning(f"Low confidence recognition: {text} (confidence: {confidence:.2f})")
                    self.status_pub.publish(String(data=f"low_confidence: {text}"))
            else:
                self.get_logger().info("No speech detected in audio buffer")
        
        except Exception as e:
            self.get_logger().error(f"Error processing audio buffer: {str(e)}")
            self.status_pub.publish(String(data=f"error: {str(e)}"))
        
        finally:
            # Clear the audio buffer regardless of success/failure
            self.audio_buffer.clear()
    
    def create_wav_file(self, audio_data):
        """
        Create a temporary WAV file from audio buffer
        """
        try:
            # Create temporary WAV file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                # Write WAV header (assuming 16kHz, 16-bit, mono)
                wav_file = wave.open(temp_file.name, 'wb')
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(16000)  # 16kHz
                wav_file.writeframes(bytes(audio_data))
                wav_file.close()
                
                return temp_file.name
        except Exception as e:
            self.get_logger().error(f"Error creating WAV file: {str(e)}")
            return None
    
    def transcribe_audio(self, wav_path):
        """
        Transcribe audio file using Whisper API
        """
        try:
            with open(wav_path, 'rb') as audio_file:
                result = openai.Audio.transcribe(
                    model=self.whisper_model,
                    file=audio_file,
                    response_format="verbose_json"  # Get detailed response with confidence
                )
            return result
        except Exception as e:
            self.get_logger().error(f"Whisper API error: {str(e)}")
            return None
    
    def calculate_confidence(self, transcription_result):
        """
        Calculate confidence from transcription result
        """
        segments = transcription_result.get("segments", [])
        if not segments:
            return 1.0  # Default to high confidence if no segments available
        
        # Calculate average confidence across segments
        total_confidence = sum(
            segment.get("avg_logprob", -0.5) for segment in segments
        )
        avg_confidence = total_confidence / len(segments) if len(segments) > 0 else 0.0
        
        # Normalize log probability to 0-1 scale (this is a simplified approach)
        # In practice, you might want more sophisticated normalization
        normalized_confidence = max(0.0, min(1.0, avg_confidence + 1.0))
        
        return normalized_confidence

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AdvancedVoiceRecognizer()
        node.is_listening = True  # Start listening when node starts
        
        try:
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

## 3. Command Mapping Examples

### Basic Command Mapping

```python
import re
from typing import Dict, Any, Optional

class SimpleCommandMapper:
    """
    Simple command mapping from recognized text to robot commands
    """
    
    def __init__(self):
        # Define command patterns
        self.movement_patterns = {
            'forward': [r'go forward', r'move forward', r'straight', r'forward'],
            'backward': [r'go backward', r'move backward', r'back', r'backward'],
            'left': [r'turn left', r'rotate left', r'left'],
            'right': [r'turn right', r'rotate right', r'right'],
            'stop': [r'stop', r'hold', r'freeze', r'pause']
        }
        
        self.object_patterns = {
            'grasp': [r'pick up', r'grasp', r'grab', r'take'],
            'release': [r'release', r'drop', r'let go', r'put down']
        }
    
    def map_command(self, text: str) -> Optional[str]:
        """
        Map text command to robot action
        """
        text_lower = text.lower()
        
        # Check movement patterns
        for action, patterns in self.movement_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    # Extract numeric parameters if present
                    distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(m|meter|meters)', text_lower)
                    if distance_match:
                        distance = float(distance_match.group(1))
                        return f"MOVE_{action.upper()}_{distance}M"
                    else:
                        return f"MOVE_{action.upper()}"
        
        # Check object manipulation patterns
        for action, patterns in self.object_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    # Extract object type if present
                    object_match = re.search(r'(cup|ball|book|box)', text_lower)
                    if object_match:
                        obj = object_match.group(1)
                        return f"{action.upper()}_OBJECT_{obj.upper()}"
                    else:
                        return f"{action.upper()}_OBJECT"
        
        return None  # Command not recognized

# Example usage
mapper = SimpleCommandMapper()
print(mapper.map_command("Move forward 2 meters"))  # Output: MOVE_FORWARD_2.0M
print(mapper.map_command("Turn left"))              # Output: MOVE_LEFT
print(mapper.map_command("Pick up the cup"))        # Output: GRASP_OBJECT_CUP
print(mapper.map_command("Not a command"))          # Output: None
```

### Advanced Command Mapping with Parameters

```python
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Any

@dataclass
class ParsedCommand:
    """
    Data class for parsed command with parameters
    """
    action: str
    parameters: Dict[str, Any]
    confidence: float = 1.0

class AdvancedCommandMapper:
    """
    Advanced command mapping with parameter extraction
    """
    
    def __init__(self):
        # Define action patterns with named groups for parameter extraction
        self.patterns = {
            'move': [
                (r'move (?P<direction>forward|backward|left|right)(?: (?P<distance>\d+(?:\.\d+)?)\s*(?P<unit>m|cm|in|ft|meter|centimeter|inch|foot|feet))?', 'movement'),
                (r'go to the (?P<location>\w+)', 'navigation'),
                (r'go (?:over to|toward|near) the (?P<location>\w+)', 'navigation')
            ],
            'grasp': [
                (r'(?P<action>pick up|grasp|grab|take) the (?P<color>\w+)?\s*(?P<object>\w+)', 'manipulation'),
                (r'(?P<action>pick up|grasp|grab|take) the (?P<object>\w+)', 'manipulation')
            ],
            'describe': [
                (r'describe|what do you see|tell me about the (?P<region>\w+)', 'perception')
            ]
        }
        
        # Unit conversion factors to meters
        self.unit_factors = {
            'm': 1.0, 'meter': 1.0, 'meters': 1.0,
            'cm': 0.01, 'centimeter': 0.01, 'centimeters': 0.01,
            'mm': 0.001, 'millimeter': 0.001, 'millimeters': 0.001,
            'in': 0.0254, 'inch': 0.0254, 'inches': 0.0254,
            'ft': 0.3048, 'foot': 0.3048, 'feet': 0.3048
        }
    
    def parse_command(self, text: str) -> Optional[ParsedCommand]:
        """
        Parse command and extract parameters
        """
        text_lower = text.lower().strip()
        
        for action, pattern_list in self.patterns.items():
            for pattern, category in pattern_list:
                match = re.search(pattern, text_lower, re.IGNORECASE)
                if match:
                    # Extract parameters from named groups
                    params = match.groupdict()
                    
                    # Convert units if distance parameter exists
                    if 'distance' in params and params['distance'] and 'unit' in params and params['unit']:
                        try:
                            distance_val = float(params['distance'])
                            unit = params['unit']
                            # Convert to meters
                            if unit in self.unit_factors:
                                params['distance_m'] = distance_val * self.unit_factors[unit]
                                params['distance_original'] = f"{distance_val}{unit[0] if len(unit) > 1 else unit}"
                        except (ValueError, KeyError):
                            pass  # Keep original distance if conversion fails
                    
                    # Clean up None values
                    params = {k: v for k, v in params.items() if v is not None}
                    
                    return ParsedCommand(action=action, parameters=params)
        
        return None
    
    def generate_ros_command(self, parsed: ParsedCommand):
        """
        Generate ROS command from parsed command
        """
        if parsed.action == 'move':
            if 'distance_m' in parsed.parameters:
                # Generate movement command with distance
                direction = parsed.parameters.get('direction', 'forward')
                distance = parsed.parameters['distance_m']
                return f"linear_velocity: {self.get_velocity_for_direction(direction)}, distance: {distance}"
            elif 'location' in parsed.parameters:
                # Generate navigation command
                location = parsed.parameters['location']
                return f"navigate_to_location: {location}"
        
        elif parsed.action == 'grasp':
            obj = parsed.parameters.get('object', 'object')
            color = parsed.parameters.get('color')
            if color:
                return f"grasp: {color} {obj}"
            else:
                return f"grasp: {obj}"
        
        elif parsed.action == 'describe':
            region = parsed.parameters.get('region', 'environment')
            return f"describe_scene: {region}"
        
        return f"unknown_command: {parsed.action}"

# Example usage
mapper = AdvancedCommandMapper()

commands = [
    "Move forward 2 meters",
    "Go to the kitchen", 
    "Pick up the red cup",
    "Turn left 45 degrees",
    "What do you see?"
]

for cmd_text in commands:
    parsed = mapper.parse_command(cmd_text)
    if parsed:
        ros_cmd = mapper.generate_ros_command(parsed)
        print(f"Input: {cmd_text}")
        print(f"Parsed: {parsed}")
        print(f"ROS Command: {ros_cmd}")
        print("---")
```

## 4. Complete Integration Example

```python
#!/usr/bin/env python3
"""
Complete integration example showing all components working together
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from typing import Optional
import openai
import os
import tempfile
import wave
import time

class VoiceCommandIntegrator(Node):
    """
    Complete integration of voice recognition, command mapping, and execution
    """
    
    def __init__(self):
        super().__init__('voice_command_integrator')
        
        # Initialize OpenAI API
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/vla/status', 10)
        
        # Subscribers
        self.text_sub = self.create_subscription(
            String,
            '/vla/recognized_text',
            self.text_callback,
            10
        )
        
        # Command mapper
        self.command_mapper = AdvancedCommandMapper()
        
        self.get_logger().info("Voice Command Integrator initialized")
    
    def text_callback(self, msg):
        """
        Handle recognized text and execute appropriate action
        """
        text = msg.data
        self.get_logger().info(f"Processing command: {text}")
        
        # Parse the command
        parsed_cmd = self.command_mapper.parse_command(text)
        
        if parsed_cmd:
            self.get_logger().info(f"Parsed command: {parsed_cmd.action}, params: {parsed_cmd.parameters}")
            
            # Execute the command
            success = self.execute_command(parsed_cmd)
            
            if success:
                self.get_logger().info(f"Command executed successfully: {text}")
                self.status_pub.publish(String(data=f"command_executed: {text}"))
            else:
                self.get_logger().error(f"Command execution failed: {text}")
                self.status_pub.publish(String(data=f"command_failed: {text}"))
        else:
            self.get_logger().warning(f"Unrecognized command: {text}")
            self.status_pub.publish(String(data=f"command_unrecognized: {text}"))
    
    def execute_command(self, parsed_cmd: ParsedCommand) -> bool:
        """
        Execute parsed command by publishing to appropriate ROS interfaces
        """
        try:
            if parsed_cmd.action == 'move':
                return self.execute_move_command(parsed_cmd)
            elif parsed_cmd.action == 'grasp':
                return self.execute_grasp_command(parsed_cmd)
            elif parsed_cmd.action == 'describe':
                return self.execute_describe_command(parsed_cmd)
            else:
                self.get_logger().warning(f"Unknown action: {parsed_cmd.action}")
                return False
        
        except Exception as e:
            self.get_logger().error(f"Error executing command: {str(e)}")
            return False
    
    def execute_move_command(self, parsed_cmd: ParsedCommand) -> bool:
        """
        Execute movement commands
        """
        direction = parsed_cmd.parameters.get('direction', 'forward')
        distance = parsed_cmd.parameters.get('distance_m', 1.0)  # default 1 meter
        
        # Create Twist message for movement
        twist = Twist()
        
        if direction == 'forward':
            twist.linear.x = 0.5  # m/s
        elif direction == 'backward':
            twist.linear.x = -0.5
        elif direction == 'left':
            twist.angular.z = 0.5  # rad/s
        elif direction == 'right':
            twist.angular.z = -0.5
        
        # Calculate duration to move the specified distance
        if direction in ['forward', 'backward']:
            duration = distance / abs(twist.linear.x) if twist.linear.x != 0 else 0
        else:  # turning
            # For simplicity, assume 90-degree turn takes 1 second
            duration = 1.0
        
        # Publish command for the calculated duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # 10 Hz publish rate
        
        # Stop robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        self.get_logger().info(f"Movement command executed: {direction} for {distance}m")
        return True
    
    def execute_grasp_command(self, parsed_cmd: ParsedCommand) -> bool:
        """
        Execute grasping commands
        """
        obj = parsed_cmd.parameters.get('object', 'object')
        color = parsed_cmd.parameters.get('color')
        
        # In a real system, this would publish to a manipulation interface
        # For this example, just log the action
        if color:
            self.get_logger().info(f"Grasping command: {color} {obj}")
        else:
            self.get_logger().info(f"Grasping command: {obj}")
        
        # In a real implementation, you would:
        # 1. Activate perception to locate the object
        # 2. Plan grasping trajectory
        # 3. Execute grasping action
        
        return True
    
    def execute_describe_command(self, parsed_cmd: ParsedCommand) -> bool:
        """
        Execute scene description commands
        """
        region = parsed_cmd.parameters.get('region', 'environment')
        self.get_logger().info(f"Description requested for: {region}")
        
        # In a real system, this would:
        # 1. Activate perception system
        # 2. Analyze the environment
        # 3. Generate natural language description
        # 4. Potentially respond via text-to-speech
        
        return True

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VoiceCommandIntegrator()
        
        try:
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

## 5. Testing Examples

```python
import unittest
from unittest.mock import Mock, patch, MagicMock
import tempfile
import wave

class TestVoiceCommandExamples(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures"""
        self.mapper = AdvancedCommandMapper()
    
    def test_command_parsing(self):
        """Test command parsing with various inputs"""
        test_cases = [
            ("Move forward 2 meters", "move", {"direction": "forward", "distance": "2", "unit": "meters", "distance_m": 2.0}),
            ("Go to the kitchen", "move", {"location": "kitchen"}),
            ("Pick up the red cup", "grasp", {"action": "pick up", "color": "red", "object": "cup"}),
            ("Turn left", "move", {"direction": "left"}),
        ]
        
        for text, expected_action, expected_params in test_cases:
            with self.subTest(text=text):
                result = self.mapper.parse_command(text)
                self.assertIsNotNone(result, f"Failed to parse: {text}")
                self.assertEqual(result.action, expected_action)
    
    def test_ros_command_generation(self):
        """Test ROS command generation from parsed commands"""
        # Test movement command
        move_cmd = ParsedCommand(
            action="move", 
            parameters={"direction": "forward", "distance_m": 1.5}
        )
        ros_cmd = self.mapper.generate_ros_command(move_cmd)
        self.assertIn("linear_velocity", ros_cmd)
        
        # Test grasp command
        grasp_cmd = ParsedCommand(
            action="grasp",
            parameters={"object": "cup", "color": "red"}
        )
        ros_cmd = self.mapper.generate_ros_command(grasp_cmd)
        self.assertIn("red cup", ros_cmd)
    
    @patch('openai.Audio.transcribe')
    def test_whisper_integration(self, mock_transcribe):
        """Test Whisper API integration with mock"""
        # Mock the API response
        mock_transcribe.return_value = {
            "text": "Move forward",
            "segments": [{"avg_logprob": -0.2}]
        }
        
        # Create temporary WAV file for testing
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_wav:
            # Create a simple WAV file
            wav_file = wave.open(temp_wav.name, 'wb')
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(16000)
            wav_file.writeframes(b'\x00' * 32000)  # 1 second of silence
            wav_file.close()
            
            # Test transcription
            result = AdvancedVoiceRecognizer.transcribe_audio(None, temp_wav.name)
            self.assertIsNotNone(result)
            self.assertEqual(result["text"], "Move forward")

if __name__ == '__main__':
    unittest.main()
```

## 6. Configuration Examples

```yaml
# Example configuration file: vla_voice_config.yaml

voice_command_integrator:
  ros__parameters:
    # Whisper API parameters
    whisper_model: 'whisper-1'
    confidence_threshold: 0.7
    language: 'en'
    max_audio_duration: 30.0
    max_retries: 3
    
    # Audio input parameters
    sample_rate: 16000
    channels: 1
    chunk_size: 1024
    enable_noise_reduction: true
    vad_threshold: 0.01
    silence_timeout: 5.0
    
    # Command mapping parameters
    command_threshold: 0.6
    enable_nlu: true
    robot_frame: 'base_link'
    map_frame: 'map'
    
    # Performance parameters
    request_timeout: 120.0
    processing_frequency: 10.0  # Hz
```

These examples demonstrate various aspects of voice command processing in the VLA system, from basic implementations to advanced integration examples. Each example shows practical code that can be used in real VLA implementations.
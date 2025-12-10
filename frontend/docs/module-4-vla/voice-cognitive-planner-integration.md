# Integration of Voice Command Pipeline with Cognitive Planner

## Overview

This document describes the integration between the voice command pipeline and the cognitive planner in the Vision-Language-Action (VLA) system. The integration enables the system to process spoken commands, convert them to text using speech recognition, and then generate appropriate action sequences using the cognitive planning system.

## Architecture of Integration

The integration follows this flow:

1. **Voice Input**: User speaks command to the robot
2. **Speech Recognition**: Audio is converted to text using Whisper API
3. **Natural Language Processing**: Text command is processed and standardized
4. **Cognitive Planning**: LLM generates action sequences based on command and context
5. **Action Execution**: Action sequences are sent to execution modules

## Implementation

### Voice Command Node

```python
#!/usr/bin/env python3
"""
Voice Command Pipeline Node

This node handles audio input, processes it through speech recognition,
and forwards natural language commands to the cognitive planner.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Int8
from sensor_msgs.msg import AudioData
from vla_msgs.srv import TranscribeAudio
from vla_msgs.msg import ActionSequence
import speech_recognition as sr
import io
import wave
import tempfile
import threading
import queue
import numpy as np
import os
from openai import OpenAI


class VoiceCommandNode(Node):
    """
    Node that processes voice commands and forwards them to cognitive planner
    """
    
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000  # Adjust for ambient noise
        
        # Initialize OpenAI client
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
        # QoS profile
        self.qos_profile = QoSProfile(depth=10)
        
        # Publishers
        self.natural_command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            self.qos_profile
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla/voice_status',
            self.qos_profile
        )
        
        self.audio_activity_publisher = self.create_publisher(
            Int8,
            '/vla/audio_activity',
            self.qos_profile
        )
        
        # Subscribers
        self.audio_subscriber = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            self.qos_profile
        )
        
        # Service client for cognitive planner
        self.cognitive_planner_client = self.create_client(
            TranscribeAudio,
            'transcribe_audio'
        )
        
        # Internal state
        self.is_listening = False
        self.audio_buffer = queue.Queue()
        self.command_history = []
        
        # Timer for continuous listening
        self.listen_timer = self.create_timer(0.1, self.process_audio_buffer)
        
        self.get_logger().info('Voice Command Node initialized')
    
    def audio_callback(self, msg):
        """
        Handle incoming audio data
        """
        if not self.is_listening:
            # Check if audio energy exceeds threshold to start listening
            audio_data = np.frombuffer(msg.data, dtype=np.int16)
            energy = np.mean(np.abs(audio_data))
            
            if energy > self.recognizer.energy_threshold * 0.5:  # Lower threshold to detect speech start
                self.is_listening = True
                self.get_logger().info('Listening started based on audio energy')
                
                # Publish audio activity
                activity_msg = Int8()
                activity_msg.data = 1
                self.audio_activity_publisher.publish(activity_msg)
        
        # Add audio data to processing queue
        self.audio_buffer.put(msg.data)
    
    def process_audio_buffer(self):
        """
        Process buffered audio data
        """
        if self.audio_buffer.empty() or not self.is_listening:
            return
        
        # Collect audio data from buffer
        audio_frames = []
        while not self.audio_buffer.empty():
            audio_frames.append(self.audio_buffer.get_nowait())
        
        if not audio_frames:
            return
        
        # Convert to audio format for speech recognition
        audio_bytes = b''.join(audio_frames)
        
        # Create a WAV file in memory
        with io.BytesIO() as wav_buffer:
            with wave.open(wav_buffer, 'wb') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(16000)  # 16kHz
                wav_file.writeframes(audio_bytes)
            
            # Convert to AudioData object for speech recognition
            wav_data = wav_buffer.getvalue()
            audio_data = sr.AudioData(wav_data, 16000, 2)
        
        try:
            # Use speech recognition to convert to text
            # For this example, we'll use whisper directly
            text = self.recognize_speech(audio_data)
            
            if text and text.strip():
                self.get_logger().info(f'Recognized: {text}')
                
                # Publish natural command
                command_msg = String()
                command_msg.data = text
                self.natural_command_publisher.publish(command_msg)
                
                # Add to command history
                self.command_history.append({
                    'text': text,
                    'timestamp': self.get_clock().now().nanoseconds
                })
                
                # Reset listening state
                self.is_listening = False
                
                # Publish audio activity
                activity_msg = Int8()
                activity_msg.data = 0
                self.audio_activity_publisher.publish(activity_msg)
                
                # Publish status
                status_msg = String()
                status_msg.data = f'Command recognized: {text}'
                self.status_publisher.publish(status_msg)
            else:
                # Check if we've been listening too long without a command
                # For simplicity, we'll just reset after processing
                pass
                
        except sr.UnknownValueError:
            self.get_logger().info('Speech recognition could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')
    
    def recognize_speech(self, audio_data):
        """
        Recognize speech from audio data using Whisper via API or local model
        """
        # Create a temporary file for Whisper processing
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            temp_file.write(audio_data.get_wav_data())
            temp_filename = temp_file.name
        
        try:
            # Use OpenAI Whisper API
            with open(temp_filename, 'rb') as audio_file:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file
                )
            return transcript.text
        except Exception as e:
            self.get_logger().error(f'Whisper API error: {e}')
            return None
        finally:
            # Clean up temporary file
            os.unlink(temp_filename)
    
    def start_listening(self):
        """
        Manually start listening mode
        """
        self.is_listening = True
        self.get_logger().info('Manual listening started')
    
    def stop_listening(self):
        """
        Manually stop listening mode
        """
        self.is_listening = False
        self.get_logger().info('Manual listening stopped')


class VoiceToActionIntegrator:
    """
    Class to manage the integration between voice pipeline and cognitive planner
    """
    
    def __init__(self, voice_node, cognitive_planner_node):
        self.voice_node = voice_node
        self.cognitive_planner = cognitive_planner_node
        
        # Subscribe to natural commands from voice node
        self.command_subscription = voice_node.create_subscription(
            String,
            '/vla/natural_command',
            self.command_forward_callback,
            QoSProfile(depth=10)
        )
        
        # Subscribe to action sequences from cognitive planner
        self.action_subscription = cognitive_planner_node.create_subscription(
            ActionSequence,
            '/vla/action_sequence',
            self.action_sequence_callback,
            QoSProfile(depth=10)
        )
        
        self.get_logger().info('Voice-to-Action integration initialized')
    
    def command_forward_callback(self, msg):
        """
        Forward natural language command to cognitive planner
        """
        self.get_logger().info(f'Forwarding command to cognitive planner: {msg.data}')
        
        # In this example, the cognitive planner already subscribes to the same topic
        # In a real system, you might have a direct service call or different architecture
        
        # For demonstration, we could also call the cognitive planner directly
        # This would require the cognitive planner to expose a service
        pass
    
    def action_sequence_callback(self, msg):
        """
        Handle action sequence from cognitive planner
        """
        self.get_logger().info(f'Received action sequence with {len(msg.actions)} actions')
        
        # In a real system, this would be forwarded to the execution layer
        # For now, just log the actions
        for i, action in enumerate(msg.actions):
            self.get_logger().info(f'  Action {i+1}: {action.type} - {action.description}')
    
    def get_logger(self):
        """
        Utility to access logger (in a real implementation, this would be a proper node)
        """
        return self.voice_node.get_logger() if self.voice_node else rclpy.logging.get_logger('integration')


def main(args=None):
    """
    Main function to run the voice command node
    """
    rclpy.init(args=args)
    
    # Create the voice command node
    voice_node = VoiceCommandNode()
    
    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        voice_node.get_logger().info('Voice Command Node interrupted by user')
    finally:
        voice_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Enhanced Cognitive Planner Integration

```python
#!/usr/bin/env python3
"""
Enhanced Cognitive Planner with Voice Integration

This version of the cognitive planner is specifically designed to work
with the voice command pipeline, processing natural language commands
that come from voice recognition.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence, VLAAction, DetectedObjects
from vla_msgs.srv import PlanCognitiveTask
from openai import OpenAI
import json
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class VoiceEnhancedContext:
    """
    Enhanced context that includes voice-specific information
    """
    robot_position: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})
    robot_state: Dict[str, any] = field(default_factory=lambda: {"holding": None, "battery": 1.0})
    detected_objects: Dict[str, Dict[str, any]] = field(default_factory=dict)
    conversation_history: List[Dict[str, str]] = field(default_factory=list)
    voice_context: Dict[str, any] = field(default_factory=dict)  # Voice-specific context
    timestamp: float = field(default_factory=time.time)


class VoiceAwareCognitivePlanner(Node):
    """
    Cognitive planner enhanced for voice command processing
    """
    
    def __init__(self):
        super().__init__('voice_aware_cognitive_planner')
        
        # Initialize OpenAI client
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
        # QoS profile
        self.qos_profile = QoSProfile(depth=10)
        
        # Publishers
        self.action_sequence_publisher = self.create_publisher(
            ActionSequence,
            '/vla/action_sequence',
            self.qos_profile
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla/cognitive_planner_status',
            self.qos_profile
        )
        
        # Subscribers
        self.natural_command_subscriber = self.create_subscription(
            String,
            '/vla/natural_command',
            self.natural_command_callback,
            self.qos_profile
        )
        
        self.perception_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.perception_callback,
            self.qos_profile
        )
        
        # Service server for planning requests
        self.plan_service = self.create_service(
            PlanCognitiveTask,
            'plan_cognitive_task',
            self.plan_cognitive_task_callback
        )
        
        # Initialize context
        self.context = VoiceEnhancedContext()
        
        # Available actions
        self.available_actions = {
            "move_to": {
                "description": "Move robot to specified coordinates",
                "params": {"x": "float", "y": "float", "z": "float (optional)"}
            },
            "grasp": {
                "description": "Grasp an object by name",
                "params": {"object_name": "string"}
            },
            "detect_object": {
                "description": "Detect a specific object in the environment",
                "params": {"object_name": "string"}
            },
            "navigate_to": {
                "description": "Navigate to an object",
                "params": {"object_name": "string"}
            },
            "place_at": {
                "description": "Place held object at specified coordinates",
                "params": {"x": "float", "y": "float", "z": "float"}
            },
            "pick_and_place": {
                "description": "Pick up an object and place it at a target location",
                "params": {"object_name": "string", "target_name": "string"}
            }
        }
        
        self.get_logger().info('Voice-Aware Cognitive Planner initialized')
    
    def natural_command_callback(self, msg: String):
        """
        Handle natural language commands from voice pipeline
        """
        command = msg.data.strip()
        self.get_logger().info(f'Received voice command: {command}')
        
        # Add to conversation history
        self.context.conversation_history.append({
            "speaker": "user",
            "text": command,
            "timestamp": time.time()
        })
        
        # Plan the action sequence
        action_sequence, reasoning = self.plan_action_sequence_with_context(command)
        
        if action_sequence:
            # Create and publish action sequence message
            action_seq_msg = ActionSequence()
            action_seq_msg.request_id = f"voice_cmd_{int(time.time())}"
            action_seq_msg.command = command
            action_seq_msg.reasoning = reasoning
            
            # Convert to VLAAction messages
            vla_actions = []
            for action_dict in action_sequence:
                vla_action = VLAAction()
                vla_action.type = action_dict.get('type', '')
                vla_action.params = json.dumps(action_dict.get('params', {}))
                vla_action.description = action_dict.get('description', '')
                vla_action.confidence = action_dict.get('confidence', 0.8)  # Default confidence
                vla_actions.append(vla_action)
            
            action_seq_msg.actions = vla_actions
            
            # Publish the action sequence
            self.action_sequence_publisher.publish(action_seq_msg)
            
            # Publish status
            status_msg = String()
            status_msg.data = f"Planned {len(action_sequence)} actions for command: {command}"
            self.status_publisher.publish(status_msg)
            
            self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')
        else:
            self.get_logger().error('Failed to generate action sequence')
            
            # Publish error status
            status_msg = String()
            status_msg.data = f"Failed to plan actions for command: {command}"
            self.status_publisher.publish(status_msg)
    
    def perception_callback(self, msg: DetectedObjects):
        """
        Update perception data in context
        """
        new_objects = {}
        for obj in msg.objects:
            new_objects[obj.name] = {
                "position": {
                    "x": obj.pose.position.x,
                    "y": obj.pose.position.y, 
                    "z": obj.pose.position.z
                },
                "confidence": obj.confidence
            }
        
        self.context.detected_objects = new_objects
        self.context.timestamp = time.time()
        
        self.get_logger().info(f'Updated perception data: {len(new_objects)} objects detected')
    
    def plan_cognitive_task_callback(self, request, response):
        """
        Service callback for cognitive task planning
        """
        self.get_logger().info(f'Received service request for: {request.command}')
        
        # Plan the action sequence
        action_sequence, reasoning = self.plan_action_sequence_with_context(request.command)
        
        if action_sequence:
            response.success = True
            response.message = f"Planned {len(action_sequence)} actions"
            
            # Create action sequence for response
            action_seq = ActionSequence()
            action_seq.request_id = f"service_{int(time.time())}"
            action_seq.command = request.command
            action_seq.reasoning = reasoning
            
            # Convert to VLAAction messages
            vla_actions = []
            for action_dict in action_sequence:
                vla_action = VLAAction()
                vla_action.type = action_dict.get('type', '')
                vla_action.params = json.dumps(action_dict.get('params', {}))
                vla_action.description = action_dict.get('description', '')
                vla_action.confidence = action_dict.get('confidence', 0.8)
                vla_actions.append(vla_action)
            
            action_seq.actions = vla_actions
            response.action_sequence = action_seq
        else:
            response.success = False
            response.message = "Failed to plan actions"
            response.action_sequence = ActionSequence()
        
        return response
    
    def plan_action_sequence_with_context(self, command: str):
        """
        Plan action sequence using voice-enhanced context
        """
        try:
            # Construct prompt with voice-specific context
            prompt = self.construct_voice_aware_prompt(command)
            
            # Call the LLM
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are a voice-aware cognitive planning system for a robot. "
                            "Users speak commands that may have speech recognition artifacts like "
                            "um, uh, or misrecognized words. Interpret the intent and generate "
                            "appropriate action sequences. Be robust to speech imperfections. "
                            "Output only valid JSON as specified."
                        )
                            },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                max_tokens=1000,
                temperature=0.3
            )
            
            # Extract and parse the response
            response_text = response.choices[0].message.content.strip()
            
            # Clean up markdown formatting
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]
            
            # Parse JSON response
            result = json.loads(response_text)
            
            # Validate the structure
            if 'actions' not in result:
                raise ValueError("Response does not contain 'actions' field")
            
            # Add to conversation history
            self.context.conversation_history.append({
                "speaker": "planner",
                "actions_planned": len(result['actions']),
                "timestamp": time.time()
            })
            
            return result['actions'], result.get('reasoning', 'No reasoning provided')
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing JSON response: {e}")
            return None, f"JSON parsing error: {e}"
        except Exception as e:
            self.get_logger().error(f"Error in action planning: {e}")
            return None, f"Planning error: {e}"
    
    def construct_voice_aware_prompt(self, command: str):
        """
        Construct a prompt that's aware of voice input characteristics
        """
        # Clean up common voice artifacts
        cleaned_command = self.clean_voice_artifacts(command)
        
        prompt = f"""
        The user spoke: "{command}"
        Interpreted as: "{cleaned_command}"
        
        Current environmental context:
        Robot position: {json.dumps(self.context.robot_position)}
        Robot state: {json.dumps(self.context.robot_state)}
        Detected objects: {json.dumps(self.context.detected_objects)}
        
        Conversation history (for context):
        {json.dumps(self.context.conversation_history[-5:])}  # Last 5 exchanges
        
        Available robot actions:
        {json.dumps(self.available_actions, indent=2)}
        
        Generate a sequence of actions for the robot to complete the user's request.
        Consider that this command came from voice input, which may have artifacts.
        Be forgiving of imprecise language and try to infer the most likely intent.
        
        If the command is unclear or ambiguous, make reasonable assumptions based on context.
        
        Respond with only valid JSON in this format:
        {{
            "actions": [
                {{
                    "type": "navigate_to",
                    "params": {{"object_name": "red cup"}},
                    "description": "Navigate to the red cup",
                    "confidence": 0.9
                }}
            ],
            "reasoning": "An explanation of how this plan addresses the user's intent, "
                       "including how voice input artifacts were handled"
        }}
        """
        
        return prompt
    
    def clean_voice_artifacts(self, command: str) -> str:
        """
        Clean common speech recognition artifacts from the command
        """
        # Remove common filler words
        import re
        
        # Convert to lowercase for processing
        cleaned = command.lower()
        
        # Remove common filler words and artifacts
        fillers = ['um', 'uh', 'ah', 'er', 'okay', 'right', 'so', 'well']
        for filler in fillers:
            # Use word boundaries to avoid partial matches
            cleaned = re.sub(rf'\b{filler}\b\s*', '', cleaned, flags=re.IGNORECASE)
        
        # Remove extra whitespace
        cleaned = ' '.join(cleaned.split())
        
        # Capitalize first letter to match original format
        if cleaned:
            cleaned = cleaned[0].upper() + cleaned[1:]
        
        return cleaned


def main(args=None):
    """
    Main function to run the voice-aware cognitive planner
    """
    rclpy.init(args=args)
    
    planner = VoiceAwareCognitivePlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Voice-Aware Cognitive Planner interrupted by user')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Voice Command Pipeline Integration

### Launch File

```xml
<!-- vla_voice_integration.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        
        # Voice command node
        Node(
            package='vla_examples',
            executable='voice_command_node',
            name='voice_command_node',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        # Voice-aware cognitive planner
        Node(
            package='vla_examples',
            executable='voice_aware_cognitive_planner',
            name='voice_aware_cognitive_planner',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        # Optional: Audio input driver (would depend on your microphone setup)
        # Node(
        #     package='audio_capture',
        #     executable='audio_capture_node',
        #     name='audio_capture',
        #     parameters=[
        #         {'use_sim_time': use_sim_time}
        #     ]
        # )
    ])
```

## Testing the Integration

```python
# test_voice_integration.py
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence
from rclpy.qos import QoSProfile
import time


class TestVoiceCognitiveIntegration(Node):
    """
    Test node to verify integration between voice pipeline and cognitive planner
    """
    
    def __init__(self):
        super().__init__('test_voice_integration')
        
        self.qos_profile = QoSProfile(depth=10)
        
        # Publisher for test commands
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            self.qos_profile
        )
        
        # Subscriber for action sequences
        self.action_subscriber = self.create_subscription(
            ActionSequence,
            '/vla/action_sequence',
            self.action_callback,
            self.qos_profile
        )
        
        self.last_action_sequence = None
        self.test_completed = False
        
    def action_callback(self, msg):
        """
        Handle received action sequences
        """
        self.last_action_sequence = msg
        self.get_logger().info(f'Received action sequence with {len(msg.actions)} actions')
        self.test_completed = True
    
    def send_test_command(self, command):
        """
        Send a test command to the system
        """
        cmd_msg = String()
        cmd_msg.data = command
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info(f'Sent test command: {command}')


def test_integration():
    """
    Test function to verify voice-to-cognitive planner integration
    """
    rclpy.init()
    
    test_node = TestVoiceCognitiveIntegration()
    
    # Send a test command
    test_command = "Pick up the red cup"
    test_node.send_test_command(test_command)
    
    # Wait for response with timeout
    timeout = time.time() + 30.0  # 30 second timeout
    while not test_node.test_completed and time.time() < timeout:
        rclpy.spin_once(test_node, timeout_sec=0.1)
    
    # Verify results
    if test_node.test_completed and test_node.last_action_sequence:
        actions = test_node.last_action_sequence.actions
        print(f"SUCCESS: Received action sequence with {len(actions)} actions")
        
        for i, action in enumerate(actions):
            print(f"  {i+1}. {action.type}: {action.description}")
        
        # Check if the actions make sense for the command
        action_types = [a.type for a in actions]
        has_navigation = 'navigate_to' in action_types
        has_grasp = 'grasp' in action_types
        
        if has_navigation and has_grasp:
            print("SUCCESS: Action sequence includes navigation and grasp as expected")
        else:
            print(f"WARNING: Expected navigation and grasp actions, got: {action_types}")
    else:
        print("FAILURE: Did not receive action sequence within timeout")
    
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    test_integration()
```

## Key Integration Points

### 1. Message Flow
- Voice pipeline publishes to `/vla/natural_command`
- Cognitive planner subscribes to the same topic
- Action sequences published to `/vla/action_sequence`

### 2. Voice-Specific Enhancements
- Voice artifact cleaning in cognitive planner
- Conversation history for context
- Robustness to speech recognition imperfections

### 3. Error Handling
- Fallbacks when speech is unclear
- Graceful degradation when commands are ambiguous
- Status reporting for debugging

### 4. Performance Considerations
- Efficient audio processing pipeline
- Quick response times for voice interactions
- Proper buffering to handle audio streams

## Running the Integrated System

To run the complete voice-to-cognitive planner integration:

```bash
# Terminal 1: Start the voice command node
ros2 run vla_examples voice_command_node

# Terminal 2: Start the voice-aware cognitive planner
ros2 run vla_examples voice_aware_cognitive_planner

# Terminal 3: (Optional) Provide audio input or simulate commands
ros2 topic pub /audio_input sensor_msgs/AudioData "data: [0, 1, 2, ...]"  # or use audio driver
# OR simulate a command directly:
ros2 topic pub /vla/natural_command std_msgs/String "data: 'Pick up the red cup'"
```

## Conclusion

This integration provides a complete pathway from voice input to action execution in the VLA system. The voice command pipeline handles audio processing and speech recognition, while the cognitive planner interprets the resulting text commands and generates appropriate action sequences. The integration includes voice-specific enhancements to handle the unique characteristics of speech input, ensuring robust performance in real-world applications.
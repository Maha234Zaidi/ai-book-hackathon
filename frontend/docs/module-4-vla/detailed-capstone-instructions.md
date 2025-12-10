# Detailed Capstone Project Instructions: Vision-Language-Action (VLA) System

## Overview

This document provides comprehensive, step-by-step instructions for completing the Vision-Language-Action (VLA) Capstone Project. It guides you through each phase of implementation, testing, and validation of the complete VLA system.

## Prerequisites

Before beginning this project, ensure you have:

### Hardware Requirements
- Compatible ROS 2 robot platform (e.g., TurtleBot3, Clearpath Jackal, or similar)
- Computer with ROS 2 Humble Hawksbill installed
- Microphone for voice input
- 3D camera (RGB-D or stereo) for perception

### Software Requirements
- ROS 2 Humble Hawksbill
- Python 3.10 or higher
- OpenAI Python package
- TensorFlow or PyTorch for perception
- Gazebo for simulation testing
- Git for version control

### Account Requirements
- OpenAI API key for Whisper and LLM services
- GitHub account for code repository (optional but recommended)

## Phase 1: Project Setup and Environment Configuration

### Step 1.1: Create the Project Workspace

1. Open a terminal and create a new ROS workspace:
   ```bash
   mkdir -p ~/vla_ws/src
   cd ~/vla_ws
   ```

2. Navigate to the source directory:
   ```bash
   cd ~/vla_ws/src
   ```

3. Create your project package:
   ```bash
   ros2 pkg create --build-type ament_python vla_examples
   ```

### Step 1.2: Install Dependencies

1. Install required ROS packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs
   ```

2. Install Python dependencies:
   ```bash
   pip3 install openai speechrecognition transformers torch torchvision torchaudio
   pip3 install opencv-python cv-bridge tf-transformations
   ```

3. Set up your OpenAI API key:
   ```bash
   export OPENAI_API_KEY="your-openai-api-key-here"
   echo 'export OPENAI_API_KEY="your-openai-api-key-here"' >> ~/.bashrc
   ```

### Step 1.3: Project Structure Setup

1. Create the project directory structure:
   ```bash
   cd ~/vla_ws/src/vla_examples
   mkdir -p vla_examples/launch
   mkdir -p vla_examples/worlds
   mkdir -p vla_examples/config
   mkdir -p vla_examples/test
   ```

2. Create the main Python package structure:
   ```bash
   touch vla_examples/__init__.py
   touch vla_examples/vla_nodes/__init__.py
   ```

3. Update the package.xml file to include dependencies:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>vla_examples</name>
     <version>0.0.1</version>
     <description>Vision-Language-Action System Examples</description>
     <maintainer email="your-email@example.com">Your Name</maintainer>
     <license>Apache-2.0</license>

     <depend>rclpy</depend>
     <depend>std_msgs</depend>
     <depend>geometry_msgs</depend>
     <depend>sensor_msgs</depend>
     <depend>nav_msgs</depend>
     <depend>message_runtime</depend>

     <test_depend>ament_copyright</test_depend>
     <test_depend>ament_flake8</test_depend>
     <test_depend>ament_pep257</test_depend>
     <test_depend>python3-pytest</test_depend>

     <export>
       <build_type>ament_python</build_type>
     </export>
   </package>
   ```

### Step 1.4: Build the Workspace

1. Return to the workspace root:
   ```bash
   cd ~/vla_ws
   ```

2. Build the workspace:
   ```bash
   colcon build --packages-select vla_examples
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Phase 2: Core Component Implementation

### Step 2.1: Implement the Voice Command Pipeline

1. Create the voice command pipeline file:
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/voice_pipeline.py
   ```

2. Add the following content to `voice_pipeline.py`:
   ```python
   #!/usr/bin/env python3
   """
   Voice Command Pipeline for VLA System
   
   This node handles audio input, processes speech using Whisper,
   and publishes natural language commands to the cognitive planner.
   """
   
   import rclpy
   from rclpy.node import Node
   from rclpy.qos import QoSProfile
   from std_msgs.msg import String, Int8
   from sensor_msgs.msg import AudioData
   from vla_msgs.srv import TranscribeAudio
   import speech_recognition as sr
   import io
   import wave
   import tempfile
   import threading
   import queue
   import os
   from openai import OpenAI
   
   
   class VoicePipelineNode(Node):
       """
       Node that processes voice commands and forwards them to cognitive planner
       """
       
       def __init__(self):
           super().__init__('voice_pipeline_node')
           
           # Initialize speech recognition
           self.recognizer = sr.Recognizer()
           self.recognizer.energy_threshold = 4000
           
           # Initialize OpenAI client
           api_key = os.getenv('OPENAI_API_KEY')
           if not api_key:
               self.get_logger().warn('OPENAI_API_KEY environment variable not set')
           self.client = OpenAI(api_key=api_key)
           
           # QoS profile
           self.qos_profile = QoSProfile(depth=10)
           
           # Publishers
           self.command_publisher = self.create_publisher(
               String,
               '/vla/natural_command',
               self.qos_profile
           )
           
           self.status_publisher = self.create_publisher(
               String,
               '/vla/voice_status',
               self.qos_profile
           )
           
           self.activity_publisher = self.create_publisher(
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
           
           # Internal state
           self.is_listening = False
           self.audio_buffer = queue.Queue()
           
           # Timer for continuous processing
           self.process_timer = self.create_timer(0.1, self.process_audio_buffer)
           
           self.get_logger().info('Voice Pipeline Node initialized')
       
       def audio_callback(self, msg):
           """
           Handle incoming audio data
           """
           # Add audio to processing queue
           self.audio_buffer.put(msg.data)
           
           # Check if we should start listening based on energy
           if not self.is_listening:
               audio_data = memoryview(msg.data).tolist()
               energy = sum(abs(b) for b in audio_data) / len(audio_data)
               
               if energy > self.recognizer.energy_threshold * 0.5:
                   self.is_listening = True
                   self.get_logger().info('Listening started based on audio energy')
                   
                   # Publish activity
                   activity_msg = Int8()
                   activity_msg.data = 1
                   self.activity_publisher.publish(activity_msg)
       
       def process_audio_buffer(self):
           """
           Process buffered audio data
           """
           if self.audio_buffer.empty() or not self.is_listening:
               return
           
           # Collect audio frames from buffer
           audio_frames = []
           try:
               while not self.audio_buffer.empty():
                   audio_frames.append(self.audio_buffer.get_nowait())
           except queue.Empty:
               pass
           
           if not audio_frames:
               return
           
           # Combine frames and convert for processing
           audio_bytes = b''.join(audio_frames)
           
           with io.BytesIO() as wav_buffer:
               with wave.open(wav_buffer, 'wb') as wav_file:
                   wav_file.setnchannels(1)
                   wav_file.setsampwidth(2)
                   wav_file.setframerate(16000)
                   wav_file.writeframes(audio_bytes)
               
               audio_data = sr.AudioData(wav_buffer.getvalue(), 16000, 2)
           
           try:
               # Transcribe using Whisper API
               with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                   temp_file.write(audio_data.get_wav_data())
                   temp_filename = temp_file.name
               
               try:
                   with open(temp_filename, 'rb') as audio_file:
                       transcript = self.client.audio.transcriptions.create(
                           model="whisper-1",
                           file=audio_file
                       )
                   
                   if transcript.text.strip():
                       self.get_logger().info(f'Recognized: {transcript.text}')
                       
                       # Publish the command
                       cmd_msg = String()
                       cmd_msg.data = transcript.text
                       self.command_publisher.publish(cmd_msg)
                       
                       # Publish status
                       status_msg = String()
                       status_msg.data = f'Command recognized: {transcript.text}'
                       self.status_publisher.publish(status_msg)
                       
                       # Reset listening state
                       self.is_listening = False
                       
                       # Publish end of activity
                       activity_msg = Int8()
                       activity_msg.data = 0
                       self.activity_publisher.publish(activity_msg)
                       
               finally:
                   os.unlink(temp_filename)
                   
           except Exception as e:
               self.get_logger().error(f'Whisper API error: {e}')
               self.is_listening = False
               
               # Publish error status
               status_msg = String()
               status_msg.data = f'Whisper error: {e}'
               self.status_publisher.publish(status_msg)
               
               # Publish end of activity
               activity_msg = Int8()
               activity_msg.data = 0
               self.activity_publisher.publish(activity_msg)
   
   
   def main(args=None):
       """
       Main function to run the voice pipeline node
       """
       rclpy.init(args=args)
       
       node = VoicePipelineNode()
       
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           node.get_logger().info('Voice Pipeline interrupted by user')
       finally:
           node.destroy_node()
           rclpy.shutdown()
   
   
   if __name__ == '__main__':
       main()
   ```

3. Add the node to the setup.py file:
   ```bash
   # Edit ~/vla_ws/src/vla_examples/setup.py
   # Add to entry_points section:
   
   entry_points={
       'console_scripts': [
           'voice_pipeline = vla_examples.voice_pipeline:main',
           # Add other nodes as implemented
       ],
   },
   ```

### Step 2.2: Implement the Cognitive Planner

1. Create the cognitive planner file:
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/cognitive_planner.py
   ```

2. Add the following content to `cognitive_planner.py`:
   ```python
   #!/usr/bin/env python3
   """
   Cognitive Planner Node for VLA System
   
   This node uses LLMs to translate natural language commands into action sequences.
   """
   
   import rclpy
   from rclpy.node import Node
   from rclpy.qos import QoSProfile
   from std_msgs.msg import String
   from vla_msgs.msg import ActionSequence, VLAAction
   from vla_msgs.srv import PlanCognitiveTask
   from openai import OpenAI
   import json
   import os
   
   
   class CognitivePlannerNode(Node):
       """
       Node that plans actions based on natural language commands
       """
       
       def __init__(self):
           super().__init__('cognitive_planner_node')
           
           # Initialize LLM client
           api_key = os.getenv('OPENAI_API_KEY')
           if not api_key:
               self.get_logger().warn('OPENAI_API_KEY environment variable not set')
           self.client = OpenAI(api_key=api_key)
           
           # QoS profile
           self.qos_profile = QoSProfile(depth=10)
           
           # Publishers
           self.action_publisher = self.create_publisher(
               ActionSequence,
               '/vla/action_sequence',
               self.qos_profile
           )
           
           self.status_publisher = self.create_publisher(
               String,
               '/vla/planner_status',
               self.qos_profile
           )
           
           # Subscribers
           self.command_subscriber = self.create_subscription(
               String,
               '/vla/natural_command',
               self.command_callback,
               self.qos_profile
           )
           
           # Services
           self.plan_service = self.create_service(
               PlanCognitiveTask,
               'plan_cognitive_task',
               self.plan_service_callback
           )
           
           # Available actions for LLM
           self.available_actions = [
               "navigate_to(object_name)", 
               "grasp(object_name)",
               "place_at(x, y, z)",
               "move_to(x, y)",
               "detect_object(object_name)",
               "pick_and_place(object_name, target_name)"
           ]
           
           self.get_logger().info('Cognitive Planner Node initialized')
       
       def command_callback(self, msg):
           """
           Process natural language command and generate action sequence
           """
           command = msg.data
           self.get_logger().info(f'Received command: {command}')
           
           # Plan actions
           action_sequence, reasoning = self.plan_actions(command)
           
           if action_sequence:
               # Create and publish action sequence
               action_msg = self.create_action_sequence_msg(action_sequence, command, reasoning)
               self.action_publisher.publish(action_msg)
               
               self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')
           else:
               self.get_logger().error(f'Failed to generate action sequence for: {command}')
       
       def plan_actions(self, command):
           """
           Plan actions for the given command using LLM
           """
           # Define a simple context for this example
           context = {
               "objects": [
                   {"name": "red cup", "position": [1.0, 1.0, 0.8]},
                   {"name": "blue box", "position": [2.0, 2.0, 0.1]},
                   {"name": "table", "position": [0.0, 0.0, 0.0]},
                   {"name": "kitchen counter", "position": [3.0, -2.0, 0.8]}
               ],
               "robot_position": [0.0, 0.0, 0.0]
           }
           
           prompt = f"""
           The user wants the robot to: "{command}"
           
           Current environmental context:
           {json.dumps(context, indent=2)}
           
           Available robot actions:
           {', '.join(self.available_actions)}
           
           Generate a sequence of actions for the robot to complete the user's request.
           Consider the environmental context and the robot's current state.
           
           Also provide brief reasoning for the plan.
           
           Respond with only valid JSON in this format:
           {{
               "actions": [
                   {{
                       "type": "navigate_to",
                       "params": {{"object_name": "kitchen counter"}},
                       "description": "Navigate to kitchen counter",
                       "reasoning_explanation": "First step is to navigate to the kitchen counter where the red cup is located."
                   }}
               ],
               "reasoning": "The plan involves navigating to the kitchen counter, grasping the red cup, and bringing it to the user's location."
           }}
           """
           
           try:
               response = self.client.chat.completions.create(
                   model="gpt-3.5-turbo",
                   messages=[
                       {
                           "role": "system",
                           "content": "You are a robot action planner. Given a command and environmental context, generate a sequence of actions for the robot. Respond with only valid JSON."
                       },
                       {
                           "role": "user",
                           "content": prompt
                       }
                   ],
                   max_tokens=800,
                   temperature=0.3
               )
               
               response_text = response.choices[0].message.content.strip()
               
               # Clean up markdown formatting
               if response_text.startswith('```json'):
                   response_text = response_text[7:]
               if response_text.endswith('```'):
                   response_text = response_text[:-3]
               
               result = json.loads(response_text)
               
               if 'actions' not in result:
                   self.get_logger().error(f'Invalid response format: {result}')
                   return None, ""
               
               return result['actions'], result.get('reasoning', '')
               
           except json.JSONDecodeError as e:
               self.get_logger().error(f'Error parsing JSON response: {e}')
               return None, ""
           except Exception as e:
               self.get_logger().error(f'Error in action planning: {e}')
               return None, ""
       
       def create_action_sequence_msg(self, actions, original_command, reasoning):
           """
           Create ROS message from action sequence
           """
           msg = ActionSequence()
           msg.request_id = self.generate_request_id()
           msg.command = original_command
           msg.reasoning = reasoning
           
           for action_dict in actions:
               action_msg = VLAAction()
               action_msg.type = action_dict.get('type', '')
               action_msg.params = json.dumps(action_dict.get('params', {}))
               action_msg.description = action_dict.get('description', '')
               action_msg.reasoning_explanation = action_dict.get('reasoning_explanation', '')
               action_msg.confidence = action_dict.get('confidence', 0.8)
               
               msg.actions.append(action_msg)
           
           return msg
       
       def generate_request_id(self):
           """
           Generate unique request ID
           """
           import uuid
           return str(uuid.uuid4())
       
       def plan_service_callback(self, request, response):
           """
           Service callback for planning requests
           """
           self.get_logger().info(f'Service request: {request.command}')
           
           actions, reasoning = self.plan_actions(request.command)
           
           if actions:
               response.success = True
               response.message = f"Planned {len(actions)} actions"
               
               # Create action sequence for response
               action_seq = ActionSequence()
               action_seq.request_id = self.generate_request_id()
               action_seq.command = request.command
               action_seq.reasoning = reasoning
               
               for action_dict in actions:
                   action_msg = VLAAction()
                   action_msg.type = action_dict.get('type', '')
                   action_msg.params = json.dumps(action_dict.get('params', {}))
                   action_msg.description = action_dict.get('description', '')
                   action_msg.confidence = action_dict.get('confidence', 0.8)
                   
                   action_seq.actions.append(action_msg)
               
               response.action_sequence = action_seq
           else:
               response.success = False
               response.message = "Failed to plan actions"
               response.action_sequence = ActionSequence()
           
           return response
   
   
   def main(args=None):
       """
       Main function to run the cognitive planner node
       """
       rclpy.init(args=args)
       
       node = CognitivePlannerNode()
       
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           node.get_logger().info('Cognitive Planner interrupted by user')
       finally:
           node.destroy_node()
           rclpy.shutdown()
   
   
   if __name__ == '__main__':
       main()
   ```

3. Update setup.py to include the cognitive planner:
   ```python
   # In setup.py, add to entry_points:
   'console_scripts': [
       'voice_pipeline = vla_examples.voice_pipeline:main',
       'cognitive_planner = vla_examples.cognitive_planner:main',
   ],
   ```

### Step 2.3: Implement the Perception Module

1. Create the perception module file:
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/perception_module.py
   ```

2. Add the following content to `perception_module.py`:
   ```python
   #!/usr/bin/env python3
   """
   Perception Module Node for VLA System
   
   This node performs object detection and recognition using camera input.
   """
   
   import rclpy
   from rclpy.node import Node
   from rclpy.qos import QoSProfile
   from sensor_msgs.msg import Image, CameraInfo
   from geometry_msgs.msg import Point, Pose
   from std_msgs.msg import Header
   from vla_msgs.msg import DetectedObjects, Object
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   import json
   import time
   from typing import List, Dict, Tuple
   
   
   class PerceptionModuleNode(Node):
       """
       Node that performs object detection and publishes detected objects
       """
       
       def __init__(self):
           super().__init__('perception_module_node')
           
           # Initialize OpenCV bridge
           self.bridge = CvBridge()
           
           # QoS profile
           self.qos_profile = QoSProfile(depth=10)
           
           # Publishers
           self.objects_publisher = self.create_publisher(
               DetectedObjects,
               '/vla/perception/objects',
               self.qos_profile
           )
           
           self.debug_publisher = self.create_publisher(
               Image,
               '/vla/perception/debug_image',
               self.qos_profile
           )
           
           self.status_publisher = self.create_publisher(
               String,
               '/vla/perception_status',
               self.qos_profile
           )
           
           # Subscribers
           self.image_subscriber = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               self.qos_profile
           )
           
           self.camera_info_subscriber = self.create_subscription(
               CameraInfo,
               '/camera/camera_info',
               self.camera_info_callback,
               self.qos_profile
           )
           
           # Internal state
           self.camera_info = None
           self.detection_frequency = 1.0  # seconds between detections
           self.last_detection_time = 0.0
           
           # Known objects for demonstration
           self.known_objects = {
               'red cup': {'color': 'red', 'shape': 'cylinder', 'graspable': True},
               'blue box': {'color': 'blue', 'shape': 'cube', 'graspable': True},
               'table': {'color': 'brown', 'shape': 'rectangle', 'graspable': False},
               'kitchen counter': {'color': 'white', 'shape': 'rectangle', 'graspable': False}
           }
           
           self.get_logger().info('Perception Module Node initialized')
       
       def camera_info_callback(self, msg):
           """
           Handle camera information
           """
           self.camera_info = msg
       
       def image_callback(self, msg):
           """
           Process incoming camera image for object detection
           """
           current_time = time.time()
           
           # Throttle detection frequency
           if current_time - self.last_detection_time < self.detection_frequency:
               return
           
           try:
               # Convert ROS Image to OpenCV image
               cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
               
               # Perform object detection (simplified for this example)
               detected_objects = self.detect_objects(cv_image)
               
               # Create and publish DetectedObjects message
               detected_objects_msg = self.create_detected_objects_message(detected_objects, msg.header)
               self.objects_publisher.publish(detected_objects_msg)
               
               # Publish debug image
               debug_image = self.draw_bounding_boxes(cv_image, detected_objects)
               debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
               debug_msg.header = msg.header
               self.debug_publisher.publish(debug_msg)
               
               # Publish status
               status_msg = String()
               status_msg.data = f'Detected {len(detected_objects)} objects'
               self.status_publisher.publish(status_msg)
               
               self.get_logger().info(f'Detected {len(detected_objects)} objects')
               self.last_detection_time = current_time
               
           except Exception as e:
               self.get_logger().error(f'Error processing image: {e}')
       
       def detect_objects(self, image):
           """
           Perform object detection on the input image
           """
           # This is a simplified detection for demonstration
           # In a real implementation, you would use a trained model
           
           height, width = image.shape[:2]
           detected_objects = []
           
           # Example detections with known objects
           for obj_name, properties in self.known_objects.items():
               if obj_name == 'red cup':
                   # Simulate detection of red cup
                   obj = {
                       'name': obj_name,
                       'position': Point(x=1.0, y=1.0, z=0.8),
                       'confidence': 0.89,
                       'properties': properties
                   }
                   detected_objects.append(obj)
               elif obj_name == 'blue box':
                   # Simulate detection of blue box
                   obj = {
                       'name': obj_name,
                       'position': Point(x=2.0, y=2.0, z=0.1),
                       'confidence': 0.76,
                       'properties': properties
                   }
                   detected_objects.append(obj)
               elif obj_name == 'table':
                   # Simulate detection of table
                   obj = {
                       'name': obj_name,
                       'position': Point(x=0.0, y=0.0, z=0.0),
                       'confidence': 0.92,
                       'properties': properties
                   }
                   detected_objects.append(obj)
           
           return detected_objects
       
       def create_detected_objects_message(self, detected_objects, header):
           """
           Create DetectedObjects message from detection results
           """
           msg = DetectedObjects()
           msg.header = header
           
           for obj_data in detected_objects:
               obj_msg = Object()
               obj_msg.name = obj_data['name']
               obj_msg.confidence = obj_data['confidence']
               obj_msg.pose.position = obj_data['position']
               obj_msg.pose.orientation.w = 1.0  # Default orientation
               
               # Store additional properties as JSON string
               props = obj_data['properties']
               props['3d_position'] = {
                   'x': obj_data['position'].x,
                   'y': obj_data['position'].y,
                   'z': obj_data['position'].z
               }
               obj_msg.properties = json.dumps(props)
               
               msg.objects.append(obj_msg)
           
           return msg
       
       def draw_bounding_boxes(self, image, detected_objects):
           """
           Draw bounding boxes on image for debugging
           """
           output_image = image.copy()
           
           for obj in detected_objects:
               # In a real implementation, you would have bounding box coordinates
               # For this example, we'll draw a simple indicator
               x, y = int(obj['position'].x * 100 + width/2), int(obj['position'].y * 100 + height/2)
               
               # Draw a circle at the object's position
               cv2.circle(output_image, (x, y), 10, (0, 255, 0), 2)
               
               # Draw label
               cv2.putText(output_image, obj['name'], (x - 20, y - 15), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
           
           return output_image
   
   
   def main(args=None):
       """
       Main function to run the perception module node
       """
       rclpy.init(args=args)
       
       node = PerceptionModuleNode()
       
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           node.get_logger().info('Perception Module interrupted by user')
       finally:
           node.destroy_node()
           rclpy.shutdown()
   
   
   if __name__ == '__main__':
       main()
   ```

3. Update setup.py to include the perception module:
   ```python
   # In setup.py, add to entry_points:
   'console_scripts': [
       'voice_pipeline = vla_examples.voice_pipeline:main',
       'cognitive_planner = vla_examples.cognitive_planner:main',
       'perception_module = vla_examples.perception_module:main',
   ],
   ```

### Step 2.4: Implement the System Orchestrator

1. Create the orchestrator file:
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/vla_system_orchestrator.py
   ```

2. Add the following content to `vla_system_orchestrator.py`:
   ```python
   #!/usr/bin/env python3
   """
   VLA System Orchestrator Node
   
   This node coordinates all VLA system components and manages the overall workflow.
   """
   
   import rclpy
   from rclpy.node import Node
   from rclpy.qos import QoSProfile
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist
   from vla_msgs.msg import ActionSequence, VLAAction
   from vla_msgs.srv import ExecuteAction
   from tf2_ros import TransformException
   from tf2_ros.buffer import Buffer
   from tf2_ros.transform_listener import TransformListener
   import json
   import time
   import threading
   from enum import Enum
   
   
   class SystemState(Enum):
       """Enumeration of system states"""
       IDLE = "idle"
       PROCESSING = "processing"
       EXECUTING = "executing"
       ERROR = "error"
       SAFETY_STOP = "safety_stop"
   
   
   class VLASystemOrchestratorNode(Node):
       """
       Node that orchestrates the complete VLA system
       """
       
       def __init__(self):
           super().__init__('vla_system_orchestrator')
           
           # Initialize TF2
           self.tf_buffer = Buffer()
           self.tf_listener = TransformListener(self.tf_buffer, self)
           
           # QoS profile
           self.qos_profile = QoSProfile(depth=10)
           
           # Publishers
           self.status_publisher = self.create_publisher(
               String,
               '/vla_system/status',
               self.qos_profile
           )
           
           self.cmd_vel_publisher = self.create_publisher(
               Twist,
               '/cmd_vel',
               self.qos_profile
           )
           
           # Subscribers
           self.action_sequence_subscriber = self.create_subscription(
               ActionSequence,
               '/vla/action_sequence',
               self.action_sequence_callback,
               self.qos_profile
           )
           
           # Service clients for component interaction
           self.action_executor_client = self.create_client(
               ExecuteAction,
               'execute_action'
           )
           
           # Wait for action executor service
           while not self.action_executor_client.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Action executor service not available, waiting...')
           
           # Internal state
           self.system_state = SystemState.IDLE
           self.current_action_sequence = []
           self.current_action_index = 0
           self.is_executing = False
           self.emergency_stop = False
           
           # Timer for status updates
           self.status_timer = self.create_timer(1.0, self.publish_status)
           
           self.get_logger().info('VLA System Orchestrator initialized')
       
       def action_sequence_callback(self, msg):
           """
           Handle received action sequences
           """
           self.get_logger().info(f'Received action sequence: {len(msg.actions)} actions')
           
           if self.system_state in [SystemState.ERROR, SystemState.SAFETY_STOP]:
               self.get_logger().warn('System in error state, ignoring action sequence')
               return
           
           if self.emergency_stop:
               self.get_logger().warn('Emergency stop active, ignoring action sequence')
               return
           
           # Store and begin execution
           self.current_action_sequence = msg.actions
           self.current_action_index = 0
           self.is_executing = True
           self.update_state(SystemState.EXECUTING)
           
           self.get_logger().info(f'Starting execution of {len(self.current_action_sequence)} actions')
           self.execute_next_action()
       
       def execute_next_action(self):
           """
           Execute the next action in the sequence
           """
           if self.current_action_index >= len(self.current_action_sequence):
               # Sequence completed
               self.get_logger().info('Action sequence completed')
               self.current_action_index = 0
               self.current_action_sequence = []
               self.is_executing = False
               self.update_state(SystemState.IDLE)
               return
           
           if self.emergency_stop:
               self.get_logger().info('Emergency stop during execution')
               self.update_state(SystemState.SAFETY_STOP)
               return
           
           # Get the next action
           action = self.current_action_sequence[self.current_action_index]
           self.get_logger().info(f'Executing action {self.current_action_index + 1}: {action.type}')
           
           # Create execution request
           request = ExecuteAction.Request()
           request.action = action
           
           # Execute the action asynchronously
           future = self.action_executor_client.call_async(request)
           future.add_done_callback(self.action_execution_callback)
       
       def action_execution_callback(self, future):
           """
           Handle action execution result
           """
           try:
               response = future.result()
               if response.success:
                   self.get_logger().info(f'Action executed successfully: {response.message}')
                   
                   # Move to next action
                   self.current_action_index += 1
                   self.execute_next_action()
               else:
                   self.get_logger().error(f'Action execution failed: {response.message}')
                   self.handle_action_error(response.error_code)
           except Exception as e:
               self.get_logger().error(f'Action execution callback error: {e}')
               self.handle_action_error('EXECUTION_ERROR')
       
       def handle_action_error(self, error_code):
           """
           Handle action execution errors
           """
           self.get_logger().error(f'Action error: {error_code}')
           
           # For now, go to error state
           # In a more sophisticated system, you might implement recovery strategies
           self.is_executing = False
           self.update_state(SystemState.ERROR)
       
       def update_state(self, new_state):
           """
           Update system state and log the change
           """
           old_state = self.system_state
           self.system_state = new_state
           self.get_logger().info(f'System state changed: {old_state.value} -> {new_state.value}')
       
       def publish_status(self):
           """
           Publish system status
           """
           status_msg = String()
           status_msg.data = json.dumps({
               'state': self.system_state.value,
               'executing_sequence': len(self.current_action_sequence) > 0,
               'actions_remaining': len(self.current_action_sequence) - self.current_action_index,
               'timestamp': time.time()
           })
           self.status_publisher.publish(status_msg)
       
       def emergency_stop(self):
           """
           Activate emergency stop
           """
           self.emergency_stop = True
           self.update_state(SystemState.SAFETY_STOP)
           
           # Stop any movement
           stop_cmd = Twist()
           self.cmd_vel_publisher.publish(stop_cmd)
           self.get_logger().warn('Emergency stop activated')
       
       def reset_emergency_stop(self):
           """
           Deactivate emergency stop
           """
           self.emergency_stop = False
           self.update_state(SystemState.IDLE)
           self.get_logger().info('Emergency stop deactivated')
   
   
   def main(args=None):
       """
       Main function to run the VLA system orchestrator
       """
       rclpy.init(args=args)
       
       node = VLASystemOrchestratorNode()
       
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           node.get_logger().info('VLA System Orchestrator interrupted by user')
       finally:
           node.destroy_node()
           rclpy.shutdown()
   
   
   if __name__ == '__main__':
       main()
   ```

3. Update setup.py to include the orchestrator:
   ```python
   # In setup.py, add to entry_points:
   'console_scripts': [
       'voice_pipeline = vla_examples.voice_pipeline:main',
       'cognitive_planner = vla_examples.cognitive_planner:main',
       'perception_module = vla_examples.perception_module:main',
       'vla_system_orchestrator = vla_examples.vla_system_orchestrator:main',
   ],
   ```

### Step 2.5: Build the Updated Package

1. Return to the workspace root:
   ```bash
   cd ~/vla_ws
   ```

2. Build the workspace with the new files:
   ```bash
   colcon build --packages-select vla_examples
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Phase 3: Integration Testing

### Step 3.1: Create a Launch File for Component Testing

1. Create a launch file for testing individual components:
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/launch/component_test.launch.py
   ```

2. Add the following content to the launch file:
   ```python
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
           
           # Voice pipeline node
           Node(
               package='vla_examples',
               executable='voice_pipeline',
               name='voice_pipeline',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ],
               output='screen'
           ),
           
           # Cognitive planner node
           Node(
               package='vla_examples',
               executable='cognitive_planner',
               name='cognitive_planner',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ],
               output='screen'
           ),
           
           # Perception module node
           Node(
               package='vla_examples',
               executable='perception_module',
               name='perception_module',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ],
               output='screen'
           ),
           
           # System orchestrator node
           Node(
               package='vla_examples',
               executable='vla_system_orchestrator',
               name='vla_system_orchestrator',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ],
               output='screen'
           )
       ])
   ```

### Step 3.2: Test Individual Components

1. Run the launch file to start all components:
   ```bash
   ros2 launch vla_examples component_test.launch.py
   ```

2. In a new terminal, publish a test command to the cognitive planner:
   ```bash
   ros2 topic pub /vla/natural_command std_msgs/String "data: 'move forward 1 meter'"
   ```

3. Observe the output in the terminal running the launch file to ensure all components are communicating properly.

### Step 3.3: Run Component Tests

1. Create a test file for the components:
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/test_component_integration.py
   ```

2. Add the following test code:
   ```python
   #!/usr/bin/env python3
   """
   Component Integration Test for VLA System
   """
   
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from vla_msgs.msg import ActionSequence
   import time
   
   
   class ComponentIntegrationTest(Node):
       """
       Test node to verify component integration
       """
       
       def __init__(self):
           super().__init__('component_integration_test')
           
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
           
           # Test state
           self.action_sequence_received = False
           self.test_completed = False
           self.start_time = time.time()
       
       def action_sequence_callback(self, msg):
           """
           Callback for action sequence messages
           """
           self.get_logger().info(f'Received action sequence with {len(msg.actions)} actions')
           self.action_sequence_received = True
           self.test_completed = True
       
       def run_test(self):
           """
           Run the integration test
           """
           self.get_logger().info('Starting component integration test')
           
           # Send a test command
           test_command = String()
           test_command.data = 'move forward 0.5 meters'
           self.command_publisher.publish(test_command)
           
           self.get_logger().info('Test command published, waiting for response...')
           
           # Wait for response or timeout
           timeout = time.time() + 10.0  # 10 second timeout
           while not self.test_completed and time.time() < timeout:
               time.sleep(0.1)
           
           # Report results
           if self.action_sequence_received:
               self.get_logger().info('✅ Component integration test PASSED')
               return True
           else:
               self.get_logger().info('❌ Component integration test FAILED')
               return False
   
   
   def main(args=None):
       rclpy.init(args=args)
       
       test_node = ComponentIntegrationTest()
       
       try:
           success = test_node.run_test()
           return 0 if success else 1
       except KeyboardInterrupt:
           test_node.get_logger().info('Test interrupted by user')
           return 1
       finally:
           test_node.destroy_node()
           rclpy.shutdown()
   
   
   if __name__ == '__main__':
       exit(main())
   ```

3. Run the integration test:
   ```bash
   ros2 run vla_examples test_component_integration.py
   ```

## Phase 4: Complete System Testing

### Step 4.1: Create Complete System Launch File

1. Create a launch file for the complete system:
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/launch/vla_complete_system.launch.py
   ```

2. Add the following content:
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from ament_index_python.packages import get_package_share_directory
   import os
   
   
   def generate_launch_description():
       # Launch configuration
       use_sim_time = LaunchConfiguration('use_sim_time', default='false')
       openai_api_key = LaunchConfiguration('openai_api_key', default='')
       
       return LaunchDescription([
           # Declare launch arguments
           DeclareLaunchArgument(
               'use_sim_time',
               default_value='false',
               description='Use simulation clock if true'
           ),
           DeclareLaunchArgument(
               'openai_api_key',
               default_value='',
               description='OpenAI API key'
           ),
           
           # Voice pipeline node
           Node(
               package='vla_examples',
               executable='voice_pipeline',
               name='voice_pipeline',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ],
               output='screen'
           ),
           
           # Cognitive planner node
           Node(
               package='vla_examples',
               executable='cognitive_planner',
               name='cognitive_planner',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ],
               output='screen'
           ),
           
           # Perception module node
           Node(
               package='vla_examples',
               executable='perception_module',
               name='perception_module',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ],
               output='screen'
           ),
           
           # System orchestrator node
           Node(
               package='vla_examples',
               executable='vla_system_orchestrator',
               name='vla_system_orchestrator',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ],
               output='screen'
           )
       ])
   ```

### Step 4.2: Run Complete System Test

1. First, make sure you have Gazebo running with a robot model, then run the complete system:
   ```bash
   ros2 launch vla_examples vla_complete_system.launch.py
   ```

2. In another terminal, send a more complex command:
   ```bash
   ros2 topic pub /vla/natural_command std_msgs/String "data: 'go to the table and pick up the red cup'"
   ```

3. Observe the complete flow from command processing to action execution.

## Phase 5: Simulation Testing

### Step 5.1: Set Up Gazebo Simulation

1. Create a simple world file for testing (if you haven't already):
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/worlds/simple_room.world
   ```

2. Add a simple world configuration (you can use the one from earlier in this document).

### Step 5.2: Create Simulation Launch File

1. Create a launch file for simulation:
   ```bash
   touch ~/vla_ws/src/vla_examples/vla_examples/launch/vla_simulation.launch.py
   ```

2. Add the following content:
   ```python
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
       world_file = LaunchConfiguration('world', default='simple_room.world')
       
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
           default_value='simple_room.world',
           description='World file to load in Gazebo'
       )
       
       # Launch Gazebo
       gazebo_launch = IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(gazebo_ros_package_dir, 'launch', 'gazebo.launch.py')
           ),
           launch_arguments={
               'world': PathJoinSubstitution([get_package_share_directory('vla_examples'), 'worlds', world_file]),
               'verbose': 'true'
           }.items()
       )
       
       # Robot state publisher
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           parameters=[
               {'use_sim_time': use_sim_time}
           ]
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
           # Add other components as needed
       ]
       
       # Create launch description
       ld = LaunchDescription([
           declare_use_sim_time,
           declare_world_file,
           gazebo_launch,
           robot_state_publisher
       ])
       
       # Add VLA components
       for component in vla_components:
           ld.add_action(component)
       
       return ld
   ```

### Step 5.3: Run Simulation Test

1. Launch the simulation:
   ```bash
   ros2 launch vla_examples vla_simulation.launch.py
   ```

2. Test the system with various commands in the simulation environment.

## Phase 6: Documentation and Completion

### Step 6.1: Update Package Setup

1. Ensure your setup.py file is complete:
   ```python
   from setuptools import setup
   from glob import glob
   import os
   
   package_name = 'vla_examples'
   
   setup(
       name=package_name,
       version='0.0.1',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           # Include launch files
           (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
           # Include worlds
           (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your-email@example.com',
       description='Vision-Language-Action System Examples',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'voice_pipeline = vla_examples.voice_pipeline:main',
               'cognitive_planner = vla_examples.cognitive_planner:main',
               'perception_module = vla_examples.perception_module:main',
               'vla_system_orchestrator = vla_examples.vla_system_orchestrator:main',
           ],
       },
   )
   ```

### Step 6.2: Build and Test Final System

1. Build the complete workspace:
   ```bash
   cd ~/vla_ws
   colcon build --packages-select vla_examples
   source install/setup.bash
   ```

2. Run a final integrated test:
   ```bash
   ros2 launch vla_examples vla_complete_system.launch.py
   ```

### Step 6.3: Verify All Components

1. Test the complete system flow:
   - Voice command processing
   - Cognitive planning
   - Action execution
   - System orchestration

2. Verify that all components communicate properly through ROS topics and services.

## Troubleshooting Common Issues

### Issue 1: OpenAI API Connection Errors
- **Problem**: Whisper or LLM services not responding
- **Solution**: Verify API key is set in environment variables
  ```bash
  echo $OPENAI_API_KEY
  export OPENAI_API_KEY="your-actual-api-key"
  ```

### Issue 2: Component Communication Issues
- **Problem**: Nodes not communicating properly
- **Solution**: Check topic names and ensure nodes are on the same ROS domain
  ```bash
  ros2 topic list
  ros2 node list
  ```

### Issue 3: Build Errors
- **Problem**: `colcon build` fails
- **Solution**: Check dependencies and Python package installations
  ```bash
  pip3 list | grep -i ros
  python3 -c "import openai; print('OpenAI module imported successfully')"
  ```

## Conclusion

This step-by-step guide has walked you through the complete implementation of the Vision-Language-Action (VLA) Capstone Project. You have:

1. Set up the development environment with all required dependencies
2. Implemented core components (voice pipeline, cognitive planner, perception module)
3. Created an orchestrator to coordinate all components
4. Tested the system integration both in standalone and complete system configurations
5. Verified the system functionality through simulation testing

The completed system forms a complete pipeline from natural language understanding to robotic action execution, demonstrating the integration of multiple cutting-edge technologies in a functional robotic system.
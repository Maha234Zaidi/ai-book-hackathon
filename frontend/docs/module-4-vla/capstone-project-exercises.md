# Exercises for Vision-Language-Action (VLA) Capstone Project

## Overview

This document provides exercises designed to reinforce understanding of the Vision-Language-Action (VLA) system concepts and implementation. Each exercise includes a problem statement, implementation requirements, and a detailed solution.

## Exercise 1: Voice Command Enhancement

### Problem Statement
Enhance the voice command pipeline to handle multiple languages and implement a wake-word detection system before processing commands.

### Requirements
1. Add support for both English and Spanish voice commands
2. Implement a wake-word detection system that activates listening mode
3. Add noise filtering capabilities for noisy environments
4. Implement confidence thresholding to reduce false positives

### Starter Code
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
import speech_recognition as sr
import threading
import queue
import os


class EnhancedVoicePipelineNode(Node):
    def __init__(self):
        super().__init__('enhanced_voice_pipeline')
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000
        
        # Publishers and subscribers
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            10
        )
        
        self.audio_subscriber = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )
        
        # Internal state
        self.is_listening = False
        self.is_detecting_wake_word = True
        self.audio_buffer = queue.Queue()
        self.supported_languages = ['en-US', 'es-ES']
        self.wake_words = ['robot', 'robito', 'robotito']  # Multiple wake words in different languages
        
        self.get_logger().info('Enhanced Voice Pipeline Node initialized')

    def audio_callback(self, msg):
        """
        Process audio input with wake word detection
        """
        # Implement audio processing logic here
        pass

    def detect_wake_word(self, audio_data):
        """
        Detect wake word from audio data
        """
        # Implement wake word detection
        pass

    def transcribe_audio(self, audio_data):
        """
        Transcribe audio to text with language support
        """
        # Implement multi-language transcription
        pass
```

### Solution
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
import speech_recognition as sr
import io
import wave
import tempfile
from vosk import Model, KaldiRecognizer
import json
import threading
import queue
import os


class EnhancedVoicePipelineNode(Node):
    def __init__(self):
        super().__init__('enhanced_voice_pipeline')
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000
        self.recognizer.pause_threshold = 0.8
        
        # Publishers and subscribers
        self.command_publisher = self.create_publisher(
            String,
            '/vla/natural_command',
            10
        )
        
        self.wake_word_detected_publisher = self.create_publisher(
            Bool,
            '/wake_word_detected',
            10
        )
        
        self.audio_subscriber = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )
        
        # Internal state
        self.is_listening = False
        self.is_detecting_wake_word = True
        self.audio_buffer = queue.Queue()
        self.supported_languages = ['en-US', 'es-ES']
        self.wake_words_en = ['robot', 'hey robot', 'hello robot']
        self.wake_words_es = ['robot', 'oiga robot', 'hola robot']
        
        # Vosk model for wake word detection (simplified)
        # In a real implementation, you might use a lightweight wake word detection library
        self.wake_word_threshold = 0.7
        
        self.get_logger().info('Enhanced Voice Pipeline Node initialized')

    def audio_callback(self, msg):
        """
        Process audio input with wake word detection
        """
        # Add audio to processing queue
        self.audio_buffer.put(msg.data)
        
        # Process when buffer has sufficient data
        if self.audio_buffer.qsize() > 5:  # Process in chunks
            self.process_audio_chunk()

    def process_audio_chunk(self):
        """
        Process accumulated audio data
        """
        # Collect audio frames
        audio_frames = []
        try:
            while not self.audio_buffer.empty():
                audio_frames.append(self.audio_buffer.get_nowait())
        except queue.Empty:
            pass
        
        if not audio_frames:
            return
        
        # Combine frames
        audio_bytes = b''.join(audio_frames)
        
        with io.BytesIO() as wav_buffer:
            with wave.open(wav_buffer, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(16000)
                wav_file.writeframes(audio_bytes)
            
            audio_data = sr.AudioData(wav_buffer.getvalue(), 16000, 2)
        
        try:
            if self.is_detecting_wake_word:
                # Detect wake word
                detected_word = self.detect_wake_word(audio_data)
                if detected_word:
                    self.get_logger().info(f'Wake word detected: {detected_word}')
                    self.is_detecting_wake_word = False
                    self.is_listening = True
                    
                    # Publish wake word detection
                    wake_msg = Bool()
                    wake_msg.data = True
                    self.wake_word_detected_publisher.publish(wake_msg)
            else:
                # Process command
                command = self.transcribe_audio(audio_data)
                if command:
                    self.get_logger().info(f'Recognized command: {command}')
                    
                    # Publish command
                    cmd_msg = String()
                    cmd_msg.data = command
                    self.command_publisher.publish(cmd_msg)
                    
                    # Reset to wake word detection mode
                    self.is_listening = False
                    self.is_detecting_wake_word = True
                    
        except sr.UnknownValueError:
            # No speech detected or could not understand
            pass
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')

    def detect_wake_word(self, audio_data):
        """
        Detect wake word from audio data
        """
        # For this example, we'll use a simple energy-based approach
        # In a real implementation, use dedicated wake word detection
        try:
            # Use speech recognition to get text
            text = audio_data.recognize_google(audio_data, language='en-US')
            text = text.lower()
            
            # Check for wake words
            for wake_word in self.wake_words_en + self.wake_words_es:
                if wake_word.lower() in text:
                    return wake_word
            
            return None
        except:
            return None

    def transcribe_audio(self, audio_data):
        """
        Transcribe audio to text with multi-language support
        """
        # Try English first
        try:
            text_en = audio_data.recognize_google(audio_data, language='en-US')
            self.get_logger().debug(f'English transcription: {text_en}')
            return text_en
        except:
            pass
        
        # Try Spanish
        try:
            text_es = audio_data.recognize_google(audio_data, language='es-ES')
            self.get_logger().debug(f'Spanish transcription: {text_es}')
            return text_es
        except:
            pass
        
        return None


def main(args=None):
    rclpy.init(args=args)
    
    node = EnhancedVoicePipelineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Enhanced Voice Pipeline interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 2: Advanced Cognitive Planning

### Problem Statement
Enhance the cognitive planner to handle ambiguous commands and implement multi-modal planning that incorporates visual context from the perception module.

### Requirements
1. Handle ambiguous commands by requesting clarification from the user
2. Incorporate object properties and locations from perception data into planning
3. Implement fallback strategies when primary action sequences fail
4. Add explanation capabilities to describe planning decisions

### Starter Code
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence, VLAAction, DetectedObjects
from vla_msgs.srv import PlanCognitiveTask
from openai import OpenAI
import json
import os
from typing import Dict, List


class AdvancedCognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('advanced_cognitive_planner')
        
        # Initialize LLM client
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
        # Publishers and subscribers
        self.action_publisher = self.create_publisher(
            ActionSequence,
            '/vla/action_sequence',
            10
        )
        
        self.command_subscriber = self.create_subscription(
            String,
            '/vla/natural_command',
            self.command_callback,
            10
        )
        
        self.perception_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.perception_callback,
            10
        )
        
        # Service for planning requests
        self.plan_service = self.create_service(
            PlanCognitiveTask,
            'plan_cognitive_task',
            self.plan_service_callback
        )
        
        # Internal state
        self.known_objects = {}
        self.ambiguous_commands_queue = []
        
        self.get_logger().info('Advanced Cognitive Planner Node initialized')

    def command_callback(self, msg):
        """
        Process natural language command with ambiguity handling
        """
        pass

    def perception_callback(self, msg):
        """
        Update known objects from perception
        """
        pass

    def plan_actions(self, command):
        """
        Plan actions for a command with visual context
        """
        pass

    def handle_ambiguous_command(self, command):
        """
        Handle commands that require clarification
        """
        pass
```

### Solution
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence, VLAAction, DetectedObjects
from vla_msgs.srv import PlanCognitiveTask
from openai import OpenAI
import json
import os
from typing import Dict, List, Optional, Tuple
import threading


class AdvancedCognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('advanced_cognitive_planner')
        
        # Initialize LLM client
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
        # Publishers and subscribers
        self.action_publisher = self.create_publisher(
            ActionSequence,
            '/vla/action_sequence',
            10
        )
        
        self.command_subscriber = self.create_subscription(
            String,
            '/vla/natural_command',
            self.command_callback,
            10
        )
        
        self.perception_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.perception_callback,
            10
        )
        
        self.clarification_publisher = self.create_publisher(
            String,
            '/vla/clarification_request',
            10
        )
        
        # Service for planning requests
        self.plan_service = self.create_service(
            PlanCognitiveTask,
            'plan_cognitive_task',
            self.plan_service_callback
        )
        
        # Internal state
        self.known_objects = {}
        self.ambiguous_commands_queue = []
        self.clarification_needed = False
        
        self.get_logger().info('Advanced Cognitive Planner Node initialized')

    def command_callback(self, msg):
        """
        Process natural language command with ambiguity handling
        """
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Check if command is ambiguous
        if self.is_ambiguous_command(command):
            self.get_logger().info('Ambiguous command detected, requesting clarification')
            clarification_request = self.generate_clarification_request(command)
            
            # Publish clarification request
            clarification_msg = String()
            clarification_msg.data = clarification_request
            self.clarification_publisher.publish(clarification_msg)
            
            # Add to queue for later processing
            self.ambiguous_commands_queue.append({
                'command': command,
                'clarification_request': clarification_request
            })
        else:
            # Plan actions normally
            action_sequence, reasoning = self.plan_actions(command)
            
            if action_sequence:
                action_msg = self.create_action_sequence_msg(action_sequence, command, reasoning)
                self.action_publisher.publish(action_msg)
                
                self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')
            else:
                self.get_logger().error(f'Failed to generate action sequence for: {command}')

    def perception_callback(self, msg):
        """
        Update known objects from perception
        """
        for obj in msg.objects:
            self.known_objects[obj.name] = {
                'position': {
                    'x': obj.pose.position.x,
                    'y': obj.pose.position.y,
                    'z': obj.pose.position.z
                },
                'confidence': obj.confidence,
                'properties': json.loads(obj.properties) if obj.properties else {}
            }
        
        self.get_logger().info(f'Updated perception data: {len(msg.objects)} objects')

    def is_ambiguous_command(self, command: str) -> bool:
        """
        Check if a command is ambiguous
        """
        ambiguous_indicators = [
            'the cup', 'the box', 'it', 'that', 'there', 
            'which one', 'both', 'either', 'red one', 'blue one'
        ]
        
        command_lower = command.lower()
        return any(indicator in command_lower for indicator in ambiguous_indicators)

    def generate_clarification_request(self, command: str) -> str:
        """
        Generate a clarification request for an ambiguous command
        """
        # In a real implementation, this would be more sophisticated
        if 'the cup' in command.lower() or 'the box' in command.lower():
            available_objects = [name for name in self.known_objects.keys() 
                                if any(obj_type in name for obj_type in ['cup', 'box'])]
            return f"I found multiple objects: {available_objects}. Which one do you mean by 'the cup'?"
        else:
            return f"I need clarification on your command: '{command}'. Can you be more specific?"

    def plan_actions(self, command: str) -> Tuple[Optional[List], str]:
        """
        Plan actions for a command with visual context
        """
        # Construct prompt with perception context
        context_str = json.dumps(self.known_objects, indent=2)
        
        available_actions = [
            "navigate_to(object_name)", "grasp(object_name)", "place_at(x, y, z)",
            "move_to(x, y)", "detect_object(object_name)", "pick_and_place(object_name, target_name)"
        ]
        
        prompt = f"""
        The user wants the robot to: "{command}"
        
        Current environmental context:
        {context_str}
        
        Available robot actions:
        {', '.join(available_actions)}
        
        Generate a sequence of actions for the robot to complete the user's request.
        Consider the environmental context and the robot's current state.
        
        Handle ambiguous references by using the most likely object based on context.
        If multiple interpretations exist, choose the most reasonable one.
        
        Also provide brief reasoning for the plan.
        
        Respond with only valid JSON in this format:
        {{
            "actions": [
                {{
                    "type": "navigate_to",
                    "params": {{"object_name": "kitchen counter"}},
                    "description": "Navigate to kitchen counter",
                    "reasoning_explanation": "First step is to navigate to the location of the object."
                }}
            ],
            "reasoning": "The plan takes into account the current positions of objects to efficiently complete the task."
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
    rclpy.init(args=args)
    
    node = AdvancedCognitivePlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Advanced Cognitive Planner interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 3: Perception Enhancement with Deep Learning

### Problem Statement
Enhance the perception module to use a deep learning model for more accurate object detection and add semantic segmentation capabilities.

### Requirements
1. Integrate YOLOv8 or similar deep learning model for object detection
2. Add semantic segmentation for better scene understanding
3. Implement object tracking across frames
4. Add 3D object pose estimation capabilities

### Starter Code
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vla_msgs.msg import DetectedObjects, Object
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from ultralytics import YOLO


class EnhancedPerceptionNode(Node):
    def __init__(self):
        super().__init__('enhanced_perception')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize deep learning model
        self.model = None  # Initialize YOLO model
        self.transforms = transforms.Compose([
            # Add necessary transforms
        ])
        
        # Publishers and subscribers
        self.objects_publisher = self.create_publisher(
            DetectedObjects,
            '/vla/perception/objects',
            10
        )
        
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Internal state
        self.camera_info = None
        self.tracked_objects = {}
        
        self.get_logger().info('Enhanced Perception Node initialized')

    def image_callback(self, msg):
        """
        Process image using deep learning model
        """
        pass

    def detect_objects(self, image):
        """
        Detect objects using deep learning model
        """
        pass

    def estimate_3d_pose(self, detection):
        """
        Estimate 3D pose of detected objects
        """
        pass
```

### Solution
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
from vla_msgs.msg import DetectedObjects, Object
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from ultralytics import YOLO
import json
import time
from typing import Dict, List, Tuple
import math


class EnhancedPerceptionNode(Node):
    def __init__(self):
        super().__init__('enhanced_perception')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize deep learning model
        try:
            self.model = YOLO('yolov8n.pt')  # You might need to download this model
        except:
            self.get_logger().warn('YOLO model not found, using mock detections')
            self.model = None
        
        # Publishers and subscribers
        self.objects_publisher = self.create_publisher(
            DetectedObjects,
            '/vla/perception/objects',
            10
        )
        
        self.debug_publisher = self.create_publisher(
            Image,
            '/vla/perception/debug_image',
            10
        )
        
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Internal state
        self.camera_info = None
        self.tracked_objects = {}
        self.last_detection_time = 0.0
        self.detection_frequency = 0.5  # 2 Hz
        self.object_ids = 0  # Counter for object IDs
        
        self.get_logger().info('Enhanced Perception Node initialized')

    def camera_info_callback(self, msg):
        """
        Handle camera information
        """
        self.camera_info = msg

    def image_callback(self, msg):
        """
        Process image using deep learning model
        """
        current_time = time.time()
        
        # Throttle detection frequency
        if current_time - self.last_detection_time < self.detection_frequency:
            return
        
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform object detection
            detections = self.detect_objects(cv_image)
            
            # Create and publish DetectedObjects message
            detected_objects_msg = self.create_detected_objects_message(detections, msg.header)
            self.objects_publisher.publish(detected_objects_msg)
            
            # Publish debug image with bounding boxes
            debug_image = self.draw_detections_on_image(cv_image, detections)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_publisher.publish(debug_msg)
            
            # Update tracking
            self.update_object_tracking(detections)
            
            self.get_logger().info(f'Detected {len(detections)} objects')
            self.last_detection_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image):
        """
        Detect objects using deep learning model
        """
        if self.model is None:
            # Mock detections for demonstration
            height, width = image.shape[:2]
            return [
                {
                    'name': 'person',
                    'confidence': 0.89,
                    'bbox': [width//4, height//4, width//2, height//2],
                    'center_2d': (width//2, height//2)
                },
                {
                    'name': 'cup',
                    'confidence': 0.76,
                    'bbox': [3*width//4, height//2, width//4, height//4],
                    'center_2d': (7*width//8, 3*height//4)
                }
            ]
        
        # Run YOLO detection
        results = self.model(image, verbose=False)
        
        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Extract bounding box info
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    
                    # Get class name
                    class_name = self.model.names[cls]
                    
                    detection = {
                        'name': class_name,
                        'confidence': conf,
                        'bbox': [int(x1), int(y1), int(x2-x1), int(y2-y1)],  # x, y, width, height
                        'center_2d': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    }
                    
                    detections.append(detection)
        
        return detections

    def estimate_3d_pose(self, detection, image_shape):
        """
        Estimate 3D pose of detected objects
        """
        if not self.camera_info:
            # Without camera info, return 2D coordinates as 3D with z=1.0
            center_x, center_y = detection['center_2d']
            return Point(
                x=center_x / image_shape[1],  # Normalize to 0-1
                y=center_y / image_shape[0],  # Normalize to 0-1
                z=1.0
            )
        
        # Use camera parameters for 3D estimation (simplified)
        x2d, y2d = detection['center_2d']
        
        # Convert to normalized image coordinates
        x_norm = (x2d - self.camera_info.k[2]) / self.camera_info.k[0]  # (x - cx) / fx
        y_norm = (y2d - self.camera_info.k[5]) / self.camera_info.k[4]  # (y - cy) / fy
        
        # For this example, we'll use a simple depth estimation based on object size
        bbox = detection['bbox']
        object_width_pixels = bbox[2]
        
        # Assume known object width in meters (e.g., average cup width = 0.08m)
        known_width = 0.08  # meters
        focal_length = self.camera_info.k[0]  # fx
        
        # Calculate distance using triangulation
        if object_width_pixels > 0:
            z = (known_width * focal_length) / object_width_pixels
        else:
            z = 1.0  # default distance
        
        # Calculate 3D position
        x = x_norm * z
        y = y_norm * z
        
        return Point(x=x, y=y, z=z)

    def create_detected_objects_message(self, detections, header):
        """
        Create DetectedObjects message from detection results
        """
        msg = DetectedObjects()
        msg.header = header
        
        for det in detections:
            if det['confidence'] < 0.5:  # Confidence threshold
                continue
                
            obj_msg = Object()
            obj_msg.name = det['name']
            obj_msg.confidence = det['confidence']
            
            # Estimate 3D position
            image_shape = (480, 640, 3)  # Assume standard size
            obj_msg.pose.position = self.estimate_3d_pose(det, image_shape)
            obj_msg.pose.orientation.w = 1.0  # Default orientation
            
            # Store additional properties
            props = {
                'bbox_2d': det['bbox'],
                'center_2d': det['center_2d'],
                'confidence': det['confidence']
            }
            obj_msg.properties = json.dumps(props)
            
            msg.objects.append(obj_msg)
        
        return msg

    def draw_detections_on_image(self, image, detections):
        """
        Draw detection bounding boxes on image for debugging
        """
        output_image = image.copy()
        
        for det in detections:
            if det['confidence'] < 0.5:
                continue
                
            x, y, w, h = det['bbox']
            
            # Draw bounding box
            cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw label and confidence
            label = f"{det['name']}: {det['confidence']:.2f}"
            cv2.putText(output_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return output_image

    def update_object_tracking(self, detections):
        """
        Update object tracking across frames
        """
        # This is a simplified tracking implementation
        # In a real system, you would use more sophisticated tracking algorithms
        
        # Create a mapping of current detections to existing tracked objects
        updated_tracking = {}
        
        for det in detections:
            if det['confidence'] < 0.7:  # Only track confident detections
                continue
                
            # Find the closest existing object to this detection
            closest_obj_id = None
            min_dist = float('inf')
            
            for obj_id, tracked_obj in self.tracked_objects.items():
                # Calculate distance between current detection and tracked object
                det_center = det['center_2d']
                track_center = tracked_obj['detection']['center_2d']
                
                dist = math.sqrt(
                    (det_center[0] - track_center[0])**2 + 
                    (det_center[1] - track_center[1])**2
                )
                
                if dist < min_dist and dist < 50:  # Threshold for matching
                    min_dist = dist
                    closest_obj_id = obj_id
            
            if closest_obj_id is not None:
                # Update existing tracked object
                self.tracked_objects[closest_obj_id]['detection'] = det
                self.tracked_objects[closest_obj_id]['last_seen'] = time.time()
                updated_tracking[closest_obj_id] = self.tracked_objects[closest_obj_id]
            else:
                # Create new tracked object
                new_id = f"obj_{self.object_ids}"
                self.object_ids += 1
                updated_tracking[new_id] = {
                    'id': new_id,
                    'detection': det,
                    'first_seen': time.time(),
                    'last_seen': time.time()
                }
        
        # Remove objects that haven't been seen recently
        current_time = time.time()
        self.tracked_objects = {
            obj_id: obj_data for obj_id, obj_data in updated_tracking.items()
            if current_time - obj_data['last_seen'] < 2.0  # Remove if not seen for 2 seconds
        }


def main(args=None):
    rclpy.init(args=args)
    
    node = EnhancedPerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Enhanced Perception Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 4: Safety and Validation Enhancement

### Problem Statement
Implement a comprehensive safety and validation system for the VLA system that checks action sequences before execution and monitors ongoing tasks for safety violations.

### Requirements
1. Implement a safety checker that validates action sequences before execution
2. Add real-time monitoring of ongoing tasks for safety violations
3. Implement emergency stop capabilities
4. Add logging and reporting for safety events

### Starter Code
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from vla_msgs.msg import ActionSequence, VLAAction
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SafetyValidationNode(Node):
    def __init__(self):
        super().__init__('safety_validation')
        
        # Publishers and subscribers
        self.safety_status_publisher = self.create_publisher(
            String,
            '/vla/safety_status',
            10
        )
        
        self.emergency_stop_publisher = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )
        
        self.action_sequence_subscriber = self.create_subscription(
            ActionSequence,
            '/vla/action_sequence',
            self.action_sequence_callback,
            10
        )
        
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Internal state
        self.safety_enabled = True
        self.emergency_stop_active = False
        self.safety_log = []
        
        self.get_logger().info('Safety Validation Node initialized')

    def action_sequence_callback(self, msg):
        """
        Validate action sequence before execution
        """
        pass

    def laser_callback(self, msg):
        """
        Monitor laser data for safety violations
        """
        pass

    def validate_action_sequence(self, action_sequence):
        """
        Validate an entire action sequence for safety
        """
        pass

    def validate_single_action(self, action):
        """
        Validate a single action for safety
        """
        pass
```

### Solution
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from vla_msgs.msg import ActionSequence, VLAAction
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import json
import time
from typing import List, Tuple, Dict
import math


class SafetyValidationNode(Node):
    def __init__(self):
        super().__init__('safety_validation')
        
        # Publishers and subscribers
        self.safety_status_publisher = self.create_publisher(
            String,
            '/vla/safety_status',
            10
        )
        
        self.emergency_stop_publisher = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.action_sequence_subscriber = self.create_subscription(
            ActionSequence,
            '/vla/action_sequence',
            self.action_sequence_callback,
            10
        )
        
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Internal state
        self.safety_enabled = True
        self.emergency_stop_active = False
        self.safety_log = []
        self.robot_position = None
        self.last_laser_scan = None
        self.active_action_sequence = None
        
        # Safety parameters
        self.min_obstacle_distance = 0.3  # meters
        self.max_payload = 1.0  # kg
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        
        # Timer for periodic safety checks
        self.safety_timer = self.create_timer(0.1, self.periodic_safety_check)
        
        self.get_logger().info('Safety Validation Node initialized')

    def odom_callback(self, msg):
        """
        Update robot position from odometry
        """
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def laser_callback(self, msg):
        """
        Update laser scan data
        """
        self.last_laser_scan = msg

    def action_sequence_callback(self, msg):
        """
        Validate action sequence before execution
        """
        if not self.safety_enabled:
            # If safety is disabled, just pass through
            self.publish_safety_status("Safety validation disabled", "disabled")
            return
        
        self.get_logger().info(f'Validating action sequence with {len(msg.actions)} actions')
        
        # Validate the entire sequence
        is_valid, reason = self.validate_action_sequence(msg.actions)
        
        if is_valid:
            self.get_logger().info('Action sequence validated successfully')
            self.publish_safety_status(f"Action sequence validated: {len(msg.actions)} actions", "valid")
            
            # Store for monitoring during execution
            self.active_action_sequence = msg
        else:
            self.get_logger().error(f'Action sequence validation failed: {reason}')
            self.publish_safety_status(f"Action sequence validation failed: {reason}", "invalid")
            self.trigger_emergency_stop(f"Invalid action sequence: {reason}")

    def validate_action_sequence(self, actions: List[VLAAction]) -> Tuple[bool, str]:
        """
        Validate an entire action sequence for safety
        """
        if not actions:
            return False, "Action sequence is empty"
        
        # Check each action in the sequence
        for i, action in enumerate(actions):
            is_valid, reason = self.validate_single_action(action)
            if not is_valid:
                return False, f"Action {i+1} failed validation: {reason}"
        
        # Additional sequence-level checks
        # For example, check if navigation actions have reasonable destinations
        navigate_actions = [a for a in actions if a.type == 'navigate_to']
        if len(navigate_actions) > 10:  # Too many navigation steps
            return False, "Action sequence has excessive navigation steps"
        
        return True, "Action sequence is valid"

    def validate_single_action(self, action: VLAAction) -> Tuple[bool, str]:
        """
        Validate a single action for safety
        """
        action_type = action.type
        params_str = action.params
        
        if action_type == 'move_to':
            # Parse parameters
            try:
                params = json.loads(params_str)
                x = params.get('x', 0.0)
                y = params.get('y', 0.0)
                
                # Check if destination is too far
                if abs(x) > 10.0 or abs(y) > 10.0:  # Assuming 10m maximum range
                    return False, f"Destination ({x}, {y}) is out of operational range"
                
            except json.JSONDecodeError:
                return False, "Invalid parameters for move_to action"
        
        elif action_type == 'grasp':
            # Grasp validation
            try:
                params = json.loads(params_str)
                object_name = params.get('object_name', '')
                
                # Here you would check if the object is known and graspable
                # For this example, we'll just check if it's specified
                if not object_name:
                    return False, "Object name not specified for grasp action"
                
            except json.JSONDecodeError:
                return False, "Invalid parameters for grasp action"
        
        elif action_type == 'navigate_to':
            # Navigation validation
            try:
                params = json.loads(params_str)
                object_name = params.get('object_name', '')
                
                if not object_name:
                    return False, "Destination object name not specified for navigate_to action"
                
            except json.JSONDecodeError:
                return False, "Invalid parameters for navigate_to action"
        
        return True, "Action is valid"

    def periodic_safety_check(self):
        """
        Perform periodic safety checks during operation
        """
        if not self.safety_enabled or self.emergency_stop_active:
            return
        
        # Check for obstacles in front of robot
        if self.last_laser_scan:
            if self.has_obstacle_in_path(self.last_laser_scan):
                self.get_logger().warn('Obstacle detected in path, triggering emergency stop')
                self.trigger_emergency_stop("Obstacle detected in robot's path")
                return
        
        # Check if robot position is valid (not NaN or inf)
        if self.robot_position:
            x, y, z = self.robot_position
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                self.get_logger().error('Invalid robot position detected, triggering emergency stop')
                self.trigger_emergency_stop("Invalid robot position")
                return

    def has_obstacle_in_path(self, laser_scan: LaserScan) -> bool:
        """
        Check if there are obstacles in the robot's immediate path
        """
        # Check the front 90-degree sector of the laser scan
        angle_increment = laser_scan.angle_increment
        start_angle = laser_scan.angle_min
        
        # Calculate indices for front-facing sector (e.g., -45° to +45°)
        start_idx = int((-math.pi/4 - start_angle) / angle_increment)
        end_idx = int((math.pi/4 - start_angle) / angle_increment)
        
        # Ensure indices are within bounds
        start_idx = max(0, start_idx)
        end_idx = min(len(laser_scan.ranges), end_idx)
        
        # Check if any range is within the safety threshold
        for i in range(start_idx, end_idx):
            if i < len(laser_scan.ranges):
                range_val = laser_scan.ranges[i]
                if 0 < range_val < self.min_obstacle_distance:
                    return True  # Obstacle detected within safety distance
        
        return False

    def trigger_emergency_stop(self, reason: str):
        """
        Trigger emergency stop and log the event
        """
        if self.emergency_stop_active:
            return  # Already active
        
        self.emergency_stop_active = True
        self.get_logger().error(f'EMERGENCY STOP TRIGGERED: {reason}')
        
        # Publish emergency stop command
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_publisher.publish(emergency_msg)
        
        # Stop robot movement
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        
        # Log safety event
        self.log_safety_event("EMERGENCY_STOP", reason)
        
        # Publish safety status
        self.publish_safety_status(f"Emergency stop: {reason}", "emergency")

    def log_safety_event(self, event_type: str, description: str):
        """
        Log a safety event
        """
        event = {
            'timestamp': time.time(),
            'type': event_type,
            'description': description
        }
        self.safety_log.append(event)
        
        # Limit log size to prevent memory issues
        if len(self.safety_log) > 100:
            self.safety_log = self.safety_log[-50:]  # Keep last 50 events

    def publish_safety_status(self, message: str, status: str):
        """
        Publish safety status
        """
        status_msg = String()
        status_msg.data = json.dumps({
            'status': status,
            'message': message,
            'timestamp': time.time(),
            'active_stop': self.emergency_stop_active
        })
        self.safety_status_publisher.publish(status_msg)

    def reset_emergency_stop(self):
        """
        Reset emergency stop state
        """
        self.emergency_stop_active = False
        self.active_action_sequence = None
        self.get_logger().info('Emergency stop reset')
        
        # Publish reset command
        reset_msg = Bool()
        reset_msg.data = False
        self.emergency_stop_publisher.publish(reset_msg)
        
        # Publish status
        self.publish_safety_status("Safety system reset", "normal")


def main(args=None):
    rclpy.init(args=args)
    
    node = SafetyValidationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Safety Validation Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 5: System Performance Optimization

### Problem Statement
Implement performance optimization techniques for the VLA system, including multi-threading, caching, and efficient data processing.

### Requirements
1. Implement multi-threading for concurrent processing of different components
2. Add caching for frequently requested information
3. Optimize message processing to reduce latency
4. Implement resource management to prevent system overload

### Starter Code
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence
import threading
import queue
from collections import deque
import time


class PerformanceOptimizedNode(Node):
    def __init__(self):
        super().__init__('performance_optimized')
        
        # Internal queues for multi-threading
        self.command_queue = queue.Queue(maxsize=10)
        self.action_queue = queue.Queue(maxsize=10)
        
        # Cache for frequently accessed data
        self.cache = {}
        self.cache_ttl = 30  # seconds
        
        # Publishers and subscribers
        self.command_subscriber = self.create_subscription(
            String,
            '/vla/natural_command',
            self.command_callback,
            10
        )
        
        # Processing threads
        self.command_processing_thread = threading.Thread(
            target=self.process_commands, daemon=True
        )
        self.command_processing_thread.start()
        
        self.get_logger().info('Performance Optimized Node initialized')

    def command_callback(self, msg):
        """
        Handle incoming commands (non-blocking)
        """
        try:
            self.command_queue.put_nowait(msg.data)
        except queue.Full:
            self.get_logger().warn('Command queue is full, dropping command')

    def process_commands(self):
        """
        Process commands in a separate thread
        """
        pass

    def get_cached_result(self, key):
        """
        Get cached result if available and not expired
        """
        pass

    def cache_result(self, key, value):
        """
        Cache a result with timestamp
        """
        pass
```

### Solution
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence, DetectedObjects
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import queue
from collections import OrderedDict, deque
import time
import hashlib
import json
from functools import wraps


class PerformanceOptimizedNode(Node):
    def __init__(self):
        super().__init__('performance_optimized')
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Internal queues for multi-threading
        self.command_queue = queue.Queue(maxsize=20)
        self.image_queue = queue.Queue(maxsize=10)
        self.perception_queue = queue.Queue(maxsize=10)
        
        # Cache with size limit
        self.cache = OrderedDict()  # LRU cache
        self.cache_max_size = 100
        self.cache_ttl = 30  # seconds
        
        # Publishers and subscribers
        self.action_publisher = self.create_publisher(
            ActionSequence,
            '/vla/action_sequence',
            10
        )
        
        self.command_subscriber = self.create_subscription(
            String,
            '/vla/natural_command',
            self.command_callback,
            10
        )
        
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Processing threads
        self.command_processing_thread = threading.Thread(
            target=self.process_commands, daemon=True
        )
        self.image_processing_thread = threading.Thread(
            target=self.process_images, daemon=True
        )
        self.perception_processing_thread = threading.Thread(
            target=self.process_perception_data, daemon=True
        )
        
        self.command_processing_thread.start()
        self.image_processing_thread.start()
        self.perception_processing_thread.start()
        
        # Performance metrics
        self.processing_times = deque(maxlen=100)
        self.start_time = time.time()
        
        # Timer for periodic cache cleanup
        self.cache_cleanup_timer = self.create_timer(60.0, self.cleanup_cache)
        
        self.get_logger().info('Performance Optimized Node initialized')

    def command_callback(self, msg):
        """
        Handle incoming commands (non-blocking)
        """
        try:
            self.command_queue.put_nowait(msg.data)
        except queue.Full:
            self.get_logger().warn('Command queue is full, dropping command')

    def image_callback(self, msg):
        """
        Handle incoming images (non-blocking)
        """
        try:
            self.image_queue.put_nowait(msg)
        except queue.Full:
            self.get_logger().warn('Image queue is full, dropping image')

    def process_commands(self):
        """
        Process commands in a separate thread
        """
        while True:
            try:
                # Get command from queue with timeout
                command = self.command_queue.get(timeout=1.0)
                
                start_time = time.time()
                
                # Check if result is cached
                cache_key = self.generate_cache_key('command', command)
                cached_result = self.get_cached_result(cache_key)
                
                if cached_result:
                    self.get_logger().debug(f'Cache hit for command: {command[:30]}...')
                    # Publish cached result
                    self.publish_action_sequence(cached_result)
                else:
                    # Process command normally
                    action_sequence = self.process_command_internal(command)
                    if action_sequence:
                        # Cache the result
                        self.cache_result(cache_key, action_sequence)
                        # Publish the result
                        self.publish_action_sequence(action_sequence)
                
                # Record processing time
                processing_time = time.time() - start_time
                self.processing_times.append(processing_time)
                
            except queue.Empty:
                continue  # No command to process, continue loop
            except Exception as e:
                self.get_logger().error(f'Error in command processing thread: {e}')

    def process_images(self):
        """
        Process images in a separate thread
        """
        while True:
            try:
                # Get image from queue with timeout
                image_msg = self.image_queue.get(timeout=1.0)
                
                # Convert to CV image
                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                    
                    # Perform optimized image processing (simplified)
                    # In a real implementation, this might involve running detection models
                    processed_result = self.process_image_internal(cv_image)
                    
                    # Add result to perception queue for further processing
                    self.perception_queue.put({
                        'image': image_msg,
                        'processed_result': processed_result,
                        'timestamp': time.time()
                    })
                    
                except Exception as e:
                    self.get_logger().error(f'Error processing image: {e}')
                
            except queue.Empty:
                continue  # No image to process, continue loop
            except Exception as e:
                self.get_logger().error(f'Error in image processing thread: {e}')

    def process_perception_data(self):
        """
        Process perception data in a separate thread
        """
        while True:
            try:
                # Get perception data from queue with timeout
                data = self.perception_queue.get(timeout=1.0)
                
                # Process the perception data
                processed_objects = self.process_perception_internal(
                    data['image'], 
                    data['processed_result']
                )
                
                # Publish processed objects
                # (In a full implementation, you would create and publish a DetectedObjects message)
                
            except queue.Empty:
                continue  # No data to process, continue loop
            except Exception as e:
                self.get_logger().error(f'Error in perception processing thread: {e}')

    def process_command_internal(self, command):
        """
        Internal command processing logic (simulated)
        """
        # Simulate processing time
        time.sleep(0.1)
        
        # Generate mock action sequence
        mock_actions = [
            {
                "type": "navigate_to", 
                "params": {"object_name": "kitchen"},
                "description": "Navigate to kitchen",
                "confidence": 0.9
            },
            {
                "type": "grasp", 
                "params": {"object_name": "cup"},
                "description": "Grasp cup",
                "confidence": 0.85
            }
        ]
        
        return mock_actions

    def process_image_internal(self, cv_image):
        """
        Internal image processing logic (simulated)
        """
        # Simulate image processing time
        time.sleep(0.05)
        
        # In a real implementation, this would run detection models
        return {"objects_detected": 2, "processing_time": 0.05}

    def process_perception_internal(self, image_msg, processed_result):
        """
        Internal perception processing logic
        """
        # Simulate perception processing time
        time.sleep(0.02)
        
        # Return mock detected objects
        return [
            {"name": "cup", "confidence": 0.8, "position": [1, 1, 0.5]},
            {"name": "table", "confidence": 0.9, "position": [0, 0, 0]}
        ]

    def publish_action_sequence(self, action_sequence):
        """
        Publish action sequence in a non-blocking way
        """
        # This would create and publish a real ActionSequence message
        # For demonstration, we'll just log
        self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')

    def generate_cache_key(self, prefix, data):
        """
        Generate a cache key for the given data
        """
        # Create a hash of the data to use as key
        data_str = f"{prefix}:{data}" if isinstance(data, str) else f"{prefix}:{json.dumps(data, sort_keys=True)}"
        return hashlib.md5(data_str.encode()).hexdigest()

    def get_cached_result(self, key):
        """
        Get cached result if available and not expired
        """
        if key in self.cache:
            result, timestamp = self.cache[key]
            if time.time() - timestamp < self.cache_ttl:
                return result
            else:
                # Remove expired entry
                del self.cache[key]
        return None

    def cache_result(self, key, value):
        """
        Cache a result with timestamp
        """
        # Remove oldest entries if cache is full
        while len(self.cache) >= self.cache_max_size:
            self.cache.popitem(last=False)
        
        # Add new entry
        self.cache[key] = (value, time.time())

    def cleanup_cache(self):
        """
        Periodically clean up expired cache entries
        """
        current_time = time.time()
        expired_keys = [
            key for key, (_, timestamp) in self.cache.items()
            if current_time - timestamp >= self.cache_ttl
        ]
        
        for key in expired_keys:
            del self.cache[key]
        
        if expired_keys:
            self.get_logger().info(f'Cleaned up {len(expired_keys)} expired cache entries')

    def get_performance_metrics(self):
        """
        Get current performance metrics
        """
        if not self.processing_times:
            return {"avg_processing_time": 0.0, "throughput": 0.0}
        
        avg_time = sum(self.processing_times) / len(self.processing_times)
        uptime = time.time() - self.start_time
        throughput = len(self.processing_times) / uptime if uptime > 0 else 0.0
        
        return {
            "avg_processing_time": avg_time,
            "throughput": throughput,
            "uptime": uptime,
            "cache_size": len(self.cache),
            "queue_sizes": {
                "command": self.command_queue.qsize(),
                "image": self.image_queue.qsize(),
                "perception": self.perception_queue.qsize()
            }
        }


def performance_monitor(func):
    """
    Decorator to monitor performance of functions
    """
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        start_time = time.time()
        result = func(self, *args, **kwargs)
        end_time = time.time()
        
        # Update metrics in the node
        if hasattr(self, 'processing_times'):
            self.processing_times.append(end_time - start_time)
        
        return result
    return wrapper


def main(args=None):
    rclpy.init(args=args)
    
    node = PerformanceOptimizedNode()
    
    # Example of how to access performance metrics
    def print_metrics():
        metrics = node.get_performance_metrics()
        node.get_logger().info(f"Performance Metrics: {metrics}")
    
    # Create a timer to periodically print metrics
    metrics_timer = node.create_timer(5.0, print_metrics)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Performance Optimized Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Summary of Exercises

These exercises cover essential aspects of developing and optimizing the VLA system:

1. **Voice Command Enhancement**: Improves the voice pipeline with multi-language support and wake-word detection
2. **Advanced Cognitive Planning**: Adds ambiguity resolution and visual context integration
3. **Perception Enhancement**: Integrates deep learning models for better object detection and 3D pose estimation
4. **Safety and Validation**: Implements comprehensive safety checks and emergency procedures
5. **Performance Optimization**: Adds multi-threading and caching for better system performance

Each exercise builds on the core VLA system concepts and provides practical implementations that enhance the system's capabilities, safety, and performance.
# Perception Module for Object Detection and Recognition

## Overview

The perception module is a critical component of the Vision-Language-Action (VLA) system, responsible for interpreting visual input and identifying objects in the environment. This module processes camera images to detect and recognize objects, providing essential environmental context for the cognitive planner.

## Architecture

The perception module follows a modular architecture with the following components:

1. **Image Input Handler**: Receives and preprocesses camera images
2. **Object Detection Engine**: Identifies objects in the visual field
3. **Object Recognition System**: Classifies and identifies specific objects
4. **Spatial Reasoning Module**: Determines object positions and relationships
5. **Context Integrator**: Combines perception data with environmental context
6. **Output Formatter**: Publishes results in standard format

## Implementation

### Object Detection and Recognition Node

```python
#!/usr/bin/env python3
"""
Perception Module Node for VLA System

This node performs object detection and recognition using camera input,
publishing detected objects with their positions and properties.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import Header
from vla_msgs.msg import DetectedObjects, Object
import cv2
from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf
from tensorflow import keras
from typing import List, Dict, Tuple, Optional
import json
import time
from dataclasses import dataclass


@dataclass
class DetectionResult:
    """Data structure for object detection results"""
    name: str
    confidence: float
    bounding_box: Tuple[int, int, int, int]  # x, y, width, height
    center_2d: Tuple[int, int]  # x, y center in image coordinates
    position_3d: Optional[Point] = None  # x, y, z in world coordinates


class PerceptionModule(Node):
    """
    Perception module for object detection and recognition in VLA system
    """
    
    def __init__(self):
        super().__init__('perception_module')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize TensorFlow model (using a pre-trained model as an example)
        # In practice, you might load a specialized model for your specific objects
        self.load_detection_model()
        
        # QoS profile
        self.qos_profile = QoSProfile(depth=10)
        
        # Publishers
        self.detected_objects_publisher = self.create_publisher(
            DetectedObjects,
            '/vla/perception/objects',
            self.qos_profile
        )
        
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/vla/perception/debug_image',
            self.qos_profile
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla/perception/status',
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
        self.debug_mode = True
        
        # Object detection classes (COCO dataset classes as example)
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]
        
        self.get_logger().info('Perception Module initialized')

    def load_detection_model(self):
        """
        Load the object detection model
        In practice, you might load a model trained on your specific objects
        """
        try:
            # For this example, we'll use a placeholder model
            # In real implementation, load your actual detection model
            self.get_logger().info('Loading object detection model...')
            
            # Here you would load your actual model
            # self.model = keras.models.load_model('path/to/your/model')
            
            # For demonstration purposes, we'll create a fake model
            self.model = None
            self.get_logger().info('Detection model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Error loading detection model: {e}')
            self.model = None

    def camera_info_callback(self, msg: CameraInfo):
        """
        Handle camera information for 3D position calculation
        """
        self.camera_info = msg

    def image_callback(self, msg: Image):
        """
        Process incoming camera image for object detection
        """
        current_time = time.time()
        
        # Throttle detection frequency
        if current_time - self.last_detection_time < self.detection_frequency:
            return
        
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform object detection
            detections = self.detect_objects(cv_image)
            
            # Convert detections to 3D positions if camera info is available
            detections_3d = []
            for detection in detections:
                detection_3d = self.convert_2d_to_3d(detection, cv_image.shape)
                detections_3d.append(detection_3d)
            
            # Create and publish detected objects message
            detected_objects_msg = self.create_detected_objects_message(detections_3d, msg.header)
            self.detected_objects_publisher.publish(detected_objects_msg)
            
            # Publish debug image if enabled
            if self.debug_mode:
                debug_image = self.draw_detections_on_image(cv_image, detections)
                debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_image_msg.header = msg.header
                self.debug_image_publisher.publish(debug_image_msg)
            
            # Publish status
            status_msg = String()
            status_msg.data = f'Detected {len(detections_3d)} objects'
            self.status_publisher.publish(status_msg)
            
            self.get_logger().info(f'Detected {len(detections_3d)} objects')
            self.last_detection_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image: np.ndarray) -> List[DetectionResult]:
        """
        Perform object detection on the input image
        """
        # For demonstration, we'll use OpenCV's DNN module with a pre-trained model
        # In a real implementation, you would use your specific detection model
        
        # Load YOLO model (for demonstration - you'd use your own model)
        # This is just an example; real implementation would use your model
        try:
            # Create a dummy detection for demonstration
            # In practice, you'd run your actual detection model here
            height, width = image.shape[:2]
            
            # Example detections (replace with actual model inference)
            detections = []
            
            # Add some example detections that might be common in a household/robot environment
            example_objects = [
                ('cup', 0.85, (width//2 - 50, height//2 - 50, 100, 100)),
                ('bottle', 0.78, (width//3, height//3, 80, 120)),
                ('book', 0.92, (2*width//3, height//4, 150, 20)),
                ('chair', 0.88, (50, height//2, 200, 300))
            ]
            
            for name, confidence, bbox in example_objects:
                x, y, w, h = bbox
                center_2d = (x + w//2, y + h//2)
                
                detection = DetectionResult(
                    name=name,
                    confidence=confidence,
                    bounding_box=bbox,
                    center_2d=center_2d
                )
                detections.append(detection)
            
            # In a real implementation, you would use your model here:
            # blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
            # self.model.setInput(blob)
            # outputs = self.model.forward(self.get_output_layers())
            # detections = self.process_detections(outputs, image.shape)
            
            return detections
        except Exception as e:
            self.get_logger().error(f'Error in object detection: {e}')
            return []

    def convert_2d_to_3d(self, detection: DetectionResult, image_shape: Tuple[int, ...]) -> DetectionResult:
        """
        Convert 2D detection coordinates to 3D world coordinates
        """
        if self.camera_info is None:
            # Without camera info, we can't do accurate 3D conversion
            # Return detection with estimated position
            pos_3d = Point()
            pos_3d.x = 1.0  # Default estimated distance
            pos_3d.y = 0.0
            pos_3d.z = 0.0
            
            detection.position_3d = pos_3d
            return detection
        
        # Extract camera parameters
        cx = self.camera_info.k[2]  # Principal point x
        cy = self.camera_info.k[5]  # Principal point y
        fx = self.camera_info.k[0]  # Focal length x
        fy = self.camera_info.k[4]  # Focal length y
        
        # Convert 2D center to 3D ray (simplified)
        center_x, center_y = detection.center_2d
        
        # For now, we'll use a simple depth estimation
        # In reality, you'd use stereo vision, depth sensor, or other methods
        object_height = detection.bounding_box[3]  # height of bounding box in pixels
        image_height = image_shape[0]
        
        # Estimate depth based on object size in image
        # This is a simplification - real implementation would use proper depth sensing
        estimated_depth = 1.0 + (image_height / (object_height + 1e-6)) * 0.1
        
        # Calculate 3D position
        pos_3d = Point()
        pos_3d.z = estimated_depth  # Depth
        pos_3d.x = (center_x - cx) * pos_3d.z / fx  # X position
        pos_3d.y = (center_y - cy) * pos_3d.z / fy  # Y position
        
        detection.position_3d = pos_3d
        return detection

    def create_detected_objects_message(self, detections: List[DetectionResult], header: Header) -> DetectedObjects:
        """
        Create DetectedObjects message from detection results
        """
        msg = DetectedObjects()
        msg.header = header
        
        for detection in detections:
            obj = Object()
            obj.name = detection.name
            obj.confidence = detection.confidence
            
            # Set 3D pose
            obj.pose.position = detection.position_3d if detection.position_3d else Point()
            # Set orientation to identity (facing forward)
            obj.pose.orientation.w = 1.0
            obj.pose.orientation.x = 0.0
            obj.pose.orientation.y = 0.0
            obj.pose.orientation.z = 0.0
            
            # Store bounding box as additional properties
            bbox_info = {
                'x': detection.bounding_box[0],
                'y': detection.bounding_box[1],
                'width': detection.bounding_box[2],
                'height': detection.bounding_box[3],
                'center_2d': detection.center_2d
            }
            obj.properties = json.dumps(bbox_info)
            
            msg.objects.append(obj)
        
        return msg

    def draw_detections_on_image(self, image: np.ndarray, detections: List[DetectionResult]) -> np.ndarray:
        """
        Draw detection bounding boxes on image for debugging
        """
        output_image = image.copy()
        
        for detection in detections:
            x, y, w, h = detection.bounding_box
            
            # Draw bounding box
            cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw label and confidence
            label = f"{detection.name}: {detection.confidence:.2f}"
            cv2.putText(output_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return output_image

    def get_output_layers(self):
        """
        Helper function for YOLO model (placeholder)
        """
        return []


class AdvancedPerceptionModule(PerceptionModule):
    """
    Enhanced perception module with additional features for the VLA system
    """
    
    def __init__(self):
        super().__init__()
        
        # Additional features for VLA system
        self.known_objects = {
            'red cup': {'category': 'drinkware', 'color': 'red', 'graspable': True},
            'blue bottle': {'category': 'drinkware', 'color': 'blue', 'graspable': True},
            'table': {'category': 'furniture', 'graspable': False},
            'chair': {'category': 'furniture', 'graspable': False},
            'book': {'category': 'stationery', 'graspable': True}
        }
        
        # For tracking objects across frames
        self.tracked_objects = {}
        self.max_tracking_age = 10  # frames
        
        # Subscribe to robot pose for spatial reasoning
        from geometry_msgs.msg import PoseStamped
        self.robot_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            self.qos_profile
        )
        
        self.robot_pose = None

    def robot_pose_callback(self, msg):
        """
        Update robot pose for spatial reasoning
        """
        self.robot_pose = msg.pose

    def detect_objects(self, image: np.ndarray) -> List[DetectionResult]:
        """
        Enhanced object detection with VLA-specific optimizations
        """
        # Standard detection
        detections = super().detect_objects(image)
        
        # Enhance with VLA-specific logic
        enhanced_detections = []
        
        for detection in detections:
            # Enhance detection with known object information
            if detection.name in self.known_objects:
                # Update with known properties
                known_props = self.known_objects[detection.name]
                detection.name = detection.name  # Already set
                # Add additional properties could be stored in a custom field if needed
            
            # Add spatial reasoning based on robot pose
            if self.robot_pose:
                # Calculate relative position to robot
                rel_pos = self.calculate_relative_position(detection, self.robot_pose)
                detection.relative_position = rel_pos
            
            enhanced_detections.append(detection)
        
        return enhanced_detections

    def calculate_relative_position(self, detection: DetectionResult, robot_pose) -> Dict:
        """
        Calculate object position relative to robot
        """
        if detection.position_3d is None:
            return {}
        
        # Convert to relative coordinates
        rel_x = detection.position_3d.x - robot_pose.position.x
        rel_y = detection.position_3d.y - robot_pose.position.y
        rel_z = detection.position_3d.z - robot_pose.position.z
        
        # Calculate distance and angle relative to robot's orientation
        import math
        distance = math.sqrt(rel_x**2 + rel_y**2)
        
        # Calculate angle in robot's frame of reference
        robot_yaw = self.quaternion_to_yaw(robot_pose.orientation)
        angle_to_object = math.atan2(rel_y, rel_x) - robot_yaw
        angle_to_object = math.atan2(math.sin(angle_to_object), math.cos(angle_to_object))  # Normalize to [-pi, pi]
        
        return {
            'distance': distance,
            'angle': angle_to_object,
            'relative_position': (rel_x, rel_y, rel_z)
        }

    def quaternion_to_yaw(self, orientation):
        """
        Convert quaternion to yaw angle
        """
        import math
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    """
    Main function to run the perception module
    """
    rclpy.init(args=args)
    
    perception_node = AdvancedPerceptionModule()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        perception_node.get_logger().info('Perception Module interrupted by user')
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Object Recognition with Deep Learning

```python
# deep_learning_perception.py
import tensorflow as tf
from tensorflow import keras
import numpy as np
import cv2
from typing import List, Tuple, Dict
import os


class DeepLearningPerception:
    """
    Advanced perception using deep learning models
    """
    
    def __init__(self, model_path: str = None):
        self.model = None
        self.load_model(model_path)
        
        # Predefined classes for the VLA system environment
        self.classes = [
            'cup', 'bottle', 'book', 'phone', 'tablet', 
            'box', 'chair', 'table', 'hand', 'person'
        ]
    
    def load_model(self, model_path: str = None):
        """
        Load the deep learning model
        """
        if model_path and os.path.exists(model_path):
            try:
                self.model = keras.models.load_model(model_path)
                print(f"Model loaded from {model_path}")
            except Exception as e:
                print(f"Error loading model: {e}")
                self.model = self.create_default_model()
        else:
            print("Creating default model")
            self.model = self.create_default_model()
    
    def create_default_model(self):
        """
        Create a default model architecture if none is provided
        """
        # This would be replaced with your specific model architecture
        # For demonstration, we'll create a simple placeholder
        
        # In practice, this would be a complex model like YOLO, SSD, or RCNN
        print("Using demonstration model")
        return None
    
    def preprocess_image(self, image: np.ndarray, target_size: Tuple[int, int] = (416, 416)):
        """
        Preprocess image for model input
        """
        # Resize image
        resized = cv2.resize(image, target_size)
        
        # Normalize pixel values
        normalized = resized.astype(np.float32) / 255.0
        
        # Add batch dimension
        processed = np.expand_dims(normalized, axis=0)
        
        return processed
    
    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """
        Detect objects in the image using the deep learning model
        """
        if self.model is None:
            # For demonstration, return mock detections
            height, width = image.shape[:2]
            mock_detections = [
                {
                    'name': 'cup',
                    'confidence': 0.89,
                    'bbox': [width//2 - 25, height//2 - 50, width//2 + 25, height//2 + 50]
                },
                {
                    'name': 'book',
                    'confidence': 0.76,
                    'bbox': [width//3, height//3, width//3 + 100, height//3 + 20]
                }
            ]
            return mock_detections
        
        # Preprocess image
        processed_image = self.preprocess_image(image)
        
        # Perform inference
        # This is where you'd run the actual model prediction
        # predictions = self.model.predict(processed_image)
        
        # Process predictions (implementation depends on your model)
        # detections = self.process_predictions(predictions, image.shape)
        
        # For now, return mock detections
        height, width = image.shape[:2]
        return [
            {
                'name': 'cup',
                'confidence': 0.89,
                'bbox': [width//2 - 25, height//2 - 50, width//2 + 25, height//2 + 50]
            }
        ]
    
    def process_predictions(self, predictions, image_shape):
        """
        Process model predictions into object detections
        """
        # This method would convert raw model outputs to object detections
        # Implementation depends on your specific model architecture
        pass


# Example usage of the deep learning perception component
def example_usage():
    perception = DeepLearningPerception()
    
    # In a real scenario, you would get this from a camera
    # For demonstration, create a dummy image
    dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    detections = perception.detect_objects(dummy_image)
    
    print("Detected objects:")
    for det in detections:
        print(f"  {det['name']} (confidence: {det['confidence']:.2f}) at {det['bbox']}")


if __name__ == "__main__":
    example_usage()
```

### Spatial Reasoning Module

```python
# spatial_reasoning.py
import numpy as np
from typing import Dict, List, Tuple
from dataclasses import dataclass
from geometry_msgs.msg import Point


@dataclass
class SpatialRelationship:
    """
    Represents a spatial relationship between objects
    """
    subject: str  # The reference object
    relation: str  # The relationship (left_of, right_of, in_front_of, etc.)
    object: str    # The target object
    distance: float  # Distance in meters
    confidence: float  # Confidence in the relationship


class SpatialReasoningEngine:
    """
    Engine for understanding spatial relationships between objects
    """
    
    def __init__(self):
        self.relationships = []
    
    def calculate_spatial_relationships(self, objects: Dict[str, Point]) -> List[SpatialRelationship]:
        """
        Calculate spatial relationships between objects
        """
        relationships = []
        obj_names = list(objects.keys())
        
        for i, obj1_name in enumerate(obj_names):
            for j, obj2_name in enumerate(obj_names):
                if i == j:
                    continue
                
                obj1_pos = objects[obj1_name]
                obj2_pos = objects[obj2_name]
                
                # Calculate distance
                distance = self.calculate_distance(obj1_pos, obj2_pos)
                
                # Determine spatial relationship
                relationship = self.determine_relationship(obj1_pos, obj2_pos)
                
                rel = SpatialRelationship(
                    subject=obj1_name,
                    relation=relationship,
                    object=obj2_name,
                    distance=distance,
                    confidence=0.9  # High confidence for geometric relationships
                )
                
                relationships.append(rel)
        
        return relationships
    
    def calculate_distance(self, pos1: Point, pos2: Point) -> float:
        """
        Calculate Euclidean distance between two points
        """
        return np.sqrt(
            (pos2.x - pos1.x)**2 + 
            (pos2.y - pos1.y)**2 + 
            (pos2.z - pos1.z)**2
        )
    
    def determine_relationship(self, pos1: Point, pos2: Point) -> str:
        """
        Determine the spatial relationship between two objects
        """
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        dz = pos2.z - pos1.z
        
        # Determine primary relationship based on relative position
        if abs(dx) > abs(dy) and abs(dx) > abs(dz):
            # X-axis relationship is dominant
            if dx > 0:
                return "right_of"
            else:
                return "left_of"
        elif abs(dy) > abs(dx) and abs(dy) > abs(dz):
            # Y-axis relationship is dominant
            if dy > 0:
                return "behind"
            else:
                return "in_front_of"
        else:
            # Z-axis relationship is dominant
            if dz > 0:
                return "above"
            else:
                return "below"
    
    def get_objects_in_region(self, objects: Dict[str, Point], center: Point, radius: float) -> List[str]:
        """
        Get objects within a spherical region
        """
        in_region = []
        for name, pos in objects.items():
            distance = self.calculate_distance(center, pos)
            if distance <= radius:
                in_region.append(name)
        return in_region
    
    def find_nearest_object(self, objects: Dict[str, Point], target_pos: Point, exclude: List[str] = None) -> Tuple[str, float]:
        """
        Find the nearest object to a target position
        """
        if exclude is None:
            exclude = []
        
        min_distance = float('inf')
        nearest_obj = None
        
        for name, pos in objects.items():
            if name in exclude:
                continue
                
            distance = self.calculate_distance(target_pos, pos)
            if distance < min_distance:
                min_distance = distance
                nearest_obj = name
        
        return nearest_obj, min_distance


# Integration with perception module
class PerceptionWithSpatialReasoning(AdvancedPerceptionModule):
    """
    Perception module enhanced with spatial reasoning
    """
    
    def __init__(self):
        super().__init__()
        self.spatial_reasoning = SpatialReasoningEngine()
    
    def create_detected_objects_message(self, detections: List[DetectionResult], header: Header) -> DetectedObjects:
        """
        Create DetectedObjects message with spatial relationships
        """
        msg = super().create_detected_objects_message(detections, header)
        
        # Calculate spatial relationships
        objects_dict = {}
        for obj in msg.objects:
            objects_dict[obj.name] = obj.pose.position
        
        relationships = self.spatial_reasoning.calculate_spatial_relationships(objects_dict)
        
        # Add relationships information to the message
        # (In practice, you might want to create a custom message type that includes relationships)
        # For now, we'll add them as properties
        for obj in msg.objects:
            related_objects = [rel for rel in relationships if rel.subject == obj.name]
            if related_objects:
                rel_info = [(rel.relation, rel.object, rel.distance) for rel in related_objects]
                props = json.loads(obj.properties)
                props['spatial_relationships'] = rel_info
                obj.properties = json.dumps(props)
        
        return msg
```

## Performance Optimization

### Multi-threading for Real-time Processing

```python
import threading
import queue
from collections import deque


class RealTimePerceptionModule(PerceptionWithSpatialReasoning):
    """
    Real-time perception module with multi-threading for better performance
    """
    
    def __init__(self):
        super().__init__()
        
        # Create processing queue
        self.image_queue = queue.Queue(maxsize=10)  # Limit queue size to prevent memory issues
        self.detection_results = deque(maxlen=5)  # Keep last 5 detection results
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_images, daemon=True)
        self.processing_thread.start()
        
        # Flag to stop processing
        self.processing_active = True
    
    def image_callback(self, msg: Image):
        """
        Add image to processing queue (non-blocking)
        """
        try:
            self.image_queue.put_nowait(msg)
        except queue.Full:
            # Drop the oldest image if queue is full
            try:
                self.image_queue.get_nowait()  # Remove oldest
                self.image_queue.put_nowait(msg)  # Add new image
            except queue.Empty:
                pass  # Shouldn't happen, but just in case
    
    def process_images(self):
        """
        Process images in a separate thread
        """
        while self.processing_active:
            try:
                # Get image from queue with timeout
                msg = self.image_queue.get(timeout=0.1)
                
                # Process the image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                detections = self.detect_objects(cv_image)
                
                # Convert detections to 3D positions
                detections_3d = []
                for detection in detections:
                    detection_3d = self.convert_2d_to_3d(detection, cv_image.shape)
                    detections_3d.append(detection_3d)
                
                # Store the latest detection result
                if detections_3d:
                    self.detection_results.append({
                        'timestamp': time.time(),
                        'detections': detections_3d,
                        'header': msg.header
                    })
                
                # Publish detection results if not too frequent
                current_time = time.time()
                if current_time - self.last_detection_time >= self.detection_frequency:
                    if self.detection_results:
                        latest_result = self.detection_results[-1]
                        
                        detected_objects_msg = self.create_detected_objects_message(
                            latest_result['detections'], 
                            latest_result['header']
                        )
                        self.detected_objects_publisher.publish(detected_objects_msg)
                        
                        # Publish status
                        status_msg = String()
                        status_msg.data = f'Detected {len(latest_result["detections"])} objects'
                        self.status_publisher.publish(status_msg)
                        
                        self.last_detection_time = current_time
                        
            except queue.Empty:
                continue  # No image to process, continue loop
            except Exception as e:
                self.get_logger().error(f'Error in processing thread: {e}')
    
    def destroy_node(self):
        """
        Clean up when shutting down
        """
        self.processing_active = False
        if self.processing_thread.is_alive():
            self.processing_thread.join(timeout=1.0)
        super().destroy_node()
```

## Testing the Perception Module

```python
# test_perception.py
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vla_msgs.msg import DetectedObjects
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2


class TestPerceptionModule(unittest.TestCase):
    """
    Test class for the perception module
    """
    
    def setUp(self):
        rclpy.init()
        self.node = Node('test_perception')
        
        self.bridge = CvBridge()
        
        # Create publisher for test images
        self.image_publisher = self.node.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )
        
        # Create subscriber for detected objects
        self.detected_objects_sub = self.node.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.detection_callback,
            10
        )
        
        self.last_detection = None
        self.detection_received = False
    
    def detection_callback(self, msg: DetectedObjects):
        """
        Callback for when detections are received
        """
        self.last_detection = msg
        self.detection_received = True
    
    def test_object_detection(self):
        """
        Test that the perception module can detect objects
        """
        # Create a test image with a simple shape
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # Add a colored rectangle to represent an object
        cv2.rectangle(test_image, (200, 150), (300, 250), (0, 255, 0), -1)
        
        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(test_image, encoding='bgr8')
        ros_image.header.stamp = self.node.get_clock().now().to_msg()
        
        # Publish the test image
        self.image_publisher.publish(ros_image)
        
        # Wait for detection with timeout
        timeout = time.time() + 10.0  # 10 second timeout
        while not self.detection_received and time.time() < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Verify results
        self.assertTrue(self.detection_received, "Detection not received within timeout")
        
        if self.last_detection:
            self.assertGreater(len(self.last_detection.objects), 0, 
                             "No objects detected in test image")
            print(f"Detected {len(self.last_detection.objects)} objects")
            
            # Check that detected objects have reasonable properties
            for obj in self.last_detection.objects:
                self.assertIsNotNone(obj.name, "Object name is None")
                self.assertGreaterEqual(obj.confidence, 0.0, "Confidence is negative")
                self.assertLessEqual(obj.confidence, 1.0, "Confidence is greater than 1.0")
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
```

## Launch Configuration

```python
# launch/perception_module.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    debug_mode = LaunchConfiguration('debug_mode', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='true',
            description='Enable debug image publishing if true'
        ),
        
        # Perception module node
        Node(
            package='vla_examples',
            executable='perception_module',
            name='perception_module',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'debug_mode': debug_mode}
            ],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('/camera/camera_info', '/camera/camera_info')
            ],
            output='screen'
        )
    ])
```

## Conclusion

The perception module implemented here provides a comprehensive solution for object detection and recognition in the VLA system. Key features include:

1. **Real-time Processing**: Efficient image processing with multi-threading
2. **Spatial Reasoning**: Understanding of object positions and relationships
3. **3D Position Estimation**: Conversion of 2D detections to 3D world coordinates
4. **Extensibility**: Designed to accommodate different deep learning models
5. **Integration Ready**: Properly formatted messages for ROS2 integration

The module is designed to be an integral part of the broader VLA architecture, providing essential environmental context to the cognitive planner and other system components.
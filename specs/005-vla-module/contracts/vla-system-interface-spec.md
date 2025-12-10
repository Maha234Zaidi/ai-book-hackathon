# VLA System Interface Specifications

## Overview
This document defines the conceptual interfaces between components of the Vision-Language-Action (VLA) system for educational purposes. Since this is an educational module rather than a software system with traditional APIs, these interfaces describe how different components interact in the implementation examples.

## 1. Voice Command Pipeline Interface

### Input: Audio Stream
```
AudioInput {
    source: string (microphone | file | stream)
    sample_rate: integer (Hz)
    channels: integer
    format: string (e.g., PCM, WAV)
}
```

### Process: Speech Recognition
```
Request: AudioInput
Response: {
    text: string
    confidence: float (0.0-1.0)
    processing_time: float (seconds)
}
```

### Integration with ROS 2:
- Topic: `/vla/audio_input`
- Message Type: `std_msgs/String`
- Service: `transcribe_audio`
- Service Type: Custom `TranscribeAudio`

## 2. Cognitive Planner Interface

### Input: Natural Language Command
```
NaturalLanguageCommand {
    text: string
    user_id: string (optional)
    context: string (environmental context)
}
```

### Process: LLM Processing
```
Request: NaturalLanguageCommand
Response: {
    action_sequence: Action[]
    reasoning: string (for educational explanation)
    confidence: float (0.0-1.0)
    error: string (if processing failed)
}
```

### Action Definition:
```
Action {
    type: string (move_to, grasp, manipulate, etc.)
    parameters: object
    priority: enum (High, Medium, Low)
}
```

### Integration with ROS 2:
- Service: `plan_cognitive_task`
- Service Type: Custom `PlanCognitiveTask`
- Action Server: `execute_action_sequence`

## 3. Perception Module Interface

### Input: Visual Data
```
VisualInput {
    source: string (camera_topic, image_file, simulation)
    width: integer (pixels)
    height: integer (pixels)
    encoding: string (e.g., RGB8, BGR8, MONO8)
}
```

### Process: Object Detection
```
Request: VisualInput
Response: {
    detected_objects: DetectedObject[]
    confidence_threshold: float (0.0-1.0)
    processing_time: float (seconds)
}
```

### Detected Object Definition:
```
DetectedObject {
    id: string
    class: string (e.g., "cup", "table", "human")
    bounding_box: {
        x: integer (top-left x)
        y: integer (top-left y)
        width: integer
        height: integer
    }
    confidence: float (0.0-1.0)
    position_3d?: {x: float, y: float, z: float} (if depth available)
}
```

### Integration with ROS 2:
- Subscriber: `/camera/image_raw`
- Message Type: `sensor_msgs/Image`
- Publisher: `/vla/perception/objects`
- Message Type: Custom `DetectedObjects`

## 4. Path Planning Component Interface

### Input: Navigation Request
```
NavigationRequest {
    start_pose: {
        x: float
        y: float
        theta: float (radians)
    }
    goal_pose: {
        x: float
        y: float
        theta: float (radians)
    }
    map: string (map topic or file)
}
```

### Process: Path Planning
```
Request: NavigationRequest
Response: {
    path: Path
    cost: float
    success: boolean
    error?: string
}
```

### Path Definition:
```
Path {
    poses: Pose[]
    resolution: float (meters between waypoints)
}

Pose {
    x: float
    y: float
    theta: float (radians)
}
```

### Integration with ROS 2:
- Action: `navigate_to_pose`
- Action Type: `nav2_msgs/NavigateToPose`
- Service: `get_path`
- Service Type: Custom `GetPath`

## 5. VLA System Integration Interface

### High-Level Command Processing
```
VLACommand {
    type: enum (voice_command, text_command, direct_control)
    content: string (the command content)
    priority: enum (High, Medium, Low)
    safety_level: enum (Safe, RequiresValidation, Dangerous)
}
```

### Processing Flow:
1. Receive `VLACommand` via `/vla/command` topic
2. Process through Voice Command Pipeline or direct to Cognitive Planner
3. Validate with Perception Module
4. Plan with Path Planning Component if needed
5. Execute through ROS 2 action interface
6. Return status via `/vla/status` topic

### Status Response:
```
VLAStatus {
    command_id: string
    state: enum (Received, Processing, Planned, Executing, Completed, Failed)
    progress: float (0.0-1.0 for ongoing actions)
    error?: string
    timestamp: string (ISO 8601)
}
```

## 6. Educational Module Interfaces

### Code Example Standards
All code examples in the module will follow these patterns:

1. ROS 2 nodes using `rclpy` for Python implementations
2. Clear input/output specifications
3. Error handling and safety validation
4. Integration with simulation environments (Gazebo)
5. Documentation following ROS 2 standards

## Validation Rules

1. All voice commands must be validated for safety before execution
2. Perception data must meet minimum confidence thresholds
3. Path planning must avoid detected obstacles
4. Action sequences must have timeout mechanisms
5. All components must provide status feedback
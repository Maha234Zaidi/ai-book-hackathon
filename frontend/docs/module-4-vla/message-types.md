# Custom ROS 2 Message Types for VLA System

## DetectedObjects.msg

This message type is used by the perception module to publish information about detected objects in the environment.

```
# Array of detected objects
ObjectInfo[] objects

# Timestamp of detection
builtin_interfaces/Time detection_time

# Processing time for detection
float32 processing_time
```

## ObjectInfo.idl (part of DetectedObjects)

```
# Unique identifier for the object
string object_id

# Class of the object (e.g., "cup", "chair", "human")
string object_class

# Confidence of detection (0.0 to 1.0)
float32 confidence

# Bounding box coordinates in image space
uint32 x
uint32 y
uint32 width
uint32 height

# 3D position in world coordinates (optional)
geometry_msgs/Point32 world_position
```

## TranscribeAudio.srv

This service handles speech-to-text conversion using the Whisper API.

### Request
```
# Audio data in the form of a string path to file, or raw audio data
string audio_input_path
string audio_format  # e.g., "wav", "mp3", "raw"

# Optional: Configuration for transcription
float32 confidence_threshold
string language_code  # e.g., "en", "es", "fr"
```

### Response
```
# Transcribed text
string transcribed_text

# Processing information
float32 processing_time
float32 confidence_score
bool success
string error_message  # Empty if success is true
```

## PlanCognitiveTask.srv

This service handles cognitive planning by converting natural language commands into action sequences.

### Request
```
# Natural language command from user
string natural_language_command

# Context about the environment
string environment_context

# Optional: Specific robot capabilities to consider
string[] supported_actions
```

### Response
```
# Planned sequence of actions
Action[] action_sequence

# Explanation of reasoning (for educational purposes)
string reasoning

# Validation results
bool safety_validated
string[] validation_warnings

# Processing information
float32 processing_time
bool success
string error_message  # Empty if success is true
```

## Action.idl (part of PlanCognitiveTask)

```
# Type of action (e.g., "move_to", "grasp", "manipulate", "navigate", "detect")
string action_type

# Parameters for the action
dictionary parameters  # key-value pairs specific to action type

# Priority level for execution
uint8 priority  # 0: low, 1: medium, 2: high

# Estimated duration in seconds
float32 estimated_duration

# Dependencies (other actions that must complete first)
string[] dependencies
```
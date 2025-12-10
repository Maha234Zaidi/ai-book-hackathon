# Data Model: Vision-Language-Action (VLA) Educational Module

## Key Entities

### VLA System
- **Description**: A complete vision-language-action framework that integrates perception, language understanding, and robotic action execution
- **Fields/Attributes**:
  - SystemID (string): Unique identifier for the VLA system
  - Name (string): Human-readable name of the system
  - Components (list): References to Voice Command Pipeline, Cognitive Planner, Perception Module, Path Planning Component
  - Status (enum): Current operational status (Idle, Processing, Executing, Error)
  - LastInteraction (datetime): Timestamp of last system interaction

### Voice Command Pipeline
- **Description**: The processing chain from audio input through speech recognition to text and then to executable commands
- **Fields/Attributes**:
  - PipelineID (string): Unique identifier for the pipeline
  - AudioInput (string): Audio source (microphone, file, stream)
  - TranscribedText (string): Text result from speech recognition
  - CommandSequence (list): Sequence of executable commands derived from text
  - ConfidenceScore (float): Confidence in speech recognition (0.0-1.0)
  - ProcessingTime (float): Time in seconds for processing the command

### Cognitive Planner
- **Description**: The component that uses LLMs to interpret natural language and generate multi-step action sequences
- **Fields/Attributes**:
  - PlannerID (string): Unique identifier for the planner
  - NaturalLanguageInput (string): Raw user command in natural language
  - ActionSequence (list): Ordered list of robot actions to execute
  - PlanningReasoning (string): Explanation of planning steps (for educational purposes)
  - SafetyValidation (boolean): Whether actions have passed safety checks
  - ExecutionPriority (enum): Priority level (High, Medium, Low)

### ROS 2 Action Sequence
- **Description**: A series of commands and behaviors that execute complex tasks in the robotic system
- **Fields/Attributes**:
  - SequenceID (string): Unique identifier for the action sequence
  - Actions (list): Individual actions in the sequence
  - Dependencies (list): Dependencies between actions
  - EstimatedDuration (float): Estimated execution time in seconds
  - ExecutionStatus (enum): Status of sequence execution (Queued, InProgress, Completed, Failed)
  - ErrorLog (list): Errors encountered during execution

### Perception Module
- **Description**: The component responsible for interpreting visual input and identifying objects and obstacles in the environment
- **Fields/Attributes**:
  - ModuleID (string): Unique identifier for the perception module
  - VisualInput (string): Image source (camera feed, file, simulation)
  - DetectedObjects (list): Objects detected in the environment
  - ObjectProperties (dict): Properties of detected objects (position, size, type)
  - ConfidenceThreshold (float): Minimum confidence to consider detections valid
  - ProcessingTime (float): Time in seconds for processing visual input

### Path Planning Component
- **Description**: The system that determines optimal routes for robot navigation based on environmental perception
- **Fields/Attributes**:
  - ComponentID (string): Unique identifier for the path planning component
  - StartPosition (vector): Starting coordinates of the path
  - EndPosition (vector): Target coordinates of the path
  - Obstacles (list): Detected obstacles to avoid
  - PlannedPath (list): Ordered list of coordinates for the planned path
  - PathCost (float): Cost measure of the planned path
  - PlanningTime (float): Time to generate the path

## Data Relationships

- A VLA System contains one or more Voice Command Pipelines
- A VLA System contains one Cognitive Planner
- A Cognitive Planner generates one or more ROS 2 Action Sequences
- A VLA System contains one Perception Module
- A VLA System contains one Path Planning Component
- A Voice Command Pipeline may trigger a Cognitive Planner
- A Perception Module provides input to the Path Planning Component
- A Path Planning Component generates input for ROS 2 Action Sequences

## State Transitions

### VLA System States
- Idle → Processing: When a voice command is received
- Processing → Executing: When action sequence is validated and starting
- Executing → Idle: When action sequence completes successfully
- Processing → Idle: When command cannot be processed
- Executing → Idle: When action sequence fails with error handling

### Voice Command Pipeline States
- Idle → Listening: When waiting for audio input
- Listening → Processing: When audio input is captured
- Processing → Processed: When text transcription is complete
- Processed → Ready: When command sequence is generated

### Cognitive Planner States
- Idle → Planning: When receiving natural language input
- Planning → Planned: When action sequence is generated
- Planned → Validating: When safety checks are performed
- Validating → Validated: When safety checks pass
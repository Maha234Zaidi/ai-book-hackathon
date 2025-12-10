# System Architecture Diagram for VLA Components Interaction

## High-Level Architecture

The Vision-Language-Action (VLA) system architecture is designed to integrate perception, language understanding, and action execution in a unified framework. This document details the architecture diagram that shows how VLA components interact.

## Component Interaction Architecture

```mermaid
graph TB
    subgraph "User Interface Layer"
        U[Human User]
        V[Voice Input]
        T[Text Command]
    end
    
    subgraph "Perception Layer"
        P[Perception Module]
        CD[Camera Data]
        SD[Sensor Data]
        DO[Detected Objects]
    end
    
    subgraph "Language Processing Layer"
        VCP[Voice Command Pipeline]
        TC[Text Conversion]
        CP[Cognitive Planner]
        AS[Action Sequence]
    end
    
    subgraph "Action Layer"
        PP[Path Planning Component]
        AE[Action Execution]
        RAS[ROS 2 Action Sequence]
    end
    
    subgraph "Robot Control"
        RP[Robot Platform]
        R[Robot Actions]
    end
    
    subgraph "Simulation Environment"
        GS[Gazebo Simulator]
        SV[Simulation Visualization]
    end
    
    %% User interactions
    U --> V
    U --> T
    
    %% Voice processing
    V --> VCP
    VCP --> TC
    T --> TC
    
    %% Perception processing
    CD --> P
    SD --> P
    P --> DO
    
    %% Cognitive planning
    TC --> CP
    DO --> CP
    CP --> AS
    
    %% Path planning
    AS --> PP
    DO --> PP  % For obstacle information
    
    %% Action execution
    PP --> AE
    AS --> AE
    AE --> RAS
    RAS --> RP
    RP --> R
    
    %% Simulation
    R --> GS
    P --> GS
    GS --> SV
    
    %% Feedback loops
    R --> U  % Robot status to user
    SV --> U  % Simulation visualization to user
    
    %% Style definitions
    style U fill:#cde4ff
    style V fill:#cde4ff
    style T fill:#cde4ff
    
    style P fill:#f9f,stroke:#333,stroke-width:2px
    style CD fill:#fef6d9
    style SD fill:#fef6d9
    style DO fill:#e4f5e2
    
    style VCP fill:#f9f,stroke:#333,stroke-width:2px
    style TC fill:#e4f5e2
    style CP fill:#f9f,stroke:#333,stroke-width:2px
    style AS fill:#e4f5e2
    
    style PP fill:#f9f,stroke:#333,stroke-width:2px
    style AE fill:#f9f,stroke:#333,stroke-width:2px
    style RAS fill:#e4f5e2
    
    style RP fill:#e4f5e2
    style R fill:#e4f5e2
    
    style GS fill:#fef6d9
    style SV fill:#fef6d9
```

## Detailed Component Descriptions

### 1. User Interface Layer
- **Human User**: The person interacting with the VLA system
- **Voice Input**: Audio input from microphones or other audio sources
- **Text Command**: Alternative text-based input method

### 2. Perception Layer
- **Perception Module**: Processes visual and sensor data to understand the environment
  - Input: Camera and sensor data
  - Output: Detected objects and environmental understanding
- **Camera Data**: Visual input from robot cameras
- **Sensor Data**: Input from various robot sensors (lidar, IMU, etc.)
- **Detected Objects**: Processed information about objects in the environment

### 3. Language Processing Layer
- **Voice Command Pipeline**: Converts voice input to text
  - Uses OpenAI Whisper API for speech-to-text conversion
- **Text Conversion**: Processes text commands from voice or direct input
- **Cognitive Planner**: Uses LLMs to plan action sequences from natural language
  - Input: Text commands and detected objects
  - Output: Sequences of planned actions
- **Action Sequence**: Planned series of actions to execute the command

### 4. Action Layer
- **Path Planning Component**: Plans navigation routes based on environmental information
  - Input: Goal positions and detected obstacles
  - Output: Navigation paths
- **Action Execution**: Coordinates the execution of action sequences
  - Input: Action sequences and path information
  - Output: ROS 2 action commands
- **ROS 2 Action Sequence**: Standardized format for robot commands in ROS 2 ecosystem

### 5. Robot Control
- **Robot Platform**: Physical or simulated robot that executes commands
- **Robot Actions**: Actual physical movements and operations

### 6. Simulation Environment
- **Gazebo Simulator**: Simulation environment for testing VLA systems
- **Simulation Visualization**: Visual output from the simulation

## Data Flow Patterns

### Primary Flow
1. User provides voice or text command
2. Voice command is converted to text (if needed)
3. Cognitive planner generates action sequence based on text command and environmental context
4. Path planning creates navigation routes if needed
5. Actions are executed on the robot platform
6. Results are observed in simulation or real environment

### Feedback Loops
1. Perception continuously updates environmental understanding
2. Robot status is reported back to the user
3. Simulation visualization provides feedback on system behavior

## Communication Protocols

### ROS 2 Topics
- `/vla/command` - Input commands to the VLA system
- `/vla/recognized_text` - Transcribed text from voice input
- `/vla/action_sequence` - Planned action sequences
- `/vla/perception/objects` - Detected objects from perception module
- `/camera/image_raw` - Camera image data

### ROS 2 Services
- `transcribe_audio` - Speech-to-text conversion
- `plan_cognitive_task` - Cognitive planning service
- `get_path` - Path planning service

### ROS 2 Actions
- `navigate_to_pose` - Navigation tasks
- `execute_action_sequence` - Complex action execution

## Safety and Validation Layers

Each component includes safety and validation mechanisms:
- Input validation at each component boundary
- Safety checks before action execution
- Environmental monitoring throughout operation
- Error handling and recovery procedures

This architecture enables the tight integration of vision, language, and action while maintaining system safety and reliability.
# VLA System Architecture Diagram

```mermaid
graph TB
    subgraph "User Interaction"
        VoiceInput[Voice Command]
    end
    
    subgraph "VLA System Components"
        A[Voice Command Pipeline]
        B[Cognitive Planner]
        C[Perception Module]
        D[Path Planning Component]
        E[Action Execution]
    end
    
    subgraph "Robot Control"
        F[ROS 2 Action Sequence]
        G[Robot Platform]
    end
    
    subgraph "Simulation Environment"
        H[Gazebo Simulator]
        I[Sensor Data]
    end
    
    VoiceInput --> A
    A --> B
    I --> C
    C --> B
    B --> D
    B --> E
    D --> E
    E --> F
    F --> G
    G --> H
    H --> I
    
    style VoiceInput fill:#cde4ff
    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#f9f,stroke:#333,stroke-width:2px
    style C fill:#f9f,stroke:#333,stroke-width:2px
    style D fill:#f9f,stroke:#333,stroke-width:2px
    style E fill:#f9f,stroke:#333,stroke-width:2px
    style F fill:#e4f5e2
    style G fill:#e4f5e2
    style H fill:#fef6d9
    style I fill:#fef6d9
```

## Component Descriptions

1. **Voice Command Pipeline**: Processes audio input using OpenAI Whisper to convert speech to text
2. **Cognitive Planner**: Uses LLMs to translate natural language commands into action sequences
3. **Perception Module**: Processes visual input to detect and identify objects in the environment
4. **Path Planning Component**: Determines optimal navigation routes based on environmental perception
5. **Action Execution**: Coordinates the execution of action sequences on the robot platform
6. **ROS 2 Action Sequence**: Standardized format for robot commands in the ROS 2 ecosystem
7. **Robot Platform**: Physical or simulated robot that executes the commands
8. **Gazebo Simulator**: Simulation environment for testing VLA systems
9. **Sensor Data**: Input from robot sensors (cameras, lidar, etc.) processed by perception module
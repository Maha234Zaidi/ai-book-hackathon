---
title: Architecture Diagrams
description: Visual representations of ROS 2 architecture
sidebar_position: 100
---

# Architecture Diagrams

## ROS 2 Architecture Overview

```mermaid
graph TB
    subgraph "ROS 2 Ecosystem"
        RMW[RMW - DDS Implementation]
        DDS[(DDS Middleware)]
    end
    
    subgraph "ROS 2 Client Libraries"
        direction LR
        RCLPY[rclpy<br/>Python]
        RCLCPP[rclcpp<br/>C++]
    end
    
    subgraph "ROS 2 Applications"
        direction TB
        NodeA[Node A]
        NodeB[Node B]
        NodeC[Node C]
    end
    
    subgraph "Communication Layer"
        Topic1[Topic: /sensor_data]
        Topic2[Topic: /cmd_vel]
        Service1[Service: /add_two_ints]
    end
    
    RCLPY -.-> RMW
    RCLCPP -.-> RMW
    RMW -.-> DDS
    
    NodeA --> Topic1
    NodeB --> Topic1
    NodeB --> Topic2
    NodeC --> Topic2
    NodeA --> Service1
    NodeB --> Service1
    
    Topic1 -.-> NodeA
    Topic1 -.-> NodeB
    Topic2 -.-> NodeB
    Topic2 -.-> NodeC
    Service1 -.-> NodeA
    Service1 -.-> NodeB
    
    style DDS fill:#f9f,stroke:#333,stroke-width:2px
    style RMW fill:#ccf,stroke:#333,stroke-width:2px
    style RCLPY fill:#cfc,stroke:#333,stroke-width:2px
    style RCLCPP fill:#cfc,stroke:#333,stroke-width:2px
    style NodeA fill:#cff,stroke:#333,stroke-width:2px
    style NodeB fill:#cff,stroke:#333,stroke-width:2px
    style NodeC fill:#cff,stroke:#333,stroke-width:2px
```

## Node-Topic-Service Communication Pattern

```mermaid
graph LR
    subgraph "Publisher Node"
        PNode[Publisher<br/>Node]
        PTimer[Timer]
        PPublisher[Publisher]
    end
    
    subgraph "Subscriber Node"
        SNode[Subscriber<br/>Node]
        SSubscriber[Subscriber]
        SCallback[Callback]
    end
    
    subgraph "Topics & Messages"
        Topic[(Topic: /chatter)]
        Msg1[Message<br/>Hello 0]
        Msg2[Message<br/>Hello 1]
        Msg3[Message<br/>Hello 2]
    end
    
    PTimer --> PPublisher
    PPublisher --> Msg1
    PPublisher --> Msg2
    PPublisher --> Msg3
    Msg1 --> Topic
    Msg2 --> Topic
    Msg3 --> Topic
    Topic --> SSubscriber
    SSubscriber --> SCallback
    PNode -.-> SNode
    
    style PNode fill:#aaffaa,stroke:#333,stroke-width:2px
    style SNode fill:#aaaaff,stroke:#333,stroke-width:2px
    style Topic fill:#ffffaa,stroke:#333,stroke-width:2px
```

## Service Client-Server Pattern

```mermaid
graph LR
    subgraph "Service Server"
        ServNode[Service<br/>Node]
        ServServer[Service<br/>Server]
        ServCallback[Service<br/>Callback]
    end
    
    subgraph "Service Client"
        CliNode[Client<br/>Node]
        CliClient[Service<br/>Client]
        CliRequest[Request]
    end
    
    subgraph "Service Interface"
        ServInt[(Service<br/>add_two_ints)]
        Request[Request<br/>a=1, b=2]
        Response[Response<br/>sum=3]
    end
    
    CliNode --> CliRequest
    CliRequest --> Request
    Request --> ServInt
    ServInt --> ServServer
    ServServer --> ServCallback
    ServCallback --> Response
    Response --> ServInt
    ServInt --> CliRequest
    CliRequest --> CliNode
    
    style ServNode fill:#aaffaa,stroke:#333,stroke-width:2px
    style CliNode fill:#aaaaff,stroke:#333,stroke-width:2px
    style ServInt fill:#ffffaa,stroke:#333,stroke-width:2px
```

## URDF Robot Model Structure

```mermaid
graph TD
    Robot[Robot Element<br/>name="simple_robot"]
    
    subgraph "Links"
        Base[Link<br/>base_link]
        Wheel1[Link<br/>wheel_1]
        Wheel2[Link<br/>wheel_2]
        Castor[Link<br/>castor_wheel]
    end
    
    subgraph "Joints"
        J1[Joint<br/>base_to_wheel1<br/>continuous]
        J2[Joint<br/>base_to_wheel2<br/>continuous]
        J3[Joint<br/>base_to_castor<br/>fixed]
    end
    
    Robot --> Base
    Robot --> J1
    Robot --> J2
    Robot --> J3
    
    J1 --> Base
    J1 --> Wheel1
    J2 --> Base
    J2 --> Wheel2
    J3 --> Base
    J3 --> Castor
    
    style Robot fill:#ffaaaa,stroke:#333,stroke-width:2px
    style Base fill:#aaffaa,stroke:#333,stroke-width:2px
    style Wheel1 fill:#aaffaa,stroke:#333,stroke-width:2px
    style Wheel2 fill:#aaffaa,stroke:#333,stroke-width:2px
    style Castor fill:#aaffaa,stroke:#333,stroke-width:2px
    style J1 fill:#aaaaff,stroke:#333,stroke-width:2px
    style J2 fill:#aaaaff,stroke:#333,stroke-width:2px
    style J3 fill:#aaaaff,stroke:#333,stroke-width:2px
```

## Lifecycle of a ROS 2 Node

```mermaid
stateDiagram-v2
    [*] --> Unconfigured
    Unconfigured --> Inactive: create_node()
    Inactive --> Active: activate()
    Active --> Inactive: deactivate()
    Inactive --> Unconfigured: cleanup()
    Unconfigured --> Finalized: destroy_node()
    
    Active --> Error: error()
    Inactive --> Error: error()
    Unconfigured --> Error: error()
    Error --> [*]
    
    note right of Active
        : spin()
        : processing callbacks
    end
```

## Quality of Service (QoS) Settings Impact

```mermaid
graph LR
    subgraph "Publisher with QoS"
        Pub[Publisher]
        QoSP[QoS Profile:<br/>Reliability: Reliable<br/>Durability: Volatile<br/>History: Keep Last 10]
    end
    
    subgraph "Message Queue"
        Queue[Message Queue<br/>Depth: 10]
    end
    
    subgraph "Subscriber with QoS"
        QoSS[QoS Profile:<br/>Reliability: Best Effort<br/>Durability: Volatile<br/>History: Keep Last 5]
        Sub[Subscriber]
    end
    
    Pub --> QoSP
    QoSP --> Queue
    Queue --> QoSS
    QoSS --> Sub
    
    style Pub fill:#aaffaa,stroke:#333,stroke-width:2px
    style Sub fill:#aaaaff,stroke:#333,stroke-width:2px
    style Queue fill:#ffffaa,stroke:#333,stroke-width:2px
```
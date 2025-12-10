# Data Model: Robotic Communication Systems Education Module

## Overview
This document defines the key entities and concepts that will be taught in the Robotic Communication Systems Education Module. These concepts form the foundation for understanding how robots communicate and coordinate their actions.

## Core Entities

### Communication Components
- **Definition**: Independent processes that coordinate with each other using robotic communication systems
- **Attributes**:
  - name: String (unique identifier for the component)
  - type: Enum (publisher, subscriber, service_client, service_server)
  - interface: String (the API or protocol used for communication)
- **Relationships**: Can communicate with other components through specific channels
- **Validation**: Must have a valid name and interface definition

### Streaming Communication
- **Definition**: Communication channels for continuous data flow between system components using publisher/subscriber pattern
- **Attributes**:
  - topic_name: String (identifier for the communication channel)
  - message_type: String (type of data being transmitted)
  - publishers: List of Communication Components (sources of data)
  - subscribers: List of Communication Components (receivers of data)
- **Relationships**: Connects multiple Communication Components
- **Validation**: Topic name must be unique within the system; matching message types required

### Request-Response Communication
- **Definition**: Communication pattern for specific query-response interactions between system components
- **Attributes**:
  - service_name: String (identifier for the service)
  - request_type: String (type of request message)
  - response_type: String (type of response message)
  - clients: List of Communication Components (requesters)
  - servers: List of Communication Components (responders)
- **Relationships**: Connects client and server Communication Components
- **Validation**: Service name must be unique; request and response types must be defined

### Robot Model
- **Definition**: A standardized format for representing robot models, defining physical components and relationships
- **Attributes**:
  - name: String (robot identifier)
  - links: List of physical components (rigid bodies)
  - joints: List of connections between links
  - materials: List of material properties
  - sensors: List of sensor components
- **Relationships**: Components are connected via joints
- **Validation**: Each joint must connect exactly two links; no unconnected links

### Humanoid Robot
- **Definition**: A robot with a human-like body structure, typically including a head, torso, two arms, and two legs
- **Attributes**:
  - model: Robot Model (the physical representation)
  - degrees_of_freedom: Integer (number of independent movements)
  - sensors: List of sensor types (camera, IMU, force/torque, etc.)
  - actuators: List of actuator types (motors for joints)
- **Relationships**: Composed of a Robot Model with specific configuration
- **Validation**: Must have at least head, torso, arms, and legs defined

### AI Agent
- **Definition**: A software component powered by artificial intelligence that can issue commands to control robotic systems
- **Attributes**:
  - name: String (agent identifier)
  - control_interface: String (how it communicates with robots)
  - sensors_access: List of sensor types it can read
  - actuators_control: List of actuator types it can command
- **Relationships**: Interacts with Communication Components and Humanoid Robots
- **Validation**: Must have appropriate interfaces to communicate with the robotic system

## Relationships

```
Humanoid Robot
    ├── has Robot Model
    │   ├── contains Links
    │   ├── connects via Joints
    │   └── contains Sensors
    │
    └── interacts with AI Agent
        └── communicates via Communication Components
            ├── uses Streaming Communication
            └── uses Request-Response Communication
```

## State Transitions (if applicable)

For AI Agents:
- IDLE → PLANNING (when a control task is received)
- PLANNING → EXECUTING (when a plan is ready to be executed)
- EXECUTING → IDLE (when task is completed)
- EXECUTING → ERROR (when an unexpected situation occurs)

## Validation Rules

1. All entities must have properly defined names that follow consistent naming conventions
2. Communication channels (topics/services) must have unique identifiers
3. Robot models must be structurally valid with properly connected components
4. AI agents must only command capabilities that exist in the target robot
5. All communication interfaces must be properly defined with matching types

## Glossary of Terms

- **Node**: An independent process that performs computation in a robotic system
- **Topic**: A named bus over which nodes exchange messages
- **Publisher**: A node that sends messages on a topic
- **Subscriber**: A node that receives messages from a topic
- **Service**: A communication pattern that allows nodes to request information or actions from other nodes
- **URDF**: Unified Robot Description Format, an XML format for representing robot models
- **rclpy**: ROS 2 Python client library that allows Python programs to interact with ROS 2
- **DDS**: Data Distribution Service, the middleware that underlies ROS 2 communication
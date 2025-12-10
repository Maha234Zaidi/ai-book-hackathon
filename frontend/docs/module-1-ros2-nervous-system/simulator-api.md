# Educational ROS 2 Simulator API

This document provides an overview of the educational ROS 2 simulator API that students will learn to work with when studying ROS 2 concepts. This represents a simplified simulation environment that demonstrates ROS 2 nodes, topics, services, and parameters without requiring actual ROS 2 installation.

## Overview

The Educational ROS 2 Simulator API provides a web-based interface to explore fundamental ROS 2 concepts including:

- Node management and communication
- Topic publishing and subscribing
- Service calls and responses
- Robot model representation (URDF)

## API Endpoints

### Base URL
```
https://edu-ros2-simulator.example.com/api/v1
```

### Common Headers
```
Content-Type: application/json
Accept: application/json
```

## Node Management

### GET /nodes
List all active nodes in the simulated ROS 2 system.

**Response (200 OK):**
```json
{
  "nodes": [
    {
      "id": "node_123",
      "name": "talker_node",
      "type": "publisher",
      "topics": ["/chatter"],
      "status": "active"
    },
    {
      "id": "node_456", 
      "name": "listener_node",
      "type": "subscriber",
      "topics": ["/chatter"],
      "status": "active"
    }
  ]
}
```

### POST /nodes
Create a new simulated node.

**Request:**
```json
{
  "name": "my_new_node",
  "type": "publisher",
  "topics": ["/new_topic"],
  "parameters": {}
}
```

**Response (201 Created):**
```json
{
  "id": "node_789",
  "name": "my_new_node",
  "type": "publisher",
  "topics": ["/new_topic"],
  "status": "active",
  "created_at": "2025-12-09T10:00:00Z"
}
```

## Topic Communication

### GET /topics
List all available topics in the simulated system.

**Response (200 OK):**
```json
{
  "topics": [
    {
      "name": "/chatter",
      "type": "std_msgs/String",
      "publishers": 1,
      "subscribers": 1
    },
    {
      "name": "/robot_status",
      "type": "std_msgs/Bool", 
      "publishers": 1,
      "subscribers": 3
    }
  ]
}
```

### POST /topics/{topic_name}/publish
Publish a message to a topic.

**Path Parameters:**
- `topic_name`: The name of the topic (e.g. "/chatter")

**Request:**
```json
{
  "node_id": "node_123",
  "data": {
    "data": "Hello, ROS 2!"
  }
}
```

**Response (200 OK):**
```json
{
  "status": "published",
  "timestamp": "2025-12-09T10:05:00Z",
  "topic": "/chatter",
  "message_id": "msg_001"
}
```

### GET /topics/{topic_name}/messages
Get recent messages from a topic (simulated subscription).

**Path Parameters:**
- `topic_name`: The name of the topic

**Query Parameters:**
- `limit` (optional): Number of messages to return (default: 10)

**Response (200 OK):**
```json
{
  "topic": "/chatter",
  "messages": [
    {
      "id": "msg_001",
      "timestamp": "2025-12-09T10:00:00Z",
      "publisher": "talker_node",
      "data": {
        "data": "Hello, ROS 2!"
      }
    }
  ]
}
```

## Services

### GET /services
List all available services in the simulated system.

**Response (200 OK):**
```json
{
  "services": [
    {
      "name": "/add_two_ints",
      "type": "example_interfaces/AddTwoInts",
      "clients": 1,
      "servers": 1
    }
  ]
}
```

### POST /services/{service_name}/call
Call a service and get a response.

**Path Parameters:**
- `service_name`: The name of the service (e.g. "/add_two_ints")

**Request:**
```json
{
  "a": 10,
  "b": 20
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "result": {
    "sum": 30
  },
  "timestamp": "2025-12-09T10:10:00Z"
}
```

## Robot Model Management

### GET /robots
List available robot models in the simulation.

**Response (200 OK):**
```json
{
  "robots": [
    {
      "id": "robot_001",
      "name": "simple_humanoid",
      "urdf": "simple_humanoid.urdf",
      "links": 5,
      "joints": 4
    }
  ]
}
```

### GET /robots/{robot_id}/model
Get the URDF representation of a robot.

**Path Parameters:**
- `robot_id`: The ID of the robot

**Response (200 OK):**
```json
{
  "id": "robot_001",
  "name": "simple_humanoid",
  "urdf": "<robot name='simple_humanoid'>\n  <link name='base_link'/>\n  <link name='head'/>\n  <joint name='head_joint' type='revolute'>\n    <parent link='base_link'/>\n    <child link='head'/>\n  </joint>\n</robot>",
  "links": [
    { "name": "base_link", "type": "fixed" },
    { "name": "head", "type": "movable" }
  ],
  "joints": [
    { "name": "head_joint", "type": "revolute", "parent": "base_link", "child": "head" }
  ]
}
```

## Error Responses

All error responses follow this format:

**400 Bad Request:**
```json
{
  "error": "invalid_request",
  "message": "The request was invalid or malformed",
  "details": "Additional error details"
}
```

**404 Not Found:**
```json
{
  "error": "resource_not_found",
  "message": "The requested resource was not found"
}
```

**500 Internal Server Error:**
```json
{
  "error": "internal_error",
  "message": "An internal server error occurred"
}
```

## Educational Use Cases

This simulator API is specifically designed for educational purposes to help students understand:

1. Node creation and management in ROS 2
2. Publisher-subscriber communication patterns
3. Service-based request-response communication
4. Robot modeling with URDF
5. Message passing and data types in ROS 2

The API simplifies complex ROS 2 concepts while maintaining the essential patterns that students need to understand.
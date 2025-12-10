# Detailed Cognitive Planning Implementation Guide

## Overview

This document provides a detailed implementation guide for building a cognitive planning system in Vision-Language-Action (VLA) systems. It covers the complete implementation from initial setup to advanced features and best practices.

## Architecture Overview

The cognitive planning system consists of several interconnected components:

1. **Command Interface**: Receives natural language commands from users
2. **Context Manager**: Gathers and maintains environmental context
3. **LLM Interface**: Communicates with Large Language Models for planning
4. **Action Validator**: Validates planned actions for safety and feasibility
5. **Execution Monitor**: Monitors plan execution in real-time

```
[User Command] 
     ↓
[Command Interface]
     ↓
[Context Manager] ←→ [Perception System]
     ↓
[LLM Interface] ←→ [Large Language Model]
     ↓
[Action Validator]
     ↓
[Action Sequence Output]
     ↓
[Execution Monitor] ←→ [Robot System]
```

## Detailed Implementation Steps

### Step 1: Environment Setup

First, set up the required dependencies and environment:

```bash
# Create virtual environment
python3 -m venv vla_env
source vla_env/bin/activate

# Install required packages
pip install openai
pip install rclpy
pip install numpy
pip install geometry_msgs
```

### Step 2: Core Data Structures

Define the core data structures for the cognitive planner:

```python
# action.py
from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass
class Action:
    """
    Represents a single action in an action sequence
    """
    type: str  # The type of action (e.g., 'move_to', 'grasp', 'navigate_to')
    params: Dict[str, Any]  # Parameters for the action
    description: Optional[str] = None  # Human-readable description
    confidence: Optional[float] = None  # Confidence score (0.0 to 1.0)
    timeout: Optional[float] = 30.0  # Execution timeout in seconds


@dataclass
class WorldState:
    """
    Represents the current state of the world/robot environment
    """
    robot_position: Dict[str, float]  # x, y, z coordinates
    robot_state: Dict[str, Any]  # Battery, holding object, etc.
    objects: Dict[str, Dict[str, Any]]  # Objects in the environment
    map: str  # Current map/environment
    timestamp: float  # When this state was captured
```

### Step 3: Context Management

Implement the context manager to maintain environmental context:

```python
import json
import copy
from datetime import datetime


class ContextManager:
    """
    Manages the environmental context for cognitive planning
    """
    
    def __init__(self):
        self.current_state = WorldState(
            robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
            robot_state={"battery": 1.0, "holding": None},
            objects={},
            map="default",
            timestamp=datetime.now().timestamp()
        )
        self.perception_interface = None  # To be set later
    
    def update_from_perception(self):
        """
        Update world state based on perception data
        """
        if self.perception_interface:
            # Get updated object positions from perception system
            detected_objects = self.perception_interface.get_detected_objects()
            
            # Update the current state
            self.current_state.objects = detected_objects
            self.current_state.timestamp = datetime.now().timestamp()
    
    def get_context_for_planning(self):
        """
        Get a dictionary representation of the context for LLM planning
        """
        return {
            "robot_position": self.current_state.robot_position,
            "robot_state": self.current_state.robot_state,
            "objects": self.current_state.objects,
            "map": self.current_state.map,
            "timestamp": self.current_state.timestamp
        }
    
    def apply_action_effects(self, action):
        """
        Update world state based on the effects of an action
        """
        new_state = copy.deepcopy(self.current_state)
        
        if action.type == "grasp":
            obj_name = action.params.get("object_name")
            if obj_name in new_state.objects:
                new_state.robot_state["holding"] = obj_name
                # Update object's position to robot's hand
                # (Implementation depends on robot's kinematics)
        
        elif action.type == "place_at":
            if new_state.robot_state["holding"]:
                held_obj = new_state.robot_state["holding"]
                new_state.objects[held_obj]["position"] = action.params
                new_state.robot_state["holding"] = None
        
        elif action.type == "move_to":
            new_state.robot_position = {
                "x": action.params.get("x", new_state.robot_position["x"]),
                "y": action.params.get("y", new_state.robot_position["y"]),
                "z": action.params.get("z", new_state.robot_position.get("z", 0.0))
            }
        
        new_state.timestamp = datetime.now().timestamp()
        self.current_state = new_state
```

### Step 4: LLM Interface

Implement the interface to communicate with the LLM:

```python
from openai import OpenAI
import os
import json


class LLMInterface:
    """
    Interface to communicate with Large Language Models for cognitive planning
    """
    
    def __init__(self, api_key=None):
        if api_key:
            self.client = OpenAI(api_key=api_key)
        else:
            self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
        # Available actions that the LLM can plan with
        self.available_actions = {
            "move_to": {
                "description": "Move robot to specified coordinates",
                "params": {"x": "float", "y": "float", "z": "float (optional)"}
            },
            "grasp": {
                "description": "Grasp an object by name",
                "params": {"object_name": "string"}
            },
            "detect_object": {
                "description": "Detect a specific object in the environment",
                "params": {"object_name": "string"}
            },
            "navigate_to": {
                "description": "Navigate to an object",
                "params": {"object_name": "string"}
            },
            "place_at": {
                "description": "Place held object at specified coordinates",
                "params": {"x": "float", "y": "float", "z": "float"}
            },
            "pick_and_place": {
                "description": "Pick up an object and place it at a target location",
                "params": {"object_name": "string", "target_name": "string"}
            }
        }
    
    def plan_action_sequence(self, command, context):
        """
        Plan an action sequence based on a command and context
        """
        # Construct the prompt with context and command
        prompt = self._construct_prompt(command, context)
        
        try:
            # Call the LLM
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are a cognitive planning system for a robot. "
                            "Given the user's command and environmental context, "
                            "generate a sequence of actions for the robot to execute. "
                            "Output only valid JSON as specified."
                        )
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                max_tokens=1000,
                temperature=0.3
            )
            
            # Extract and parse the response
            response_text = response.choices[0].message.content.strip()
            
            # Clean up markdown formatting
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]
            
            # Parse JSON response
            result = json.loads(response_text)
            
            # Validate the structure
            if 'actions' not in result:
                raise ValueError("Response does not contain 'actions' field")
            
            # Convert to Action objects
            actions = []
            for action_dict in result['actions']:
                action = Action(
                    type=action_dict['type'],
                    params=action_dict.get('params', {}),
                    description=action_dict.get('description'),
                    confidence=action_dict.get('confidence')
                )
                actions.append(action)
            
            return actions, result.get('reasoning', '')
        
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON response: {e}")
            return None, ""
        except Exception as e:
            print(f"Error in LLM planning: {e}")
            return None, ""
    
    def _construct_prompt(self, command, context):
        """
        Construct the prompt for the LLM
        """
        prompt = f"""
        The user wants the robot to: "{command}"
        
        Current environmental context:
        {json.dumps(context, indent=2)}
        
        Available robot actions:
        {json.dumps(self.available_actions, indent=2)}
        
        Generate a sequence of actions for the robot to complete the user's request.
        Consider the environmental context and the robot's current state.
        
        Respond with only valid JSON in this format:
        {{
            "actions": [
                {{
                    "type": "navigate_to",
                    "params": {{"object_name": "red cup"}},
                    "description": "Navigate to the red cup",
                    "confidence": 0.9
                }},
                {{
                    "type": "grasp",
                    "params": {{"object_name": "red cup"}},
                    "description": "Grasp the red cup",
                    "confidence": 0.95
                }}
            ],
            "reasoning": "A brief explanation of how the plan achieves the goal"
        }}
        """
        
        return prompt
```

### Step 5: Action Validation System

Implement the validation system for action sequences:

```python
class ActionValidator:
    """
    Validates action sequences for safety and feasibility
    """
    
    def __init__(self, robot_interface, context_manager):
        self.robot_interface = robot_interface
        self.context_manager = context_manager
    
    def validate_action_sequence(self, actions):
        """
        Validate an entire action sequence
        """
        # Check syntax of each action
        for i, action in enumerate(actions):
            if not self._validate_syntax(action):
                return False, f"Invalid syntax in action {i}: {action}"
        
        # Check semantics of each action
        for i, action in enumerate(actions):
            is_valid, reason = self._validate_semantics(action)
            if not is_valid:
                return False, f"Semantic error in action {i}: {reason}"
        
        # Check safety of each action
        for i, action in enumerate(actions):
            is_safe, reason = self._validate_safety(action)
            if not is_safe:
                return False, f"Safety error in action {i}: {reason}"
        
        # Check sequence coherence
        is_coherent, reason = self._validate_sequence_coherence(actions)
        if not is_coherent:
            return False, f"Sequence coherence error: {reason}"
        
        return True, "Action sequence is valid"
    
    def _validate_syntax(self, action):
        """
        Validate the syntax of a single action
        """
        if not isinstance(action, Action):
            return False
        
        if not isinstance(action.type, str):
            return False
        
        if not isinstance(action.params, dict):
            return False
        
        # Check required parameters based on action type
        required_params = {
            'move_to': ['x', 'y'],
            'grasp': ['object_name'],
            'detect_object': ['object_name'],
            'navigate_to': ['object_name'],
            'place_at': ['x', 'y', 'z'],
            'pick_and_place': ['object_name', 'target_name']
        }
        
        if action.type in required_params:
            for param in required_params[action.type]:
                if param not in action.params:
                    return False
        
        return True
    
    def _validate_semantics(self, action):
        """
        Validate that the action makes semantic sense
        """
        context = self.context_manager.current_state
        
        # Check if robot can perform the action given its state
        if action.type == 'grasp' and context.robot_state.get('holding') is not None:
            return False, "Robot is already holding an object"
        
        # Check if required objects exist
        if action.type in ['grasp', 'detect_object', 'navigate_to']:
            obj_name = action.params.get('object_name')
            if obj_name not in context.objects:
                return False, f"Object '{obj_name}' does not exist in environment"
        
        return True, "Action is semantically valid"
    
    def _validate_safety(self, action):
        """
        Validate that the action is safe to execute
        """
        # Placeholder for safety validation
        # In a real system, this would check for obstacles, robot limits, etc.
        
        # For move_to actions, check if destination is safe
        if action.type in ['move_to', 'navigate_to']:
            # In a real system, check path planning and obstacles
            pass
        
        # For grasp actions, check if object is safe to grasp
        if action.type == 'grasp':
            obj_name = action.params.get('object_name')
            if obj_name in self.context_manager.current_state.objects:
                obj = self.context_manager.current_state.objects[obj_name]
                # Check if object properties are safe for grasping
                if obj.get('temperature', 'normal') == 'hot':
                    return False, f"Object '{obj_name}' is too hot to grasp safely"
        
        return True, "Action is safe to execute"
    
    def _validate_sequence_coherence(self, actions):
        """
        Validate that the sequence of actions makes logical sense
        """
        # For now, just check if the sequence is not empty
        if not actions:
            return False, "Action sequence is empty"
        
        # Check if actions can be chained together properly
        # For example, you should not try to place an object when not holding anything
        holding_state = None  # Initially, robot is not holding anything
        
        for action in actions:
            if action.type == 'grasp':
                if holding_state is not None:
                    return False, "Cannot grasp object when already holding one"
                holding_state = action.params.get('object_name')
            elif action.type == 'place_at' or action.type == 'place_object':
                if holding_state is None:
                    return False, "Cannot place object when not holding anything"
                holding_state = None  # After placing, robot is not holding anything
        
        return True, "Action sequence is coherent"
```

### Step 6: Complete Cognitive Planner ROS Node

Now, we'll tie everything together in a complete ROS node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence
from rclpy.qos import QoSProfile


class DetailedCognitivePlannerNode(Node):
    """
    Complete cognitive planner implementation with all components
    """
    
    def __init__(self):
        super().__init__('detailed_cognitive_planner_node')
        
        # Initialize components
        self.context_manager = ContextManager()
        self.llm_interface = LLMInterface()
        self.action_validator = ActionValidator(self, self.context_manager)
        
        # Set up ROS interfaces
        self.command_subscription = self.create_subscription(
            String,
            '/vla/natural_command',
            self.command_callback,
            QoSProfile(depth=10)
        )
        
        self.action_publisher = self.create_publisher(
            ActionSequence,
            '/vla/action_sequence',
            QoSProfile(depth=10)
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla/cognitive_planner_status',
            QoSProfile(depth=10)
        )
        
        self.get_logger().info('Detailed Cognitive Planner Node initialized')
    
    def command_callback(self, msg):
        """
        Process natural language command and generate action sequence
        """
        try:
            command_text = msg.data
            self.get_logger().info(f'Received command: {command_text}')
            
            # Update context from perception
            self.context_manager.update_from_perception()
            
            # Plan the action sequence
            actions, reasoning = self.llm_interface.plan_action_sequence(
                command_text, 
                self.context_manager.get_context_for_planning()
            )
            
            if actions:
                # Validate the planned action sequence
                is_valid, reason = self.action_validator.validate_action_sequence(actions)
                
                if is_valid:
                    # Publish the validated action sequence
                    action_msg = self._create_action_sequence_msg(actions, command_text)
                    self.action_publisher.publish(action_msg)
                    
                    self.get_logger().info(f'Published validated action sequence with {len(actions)} actions')
                    self._publish_status(f'Successfully planned {len(actions)} actions')
                else:
                    self.get_logger().error(f'Action sequence validation failed: {reason}')
                    self._publish_status(f'Validation failed: {reason}')
            else:
                self.get_logger().error('Failed to generate action sequence')
                self._publish_status('Failed to generate action sequence')
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            self._publish_status(f'Error processing command: {str(e)}')
    
    def _create_action_sequence_msg(self, actions, original_command):
        """
        Convert internal action sequence to ROS message
        """
        msg = ActionSequence()
        msg.request_id = self._generate_request_id()
        msg.command = original_command
        msg.actions = [self._action_to_ros_action(action) for action in actions]
        
        return msg
    
    def _action_to_ros_action(self, action):
        """
        Convert internal action to ROS action message
        """
        # This would convert our Action dataclass to the appropriate ROS message
        # For now, we'll return a placeholder
        from vla_msgs.msg import VLAAction
        ros_action = VLAAction()
        ros_action.type = action.type
        ros_action.params = str(action.params)  # Simplified conversion
        ros_action.description = action.description or ""
        ros_action.confidence = action.confidence or 0.0
        
        return ros_action
    
    def _publish_status(self, status_msg):
        """
        Publish status message
        """
        status = String()
        status.data = status_msg
        self.status_publisher.publish(status)
    
    def _generate_request_id(self):
        """
        Generate unique request ID
        """
        import uuid
        return str(uuid.uuid4())


def main(args=None):
    """
    Main function to run the detailed cognitive planner node
    """
    rclpy.init(args=args)
    node = DetailedCognitivePlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Detailed Cognitive Planner Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced Considerations

### Error Handling and Recovery

Implement robust error handling to manage failures during planning and execution:

```python
def handle_planning_error(self, error_type, error_details):
    """
    Handle different types of planning errors
    """
    if error_type == "llm_timeout":
        # Retry with a simpler prompt or different model
        self._retry_with_simpler_prompt(error_details)
    elif error_type == "context_unavailable":
        # Wait for context to be updated
        self._wait_for_context_update()
    elif error_type == "invalid_command":
        # Request clarification from user
        self._request_command_clarification()
```

### Performance Optimization

For production systems, consider:

1. **Caching**: Cache common action sequences to reduce LLM calls
2. **Parallel Processing**: Process perception and planning in parallel
3. **Model Optimization**: Use fine-tuned models for specific tasks

### Security Considerations

1. **API Key Management**: Use secure methods to store and access API keys
2. **Command Validation**: Validate all commands before processing
3. **Rate Limiting**: Implement rate limiting to prevent abuse

## Testing and Validation

Create comprehensive tests for the cognitive planning system:

```python
import unittest


class TestCognitivePlanner(unittest.TestCase):
    def setUp(self):
        self.planner = DetailedCognitivePlannerNode()
        # Set up mock interfaces for testing
    
    def test_simple_navigation_command(self):
        # Test a simple navigation command
        command = "Go to the table"
        actions = self.planner.llm_interface.plan_action_sequence(
            command, 
            {"robot_position": {"x": 0, "y": 0}, "objects": {"table": {"position": {"x": 2, "y": 2}}}}
        )
        
        self.assertIsNotNone(actions)
        self.assertGreater(len(actions), 0)
        self.assertTrue(any(action.type == "navigate_to" for action in actions))
    
    def test_grasp_command(self):
        # Test a grasping command
        command = "Pick up the red cup"
        actions = self.planner.llm_interface.plan_action_sequence(
            command, 
            {"robot_position": {"x": 0, "y": 0}, 
             "objects": {"red_cup": {"position": {"x": 1, "y": 1}}}}
        )
        
        self.assertIsNotNone(actions)
        # Verify that the sequence includes navigate_to followed by grasp
        nav_action = next((a for a in actions if a.type == "navigate_to"), None)
        grasp_action = next((a for a in actions if a.type == "grasp"), None)
        self.assertIsNotNone(nav_action)
        self.assertIsNotNone(grasp_action)
```

## Conclusion

This detailed implementation guide covers the complete cognitive planning system for VLA systems. Key points to remember:

1. Always validate action sequences for safety before execution
2. Maintain accurate environmental context for effective planning
3. Implement robust error handling and recovery mechanisms
4. Consider performance and security in production deployments
5. Test thoroughly with various command types and scenarios

The cognitive planning system serves as the intelligence layer that bridges natural language understanding with robotic action execution, enabling intuitive human-robot interaction.
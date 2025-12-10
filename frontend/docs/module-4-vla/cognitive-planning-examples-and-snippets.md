# Cognitive Planning: Examples and Code Snippets

## Overview

This document provides practical examples and code snippets for implementing cognitive planning in Vision-Language-Action (VLA) systems. These examples demonstrate core concepts and can be used as starting points for your own implementations.

## Basic Cognitive Planner Node

### Simple Cognitive Planner Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import json
import os
from rclpy.qos import QoSProfile


class BasicCognitivePlannerNode(Node):
    """
    A basic cognitive planner that translates natural language to action sequences
    """
    
    def __init__(self):
        super().__init__('basic_cognitive_planner')
        
        # Initialize OpenAI client
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
        # Set up communication
        self.subscription = self.create_subscription(
            String,
            '/vla/natural_command',
            self.command_callback,
            QoSProfile(depth=10)
        )
        
        self.action_publisher = self.create_publisher(
            String,  # In production, use proper action sequence message
            '/vla/action_sequence',
            QoSProfile(depth=10)
        )
        
        self.get_logger().info('Basic Cognitive Planner initialized')
    
    def command_callback(self, msg):
        """
        Process natural language command and generate action sequence
        """
        command_text = msg.data
        self.get_logger().info(f'Processing command: {command_text}')
        
        # Plan actions using LLM
        action_sequence = self.plan_actions(command_text)
        
        if action_sequence:
            # Publish the action sequence
            action_msg = String()
            action_msg.data = json.dumps(action_sequence)  # In practice, use proper message type
            self.action_publisher.publish(action_msg)
            
            self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')
        else:
            self.get_logger().error('Failed to generate action sequence')
    
    def plan_actions(self, command):
        """
        Plan actions for the given command using LLM
        """
        # Define a simple environment context for this example
        context = {
            "objects": [
                {"name": "red cup", "position": [1.0, 0.5]},
                {"name": "blue box", "position": [1.5, 1.0]},
                {"name": "table", "position": [0.8, 0.8]}
            ],
            "robot_position": [0.0, 0.0]
        }
        
        # Construct prompt
        prompt = f"""
        Given the user command "{command}" and the following environment:
        {json.dumps(context, indent=2)}
        
        Generate a sequence of actions from this list:
        - navigate_to(object_name)
        - grasp(object_name)
        - place_at(x, y)
        - move_to(x, y)
        
        Respond with valid JSON like:
        {{
            "actions": [
                {{"type": "navigate_to", "params": {{"object_name": "red cup"}}}},
                {{"type": "grasp", "params": {{"object_name": "red cup"}}}}
            ]
        }}
        """
        
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot action planner. Respond with valid JSON only."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=500
            )
            
            # Parse response
            response_text = response.choices[0].message.content.strip()
            
            # Remove markdown formatting if present
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]
            
            result = json.loads(response_text)
            return result.get('actions', [])
        
        except Exception as e:
            self.get_logger().error(f'Error in action planning: {e}')
            return []


def main(args=None):
    rclpy.init(args=args)
    node = BasicCognitivePlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cognitive planner stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Advanced Examples

### Example 1: Object Retrieval Task

```python
def plan_object_retrieval(self, object_name, destination):
    """
    Plan actions to retrieve an object and bring it to a destination
    """
    prompt = f"""
    The robot needs to retrieve the '{object_name}' and bring it to the '{destination}'.
    
    Current environment:
    - Robot is at position (0, 0)
    - {object_name} is at position (2, 1)
    - {destination} is at position (3, 3)
    
    Plan the following sequence:
    1. Navigate to the {object_name}
    2. Grasp the {object_name}
    3. Navigate to the {destination}
    4. Place the {object_name}
    
    Output JSON with actions.
    """
    
    # Call LLM with prompt
    response = self.client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a robot action planner. Respond with valid JSON only."},
            {"role": "user", "content": prompt}
        ],
        max_tokens=500
    )
    
    try:
        response_text = response.choices[0].message.content.strip()
        if response_text.startswith('```json'):
            response_text = response_text[7:]
        if response_text.endswith('```'):
            response_text = response_text[:-3]
        
        result = json.loads(response_text)
        return result.get('actions', [])
    except Exception as e:
        self.get_logger().error(f'Error in object retrieval planning: {e}')
        return []

# Usage example
def example_object_retrieval():
    planner = BasicCognitivePlannerNode()
    actions = planner.plan_object_retrieval("red cup", "kitchen counter")
    
    for action in actions:
        print(f"{action['type']}: {action.get('params', {})}")
```

### Example 2: Multi-Step Task Planning

```python
class MultiStepPlanner:
    """
    Planner for complex multi-step tasks
    """
    
    def plan_cleaning_task(self, room_name):
        """
        Plan a sequence to clean a room by collecting items
        """
        prompt = f"""
        Plan a cleaning task for the '{room_name}'.
        
        Steps needed:
        1. Detect all objects not in their designated locations
        2. For each misplaced object, navigate to it and move to its proper location
        3. Return to the starting position
        
        Environment context:
        - Room contains: 'red toy', 'blue block', 'green book', 'wastebasket'
        - Designated locations: 
          * 'red toy' -> 'toy box'
          * 'blue block' -> 'toy box' 
          * 'green book' -> 'bookshelf'
        
        Output a sequence of actions as JSON.
        """
        
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot action planner for cleaning tasks. Respond with valid JSON only."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=800
            )
            
            response_text = response.choices[0].message.content.strip()
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]
            
            result = json.loads(response_text)
            return result.get('actions', [])
        except Exception as e:
            print(f"Error in cleaning task planning: {e}")
            return []

# Usage
multi_planner = MultiStepPlanner()
cleaning_actions = multi_planner.plan_cleaning_task("living room")
for i, action in enumerate(cleaning_actions):
    print(f"Step {i+1}: {action['type']} - {action.get('params', {})}")
```

## Common Prompt Patterns

### Pattern 1: Simple Action Mapping

```python
def create_simple_mapping_prompt(command, available_actions):
    """
    Create a prompt for basic command-to-action mapping
    """
    return f"""
    Map the command "{command}" to a sequence of actions.
    
    Available actions: {', '.join(available_actions)}
    
    Output the action sequence as JSON:
    {{
        "actions": [
            {{"type": "action_name", "params": {{"param1": "value1"}}}}
        ]
    }}
    """
```

### Pattern 2: Context-Aware Planning

```python
def create_context_aware_prompt(command, environment_context, available_actions):
    """
    Create a prompt that incorporates environmental context
    """
    return f"""
    Given the command "{command}" and the following environmental context:
    {json.dumps(environment_context, indent=2)}
    
    Select actions from: {', '.join(available_actions)}
    
    Consider:
    - Current robot state and position
    - Locations of relevant objects
    - Obstacles or constraints
    
    Output the action sequence as JSON:
    {{
        "actions": [
            {{"type": "action_name", "params": {{"param1": "value1"}}}}
        ],
        "reasoning": "Brief explanation of the plan"
    }}
    """
```

### Pattern 3: Safety-Conscious Planning

```python
def create_safe_planning_prompt(command, environment_context):
    """
    Create a prompt that emphasizes safety in action planning
    """
    return f"""
    Plan actions for command "{command}" considering safety.
    
    Environment: {json.dumps(environment_context, indent=2)}
    
    Safety Guidelines:
    - Avoid collisions with obstacles
    - Don't attempt to grasp unsafe objects
    - Consider robot operational limits
    - Verify action feasibility
    
    Available Actions:
    - move_to(x, y): Move to coordinates
    - navigate_to(object): Navigate to named object
    - grasp(object): Grasp an object
    - place_at(x, y, z): Place held object
    - detect_object(object): Detect object in environment
    
    Output JSON with actions and safety considerations:
    {{
        "actions": [...],
        "safety_checks": [...]
    }}
    """
```

## Utility Functions

### Action Validation

```python
def validate_action_sequence(actions, robot_capabilities):
    """
    Validate an action sequence against robot capabilities
    """
    valid_types = robot_capabilities.get('action_types', [])
    required_params = robot_capabilities.get('required_params', {})
    
    for action in actions:
        # Check if action type is supported
        if action['type'] not in valid_types:
            print(f"Invalid action type: {action['type']}")
            return False
        
        # Check required parameters
        action_type = action['type']
        if action_type in required_params:
            for param in required_params[action_type]:
                if param not in action.get('params', {}):
                    print(f"Missing required parameter {param} for {action_type}")
                    return False
    
    return True

# Example usage
robot_caps = {
    'action_types': ['move_to', 'grasp', 'navigate_to', 'place_at'],
    'required_params': {
        'move_to': ['x', 'y'],
        'grasp': ['object_name'],
        'navigate_to': ['object_name'],
        'place_at': ['x', 'y', 'z']
    }
}

actions = [
    {'type': 'navigate_to', 'params': {'object_name': 'red cup'}},
    {'type': 'grasp', 'params': {'object_name': 'red cup'}}
]

is_valid = validate_action_sequence(actions, robot_caps)
print(f"Action sequence valid: {is_valid}")
```

### Environment Context Builder

```python
class EnvironmentContextBuilder:
    """
    Helper class to build standardized environment contexts for planning
    """
    
    def __init__(self):
        self.context = {
            "robot_state": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "battery_level": 1.0,
                "holding_object": None
            },
            "objects": [],
            "map": "default_map",
            "obstacles": [],
            "timestamp": None
        }
    
    def add_object(self, name, position, properties=None):
        """Add an object to the environment"""
        obj = {
            "name": name,
            "position": position,
            "properties": properties or {}
        }
        self.context["objects"].append(obj)
        return self
    
    def set_robot_position(self, x, y, z=0.0):
        """Set robot's current position"""
        self.context["robot_state"]["position"] = {"x": x, "y": y, "z": z}
        return self
    
    def set_robot_battery(self, level):
        """Set robot's battery level (0.0 to 1.0)"""
        self.context["robot_state"]["battery_level"] = level
        return self
    
    def set_robot_holding(self, object_name):
        """Set what object the robot is holding"""
        self.context["robot_state"]["holding_object"] = object_name
        return self
    
    def build(self):
        """Return the constructed context"""
        self.context["timestamp"] = time.time()
        return self.context

# Usage example
context_builder = EnvironmentContextBuilder()
env_context = (context_builder
               .add_object("red cup", [1.0, 0.5])
               .add_object("table", [1.2, 1.0])
               .set_robot_position(0.0, 0.0)
               .set_robot_battery(0.85)
               .build())

print(json.dumps(env_context, indent=2))
```

## Error Handling Patterns

### Graceful Degradation

```python
def plan_with_graceful_degradation(self, command, context):
    """
    Plan actions with fallback strategies if initial approach fails
    """
    strategies = [
        ("precise", self._plan_precise_approach),
        ("approximate", self._plan_approximate_approach),
        ("simplified", self._plan_simplified_approach)
    ]
    
    for strategy_name, strategy_func in strategies:
        try:
            self.get_logger().info(f"Trying {strategy_name} strategy")
            actions = strategy_func(command, context)
            if actions:
                self.get_logger().info(f"Successfully planned using {strategy_name} strategy")
                return actions
        except Exception as e:
            self.get_logger().warning(f"{strategy_name} strategy failed: {e}")
            continue
    
    # If all strategies fail, return empty sequence
    self.get_logger().error("All planning strategies failed")
    return []

def _plan_precise_approach(self, command, context):
    """Try detailed planning first"""
    # Implementation for precise planning
    pass

def _plan_approximate_approach(self, command, context):
    """Try approximate planning if precise fails"""
    # Implementation for approximate planning
    pass

def _plan_simplified_approach(self, command, context):
    """Try simplified planning as last resort"""
    # Implementation for simplified planning
    pass
```

### Timeout Handling

```python
def plan_with_timeout(self, command, context, timeout_seconds=30):
    """
    Plan actions with timeout handling
    """
    import signal
    
    def timeout_handler(signum, frame):
        raise TimeoutError(f"Action planning timed out after {timeout_seconds}s")
    
    # Set up timeout
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(timeout_seconds)
    
    try:
        # Call LLM for planning
        result = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "user", "content": self.construct_prompt(command, context)}
            ],
            max_tokens=500
        )
        
        # Cancel timeout
        signal.alarm(0)
        
        return self.parse_response(result)
    
    except TimeoutError:
        self.get_logger().error(f"Planning timed out for command: {command}")
        return []
    
    except Exception as e:
        # Cancel timeout in case of other errors
        signal.alarm(0)
        self.get_logger().error(f"Error in planning: {e}")
        return []
```

## Testing Examples

### Unit Test for Action Planning

```python
import unittest
from unittest.mock import Mock, patch


class TestCognitivePlanner(unittest.TestCase):
    
    def setUp(self):
        self.planner = BasicCognitivePlannerNode()
    
    @patch('openai.OpenAI')
    def test_plan_simple_navigation(self, mock_openai):
        # Mock LLM response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message.content = '''
        {
            "actions": [
                {"type": "move_to", "params": {"x": 1.0, "y": 2.0}}
            ]
        }
        '''
        
        mock_openai.return_value.chat.completions.create.return_value = mock_response
        
        # Test planning
        actions = self.planner.plan_actions("Move to location (1, 2)")
        
        self.assertEqual(len(actions), 1)
        self.assertEqual(actions[0]['type'], 'move_to')
        self.assertEqual(actions[0]['params']['x'], 1.0)
    
    def test_validate_action_sequence(self):
        # Test action validation
        robot_caps = {
            'action_types': ['move_to', 'grasp'],
            'required_params': {
                'move_to': ['x', 'y'],
                'grasp': ['object_name']
            }
        }
        
        valid_actions = [
            {'type': 'move_to', 'params': {'x': 1.0, 'y': 2.0}}
        ]
        
        invalid_actions = [
            {'type': 'invalid_action', 'params': {}}
        ]
        
        self.assertTrue(validate_action_sequence(valid_actions, robot_caps))
        self.assertFalse(validate_action_sequence(invalid_actions, robot_caps))


# To run tests
if __name__ == '__main__':
    unittest.main()
```

## Summary

This collection of examples and code snippets provides practical building blocks for implementing cognitive planning in VLA systems. Each snippet demonstrates a specific aspect of the planning process and can be combined to create more complex planning systems. The examples include error handling, validation, and best practices for educational VLA implementations.
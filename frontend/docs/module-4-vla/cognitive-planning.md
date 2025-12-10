# Cognitive Planning with Large Language Models

## Overview

Cognitive planning represents a critical component of Vision-Language-Action (VLA) systems. It leverages the reasoning capabilities of Large Language Models (LLMs) to interpret natural language commands and generate executable action sequences for robots. This process involves transforming high-level human instructions into detailed, step-by-step robotic actions that consider environmental constraints and safety requirements.

In this module, we'll explore how LLMs serve as the "cognitive engine" for robots, enabling them to understand complex commands and reason about their execution in dynamic environments. The cognitive planner bridges the gap between human communication and robotic action, allowing for intuitive human-robot interaction.

## The Cognitive Planning Process

The cognitive planning process in VLA systems involves several key steps:

1. **Natural Language Understanding**: Interpreting the user's command in natural language
2. **Environmental Context Integration**: Incorporating sensor data about the current environment
3. **Action Sequence Generation**: Creating a series of low-level robot commands
4. **Safety Validation**: Ensuring the planned sequence is safe for execution
5. **Action Prioritization**: Ordering and scheduling commands appropriately

### Natural Language Understanding

LLMs excel at understanding the semantics of natural language commands. However, in robotic contexts, we need to ensure the model generates structured outputs that can be directly translated to robot actions. This requires careful prompt engineering to constrain the output format.

Example prompt structure:
```
You are a cognitive planning system for a robot. Given the user command and environment state, generate a sequence of actions for the robot.

Environment: {object positions, obstacles, robot position}
User Command: "{natural language command}"
Available Actions: [move_to(x, y), grasp(object), detect_object(name), navigate_to(object)]

Output format:
- Action 1: action_type(parameters)
- Action 2: action_type(parameters)
...
```

### Environmental Context Integration

The cognitive planner must incorporate environmental information to generate contextually appropriate actions. This includes:

- Current positions of objects in the environment
- Robot's current state (location, battery level, etc.)
- Obstacle locations that may affect navigation
- Map information for navigation planning

### Action Sequence Generation

The planner generates a sequence of actions that transform the environment from its current state to satisfy the user's command. This requires:

- Spatial reasoning to determine object relationships
- Temporal reasoning to order actions appropriately
- Conditional reasoning to adapt to environmental changes

## Implementation Architecture

### Cognitive Planner Node Design

The cognitive planner is implemented as a ROS 2 node that subscribes to natural language commands and publishes action sequences. Key responsibilities include:

1. Receiving natural language commands
2. Querying LLM with appropriate context
3. Parsing LLM response into structured actions
4. Validating action sequences for safety
5. Publishing action sequences to execution nodes

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI  # Using OpenAI API for this example
import json
import os
from rclpy.qos import QoSProfile
from vla_msgs.msg import PlanCognitiveTask, ActionSequence

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')
        
        # Initialize OpenAI client
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
        # Subscription to natural language commands
        self.command_subscription = self.create_subscription(
            String,
            '/vla/natural_command',
            self.command_callback,
            QoSProfile(depth=10)
        )
        
        # Publisher for action sequences
        self.action_publisher = self.create_publisher(
            ActionSequence,
            '/vla/action_sequence',
            QoSProfile(depth=10)
        )
        
        self.get_logger().info('Cognitive Planner Node initialized')

    def command_callback(self, msg):
        """Process natural language command and generate action sequence"""
        try:
            command_text = msg.data
            self.get_logger().info(f'Received command: {command_text}')
            
            # Query LLM for action planning
            action_sequence = self.plan_actions(command_text)
            
            if action_sequence:
                # Publish the action sequence
                action_msg = ActionSequence()
                action_msg.actions = action_sequence
                action_msg.request_id = self.generate_request_id()
                
                self.action_publisher.publish(action_msg)
                self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')
            else:
                self.get_logger().error('Failed to generate action sequence')
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def plan_actions(self, natural_language_command):
        """Use LLM to plan multi-step actions from natural language"""
        try:
            # Construct prompt with environmental context
            prompt = self.construct_prompt(natural_language_command)
            
            # Call the LLM
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",  # Or gpt-4 for more complex tasks
                messages=[
                    {"role": "system", "content": "You are a cognitive planning system for a robot. Generate a sequence of actions that the robot should perform to complete the user's request. Output only valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=500,
                temperature=0.3
            )
            
            # Extract and parse the response
            response_text = response.choices[0].message.content.strip()
            
            # Remove any markdown formatting
            if response_text.startswith('```json'):
                response_text = response_text[7:]  # Remove ```json
            if response_text.endswith('```'):
                response_text = response_text[:-3]  # Remove ```
                
            action_sequence = json.loads(response_text)
            
            return action_sequence['actions']
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response as JSON: {str(e)}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error in LLM query: {str(e)}')
            return None

    def construct_prompt(self, natural_language_command):
        """Construct prompt with environmental context"""
        # In a real implementation, this would come from perception system
        environment_context = {
            "objects": [
                {"name": "red cup", "position": {"x": 1.2, "y": 0.5, "z": 0.8}},
                {"name": "blue box", "position": {"x": 0.8, "y": 1.0, "z": 0.8}},
                {"name": "table", "position": {"x": 1.0, "y": 0.7, "z": 0.0}}
            ],
            "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "robot_capabilities": ["move", "grasp", "detect_object"],
            "map": "simple_room_with_table"
        }
        
        prompt = f"""
        The user wants the robot to: "{natural_language_command}"
        
        Current environment:
        {json.dumps(environment_context, indent=2)}
        
        Available robot actions:
        - move_to(x, y): Move robot to specified coordinates
        - grasp(object_name): Grasp an object by name
        - detect_object(object_name): Detect a specific object in the environment
        - navigate_to(object_name): Navigate to an object
        - place_at(x, y, z): Place held object at specified coordinates
        
        Generate a sequence of actions for the robot to complete the user's request.
        Respond with only valid JSON in this format:
        {{
            "actions": [
                {{"type": "move_to", "params": {{"x": 1.0, "y": 1.0}}}},
                {{"type": "detect_object", "params": {{"object_name": "red cup"}}}},
                {{"type": "grasp", "params": {{"object_name": "red cup"}}}}
            ]
        }}
        """
        
        return prompt

    def generate_request_id(self):
        """Generate unique request ID for tracking"""
        import uuid
        return str(uuid.uuid4())

def main(args=None):
    rclpy.init(args=args)
    cognitive_planner_node = CognitivePlannerNode()
    
    try:
        rclpy.spin(cognitive_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        cognitive_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety Considerations in Cognitive Planning

Cognitive planning with LLMs introduces unique safety challenges. Since LLMs can generate unpredictable outputs, it's essential to implement multiple safety layers:

1. **Action Validation**: Verify each generated action is safe and executable
2. **Context Awareness**: Ensure actions align with environmental constraints
3. **Timeout Mechanisms**: Implement timeouts for action sequences
4. **Human Oversight**: Provide mechanisms for human intervention

### Action Validation Example

```python
def validate_action(self, action):
    """Validate an action for safety and executability"""
    if action['type'] == 'move_to':
        # Check if destination is navigable
        x, y = action['params']['x'], action['params']['y']
        if not self.is_navigable(x, y):
            return False, f"Destination ({x}, {y}) is not navigable"
    
    elif action['type'] == 'grasp':
        # Check if object is graspable
        obj_name = action['params']['object_name']
        if not self.is_graspable(obj_name):
            return False, f"Object {obj_name} is not graspable"
    
    return True, "Valid action"
```

## Prompt Engineering Strategies

Effective cognitive planning relies heavily on well-crafted prompts that guide the LLM toward generating structured, executable actions. Here are several key strategies:

1. **Explicit Output Format**: Clearly specify the expected output format
2. **Role Definition**: Define the LLM's role as a planner
3. **Constraint Specification**: Outline limitations and constraints
4. **Context Provision**: Provide necessary environmental information
5. **Example Provision**: Include examples of correct action sequences

### Advanced Prompt Engineering

For more complex cognitive planning scenarios, consider implementing few-shot learning examples within your prompts. This technique provides the LLM with examples of how to properly structure its response:

```
Example 1:
User: "Pick up the red cup and place it on the table"
System:
- Detect the red cup at position (1.2, 0.5, 0.8)
- Navigate to the red cup
- Grasp the red cup
- Navigate to the table at position (1.0, 0.7, 0.0)
- Place the red cup on the table

Example 2:
User: "Move to the blue box and detect what's near it"
System:
- Navigate to the blue box at position (0.8, 1.0, 0.8)
- Use object detection to identify nearby objects
- Report findings

Now process this command: "{user_command}"
```

### Context-Aware Prompting

For more sophisticated planning, incorporate dynamic environmental context. The cognitive planner should request current environmental data before constructing the prompt to ensure the LLM has accurate information about object locations and robot state.

## Action Sequence Generation

The cognitive planner must generate valid action sequences that are executable by the robot. This involves translating high-level commands into specific, low-level robot actions.

### Multi-Step Task Planning

For complex commands, the planner needs to break down the task into subgoals. For example, "Bring the ball to the kitchen" might involve:
1. Locate the ball
2. Navigate to the ball
3. Grasp the ball
4. Locate the kitchen
5. Navigate to the kitchen
6. Place the ball

### Approaches to Action Sequence Generation

There are several approaches to generating action sequences from natural language:

1. **Direct Mapping**: Map language directly to actions (simple but limited)
2. **Intermediate Representation**: Convert language to an intermediate representation before mapping to actions
3. **Symbolic Planning**: Use formal logic or planning languages as an intermediate step
4. **Neural-Symbolic Integration**: Combine neural understanding with symbolic reasoning

### State Representation for Planning

Effective action sequence generation requires maintaining and updating the world state:

```python
class WorldState:
    def __init__(self):
        self.objects = {}  # {object_name: properties}
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'z': 0},
            'holding': None,
            'battery': 1.0
        }
        self.environment = {
            'map': 'default_map',
            'obstacles': []
        }

    def update_after_action(self, action):
        """Update world state based on action effects"""
        action_type = action['type']
        if action_type == 'grasp':
            obj_name = action['params']['object_name']
            if obj_name in self.objects:
                self.robot_state['holding'] = obj_name
                # Update object's position to robot's hand
        elif action_type == 'place_at':
            if self.robot_state['holding']:
                # Place held object at new location
                held_obj = self.robot_state['holding']
                self.objects[held_obj]['position'] = action['params']
                self.robot_state['holding'] = None
```

### Reasoning Explanation

Educational VLA systems should include explanations of how the cognitive planner reasons about tasks. This enhances the learning experience:

```python
def plan_with_explanation(self, command):
    """
    Plan actions with educational explanation of the reasoning
    """
    prompt = f"""
    Plan actions to: {command}

    After each action, provide:
    1. The action
    2. Why this action is needed
    3. What state it brings the system to
    4. How it contributes to the overall goal

    Format response as JSON with 'actions' and 'reasoning' fields.
    """

    # ... execute LLM call

    return result
```

### Handling Ambiguity

Natural language often contains ambiguity that must be resolved during action sequence generation:

- **Referential Ambiguity**: "Pick up the cup" when there are multiple cups
- **Spatial Ambiguity**: "Go to the table" when there are multiple tables
- **Action Ambiguity**: "Move the box" when the target location isn't specified

The cognitive planner should either resolve these ambiguities using context or ask for clarification:

```python
def resolve_ambiguity(self, command, environment):
    """
    Identify and resolve ambiguities in the command
    """
    # Check for multiple objects of the same type
    objects_mentioned = self.extract_objects(command)
    for obj in objects_mentioned:
        matches = [o for o in environment.objects if o.name == obj]
        if len(matches) > 1:
            # Ask for clarification
            return self.request_clarification(command, matches)

    # If no ambiguities, return original command
    return command
```

### Implementation of Action Sequence Generation

To implement action sequence generation effectively, we need to follow a structured approach:

1. Parse the natural language command
2. Integrate environmental context
3. Generate action sequence using LLM
4. Validate and refine the sequence
5. Output the final sequence

Here's a complete implementation of the action sequence generation:

```python
import json
from openai import OpenAI
import os


class ActionSequenceGenerator:
    def __init__(self):
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

    def generate_action_sequence(self, natural_language_command, environment_context):
        """
        Generate an action sequence from natural language command and environmental context
        """
        # Construct the prompt with context
        prompt = self.construct_generation_prompt(natural_language_command, environment_context)

        try:
            # Query the LLM
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": "You are a robot action sequence planner. Generate a step-by-step sequence of actions for the robot to execute the user's command. Respond with only valid JSON."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                max_tokens=1000,
                temperature=0.2
            )

            # Parse and validate the response
            response_text = response.choices[0].message.content.strip()

            # Clean up markdown formatting
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]

            # Attempt to parse JSON
            action_sequence_data = json.loads(response_text)

            # Validate the structure
            if 'actions' not in action_sequence_data:
                raise ValueError("Response does not contain 'actions' field")

            return action_sequence_data['actions']

        except json.JSONDecodeError as e:
            print(f"Error parsing JSON response: {e}")
            return None
        except Exception as e:
            print(f"Error generating action sequence: {e}")
            return None

    def construct_generation_prompt(self, command, environment):
        """
        Construct the prompt for action sequence generation
        """
        prompt = f"""
        The user wants the robot to: "{command}"

        Current environment context:
        {json.dumps(environment, indent=2)}

        Available robot actions:
        1. move_to(x, y) - Move robot to coordinates (x, y)
        2. grasp(object_name) - Grasp an object by name
        3. detect_object(object_name) - Detect a specific object
        4. navigate_to(object_name) - Navigate to an object
        5. place_at(x, y, z) - Place held object at coordinates
        6. pick_and_place(object_name, target_name) - Pick up and place an object

        Generate a sequence of actions for the robot to complete the user's request.
        Consider the environmental context and the robot's current state.

        Respond with only valid JSON in this format:
        {{
            "actions": [
                {{"type": "navigate_to", "params": {{"object_name": "red cup"}}, "description": "Navigate to the red cup"}},
                {{"type": "grasp", "params": {{"object_name": "red cup"}}, "description": "Grasp the red cup"}},
                {{"type": "navigate_to", "params": {{"object_name": "table"}}, "description": "Navigate to the table"}},
                {{"type": "place_at", "params": {{"x": 1.0, "y": 0.5, "z": 0.8}}, "description": "Place the cup on the table"}}
            ],
            "reasoning": "Explanation of the plan"
        }}
        """

        return prompt
```

## Safety Validation System

Since LLMs can generate potentially unsafe action sequences, a multi-layered validation system is essential:

1. **Syntax Validation**: Verify actions follow expected format
2. **Semantic Validation**: Check if actions make sense in the current context
3. **Safety Validation**: Ensure actions won't cause harm to robot or environment
4. **Execution Validation**: Verify the robot has necessary capabilities
5. **Contextual Validation**: Ensure actions are appropriate given current state

### Safety Validation Implementation

```python
def validate_action_sequence(self, actions):
    """
    Run multiple validation layers on an action sequence
    """
    for action in actions:
        # Syntax validation
        if not self.validate_syntax(action):
            return False, f"Invalid syntax in action: {action}"

        # Semantic validation
        if not self.validate_semantics(action):
            return False, f"Semantically invalid action: {action}"

        # Safety validation
        if not self.validate_safety(action):
            return False, f"Unsafe action: {action}"

    # Validate the sequence as a whole
    if not self.validate_sequence_coherence(actions):
        return False, "Action sequence is not coherent"

    return True, "Valid action sequence"


def validate_syntax(self, action):
    """
    Validate the syntax of a single action
    """
    if not isinstance(action, dict):
        return False

    if 'type' not in action:
        return False

    if 'params' not in action or not isinstance(action['params'], dict):
        return False

    # Check for required parameters based on action type
    action_type = action['type']
    required_params = {
        'move_to': ['x', 'y'],
        'grasp': ['object_name'],
        'detect_object': ['object_name'],
        'navigate_to': ['object_name'],
        'place_at': ['x', 'y', 'z'],
        'pick_and_place': ['object_name', 'target_name']
    }

    if action_type in required_params:
        for param in required_params[action_type]:
            if param not in action['params']:
                return False

    return True


def validate_semantics(self, action):
    """
    Validate that the action makes semantic sense
    """
    action_type = action['type']
    params = action['params']

    # Check if robot can perform the action given its state
    if action_type == 'grasp' and self.robot_state['holding'] is not None:
        # Robot is already holding an object
        return False

    # Check if required objects exist
    if action_type in ['grasp', 'detect_object', 'navigate_to']:
        obj_name = params['object_name']
        if obj_name not in self.world_state.objects:
            return False  # Object doesn't exist in environment

    return True


def validate_safety(self, action):
    """
    Validate that the action is safe to execute
    """
    action_type = action['type']
    params = action['params']

    if action_type == 'move_to' or action_type == 'navigate_to':
        x, y = params.get('x', 0), params.get('y', 0)
        # Check if destination is in safe area (not near obstacles or edges)
        if self.is_in_dangerous_area(x, y):
            return False

    elif action_type == 'grasp':
        obj_name = params['object_name']
        obj = self.world_state.objects[obj_name]
        # Check if object is safe to grasp (not too hot, heavy, sharp, etc.)
        if not self.is_safe_to_grasp(obj):
            return False

    # Check battery level for energy-intensive actions
    if self.robot_state['battery'] < 0.1 and action_type in ['navigate_to', 'move_to']:
        return False  # Low battery, avoid unnecessary movement

    return True


def validate_sequence_coherence(self, actions):
    """
    Validate that the sequence of actions makes sense as a whole
    """
    current_state = copy.deepcopy(self.world_state)

    for i, action in enumerate(actions):
        # Simulate the effect of the action on the state
        if not self.would_be_valid_with_state(action, current_state):
            return False

        # Update the simulated state
        current_state = self.apply_action_to_state(action, current_state)

    return True
```

### Real-time Safety Monitoring

In addition to validating the action sequence before execution, a VLA system should implement real-time safety monitoring during execution:

```python
import time
import copy


class SafetyMonitor:
    def __init__(self, robot_interface):
        self.robot_interface = robot_interface
        self.emergency_stop = False

    def monitor_execution(self, action_sequence):
        """
        Monitor the execution of an action sequence for safety violations
        """
        for i, action in enumerate(action_sequence):
            # Check if emergency stop has been triggered
            if self.emergency_stop:
                return False, "Emergency stop activated"

            # Check robot state during execution
            if not self.is_safe_to_continue():
                return False, f"Safety violation during action {i}"

            # Execute action and monitor for issues
            success, error = self.execute_with_monitoring(action)
            if not success:
                return False, f"Action {i} failed: {error}"

        return True, "Sequence executed safely"

    def is_safe_to_continue(self):
        """
        Check if it's safe to continue execution based on sensor data
        """
        # Check for unexpected obstacles
        if self.robot_interface.has_new_obstacle():
            return False

        # Check for safety-critical sensor values
        if self.robot_interface.is_battery_critical():
            return False

        return True

    def execute_with_monitoring(self, action):
        """
        Execute an action while monitoring for safety
        """
        # Start action execution
        self.robot_interface.execute_action(action)

        # Monitor during execution
        timeout = action.get('timeout', 30.0)  # Default 30 second timeout
        start_time = time.time()

        while self.robot_interface.is_action_running():
            # Check for safety issues
            if not self.is_safe_to_continue():
                self.robot_interface.emergency_stop()
                return False, "Safety violation during action"

            # Check for timeout
            if time.time() - start_time > timeout:
                self.robot_interface.emergency_stop()
                return False, "Action timeout"

            time.sleep(0.1)  # Check every 100ms

        return True, "Action completed successfully"
```

### Safety Guidelines for VLA Systems

When implementing safety validation in VLA systems, consider these guidelines:

1. **Fail-Safe Approach**: If uncertain about safety, err on the side of caution
2. **Layered Validation**: Multiple validation layers provide defense in depth
3. **Human Override**: Always provide a mechanism for human intervention
4. **Continuous Monitoring**: Monitor during execution and stop if safety is compromised
5. **Safe States**: Ensure robot can always return to a known safe state

### Handling Uncertainty

LLMs may generate actions with uncertain outcomes. The safety validation system should:

- Assign confidence scores to each action
- Query for additional information when needed
- Request human confirmation for uncertain actions
- Maintain safe default behavior when uncertain

## Practical Exercise: Implementing a Cognitive Planner

Let's implement a complete cognitive planner that can handle complex commands like "pick up the red cup and place it on the table" or "navigate to the kitchen and wait there".

### Exercise Requirements:

1. Create a cognitive planner node that receives natural language commands
2. Use an LLM to generate action sequences with reasoning explanations
3. Implement multi-layer validation for safety
4. Publish validated action sequences to a ROS 2 topic
5. Include error handling and status reporting

### Implementation Checklist:

- [ ] Environment context retrieval
- [ ] Prompt construction with context
- [ ] LLM query with proper error handling
- [ ] Response parsing and validation
- [ ] Action sequence publishing
- [ ] Status reporting
- [ ] Safety validation

## Summary

Cognitive planning with LLMs enables robots to understand and execute complex natural language commands by breaking them down into structured action sequences. Successful implementation requires careful attention to prompt engineering, safety validation, and environmental context integration. The cognitive planner serves as the intelligence layer that bridges high-level natural language understanding with low-level robotic action execution.
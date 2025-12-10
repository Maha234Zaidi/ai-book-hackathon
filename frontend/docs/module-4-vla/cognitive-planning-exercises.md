# Cognitive Planning Exercises for VLA Systems

## Overview

This collection of exercises is designed to help students and developers practice implementing cognitive planning in Vision-Language-Action (VLA) systems. Each exercise includes problem statements, implementation requirements, and detailed solutions.

## Exercise 1: Basic Command Parser

### Problem Statement
Implement a basic command parser that converts simple natural language commands into structured action sequences. The system should handle commands like "Go to the kitchen", "Pick up the red cup", and "Move forward 2 meters".

### Requirements
1. Parse natural language commands into structured actions
2. Support at least 4 action types: `move_to`, `grasp`, `navigate_to`, `detect_object`
3. Extract relevant parameters from the command
4. Validate that actions make sense in the current context

### Starter Code
```python
class BasicCommandParser:
    def __init__(self):
        self.action_types = ["move_to", "grasp", "navigate_to", "detect_object"]
        self.objects = {}  # To be populated with environment objects
    
    def parse_command(self, command, context=None):
        """
        Parse a natural language command into an action sequence
        """
        # TODO: Implement command parsing logic
        pass
    
    def validate_action(self, action, context):
        """
        Validate that an action is feasible in the current context
        """
        # TODO: Implement validation logic
        pass
```

### Solution
```python
import re
import json


class BasicCommandParser:
    def __init__(self):
        self.action_types = ["move_to", "grasp", "navigate_to", "detect_object"]
        self.objects = {}  # To be populated with environment objects
        self.prepositions = ["to", "at", "on", "in", "up", "down", "forward", "backward"]
    
    def parse_command(self, command, context=None):
        """
        Parse a natural language command into an action sequence
        """
        command = command.lower().strip()
        actions = []
        
        # Handle "go to" commands
        if "go to" in command or "navigate to" in command:
            match = re.search(r"(?:go to|navigate to) (.+)", command)
            if match:
                target = match.group(1).strip()
                action = {
                    "type": "navigate_to",
                    "params": {"object_name": target},
                    "description": f"Navigate to {target}"
                }
                actions.append(action)
        
        # Handle "pick up" or "grasp" commands
        elif "pick up" in command or "grasp" in command:
            # Extract object to grasp
            match = re.search(r"(?:pick up|grasp) (?:the )?(.+?)(?: and|\s*$)", command)
            if match:
                obj = match.group(1).strip()
                action = {
                    "type": "grasp",
                    "params": {"object_name": obj},
                    "description": f"Grasp {obj}"
                }
                actions.append(action)
        
        # Handle "move forward/backward" commands
        elif "move" in command:
            forward_match = re.search(r"move (?:forward|backward) ([0-9.]+) meters?", command)
            if forward_match:
                distance = float(forward_match.group(1))
                direction = 1 if "forward" in command else -1
                action = {
                    "type": "move_to",
                    "params": {
                        "x": direction * distance,  # Simplified - only x direction
                        "y": 0.0
                    },
                    "description": f"Move {'forward' if direction > 0 else 'backward'} {distance} meters"
                }
                actions.append(action)
        
        # Handle "detect object" commands
        elif "detect" in command or "find" in command:
            match = re.search(r"(?:detect|find) (?:the )?(.+)", command)
            if match:
                obj = match.group(1).strip()
                action = {
                    "type": "detect_object",
                    "params": {"object_name": obj},
                    "description": f"Detect {obj}"
                }
                actions.append(action)
        
        return actions
    
    def validate_action(self, action, context):
        """
        Validate that an action is feasible in the current context
        """
        if not context:
            context = {"objects": [], "robot_state": {"holding": None}}
        
        action_type = action["type"]
        
        if action_type == "grasp":
            obj_name = action["params"]["object_name"]
            # Check if the object exists in the environment
            obj_exists = any(obj.get("name") == obj_name for obj in context.get("objects", []))
            if not obj_exists:
                return False, f"Object '{obj_name}' does not exist in the environment"
            
            # Check if robot is already holding an object
            if context["robot_state"].get("holding"):
                return False, "Robot is already holding an object"
        
        elif action_type == "navigate_to":
            obj_name = action["params"]["object_name"]
            # Check if the destination object exists
            obj_exists = any(obj.get("name") == obj_name for obj in context.get("objects", []))
            if not obj_exists:
                return False, f"Destination '{obj_name}' does not exist in the environment"
        
        return True, "Action is valid"


# Test the implementation
def test_basic_parser():
    parser = BasicCommandParser()
    
    # Test cases
    test_commands = [
        "Go to the kitchen",
        "Pick up the red cup",
        "Move forward 2 meters",
        "Detect the blue ball"
    ]
    
    context = {
        "objects": [
            {"name": "kitchen", "position": [3.0, 4.0]},
            {"name": "red cup", "position": [1.0, 1.0]},
            {"name": "blue ball", "position": [2.0, 2.0]}
        ],
        "robot_state": {"holding": None}
    }
    
    for cmd in test_commands:
        actions = parser.parse_command(cmd, context)
        print(f"Command: '{cmd}'")
        print(f"Actions: {actions}")
        
        for action in actions:
            is_valid, reason = parser.validate_action(action, context)
            print(f"  Validation: {is_valid} - {reason}")
        print()


if __name__ == "__main__":
    test_basic_parser()
```

## Exercise 2: Prompt Engineering for Complex Commands

### Problem Statement
Develop a system that uses prompt engineering to handle more complex commands like "Go to the kitchen, pick up the red cup, and bring it to the dining table". The system should generate a sequence of actions that accomplishes the complete task.

### Requirements
1. Use LLM to interpret complex, multi-step commands
2. Generate appropriate action sequences for complex tasks
3. Consider environmental context when planning
4. Include reasoning explanations for each action

### Starter Code
```python
class ComplexCommandPlanner:
    def __init__(self, api_key=None):
        # Initialize your LLM interface here
        pass
    
    def plan_complex_command(self, command, context):
        """
        Plan actions for complex, multi-step commands using LLM
        """
        # TODO: Implement complex command planning
        pass
    
    def construct_prompt(self, command, context):
        """
        Construct an appropriate prompt for the LLM
        """
        # TODO: Implement prompt construction
        pass
```

### Solution
```python
from openai import OpenAI
import os
import json


class ComplexCommandPlanner:
    def __init__(self, api_key=None):
        if api_key:
            self.client = OpenAI(api_key=api_key)
        else:
            self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
        self.available_actions = [
            "navigate_to(object_name)",
            "grasp(object_name)", 
            "place_at(x, y, z)",
            "move_to(x, y)",
            "detect_object(object_name)",
            "pick_and_place(object_name, target_name)"
        ]
    
    def plan_complex_command(self, command, context):
        """
        Plan actions for complex, multi-step commands using LLM
        """
        # Construct the prompt
        prompt = self.construct_prompt(command, context)
        
        try:
            # Call the LLM
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system", 
                        "content": "You are a robot action planner. Given a complex command and environmental context, generate a sequence of actions for the robot. Respond with only valid JSON."
                    },
                    {"role": "user", "content": prompt}
                ],
                max_tokens=800,
                temperature=0.3
            )
            
            # Parse the response
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
            
            return result['actions'], result.get('reasoning', '')
        
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON response: {e}")
            return [], ""
        except Exception as e:
            print(f"Error in complex command planning: {e}")
            return [], ""
    
    def construct_prompt(self, command, context):
        """
        Construct an appropriate prompt for the LLM
        """
        prompt = f"""
        The user wants the robot to: "{command}"
        
        Current environmental context:
        {json.dumps(context, indent=2)}
        
        Available robot actions:
        - navigate_to(object_name): Navigate to a named object
        - grasp(object_name): Grasp an object by name
        - place_at(x, y, z): Place held object at coordinates
        - move_to(x, y): Move to coordinates
        - detect_object(object_name): Detect a specific object
        - pick_and_place(object_name, target_name): Pick up and place at target
        
        Generate a sequence of actions for the robot to complete the user's request.
        Consider the environmental context and any constraints.
        
        Also provide brief reasoning for the plan.
        
        Respond with only valid JSON in this format:
        {{
            "actions": [
                {{
                    "type": "navigate_to",
                    "params": {{"object_name": "kitchen"}},
                    "description": "Navigate to kitchen",
                    "reasoning_explanation": "First step is to go to the kitchen to find the red cup"
                }}
            ],
            "reasoning": "The plan involves navigating to the kitchen, picking up the red cup, and bringing it to the dining table. Each step is necessary to complete the overall task."
        }}
        """
        
        return prompt


# Test the implementation
def test_complex_planner():
    # Example context
    context = {
        "robot_state": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "holding": None
        },
        "objects": [
            {"name": "kitchen", "position": [3.0, 4.0, 0.0]},
            {"name": "dining table", "position": [5.0, 5.0, 0.0]},
            {"name": "red cup", "position": [3.2, 4.1, 0.8]}
        ],
        "map": "home_environment"
    }
    
    # Create planner
    planner = ComplexCommandPlanner()
    
    # Test command
    command = "Go to the kitchen, pick up the red cup, and bring it to the dining table"
    
    try:
        actions, reasoning = planner.plan_complex_command(command, context)
        
        print(f"Command: {command}")
        print(f"Generated {len(actions)} actions:")
        
        for i, action in enumerate(actions, 1):
            print(f"  {i}. {action['type']}: {action.get('params', {})}")
            print(f"     Reasoning: {action.get('reasoning_explanation', 'N/A')}")
        
        print(f"\nOverall reasoning: {reasoning}")
    
    except Exception as e:
        print(f"Error in complex command planning: {e}")
        print("Make sure your OpenAI API key is set in the environment variable OPENAI_API_KEY")


if __name__ == "__main__":
    test_complex_planner()
```

## Exercise 3: Safety Validation Engine

### Problem Statement
Implement a safety validation engine that checks action sequences for potential safety issues before execution. The system should validate each action against environmental constraints, robot capabilities, and safety guidelines.

### Requirements
1. Validate action sequences for safety before execution
2. Check for environmental hazards
3. Verify robot capabilities and constraints
4. Implement multiple safety validation layers

### Starter Code
```python
class SafetyValidator:
    def __init__(self, robot_specs, environment_map):
        """
        Initialize with robot specifications and environment information
        """
        self.robot_specs = robot_specs
        self.environment_map = environment_map
    
    def validate_action_sequence(self, action_sequence):
        """
        Validate a sequence of actions for safety
        """
        # TODO: Implement sequence validation
        pass
    
    def validate_single_action(self, action, current_state):
        """
        Validate a single action in the context of current robot state
        """
        # TODO: Implement single action validation
        pass
```

### Solution
```python
import math
from typing import Dict, List, Tuple, Optional


class SafetyValidator:
    def __init__(self, robot_specs: Dict, environment_map: Dict):
        """
        Initialize with robot specifications and environment information
        """
        self.robot_specs = robot_specs  # Contains max_speed, max_payload, etc.
        self.environment_map = environment_map  # Contains obstacles, safe zones, etc.
    
    def validate_action_sequence(self, action_sequence: List[Dict]) -> Tuple[bool, str]:
        """
        Validate a sequence of actions for safety
        """
        for i, action in enumerate(action_sequence):
            is_valid, reason = self.validate_single_action(action, i, action_sequence)
            if not is_valid:
                return False, f"Action {i+1} failed validation: {reason}"
        
        return True, "Action sequence is safe"
    
    def validate_single_action(self, action: Dict, index: int, sequence: List[Dict]) -> Tuple[bool, str]:
        """
        Validate a single action in the context of current robot state
        """
        action_type = action.get('type')
        
        if action_type == 'move_to':
            return self._validate_move_to(action)
        elif action_type == 'navigate_to':
            return self._validate_navigate_to(action)
        elif action_type == 'grasp':
            return self._validate_grasp(action)
        elif action_type == 'place_at':
            return self._validate_place_at(action)
        else:
            return True, "Action type not requiring safety validation"
    
    def _validate_move_to(self, action: Dict) -> Tuple[bool, str]:
        """
        Validate move_to action for safety
        """
        params = action.get('params', {})
        target_x = params.get('x', 0)
        target_y = params.get('y', 0)
        
        # Check if destination is in safe area
        if not self._is_safe_coordinate(target_x, target_y):
            return False, f"Destination ({target_x}, {target_y}) is not in a safe area"
        
        # Check if path is clear of obstacles
        if self._has_obstacles_on_path((0, 0), (target_x, target_y)):  # Assuming starting from (0,0)
            return False, f"Path to ({target_x}, {target_y}) has obstacles"
        
        # Check if destination is within operational range
        max_range = self.robot_specs.get('max_operational_range', 10.0)
        distance = math.sqrt(target_x**2 + target_y**2)
        if distance > max_range:
            return False, f"Destination ({target_x}, {target_y}) is out of operational range ({max_range}m)"
        
        return True, "Move action is safe"
    
    def _validate_navigate_to(self, action: Dict) -> Tuple[bool, str]:
        """
        Validate navigate_to action for safety
        """
        params = action.get('params', {})
        object_name = params.get('object_name')
        
        # Find object in environment
        obj_info = self._find_object_in_environment(object_name)
        if not obj_info:
            return False, f"Object '{object_name}' not found in environment"
        
        obj_pos = obj_info.get('position', {})
        target_x = obj_pos.get('x', 0)
        target_y = obj_pos.get('y', 0)
        
        # Reuse move validation logic
        return self._validate_move_to({
            'type': 'move_to',
            'params': {'x': target_x, 'y': target_y}
        })
    
    def _validate_grasp(self, action: Dict) -> Tuple[bool, str]:
        """
        Validate grasp action for safety
        """
        params = action.get('params', {})
        object_name = params.get('object_name')
        
        # Check if object exists in environment
        obj_info = self._find_object_in_environment(object_name)
        if not obj_info:
            return False, f"Object '{object_name}' not found in environment"
        
        # Check if object properties are safe to grasp
        obj_properties = obj_info.get('properties', {})
        
        # Check temperature
        temperature = obj_properties.get('temperature', 'normal')
        if temperature == 'hot':
            return False, f"Cannot grasp object '{object_name}' - it's too hot"
        
        # Check weight
        weight = obj_properties.get('weight', 0.0)
        max_payload = self.robot_specs.get('max_payload', 1.0)
        if weight > max_payload:
            return False, f"Cannot grasp object '{object_name}' - it's too heavy ({weight}kg > {max_payload}kg)"
        
        # Check if robot is already holding something
        currently_holding = self.robot_specs.get('current_holding', None)
        if currently_holding:
            return False, f"Cannot grasp '{object_name}' - robot is already holding '{currently_holding}'"
        
        return True, "Grasp action is safe"
    
    def _validate_place_at(self, action: Dict) -> Tuple[bool, str]:
        """
        Validate place_at action for safety
        """
        params = action.get('params', {})
        x = params.get('x', 0)
        y = params.get('y', 0)
        z = params.get('z', 0)  # Height
        
        # Check if robot is holding an object
        currently_holding = self.robot_specs.get('current_holding', None)
        if not currently_holding:
            return False, "Cannot place object - robot is not holding anything"
        
        # Check if placement location is safe
        if not self._is_safe_coordinate(x, y):
            return False, f"Placement location ({x}, {y}) is not safe"
        
        # Check if placement height is appropriate
        max_place_height = self.robot_specs.get('max_place_height', 1.5)
        if z > max_place_height:
            return False, f"Placement height {z}m exceeds maximum {max_place_height}m"
        
        return True, "Place action is safe"
    
    def _is_safe_coordinate(self, x: float, y: float) -> bool:
        """
        Check if a coordinate is in a safe area
        """
        # Check against safe zones
        safe_zones = self.environment_map.get('safe_zones', [])
        for zone in safe_zones:
            center_x, center_y = zone['center']['x'], zone['center']['y']
            radius = zone['radius']
            distance = math.sqrt((x - center_x)**2 + (y - center_y)**2)
            if distance <= radius:
                return True
        
        # Check against restricted zones
        restricted_zones = self.environment_map.get('restricted_zones', [])
        for zone in restricted_zones:
            center_x, center_y = zone['center']['x'], zone['center']['y']
            radius = zone['radius']
            distance = math.sqrt((x - center_x)**2 + (y - center_y)**2)
            if distance <= radius:
                return False  # In restricted zone
        
        # If not in any safe zone and not in restricted zone, default to unsafe
        return False
    
    def _has_obstacles_on_path(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """
        Check if there are obstacles on the path from start to end
        """
        # This is a simplified implementation
        # In a real system, this would use path planning algorithms
        obstacles = self.environment_map.get('obstacles', [])
        
        for obstacle in obstacles:
            obs_x, obs_y = obstacle['position']['x'], obstacle['position']['y']
            obs_radius = obstacle.get('radius', 0.1)
            
            # Simple check: if obstacle is within the bounding box of the path
            min_x, max_x = min(start[0], end[0]), max(start[0], end[0])
            min_y, max_y = min(start[1], end[1]), max(start[1], end[1])
            
            if (min_x - obs_radius <= obs_x <= max_x + obs_radius and 
                min_y - obs_radius <= obs_y <= max_y + obs_radius):
                # More precise check: distance from point to line segment
                if self._distance_to_segment((obs_x, obs_y), start, end) <= obs_radius:
                    return True
        
        return False
    
    def _distance_to_segment(self, point: Tuple[float, float], 
                             seg_start: Tuple[float, float], 
                             seg_end: Tuple[float, float]) -> float:
        """
        Calculate distance from a point to a line segment
        """
        x, y = point
        x1, y1 = seg_start
        x2, y2 = seg_end
        
        # Vector from start to end of segment
        seg_vec_x, seg_vec_y = x2 - x1, y2 - y1
        seg_len_sq = seg_vec_x**2 + seg_vec_y**2
        
        if seg_len_sq == 0:
            # Segment is a point
            return math.sqrt((x - x1)**2 + (y - y1)**2)
        
        # Parameter for projection of point onto line
        t = max(0, min(1, ((x - x1) * seg_vec_x + (y - y1) * seg_vec_y) / seg_len_sq))
        
        # Closest point on segment
        proj_x = x1 + t * seg_vec_x
        proj_y = y1 + t * seg_vec_y
        
        return math.sqrt((x - proj_x)**2 + (y - proj_y)**2)
    
    def _find_object_in_environment(self, object_name: str) -> Optional[Dict]:
        """
        Find an object in the environment by name
        """
        objects = self.environment_map.get('objects', [])
        for obj in objects:
            if obj.get('name') == object_name:
                return obj
        return None


# Test the safety validator
def test_safety_validator():
    # Define robot specifications
    robot_specs = {
        'max_payload': 1.0,  # 1kg max
        'max_operational_range': 10.0,  # 10m max
        'max_place_height': 1.5,  # 1.5m max
        'current_holding': None  # Nothing held initially
    }
    
    # Define environment map
    environment_map = {
        'safe_zones': [
            {'center': {'x': 0, 'y': 0}, 'radius': 10.0},  # Everywhere is safe in this example
        ],
        'restricted_zones': [
            {'center': {'x': 5, 'y': 5}, 'radius': 0.5},  # Small restricted area
        ],
        'obstacles': [
            {'position': {'x': 2.0, 'y': 2.0}, 'radius': 0.3},
            {'position': {'x': 3.5, 'y': 1.5}, 'radius': 0.2}
        ],
        'objects': [
            {'name': 'red cup', 'position': {'x': 1.0, 'y': 1.0, 'z': 0.8}, 
             'properties': {'weight': 0.2, 'temperature': 'normal'}},
            {'name': 'heavy box', 'position': {'x': 2.0, 'y': 0.5, 'z': 0.5}, 
             'properties': {'weight': 2.0, 'temperature': 'normal'}},
            {'name': 'hot mug', 'position': {'x': 0.5, 'y': 2.0, 'z': 0.8}, 
             'properties': {'weight': 0.3, 'temperature': 'hot'}}
        ]
    }
    
    # Create validator
    validator = SafetyValidator(robot_specs, environment_map)
    
    # Test sequences
    test_sequences = [
        [
            {'type': 'navigate_to', 'params': {'object_name': 'red cup'}},
            {'type': 'grasp', 'params': {'object_name': 'red cup'}},
            {'type': 'move_to', 'params': {'x': 1.5, 'y': 1.5}}
        ],
        [
            {'type': 'navigate_to', 'params': {'object_name': 'heavy box'}},  # Should fail - too heavy
            {'type': 'grasp', 'params': {'object_name': 'heavy box'}}
        ],
        [
            {'type': 'navigate_to', 'params': {'object_name': 'hot mug'}},  # Should fail - too hot
            {'type': 'grasp', 'params': {'object_name': 'hot mug'}}
        ],
        [
            {'type': 'move_to', 'params': {'x': 12.0, 'y': 0.0}}  # Should fail - out of range
        ]
    ]
    
    for i, sequence in enumerate(test_sequences):
        is_safe, reason = validator.validate_action_sequence(sequence)
        print(f"Test sequence {i+1}: {'SAFE' if is_safe else 'UNSAFE'}")
        print(f"  Reason: {reason}")
        print(f"  Actions: {[action['type'] for action in sequence]}")
        print()


if __name__ == "__main__":
    test_safety_validator()
```

## Exercise 4: Context-Aware Planning

### Problem Statement
Build a cognitive planner that maintains and updates environmental context during planning and execution. The system should handle dynamic environments where object positions and robot state change.

### Requirements
1. Maintain an up-to-date environmental context
2. Update context based on action effects
3. Plan considering the latest context
4. Handle changes in the environment during execution

### Starter Code
```python
class ContextAwarePlanner:
    def __init__(self):
        self.current_context = {
            "robot_position": {"x": 0, "y": 0, "z": 0},
            "robot_state": {"holding": None, "battery": 1.0},
            "objects": {},
            "timestamp": None
        }
    
    def update_context_from_perception(self, perception_data):
        """
        Update the current context based on perception data
        """
        # TODO: Implement context update logic
        pass
    
    def update_context_after_action(self, action):
        """
        Update the context to reflect the effects of an action
        """
        # TODO: Implement context update after action
        pass
    
    def plan_with_context(self, command):
        """
        Plan actions considering the current context
        """
        # TODO: Implement context-aware planning
        pass
```

### Solution
```python
import time
from datetime import datetime
from copy import deepcopy


class ContextAwarePlanner:
    def __init__(self):
        self.current_context = {
            "robot_position": {"x": 0, "y": 0, "z": 0},
            "robot_state": {"holding": None, "battery": 1.0},
            "objects": {},
            "timestamp": time.time(),
            "environments": {}  # To store multiple environment states
        }
    
    def update_context_from_perception(self, perception_data):
        """
        Update the current context based on perception data
        """
        # Update robot position if available
        if 'robot_position' in perception_data:
            self.current_context['robot_position'] = perception_data['robot_position']
        
        # Update detected objects
        if 'detected_objects' in perception_data:
            for obj_data in perception_data['detected_objects']:
                obj_name = obj_data.get('name')
                self.current_context['objects'][obj_name] = obj_data
        
        # Update timestamp
        self.current_context['timestamp'] = time.time()
        
        print(f"Context updated at {datetime.fromtimestamp(self.current_context['timestamp'])}")
    
    def update_context_after_action(self, action):
        """
        Update the context to reflect the effects of an action
        """
        action_type = action.get('type')
        params = action.get('params', {})
        
        if action_type == 'grasp':
            obj_name = params.get('object_name')
            if obj_name in self.current_context['objects']:
                # Update robot state to holding the object
                self.current_context['robot_state']['holding'] = obj_name
                
                # Update object position to robot's hand
                robot_pos = self.current_context['robot_position']
                self.current_context['objects'][obj_name]['position'] = {
                    'x': robot_pos['x'],
                    'y': robot_pos['y'],
                    'z': robot_pos['z'] + 0.5  # Hand height
                }
        
        elif action_type == 'place_at':
            holding_obj = self.current_context['robot_state']['holding']
            if holding_obj:
                # Update object position to placement location
                self.current_context['objects'][holding_obj]['position'] = {
                    'x': params.get('x', 0),
                    'y': params.get('y', 0),
                    'z': params.get('z', 0)
                }
                
                # Update robot state to not holding anything
                self.current_context['robot_state']['holding'] = None
        
        elif action_type == 'move_to':
            # Update robot position
            self.current_context['robot_position']['x'] = params.get('x', 0)
            self.current_context['robot_position']['y'] = params.get('y', 0)
            # z typically stays the same for movement on ground plane
        
        elif action_type == 'navigate_to':
            # If navigating to an object, move robot to object position
            obj_name = params.get('object_name')
            if obj_name in self.current_context['objects']:
                obj_pos = self.current_context['objects'][obj_name]['position']
                self.current_context['robot_position'] = obj_pos
        
        # Update timestamp
        self.current_context['timestamp'] = time.time()
    
    def plan_with_context(self, command):
        """
        Plan actions considering the current context
        """
        # For this example, we'll use a simple rule-based planner
        # In practice, this would interface with an LLM or more sophisticated planner
        actions = []
        
        if "pick up" in command.lower() or "grasp" in command.lower():
            # Extract object name
            import re
            match = re.search(r'(?:pick up|grasp) (?:the )?(.+?)(?: and|\s*$)', command.lower())
            if match:
                obj_name = match.group(1).strip()
                
                # Check if object exists in context
                if obj_name in self.current_context['objects']:
                    # Add navigation and grasping actions
                    actions.append({
                        "type": "navigate_to",
                        "params": {"object_name": obj_name},
                        "description": f"Navigate to {obj_name}"
                    })
                    
                    actions.append({
                        "type": "grasp",
                        "params": {"object_name": obj_name},
                        "description": f"Grasp {obj_name}"
                    })
                else:
                    print(f"Object '{obj_name}' not found in current context")
        
        elif "move to" in command.lower() or "go to" in command.lower():
            # Extract destination
            match = re.search(r'(?:move to|go to) (.+)', command.lower())
            if match:
                dest = match.group(1).strip()
                
                # Check if destination is a known object
                if dest in self.current_context['objects']:
                    actions.append({
                        "type": "navigate_to",
                        "params": {"object_name": dest},
                        "description": f"Navigate to {dest}"
                    })
                else:
                    print(f"Destination '{dest}' not found in current context")
        
        elif "place" in command.lower():
            # Place currently held object
            holding = self.current_context['robot_state']['holding']
            if holding:
                # Extract placement location if specified
                import re
                match = re.search(r'place .+? (?:on|at|to) (.+)', command.lower())
                if match:
                    dest = match.group(1).strip()
                    if dest in self.current_context['objects']:
                        # Navigate to destination then place
                        actions.append({
                            "type": "navigate_to",
                            "params": {"object_name": dest},
                            "description": f"Navigate to {dest} for placement"
                        })
                        
                        obj_pos = self.current_context['objects'][dest]['position']
                        actions.append({
                            "type": "place_at",
                            "params": obj_pos,
                            "description": f"Place {holding} at {dest}'s location"
                        })
        
        # Update context timestamp
        self.current_context['timestamp'] = time.time()
        
        return actions
    
    def get_context_summary(self):
        """
        Get a summary of the current context
        """
        holding = self.current_context['robot_state']['holding']
        pos = self.current_context['robot_position']
        obj_count = len(self.current_context['objects'])
        
        summary = f"""
        Context Summary:
        - Robot Position: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})
        - Holding: {holding or 'Nothing'}
        - Battery: {self.current_context['robot_state']['battery']:.2f}
        - Known Objects: {obj_count}
        - Last Update: {datetime.fromtimestamp(self.current_context['timestamp'])}
        """
        
        return summary


# Test the context-aware planner
def test_context_aware_planner():
    planner = ContextAwarePlanner()
    
    # Initial context
    print("Initial Context:")
    print(planner.get_context_summary())
    
    # Simulate perception update with some objects
    perception_update = {
        "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "detected_objects": [
            {"name": "red cup", "position": {"x": 1.0, "y": 1.0, "z": 0.8}, "properties": {"graspable": True}},
            {"name": "table", "position": {"x": 2.0, "y": 1.5, "z": 0.0}, "properties": {"surface": True}},
            {"name": "box", "position": {"x": 0.5, "y": 2.0, "z": 0.5}, "properties": {"graspable": True}}
        ]
    }
    
    planner.update_context_from_perception(perception_update)
    print("\nAfter Perception Update:")
    print(planner.get_context_summary())
    
    # Plan an action
    command = "Pick up the red cup"
    actions = planner.plan_with_context(command)
    
    print(f"\nCommand: '{command}'")
    print(f"Planned Actions: {len(actions)}")
    for i, action in enumerate(actions, 1):
        print(f"  {i}. {action['description']}")
    
    # Execute the first action (navigation) and update context
    if actions:
        planner.update_context_after_action(actions[0])  # Navigate to red cup
        print(f"\nAfter executing: {actions[0]['description']}")
        print(planner.get_context_summary())
        
        # Execute the second action (grasping) and update context
        if len(actions) > 1:
            planner.update_context_after_action(actions[1])  # Grasp red cup
            print(f"\nAfter executing: {actions[1]['description']}")
            print(planner.get_context_summary())
            
            # Now try placing the object
            place_command = "Place the object on the table"
            place_actions = planner.plan_with_context(place_command)
            
            print(f"\nCommand: '{place_command}'")
            print(f"Planned Actions: {len(place_actions)}")
            for i, action in enumerate(place_actions, 1):
                print(f"  {i}. {action['description']}")


if __name__ == "__main__":
    test_context_aware_planner()
```

## Exercise 5: Error Recovery System

### Problem Statement
Develop an error recovery system that handles failures during cognitive planning and execution. The system should detect failures, attempt recovery, and provide fallback behaviors.

### Requirements
1. Detect when an action fails during execution
2. Implement recovery strategies for different failure types
3. Provide graceful fallback behaviors
4. Log failures for analysis and improvement

### Solution
```python
from enum import Enum
import time
import random


class FailureType(Enum):
    COMMUNICATION_ERROR = "communication_error"
    OBSTACLE_DETECTED = "obstacle_detected"
    OBJECT_NOT_FOUND = "object_not_found"
    GRASP_FAILED = "grasp_failed"
    NAVIGATION_FAILED = "navigation_failed"
    SAFETY_VIOLATION = "safety_violation"


class ErrorRecoverySystem:
    def __init__(self):
        self.failure_log = []
        self.recovery_strategies = {
            FailureType.OBSTACLE_DETECTED: self._handle_obstacle_detected,
            FailureType.OBJECT_NOT_FOUND: self._handle_object_not_found,
            FailureType.NAVIGATION_FAILED: self._handle_navigation_failed,
            FailureType.GRASP_FAILED: self._handle_grasp_failed,
            FailureType.COMMUNICATION_ERROR: self._handle_communication_error,
            FailureType.SAFETY_VIOLATION: self._handle_safety_violation
        }
    
    def log_failure(self, action, failure_type, details=""):
        """
        Log a failure for analysis and improvement
        """
        failure_record = {
            "timestamp": time.time(),
            "action": action,
            "failure_type": failure_type,
            "details": details,
            "recovery_attempted": False
        }
        self.failure_log.append(failure_record)
        print(f"FAILURE LOGGED: {failure_type.value} - {details}")
    
    def attempt_recovery(self, action, failure_type, context=None):
        """
        Attempt to recover from a failure
        """
        if failure_type in self.recovery_strategies:
            recovery_func = self.recovery_strategies[failure_type]
            return recovery_func(action, context)
        else:
            # No specific recovery strategy, try general recovery
            return self._general_recovery(action, context)
    
    def _handle_obstacle_detected(self, action, context):
        """
        Handle obstacle detection during navigation
        """
        print("Recovery: Attempting path replanning to avoid obstacle")
        
        # If action was navigation, try to find an alternative route
        if action.get('type') == 'navigate_to':
            object_name = action.get('params', {}).get('object_name')
            if context and object_name in context.get('objects', {}):
                # Create a new action with an adjusted path (simplified implementation)
                obj_position = context['objects'][object_name]['position']
                
                # Add some offset to try a different approach path
                new_action = {
                    "type": "move_to",
                    "params": {
                        "x": obj_position['x'] + random.uniform(-0.1, 0.1),
                        "y": obj_position['y'] + random.uniform(-0.1, 0.1)
                    },
                    "description": f"Alternative approach to {object_name}"
                }
                return [new_action]
        
        return []  # No recovery possible
    
    def _handle_object_not_found(self, action, context):
        """
        Handle case where expected object is not found
        """
        print("Recovery: Expanding search area for object")
        
        # Perform a wider search for the object
        search_actions = [
            {
                "type": "detect_object",
                "params": action.get('params', {}),
                "description": f"Searching for {action.get('params', {}).get('object_name', 'object')} in wider area"
            }
        ]
        
        return search_actions
    
    def _handle_navigation_failed(self, action, context):
        """
        Handle navigation failure
        """
        print("Recovery: Attempting alternative navigation approach")
        
        # Try to get closer in steps if a direct navigation failed
        if action.get('type') == 'navigate_to' and context:
            obj_name = action.get('params', {}).get('object_name')
            if obj_name in context.get('objects', {}):
                obj_pos = context['objects'][obj_name]['position']
                current_pos = context.get('robot_position', {'x': 0, 'y': 0})
                
                # Calculate intermediate point
                mid_x = (current_pos['x'] + obj_pos['x']) / 2
                mid_y = (current_pos['y'] + obj_pos['y']) / 2
                
                recovery_actions = [
                    {
                        "type": "move_to",
                        "params": {"x": mid_x, "y": mid_y},
                        "description": "Move to intermediate point"
                    },
                    {
                        "type": "navigate_to",
                        "params": action.get('params', {}),
                        "description": f"Continue to {obj_name} from intermediate point"
                    }
                ]
                
                return recovery_actions
        
        return []
    
    def _handle_grasp_failed(self, action, context):
        """
        Handle grasp failure
        """
        print("Recovery: Adjusting approach for grasp attempt")
        
        # For grasp failure, we might need to reposition and try again
        obj_name = action.get('params', {}).get('object_name')
        if context and obj_name in context.get('objects', {}):
            obj_pos = context['objects'][obj_name]['position']
            
            # Approach from a slightly different angle
            recovery_actions = [
                {
                    "type": "move_to",
                    "params": {
                        "x": obj_pos['x'] + random.uniform(-0.2, 0.2),
                        "y": obj_pos['y'] + random.uniform(-0.2, 0.2)
                    },
                    "description": f"Reposition for grasp on {obj_name}"
                },
                {
                    "type": "grasp",
                    "params": action.get('params', {}),
                    "description": f"Attempt grasp on {obj_name} again"
                }
            ]
            
            return recovery_actions
        
        return []
    
    def _handle_communication_error(self, action, context):
        """
        Handle communication error with robot
        """
        print("Recovery: Retrying communication")
        
        # Simply retry the same action
        return [action]
    
    def _handle_safety_violation(self, action, context):
        """
        Handle safety constraint violation
        """
        print("Recovery: Safety check failed, requesting human intervention or safe stop")
        
        # Return a safe stop action
        safe_actions = [
            {
                "type": "move_to",
                "params": {"x": 0, "y": 0},
                "description": "Return to safe home position"
            }
        ]
        
        return safe_actions
    
    def _general_recovery(self, action, context):
        """
        General recovery for unspecified failure types
        """
        print("Recovery: General recovery strategy - returning to safe state")
        
        # Return to a safe state
        return [
            {
                "type": "move_to",
                "params": {"x": 0, "y": 0},
                "description": "Return to safe home position"
            }
        ]
    
    def get_failure_statistics(self):
        """
        Get statistics about failures for analysis
        """
        if not self.failure_log:
            return "No failures logged"
        
        failure_counts = {}
        for record in self.failure_log:
            ft = record['failure_type'].value
            failure_counts[ft] = failure_counts.get(ft, 0) + 1
        
        stats = "FAILURE STATISTICS:\n"
        for ft, count in failure_counts.items():
            stats += f"  {ft}: {count} occurrences\n"
        
        return stats


# Test the error recovery system
def test_error_recovery():
    recovery_system = ErrorRecoverySystem()
    
    # Simulate various failures and their recovery
    test_cases = [
        {
            "action": {"type": "navigate_to", "params": {"object_name": "red cup"}},
            "failure": FailureType.OBSTACLE_DETECTED,
            "context": {
                "objects": {"red cup": {"position": {"x": 1.0, "y": 1.0}}},
                "robot_position": {"x": 0.0, "y": 0.0}
            }
        },
        {
            "action": {"type": "grasp", "params": {"object_name": "blue mug"}},
            "failure": FailureType.GRASP_FAILED,
            "context": {
                "objects": {"blue mug": {"position": {"x": 1.5, "y": 1.0}}}
            }
        },
        {
            "action": {"type": "navigate_to", "params": {"object_name": "box"}},
            "failure": FailureType.OBJECT_NOT_FOUND,
            "context": {}
        }
    ]
    
    for i, test_case in enumerate(test_cases):
        print(f"\nTest Case {i+1}: {test_case['failure'].value}")
        print(f"Action: {test_case['action']}")
        
        # Log the failure
        recovery_system.log_failure(
            test_case['action'], 
            test_case['failure'], 
            f"Simulated {test_case['failure'].value}"
        )
        
        # Attempt recovery
        recovery_actions = recovery_system.attempt_recovery(
            test_case['action'], 
            test_case['failure'], 
            test_case.get('context')
        )
        
        print(f"Recovery actions: {recovery_actions}")
    
    # Print statistics
    print(f"\n{recovery_system.get_failure_statistics()}")


if __name__ == "__main__":
    test_error_recovery()
```

## Summary of Exercises

These exercises cover essential aspects of cognitive planning in VLA systems:

1. **Basic Command Parsing**: Understanding how to convert natural language to structured actions
2. **Complex Command Planning**: Using LLMs for sophisticated multi-step planning
3. **Safety Validation**: Ensuring action sequences are safe before execution
4. **Context Awareness**: Maintaining and updating environmental context
5. **Error Recovery**: Handling failures gracefully during execution

Each exercise builds upon the previous ones, providing a comprehensive learning path for implementing cognitive planning systems in VLA applications.
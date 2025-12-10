# Reasoning Explanation Feature for Educational VLA Systems

## Overview

The reasoning explanation feature is a critical component in educational Vision-Language-Action (VLA) systems. It provides students and developers with insights into how the cognitive planner interprets commands and makes decisions, enhancing understanding and learning outcomes.

## Purpose of Reasoning Explanation

In educational settings, it's not sufficient to simply execute commands successfully. Learners need to understand:
1. How the system interprets their commands
2. Why specific action sequences are chosen
3. How the system handles ambiguity
4. What factors influence decision-making
5. How safety considerations affect planning

## Implementation Architecture

### Extended Action Data Structure

First, we'll extend our Action data structure to include reasoning information:

```python
# extended_action.py
from dataclasses import dataclass
from typing import Dict, Any, Optional
from datetime import datetime


@dataclass
class ActionWithReasoning:
    """
    Represents an action in an action sequence with educational reasoning
    """
    type: str  # The type of action (e.g., 'move_to', 'grasp', 'navigate_to')
    params: Dict[str, Any]  # Parameters for the action
    description: Optional[str] = None  # Human-readable description
    confidence: Optional[float] = None  # Confidence score (0.0 to 1.0)
    timeout: Optional[float] = 30.0  # Execution timeout in seconds
    
    # Educational fields
    reasoning_explanation: Optional[str] = None  # Explanation of why this action was chosen
    prerequisite_action: Optional[str] = None  # Action that must be completed before this one
    environment_context: Optional[Dict[str, Any]] = None  # Context at planning time
    safety_considerations: Optional[list] = None  # Safety factors considered
    alternative_actions: Optional[list] = None  # Other considered but rejected actions
    timestamp: Optional[float] = None  # When this action was planned


@dataclass
class ReasoningStep:
    """
    Represents a single reasoning step in the planning process
    """
    step_number: int
    decision_made: str
    reasoning_process: str
    influencing_factors: list
    outcome: str
    timestamp: float = datetime.now().timestamp()
```

### Enhanced Cognitive Planner with Reasoning

Now we'll enhance our cognitive planner to include reasoning explanations:

```python
# enhanced_cognitive_planner.py
import json
from openai import OpenAI
import os
from typing import Tuple, List
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionSequence
from rclpy.qos import QoSProfile


class CognitivePlannerWithReasoning(Node):
    """
    Cognitive planner that provides reasoning explanations for educational purposes
    """
    
    def __init__(self):
        super().__init__('cognitive_planner_with_reasoning')
        
        # Initialize components
        self.llm_interface = LLMInterfaceWithReasoning()
        self.context_manager = ContextManager()
        self.validator = ActionValidator(self, self.context_manager)
        
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
        
        self.explanation_publisher = self.create_publisher(
            String,
            '/vla/reasoning_explanation',
            QoSProfile(depth=10)
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla/cognitive_planner_status',
            QoSProfile(depth=10)
        )
        
        self.get_logger().info('Cognitive Planner with Reasoning initialized')
    
    def command_callback(self, msg):
        """
        Process natural language command with reasoning explanation
        """
        try:
            command_text = msg.data
            self.get_logger().info(f'Received command: {command_text}')
            
            # Update context from perception
            self.context_manager.update_from_perception()
            context = self.context_manager.get_context_for_planning()
            
            # Plan with reasoning
            action_sequence, reasoning_explanation = self.llm_interface.plan_action_sequence_with_reasoning(
                command_text, 
                context
            )
            
            if action_sequence:
                # Validate the planned action sequence
                is_valid, validation_reason = self.validator.validate_action_sequence(action_sequence)
                
                if is_valid:
                    # Publish the validated action sequence
                    action_msg = self._create_action_sequence_msg(action_sequence, command_text)
                    self.action_publisher.publish(action_msg)
                    
                    # Publish reasoning explanation for educational purposes
                    explanation_msg = String()
                    explanation_msg.data = reasoning_explanation
                    self.explanation_publisher.publish(explanation_msg)
                    
                    self.get_logger().info(f'Published validated action sequence with {len(action_sequence)} actions')
                    self._publish_status(f'Successfully planned {len(action_sequence)} actions with reasoning')
                else:
                    self.get_logger().error(f'Action sequence validation failed: {validation_reason}')
                    self._publish_status(f'Validation failed: {validation_reason}')
            else:
                self.get_logger().error('Failed to generate action sequence with reasoning')
                self._publish_status('Failed to generate action sequence with reasoning')
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            self._publish_status(f'Error processing command: {str(e)}')
    
    def _create_action_sequence_msg(self, actions, original_command):
        """
        Create ROS message from action sequence
        """
        from vla_msgs.msg import ActionSequence
        msg = ActionSequence()
        msg.request_id = self._generate_request_id()
        msg.command = original_command
        msg.actions = [self._action_to_ros_action(action) for action in actions]
        
        return msg
    
    def _action_to_ros_action(self, action):
        """
        Convert internal action to ROS action message
        """
        # This would convert our ActionWithReasoning to the appropriate ROS message
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


class LLMInterfaceWithReasoning:
    """
    LLM interface that provides reasoning explanations for educational purposes
    """
    
    def __init__(self, api_key=None):
        if api_key:
            self.client = OpenAI(api_key=api_key)
        else:
            self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        
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
    
    def plan_action_sequence_with_reasoning(self, command, context) -> Tuple[List[ActionWithReasoning], str]:
        """
        Plan an action sequence with detailed reasoning for educational purposes
        """
        # Construct the prompt with reasoning requirements
        prompt = self._construct_reasoning_prompt(command, context)
        
        try:
            # Call the LLM
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are an educational cognitive planning system for a robot. "
                            "Given the user's command and environmental context, "
                            "generate a sequence of actions for the robot to execute. "
                            "Provide detailed reasoning for each decision. "
                            "Output only valid JSON as specified."
                        )
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                max_tokens=1500,
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
            
            # Convert to ActionWithReasoning objects
            actions = []
            for action_dict in result['actions']:
                action = ActionWithReasoning(
                    type=action_dict['type'],
                    params=action_dict.get('params', {}),
                    description=action_dict.get('description'),
                    confidence=action_dict.get('confidence'),
                    reasoning_explanation=action_dict.get('reasoning_explanation'),
                    prerequisite_action=action_dict.get('prerequisite_action'),
                    environment_context=action_dict.get('environment_context'),
                    safety_considerations=action_dict.get('safety_considerations'),
                    alternative_actions=action_dict.get('alternative_actions')
                )
                actions.append(action)
            
            # Get overall reasoning explanation
            reasoning_explanation = result.get('overall_reasoning', 'No overall reasoning provided')
            
            return actions, reasoning_explanation
        
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON response: {e}")
            return None, f"Error parsing JSON response: {e}"
        except Exception as e:
            print(f"Error in LLM planning with reasoning: {e}")
            return None, f"Error in LLM planning with reasoning: {e}"
    
    def _construct_reasoning_prompt(self, command, context):
        """
        Construct the prompt for LLM to include reasoning explanations
        """
        prompt = f"""
        The user wants the robot to: "{command}"
        
        Current environmental context:
        {json.dumps(context, indent=2)}
        
        Available robot actions:
        {json.dumps(self.available_actions, indent=2)}
        
        Generate a sequence of actions for the robot to complete the user's request.
        Consider the environmental context and the robot's current state.
        
        For each action, provide:
        1. The action type and parameters
        2. A description of the action
        3. Reasoning explanation: Why this action is necessary and how it contributes to the goal
        4. Prerequisite action: What action must be completed before this one (if any)
        5. Environment context: Key environmental factors considered when selecting this action
        6. Safety considerations: Safety factors evaluated for this action
        7. Alternative actions: Other actions that were considered but not selected
        
        Provide an overall reasoning section explaining how the entire plan achieves the goal.
        
        Respond with only valid JSON in this format:
        {{
            "actions": [
                {{
                    "type": "navigate_to",
                    "params": {{"object_name": "red cup"}},
                    "description": "Navigate to the red cup",
                    "confidence": 0.9,
                    "reasoning_explanation": "The first step is to navigate to the target object. Since the command is to pick up the red cup, navigation is required before grasping.",
                    "prerequisite_action": null,
                    "environment_context": {{"red_cup_position": [1.2, 0.5], "robot_position": [0.0, 0.0]}},
                    "safety_considerations": ["Check path for obstacles"],
                    "alternative_actions": ["detect_object(red cup) as an alternative if location uncertain"]
                }},
                {{
                    "type": "grasp",
                    "params": {{"object_name": "red cup"}},
                    "description": "Grasp the red cup",
                    "confidence": 0.95,
                    "reasoning_explanation": "After navigating to the red cup, the next step is to grasp it to complete the retrieval task.",
                    "prerequisite_action": "navigate_to",
                    "environment_context": {{"red_cup_position": [1.2, 0.5], "robot_gripper_state": "open"}},
                    "safety_considerations": ["Verify object properties are graspable"],
                    "alternative_actions": []
                }}
            ],
            "overall_reasoning": "The plan involves navigating to the red cup and grasping it, which directly addresses the user's request to pick up the red cup."
        }}
        """
        
        return prompt
```

### Educational Visualization Component

We'll also create a component to visualize and present the reasoning to learners:

```python
# reasoning_visualizer.py
from typing import List
import json


class ReasoningVisualizer:
    """
    Component to visualize and present reasoning explanations to learners
    """
    
    def __init__(self):
        pass
    
    def generate_educational_explanation(self, action_sequence: List[ActionWithReasoning], command: str) -> str:
        """
        Generate an educational explanation of the action sequence
        """
        explanation = f"EDUCATIONAL BREAKDOWN OF TASK: '{command}'\n"
        explanation += "=" * 50 + "\n\n"
        
        for i, action in enumerate(action_sequence):
            explanation += f"STEP {i+1}: {action.description}\n"
            explanation += f"  Action Type: {action.type}\n"
            explanation += f"  Parameters: {action.params}\n"
            
            if action.reasoning_explanation:
                explanation += f"  Reasoning: {action.reasoning_explanation}\n"
            
            if action.prerequisite_action:
                explanation += f"  Prerequisite: {action.prerequisite_action}\n"
            
            if action.environment_context:
                explanation += f"  Context: {json.dumps(action.environment_context, indent=4)}\n"
            
            if action.safety_considerations:
                explanation += f"  Safety Factors: {', '.join(action.safety_considerations)}\n"
            
            if action.alternative_actions:
                explanation += f"  Alternatives Considered: {', '.join(action.alternative_actions)}\n"
            
            explanation += "\n"
        
        return explanation
    
    def generate_decision_tree(self, action_sequence: List[ActionWithReasoning]) -> str:
        """
        Generate a decision tree representation of the planning process
        """
        tree = "PLANNING DECISION TREE\n"
        tree += "=" * 30 + "\n"
        
        # Create a simplified representation of decision points
        for i, action in enumerate(action_sequence):
            indent = "  " * i
            tree += f"{indent}├─ Step {i+1}: {action.type} -> {action.description}\n"
            if action.reasoning_explanation:
                tree += f"{indent}   └─ Reason: {action.reasoning_explanation[:100]}...\n"
        
        return tree
    
    def generate_educational_summary(self, action_sequence: List[ActionWithReasoning], 
                                   command: str, overall_reasoning: str) -> dict:
        """
        Generate a comprehensive educational summary
        """
        summary = {
            "command": command,
            "total_steps": len(action_sequence),
            "action_types_used": list(set([action.type for action in action_sequence])),
            "safety_considerations": [],
            "key_learning_points": [],
            "overall_reasoning": overall_reasoning,
            "step_by_step_explanation": []
        }
        
        for action in action_sequence:
            step_explanation = {
                "step": action.description,
                "action_type": action.type,
                "reasoning": action.reasoning_explanation,
                "safety_factors": action.safety_considerations or []
            }
            summary["step_by_step_explanation"].append(step_explanation)
            
            if action.safety_considerations:
                summary["safety_considerations"].extend(action.safety_considerations)
        
        # Add key learning points
        if any('grasp' in action.type for action in action_sequence):
            summary["key_learning_points"].append("Object manipulation requires navigation before grasping")
        
        if any('navigate_to' in action.type for action in action_sequence):
            summary["key_learning_points"].append("Path planning and obstacle avoidance are critical for navigation")
        
        if any(action.safety_considerations for action in action_sequence):
            summary["key_learning_points"].append("Safety validation is essential at each step")
        
        return summary


# Example usage of reasoning explanation system
def example_with_reasoning():
    """
    Example demonstrating the reasoning explanation feature
    """
    # Initialize components 
    llm_interface = LLMInterfaceWithReasoning()
    context_manager = ContextManager()
    
    # Create planner with reasoning
    planner = CognitivePlannerWithReasoning()
    
    # Example command
    command = "Pick up the red cup on the table and place it in the kitchen"
    
    # Get context
    context = context_manager.get_context_for_planning()
    
    # Plan with reasoning
    action_sequence, overall_reasoning = llm_interface.plan_action_sequence_with_reasoning(
        command, 
        context
    )
    
    if action_sequence:
        # Create visualizer
        visualizer = ReasoningVisualizer()
        
        # Generate educational explanation
        explanation = visualizer.generate_educational_explanation(action_sequence, command)
        print(explanation)
        
        # Generate decision tree
        decision_tree = visualizer.generate_decision_tree(action_sequence)
        print(decision_tree)
        
        # Generate educational summary
        summary = visualizer.generate_educational_summary(action_sequence, command, overall_reasoning)
        print("\nEDUCATIONAL SUMMARY:")
        print(json.dumps(summary, indent=2))
    else:
        print("Failed to generate plan with reasoning")


# Implementation for ROS node
def reasoning_explanation_node():
    """
    Example of how to implement reasoning explanation in a ROS node
    """
    import rclpy
    
    rclpy.init()
    
    # Create the cognitive planner with reasoning
    planner_node = CognitivePlannerWithReasoning()
    
    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        planner_node.get_logger().info('Reasoning Explanation Node stopped by user')
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()
```

## Educational Use Cases

### Use Case 1: Classroom Learning
In a classroom setting, the reasoning explanation helps students understand:
- How natural language commands are interpreted
- Why certain action sequences are chosen
- How the system reasons about environment context
- What safety considerations are important

### Use Case 2: Developer Training
For developers learning VLA systems, the explanations provide:
- Insight into the planning process
- Understanding of context integration
- Examples of proper action sequencing
- Safety validation procedures

### Use Case 3: Research and Debugging
For researchers and engineers, the explanations help:
- Debug complex planning failures
- Understand decision-making heuristics
- Validate system behavior
- Refine prompt engineering

## Implementation Tips for Educational Settings

1. **Interactive Exploration**: Allow students to modify commands and observe how the reasoning changes
2. **Step-by-Step Execution**: Enable pausing at each step to review reasoning
3. **Comparison Tool**: Show how different commands lead to different reasoning paths
4. **Visualization**: Create diagrams showing the decision-making process
5. **Feedback Mechanism**: Allow students to provide feedback on explanations to improve the system

## Conclusion

The reasoning explanation feature transforms a VLA system from a "black box" into an educational tool that promotes understanding. By providing clear explanations of why specific actions are chosen, how context influences decisions, and what safety considerations are evaluated, students and developers gain valuable insights into cognitive planning systems.
#!/usr/bin/env python3
"""
Cognitive Planner Node for VLA Systems

This node uses Large Language Models (LLMs) to interpret natural language commands
and generate executable action sequences for robots in the VLA system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI  # Using OpenAI API for this example
import json
import os
from rclpy.qos import QoSProfile
from vla_msgs.msg import PlanCognitiveTask, ActionSequence


class CognitivePlannerNode(Node):
    """
    Cognitive Planner Node that translates natural language commands into action sequences
    using Large Language Models (LLMs) for reasoning and planning.
    """
    
    def __init__(self):
        super().__init__('cognitive_planner_node')
        
        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().warn('OPENAI_API_KEY environment variable not set')
        self.client = OpenAI(api_key=api_key)
        
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
        
        # Publisher for status updates
        self.status_publisher = self.create_publisher(
            String,
            '/vla/cognitive_planner_status',
            QoSProfile(depth=10)
        )
        
        self.get_logger().info('Cognitive Planner Node initialized')

    def command_callback(self, msg):
        """
        Process natural language command and generate action sequence
        
        Args:
            msg (String): Natural language command message
        """
        try:
            command_text = msg.data
            self.get_logger().info(f'Received command: {command_text}')
            
            # Publish status update
            status_msg = String()
            status_msg.data = f'Processing command: {command_text}'
            self.status_publisher.publish(status_msg)
            
            # Query LLM for action planning
            action_sequence = self.plan_actions(command_text)
            
            if action_sequence:
                # Validate the action sequence before publishing
                if self.validate_action_sequence(action_sequence):
                    # Publish the validated action sequence
                    action_msg = ActionSequence()
                    action_msg.actions = action_sequence
                    action_msg.request_id = self.generate_request_id()
                    action_msg.command = command_text
                    
                    self.action_publisher.publish(action_msg)
                    self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')
                    
                    # Publish completion status
                    status_msg.data = f'Successfully planned {len(action_sequence)} actions'
                    self.status_publisher.publish(status_msg)
                else:
                    self.get_logger().error('Action sequence failed validation')
                    status_msg.data = 'Action sequence failed validation'
                    self.status_publisher.publish(status_msg)
            else:
                self.get_logger().error('Failed to generate action sequence')
                status_msg.data = 'Failed to generate action sequence'
                self.status_publisher.publish(status_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            status_msg = String()
            status_msg.data = f'Error processing command: {str(e)}'
            self.status_publisher.publish(status_msg)

    def plan_actions(self, natural_language_command):
        """
        Use LLM to plan multi-step actions from natural language
        
        Args:
            natural_language_command (str): Natural language command to process
            
        Returns:
            list: List of action dictionaries, or None if planning failed
        """
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
                
            # Parse the JSON response
            try:
                parsed_response = json.loads(response_text)
                if 'actions' in parsed_response:
                    return parsed_response['actions']
                else:
                    self.get_logger().error(f'LLM response missing "actions" field: {response_text}')
                    return None
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Failed to parse LLM response as JSON: {response_text}')
                return None
            
        except Exception as e:
            self.get_logger().error(f'Error in LLM query: {str(e)}')
            return None

    def construct_prompt(self, natural_language_command):
        """
        Construct prompt with environmental context
        
        Args:
            natural_language_command (str): The command to process
            
        Returns:
            str: Formatted prompt for the LLM
        """
        # In a real implementation, this would come from perception system
        # For this example, we'll use a simple static environment
        environment_context = {
            "objects": [
                {"name": "red cup", "position": {"x": 1.2, "y": 0.5, "z": 0.8}, "properties": {"graspable": True}},
                {"name": "blue box", "position": {"x": 0.8, "y": 1.0, "z": 0.8}, "properties": {"graspable": True}},
                {"name": "table", "position": {"x": 1.0, "y": 0.7, "z": 0.0}, "properties": {"graspable": False}},
                {"name": "ball", "position": {"x": 1.5, "y": 1.2, "z": 0.8}, "properties": {"graspable": True}}
            ],
            "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "robot_capabilities": ["move_to", "grasp", "detect_object", "navigate_to", "place_at"],
            "map": "simple_room_with_table",
            "robot_state": {
                "holding_object": False,
                "battery_level": 0.85
            }
        }
        
        available_actions = [
            "move_to(x, y): Move robot to specified coordinates",
            "grasp(object_name): Grasp an object by name",
            "detect_object(object_name): Detect a specific object in the environment",
            "navigate_to(object_name): Navigate to an object",
            "place_at(x, y, z): Place held object at specified coordinates",
            "pick_and_place(object_name, target_name): Pick up an object and place it at a target location"
        ]
        
        prompt = f"""
        The user wants the robot to: "{natural_language_command}"
        
        Current environment:
        {json.dumps(environment_context, indent=2)}
        
        Available robot actions:
        {json.dumps(available_actions, indent=2)}
        
        Generate a sequence of actions for the robot to complete the user's request.
        Consider safety, efficiency, and the current state of the environment.
        
        Respond with only valid JSON in this format:
        {{
            "actions": [
                {{"type": "move_to", "params": {{"x": 1.0, "y": 1.0}}, "description": "Move to location"}},
                {{"type": "detect_object", "params": {{"object_name": "red cup"}}, "description": "Detect the red cup"}},
                {{"type": "grasp", "params": {{"object_name": "red cup"}}, "description": "Grasp the red cup"}}
            ],
            "reasoning": "Explanation of how the plan achieves the goal"
        }}
        
        Important guidelines:
        - Each action must be from the available actions list
        - Include a description for each action
        - The plan should be efficient and safe
        - Consider the robot's current state (e.g., if it's holding an object)
        """
        
        return prompt

    def validate_action_sequence(self, action_sequence):
        """
        Validate an action sequence for safety and logical consistency
        
        Args:
            action_sequence (list): List of action dictionaries to validate
            
        Returns:
            bool: True if all actions are valid, False otherwise
        """
        if not action_sequence or not isinstance(action_sequence, list):
            self.get_logger().error('Invalid action sequence: not a list or empty')
            return False
        
        for i, action in enumerate(action_sequence):
            if not isinstance(action, dict) or 'type' not in action:
                self.get_logger().error(f'Invalid action at index {i}: {action}')
                return False
            
            action_type = action['type']
            valid_action_types = ['move_to', 'grasp', 'detect_object', 'navigate_to', 'place_at', 'pick_and_place']
            
            if action_type not in valid_action_types:
                self.get_logger().error(f'Invalid action type at index {i}: {action_type}')
                return False
            
            # Validate action parameters based on type
            if action_type == 'move_to':
                if 'params' not in action or 'x' not in action['params'] or 'y' not in action['params']:
                    self.get_logger().error(f'Invalid move_to action at index {i}, missing x or y params: {action}')
                    return False
            elif action_type == 'grasp':
                if 'params' not in action or 'object_name' not in action['params']:
                    self.get_logger().error(f'Invalid grasp action at index {i}, missing object_name param: {action}')
                    return False
            elif action_type == 'detect_object':
                if 'params' not in action or 'object_name' not in action['params']:
                    self.get_logger().error(f'Invalid detect_object action at index {i}, missing object_name param: {action}')
                    return False
            elif action_type == 'navigate_to':
                if 'params' not in action or 'object_name' not in action['params']:
                    self.get_logger().error(f'Invalid navigate_to action at index {i}, missing object_name param: {action}')
                    return False
            elif action_type == 'place_at':
                if 'params' not in action or 'x' not in action['params'] or 'y' not in action['params'] or 'z' not in action['params']:
                    self.get_logger().error(f'Invalid place_at action at index {i}, missing x, y, or z params: {action}')
                    return False
            elif action_type == 'pick_and_place':
                if 'params' not in action or 'object_name' not in action['params'] or 'target_name' not in action['params']:
                    self.get_logger().error(f'Invalid pick_and_place action at index {i}, missing object_name or target_name param: {action}')
                    return False
        
        return True

    def generate_request_id(self):
        """
        Generate unique request ID for tracking
        
        Returns:
            str: Unique identifier for the request
        """
        import uuid
        return str(uuid.uuid4())


def main(args=None):
    """
    Main function to run the cognitive planner node
    """
    rclpy.init(args=args)
    cognitive_planner_node = CognitivePlannerNode()
    
    try:
        rclpy.spin(cognitive_planner_node)
    except KeyboardInterrupt:
        cognitive_planner_node.get_logger().info('Cognitive Planner Node interrupted by user')
    finally:
        cognitive_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
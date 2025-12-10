# Troubleshooting Cognitive Planning Issues in VLA Systems

## Overview

This troubleshooting guide addresses common issues encountered when implementing and using cognitive planning in Vision-Language-Action (VLA) systems. It provides diagnostic steps, solutions, and best practices to resolve problems effectively.

## Common Issues and Solutions

### 1. API Connection Failures

**Problem**: The cognitive planner fails to connect to the LLM API (e.g., OpenAI).

**Symptoms**:
- Error messages about API keys or network connectivity
- Timeout errors when generating action sequences
- Planner returns empty or default responses

**Diagnosis Steps**:
1. Check if the API key is correctly set in environment variables:
   ```bash
   echo $OPENAI_API_KEY
   ```

2. Verify network connectivity to the API endpoint:
   ```bash
   curl https://api.openai.com/v1/models -H "Authorization: Bearer $YOUR_API_KEY"
   ```

3. Check the ROS node logs for detailed error messages:
   ```bash
   ros2 run vla_examples cognitive_planner_node
   ```

**Solutions**:
1. Ensure the API key is set properly:
   ```bash
   export OPENAI_API_KEY='your-actual-api-key'
   ```

2. Verify the API key has sufficient permissions and hasn't exceeded rate limits.

3. Add error handling to manage API failures gracefully:
   ```python
   def plan_with_api_fallback(self, command, context):
       try:
           # Primary API call
           result = self.primary_llm_call(command, context)
           return result
       except APIError as e:
           self.get_logger().warn(f"Primary API failed: {e}")
           # Implement fallback mechanism
           return self.fallback_planning(command, context)
   ```

### 2. Unexpected Action Sequences

**Problem**: The planner generates action sequences that don't match the command or are inappropriate for the environment.

**Symptoms**:
- Actions that don't address the user's command
- Actions that reference non-existent objects
- Unsafe or invalid action sequences

**Diagnosis Steps**:
1. Examine the prompt sent to the LLM to ensure it contains all necessary context
2. Review the LLM response to see if it correctly interpreted the command
3. Check if the environment context is up-to-date and accurate

**Solutions**:
1. Improve prompt engineering with more explicit instructions:
   ```python
   def construct_improved_prompt(self, command, context):
       prompt = f"""
       Given the user command: "{command}"
       
       Current environment context:
       {json.dumps(context, indent=2)}
       
       Available actions: move_to(x, y), grasp(object), detect_object(object), navigate_to(object)
       
       INSTRUCTIONS:
       1. Only use objects that exist in the environment
       2. Ensure action parameters are appropriate for the action type
       3. Consider the robot's current state when generating sequences
       4. Output only valid JSON with the format specified
       
       Plan the action sequence:
       """
       return prompt
   ```

2. Add validation to ensure actions reference valid objects:
   ```python
   def validate_action_objects(self, actions, context):
       valid_objects = {obj['name'] for obj in context.get('objects', [])}
       for action in actions:
           if action['type'] in ['grasp', 'navigate_to', 'detect_object']:
               obj_name = action['params'].get('object_name')
               if obj_name not in valid_objects:
                   raise ValidationError(f"Action references non-existent object: {obj_name}")
   ```

### 3. Parsing Errors

**Problem**: The system fails to parse the LLM's response into a valid action sequence.

**Symptoms**:
- JSON parsing errors
- Action sequences with incorrect format
- Unexpected exceptions during response processing

**Diagnosis Steps**:
1. Log the raw LLM response to see its exact format
2. Check if the response contains markdown formatting (```json, ```)
3. Verify the JSON structure matches expected format

**Solutions**:
1. Implement robust parsing with cleaning:
   ```python
   def parse_llm_response(self, response_text):
       # Clean up markdown formatting
       cleaned = response_text.strip()
       if cleaned.startswith('```json'):
           cleaned = cleaned[7:]  # Remove ```json
       if cleaned.endswith('```'):
           cleaned = cleaned[:-3]  # Remove ```
       
       try:
           return json.loads(cleaned)
       except json.JSONDecodeError:
           # Try to fix common JSON issues
           fixed = self.attempt_json_fix(cleaned)
           if fixed:
               return json.loads(fixed)
           else:
               raise ValueError(f"Could not parse LLM response as JSON: {response_text}")
   ```

2. Handle partial responses gracefully:
   ```python
   def attempt_json_fix(self, json_str):
       # Attempt to fix common JSON issues
       import re
       
       # Fix missing quotes around keys
       json_str = re.sub(r'(\w+):', r'"\1":', json_str)
       
       # Fix trailing commas
       json_str = re.sub(r',(\s*[}\]])', r'\1', json_str)
       
       try:
           json.loads(json_str)
           return json_str
       except json.JSONDecodeError:
           return None
   ```

### 4. Context Awareness Problems

**Problem**: The planner doesn't consider the current environment context appropriately.

**Symptoms**:
- Actions that ignore object locations
- Robot attempting to grasp objects it's already holding
- Navigation to locations with obstacles

**Diagnosis Steps**:
1. Verify the context being sent to the LLM is comprehensive
2. Check if the robot's state (holding object, position) is included
3. Ensure perception data is current and accurate

**Solutions**:
1. Create a comprehensive context with all relevant information:
   ```python
   def build_comprehensive_context(self):
       context = {
           "robot_state": {
               "position": self.robot_position,
               "holding_object": self.currently_holding,
               "battery_level": self.battery_level
           },
           "environment": {
               "objects": self.get_detected_objects(),
               "obstacles": self.get_obstacles(),
               "map": self.current_map
           },
           "task_history": self.get_recent_task_history()
       }
       return context
   ```

2. Update context before each planning call:
   ```python
   def plan_with_fresh_context(self, command):
       # Update perception data
       self.update_perception()
       
       # Get fresh context
       context = self.build_comprehensive_context()
       
       # Plan with updated context
       return self.plan_action_sequence(command, context)
   ```

### 5. Safety Validation Failures

**Problem**: Valid action sequences are incorrectly flagged as unsafe, or unsafe sequences pass validation.

**Symptoms**:
- Valid plans getting rejected by safety validator
- Unsafe actions being executed
- Overly restrictive or permissive safety checks

**Diagnosis Steps**:
1. Examine which specific safety checks are failing
2. Review the safety validation logic for correctness
3. Check if environmental conditions are properly considered

**Solutions**:
1. Implement layered safety validation with clear logging:
   ```python
   def validate_with_detailed_logging(self, action_sequence):
       for i, action in enumerate(action_sequence):
           # Log each validation step
           self.get_logger().debug(f"Validating action {i}: {action['type']}")
           
           # Syntax validation
           if not self.validate_syntax(action):
               self.get_logger().warn(f"Syntax validation failed for action {i}")
               return False, f"Syntax error in action {i}"
           
           # Semantic validation
           if not self.validate_semantics(action):
               self.get_logger().warn(f"Semantic validation failed for action {i}")
               return False, f"Semantic error in action {i}"
           
           # Safety validation
           if not self.validate_safety(action):
               self.get_logger().warn(f"Safety validation failed for action {i}")
               return False, f"Safety error in action {i}"
       
       return True, "All validations passed"
   ```

2. Fine-tune safety thresholds based on testing:
   ```python
   class TunableSafetyValidator:
       def __init__(self):
           self.safety_thresholds = {
               'min_distance_to_obstacle': 0.3,  # meters
               'max_battery_usage_per_task': 0.8,  # fraction of battery
               'max_payload_weight': 1.0  # kilograms
           }
       
       def adjust_threshold(self, parameter, new_value):
           if parameter in self.safety_thresholds:
               self.safety_thresholds[parameter] = new_value
   ```

## Performance Issues

### Slow Response Times

**Problem**: The cognitive planner takes too long to generate action sequences.

**Diagnosis**:
1. Profile the LLM API call duration
2. Check if context building is taking excessive time
3. Verify the prompt is not too large

**Solutions**:
1. Implement caching for common queries:
   ```python
   from functools import lru_cache
   
   class CachedCognitivePlanner:
       @lru_cache(maxsize=128)
       def cached_plan(self, command_hash, context_hash):
           # Generate action sequence based on hashed inputs
           pass
   ```

2. Optimize context to include only essential information:
   ```python
   def optimize_context(self, full_context):
       # Only include objects relevant to the current command
       relevant_objects = self.extract_relevant_objects(
           full_context['objects'], 
           self.current_command
       )
       
       return {
           'robot_state': full_context['robot_state'],
           'relevant_objects': relevant_objects,
           'local_obstacles': full_context.get('local_obstacles', [])
       }
   ```

## Debugging Tools

### Adding Debugging Information

```python
def debug_plan_generation(self, command, context):
    """
    Enhanced planning function with comprehensive debugging
    """
    self.get_logger().info(f"Planning for command: {command}")
    self.get_logger().info(f"Context keys: {list(context.keys())}")
    
    # Log detailed context information
    if 'objects' in context:
        self.get_logger().info(f"Objects in context: {[obj.get('name') for obj in context['objects']]}")
    
    # Construct and log the prompt
    prompt = self.construct_prompt(command, context)
    self.get_logger().debug(f"Sent prompt to LLM: {prompt[:200]}...")  # Log first 200 chars
    
    # Make LLM call and log the response
    response = self.llm_client.generate(prompt)
    self.get_logger().debug(f"LLM response: {response[:200]}...")  # Log first 200 chars
    
    # Parse and log the action sequence
    action_sequence = self.parse_response(response)
    self.get_logger().info(f"Generated action sequence with {len(action_sequence)} actions")
    
    return action_sequence
```

### Logging Configuration

```python
import logging

def setup_detailed_logging(self):
    """
    Set up detailed logging for troubleshooting
    """
    # Configure logging
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('/tmp/vla_cognitive_planner.log'),
            logging.StreamHandler()
        ]
    )
    
    # Set specific loggers to DEBUG level
    logging.getLogger('openai').setLevel(logging.DEBUG)
    logging.getLogger('rclpy').setLevel(logging.INFO)
```

## Testing and Validation

### Unit Tests for Troubleshooting

```python
import unittest
from unittest.mock import Mock, patch
import json


class TestCognitivePlanningTroubleshooting(unittest.TestCase):
    
    def setUp(self):
        self.planner = CognitivePlannerNode()  # Your planner instance
    
    def test_api_failure_handling(self):
        """Test that API failures are handled gracefully"""
        with patch('openai.OpenAI') as mock_openai:
            # Simulate API failure
            mock_openai.return_value.chat.completions.create.side_effect = Exception("API Error")
            
            # Should handle gracefully, not crash
            result = self.planner.plan_actions("simple command")
            
            # Verify fallback behavior
            self.assertIsNotNone(result)  # Should not be None
    
    def test_parsing_error_handling(self):
        """Test that parsing errors are handled gracefully"""
        # Simulate invalid LLM response
        invalid_response = "This is not valid JSON"
        
        # Should not raise exception
        with self.assertRaises(ValueError):
            self.planner.parse_llm_response(invalid_response)
    
    def test_context_validation(self):
        """Test that invalid contexts are handled properly"""
        # Test with minimal context
        minimal_context = {"objects": [], "robot_state": {}}
        command = "Do something"
        
        # Should handle minimal context
        result = self.planner.plan_with_context(command, minimal_context)
        
        # Verify result is appropriate for minimal context
        self.assertIsInstance(result, list)  # Should return a list of actions or empty list
```

## Best Practices for Avoiding Issues

1. **Always validate inputs**: Verify commands and context before sending to the LLM
2. **Implement fallback mechanisms**: Have alternatives when primary systems fail
3. **Use comprehensive logging**: Log all important steps for debugging
4. **Test with edge cases**: Include tests for unusual or invalid inputs
5. **Monitor performance**: Track response times and identify bottlenecks
6. **Maintain updated dependencies**: Keep LLM clients and ROS packages current

## Conclusion

Troubleshooting cognitive planning in VLA systems requires a systematic approach to diagnose and resolve issues. By understanding the common failure modes and implementing appropriate validation, error handling, and debugging tools, you can create more robust and reliable cognitive planning systems. The key is to anticipate potential failures and implement graceful handling mechanisms while maintaining comprehensive logging for effective debugging.
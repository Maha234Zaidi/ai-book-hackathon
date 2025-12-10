# Multi-Step Task Planning Examples for VLA Systems

## Overview

Multi-step task planning is a crucial capability in Vision-Language-Action (VLA) systems, enabling robots to execute complex commands by breaking them down into sequences of simpler actions. This document provides examples of multi-step task planning scenarios with detailed code implementations.

## Example 1: Object Retrieval and Delivery

### Task Description
Command: "Go to the kitchen, pick up the red cup, and bring it to the living room table."

### Step-by-Step Breakdown
1. Navigate to the kitchen
2. Detect the red cup
3. Grasp the red cup
4. Navigate to the living room
5. Navigate to the table in the living room
6. Place the red cup on the table

### Code Implementation

```python
class ObjectRetrievalPlanner:
    def __init__(self, llm_interface, context_manager):
        self.llm_interface = llm_interface
        self.context_manager = context_manager
    
    def plan_object_retrieval_task(self, target_object, destination):
        """
        Plan a sequence to retrieve an object and place it at a destination
        """
        # Get current context
        context = self.context_manager.get_context_for_planning()
        
        # Break down the complex task into subtasks
        subtasks = [
            f"Navigate to the location of {target_object}",
            f"Grasp the {target_object}",
            f"Navigate to the {destination}",
            f"Place the {target_object} at the {destination}"
        ]
        
        # Plan each subtask
        full_action_sequence = []
        
        for i, subtask in enumerate(subtasks):
            # Update context after each subtask planning to reflect expected state changes
            if i > 0:
                self._update_context_for_subtask(subtask, full_action_sequence[-1])
            
            # Plan actions for the current subtask
            subtask_actions, subtask_reasoning = self.llm_interface.plan_action_sequence(
                subtask,
                self.context_manager.get_context_for_planning()
            )
            
            if subtask_actions:
                full_action_sequence.extend(subtask_actions)
            else:
                print(f"Failed to plan for subtask: {subtask}")
                # Return partial sequence or handle error
                return full_action_sequence, f"Failed at subtask: {subtask}"
        
        return full_action_sequence, "Successfully planned object retrieval task"
    
    def _update_context_for_subtask(self, subtask, last_action):
        """
        Update the context based on the expected effect of the previous action
        """
        # For example, if the last action was a grasp, update the robot state to holding
        if last_action and last_action.type == "grasp":
            self.context_manager.current_state.robot_state["holding"] = last_action.params.get("object_name")
    
    def execute_retrieval_task(self, target_object, destination):
        """
        Execute the full retrieval task with safety monitoring
        """
        # Plan the task
        action_sequence, reason = self.plan_object_retrieval_task(target_object, destination)
        
        if not action_sequence:
            print(f"Could not plan task: {reason}")
            return False
        
        # Validate the sequence
        validator = ActionValidator(None, self.context_manager)  # Use appropriate robot interface
        is_valid, validation_reason = validator.validate_action_sequence(action_sequence)
        
        if not is_valid:
            print(f"Action sequence validation failed: {validation_reason}")
            return False
        
        # Execute with monitoring
        monitor = SafetyMonitor(None)  # Use appropriate robot interface
        success, execution_reason = monitor.monitor_execution(action_sequence)
        
        if success:
            print(f"Successfully completed retrieval task: {target_object} -> {destination}")
        else:
            print(f"Task execution failed: {execution_reason}")
        
        return success


# Example usage
def example_usage():
    # Initialize components (assuming these are properly instantiated)
    llm_interface = LLMInterface()  # From detailed implementation
    context_manager = ContextManager()  # From detailed implementation
    
    # Create the planner
    planner = ObjectRetrievalPlanner(llm_interface, context_manager)
    
    # Execute a retrieval task
    success = planner.execute_retrieval_task("red cup", "living room table")
    
    if success:
        print("Task completed successfully!")
    else:
        print("Task failed!")
```

## Example 2: Room Cleaning Task

### Task Description
Command: "Clean the room by picking up all the toys and placing them in the toy box."

### Step-by-Step Breakdown
1. Detect all toys in the room
2. For each detected toy:
   a. Navigate to the toy
   b. Grasp the toy
   c. Navigate to the toy box
   d. Place the toy in the toy box
3. Return to the starting position

### Code Implementation

```python
class RoomCleaningPlanner:
    def __init__(self, llm_interface, context_manager):
        self.llm_interface = llm_interface
        self.context_manager = context_manager
    
    def plan_cleaning_task(self, toy_category="toy", storage_location="toy box"):
        """
        Plan a sequence to clean a room by collecting all items of a category
        """
        # Update context to get the latest environment information
        self.context_manager.update_from_perception()
        context = self.context_manager.get_context_for_planning()
        
        # Detect all toys in the environment
        toys = self._find_objects_by_category(context, toy_category)
        storage_location_obj = self._find_object_by_name(context, storage_location)
        
        if not toys:
            print(f"No {toy_category}s found in the environment")
            return [], "No toys found to clean"
        
        if not storage_location_obj:
            print(f"Storage location '{storage_location}' not found")
            return [], f"Storage location '{storage_location}' not found"
        
        # Plan the complete cleaning sequence
        full_action_sequence = []
        
        for toy in toys:
            # Plan the sequence for this specific toy
            toy_sequence = self._plan_single_toy_sequence(toy, storage_location_obj)
            full_action_sequence.extend(toy_sequence)
        
        # Add return to starting position
        return_action = Action(
            type="navigate_to",
            params={"object_name": "starting_position"},
            description="Return to starting position"
        )
        full_action_sequence.append(return_action)
        
        return full_action_sequence, f"Planned cleaning for {len(toys)} {toy_category}s"
    
    def _find_objects_by_category(self, context, category):
        """
        Find all objects in the environment that match a category
        """
        matching_objects = []
        
        for obj_name, obj_data in context["objects"].items():
            # In a real implementation, objects would have categories
            # For this example, we'll consider all objects as toys
            if category.lower() in obj_name.lower() or obj_data.get("category") == category:
                obj_data["name"] = obj_name
                matching_objects.append(obj_data)
        
        return matching_objects
    
    def _find_object_by_name(self, context, name):
        """
        Find a specific object by name
        """
        for obj_name, obj_data in context["objects"].items():
            if name.lower() in obj_name.lower():
                obj_data["name"] = obj_name
                return obj_data
        return None
    
    def _plan_single_toy_sequence(self, toy, storage_location):
        """
        Plan the sequence to move a single toy to storage
        """
        sequence = []
        
        # Navigate to the toy
        nav_to_toy = Action(
            type="navigate_to",
            params={"object_name": toy["name"]},
            description=f"Navigate to {toy['name']}"
        )
        sequence.append(nav_to_toy)
        
        # Grasp the toy
        grasp_action = Action(
            type="grasp",
            params={"object_name": toy["name"]},
            description=f"Grasp {toy['name']}"
        )
        sequence.append(grasp_action)
        
        # Navigate to storage location
        nav_to_storage = Action(
            type="navigate_to",
            params={"object_name": storage_location["name"]},
            description=f"Navigate to {storage_location['name']}"
        )
        sequence.append(nav_to_storage)
        
        # Place the toy
        place_action = Action(
            type="place_at",
            params=storage_location.get("position", {"x": 0, "y": 0, "z": 0}),
            description=f"Place toy in {storage_location['name']}"
        )
        sequence.append(place_action)
        
        return sequence


# Example usage for room cleaning
def room_cleaning_example():
    llm_interface = LLMInterface()
    context_manager = ContextManager()
    
    cleaner = RoomCleaningPlanner(llm_interface, context_manager)
    
    # Plan a cleaning task
    action_sequence, reason = cleaner.plan_cleaning_task("toy", "toy box")
    
    if action_sequence:
        print(f"Planned cleaning sequence with {len(action_sequence)} actions")
        for i, action in enumerate(action_sequence):
            print(f"  {i+1}. {action.description}")
    else:
        print(f"Cleaning plan failed: {reason}")
```

## Example 3: Complex Multi-Object Manipulation

### Task Description
Command: "Stack the blocks in order: blue block on the table, red block on top of blue block, yellow block on top of red block."

### Step-by-Step Breakdown
1. Navigate to blue block
2. Grasp blue block
3. Navigate to table
4. Place blue block on table
5. Navigate to red block
6. Grasp red block
7. Navigate to blue block (on table)
8. Place red block on top of blue block
9. Navigate to yellow block
10. Grasp yellow block
11. Navigate to red block (on blue block)
12. Place yellow block on top of red block

### Code Implementation

```python
class BlockStackingPlanner:
    def __init__(self, llm_interface, context_manager):
        self.llm_interface = llm_interface
        self.context_manager = context_manager
    
    def plan_block_stacking_task(self, blocks_order, base_object="table"):
        """
        Plan a sequence to stack blocks in a specific order
        """
        # Update context to get the latest environment information
        self.context_manager.update_from_perception()
        context = self.context_manager.get_context_for_planning()
        
        # Find all required blocks and the base object
        block_objects = []
        for block_name in blocks_order:
            block_obj = self._find_object_by_name(context, block_name)
            if block_obj:
                block_obj["name"] = block_name
                block_objects.append(block_obj)
            else:
                print(f"Block '{block_name}' not found in environment")
                return [], f"Block '{block_name}' not found"
        
        base_obj = self._find_object_by_name(context, base_object)
        if not base_obj:
            print(f"Base object '{base_object}' not found in environment")
            return [], f"Base object '{base_object}' not found"
        
        # Plan the stacking sequence
        full_action_sequence = []
        
        # Place the first block on the base
        first_block = block_objects[0]
        first_sequence = self._plan_place_block_sequence(first_block, base_obj)
        full_action_sequence.extend(first_sequence)
        
        # For subsequent blocks, place each on top of the previous one
        for i in range(1, len(block_objects)):
            current_block = block_objects[i]
            previous_block = block_objects[i-1]  # Place on top of this
            
            # For simplicity, we'll place on the previous block
            # In a real system, you'd calculate the precise position
            block_sequence = self._plan_place_block_sequence(current_block, previous_block)
            full_action_sequence.extend(block_sequence)
        
        return full_action_sequence, f"Planned stacking sequence for {len(blocks_order)} blocks"
    
    def _plan_place_block_sequence(self, block_to_place, base_object):
        """
        Plan the sequence to place one block on another
        """
        sequence = []
        
        # Navigate to the block to place
        nav_to_block = Action(
            type="navigate_to",
            params={"object_name": block_to_place["name"]},
            description=f"Navigate to {block_to_place['name']}"
        )
        sequence.append(nav_to_block)
        
        # Grasp the block
        grasp_action = Action(
            type="grasp",
            params={"object_name": block_to_place["name"]},
            description=f"Grasp {block_to_place['name']}"
        )
        sequence.append(grasp_action)
        
        # Navigate to the base object
        nav_to_base = Action(
            type="navigate_to",
            params={"object_name": base_object["name"]},
            description=f"Navigate to {base_object['name']}"
        )
        sequence.append(nav_to_base)
        
        # Place the block on the base
        place_action = Action(
            type="place_at",
            params={
                "x": base_object["position"]["x"],
                "y": base_object["position"]["y"],
                "z": base_object["position"]["z"] + 0.1  # Slightly higher to stack
            },
            description=f"Place {block_to_place['name']} on {base_object['name']}"
        )
        sequence.append(place_action)
        
        return sequence
    
    def _find_object_by_name(self, context, name):
        """
        Find a specific object by name
        """
        for obj_name, obj_data in context["objects"].items():
            if name.lower() in obj_name.lower():
                obj_data["name"] = obj_name
                return obj_data
        return None


# Example usage for block stacking
def block_stacking_example():
    llm_interface = LLMInterface()
    context_manager = ContextManager()
    
    stacker = BlockStackingPlanner(llm_interface, context_manager)
    
    # Plan a block stacking task
    blocks_order = ["blue block", "red block", "yellow block"]
    action_sequence, reason = stacker.plan_block_stacking_task(blocks_order, "table")
    
    if action_sequence:
        print(f"Planned stacking sequence with {len(action_sequence)} actions")
        for i, action in enumerate(action_sequence):
            print(f"  {i+1}. {action.description}")
    else:
        print(f"Stacking plan failed: {reason}")
```

## Advanced Multi-Step Planning: Conditional Execution

Sometimes, multi-step tasks require conditional execution based on the state of the environment during execution:

```python
class ConditionalExecutionPlanner:
    def __init__(self, llm_interface, context_manager):
        self.llm_interface = llm_interface
        self.context_manager = context_manager
    
    def plan_conditional_task(self, command):
        """
        Plan a task with conditional execution based on environment state
        """
        # Example: "Go to the kitchen and if you see the red cup, bring it to me,
        # otherwise find the blue mug and bring that instead."
        
        # This requires breaking down the command into conditional branches
        primary_condition = "detect_object(red cup)"
        primary_action = "bring red cup to user"
        alternative_condition = "detect_object(blue mug)" 
        alternative_action = "bring blue mug to user"
        
        # Plan conditional action sequence
        sequence = []
        
        # First, go to the kitchen
        nav_to_kitchen = Action(
            type="navigate_to",
            params={"object_name": "kitchen"},
            description="Navigate to the kitchen"
        )
        sequence.append(nav_to_kitchen)
        
        # Add conditional detection
        detect_red_cup = Action(
            type="detect_object",
            params={"object_name": "red cup"},
            description="Detect red cup"
        )
        sequence.append(detect_red_cup)
        
        # At execution time, check if red cup was detected and proceed accordingly
        # This would require a more sophisticated execution system that can handle conditionals
        
        return sequence, "Planned conditional execution task"
    
    def execute_conditional_task(self, action_sequence):
        """
        Execute a sequence with conditional logic
        """
        for action in action_sequence:
            # Execute action
            result = self._execute_action(action)
            
            # If action was a detection, check the result and adjust plan
            if action.type == "detect_object":
                if not result['detected']:
                    # Modify plan based on alternative actions
                    self._handle_detection_failure(action)
        
        return True
    
    def _execute_action(self, action):
        """
        Execute a single action and return result
        """
        # Placeholder for actual action execution
        return {"success": True, "detected": True}
    
    def _handle_detection_failure(self, detection_action):
        """
        Handle the case where a detection action fails
        """
        # Implement alternative actions when detection fails
        print(f"Detection of {detection_action.params['object_name']} failed, executing alternative plan")
```

## Best Practices for Multi-Step Task Planning

1. **Modular Design**: Break complex tasks into reusable subtask modules
2. **Context Awareness**: Always update context based on action effects
3. **Error Handling**: Implement fallback plans for failed actions
4. **Validation**: Validate intermediate states during complex task execution
5. **Monitoring**: Continuously monitor execution for deviations from expected behavior

## Summary

Multi-step task planning in VLA systems requires careful consideration of environment context, action effects, and conditional execution logic. The examples provided demonstrate how to handle various types of multi-step tasks, from simple object retrieval to complex conditional execution scenarios. Key to success is maintaining an accurate world state representation and implementing robust validation and error handling mechanisms.
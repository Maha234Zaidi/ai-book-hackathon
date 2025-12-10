# Main VLA System Orchestrator Node Implementation

## Overview

The VLA System Orchestrator is the central component that coordinates all modules in the Vision-Language-Action system. It manages the flow of information between the voice command pipeline, cognitive planner, perception module, path planning component, and manipulation system. The orchestrator ensures system state consistency and handles error recovery.

## Implementation

```python
#!/usr/bin/env python3
"""
VLA System Orchestrator Node

This node coordinates all components in the Vision-Language-Action system,
managing the flow of information and ensuring system state consistency.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from vla_msgs.msg import ActionSequence, VLAAction, DetectedObjects
from vla_msgs.srv import ExecuteAction, PlanCognitiveTask, GetPath
import json
import time
import threading
from enum import Enum
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from datetime import datetime


class SystemState(Enum):
    """Enumeration of possible system states"""
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    EXECUTING = "executing"
    ERROR = "error"
    SAFETY_STOP = "safety_stop"


@dataclass
class SystemContext:
    """Maintains the system's current context"""
    robot_position: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})
    robot_state: Dict[str, Any] = field(default_factory=lambda: {"holding": None, "battery": 1.0})
    detected_objects: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    current_command: str = ""
    action_sequence: List[Dict] = field(default_factory=list)
    system_state: SystemState = SystemState.IDLE
    last_update: float = 0.0
    
    def update_timestamp(self):
        self.last_update = time.time()


class VLASystemOrchestrator(Node):
    """
    Main orchestrator node for the VLA system
    """
    
    def __init__(self):
        super().__init__('vla_system_orchestrator')
        
        # Initialize system context
        self.context = SystemContext()
        
        # QoS profile for reliable communication
        self.qos_profile = QoSProfile(depth=10)
        
        # Publishers for system communication
        self.status_publisher = self.create_publisher(
            String,
            '/vla/status',
            self.qos_profile
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            self.qos_profile
        )
        
        self.action_sequence_publisher = self.create_publisher(
            ActionSequence,
            '/vla/action_sequence',
            self.qos_profile
        )
        
        # Subscribers for system inputs
        self.natural_command_subscriber = self.create_subscription(
            String,
            '/vla/natural_command',
            self.natural_command_callback,
            self.qos_profile
        )
        
        self.perception_objects_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.perception_objects_callback,
            self.qos_profile
        )
        
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            self.qos_profile
        )
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.qos_profile
        )
        
        # Service clients for communication with other components
        self.cognitive_planner_client = self.create_client(
            PlanCognitiveTask,
            'plan_cognitive_task'
        )
        
        self.path_planner_client = self.create_client(
            GetPath,
            'get_path'
        )
        
        self.action_executor_client = self.create_client(
            ExecuteAction,
            'execute_action'
        )
        
        # Timers for periodic tasks
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.state_monitor_timer = self.create_timer(0.5, self.monitor_system_state)
        
        # Internal state
        self.current_action_sequence: List[VLAAction] = []
        self.current_action_index = 0
        self.is_executing = False
        self.emergency_stop = False
        
        # Wait for services to be available
        while not self.cognitive_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Cognitive planner service not available, waiting again...')
        
        while not self.path_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Path planner service not available, waiting again...')
            
        while not self.action_executor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Action executor service not available, waiting again...')
        
        self.get_logger().info('VLA System Orchestrator initialized and ready')
        self.update_system_state(SystemState.IDLE)
    
    def update_system_state(self, new_state: SystemState):
        """Update the system state and log the change"""
        old_state = self.context.system_state
        self.context.system_state = new_state
        self.context.update_timestamp()
        
        self.get_logger().info(f'System state changed from {old_state.value} to {new_state.value}')
        
        # Publish status update
        status_msg = String()
        status_msg.data = json.dumps({
            'state': new_state.value,
            'timestamp': self.context.last_update,
            'context_summary': {
                'num_detected_objects': len(self.context.detected_objects),
                'holding_object': self.context.robot_state['holding'],
                'battery_level': self.context.robot_state['battery']
            }
        })
        self.status_publisher.publish(status_msg)
    
    def natural_command_callback(self, msg: String):
        """Handle incoming natural language commands"""
        command = msg.data
        self.get_logger().info(f'Received natural command: {command}')
        
        if self.context.system_state in [SystemState.ERROR, SystemState.SAFETY_STOP]:
            self.get_logger().warn('System in error state, ignoring command')
            return
        
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop active, ignoring command')
            return
        
        # Update context
        self.context.current_command = command
        self.context.update_timestamp()
        
        # Trigger cognitive planning
        self.update_system_state(SystemState.PROCESSING)
        future = self.cognitive_planner_client.call_async(
            PlanCognitiveTask.Request(command=command)
        )
        future.add_done_callback(self.cognitive_planning_callback)
    
    def cognitive_planning_callback(self, future):
        """Handle response from cognitive planner"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Cognitive planning successful, got {len(response.action_sequence.actions)} actions')
                
                # Store the action sequence
                self.current_action_sequence = response.action_sequence.actions
                self.current_action_index = 0
                
                # Publish the action sequence
                action_seq_msg = ActionSequence()
                action_seq_msg.request_id = response.action_sequence.request_id
                action_seq_msg.command = response.action_sequence.command
                action_seq_msg.actions = self.current_action_sequence
                
                self.action_sequence_publisher.publish(action_seq_msg)
                
                # Start execution
                self.update_system_state(SystemState.EXECUTING)
                self.execute_next_action()
            else:
                self.get_logger().error(f'Cognitive planning failed: {response.message}')
                self.update_system_state(SystemState.ERROR)
                
        except Exception as e:
            self.get_logger().error(f'Error in cognitive planning callback: {e}')
            self.update_system_state(SystemState.ERROR)
    
    def execute_next_action(self):
        """Execute the next action in the sequence"""
        if self.current_action_index >= len(self.current_action_sequence):
            # Sequence completed
            self.get_logger().info('Action sequence completed')
            self.current_action_index = 0
            self.current_action_sequence = []
            self.is_executing = False
            self.update_system_state(SystemState.IDLE)
            return
        
        if self.emergency_stop:
            self.get_logger().info('Emergency stop during execution')
            self.update_system_state(SystemState.SAFETY_STOP)
            return
        
        # Get the next action
        action = self.current_action_sequence[self.current_action_index]
        self.get_logger().info(f'Executing action {self.current_action_index + 1}/{len(self.current_action_sequence)}: {action.type}')
        
        # Create action execution request
        request = ExecuteAction.Request()
        request.action = action
        request.context = json.dumps({
            'robot_state': self.context.robot_state,
            'detected_objects': self.context.detected_objects
        })
        
        # Execute the action
        self.is_executing = True
        future = self.action_executor_client.call_async(request)
        future.add_done_callback(self.action_execution_callback)
    
    def action_execution_callback(self, future):
        """Handle response from action executor"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Action executed successfully: {response.message}')
                
                # Update internal state based on action type
                self.update_context_from_action(
                    self.current_action_sequence[self.current_action_index]
                )
                
                # Move to next action
                self.current_action_index += 1
                self.execute_next_action()
            else:
                self.get_logger().error(f'Action execution failed: {response.message}')
                
                # Attempt recovery
                self.attempt_recovery(response.error_code)
                
        except Exception as e:
            self.get_logger().error(f'Error in action execution callback: {e}')
            self.update_system_state(SystemState.ERROR)
    
    def update_context_from_action(self, action: VLAAction):
        """Update system context based on the effect of an action"""
        action_type = action.type
        params = json.loads(action.params) if action.params else {}
        
        if action_type == 'grasp':
            obj_name = params.get('object_name')
            if obj_name:
                self.context.robot_state['holding'] = obj_name
                self.get_logger().info(f'Robot is now holding: {obj_name}')
        
        elif action_type == 'place_at':
            if self.context.robot_state['holding']:
                holding_obj = self.context.robot_state['holding']
                self.context.robot_state['holding'] = None
                self.get_logger().info(f'Robot placed: {holding_obj}')
        
        elif action_type == 'move_to':
            self.context.robot_position['x'] = params.get('x', self.context.robot_position['x'])
            self.context.robot_position['y'] = params.get('y', self.context.robot_position['y'])
            self.get_logger().info(f'Robot moved to position: ({self.context.robot_position["x"]}, {self.context.robot_position["y"]})')
    
    def attempt_recovery(self, error_code: str):
        """Attempt to recover from an action execution error"""
        self.get_logger().info(f'Attempting to recover from error: {error_code}')
        
        # Different recovery strategies based on error type
        if error_code == 'OBSTACLE_DETECTED':
            self.get_logger().warn('Obstacle detected during navigation, replanning path')
            # For simplicity, just return to idle and request new plan
            # In a real system, this would trigger path replanning
            self.update_system_state(SystemState.IDLE)
        elif error_code == 'OBJECT_NOT_FOUND':
            self.get_logger().warn('Object not found, requesting perception update')
            # Trigger a new perception cycle
            self.request_perception_update()
        elif error_code == 'GRASP_FAILED':
            self.get_logger().warn('Grasp failed, attempting alternative approach')
            # Retry with modified parameters
            self.retry_current_action_with_modifications()
        else:
            self.get_logger().error(f'Unknown error code, going to error state: {error_code}')
            self.update_system_state(SystemState.ERROR)
    
    def request_perception_update(self):
        """Request a fresh perception update"""
        # This would trigger a more intensive perception process
        self.get_logger().info('Requesting perception update')
        # Implementation would depend on the specific perception system
    
    def retry_current_action_with_modifications(self):
        """Retry the current action with modified parameters"""
        if self.current_action_index < len(self.current_action_sequence):
            action = self.current_action_sequence[self.current_action_index]
            self.get_logger().info(f'Retrying action with modifications: {action.type}')
            # In a real implementation, this would modify the action parameters
            # and attempt execution again
    
    def perception_objects_callback(self, msg: DetectedObjects):
        """Handle updates from perception module"""
        for obj in msg.objects:
            self.context.detected_objects[obj.name] = {
                'position': {'x': obj.pose.position.x, 'y': obj.pose.position.y, 'z': obj.pose.position.z},
                'pose': obj.pose,
                'confidence': obj.confidence
            }
        self.context.update_timestamp()
        self.get_logger().info(f'Updated perception data: {len(msg.objects)} objects detected')
    
    def camera_callback(self, msg: Image):
        """Handle camera input (for potential real-time perception)"""
        # In a real system, this might trigger real-time object detection
        pass
    
    def scan_callback(self, msg: LaserScan):
        """Handle laser scan input (for navigation safety)"""
        # Monitor for obstacles in the current path
        if self.context.system_state == SystemState.EXECUTING:
            # Check if there are obstacles directly ahead
            if self.is_path_obstructed(msg):
                self.get_logger().warn('Path obstruction detected during navigation')
                # Trigger emergency stop or path replanning
                self.emergency_stop = True
                self.update_system_state(SystemState.SAFETY_STOP)
    
    def is_path_obstructed(self, scan_msg: LaserScan) -> bool:
        """Check if the path ahead is obstructed based on laser scan"""
        # This is a simplified check - only look at the front-facing region
        front_range = len(scan_msg.ranges) // 2  # Front-facing direction
        
        # Check a small range around the front
        for i in range(max(0, front_range - 10), min(len(scan_msg.ranges), front_range + 10)):
            if 0 < scan_msg.ranges[i] < 0.5:  # Object within 0.5m
                return True
        return False
    
    def publish_status(self):
        """Periodically publish system status"""
        status_msg = String()
        status_msg.data = json.dumps({
            'state': self.context.system_state.value,
            'timestamp': time.time(),
            'context_summary': {
                'num_detected_objects': len(self.context.detected_objects),
                'holding_object': self.context.robot_state['holding'],
                'battery_level': self.context.robot_state['battery'],
                'robot_position': self.context.robot_position,
                'current_command': self.context.current_command,
                'actions_remaining': len(self.current_action_sequence) - self.current_action_index if self.current_action_sequence else 0
            }
        })
        self.status_publisher.publish(status_msg)
    
    def monitor_system_state(self):
        """Monitor system state and trigger corrective actions if needed"""
        current_time = time.time()
        
        # If in execution state for too long without progress, trigger safety stop
        if (self.context.system_state == SystemState.EXECUTING and 
            current_time - self.context.last_update > 30.0):  # 30 seconds timeout
            self.get_logger().error('Execution timeout detected, triggering safety stop')
            self.emergency_stop = True
            self.update_system_state(SystemState.SAFETY_STOP)
        
        # Monitor battery level
        if self.context.robot_state['battery'] < 0.1:  # Below 10%
            if self.context.system_state in [SystemState.PROCESSING, SystemState.EXECUTING]:
                self.get_logger().warn('Low battery, returning to charging station')
                self.return_to_charging_station()
    
    def return_to_charging_station(self):
        """Initiate return to charging station due to low battery"""
        self.get_logger().info('Initiating return to charging station due to low battery')
        
        # For now, just go to a fixed charging position (0, 0)
        charging_action = VLAAction()
        charging_action.type = 'move_to'
        charging_action.params = json.dumps({'x': 0.0, 'y': 0.0})
        charging_action.description = 'Return to charging station'
        
        # Create a simple action sequence to return to charging station
        action_seq = ActionSequence()
        action_seq.request_id = 'battery_return_' + str(time.time())
        action_seq.command = 'Return to charging station due to low battery'
        action_seq.actions = [charging_action]
        
        # Publish the emergency action sequence
        self.action_sequence_publisher.publish(action_seq)
        
        # Update state
        self.current_action_sequence = [charging_action]
        self.current_action_index = 0
        self.update_system_state(SystemState.EXECUTING)
        self.execute_next_action()
    
    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop command"""
        if msg.data:
            self.get_logger().warn('Emergency stop activated')
            self.emergency_stop = True
            self.update_system_state(SystemState.SAFETY_STOP)
            
            # Stop any movement
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(stop_cmd)
        else:
            self.get_logger().info('Emergency stop released')
            self.emergency_stop = False
            self.update_system_state(SystemState.IDLE)


def main(args=None):
    """
    Main function to run the VLA System Orchestrator node
    """
    rclpy.init(args=args)
    
    orchestrator = VLASystemOrchestrator()
    
    try:
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        orchestrator.get_logger().info('VLA System Orchestrator interrupted by user')
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Key Features of the Orchestrator

### 1. System State Management
The orchestrator maintains the overall system state and transitions between states based on events and component responses. The states include:

- **IDLE**: System waiting for commands
- **LISTENING**: Processing audio input
- **PROCESSING**: Cognitive planning in progress
- **EXECUTING**: Action sequence execution
- **ERROR**: Error state requiring intervention
- **SAFETY_STOP**: Safety stop initiated

### 2. Context Management
The orchestrator maintains important system context including:
- Robot position and state (holding object, battery level)
- Detected objects in the environment
- Current command and action sequence
- System state and timing information

### 3. Communication Coordination
The orchestrator manages communication between all system components through ROS 2 topics and services, ensuring proper message flow and synchronization.

### 4. Safety and Error Handling
The orchestrator implements multiple safety mechanisms:
- Path obstruction detection using laser scan data
- Execution timeout monitoring
- Battery level monitoring
- Emergency stop capability
- Error recovery procedures

### 5. Execution Management
The orchestrator manages the execution of action sequences, handling:
- Sequential execution of actions
- Context updates after each action
- Recovery from failed actions
- Proper sequencing of complex multi-step tasks

## Integration with Other Components

The orchestrator integrates with other system components as follows:

1. **Voice Command Pipeline**: Subscribes to `/vla/natural_command` to receive processed voice commands
2. **Cognitive Planner**: Uses the `plan_cognitive_task` service to generate action sequences
3. **Perception Module**: Subscribes to `/vla/perception/objects` for environmental information
4. **Action Executor**: Uses the `execute_action` service to execute individual actions
5. **Path Planner**: Uses the `get_path` service for navigation planning

## Running the Orchestrator Node

To run the orchestrator node:

```bash
ros2 run vla_examples vla_system_orchestrator_node
```

The orchestrator will wait for services from other components to be available and then begin coordinating the VLA system.

## Conclusion

The VLA System Orchestrator serves as the central nervous system of the VLA architecture, coordinating all components to achieve the system's goals. It manages system state, handles errors gracefully, and ensures safe and effective operation of the complete VLA system.
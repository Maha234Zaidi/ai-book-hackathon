# Manipulation System for Object Interaction

## Overview

The manipulation system is a critical component of the Vision-Language-Action (VLA) system that enables the robot to physically interact with objects in its environment. This system controls the robot's end-effectors (grippers, arms) to perform tasks such as grasping, moving, and placing objects based on commands from the cognitive planner.

## Architecture

The manipulation system follows a modular architecture with the following components:

1. **Grasp Planning Engine**: Plans appropriate grasp configurations
2. **Motion Control**: Controls arm and gripper movements
3. **Force Control**: Manages grasp strength and manipulation forces
4. **Object Interaction Manager**: Coordinates complex object manipulation tasks
5. **Safety Validator**: Ensures safe manipulation operations
6. **Sensor Integration**: Incorporates tactile and force feedback

## Implementation

### Manipulation Control Node

```python
#!/usr/bin/env python3
"""
Manipulation System for VLA System

This node controls the robot's manipulation capabilities, including grasping,
releasing, and manipulating objects in the environment.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import JointState, Image
from vla_msgs.msg import VLAAction, DetectedObjects, Object
from vla_msgs.srv import ExecuteAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryController
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import math
from typing import Dict, List, Optional, Tuple
import time


class ManipulationSystem(Node):
    """
    Manipulation system for object interaction in the VLA system
    """
    
    def __init__(self):
        super().__init__('manipulation_system')
        
        # QoS profile
        self.qos_profile = QoSProfile(depth=10)
        
        # TF2 setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Service for action execution
        self.action_execution_service = self.create_service(
            ExecuteAction,
            'execute_action',
            self.execute_action_callback
        )
        
        # Publishers
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.qos_profile
        )
        
        self.gripper_command_publisher = self.create_publisher(
            Float64,
            '/gripper_controller/command',
            self.qos_profile
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/manipulation/status',
            self.qos_profile
        )
        
        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            self.qos_profile
        )
        
        self.detected_objects_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            self.detected_objects_callback,
            self.qos_profile
        )
        
        # Internal state
        self.robot_joints = {}
        self.detected_objects = {}
        self.is_grasping = False
        self.currently_holding = None
        self.grasp_success_threshold = 0.1  # Distance threshold for successful grasp
        self.max_gripper_force = 100.0  # Maximum grip force in Newtons
        
        # Robot-specific parameters
        self.gripper_open_position = 0.05  # Gripper position when open
        self.gripper_close_position = 0.0  # Gripper position when closed
        self.gripper_approach_position = 0.02  # Gripper position for approach
        self.arm_approach_height = 0.1  # Height to approach object from above
        
        self.get_logger().info('Manipulation System initialized')

    def execute_action_callback(self, request, response):
        """
        Handle manipulation action execution requests
        """
        action = request.action
        self.get_logger().info(f'Executing manipulation action: {action.type}')
        
        try:
            if action.type == 'grasp':
                # Execute grasp action
                success, message = self.execute_grasp_action(action)
                response.success = success
                response.message = message
                response.error_code = "" if success else "GRASP_FAILED"
                
            elif action.type == 'place_at':
                # Execute place action
                success, message = self.execute_place_action(action)
                response.success = success
                response.message = message
                response.error_code = "" if success else "PLACE_FAILED"
                
            elif action.type == 'move_to':
                # Execute move action (for arm positioning)
                success, message = self.execute_move_action(action)
                response.success = success
                response.message = message
                response.error_code = "" if success else "MOVE_FAILED"
                
            elif action.type == 'release':
                # Execute release action
                success, message = self.execute_release_action()
                response.success = success
                response.message = message
                response.error_code = "" if success else "RELEASE_FAILED"
                
            else:
                response.success = False
                response.message = f"Unknown action type: {action.type}"
                response.error_code = "UNKNOWN_ACTION"
        
        except Exception as e:
            self.get_logger().error(f'Error executing action: {e}')
            response.success = False
            response.message = f'Action execution error: {str(e)}'
            response.error_code = "EXECUTION_ERROR"
        
        return response

    def execute_grasp_action(self, action: VLAAction) -> Tuple[bool, str]:
        """
        Execute a grasp action
        """
        # Extract object name from action parameters
        params = json.loads(action.params) if action.params else {}
        object_name = params.get('object_name')
        
        if not object_name:
            return False, "Object name not specified in grasp action"
        
        # Find object in detected objects
        obj = self.detected_objects.get(object_name)
        if not obj:
            return False, f"Object '{object_name}' not detected in environment"
        
        # Calculate approach position (above object)
        approach_pos = Point()
        approach_pos.x = obj.pose.position.x
        approach_pos.y = obj.pose.position.y
        approach_pos.z = obj.pose.position.z + self.arm_approach_height  # Approach from above
        
        # Move arm to approach position
        self.get_logger().info(f'Moving to approach position for {object_name}')
        success = self.move_arm_to_position(approach_pos)
        if not success:
            return False, f"Failed to move to approach position for {object_name}"
        
        # Lower arm to object height
        grasp_pos = Point()
        grasp_pos.x = obj.pose.position.x
        grasp_pos.y = obj.pose.position.y
        grasp_pos.z = obj.pose.position.z + 0.05  # Slightly above object to avoid collision
        
        self.get_logger().info(f'Moving to grasp position for {object_name}')
        success = self.move_arm_to_position(grasp_pos)
        if not success:
            return False, f"Failed to move to grasp position for {object_name}"
        
        # Open gripper before grasping
        self.get_logger().info('Opening gripper')
        self.open_gripper()
        time.sleep(0.5)  # Wait for gripper to open
        
        # Close gripper to grasp object
        self.get_logger().info(f'Closing gripper to grasp {object_name}')
        self.close_gripper()
        
        # Wait for grasp completion
        time.sleep(1.0)
        
        # Verify grasp success (simplified check)
        if self.verify_grasp_success():
            self.currently_holding = object_name
            self.is_grasping = True
            self.get_logger().info(f'Successfully grasped {object_name}')
            return True, f"Successfully grasped {object_name}"
        else:
            self.get_logger().warn(f'Grasp may have failed for {object_name}')
            return False, f"Grasp verification failed for {object_name}"

    def execute_place_action(self, action: VLAAction) -> Tuple[bool, str]:
        """
        Execute a place action
        """
        if not self.currently_holding:
            return False, "No object is currently held"
        
        held_object = self.currently_holding
        
        # Extract placement position from action parameters
        params = json.loads(action.params) if action.params else {}
        
        # If position is specified in params, use it
        if 'x' in params and 'y' in params and 'z' in params:
            place_pos = Point()
            place_pos.x = params['x']
            place_pos.y = params['y']
            place_pos.z = params['z']
        else:
            # If object name is given, find its position
            target_object_name = params.get('target_name')
            if target_object_name and target_object_name in self.detected_objects:
                target_obj = self.detected_objects[target_object_name]
                place_pos = Point()
                place_pos.x = target_obj.pose.position.x
                place_pos.y = target_obj.pose.position.y
                place_pos.z = target_obj.pose.position.z + 0.1  # Place slightly above surface
            else:
                return False, "No placement position or target object specified"
        
        # Calculate approach position (above placement location)
        approach_pos = Point()
        approach_pos.x = place_pos.x
        approach_pos.y = place_pos.y
        approach_pos.z = place_pos.z + self.arm_approach_height  # Approach from above
        
        # Move arm to approach position
        self.get_logger().info(f'Moving to approach position for placement at ({place_pos.x}, {place_pos.y}, {place_pos.z})')
        success = self.move_arm_to_position(approach_pos)
        if not success:
            return False, f"Failed to move to approach position for placement"
        
        # Move arm to placement position
        self.get_logger().info(f'Moving to placement position for {held_object}')
        success = self.move_arm_to_position(place_pos)
        if not success:
            return False, f"Failed to move to placement position"
        
        # Release the object
        success, message = self.execute_release_action()
        if not success:
            return False, f"Failed to release {held_object}: {message}"
        
        self.get_logger().info(f'Successfully placed {held_object} at ({place_pos.x}, {place_pos.y}, {place_pos.z})')
        return True, f"Successfully placed {held_object} at target location"

    def execute_move_action(self, action: VLAAction) -> Tuple[bool, str]:
        """
        Execute a move action for arm positioning
        """
        params = json.loads(action.params) if action.params else {}
        
        target_pos = Point()
        target_pos.x = params.get('x', 0.0)
        target_pos.y = params.get('y', 0.0)
        target_pos.z = params.get('z', 0.0)
        
        success = self.move_arm_to_position(target_pos)
        if success:
            return True, f"Successfully moved arm to ({target_pos.x}, {target_pos.y}, {target_pos.z})"
        else:
            return False, f"Failed to move arm to ({target_pos.x}, {target_pos.y}, {target_pos.z})"

    def execute_release_action(self) -> Tuple[bool, str]:
        """
        Execute a release action
        """
        if not self.currently_holding:
            return False, "No object is currently held"
        
        held_object = self.currently_holding
        
        # Open gripper to release object
        self.get_logger().info(f'Opening gripper to release {held_object}')
        self.open_gripper()
        
        # Wait for gripper to open
        time.sleep(0.5)
        
        # Update state
        self.currently_holding = None
        self.is_grasping = False
        
        self.get_logger().info(f'Successfully released {held_object}')
        return True, f"Successfully released {held_object}"

    def move_arm_to_position(self, target_position: Point) -> bool:
        """
        Move the robot arm to a specified position
        """
        # This is a simplified implementation
        # In a real system, you would use inverse kinematics to calculate joint angles
        
        # For demonstration, we'll just publish a dummy joint trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # Example joint names
        
        point = JointTrajectoryPoint()
        # Calculate joint positions for target (simplified)
        # This would require inverse kinematics in a real implementation
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Dummy values
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 2  # Move should complete in 2 seconds
        
        trajectory.points.append(point)
        
        # Publish trajectory command
        self.joint_trajectory_publisher.publish(trajectory)
        
        # Wait for movement to complete (simplified)
        time.sleep(2.0)
        
        # In a real implementation, you would wait for feedback or use action servers
        return True

    def open_gripper(self):
        """
        Open the robot gripper
        """
        cmd = Float64()
        cmd.data = self.gripper_open_position
        self.gripper_command_publisher.publish(cmd)

    def close_gripper(self):
        """
        Close the robot gripper
        """
        cmd = Float64()
        cmd.data = self.gripper_close_position
        self.gripper_command_publisher.publish(cmd)

    def verify_grasp_success(self) -> bool:
        """
        Verify that the grasp was successful
        """
        # In a real implementation, this would check tactile sensors,
        # force/torque sensors, or visual confirmation
        # For now, we'll simulate success based on gripper position and time
        return True  # Simulated success

    def joint_state_callback(self, msg: JointState):
        """
        Handle joint state updates
        """
        for i, name in enumerate(msg.name):
            self.robot_joints[name] = msg.position[i]

    def detected_objects_callback(self, msg: DetectedObjects):
        """
        Update detected objects
        """
        for obj in msg.objects:
            self.detected_objects[obj.name] = obj


class AdvancedManipulationSystem(ManipulationSystem):
    """
    Enhanced manipulation system with advanced features
    """
    
    def __init__(self):
        super().__init__()
        
        # Advanced features
        self.force_sensors_subscriber = self.create_subscription(
            WrenchStamped,
            '/wrench',
            self.force_sensor_callback,
            self.qos_profile
        )
        
        self.tactile_sensors_subscriber = self.create_subscription(
            ContactState,
            '/contact_sensors',
            self.tactile_sensor_callback,
            self.qos_profile
        )
        
        # Grasp planning parameters
        self.grasp_approach_distance = 0.05  # 5cm approach before grasp
        self.grasp_lift_distance = 0.05     # 5cm lift after grasp
        self.pregrasp_orientation = (0, 0, 0)  # Default approach orientation
        
        # Object properties database
        self.object_properties = {
            # Example object properties database
            # In practice, this could be populated from perception or external knowledge
        }
    
    def force_sensor_callback(self, msg):
        """
        Handle force/torque sensor data
        """
        # Process force data for grasp control
        force = msg.wrench.force
        torque = msg.wrench.torque
        
        # Use force data for compliant grasping
        if self.is_grasping and self.currently_holding:
            self.adjust_grip_force(force, torque)
    
    def tactile_sensor_callback(self, msg):
        """
        Handle tactile sensor data
        """
        # Process tactile data for grasp confirmation
        if self.is_grasping and self.currently_holding:
            if self.check_tactile_confirmation(msg):
                self.get_logger().info(f'Tactile confirmation: Successfully holding {self.currently_holding}')
    
    def adjust_grip_force(self, force, torque):
        """
        Adjust grip force based on force/torque sensor feedback
        """
        # Calculate required grip force based on load
        current_force_magnitude = np.sqrt(force.x**2 + force.y**2 + force.z**2)
        
        # For safety, limit grip force
        if current_force_magnitude > self.max_gripper_force:
            self.get_logger().warn('Force limit exceeded, releasing object')
            self.execute_release_action()
    
    def check_tactile_confirmation(self, contact_state) -> bool:
        """
        Check if tactile sensors confirm successful grasp
        """
        # Implementation would check if contact points indicate successful grasp
        # This depends on the specific tactile sensor configuration
        return True  # Simplified implementation

    def execute_grasp_action(self, action: VLAAction) -> Tuple[bool, str]:
        """
        Enhanced grasp action with force control and approach planning
        """
        params = json.loads(action.params) if action.params else {}
        object_name = params.get('object_name')
        
        if not object_name:
            return False, "Object name not specified in grasp action"
        
        # Find object in detected objects
        obj = self.detected_objects.get(object_name)
        if not obj:
            return False, f"Object '{object_name}' not detected in environment"
        
        # Enhance with object properties if available
        obj_properties = self.object_properties.get(object_name, {})
        approach_height = obj_properties.get('height', 0.1) + 0.05  # 5cm clearance
        
        # Calculate approach position with object-specific parameters
        approach_pos = Point()
        approach_pos.x = obj.pose.position.x
        approach_pos.y = obj.pose.position.y
        approach_pos.z = obj.pose.position.z + approach_height
        
        # Check approach path for obstacles
        if not self.check_approach_path_clear(obj.pose.position, approach_pos):
            self.get_logger().warn('Approach path is not clear, attempting alternative approach')
            # You might implement alternative approach planning here
        
        # Move arm to approach position
        success = self.move_arm_to_position(approach_pos)
        if not success:
            return False, f"Failed to move to approach position for {object_name}"
        
        # Calculate precise grasp position
        grasp_pos = Point()
        grasp_pos.x = obj.pose.position.x
        grasp_pos.y = obj.pose.position.y
        grasp_pos.z = obj.pose.position.z + obj_properties.get('height', 0.05) / 2.0  # Middle of object
        
        # Move arm to grasp position
        success = self.move_arm_to_position(grasp_pos)
        if not success:
            return False, f"Failed to move to grasp position for {object_name}"
        
        # Determine appropriate grasp strategy based on object properties
        grasp_strategy = obj_properties.get('grasp_strategy', 'pinch')  # pinch, wrap, etc.
        
        # Execute grasp with appropriate strategy
        success = self.execute_object_specific_grasp(object_name, grasp_strategy, obj_properties)
        
        if success:
            self.currently_holding = object_name
            self.is_grasping = True
            
            # Lift object slightly after grasp
            lift_pos = Point()
            lift_pos.x = grasp_pos.x
            lift_pos.y = grasp_pos.y
            lift_pos.z = grasp_pos.z + self.grasp_lift_distance
            
            self.move_arm_to_position(lift_pos)
            
            return True, f"Successfully grasped {object_name}"
        else:
            return False, f"Grasp failed for {object_name}"

    def execute_object_specific_grasp(self, object_name: str, strategy: str, properties: Dict) -> bool:
        """
        Execute grasp strategy specific to the object
        """
        # Open gripper to pregrasp position
        self.get_logger().info(f'Opening gripper for {object_name}')
        self.open_gripper()
        time.sleep(0.5)
        
        # Apply strategy-specific grasp logic
        if strategy == 'pinch':
            # Pinch grasp is the standard grasp
            self.close_gripper()
        elif strategy == 'wrap':
            # Wrap grasp for cylindrical objects
            self.close_gripper_wrapping()
        elif strategy == 'suction':
            # Suction grasp for flat objects
            return self.execute_suction_grasp()
        else:
            # Default to pinch grasp
            self.close_gripper()
        
        # Wait for grasp completion
        time.sleep(1.0)
        
        # Verify grasp with sensors
        return self.verify_grasp_success_with_sensors(object_name, properties)

    def close_gripper_wrapping(self):
        """
        Close gripper in a wrapping motion for cylindrical objects
        """
        # This would involve a more complex closing motion
        # where fingers wrap around the object
        # For simulation, we'll just close normally
        self.close_gripper()

    def execute_suction_grasp(self) -> bool:
        """
        Execute a suction-based grasp
        """
        # This would activate a suction cup instead of mechanical grippers
        # For simulation, return success
        return True

    def verify_grasp_success_with_sensors(self, object_name: str, properties: Dict) -> bool:
        """
        Verify grasp success using all available sensors
        """
        # This would combine visual, tactile, and force feedback
        # For now, we'll use a combination of simulated checks
        
        # Get object weight from properties
        object_weight = properties.get('weight', 0.1)  # Default 100g
        expected_force = object_weight * 9.81  # Weight in Newtons
        
        # Simulate sensor checks
        visual_confirmation = True  # Assume visual confirmation
        tactile_confirmation = True  # Assume tactile confirmation
        
        return visual_confirmation and tactile_confirmation

    def check_approach_path_clear(self, object_pos: Point, approach_pos: Point) -> bool:
        """
        Check if the approach path to an object is clear of obstacles
        """
        # This would check the path between approach_pos and object_pos
        # against the known map or temporary obstacles
        # For simulation, return True
        return True


# Grasp Planning Module
class GraspPlanner:
    """
    Module for planning appropriate grasps based on object properties
    """
    
    def __init__(self):
        # Predefined grasp types
        self.grasp_types = {
            'pinch': {
                'description': 'Standard pinch grasp for small objects',
                'finger_count': 2,
                'approach_angle_range': (-45, 45)
            },
            'wrap': {
                'description': 'Wrap grasp for cylindrical objects',
                'finger_count': 3,
                'approach_angle_range': (-180, 180)
            },
            'suction': {
                'description': 'Suction-based grasp for flat objects',
                'finger_count': 0,
                'approach_angle_range': (-180, 180)
            }
        }
    
    def plan_grasp(self, object_info: Object) -> str:
        """
        Plan the best grasp type for the given object
        """
        # Analyze object properties to determine best grasp
        shape = self.estimate_shape(object_info)
        
        if shape == 'cylindrical':
            return 'wrap'
        elif shape == 'flat':
            return 'suction'
        else:
            # Default to pinch for unknown shapes
            return 'pinch'
    
    def estimate_shape(self, object_info: Object) -> str:
        """
        Estimate object shape from perception data
        """
        # In a real implementation, this would analyze the object's dimensions and features
        # from 3D point cloud or image data
        
        # For simulation, we'll use the object's name as a hint
        name = object_info.name.lower()
        
        if any(keyword in name for keyword in ['cup', 'bottle', 'can', 'cylinder']):
            return 'cylindrical'
        elif any(keyword in name for keyword in ['book', 'paper', 'cardboard', 'plate']):
            return 'flat'
        else:
            return 'unknown'


# Force Control Module
class ForceController:
    """
    Controller for managing forces during manipulation
    """
    
    def __init__(self, max_force=100.0):
        self.max_force = max_force
        self.current_force_limit = max_force
        self.safety_margin = 0.1  # 10% safety margin
    
    def adjust_force_for_object(self, object_weight: float, object_fragility: str = 'normal'):
        """
        Adjust force limits based on object properties
        """
        # Calculate appropriate force based on object weight
        base_force = object_weight * 9.81  # Weight in Newtons
        
        # Adjust based on fragility
        fragility_multipliers = {
            'fragile': 0.5,
            'normal': 1.0,
            'durable': 2.0
        }
        
        multiplier = fragility_multipliers.get(object_fragility, 1.0)
        
        # Set force with safety margin
        self.current_force_limit = min(
            base_force * multiplier * (1 + self.safety_margin),
            self.max_force
        )
        
        return self.current_force_limit
    
    def check_force_limit(self, current_force: float) -> bool:
        """
        Check if current force is within safe limits
        """
        return current_force <= self.current_force_limit


# Integration with Perception for Object Properties
class ManipulationWithPerceptionIntegration(AdvancedManipulationSystem):
    """
    Manipulation system with tight integration to perception
    """
    
    def __init__(self):
        super().__init__()
        
        # Grasp planner instance
        self.grasp_planner = GraspPlanner()
        self.force_controller = ForceController()
        
        # Subscribe to more detailed object information
        self.detailed_object_subscriber = self.create_subscription(
            ObjectProperties,
            '/object_properties',
            self.object_properties_callback,
            self.qos_profile
        )
        
        self.object_properties = {}  # Store detailed properties
    
    def object_properties_callback(self, msg):
        """
        Handle detailed object properties from advanced perception
        """
        self.object_properties[msg.name] = {
            'shape': msg.shape,
            'weight': msg.weight,
            'fragility': msg.fragility,
            'dimensions': msg.dimensions,
            'material': msg.material
        }
    
    def execute_grasp_action(self, action: VLAAction) -> Tuple[bool, str]:
        """
        Enhanced grasp with object property integration
        """
        params = json.loads(action.params) if action.params else {}
        object_name = params.get('object_name')
        
        if not object_name:
            return False, "Object name not specified in grasp action"
        
        # Get object from detected objects
        obj = self.detected_objects.get(object_name)
        if not obj:
            return False, f"Object '{object_name}' not detected in environment"
        
        # Get detailed properties if available
        obj_properties = self.object_properties.get(object_name, {})
        
        # Plan grasp based on object properties
        if obj_properties:
            grasp_strategy = self.grasp_planner.plan_grasp(obj)
        else:
            # Use basic properties from perception
            basic_obj = Object()
            basic_obj.name = object_name
            grasp_strategy = self.grasp_planner.plan_grasp(basic_obj)
        
        # Adjust force limits based on object properties
        object_weight = obj_properties.get('weight', 0.1)  # Default 100g
        object_fragility = obj_properties.get('fragility', 'normal')
        required_force = self.force_controller.adjust_force_for_object(object_weight, object_fragility)
        
        self.get_logger().info(f'Using {grasp_strategy} grasp for {object_name}, required force: {required_force:.2f}N')
        
        # Use the parent implementation but with our strategy
        return super().execute_grasp_action(action)


def main(args=None):
    """
    Main function to run the manipulation system
    """
    rclpy.init(args=args)
    
    manipulator = ManipulationWithPerceptionIntegration()
    
    try:
        rclpy.spin(manipulator)
    except KeyboardInterrupt:
        manipulator.get_logger().info('Manipulation System interrupted by user')
    finally:
        manipulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Grasp Planning and Execution

```python
# grasp_planning.py
import numpy as np
import math
from geometry_msgs.msg import Point, Pose, Vector3
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum


class GraspType(Enum):
    """Enumeration of grasp types"""
    PINCH = "pinch"
    WRAP = "wrap"
    SUCTION = "suction"
    PINCH_TIP = "pinch_tip"
    LATERAL = "lateral"


@dataclass
class GraspPose:
    """Represents a potential grasp pose"""
    position: Point
    orientation: Tuple[float, float, float, float]  # quaternion (x, y, z, w)
    approach_direction: Vector3
    grasp_type: GraspType
    score: float  # Quality score for this grasp


@dataclass
class ObjectProperties:
    """Represents properties of an object for grasp planning"""
    name: str
    position: Point
    dimensions: Vector3  # width, height, depth
    shape: str  # 'cylindrical', 'rectangular', 'spherical', etc.
    weight: float  # in kg
    fragility: str  # 'fragile', 'normal', 'durable'
    material: str  # 'metal', 'plastic', 'glass', etc.
    surface_texture: str  # 'smooth', 'rough', 'textured'


class GraspPlanner:
    """
    Advanced grasp planner that generates potential grasp poses for objects
    """
    
    def __init__(self):
        self.approach_distance = 0.05  # 5cm approach distance
        self.grasp_clearance = 0.02    # 2cm clearance around object
    
    def generate_grasps(self, obj_props: ObjectProperties) -> List[GraspPose]:
        """
        Generate potential grasp poses for an object
        """
        grasps = []
        
        if obj_props.shape == 'cylindrical':
            grasps.extend(self._generate_cylindrical_grasps(obj_props))
        elif obj_props.shape == 'rectangular':
            grasps.extend(self._generate_rectangular_grasps(obj_props))
        elif obj_props.shape == 'spherical':
            grasps.extend(self._generate_spherical_grasps(obj_props))
        else:
            # Default to general grasps
            grasps.extend(self._generate_general_grasps(obj_props))
        
        # Score and filter grasps
        scored_grasps = [(g, self._score_grasp(g, obj_props)) for g in grasps]
        scored_grasps.sort(key=lambda x: x[1], reverse=True)  # Sort by score (highest first)
        
        # Return top grasps
        top_grasps = [g for g, score in scored_grasps[:5]]  # Return top 5 grasps
        
        return top_grasps
    
    def _generate_cylindrical_grasps(self, obj_props: ObjectProperties) -> List[GraspPose]:
        """
        Generate grasps for cylindrical objects
        """
        grasps = []
        
        # Calculate grasp positions around the cylinder
        steps = 8  # Number of grasp positions around the cylinder
        for i in range(steps):
            angle = (2 * math.pi * i) / steps
            
            # Side grasp
            grasp_pos = Point()
            grasp_pos.x = obj_props.position.x + (obj_props.dimensions.x / 2 + self.grasp_clearance) * math.cos(angle)
            grasp_pos.y = obj_props.position.y + (obj_props.dimensions.x / 2 + self.grasp_clearance) * math.sin(angle)
            grasp_pos.z = obj_props.position.z  # At the center height
            
            # Calculate approach direction (from cylinder center outward)
            approach_dir = Vector3()
            approach_dir.x = math.cos(angle)
            approach_dir.y = math.sin(angle)
            approach_dir.z = 0.0
            
            # Calculate orientation for side grasp
            # For side grasps, the gripper should approach perpendicular to the surface
            roll = 0.0
            pitch = 0.0  # Gripper parallel to ground for side grasp
            yaw = angle  # Facing toward the cylinder center
            quat = self._euler_to_quaternion(roll, pitch, yaw)
            
            grasp = GraspPose(
                position=grasp_pos,
                orientation=quat,
                approach_direction=approach_dir,
                grasp_type=GraspType.WRAP if obj_props.dimensions.x > 0.05 else GraspType.PINCH,
                score=0.0  # Will be scored later
            )
            grasps.append(grasp)
        
        # Top grasp for cylindrical objects
        top_grasp_pos = Point()
        top_grasp_pos.x = obj_props.position.x
        top_grasp_pos.y = obj_props.position.y
        top_grasp_pos.z = obj_props.position.z + obj_props.dimensions.y / 2 + self.grasp_clearance
        
        # Approach from above
        top_approach_dir = Vector3()
        top_approach_dir.x = 0.0
        top_approach_dir.y = 0.0
        top_approach_dir.z = -1.0  # Approaching from above
        
        # For top grasp, gripper should face down
        top_quat = self._euler_to_quaternion(0.0, math.pi/2, 0.0)  # 90-degree pitch for downward grasp
        
        top_grasp = GraspPose(
            position=top_grasp_pos,
            orientation=top_quat,
            approach_direction=top_approach_dir,
            grasp_type=GraspType.PINCH_TIP,
            score=0.0
        )
        grasps.append(top_grasp)
        
        return grasps
    
    def _generate_rectangular_grasps(self, obj_props: ObjectProperties) -> List[GraspPose]:
        """
        Generate grasps for rectangular objects
        """
        grasps = []
        
        # Corner grasps
        corners = [
            (obj_props.dimensions.x / 2, obj_props.dimensions.y / 2),
            (obj_props.dimensions.x / 2, -obj_props.dimensions.y / 2),
            (-obj_props.dimensions.x / 2, obj_props.dimensions.y / 2),
            (-obj_props.dimensions.x / 2, -obj_props.dimensions.y / 2)
        ]
        
        for dx, dy in corners:
            grasp_pos = Point()
            grasp_pos.x = obj_props.position.x + dx
            grasp_pos.y = obj_props.position.y + dy
            grasp_pos.z = obj_props.position.z  # Mid-height
            
            # Approach from the side
            approach_dir = Vector3()
            approach_dir.x = 1.0 if dx > 0 else -1.0
            approach_dir.y = 0.0
            approach_dir.z = 0.0
            
            # Orientation for corner grasp
            quat = self._euler_to_quaternion(0.0, 0.0, math.pi/2 if dy > 0 else -math.pi/2)
            
            grasp = GraspPose(
                position=grasp_pos,
                orientation=quat,
                approach_direction=approach_dir,
                grasp_type=GraspType.PINCH,
                score=0.0
            )
            grasps.append(grasp)
        
        # Mid-side grasps
        mid_sides = [
            (obj_props.dimensions.x / 2, 0),
            (-obj_props.dimensions.x / 2, 0),
            (0, obj_props.dimensions.y / 2),
            (0, -obj_props.dimensions.y / 2)
        ]
        
        for dx, dy in mid_sides:
            grasp_pos = Point()
            grasp_pos.x = obj_props.position.x + dx
            grasp_pos.y = obj_props.position.y + dy
            grasp_pos.z = obj_props.position.z
            
            approach_dir = Vector3()
            approach_dir.x = 1.0 if dx > 0 else -1.0 if dx < 0 else 0.0
            approach_dir.y = 1.0 if dy > 0 else -1.0 if dy < 0 else 0.0
            approach_dir.z = 0.0
            
            # Calculate appropriate orientation
            if dx != 0:
                quat = self._euler_to_quaternion(0.0, 0.0, 0.0 if dx > 0 else math.pi)
            else:
                quat = self._euler_to_quaternion(0.0, 0.0, math.pi/2 if dy > 0 else -math.pi/2)
            
            grasp = GraspPose(
                position=grasp_pos,
                orientation=quat,
                approach_direction=approach_dir,
                grasp_type=GraspType.PINCH,
                score=0.0
            )
            grasps.append(grasp)
        
        # Top grasp
        top_grasp_pos = Point()
        top_grasp_pos.x = obj_props.position.x
        top_grasp_pos.y = obj_props.position.y
        top_grasp_pos.z = obj_props.position.z + obj_props.dimensions.z / 2 + self.grasp_clearance
        
        top_approach_dir = Vector3()
        top_approach_dir.x = 0.0
        top_approach_dir.y = 0.0
        top_approach_dir.z = -1.0
        
        top_quat = self._euler_to_quaternion(0.0, math.pi/2, 0.0)
        
        top_grasp = GraspPose(
            position=top_grasp_pos,
            orientation=top_quat,
            approach_direction=top_approach_dir,
            grasp_type=GraspType.PINCH_TIP,
            score=0.0
        )
        grasps.append(top_grasp)
        
        return grasps
    
    def _generate_spherical_grasps(self, obj_props: ObjectProperties) -> List[GraspPose]:
        """
        Generate grasps for spherical objects
        """
        grasps = []
        
        # Generate grasps around the sphere
        for i in range(8):
            for j in range(4):
                # Spherical coordinates
                theta = (2 * math.pi * i) / 8  # Azimuthal angle
                phi = (math.pi * j) / 4  # Polar angle
                
                # Position on sphere surface plus clearance
                grasp_x = obj_props.position.x + (obj_props.dimensions.x / 2 + self.grasp_clearance) * math.sin(phi) * math.cos(theta)
                grasp_y = obj_props.position.y + (obj_props.dimensions.x / 2 + self.grasp_clearance) * math.sin(phi) * math.sin(theta)
                grasp_z = obj_props.position.z + (obj_props.dimensions.x / 2 + self.grasp_clearance) * math.cos(phi)
                
                grasp_pos = Point()
                grasp_pos.x = grasp_x
                grasp_pos.y = grasp_y
                grasp_pos.z = grasp_z
                
                # Approach direction (from sphere center to surface point)
                approach_dir = Vector3()
                approach_dir.x = math.sin(phi) * math.cos(theta)
                approach_dir.y = math.sin(phi) * math.sin(theta)
                approach_dir.z = math.cos(phi)
                
                # Orientation - gripper should face toward center
                roll = 0.0
                pitch = phi - math.pi/2  # Adjust pitch to grip the sphere
                yaw = theta  # Rotate around vertical axis
                quat = self._euler_to_quaternion(roll, pitch, yaw)
                
                grasp = GraspPose(
                    position=grasp_pos,
                    orientation=quat,
                    approach_direction=approach_dir,
                    grasp_type=GraspType.PINCH,
                    score=0.0
                )
                grasps.append(grasp)
        
        return grasps
    
    def _generate_general_grasps(self, obj_props: ObjectProperties) -> List[GraspPose]:
        """
        Generate general grasps for unknown object shapes
        """
        grasps = []
        
        # Top grasp (safest for most objects)
        top_grasp_pos = Point()
        top_grasp_pos.x = obj_props.position.x
        top_grasp_pos.y = obj_props.position.y
        top_grasp_pos.z = obj_props.position.z + obj_props.dimensions.z / 2 + self.grasp_clearance
        
        top_approach_dir = Vector3()
        top_approach_dir.x = 0.0
        top_approach_dir.y = 0.0
        top_approach_dir.z = -1.0
        
        top_quat = self._euler_to_quaternion(0.0, math.pi/2, 0.0)
        
        top_grasp = GraspPose(
            position=top_grasp_pos,
            orientation=top_quat,
            approach_direction=top_approach_dir,
            grasp_type=GraspType.PINCH_TIP,
            score=0.0
        )
        grasps.append(top_grasp)
        
        # Side grasp from multiple directions
        for angle in [0, math.pi/2, math.pi, -math.pi/2]:
            grasp_pos = Point()
            grasp_pos.x = obj_props.position.x + (obj_props.dimensions.x / 2 + self.grasp_clearance) * math.cos(angle)
            grasp_pos.y = obj_props.position.y + (obj_props.dimensions.y / 2 + self.grasp_clearance) * math.sin(angle)
            grasp_pos.z = obj_props.position.z  # Mid height
            
            approach_dir = Vector3()
            approach_dir.x = math.cos(angle)
            approach_dir.y = math.sin(angle)
            approach_dir.z = 0.0
            
            quat = self._euler_to_quaternion(0.0, 0.0, angle + math.pi/2)
            
            grasp = GraspPose(
                position=grasp_pos,
                orientation=quat,
                approach_direction=approach_dir,
                grasp_type=GraspType.PINCH,
                score=0.0
            )
            grasps.append(grasp)
        
        return grasps
    
    def _score_grasp(self, grasp: GraspPose, obj_props: ObjectProperties) -> float:
        """
        Score a grasp based on various factors
        """
        score = 1.0  # Base score
        
        # Adjust score based on object fragility
        if obj_props.fragility == 'fragile':
            if grasp.grasp_type == GraspType.PINCH:
                score *= 0.8  # Pinch grasps might be too aggressive for fragile items
        elif obj_props.fragility == 'durable':
            score *= 1.1  # Durable items can handle more aggressive grasps
        
        # Adjust based on grasp stability
        if grasp.grasp_type in [GraspType.WRAP, GraspType.SUCTION]:
            score *= 1.2  # More stable grasp types
        elif grasp.grasp_type == GraspType.LATERAL:
            score *= 0.7  # Lateral grasps are less stable
        
        # Adjust based on approach direction (top-down approaches generally more stable)
        if abs(grasp.approach_direction.z) > 0.7:  # Approaching from top or bottom
            score *= 1.1
        
        # Adjust based on object weight
        if obj_props.weight > 1.0:  # Heavy objects
            # Prefer stable grasp types
            if grasp.grasp_type not in [GraspType.WRAP, GraspType.SUCTION]:
                score *= 0.8
        
        # Distance from object center (closer to center is generally better)
        dist_from_center = math.sqrt(
            (grasp.position.x - obj_props.position.x)**2 +
            (grasp.position.y - obj_props.position.y)**2 +
            (grasp.position.z - obj_props.position.z)**2
        )
        
        # Normalize by object dimensions to make it relative
        avg_dimension = (obj_props.dimensions.x + obj_props.dimensions.y + obj_props.dimensions.z) / 3
        relative_dist = dist_from_center / avg_dimension
        
        if relative_dist > 1.5:  # Too far from center
            score *= 0.7
        elif relative_dist < 0.3:  # Very close to center (might not be reachable)
            score *= 0.8
        
        return score
    
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert Euler angles to quaternion
        """
        # Calculate trigonometric values
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        
        # Calculate quaternion components
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)


# Example usage of the grasp planner
def example_usage():
    # Create object properties
    obj_props = ObjectProperties(
        name="red_cup",
        position=Point(x=1.0, y=1.0, z=0.0),
        dimensions=Vector3(x=0.08, y=0.08, z=0.1),  # 8cm x 8cm x 10cm
        shape="cylindrical",
        weight=0.3,  # 300g
        fragility="normal",
        material="plastic",
        surface_texture="smooth"
    )
    
    # Create grasp planner
    planner = GraspPlanner()
    
    # Generate grasps
    grasps = planner.generate_grasps(obj_props)
    
    print(f"Generated {len(grasps)} grasp candidates for {obj_props.name}:")
    for i, grasp in enumerate(grasps):
        print(f"  {i+1}. Type: {grasp.grasp_type.value}, Score: {grasp.score:.2f}")
        print(f"      Position: ({grasp.position.x:.2f}, {grasp.position.y:.2f}, {grasp.position.z:.2f})")
        print(f"      Orientation: ({grasp.orientation[0]:.2f}, {grasp.orientation[1]:.2f}, {grasp.orientation[2]:.2f}, {grasp.orientation[3]:.2f})")


if __name__ == "__main__":
    example_usage()
```

### Manipulation Safety and Validation

```python
# manipulation_safety.py
from geometry_msgs.msg import Point, Pose, Wrench
from sensor_msgs.msg import JointState
from typing import Dict, List, Optional
import numpy as np


class ManipulationSafetyValidator:
    """
    Validates manipulation actions for safety before execution
    """
    
    def __init__(self, robot_params: Dict):
        # Robot-specific parameters
        self.max_payload = robot_params.get('max_payload', 1.0)  # kg
        self.max_velocity = robot_params.get('max_velocity', 1.0)  # m/s
        self.max_force = robot_params.get('max_force', 100.0)  # N
        self.reachable_workspace = robot_params.get('workspace', {
            'min_x': -1.0, 'max_x': 1.0,
            'min_y': -1.0, 'max_y': 1.0,
            'min_z': 0.0, 'max_z': 1.5
        })
        self.joint_limits = robot_params.get('joint_limits', {})
        
    def validate_grasp_action(self, object_props: ObjectProperties, current_joint_state: JointState) -> Tuple[bool, str]:
        """
        Validate a grasp action for safety
        """
        # Check if object is too heavy
        if object_props.weight > self.max_payload:
            return False, f"Object weighs {object_props.weight}kg, exceeds max payload of {self.max_payload}kg"
        
        # Check if object is in reachable workspace
        if not self._is_in_workspace(object_props.position):
            return False, f"Object at ({object_props.position.x}, {object_props.position.y}, {object_props.position.z}) is outside reachable workspace"
        
        # Check if grasp would cause joint limit violations
        if not self._validate_joint_limits_for_grasp(object_props, current_joint_state):
            return False, "Grasp action would cause joint limit violations"
        
        # Check if object is fragile and appropriate grasp type is used
        if object_props.fragility == 'fragile' and not self._is_safe_grasp_type_for_fragile(object_props):
            return False, "Unsafe grasp type for fragile object"
        
        return True, "Grasp action is safe"
    
    def validate_place_action(self, placement_position: Point, currently_holding: bool) -> Tuple[bool, str]:
        """
        Validate a place action for safety
        """
        # Check if robot is actually holding something
        if not currently_holding:
            return False, "Cannot place object - robot is not holding anything"
        
        # Check if placement position is in workspace
        if not self._is_in_workspace(placement_position):
            return False, f"Placement position ({placement_position.x}, {placement_position.y}, {placement_position.z}) is outside reachable workspace"
        
        # Check if placement position is stable (e.g., not on edge of surface)
        if not self._is_stable_placement(placement_position):
            return False, "Placement position is not stable"
        
        return True, "Place action is safe"
    
    def _is_in_workspace(self, position: Point) -> bool:
        """
        Check if a position is within the robot's workspace
        """
        ws = self.reachable_workspace
        return (ws['min_x'] <= position.x <= ws['max_x'] and
                ws['min_y'] <= position.y <= ws['max_y'] and
                ws['min_z'] <= position.z <= ws['max_z'])
    
    def _validate_joint_limits_for_grasp(self, object_props: ObjectProperties, joint_state: JointState) -> bool:
        """
        Check if grasping this object would require joint movements beyond limits
        """
        # This is a simplified check - a real implementation would use inverse kinematics
        # to determine required joint angles for the grasp
        return True
    
    def _is_safe_grasp_type_for_fragile(self, object_props: ObjectProperties) -> bool:
        """
        Check if the grasp type is safe for fragile objects
        """
        # For fragile objects, avoid pinch grasps with high force
        # This would be integrated with grasp planning
        return True
    
    def _is_stable_placement(self, position: Point) -> bool:
        """
        Check if a placement position is stable
        """
        # Check if position is on a known stable surface
        # This would be determined from map or perception data
        return True


# Force control and compliance
class ForceController:
    """
    Controller for managing forces during manipulation
    """
    
    def __init__(self):
        self.max_force_thresholds = {
            'grasp': 50.0,    # N for grasping
            'contact': 10.0,  # N for contact detection
            'manipulation': 20.0  # N for manipulation
        }
        self.compliance_mode = False
        self.current_force_limits = self.max_force_thresholds.copy()
    
    def set_compliance(self, enabled: bool):
        """
        Enable or disable compliant control
        """
        self.compliance_mode = enabled
    
    def adjust_force_for_object(self, object_props: ObjectProperties):
        """
        Adjust force limits based on object properties
        """
        base_factor = 1.0
        
        # Adjust for fragility
        if object_props.fragility == 'fragile':
            base_factor = 0.5
        elif object_props.fragility == 'durable':
            base_factor = 1.5
        
        # Adjust for weight
        if object_props.weight < 0.1:  # Very light
            base_factor *= 0.7
        elif object_props.weight > 2.0:  # Very heavy
            base_factor *= 1.2
        
        # Update force limits
        for action_type in self.current_force_limits:
            self.current_force_limits[action_type] = self.max_force_thresholds[action_type] * base_factor
    
    def check_force_limits(self, current_forces: Wrench, action_type: str = 'grasp') -> bool:
        """
        Check if current forces are within safe limits
        """
        if action_type not in self.current_force_limits:
            return False
        
        force_magnitude = np.sqrt(
            current_forces.force.x**2 + 
            current_forces.force.y**2 + 
            current_forces.force.z**2
        )
        
        return force_magnitude <= self.current_force_limits[action_type]
    
    def get_compliance_adjustment(self, current_force: Wrench, target_force: Wrench) -> float:
        """
        Calculate compliance adjustment based on force feedback
        """
        # Calculate force error
        error_x = target_force.force.x - current_force.force.x
        error_y = target_force.force.y - current_force.force.y
        error_z = target_force.force.z - current_force.force.z
        
        # Simple proportional control
        kp = 0.1  # Proportional gain
        adjustment = kp * np.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        return min(adjustment, 0.1)  # Limit adjustment to 10%


# Integration with the main manipulation system
class SafeManipulationSystem(ManipulationWithPerceptionIntegration):
    """
    Manipulation system with integrated safety validation
    """
    
    def __init__(self):
        super().__init__()
        
        # Initialize safety validator
        robot_params = {
            'max_payload': 3.0,
            'workspace': {
                'min_x': -1.0, 'max_x': 1.0,
                'min_y': -1.0, 'max_y': 1.0,
                'min_z': 0.1, 'max_z': 1.5
            }
        }
        self.safety_validator = ManipulationSafetyValidator(robot_params)
        self.force_controller = ForceController()
        
        # Current state tracking
        self.current_joint_state = None
    
    def execute_action_callback(self, request, response):
        """
        Enhanced action execution with safety validation
        """
        action = request.action
        
        # Validate action before execution
        is_valid, validation_msg = self.validate_action(action)
        if not is_valid:
            response.success = False
            response.message = f"Safety validation failed: {validation_msg}"
            response.error_code = "SAFETY_VIOLATION"
            
            # Publish safety violation
            safety_msg = String()
            safety_msg.data = validation_msg
            self.safety_publisher.publish(safety_msg)
            
            return response
        
        # Check force limits before execution
        if self.force_sensors_available:
            current_forces = self.get_current_forces()
            if not self.force_controller.check_force_limits(current_forces, action.type):
                response.success = False
                response.message = "Force limits would be exceeded"
                response.error_code = "FORCE_LIMIT_EXCEEDED"
                return response
        
        # Execute the validated action
        return super().execute_action_callback(request, response)
    
    def validate_action(self, action: VLAAction) -> Tuple[bool, str]:
        """
        Validate an action before execution
        """
        if action.type == 'grasp':
            params = json.loads(action.params) if action.params else {}
            object_name = params.get('object_name')
            
            if object_name and object_name in self.object_properties:
                obj_props = self.object_properties[object_name]
                
                # Also need current joint state for full validation
                if self.current_joint_state:
                    return self.safety_validator.validate_grasp_action(obj_props, self.current_joint_state)
                else:
                    return False, "Cannot validate grasp without current joint state"
            else:
                return False, f"Object properties for '{object_name}' not available for validation"
        
        elif action.type == 'place_at':
            params = json.loads(action.params) if action.params else {}
            
            # Create placement position
            placement_pos = Point()
            placement_pos.x = params.get('x', 0.0)
            placement_pos.y = params.get('y', 0.0)
            placement_pos.z = params.get('z', 0.0)
            
            return self.safety_validator.validate_place_action(placement_pos, bool(self.currently_holding))
        
        # For other action types, assume they are valid
        return True, "Action passed validation"
    
    def joint_state_callback(self, msg: JointState):
        """
        Update current joint state for validation
        """
        super().joint_state_callback(msg)
        self.current_joint_state = msg
```

## Testing the Manipulation System

```python
# test_manipulation_system.py
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String
from vla_msgs.srv import ExecuteAction
from vla_msgs.msg import VLAAction
import json


class TestManipulationSystem(unittest.TestCase):
    """
    Test class for the manipulation system
    """
    
    def setUp(self):
        rclpy.init()
        self.node = Node('test_manipulation_system')
        
        # Create client for action execution service
        self.action_client = self.node.create_client(ExecuteAction, 'execute_action')
        
        # Wait for service to be available
        while not self.action_client.wait_for_service(timeout_sec=1.0):
            print('Action execution service not available, waiting again...')
    
    def test_grasp_action(self):
        """
        Test grasp action execution
        """
        # Create a grasp action
        action = VLAAction()
        action.type = 'grasp'
        action.params = json.dumps({'object_name': 'test_cup'})
        action.description = 'Grasp test cup'
        
        # Create the service request
        request = ExecuteAction.Request()
        request.action = action
        
        # Call the service
        future = self.action_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        # Check response
        response = future.result()
        self.assertIsNotNone(response, "Action execution service did not respond")
        print(f"Grasp action result: {response.success}, message: {response.message}")
    
    def test_place_action(self):
        """
        Test place action execution
        """
        # Create a place action
        action = VLAAction()
        action.type = 'place_at'
        action.params = json.dumps({'x': 1.0, 'y': 1.0, 'z': 0.1})
        action.description = 'Place at (1.0, 1.0, 0.1)'
        
        # Create the service request
        request = ExecuteAction.Request()
        request.action = action
        
        # Call the service
        future = self.action_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        # Check response
        response = future.result()
        self.assertIsNotNone(response, "Action execution service did not respond")
        print(f"Place action result: {response.success}, message: {response.message}")
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
```

## Launch Configuration

```python
# launch/manipulation_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    max_payload = LaunchConfiguration('max_payload', default='3.0')
    workspace_limits = LaunchConfiguration('workspace_limits', 
                                          default='{"min_x": -1.0, "max_x": 1.0, "min_y": -1.0, "max_y": 1.0, "min_z": 0.1, "max_z": 1.5}')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'max_payload',
            default_value='3.0',
            description='Maximum payload in kg'
        ),
        DeclareLaunchArgument(
            'workspace_limits',
            default_value='{"min_x": -1.0, "max_x": 1.0, "min_y": -1.0, "max_y": 1.0, "min_z": 0.1, "max_z": 1.5}',
            description='Robot workspace limits as JSON'
        ),
        
        # Manipulation system node
        Node(
            package='vla_examples',
            executable='manipulation_system',
            name='manipulation_system',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'max_payload': max_payload},
                {'workspace_limits': workspace_limits}
            ],
            remappings=[
                ('/joint_states', '/joint_states'),
                ('/wrench', '/wrench'),
            ],
            output='screen'
        )
    ])
```

## Conclusion

The manipulation system implemented here provides comprehensive capabilities for object interaction in the VLA system. Key features include:

1. **Grasp Planning**: Generates appropriate grasp poses based on object properties
2. **Force Control**: Manages grip forces and compliance for safe manipulation
3. **Safety Validation**: Validates actions before execution to prevent damage
4. **Multi-Type Support**: Handles different grasp types (pinch, wrap, suction)
5. **Integration Ready**: Designed to work with perception and path planning systems

The system is designed to be a core component of the VLA architecture, enabling the robot to physically interact with objects in response to natural language commands processed by the cognitive planner.
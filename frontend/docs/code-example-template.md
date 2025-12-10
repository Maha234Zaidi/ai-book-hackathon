# Template for Code Examples

## Code Example Standards Template

This document provides templates and standards for creating code examples in the ROS 2 module documentation.

## Python Code Example Template

```python
import rclpy
from rclpy.node import Node
# Import other required modules

class [DescriptiveClassName]Node(Node):
    def __init__(self):
        # Initialize the node with a descriptive name
        super().__init__('[node_name]')
        
        # Declare parameters with default values if applicable
        # self.declare_parameter('param_name', default_value)
        
        # Initialize class variables
        # self.variable = initial_value
        
        # Create subscribers
        # self.subscription = self.create_subscription(
        #     MessageType,
        #     'topic_name',
        #     self.callback_function,
        #     qos_profile_or_depth
        # )
        
        # Create publishers
        # self.publisher = self.create_publisher(
        #     MessageType,
        #     'topic_name',
        #     qos_profile_or_depth
        # )
        
        # Create timers if needed
        # self.timer = self.create_timer(
        #     timer_period,  # seconds
        #     self.timer_callback
        # )
        
        # Create services if needed
        # self.srv = self.create_service(
        #     ServiceType,
        #     'service_name',
        #     self.service_callback
        # )
        
        # Log initialization
        self.get_logger().info('[Node name] initialized')

    def callback_function(self, msg):
        """Callback function for processing messages.
        
        Args:
            msg: The received message of appropriate type
        """
        # Process the message
        # self.get_logger().info(f'Received: {msg.data}')
        
        # Add your processing logic here
        pass

    def timer_callback(self):
        """Timer callback function."""
        # Add timer-based logic here
        pass

    def service_callback(self, request, response):
        """Service callback function.
        
        Args:
            request: The service request object
            response: The service response object to be filled
        
        Returns:
            response: The filled response object
        """
        # Process the request
        # Fill the response
        return response

def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    
    node = [DescriptiveClassName]Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## URDF Example Template

```xml
<?xml version="1.0"?>
<robot name="[robot_name]" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <[shape] [dimensions_parameter]="[value]"/>
      </geometry>
      <material name="blue"/>
    </visual>
    
    <collision>
      <geometry>
        <[shape] [dimensions_parameter]="[value]"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="[mass_value]"/>
      <inertia 
        ixx="[ixx_value]" ixy="[ixy_value]" ixz="[ixz_value]"
        iyy="[iyy_value]" iyz="[iyz_value]" izz="[izz_value]"/>
    </inertial>
  </link>
  
  <!-- Additional links and joints -->
  <joint name="[joint_name]" type="[joint_type]">
    <parent link="[parent_link]"/>
    <child link="[child_link]"/>
    <origin xyz="[x] [y] [z]" rpy="[roll] [pitch] [yaw]"/>
    <axis xyz="[x] [y] [z]"/>
    <limit lower="[lower_limit]" upper="[upper_limit]" effort="[effort_limit]" velocity="[velocity_limit]"/>
  </joint>
  
  <!-- Gazebo-specific elements if needed -->
  <gazebo reference="[link_name]">
    <material>Gazebo/Blue</material>
  </gazebo>
</robot>
```

## YAML Example Template

```yaml
# Configuration file for [system/component name]
# Description: [Brief description of what this file configures]

# Node-specific parameters
node_name:
  ros__parameters:
    # Parameter group for clarity
    parameter_group:
      param1: value1
      param2: value2
      nested_params:
        param3: value3
        param4: value4

# Alternative format for parameters
parameter_namespace:
  ros__parameters:
    # List example
    list_param: [item1, item2, item3]
    # Dictionary example
    dict_param:
      key1: value1
      key2: value2
    # Boolean example
    flag_param: true
    # Numeric example
    num_param: 42
```

## Launch File Template

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for [system name]."""
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Create nodes
    example_node = Node(
        package='package_name',
        executable='executable_name',
        name='node_name',
        parameters=[
            {'param1': 'value1'},
            {'param2': 42}
        ],
        remappings=[
            ('original_topic', 'remapped_topic')
        ],
        arguments=['arg1', 'arg2'],
        output='screen'
    )
    
    # Return the launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(example_node)
    
    return ld
```

## Code Block Formatting for Documentation

When including code in documentation markdown files, use these formatting guidelines:

### Python Code Block
```markdown
​```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # More code here
​```
```

### URDF/XML Code Block
```markdown
​```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </visual>
</link>
​```
```

### YAML Code Block
```markdown
​```yaml
node_name:
  ros__parameters:
    param: value
​```
```

### Command Line Examples
```markdown
​```bash
# Terminal command
ros2 run package_name executable_name
​```
```

## Best Practices for Code Examples

### 1. Clarity and Completeness
- Include all necessary imports
- Add meaningful comments explaining key concepts
- Use descriptive variable and function names
- Include error handling where appropriate

### 2. Educational Value
- Add comments that explain not just what the code does, but why
- Include references to relevant documentation
- Use simple, clear examples before complex ones
- Show best practices for the technology

### 3. Reusability
- Make examples modular where possible
- Use parameters instead of hardcoded values when appropriate
- Follow established patterns and conventions
- Design examples that can be easily extended

### 4. Verification
- Test all code examples in a ROS 2 environment before including
- Ensure examples are complete and self-contained
- Verify examples follow ROS 2 best practices
- Check compatibility with targeted ROS 2 distribution (Humble Hawksbill)

### 5. Documentation Context
- Include a brief description of what the code does
- Explain any non-obvious aspects of the implementation
- Reference related concepts or documentation sections
- Mention any prerequisites or setup requirements
# Reusable Content Components for Code Examples

This guide provides templates and standards for creating code examples in the NVIDIA Isaac AI-Robot Brain module, following Docusaurus standards.

## Code Block Standards

All code examples should follow these standards:

### 1. Language Tagging
Use appropriate language tags for syntax highlighting:

```python
# Python examples for ROS 2 and Isaac ROS
import rclpy
from rclpy.node import Node

class IsaacExampleNode(Node):
    def __init__(self):
        super().__init__('isaac_example_node')
        self.get_logger().info('Isaac example node initialized')
```

```bash
# Bash examples for Isaac Sim and system commands
# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-*  # Install all Isaac ROS packages
```

```dockerfile
# Docker examples when needed
FROM nvcr.io/nvidia/isaac/ros:humble-ros-base-l4t-r35.2.1
```

## Isaac-Specific Code Examples

### ROS 2 Node Template
```python
import rclpy
from rclpy.node import Node

# Isaac ROS specific imports
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image

class IsaacPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_perception_node')
        
        # Create subscriptions
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
            
        # Create publishers
        self.publisher = self.create_publisher(
            AprilTagDetectionArray,
            'tag_detections',
            10)

    def image_callback(self, msg):
        self.get_logger().info('Received image')
```

### Isaac Sim Python API Example
```python
# Isaac Sim Python API template
from omni.isaac.kit import SimulationApp
import omni
from pxr import Gf

# Initialize simulation app
config = {'headless': False}
simulation_app = SimulationApp(config)

# Simulation setup
world = World(stage_units_in_meters=1.0)
# Add your robot and environment here

# Run simulation
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
```

## Best Practices for Code Examples

1. **Always include comments** explaining key steps
2. **Use consistent variable naming** following ROS 2/Isaac conventions
3. **Include error handling** where appropriate
4. **Add context** about what the code does in the preceding text
5. **Make examples self-contained** when possible but note dependencies

## Exercise Example Structure

Each hands-on exercise should follow this template:

```markdown
### Exercise: [Exercise Title]

**Objective**: [What the user will accomplish]

**Prerequisites**: 
- [List any requirements]

**Steps**:
1. [Step-by-step instructions with code examples]
2. [More steps as needed]

**Expected Output**: [What users should see]

**Troubleshooting**: [Common issues and solutions]
```
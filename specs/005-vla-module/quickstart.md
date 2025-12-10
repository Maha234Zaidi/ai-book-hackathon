# Quickstart Guide: Vision-Language-Action (VLA) Systems

## Overview
This quickstart guide provides a fast track to understanding and implementing Vision-Language-Action (VLA) systems. You'll learn to build a system that can take voice commands, process them through cognitive planning, and execute robotic actions in simulation.

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Ubuntu 22.04 or compatible Linux system
- Python 3.10 or 3.11
- Gazebo simulation environment
- Access to OpenAI API (for Whisper and LLMs) or open-source alternatives
- Basic knowledge of ROS 2 concepts (nodes, topics, services)

## Setup Environment

### 1. Install ROS 2 Humble Hawksbill
```bash
# Follow official ROS 2 Humble installation guide for your platform:
# https://docs.ros.org/en/humble/Installation.html

# Install additional packages needed for VLA
sudo apt update
sudo apt install ros-humble-rosbridge-suite ros-humble-navigation2 ros-humble-nav2-bringup
```

### 2. Setup Python Environment
```bash
# Create virtual environment
python3 -m venv ~/vla_env
source ~/vla_env/bin/activate
pip install --upgrade pip

# Install required packages
pip install openai      # For Whisper and LLM API access
pip install speechrecognition  # Alternative local speech recognition
pip install transformers        # For local LLM models if using open-source alternatives
pip install torch torchvision torchaudio  # For local AI processing
```

### 3. Clone and Setup Simulation Environment
```bash
# Create workspace
mkdir -p ~/vla_ws/src
cd ~/vla_ws

# If using existing simulation packages:
# git clone <simulation_repo> src/<repo_name>

# Build workspace
colcon build
source install/setup.bash
```

## Run the Simple VLA Demo

### 1. Launch Simulation Environment
```bash
# Terminal 1: Start Gazebo simulation
cd ~/vla_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 2. Start VLA System Components
```bash
# Terminal 2: Start voice recognition node
cd ~/vla_ws
source install/setup.bash
source ~/vla_env/bin/activate
python3 src/vla_examples/vla_nodes/voice_recognition.py
```

### 3. Test Basic Voice Command
```bash
# Terminal 3: Send a simple voice command
# Say "move forward" or run:
ros2 topic pub /vla/command std_msgs/String "data: 'move forward'"
```

## Building Components Step by Step

### 1. Voice Command Pipeline
First, implement the basic voice-to-text conversion:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
# Or import speech recognition libraries for local processing

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher = self.create_publisher(String, '/vla/recognized_text', 10)
        self.subscription = self.create_subscription(
            String,
            '/vla/command',
            self.listener_callback,
            10)
        
    def transcribe_speech(self, audio_file_path):
        """Transcribe speech using OpenAI Whisper API"""
        # Implementation using OpenAI API
        # Or alternative local transcription
        pass

def main(args=None):
    rclpy.init(args=args)
    voice_command_node = VoiceCommandNode()
    rclpy.spin(voice_command_node)
    voice_command_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Cognitive Planner Node
Then, implement the LLM-based cognitive planning:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')
        self.text_subscription = self.create_subscription(
            String,
            '/vla/recognized_text',
            self.text_callback,
            10)
        self.action_publisher = self.create_publisher(String, '/vla/action_sequence', 10)

    def plan_actions(self, text_command):
        """Use LLM to plan multi-step actions from natural language"""
        # Send text to LLM with appropriate prompting
        # Parse response into action sequence
        # Return structured sequence of robot commands
        pass

def main(args=None):
    rclpy.init(args=args)
    cognitive_planner = CognitivePlannerNode()
    rclpy.spin(cognitive_planner)
    cognitive_planner.destroy_node()
    rclpy.shutdown()
```

### 3. Integration with Perception
Connect perception systems to validate and adapt actions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vla_msgs.msg import DetectedObjects  # Custom message

class PerceptionIntegrationNode(Node):
    def __init__(self):
        super().__init__('perception_integration_node')
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.object_publisher = self.create_publisher(DetectedObjects, '/vla/perception/objects', 10)

    def detect_objects(self, image_msg):
        """Process image to detect objects relevant to commands"""
        # Run object detection
        # Return detected objects with confidence scores
        pass
```

## Complete VLA Demo

### Run the Full System
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch my_robot_sim my_robot_world.launch.py

# Terminal 2: Start all VLA components
source ~/vla_env/bin/activate
ros2 run vla_examples voice_command_node
ros2 run vla_examples cognitive_planner_node
ros2 run vla_examples perception_integration_node

# Terminal 3: Execute a complex command
# Say: "Pick up the red cup and bring it to the kitchen"
# Or publish: ros2 topic pub /vla/command std_msgs/String "data: 'pick up the red cup and bring it to the kitchen'"
```

## Troubleshooting

### Common Issues:
1. **Audio Input Problems**: Check microphone permissions and pulseaudio configuration
2. **API Access Issues**: Verify OpenAI API key is set in environment variables
3. **ROS 2 Network Issues**: Ensure ROS_DOMAIN_ID is consistent across terminals
4. **Simulation Not Responding**: Check that robot controllers are properly loaded

### Debugging Tips:
- Use `ros2 topic list` to verify topic connections
- Use `ros2 run rqt_graph rqt_graph` to visualize node connections
- Check logs with `ros2 launch` output or `journalctl -f`

## Next Steps
1. Expand vocabulary and command understanding
2. Implement more complex cognitive planning with memory
3. Add path planning integration for navigation tasks
4. Test in more complex simulation environments
5. Implement safety validation mechanisms
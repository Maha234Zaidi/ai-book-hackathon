# Basic VLA System Node Structure

This document outlines the basic ROS 2 node structure for VLA system components with proper logging and error handling.

## Base Node Template

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import logging
import traceback
from datetime import datetime


class VLABaseNode(Node):
    """
    Base class for all VLA system nodes with standardized logging and error handling.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        
        # Set up standardized logging
        self.logger = self.get_logger()
        
        # Log node initialization
        self.logger.info(f"{node_name} initialized at {datetime.now()}")
        
        # Common error handling patterns
        self.error_count = 0
        self.max_errors = 10  # Threshold before shutting down
        
    def safe_execute(self, func, *args, **kwargs):
        """
        Safely execute a function with error handling and logging.
        """
        try:
            result = func(*args, **kwargs)
            self.error_count = 0  # Reset on successful execution
            return result
        except Exception as e:
            self.error_count += 1
            self.logger.error(f"Error in {func.__name__}: {str(e)}")
            self.logger.error(f"Traceback: {traceback.format_exc()}")
            
            # Shut down if too many consecutive errors
            if self.error_count >= self.max_errors:
                self.logger.error("Too many errors, shutting down node")
                rclpy.shutdown()
                
            return None


class VoiceCommandNode(VLABaseNode):
    """
    Node for processing voice commands using OpenAI Whisper.
    """
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Create publisher and subscriber
        self.publisher = self.create_publisher(String, '/vla/recognized_text', 10)
        self.subscription = self.create_subscription(
            String,
            '/vla/command',
            self.listener_callback,
            10)
        
        # Log setup completion
        self.logger.info("Voice Command Node initialized")
    
    def listener_callback(self, msg):
        """
        Callback for processing incoming voice commands.
        """
        self.safe_execute(self.process_voice_command, msg.data)
    
    def process_voice_command(self, audio_data):
        """
        Process the voice command with proper error handling.
        """
        # Process audio through Whisper API
        try:
            transcribed_text = self.transcribe_speech(audio_data)
            
            if transcribed_text:
                # Publish the recognized text
                response_msg = String()
                response_msg.data = transcribed_text
                self.publisher.publish(response_msg)
                self.logger.info(f"Published transcribed text: {transcribed_text}")
            else:
                self.logger.warning("Could not transcribe speech")
                
        except Exception as e:
            self.logger.error(f"Error processing voice command: {str(e)}")
    
    def transcribe_speech(self, audio_data):
        """
        Transcribe speech using OpenAI Whisper API.
        """
        # Implementation would go here
        pass


class CognitivePlannerNode(VLABaseNode):
    """
    Node for cognitive planning with LLMs.
    """
    def __init__(self):
        super().__init__('cognitive_planner_node')
        
        # Create publisher and subscriber
        self.text_subscription = self.create_subscription(
            String,
            '/vla/recognized_text',
            self.text_callback,
            10)
        self.action_publisher = self.create_publisher(String, '/vla/action_sequence', 10)
        
        # Log setup completion
        self.logger.info("Cognitive Planner Node initialized")
    
    def text_callback(self, msg):
        """
        Callback for processing recognized text.
        """
        self.safe_execute(self.plan_actions, msg.data)
    
    def plan_actions(self, text_command):
        """
        Plan actions using LLM with proper error handling.
        """
        try:
            action_sequence = self.generate_action_sequence(text_command)
            
            if action_sequence:
                # Publish the action sequence
                response_msg = String()
                response_msg.data = action_sequence
                self.action_publisher.publish(response_msg)
                self.logger.info(f"Published action sequence for: {text_command}")
            else:
                self.logger.warning(f"Could not generate action sequence for: {text_command}")
                
        except Exception as e:
            self.logger.error(f"Error planning actions for '{text_command}': {str(e)}")
    
    def generate_action_sequence(self, text_command):
        """
        Generate action sequence using LLM.
        """
        # Implementation would go here
        pass


# Example usage
def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create nodes
        voice_node = VoiceCommandNode()
        cognitive_node = CognitivePlannerNode()
        
        # Spin both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(voice_node)
        executor.add_node(cognitive_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            voice_node.destroy_node()
            cognitive_node.destroy_node()
    
    except Exception as e:
        logging.error(f"Error in main: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Features of the Base Node Structure

1. **Standardized Logging**: Each node has a consistent logging approach
2. **Error Handling**: Safe execution wrapper that handles errors gracefully
3. **Error Threshold**: Automatic shutdown if too many consecutive errors occur
4. **Resource Management**: Proper cleanup of nodes and resources
5. **Extensibility**: Easy to extend for different types of VLA components
6. **Thread Safety**: Uses MultiThreadedExecutor for concurrent processing
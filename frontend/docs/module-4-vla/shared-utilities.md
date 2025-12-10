# Shared Utilities for VLA System

This document outlines the shared utilities for API communication, configuration management, and simulation interaction in the VLA system.

## API Communication Utilities

### OpenAI API Client

```python
import openai
import os
from typing import Optional
import logging
from datetime import datetime


class OpenAIClient:
    """
    Utility class for interacting with OpenAI APIs (Whisper, LLMs).
    """
    def __init__(self):
        # Load API key from environment variable
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")
        
        openai.api_key = api_key
        self.logger = logging.getLogger(__name__)
    
    def transcribe_audio(self, audio_file_path: str, language: Optional[str] = None) -> str:
        """
        Transcribe audio using OpenAI Whisper API.
        
        Args:
            audio_file_path: Path to the audio file to transcribe
            language: Optional language code (e.g., 'en', 'es')
            
        Returns:
            Transcribed text
        """
        try:
            with open(audio_file_path, 'rb') as audio_file:
                response = openai.Audio.transcribe(
                    "whisper-1",
                    audio_file,
                    language=language
                )
            
            transcription = response.get("text", "")
            self.logger.info(f"Successfully transcribed audio: {audio_file_path}")
            return transcription
            
        except Exception as e:
            self.logger.error(f"Error transcribing audio: {str(e)}")
            return ""
    
    def generate_completion(self, prompt: str, model: str = "gpt-3.5-turbo", 
                           max_tokens: int = 500) -> str:
        """
        Generate text completion using OpenAI GPT models.
        
        Args:
            prompt: Input prompt for the model
            model: GPT model to use
            max_tokens: Maximum tokens in the response
            
        Returns:
            Generated text completion
        """
        try:
            response = openai.ChatCompletion.create(
                model=model,
                messages=[{"role": "user", "content": prompt}],
                max_tokens=max_tokens,
                temperature=0.3
            )
            
            completion = response.choices[0].message['content'].strip()
            self.logger.info(f"Generated completion using {model}")
            return completion
            
        except Exception as e:
            self.logger.error(f"Error generating completion: {str(e)}")
            return ""


# Usage example:
# client = OpenAIClient()
# transcription = client.transcribe_audio("path/to/audio.wav")
# completion = client.generate_completion("Translate this to French: Hello world")
```

## Configuration Management Utilities

```python
import json
import os
from typing import Any, Dict
import logging


class ConfigManager:
    """
    Utility class for managing configuration settings across the VLA system.
    """
    def __init__(self, config_file_path: str = None):
        self.config_file_path = config_file_path or "vla_config.json"
        self.config = self.load_config()
        self.logger = logging.getLogger(__name__)
    
    def load_config(self) -> Dict[str, Any]:
        """
        Load configuration from file or use defaults.
        """
        try:
            if os.path.exists(self.config_file_path):
                with open(self.config_file_path, 'r') as f:
                    config = json.load(f)
                self.logger.info(f"Configuration loaded from {self.config_file_path}")
                return config
            else:
                self.logger.warning(f"Config file {self.config_file_path} not found, using defaults")
                return self.get_default_config()
        except Exception as e:
            self.logger.error(f"Error loading config, using defaults: {str(e)}")
            return self.get_default_config()
    
    def get_default_config(self) -> Dict[str, Any]:
        """
        Get default configuration values.
        """
        return {
            "ros_domain_id": 0,
            "whisper_model": "whisper-1",
            "llm_model": "gpt-3.5-turbo",
            "confidence_threshold": 0.8,
            "max_retries": 3,
            "timeout_seconds": 30,
            "simulation": {
                "gazebo_world": "empty_world",
                "robot_model": "turtlebot3_waffle"
            },
            "safety": {
                "max_velocity": 1.0,
                "min_distance_obstacle": 0.5,
                "action_timeout": 60.0
            }
        }
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration value by key.
        
        Args:
            key: Configuration key (supports dot notation for nested values)
            default: Default value if key not found
            
        Returns:
            Configuration value or default
        """
        keys = key.split('.')
        value = self.config
        
        try:
            for k in keys:
                value = value[k]
            return value
        except (KeyError, TypeError):
            return default
    
    def set(self, key: str, value: Any) -> None:
        """
        Set a configuration value by key.
        
        Args:
            key: Configuration key (supports dot notation for nested values)
            value: Value to set
        """
        keys = key.split('.')
        config_ref = self.config
        
        # Navigate to the parent of the target key
        for k in keys[:-1]:
            if k not in config_ref:
                config_ref[k] = {}
            config_ref = config_ref[k]
        
        # Set the final key
        config_ref[keys[-1]] = value
        
        # Save the updated config
        self.save_config()
    
    def save_config(self) -> None:
        """
        Save the current configuration to file.
        """
        try:
            with open(self.config_file_path, 'w') as f:
                json.dump(self.config, f, indent=2)
            self.logger.info(f"Configuration saved to {self.config_file_path}")
        except Exception as e:
            self.logger.error(f"Error saving config: {str(e)}")


# Usage example:
# config = ConfigManager()
# domain_id = config.get("ros_domain_id", 0)
# config.set("safety.max_velocity", 0.5)
```

## Simulation Interaction Utilities

```python
import subprocess
import time
import os
import signal
from typing import Optional, Dict, Any
import logging


class SimulationManager:
    """
    Utility class for managing Gazebo simulation interactions.
    """
    def __init__(self):
        self.processes = {}
        self.logger = logging.getLogger(__name__)
    
    def start_gazebo_simulation(self, world_name: str = "empty_world", 
                                headless: bool = False) -> Optional[int]:
        """
        Start a Gazebo simulation instance.
        
        Args:
            world_name: Name of the Gazebo world to load
            headless: Whether to run simulation without GUI
            
        Returns:
            Process ID of the simulation, or None if failed
        """
        try:
            cmd = ["ros2", "launch", "turtlebot3_gazebo", f"empty_world.launch.py"]
            
            if headless:
                cmd.extend(["gui:=false"])
            
            # Set environment variables
            env = os.environ.copy()
            env["GAZEBO_MODEL_PATH"] = os.path.expanduser("~/.gazebo/models")
            
            # Start the simulation process
            proc = subprocess.Popen(cmd, env=env)
            
            # Store process reference
            self.processes["gazebo"] = proc
            
            self.logger.info(f"Started Gazebo simulation for world: {world_name}")
            return proc.pid
            
        except Exception as e:
            self.logger.error(f"Error starting Gazebo simulation: {str(e)}")
            return None
    
    def stop_simulation(self, sim_type: str = "gazebo") -> bool:
        """
        Stop a running simulation.
        
        Args:
            sim_type: Type of simulation to stop (currently only supports "gazebo")
            
        Returns:
            True if successfully stopped, False otherwise
        """
        try:
            if sim_type in self.processes and self.processes[sim_type].poll() is None:
                proc = self.processes[sim_type]
                
                # Terminate the process
                proc.terminate()
                
                # Wait for graceful shutdown
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Force kill if it doesn't terminate gracefully
                    proc.kill()
                    proc.wait()
                
                del self.processes[sim_type]
                self.logger.info(f"Stopped {sim_type} simulation")
                return True
            else:
                self.logger.warning(f"No running {sim_type} simulation to stop")
                return False
                
        except Exception as e:
            self.logger.error(f"Error stopping {sim_type} simulation: {str(e)}")
            return False
    
    def check_simulation_status(self, sim_type: str = "gazebo") -> bool:
        """
        Check if simulation is currently running.
        
        Args:
            sim_type: Type of simulation to check
            
        Returns:
            True if running, False otherwise
        """
        if sim_type in self.processes:
            proc = self.processes[sim_type]
            return proc.poll() is None
        return False
    
    def reset_simulation(self) -> bool:
        """
        Reset the simulation to initial state.
        This is a simplified version; in practice, would send a reset command via ROS.
        
        Returns:
            True if reset was initiated successfully
        """
        try:
            # In a real implementation, this would be a ROS service call
            # ros2 service call /reset_simulation std_srvs/Empty
            self.logger.info("Simulation reset initiated")
            return True
        except Exception as e:
            self.logger.error(f"Error resetting simulation: {str(e)}")
            return False


# Usage example:
# sim_manager = SimulationManager()
# pid = sim_manager.start_gazebo_simulation("empty_world")
# time.sleep(5)  # Allow simulation to start
# is_running = sim_manager.check_simulation_status()
# sim_manager.stop_simulation()
```

## Utility Integration Example

```python
def setup_vla_system():
    """
    Example function showing how to integrate the shared utilities.
    """
    # Initialize configuration
    config = ConfigManager("vla_config.json")
    
    # Initialize API client
    api_client = OpenAIClient()
    
    # Initialize simulation manager
    sim_manager = SimulationManager()
    
    # Start simulation
    sim_pid = sim_manager.start_gazebo_simulation(
        config.get("simulation.gazebo_world", "empty_world")
    )
    
    if sim_pid:
        print(f"VLA system initialized with simulation PID: {sim_pid}")
        return config, api_client, sim_manager
    else:
        print("Failed to initialize VLA system")
        return None, None, None


if __name__ == "__main__":
    config, api_client, sim_manager = setup_vla_system()
    
    if config and api_client and sim_manager:
        print("VLA system is ready for operation")
        
        # Example usage of components
        # text = api_client.transcribe_audio("sample.wav")
        # response = api_client.generate_completion(f"Plan actions for: {text}")
        
        # Stop simulation when done
        # sim_manager.stop_simulation()
```

## Best Practices for Shared Utilities

1. **Error Handling**: All utilities include comprehensive error handling and logging
2. **Environment Variables**: Sensitive information (like API keys) are loaded from environment variables
3. **Configuration Flexibility**: System behavior can be adjusted through configuration files
4. **Resource Management**: Proper cleanup of processes and resources
5. **Modularity**: Each utility is self-contained and can be used independently
6. **Logging**: Consistent logging across all utilities for debugging and monitoring
7. **Type Hints**: All functions include proper type hints for better code understanding
8. **Documentation**: Each function includes docstrings explaining parameters and return values
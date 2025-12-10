# Audio Input Handling and Preprocessing Node

This document provides a complete implementation of a ROS 2 node for audio input handling and preprocessing. This node captures audio from various sources and preprocesses it before sending it to the voice recognition system.

## Complete Implementation

```python
#!/usr/bin/env python3
"""
Audio Input and Preprocessing Node for VLA System

This node handles audio capture from various input sources, performs preprocessing,
and publishes audio data for the voice recognition system.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import pyaudio
import numpy as np
import threading
import queue
import time
from typing import Optional, Callable
import struct

# Import standard ROS 2 message types
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from builtin_interfaces.msg import Time

# Optional: Import for audio file processing
import wave
import io


class AudioInputNode(Node):
    """
    Audio input and preprocessing node for the VLA system.
    """
    
    def __init__(self):
        super().__init__('audio_input_node')
        
        # Configuration parameters
        self.declare_parameter('input_device_index', -1)  # Default to system default
        self.declare_parameter('sample_rate', 16000)      # Standard for speech recognition
        self.declare_parameter('channels', 1)             # Mono for speech
        self.declare_parameter('chunk_size', 1024)        # Audio chunk size
        self.declare_parameter('enable_noise_reduction', True)
        self.declare_parameter('vad_threshold', 0.01)     # Voice activity detection threshold
        self.declare_parameter('silence_timeout', 5.0)    # Seconds of silence before stopping
        self.declare_parameter('preprocessing_enabled', True)
        
        self.input_device_index = self.get_parameter('input_device_index').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.enable_noise_reduction = self.get_parameter('enable_noise_reduction').value
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.silence_timeout = self.get_parameter('silence_timeout').value
        self.preprocessing_enabled = self.get_parameter('preprocessing_enabled').value
        
        # Audio processing parameters
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.recording_thread = None
        self.last_audio_time = time.time()
        
        # Initialize PyAudio
        self.pyaudio_instance = pyaudio.PyAudio()
        
        # Validate audio device
        if self.input_device_index == -1:
            self.input_device_index = self.pyaudio_instance.get_default_input_device_info()['index']
        
        if self.input_device_index >= self.pyaudio_instance.get_device_count():
            raise ValueError(f"Audio device index {self.input_device_index} is out of range")
        
        # Create publishers
        self.audio_publisher = self.create_publisher(
            AudioData,
            '/vla/audio_input',
            qos_profile_sensor_data
        )
        
        # Publisher for audio status
        self.status_publisher = self.create_publisher(
            String,
            '/vla/audio_status',
            10
        )
        
        # Setup audio stream
        try:
            self.audio_stream = self.pyaudio_instance.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size,
                input_device_index=self.input_device_index,
                stream_callback=self.audio_callback
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {str(e)}")
            raise
        
        # Start the audio processing
        self.audio_stream.start_stream()
        self.is_recording = True
        
        # Create a timer to process audio data from queue
        self.processing_timer = self.create_timer(0.1, self.process_audio_data)
        
        self.get_logger().info("Audio Input Node initialized")
        self.get_logger().info(f"Sample rate: {self.sample_rate}Hz, Channels: {self.channels}")
        self.get_logger().info(f"Input device: {self.input_device_index}")

    def audio_callback(self, in_data, frame_count, time_info, status):
        """
        PyAudio callback for receiving audio data.
        """
        # Add audio data to queue for processing
        self.audio_queue.put(in_data)
        
        # Update last audio time for timeout management
        self.last_audio_time = time.time()
        
        return (None, pyaudio.paContinue)

    def process_audio_data(self):
        """
        Process audio data from the queue and publish to ROS topic.
        """
        # Check for silence timeout
        if time.time() - self.last_audio_time > self.silence_timeout:
            if self.is_recording:
                self.get_logger().info("Silence timeout - stopping recording")
                self.status_publisher.publish(String(data="silence_timeout"))
        
        # Process all available audio data in queue
        processed_count = 0
        while not self.audio_queue.empty() and processed_count < 10:  # Limit per timer callback
            try:
                audio_chunk = self.audio_queue.get_nowait()
                processed_count += 1
                
                # Preprocess the audio if enabled
                if self.preprocessing_enabled:
                    audio_chunk = self.preprocess_audio(audio_chunk)
                
                # Publish the audio data as AudioData message
                audio_msg = AudioData()
                audio_msg.data = list(audio_chunk)
                
                # Set timestamp
                current_time = self.get_clock().now().to_msg()
                audio_msg.layout.dim = [String(data="audio_data")]
                audio_msg.layout.data_offset = 0
                
                self.audio_publisher.publish(audio_msg)
                
                # Publish status indicating active recording
                if processed_count == 1:  # Only publish first in batch
                    self.status_publisher.publish(String(data="recording"))
                    
            except queue.Empty:
                break  # No more data to process

    def preprocess_audio(self, audio_chunk: bytes) -> bytes:
        """
        Apply preprocessing to audio data.
        """
        # Convert bytes to numpy array for processing
        audio_array = np.frombuffer(audio_chunk, dtype=np.int16)
        
        # Apply noise reduction if enabled
        if self.enable_noise_reduction:
            audio_array = self.apply_noise_reduction(audio_array)
        
        # Apply voice activity detection
        if not self.is_voice_activity(audio_array):
            # If no voice activity, return empty chunk to reduce noise transmission
            return b""
        
        # Normalize audio
        audio_array = self.normalize_audio(audio_array)
        
        # Convert back to bytes
        return audio_array.tobytes()

    def apply_noise_reduction(self, audio_array: np.ndarray) -> np.ndarray:
        """
        Apply basic noise reduction using spectral subtraction.
        """
        # For simplicity, using a basic approach
        # In practice, more sophisticated methods like spectral subtraction or ML-based NR
        threshold = np.std(audio_array) * 0.2  # 20% of std as noise threshold
        audio_array = np.where(np.abs(audio_array) > threshold, audio_array, 0)
        return audio_array

    def is_voice_activity(self, audio_array: np.ndarray) -> bool:
        """
        Simple voice activity detection based on energy threshold.
        """
        energy = np.mean(np.abs(audio_array))
        return energy > self.vad_threshold

    def normalize_audio(self, audio_array: np.ndarray) -> np.ndarray:
        """
        Normalize audio signal to standard range.
        """
        max_val = np.max(np.abs(audio_array))
        if max_val > 0:
            normalized = audio_array / max_val
            # Scale back to int16 range
            return (normalized * 32767).astype(np.int16)
        return audio_array

    def get_device_info(self):
        """
        Get information about the current audio input device.
        """
        try:
            device_info = self.pyaudio_instance.get_device_info_by_index(self.input_device_index)
            return {
                'name': device_info['name'],
                'max_input_channels': device_info['maxInputChannels'],
                'default_sample_rate': device_info['defaultSampleRate'],
                'host_api': device_info['hostApi']
            }
        except Exception:
            return None

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        self.get_logger().info("Shutting down Audio Input Node...")
        
        # Stop audio stream
        if self.audio_stream and self.audio_stream.is_active():
            self.audio_stream.stop_stream()
            self.audio_stream.close()
        
        # Terminate PyAudio
        if self.pyaudio_instance:
            self.pyaudio_instance.terminate()
        
        super().destroy_node()


class AudioFromFileNode(Node):
    """
    Alternative audio input node that reads from audio files instead of microphone.
    Useful for testing and offline processing.
    """
    
    def __init__(self):
        super().__init__('audio_from_file_node')
        
        # Configuration parameters
        self.declare_parameter('audio_file_path', '')
        self.declare_parameter('loop_playback', False)
        self.declare_parameter('playback_speed', 1.0)
        
        self.audio_file_path = self.get_parameter('audio_file_path').value
        self.loop_playback = self.get_parameter('loop_playback').value
        self.playback_speed = self.get_parameter('playback_speed').value
        
        # Create publishers
        self.audio_publisher = self.create_publisher(
            AudioData,
            '/vla/audio_input',
            qos_profile_sensor_data
        )
        
        # Timer to read and publish audio data
        self.read_timer = self.create_timer(0.01, self.read_audio_file)
        
        # Audio file state
        self.current_position = 0
        self.audio_data = None
        self.sample_rate = 16000  # Default, will be read from file
        
        # Read the audio file
        self.load_audio_file()
        
        self.get_logger().info(f"Audio From File Node initialized: {self.audio_file_path}")

    def load_audio_file(self):
        """
        Load audio file and prepare for streaming.
        """
        if not self.audio_file_path or not os.path.exists(self.audio_file_path):
            self.get_logger().error(f"Audio file does not exist: {self.audio_file_path}")
            return
        
        try:
            with wave.open(self.audio_file_path, 'rb') as wf:
                # Read audio parameters
                self.sample_rate = wf.getframerate()
                frames = wf.readframes(wf.getnframes())
                self.audio_data = frames
                
                self.get_logger().info(
                    f"Loaded audio file: {self.audio_file_path}, "
                    f"Sample rate: {self.sample_rate}Hz, "
                    f"Duration: {wf.getnframes() / self.sample_rate:.2f}s"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to load audio file: {str(e)}")

    def read_audio_file(self):
        """
        Read and publish audio data from file.
        """
        if not self.audio_data:
            return
        
        # Calculate chunk size based on playback speed
        chunk_size = int(self.sample_rate * 0.1 * self.playback_speed)  # 0.1 second chunks
        
        if self.current_position >= len(self.audio_data):
            if self.loop_playback:
                self.current_position = 0
            else:
                self.get_logger().info("Reached end of audio file")
                return
        
        # Extract audio chunk
        end_pos = min(self.current_position + chunk_size, len(self.audio_data))
        audio_chunk = self.audio_data[self.current_position:end_pos]
        self.current_position = end_pos
        
        # Publish the audio data
        audio_msg = AudioData()
        audio_msg.data = list(audio_chunk)
        
        self.audio_publisher.publish(audio_msg)

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        self.get_logger().info("Shutting down Audio From File Node...")
        super().destroy_node()


def main(args=None):
    """
    Main function to run the audio input node.
    """
    rclpy.init(args=args)
    
    try:
        # Choose which node to run based on parameters or use case
        node_type = os.getenv("AUDIO_INPUT_NODE_TYPE", "mic")  # "mic" or "file"
        
        if node_type == "file":
            node = AudioFromFileNode()
        else:
            node = AudioInputNode()
        
        try:
            node.get_logger().info(f"Starting {node.get_name()}...")
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Interrupted by user")
        finally:
            node.destroy_node()
    
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Key Implementation Features

### 1. Audio Input Handling
- Captures audio from microphone using PyAudio
- Configurable audio parameters (sample rate, channels, device)
- Real-time audio data processing with callback mechanism

### 2. Preprocessing Capabilities
- Noise reduction algorithms
- Voice activity detection (VAD)
- Audio normalization
- Configurable preprocessing chain

### 3. Audio File Support
- Alternative node to read from audio files (useful for testing)
- Loop playback option
- Playback speed control

### 4. ROS 2 Integration
- Publishes to `/vla/audio_input` topic using AudioData message
- Status publishing for monitoring audio state
- Parameter configuration through ROS parameter system

### 5. Performance Optimization
- Asynchronous audio processing with queues
- Timer-based batch processing
- Memory management for continuous operation

### 6. Error Handling and Logging
- Comprehensive exception handling
- Detailed logging for debugging
- Resource cleanup on node destruction

## Dependencies

To use this node, you'll need to install the following Python packages:

```bash
pip install pyaudio numpy rclpy sensor_msgs
```

## Configuration Parameters

The node supports the following parameters:

- `input_device_index`: Audio input device (use -1 for default)
- `sample_rate`: Audio sample rate (default: 16000Hz)
- `channels`: Number of audio channels (default: 1 for mono)
- `chunk_size`: Audio chunk size for processing (default: 1024)
- `enable_noise_reduction`: Enable/disable noise reduction (default: True)
- `vad_threshold`: Voice activity detection threshold (default: 0.01)
- `silence_timeout`: Seconds of silence before stopping (default: 5.0)
- `preprocessing_enabled`: Enable/disable audio preprocessing (default: True)

For the file-based node:
- `audio_file_path`: Path to audio file for input
- `loop_playback`: Whether to loop the audio file (default: False)
- `playback_speed`: Speed multiplier for playback (default: 1.0)

## Usage Examples

1. Run the microphone-based node:
```bash
ros2 run your_package audio_input_node
```

2. Run with file-based input:
```bash
# Set environment variable to use file node
export AUDIO_INPUT_NODE_TYPE=file
ros2 run your_package audio_input_node
```

3. Set parameters when running:
```bash
ros2 run your_package audio_input_node --ros-args -p input_device_index:=1 -p sample_rate:=44100
```

This implementation provides a robust audio input solution for the VLA system, handling both real-time microphone input and file-based input for testing and development.
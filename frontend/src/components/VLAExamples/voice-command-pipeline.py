# Voice Recognition Node Implementation

This document provides a complete Python implementation for the voice recognition node using OpenAI Whisper API. The implementation follows ROS 2 standards and includes proper error handling and logging.

## Complete Implementation

```python
#!/usr/bin/env python3
"""
Voice Recognition Node for VLA System

This node listens for audio input and converts it to text using OpenAI Whisper API.
It then processes the text and publishes it to other VLA system components.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import os
import tempfile
import wave
import audioop
from typing import Optional
import logging

# Import standard ROS 2 message types
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from vla_msgs.srv import TranscribeAudio  # Custom service, defined in message-types.md

# Import OpenAI for Whisper API
import openai


class VoiceRecognitionNode(Node):
    """
    Voice recognition node that converts speech to text using OpenAI Whisper API.
    """
    
    def __init__(self):
        super().__init__('voice_recognition_node')
        
        # Initialize OpenAI API
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error("OPENAI_API_KEY environment variable not set")
            raise ValueError("OPENAI_API_KEY environment variable not set")
        
        openai.api_key = api_key
        
        # Configuration parameters
        self.declare_parameter('whisper_model', 'whisper-1')
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('language', 'en')
        self.declare_parameter('max_audio_duration', 30.0)
        
        self.whisper_model = self.get_parameter('whisper_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.language = self.get_parameter('language').value
        self.max_audio_duration = self.get_parameter('max_audio_duration').value
        
        # Set up QoS profile for audio data
        audio_qos = qos_profile_sensor_data
        
        # Create subscribers
        self.audio_subscriber = self.create_subscription(
            AudioData,
            '/vla/audio_input',
            self.audio_callback,
            audio_qos
        )
        
        # Create publishers
        self.text_publisher = self.create_publisher(
            String,
            '/vla/recognized_text',
            10
        )
        
        # Set up service for direct transcription requests
        self.transcribe_service = self.create_service(
            TranscribeAudio,
            'transcribe_audio',
            self.transcribe_service_callback
        )
        
        # Storage for audio chunks (for real-time processing)
        self.audio_buffer = bytearray()
        self.audio_buffer_max_size = 48000 * 10  # 10 seconds of audio at 48kHz
        
        # Statistics
        self.total_requests = 0
        self.successful_transcriptions = 0
        self.failed_transcriptions = 0
        
        self.get_logger().info("Voice Recognition Node initialized")
        self.get_logger().info(f"Using Whisper model: {self.whisper_model}")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")

    def audio_callback(self, msg: AudioData):
        """
        Callback for audio input messages.
        """
        self.get_logger().debug(f"Received audio data of {len(msg.data)} bytes")
        
        # Process the audio data
        try:
            # Add to buffer for potential real-time processing
            self.audio_buffer.extend(msg.data)
            
            # Keep buffer at reasonable size
            if len(self.audio_buffer) > self.audio_buffer_max_size:
                # Keep only the most recent data
                self.audio_buffer = self.audio_buffer[-self.audio_buffer_max_size:]
            
            # Check if this is a complete command (e.g., longer audio segment)
            # For demo purposes, we'll process if we have enough data
            if len(self.audio_buffer) > 8000:  # At least 1 second of 8kHz audio
                # Process the accumulated audio
                audio_data = bytes(self.audio_buffer)
                self.audio_buffer.clear()  # Clear buffer after processing
                
                # Transcribe the audio data
                transcription = self.transcribe_audio_data(audio_data)
                
                if transcription:
                    # Publish the recognized text
                    text_msg = String()
                    text_msg.data = transcription.strip()
                    self.text_publisher.publish(text_msg)
                    self.get_logger().info(f"Published transcription: {text_msg.data}")
                else:
                    self.get_logger().warning("Failed to transcribe audio data")
                    
        except Exception as e:
            self.get_logger().error(f"Error in audio callback: {str(e)}")

    def transcribe_audio_data(self, audio_data: bytes) -> Optional[str]:
        """
        Transcribe raw audio data using OpenAI Whisper API.
        
        Args:
            audio_data: Raw audio data as bytes
            
        Returns:
            Transcribed text, or None if transcription failed
        """
        self.total_requests += 1
        
        try:
            # Create a temporary WAV file for Whisper API
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                # Write WAV header and audio data
                # For this example, assuming 16kHz, 16-bit, mono audio
                # In practice, you'd want to handle different formats
                self.write_wav_header(temp_file, len(audio_data), 16000, 1, 16)
                temp_file.write(audio_data)
                temp_wav_path = temp_file.name
            
            # Use OpenAI Whisper API to transcribe
            with open(temp_wav_path, 'rb') as audio_file:
                response = openai.Audio.transcribe(
                    model=self.whisper_model,
                    file=audio_file,
                    language=self.language,
                    response_format='verbose_json',  # Get more detailed response with confidence
                )
            
            # Clean up temporary file
            os.unlink(temp_wav_path)
            
            # Extract transcription and confidence scores
            transcription = response.get("text", "").strip()
            
            # Calculate average confidence from segments if available
            avg_confidence = 1.0  # Default to high confidence if not available
            segments = response.get("segments", [])
            if segments:
                total_confidence = sum(segment.get("avg_logprob", 0) for segment in segments)
                avg_confidence = total_confidence / len(segments) if len(segments) > 0 else 0.0
            
            # Apply confidence threshold
            if avg_confidence < self.confidence_threshold:
                self.get_logger().warning(
                    f"Transcription confidence ({avg_confidence:.2f}) below threshold ({self.confidence_threshold})"
                )
                self.failed_transcriptions += 1
                return None
            
            self.successful_transcriptions += 1
            self.get_logger().info(f"Transcription successful: '{transcription}' (confidence: {avg_confidence:.2f})")
            return transcription
            
        except Exception as e:
            self.get_logger().error(f"Error transcribing audio: {str(e)}")
            self.failed_transcriptions += 1
            return None

    def write_wav_header(self, file, data_size, sample_rate, channels, bits_per_sample):
        """
        Write WAV file header.
        
        Args:
            file: File object to write to
            data_size: Size of audio data in bytes
            sample_rate: Sample rate in Hz
            channels: Number of audio channels
            bits_per_sample: Bits per sample
        """
        # Calculate derived values
        byte_rate = sample_rate * channels * bits_per_sample // 8
        block_align = channels * bits_per_sample // 8
        
        # Write the header
        file.write(b'RIFF')  # Chunk ID
        file.write((data_size + 36).to_bytes(4, 'little'))  # Chunk Size
        file.write(b'WAVE')  # Format
        
        # fmt subchunk
        file.write(b'fmt ')  # Subchunk1 ID
        file.write((16).to_bytes(4, 'little'))  # Subchunk1 Size
        file.write((1).to_bytes(2, 'little'))  # Audio Format (1 = PCM)
        file.write(channels.to_bytes(2, 'little'))  # Num Channels
        file.write(sample_rate.to_bytes(4, 'little'))  # Sample Rate
        file.write(byte_rate.to_bytes(4, 'little'))  # Byte Rate
        file.write(block_align.to_bytes(2, 'little'))  # Block Align
        file.write(bits_per_sample.to_bytes(2, 'little'))  # Bits Per Sample
        
        # data subchunk
        file.write(b'data')  # Subchunk2 ID
        file.write(data_size.to_bytes(4, 'little'))  # Subchunk2 Size

    def transcribe_service_callback(self, request, response):
        """
        Service callback for direct transcription requests.
        """
        self.get_logger().info(f"Received transcription request for audio file: {request.audio_input_path}")
        
        try:
            # Transcribe the audio file
            if request.audio_input_path and os.path.exists(request.audio_input_path):
                # Read and transcribe the file
                with open(request.audio_input_path, 'rb') as audio_file:
                    response_raw = openai.Audio.transcribe(
                        model=self.whisper_model,
                        file=audio_file,
                        language=request.language_code if request.language_code else self.language,
                    )
                
                response.transcribed_text = response_raw.get("text", "")
                response.confidence_score = 1.0  # Placeholder, actual confidence would require detailed response
                response.success = True
                response.processing_time = 0.0  # Placeholder, would need actual timing
                
                self.get_logger().info(f"Service transcription result: {response.transcribed_text[:50]}...")
            else:
                response.success = False
                response.error_message = f"File does not exist: {request.audio_input_path}"
                
        except Exception as e:
            self.get_logger().error(f"Service transcription error: {str(e)}")
            response.success = False
            response.error_message = str(e)
        
        return response

    def get_statistics(self):
        """
        Return statistics about transcription performance.
        """
        return {
            'total_requests': self.total_requests,
            'successful_transcriptions': self.successful_transcriptions,
            'failed_transcriptions': self.failed_transcriptions,
            'success_rate': self.successful_transcriptions / self.total_requests if self.total_requests > 0 else 0.0
        }


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create the voice recognition node
        node = VoiceRecognitionNode()
        
        # Create multi-threaded executor for handling multiple callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            # Log startup information
            node.get_logger().info("Voice Recognition Node starting...")
            
            # Spin the node
            executor.spin()
            
        except KeyboardInterrupt:
            node.get_logger().info("Interrupted by user")
        finally:
            # Cleanup
            executor.shutdown()
            node.destroy_node()
    
    except Exception as e:
        logging.error(f"Error in main: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Key Implementation Features

### 1. Initialization and Configuration
- Sets up OpenAI API with key from environment variables
- Configurable parameters for Whisper model, confidence threshold, language, and audio duration
- Proper ROS 2 node initialization with parameter declaration

### 2. Audio Handling
- Subscribes to `/vla/audio_input` topic for audio data
- Implements audio buffering for real-time processing
- Handles raw audio data and converts to appropriate format for Whisper API

### 3. Whisper Integration
- Uses OpenAI's Whisper API for speech-to-text conversion
- Implements confidence threshold filtering
- Handles different languages and response formats

### 4. Message Handling
- Publishes transcribed text to `/vla/recognized_text` topic
- Implements a service for direct transcription requests
- Proper error handling and logging

### 5. Error Handling
- Comprehensive exception handling for API and file operations
- Audio buffer management to prevent memory issues
- Detailed logging for debugging and monitoring

### 6. Performance Considerations
- Efficient audio data handling
- Temporary file management for WAV conversion
- Statistics tracking for performance monitoring

## Installation and Setup

To use this node, you'll need to:

1. Install required Python packages:
```bash
pip install openai rclpy sensor_msgs
```

2. Set your OpenAI API key:
```bash
export OPENAI_API_KEY="your-api-key-here"
```

3. Make sure your custom message types are properly defined and built in your ROS 2 workspace.

## Testing the Implementation

You can test this implementation by:

1. Running the node:
```bash
ros2 run your_package voice_recognition_node
```

2. Publishing audio data to `/vla/audio_input` or using the transcription service

3. Monitoring the output on `/vla/recognized_text`

This implementation provides a robust foundation for voice recognition in the VLA system while handling all the technical complexities of audio processing and API integration.
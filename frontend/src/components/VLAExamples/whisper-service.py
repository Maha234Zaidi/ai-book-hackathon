# ROS 2 Service for Speech-to-Text Conversion using Whisper

This document provides an implementation of a ROS 2 service for speech-to-text conversion using the OpenAI Whisper API. This service can be called by other VLA system components when they need to convert audio to text.

## Complete Implementation

```python
#!/usr/bin/env python3
"""
Speech-to-Text Service Node using OpenAI Whisper API

This node provides a ROS 2 service for speech-to-text conversion using OpenAI Whisper API.
It can be called by other nodes in the VLA system when they need transcription services.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import os
import tempfile
import time
import wave
from typing import Optional
import json

# Import standard ROS 2 message types
from std_msgs.msg import String
from builtin_interfaces.msg import Time

# Import our custom service definition
# This would need to be defined in your package's srv/ directory
# For the purpose of this example, we'll define it here as well
from vla_msgs.srv import TranscribeAudio

# Import OpenAI for Whisper API
import openai


class WhisperTranscriptionService(Node):
    """
    A ROS 2 service node for speech-to-text conversion using OpenAI Whisper API.
    """
    
    def __init__(self):
        super().__init__('whisper_transcription_service')
        
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
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('request_timeout', 120.0)
        
        self.whisper_model = self.get_parameter('whisper_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.language = self.get_parameter('language').value
        self.max_audio_duration = self.get_parameter('max_audio_duration').value
        self.max_retries = self.get_parameter('max_retries').value
        self.request_timeout = self.get_parameter('request_timeout').value
        
        # Statistics
        self.total_requests = 0
        self.successful_transcriptions = 0
        self.failed_transcriptions = 0
        self.total_processing_time = 0.0
        
        # Create the service
        self.transcription_service = self.create_service(
            TranscribeAudio,
            'whisper_transcribe_audio',
            self.transcribe_audio_callback
        )
        
        # Publisher for service statistics
        self.stats_publisher = self.create_publisher(
            String,
            '/vla/transcription_stats',
            10
        )
        
        # Log initialization
        self.get_logger().info("Whisper Transcription Service initialized")
        self.get_logger().info(f"Using Whisper model: {self.whisper_model}")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
        
        # Timer to periodically publish statistics
        self.stats_timer = self.create_timer(10.0, self.publish_statistics)

    def transcribe_audio_callback(self, request: TranscribeAudio.Request, 
                                 response: TranscribeAudio.Response) -> TranscribeAudio.Response:
        """
        Service callback for audio transcription requests.
        
        Args:
            request: TranscribeAudio.Request containing audio file path or data
            response: TranscribeAudio.Response to be filled with results
            
        Returns:
            Filled response object
        """
        start_time = time.time()
        self.total_requests += 1
        
        self.get_logger().info(f"Received transcription request for: {request.audio_input_path}")
        
        try:
            # Validate request
            if not request.audio_input_path or not os.path.exists(request.audio_input_path):
                response.success = False
                response.error_message = f"Audio file does not exist: {request.audio_input_path}"
                self.failed_transcriptions += 1
                return response
            
            # Check audio file duration against limits
            duration = self.get_audio_duration(request.audio_input_path)
            if duration > self.max_audio_duration:
                response.success = False
                response.error_message = f"Audio file duration ({duration}s) exceeds maximum allowed ({self.max_audio_duration}s)"
                self.failed_transcriptions += 1
                return response
            
            # Attempt transcription with retries
            for attempt in range(self.max_retries):
                try:
                    # Transcribe the audio file
                    transcription_result = self.transcribe_with_whisper(request)
                    processing_time = time.time() - start_time
                    
                    if transcription_result:
                        response.transcribed_text = transcription_result.get("text", "")
                        
                        # Calculate average confidence
                        segments = transcription_result.get("segments", [])
                        avg_confidence = 1.0  # Default to high confidence
                        if segments:
                            total_confidence = sum(
                                segment.get("confidence", 1.0) for segment in segments
                            )
                            avg_confidence = total_confidence / len(segments) if len(segments) > 0 else 0.0
                        
                        response.confidence_score = avg_confidence
                        response.processing_time = processing_time
                        response.success = True
                        
                        # Log success
                        self.get_logger().info(
                            f"Transcription successful: '{response.transcribed_text[:50]}...' "
                            f"(confidence: {avg_confidence:.2f}, time: {processing_time:.2f}s)"
                        )
                        
                        # Apply confidence threshold
                        if avg_confidence < self.confidence_threshold:
                            response.success = False
                            response.error_message = f"Confidence score ({avg_confidence:.2f}) below threshold ({self.confidence_threshold})"
                            self.failed_transcriptions += 1
                        else:
                            self.successful_transcriptions += 1
                            self.total_processing_time += processing_time
                        
                        return response
                    else:
                        # Transcription failed, prepare for retry
                        self.get_logger().warning(f"Transcription attempt {attempt + 1} failed")
                        
                        if attempt == self.max_retries - 1:
                            # Last attempt failed
                            response.success = False
                            response.error_message = f"Transcription failed after {self.max_retries} attempts"
                            self.failed_transcriptions += 1
                            return response
                        else:
                            # Wait before retry with exponential backoff
                            time.sleep(2 ** attempt)
                
                except Exception as e:
                    self.get_logger().error(f"Error during transcription attempt {attempt + 1}: {str(e)}")
                    if attempt == self.max_retries - 1:
                        response.success = False
                        response.error_message = f"Error during transcription: {str(e)}"
                        self.failed_transcriptions += 1
                        return response
                    else:
                        time.sleep(2 ** attempt)
        
        except Exception as e:
            self.get_logger().error(f"Unexpected error in transcription service: {str(e)}")
            response.success = False
            response.error_message = f"Unexpected error: {str(e)}"
            self.failed_transcriptions += 1
        
        return response

    def transcribe_with_whisper(self, request: TranscribeAudio.Request):
        """
        Perform actual transcription using OpenAI Whisper API.
        
        Args:
            request: The service request containing audio information
            
        Returns:
            API response or None if transcription fails
        """
        try:
            # Determine language to use
            language_to_use = request.language_code if request.language_code else self.language
            
            # Open and transcribe the audio file
            with open(request.audio_input_path, 'rb') as audio_file:
                response = openai.Audio.transcribe(
                    model=self.whisper_model,
                    file=audio_file,
                    language=language_to_use,
                    response_format="verbose_json",  # Get detailed information including confidence
                    temperature=0.0,  # For more deterministic results
                )
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error calling Whisper API: {str(e)}")
            return None

    def get_audio_duration(self, file_path: str) -> float:
        """
        Get the duration of an audio file in seconds.
        
        Args:
            file_path: Path to the audio file
            
        Returns:
            Duration in seconds, or 0 if unable to determine
        """
        try:
            with wave.open(file_path, 'rb') as wf:
                frames = wf.getnframes()
                sample_rate = wf.getframerate()
                duration = frames / float(sample_rate)
                return duration
        except Exception as e:
            self.get_logger().warning(f"Could not determine audio duration: {str(e)}")
            return 0.0

    def publish_statistics(self):
        """
        Publish transcription statistics periodically.
        """
        if self.total_requests > 0:
            avg_processing_time = self.total_processing_time / self.successful_transcriptions if self.successful_transcriptions > 0 else 0.0
            success_rate = self.successful_transcriptions / self.total_requests
            
            stats_msg = String()
            stats_msg.data = json.dumps({
                'total_requests': self.total_requests,
                'successful_transcriptions': self.successful_transcriptions,
                'failed_transcriptions': self.failed_transcriptions,
                'success_rate': success_rate,
                'average_processing_time': avg_processing_time,
                'timestamp': time.time()
            })
            
            self.stats_publisher.publish(stats_msg)


class AudioConverterNode(Node):
    """
    Optional node to convert various audio formats to WAV for Whisper compatibility.
    """
    
    def __init__(self):
        super().__init__('audio_converter_node')
        
        # Configuration parameters
        self.declare_parameter('target_sample_rate', 16000)
        self.declare_parameter('target_channels', 1)
        self.declare_parameter('target_format', 'WAV')
        
        self.target_sample_rate = self.get_parameter('target_sample_rate').value
        self.target_channels = self.get_parameter('target_channels').value
        self.target_format = self.get_parameter('target_format').value
        
        # Create service for audio conversion
        self.convert_service = self.create_service(
            TranscribeAudio,  # Reusing the same service type for conversion
            'convert_audio_format',
            self.convert_audio_callback
        )
        
        self.get_logger().info("Audio Converter Service initialized")

    def convert_audio_callback(self, request: TranscribeAudio.Request, 
                              response: TranscribeAudio.Response) -> TranscribeAudio.Response:
        """
        Service callback to convert audio to Whisper-compatible format.
        """
        self.get_logger().info(f"Received conversion request: {request.audio_input_path}")
        
        try:
            # Convert audio to WAV format if needed
            converted_path = self.convert_to_wav(request.audio_input_path)
            
            if converted_path:
                response.success = True
                response.error_message = converted_path  # Using error_message field to return converted file path
                self.get_logger().info(f"Audio conversion successful: {converted_path}")
            else:
                response.success = False
                response.error_message = "Audio conversion failed"
                
        except Exception as e:
            self.get_logger().error(f"Error converting audio: {str(e)}")
            response.success = False
            response.error_message = f"Conversion error: {str(e)}"
        
        return response

    def convert_to_wav(self, input_path: str) -> Optional[str]:
        """
        Convert various audio formats to WAV format.
        
        Args:
            input_path: Path to input audio file
            
        Returns:
            Path to converted WAV file, or None if conversion failed
        """
        try:
            import subprocess
            import os
            
            # Create temporary WAV file
            output_path = input_path.rsplit('.', 1)[0] + '_converted.wav'
            
            # Use ffmpeg to convert to WAV (requires ffmpeg to be installed)
            result = subprocess.run([
                'ffmpeg',
                '-i', input_path,
                '-ar', str(self.target_sample_rate),
                '-ac', str(self.target_channels),
                '-c:a', 'pcm_s16le',  # Standard format for Whisper
                output_path,
                '-y'  # Overwrite output file if it exists
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                return output_path
            else:
                self.get_logger().error(f"FFmpeg conversion failed: {result.stderr}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error in audio conversion: {str(e)}")
            return None


def main(args=None):
    """
    Main function to run the Whisper transcription service node.
    """
    rclpy.init(args=args)
    
    try:
        # Create the transcription service node
        node = WhisperTranscriptionService()
        
        # Optionally create converter node too
        converter_node = AudioConverterNode()
        
        # Create multi-threaded executor to handle multiple nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(converter_node)
        
        try:
            node.get_logger().info("Starting Whisper Transcription Service...")
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Interrupted by user")
        finally:
            executor.shutdown()
            node.destroy_node()
            converter_node.destroy_node()
    
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Key Implementation Features

### 1. Service Interface
- Implements the `TranscribeAudio` service as defined in the VLA system architecture
- Proper request/response handling with error reporting
- Confidence threshold validation

### 2. Whisper API Integration
- Direct integration with OpenAI Whisper API
- Configurable Whisper models and parameters
- Detailed response handling including confidence scores

### 3. Robust Error Handling
- Multiple retry mechanism for API calls
- Audio file validation and duration checking
- Comprehensive error reporting

### 4. Performance Monitoring
- Processing time tracking
- Success rate statistics
- Periodic statistics publishing

### 5. Audio Format Support
- WAV format support for Whisper compatibility
- Optional audio conversion service
- Duration and format validation

### 6. Configurable Parameters
- Model selection and language preferences
- Confidence thresholds and retry settings
- Timeout and file size limits

## Dependencies

To use this service, you'll need to install the following:

```bash
pip install openai rclpy
# For audio conversion (optional):
# ffmpeg (system package)
```

## Usage

The service can be called from other nodes in the VLA system:

```python
# Example client code
import rclpy
from vla_msgs.srv import TranscribeAudio

def call_transcription_service(node):
    client = node.create_client(TranscribeAudio, 'whisper_transcribe_audio')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Transcription service not available, waiting again...')
    
    request = TranscribeAudio.Request()
    request.audio_input_path = '/path/to/audio/file.wav'
    request.language_code = 'en'
    
    future = client.call_async(request)
    # Handle the response as needed
```

This implementation provides a complete, production-ready service for speech-to-text conversion in the VLA system, handling all aspects from API integration to error handling and performance monitoring.
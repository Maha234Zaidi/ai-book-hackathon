# Error Handling for Voice Recognition Failures

This document provides comprehensive error handling strategies for voice recognition failures in the VLA system. It covers detection, classification, recovery, and prevention of voice recognition errors.

## Overview of Voice Recognition Errors

Voice recognition systems in the VLA system can fail due to various factors including audio quality issues, API limitations, network problems, and environmental conditions. Proper error handling is essential for maintaining system reliability and user experience.

## Types of Voice Recognition Errors

### 1. Audio Input Errors

#### Common Issues:
- Microphone hardware failures
- Audio driver problems
- Permission denials
- Audio format incompatibilities

#### Implementation:
```python
class AudioInputError(Exception):
    """Base exception for audio input errors"""
    pass

class MicrophoneUnavailableError(AudioInputError):
    """Raised when microphone is not accessible"""
    pass

class AudioFormatError(AudioInputError):
    """Raised when audio format is incompatible"""
    pass

def handle_microphone_error(self):
    """Handle microphone unavailability gracefully"""
    self.logger.warning("Microphone unavailable, switching to file input mode")
    
    # Change to file-based input if available
    if hasattr(self, 'file_input_enabled') and self.file_input_enabled:
        self.switch_to_file_input()
        self.status_publisher.publish(String(data="switched_to_file_input"))
    else:
        # Publish error status
        self.status_publisher.publish(String(data="audio_input_error"))
    
    # Attempt to reconnect later
    self.create_timer(5.0, self.retry_microphone_connection)

def retry_microphone_connection(self):
    """Attempt to reinitialize microphone connection"""
    try:
        # Reinitialize audio stream
        if hasattr(self, 'audio_stream') and self.audio_stream.is_active():
            self.audio_stream.stop_stream()
            self.audio_stream.close()
        
        self.audio_stream = self.pyaudio_instance.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            input_device_index=self.input_device_index,
            stream_callback=self.audio_callback
        )
        self.audio_stream.start_stream()
        
        self.get_logger().info("Microphone reconnection successful")
        self.status_publisher.publish(String(data="audio_input_restored"))
        
    except Exception as e:
        self.get_logger().error(f"Failed to reconnect microphone: {str(e)}")
        # Schedule another retry
        self.create_timer(10.0, self.retry_microphone_connection)
```

### 2. API Communication Errors

#### Common Issues:
- OpenAI API rate limits
- Network connectivity problems
- Authentication failures
- Service unavailability

#### Implementation:
```python
from openai.error import RateLimitError, AuthenticationError, APIError
import requests
from requests.exceptions import RequestException, Timeout, ConnectionError

class WhisperAPIError(Exception):
    """Base exception for Whisper API errors"""
    def __init__(self, message, error_type=None):
        super().__init__(message)
        self.error_type = error_type

class RateLimitExceededError(WhisperAPIError):
    """Raised when API rate limits are exceeded"""
    pass

class AuthenticationFailedError(WhisperAPIError):
    """Raised when API authentication fails"""
    pass

def robust_transcription_with_retry(self, audio_file_path, max_retries=3):
    """
    Transcribe audio with comprehensive error handling and retry logic.
    """
    last_error = None
    
    for attempt in range(max_retries):
        try:
            self.get_logger().debug(f"Transcription attempt {attempt + 1}/{max_retries}")
            
            # Make API call
            with open(audio_file_path, 'rb') as audio_file:
                response = openai.Audio.transcribe(
                    model=self.whisper_model,
                    file=audio_file,
                    language=self.language,
                    response_format="verbose_json",
                    temperature=0.0,
                )
            
            # If successful, return the result
            return response
            
        except RateLimitError as e:
            last_error = RateLimitExceededError(f"Rate limit exceeded: {str(e)}")
            wait_time = 2 ** attempt  # Exponential backoff
            self.get_logger().warning(f"Rate limit exceeded, waiting {wait_time}s: {str(e)}")
            
        except AuthenticationError as e:
            last_error = AuthenticationFailedError(f"Authentication failed: {str(e)}")
            self.get_logger().error(f"API authentication failed: {str(e)}")
            # Authentication errors are not retryable
            break
            
        except ConnectionError as e:
            last_error = WhisperAPIError(f"Connection error: {str(e)}", "connection")
            wait_time = 2 ** attempt
            self.get_logger().warning(f"Connection error, waiting {wait_time}s: {str(e)}")
            
        except Timeout as e:
            last_error = WhisperAPIError(f"Request timeout: {str(e)}", "timeout")
            wait_time = 2 ** attempt
            self.get_logger().warning(f"Request timeout, waiting {wait_time}s: {str(e)}")
            
        except APIError as e:
            last_error = WhisperAPIError(f"OpenAI API error: {str(e)}", "api")
            wait_time = 2 ** attempt
            self.get_logger().warning(f"API error, waiting {wait_time}s: {str(e)}")
            
        except Exception as e:
            last_error = WhisperAPIError(f"Unexpected error: {str(e)}", "unexpected")
            wait_time = 2 ** attempt
            self.get_logger().error(f"Unexpected error, waiting {wait_time}s: {str(e)}")
        
        # Wait before retry
        if attempt < max_retries - 1:
            time.sleep(wait_time)
    
    # If all retries failed, raise the last error
    if last_error:
        self.get_logger().error(f"All transcription retries failed: {str(last_error)}")
        raise last_error
    
    return None

def handle_transcription_error(self, error):
    """
    Handle transcription errors with appropriate recovery strategies.
    """
    error_type = type(error).__name__
    self.get_logger().error(f"Transcription error [{error_type}]: {str(error)}")
    
    # Update statistics
    self.failed_transcriptions += 1
    
    # Determine appropriate recovery action based on error type
    if isinstance(error, RateLimitExceededError):
        # Rate limit exceeded - throttle further requests
        self.throttle_requests()
        self.status_publisher.publish(String(data="rate_limit_exceeded"))
        
    elif isinstance(error, AuthenticationFailedError):
        # Authentication failure - alert system administrator
        self.status_publisher.publish(String(data="authentication_error"))
        # This is likely a configuration issue, not automatically recoverable
        
    elif isinstance(error, (ConnectionError, Timeout)):
        # Network-related errors - adjust timeout and retry strategy
        self.adjust_network_parameters()
        self.status_publisher.publish(String(data="network_error"))
        
    else:
        # Other errors - log and continue
        self.status_publisher.publish(String(data="transcription_error"))

def throttle_requests(self):
    """Throttle API requests when rate limits are approached."""
    # Back off by reducing request frequency
    if hasattr(self, 'transcription_timer'):
        # Double the timer period temporarily
        current_period = self.transcription_timer.timer_period_ns / 1e9
        new_period = min(current_period * 2, 5.0)  # Max 5 second delay
        self.transcription_timer.destroy()
        self.transcription_timer = self.create_timer(new_period, self.process_pending_requests)
        self.get_logger().info(f"Request throttling: increased period to {new_period}s")

def adjust_network_parameters(self):
    """Adjust network-related parameters after connection errors."""
    # For example, increase timeout values
    self.request_timeout = min(self.request_timeout * 1.5, 300.0)  # Max 5 minutes
```

### 3. Low-Quality Audio Errors

#### Common Issues:
- Background noise
- Poor microphone placement
- Low speech volume
- Audio distortion

#### Implementation:
```python
import numpy as np

class AudioQualityError(Exception):
    """Raised when audio quality is insufficient for reliable transcription"""
    pass

def evaluate_audio_quality(self, audio_data):
    """
    Evaluate audio quality and determine if it's suitable for transcription.
    
    Returns:
        tuple: (is_sufficient, quality_metrics)
    """
    # Convert bytes to numpy array for analysis
    audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
    
    # Calculate quality metrics
    metrics = {
        'rms_energy': np.sqrt(np.mean(audio_array ** 2)),
        'zero_crossing_rate': np.mean(audio_array[1:] * audio_array[:-1] < 0),
        'signal_to_noise_ratio': self.estimate_snr(audio_array),
        'peak_amplitude': np.max(np.abs(audio_array)),
        'duration': len(audio_array) / self.sample_rate
    }
    
    # Check against thresholds
    is_sufficient = (
        metrics['rms_energy'] > self.min_audio_energy and
        metrics['peak_amplitude'] > self.min_peak_amplitude and
        metrics['duration'] >= self.min_audio_duration
    )
    
    return is_sufficient, metrics

def estimate_snr(self, audio_array):
    """
    Estimate signal-to-noise ratio (simplified approach).
    """
    # Calculate approximate SNR by comparing signal power to noise floor estimate
    signal_power = np.mean(audio_array ** 2)
    noise_floor = np.median(np.abs(audio_array))  # Robust estimate of noise
    noise_power = noise_floor ** 2
    
    if noise_power == 0:
        return float('inf')  # Perfect signal
    
    snr = 10 * np.log10(signal_power / noise_power) if noise_power > 0 else 0
    return snr

def preprocess_audio_with_quality_check(self, audio_chunk):
    """
    Preprocess audio with quality assessment.
    """
    is_sufficient, metrics = self.evaluate_audio_quality(audio_chunk)
    
    if not is_sufficient:
        # Log the quality issue
        self.get_logger().warning(
            f"Low audio quality detected: energy={metrics['rms_energy']:.3f}, "
            f"duration={metrics['duration']:.2f}s, "
            f"peak={metrics['peak_amplitude']:.0f}"
        )
        
        # Decide whether to attempt transcription based on quality
        if metrics['rms_energy'] < self.critical_audio_energy:
            # Audio quality is too poor to attempt transcription
            return None
    
    # Apply preprocessing
    audio_array = np.frombuffer(audio_chunk, dtype=np.int16)
    
    # Noise reduction
    if self.enable_noise_reduction:
        audio_array = self.apply_noise_reduction(audio_array)
    
    # Normalize
    audio_array = self.normalize_audio(audio_array)
    
    # Convert back to bytes
    return audio_array.tobytes()

def provide_audio_quality_feedback(self, quality_metrics):
    """
    Provide feedback to user about audio quality.
    """
    feedback = []
    
    if quality_metrics['rms_energy'] < self.min_audio_energy:
        feedback.append("Audio level too low - please speak louder")
    
    if quality_metrics['duration'] < self.min_audio_duration:
        feedback.append("Audio too short - please speak longer phrases")
    
    if quality_metrics['zero_crossing_rate'] > self.max_zero_crossing_rate:
        feedback.append("Background noise detected - please move to quieter location")
    
    if feedback:
        feedback_msg = "Audio quality issues: " + "; ".join(feedback)
        self.status_publisher.publish(String(data=feedback_msg))
```

### 4. Confidence-Based Error Handling

#### Common Issues:
- Low-confidence transcriptions
- Uncertainty in recognition
- Ambiguous commands

#### Implementation:
```python
def handle_confidence_threshold(self, transcription_result, required_threshold=0.8):
    """
    Handle transcription based on confidence level.
    
    Args:
        transcription_result: Result from Whisper API
        required_threshold: Minimum confidence threshold
        
    Returns:
        tuple: (is_acceptable, confidence_score)
    """
    text = transcription_result.get("text", "")
    
    # Calculate confidence from segments if available
    segments = transcription_result.get("segments", [])
    if segments:
        # Calculate average confidence from segments
        avg_confidence = sum(
            segment.get("confidence", 0.0) for segment in segments
        ) / len(segments) if len(segments) > 0 else 0.0
    else:
        # Fallback: estimate confidence based on other metrics
        avg_confidence = self.estimate_confidence(text)
    
    # Check against threshold
    is_acceptable = avg_confidence >= required_threshold
    
    self.get_logger().info(
        f"Transcription confidence: {avg_confidence:.3f} "
        f"({'acceptable' if is_acceptable else 'below threshold'})"
    )
    
    return is_acceptable, avg_confidence

def estimate_confidence(self, text):
    """
    Estimate confidence when segment-level confidence is not available.
    """
    # Simple heuristics-based confidence estimation
    confidence = 1.0
    
    # Penalize for signs of poor quality
    if len(text.strip()) < 5:
        confidence *= 0.7  # Short response may be low quality
    
    # Check for common transcription artifacts
    if re.search(r'\b(?:you know|um|uh|like)\b', text, re.IGNORECASE):
        # Filler words are normal, don't penalize heavily
        confidence *= 0.9
    
    # Check for unusual character patterns
    unusual_chars = set(text) - set(string.printable)
    if unusual_chars:
        # Unusual characters suggest poor transcription
        confidence *= 0.5
    
    # Length-based adjustment
    if len(text) > 100:  # Long responses may have errors
        confidence *= 0.9
    
    return max(0.0, min(1.0, confidence))

def request_clarification_if_uncertain(self, transcription, confidence):
    """
    Request clarification for low-confidence transcriptions.
    """
    if confidence < self.clarification_threshold:
        self.get_logger().info(f"Requesting clarification for: {transcription}")
        
        # Publish request for clarification
        clar_msg = String()
        clar_msg.data = f"clarification_needed: {transcription}"
        self.status_publisher.publish(clar_msg)
        
        # Optionally trigger a follow-up request via text-to-speech
        # self.speak_request_for_clarification()
        
        return True
    
    return False
```

### 5. Comprehensive Error Recovery System

#### Implementation:
```python
class VoiceRecognitionErrorRecovery:
    """
    Comprehensive error recovery system for voice recognition.
    """
    
    def __init__(self, node):
        self.node = node
        self.recovery_states = {}
        self.max_recovery_attempts = 3
        self.recovery_timeout = 30.0  # seconds
        
    def process_with_error_recovery(self, audio_data):
        """
        Process audio with integrated error recovery.
        """
        try:
            # Quality check
            is_sufficient, quality_metrics = self.node.evaluate_audio_quality(audio_data)
            if not is_sufficient:
                self.handle_quality_issue(quality_metrics)
                return None
            
            # Attempt transcription
            result = self.node.robust_transcription_with_retry(audio_data)
            
            if result:
                # Confidence check
                is_acceptable, confidence = self.node.handle_confidence_threshold(
                    result, self.node.confidence_threshold
                )
                
                if is_acceptable:
                    # Check if clarification is needed
                    if not self.node.request_clarification_if_uncertain(result.get("text", ""), confidence):
                        return result
                else:
                    self.handle_low_confidence(result, confidence)
            
        except WhisperAPIError as e:
            self.handle_api_error(e)
        except AudioInputError as e:
            self.handle_audio_error(e)
        except Exception as e:
            self.handle_unexpected_error(e)
        
        return None
    
    def handle_quality_issue(self, quality_metrics):
        """Handle low audio quality issues."""
        self.node.provide_audio_quality_feedback(quality_metrics)
        
    def handle_api_error(self, error):
        """Handle API-related errors."""
        self.node.handle_transcription_error(error)
        
        # Attempt recovery based on error type
        if isinstance(error, RateLimitExceededError):
            # Wait and queue for later processing
            self.schedule_retry_for_rate_limit()
        else:
            # Other API errors - log and continue
            pass
    
    def handle_audio_error(self, error):
        """Handle audio input errors."""
        self.node.logger.error(f"Audio error: {str(error)}")
        
        # Attempt to recover audio input
        if hasattr(self.node, 'retry_microphone_connection'):
            self.node.create_timer(5.0, self.node.retry_microphone_connection)
    
    def handle_low_confidence(self, result, confidence):
        """Handle low-confidence transcriptions."""
        text = result.get("text", "")
        self.node.logger.warning(f"Low confidence transcription ({confidence:.3f}): {text}")
        
        # Add to low-confidence queue for potential user confirmation
        self.add_to_confirmation_queue(text, confidence)
    
    def handle_unexpected_error(self, error):
        """Handle unexpected errors."""
        self.node.logger.error(f"Unexpected error in voice recognition: {str(error)}")
        
        # Increment error counter and potentially take more aggressive action
        if hasattr(self.node, 'error_recovery_counter'):
            self.node.error_recovery_counter += 1
            
            # If too many errors in a short time, reduce processing load
            if self.node.error_recovery_counter > 10:
                self.node.logger.warning("High error rate detected, reducing processing frequency")
                self.reduce_processing_frequency()
    
    def schedule_retry_for_rate_limit(self):
        """Schedule transcription retry after rate limit."""
        # In a real implementation, this would queue the audio for later processing
        pass
    
    def add_to_confirmation_queue(self, text, confidence):
        """Add low-confidence result to confirmation queue."""
        # Implementation would add to a queue for later user confirmation
        pass
    
    def reduce_processing_frequency(self):
        """Reduce processing frequency to reduce error rate."""
        # Implementation would adjust processing parameters
        pass
```

## Testing Error Handling

```python
import unittest
from unittest.mock import Mock, patch, MagicMock

class TestVoiceRecognitionErrorHandling(unittest.TestCase):
    
    def setUp(self):
        # Set up test node
        self.node = VoiceRecognitionNode()  # Assuming this class exists
        self.recovery_system = VoiceRecognitionErrorRecovery(self.node)
    
    def test_audio_quality_evaluation(self):
        """Test audio quality evaluation with various inputs."""
        # Generate test audio data
        test_audio = b'\x00\x00' * 1000  # Silent audio
        
        is_sufficient, metrics = self.node.evaluate_audio_quality(test_audio)
        
        # Silent audio should have low energy
        self.assertLess(metrics['rms_energy'], 0.1)
        self.assertFalse(is_sufficient)
    
    @patch('openai.Audio.transcribe')
    def test_api_error_handling(self, mock_transcribe):
        """Test handling of various API errors."""
        mock_transcribe.side_effect = RateLimitError("Rate limit exceeded")
        
        with self.assertRaises(RateLimitExceededError):
            self.node.robust_transcription_with_retry("dummy.wav")
    
    def test_confidence_threshold(self):
        """Test confidence threshold handling."""
        test_result = {
            "text": "test transcription",
            "segments": [
                {"text": "test", "confidence": 0.9},
                {"text": "transcription", "confidence": 0.6}
            ]
        }
        
        is_acceptable, confidence = self.node.handle_confidence_threshold(
            test_result, required_threshold=0.7
        )
        
        self.assertFalse(is_acceptable)  # Average confidence < 0.7
        self.assertAlmostEqual(confidence, 0.75, places=2)  # (0.9+0.6)/2
    
    def tearDown(self):
        self.node.destroy_node()

# Example of integration with ROS 2 testing
def test_voice_recognition_with_errors():
    """
    Function to test voice recognition error handling with ROS 2 infrastructure.
    """
    rclpy.init()
    
    try:
        node = VoiceRecognitionNode()
        recovery_system = VoiceRecognitionErrorRecovery(node)
        
        # Test quality checking
        # (In a real test, you'd use simulated audio data)
        
        # Test error recovery
        # result = recovery_system.process_with_error_recovery(test_audio_data)
        
        node.destroy_node()
        
    finally:
        rclpy.shutdown()
```

## Error Monitoring and Reporting

```python
class VoiceRecognitionErrorMonitor:
    """
    Monitor and report voice recognition errors.
    """
    
    def __init__(self, node):
        self.node = node
        self.error_counts = {}
        self.error_timestamps = []
        self.max_error_history = 100
        
    def log_error(self, error_type, error_message):
        """Log an error with timestamp."""
        timestamp = time.time()
        
        # Update error counts
        if error_type not in self.error_counts:
            self.error_counts[error_type] = 0
        self.error_counts[error_type] += 1
        
        # Add to history
        self.error_timestamps.append((timestamp, error_type, error_message))
        
        # Maintain history size
        if len(self.error_timestamps) > self.max_error_history:
            self.error_timestamps = self.error_timestamps[-self.max_error_history:]
        
        # Publish error statistics
        self.publish_error_statistics()
    
    def publish_error_statistics(self):
        """Publish error statistics via ROS topic."""
        stats = {
            'error_counts': self.error_counts.copy(),
            'recent_errors': len(self.error_timestamps),
            'timestamp': time.time()
        }
        
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.node.error_stats_publisher.publish(stats_msg)
    
    def get_error_rate(self, time_window=60):
        """Get error rate in the specified time window (in seconds)."""
        current_time = time.time()
        recent_errors = [
            ts for ts, _, _ in self.error_timestamps
            if current_time - ts <= time_window
        ]
        return len(recent_errors) / time_window if time_window > 0 else 0
    
    def is_error_rate_excessive(self):
        """Check if error rate is excessive and system should take action."""
        return self.get_error_rate() > 0.1  # More than 1 error per 10 seconds

# Integration with the main node
class EnhancedVoiceRecognitionNode(VoiceRecognitionNode):
    """
    Voice recognition node with enhanced error monitoring.
    """
    
    def __init__(self):
        super().__init__()
        
        # Initialize error monitor
        self.error_monitor = VoiceRecognitionErrorMonitor(self)
        
        # Create publisher for error statistics
        self.error_stats_publisher = self.create_publisher(
            String,
            '/vla/error_statistics',
            10
        )
        
        # Timer for periodic health checks
        self.health_check_timer = self.create_timer(10.0, self.perform_health_check)
    
    def perform_health_check(self):
        """Perform periodic health checks."""
        if self.error_monitor.is_error_rate_excessive():
            self.get_logger().warning("High error rate detected, performing recovery actions")
            
            # Take appropriate recovery actions
            self.throttle_requests()
```

This comprehensive error handling system ensures that voice recognition failures in the VLA system are properly detected, classified, and recovered from, maintaining system reliability and user experience even under adverse conditions.
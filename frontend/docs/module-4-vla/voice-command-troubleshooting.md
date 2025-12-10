# Troubleshooting Section for Common Voice Command Issues

This document provides a comprehensive troubleshooting guide for common issues encountered when implementing and using voice command systems in VLA systems. It covers diagnosis, solutions, and preventive measures for various problems.

## Overview

Voice command systems can encounter various issues that affect their functionality and user experience. This troubleshooting guide helps identify, diagnose, and resolve these issues efficiently. The guide is organized by issue categories and includes both immediate solutions and preventive measures.

## Common Audio Input Issues

### 1. No Audio Detected

**Symptoms:**
- Microphone shows as active but no audio input
- Voice commands are not being recognized
- Status indicates "no audio input"

**Possible Causes:**
- Microphone not properly connected
- Audio driver issues
- Incorrect audio device selected
- Microphone permissions not granted

**Solutions:**
```python
# Check audio input configuration
def check_audio_input_config():
    """
    Helper function to verify audio input configuration
    """
    import pyaudio
    import rclpy
    from rclpy.node import Node
    
    class AudioCheckNode(Node):
        def __init__(self):
            super().__init__('audio_check_node')
            
        def check_audio_devices(self):
            """
            Check all available audio devices
            """
            pa = pyaudio.PyAudio()
            info = pa.get_host_api_info_by_index(0)
            n_devices = info.get('deviceCount')
            
            self.get_logger().info(f"Found {n_devices} audio devices:")
            for i in range(n_devices):
                device_info = pa.get_device_info_by_host_api_device_index(0, i)
                device_type = "INPUT" if device_info.get('maxInputChannels') > 0 else "OUTPUT"
                self.get_logger().info(f"  {i}: {device_info.get('name')} ({device_type})")
            
            pa.terminate()
    
    # Example usage
    rclpy.init()
    node = AudioCheckNode()
    node.check_audio_devices()
    node.destroy_node()
    rclpy.shutdown()

# Set correct audio device
def set_specific_audio_device(device_index):
    """
    Configure the system to use a specific audio device
    """
    # In your audio input node:
    self.input_device_index = device_index
```

**Prevention:**
- Always check available devices during setup
- Implement automatic device selection if default fails
- Monitor device availability during operation

### 2. Poor Audio Quality

**Symptoms:**
- Voice recognition failing frequently
- Transcriptions with errors
- Choppy or distorted audio input

**Possible Causes:**
- Background noise interference
- Low microphone sensitivity
- Audio clipping or saturation
- Poor microphone placement

**Solutions:**
```python
def enhance_audio_quality(audio_data, sample_rate=16000):
    """
    Apply audio enhancement techniques
    """
    import numpy as np
    from scipy import signal
    
    # Convert to numpy array
    audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
    
    # Apply noise reduction (simplified approach)
    # In practice, use libraries like noisereduce or specialized algorithms
    if len(audio_array) > 0:
        # Normalize audio
        max_val = np.max(np.abs(audio_array))
        if max_val > 0:
            audio_array = audio_array / max_val
        
        # Apply simple high-pass filter to remove low-frequency noise
        nyquist = sample_rate / 2
        cutoff_freq = 300  # Hz
        b, a = signal.butter(4, cutoff_freq / nyquist, btype='high')
        audio_array = signal.filtfilt(b, a, audio_array)
    
    return audio_array.astype(np.int16).tobytes()

def detect_audio_clipping(audio_data):
    """
    Detect if audio is clipping
    """
    import numpy as np
    
    audio_array = np.frombuffer(audio_data, dtype=np.int16)
    
    # Define clipping as values at the maximum range
    max_val = np.max(np.abs(audio_array))
    threshold = 0.95 * 32767  # 95% of max int16 value
    
    clipping_detected = max_val > threshold
    
    if clipping_detected:
        clipping_percentage = np.sum(np.abs(audio_array) >= threshold) / len(audio_array) * 100
        return True, clipping_percentage
    else:
        return False, 0
```

**Configuration Settings:**
```yaml
# In your configuration YAML
audio_input_node:
  ros__parameters:
    enable_noise_reduction: true
    vad_threshold: 0.01  # Voice activity detection threshold
    audio_gain: 1.5      # Amplification factor
    clipping_threshold: 95  # Percentage of max value considered clipping
```

## Common Recognition Issues

### 1. Low Transcription Accuracy

**Symptoms:**
- Commands frequently misunderstood
- Transcriptions with many errors
- System asking for repetitions frequently

**Possible Causes:**
- Low audio quality
- Whisper model limitations
- Language mismatch
- Confidence threshold too low

**Solutions:**
```python
class TranscriptionQualityAnalyzer:
    """
    Analyze and improve transcription quality
    """
    
    def __init__(self, node):
        self.node = node
        self.transcription_history = []
        self.error_patterns = {}
    
    def analyze_transcription_quality(self, original_text, transcribed_text):
        """
        Analyze quality of transcription
        """
        import difflib
        
        # Calculate similarity ratio
        similarity = difflib.SequenceMatcher(
            None, original_text.lower(), transcribed_text.lower()
        ).ratio()
        
        # Identify common errors
        self.identify_error_patterns(original_text, transcribed_text)
        
        return {
            'similarity_ratio': similarity,
            'original_length': len(original_text),
            'transcribed_length': len(transcribed_text),
            'error_patterns': self.error_patterns.copy()
        }
    
    def identify_error_patterns(self, original, transcribed):
        """
        Identify common transcription error patterns
        """
        import re
        
        # Add to history
        self.transcription_history.append({
            'original': original,
            'transcribed': transcribed
        })
        
        # Simple pattern matching for common errors
        if len(self.transcription_history) > 10:  # Only analyze with sufficient data
            # Look for common substitution patterns
            for item in self.transcription_history[-10:]:  # Last 10 items
                orig_words = item['original'].lower().split()
                trans_words = item['transcribed'].lower().split()
                
                # Simple alignment to find potential substitutions
                for i, (o, t) in enumerate(zip(orig_words, trans_words)):
                    if o != t:
                        pattern_key = f"'{o}' vs '{t}'"
                        if pattern_key not in self.error_patterns:
                            self.error_patterns[pattern_key] = 0
                        self.error_patterns[pattern_key] += 1

def improve_transcription_accuracy(node, audio_file_path):
    """
    Apply multiple techniques to improve transcription accuracy
    """
    try:
        import openai
        import tempfile
        import os
        
        # Try different models and parameters
        models_to_try = ['whisper-1', 'whisper-1']  # Whisper-1 is the current best
        results = []
        
        for model in models_to_try:
            try:
                with open(audio_file_path, 'rb') as audio_file:
                    result = openai.Audio.transcribe(
                        model=model,
                        file=audio_file,
                        response_format="verbose_json",
                        temperature=0.0,  # More deterministic
                        language=node.language if hasattr(node, 'language') else 'en'
                    )
                
                results.append({
                    'model': model,
                    'text': result.get('text', ''),
                    'confidence': calculate_transcription_confidence(result)
                })
            except Exception as e:
                node.get_logger().warning(f"Model {model} failed: {str(e)}")
        
        # Return the result with highest confidence
        if results:
            best_result = max(results, key=lambda x: x['confidence'])
            return best_result
        
        return None
        
    except Exception as e:
        node.get_logger().error(f"Error in improved transcription: {str(e)}")
        return None

def calculate_transcription_confidence(transcription_result):
    """
    Calculate confidence from transcription result
    """
    segments = transcription_result.get('segments', [])
    
    if segments:
        # Calculate average confidence across segments
        avg_confidence = sum(
            segment.get('confidence', 0.5) for segment in segments
        ) / len(segments) if len(segments) > 0 else 0.0
        return avg_confidence
    
    # Fallback: estimate from other metrics
    text = transcription_result.get('text', '')
    if len(text) < 5:  # Very short responses might be unreliable
        return 0.3
    return 0.8  # Default confidence
```

### 2. High API Error Rates

**Symptoms:**
- Frequent "rate limit exceeded" errors
- API requests failing regularly
- Service unavailable messages

**Possible Causes:**
- Exceeding API rate limits
- Network connectivity issues
- Service outages

**Solutions:**
```python
import asyncio
import time
from functools import wraps

def rate_limit_retry(max_retries=3, backoff_factor=1, 
                    status_forcelist=(429, 500, 502, 503, 504)):
    """
    Decorator to retry API calls with exponential backoff
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    if attempt == max_retries - 1:  # Last attempt
                        raise e
                    
                    # Determine wait time with exponential backoff
                    wait_time = backoff_factor * (2 ** attempt)
                    print(f"Attempt {attempt + 1} failed, waiting {wait_time}s: {str(e)}")
                    time.sleep(wait_time)
            
            return None
        return wrapper
    return decorator

class APIThrottler:
    """
    Manage API request throttling to avoid rate limits
    """
    
    def __init__(self, max_requests_per_minute=10):
        self.max_requests_per_minute = max_requests_per_minute
        self.request_timestamps = []
        self.lock = threading.Lock()
    
    def can_make_request(self):
        """
        Check if a request can be made based on rate limits
        """
        with self.lock:
            current_time = time.time()
            
            # Remove timestamps older than 1 minute
            self.request_timestamps = [
                ts for ts in self.request_timestamps
                if current_time - ts < 60
            ]
            
            # Check if we're under the limit
            if len(self.request_timestamps) < self.max_requests_per_minute:
                self.request_timestamps.append(current_time)
                return True
            
            return False
    
    def wait_for_available_slot(self):
        """
        Wait until a request slot is available
        """
        while not self.can_make_request():
            time.sleep(1)  # Wait 1 second before checking again
        return True

# Usage in transcription node
def safe_transcribe_with_throttling(node, audio_file_path, throttler):
    """
    Transcribe with rate limiting
    """
    if throttler.wait_for_available_slot():
        try:
            with open(audio_file_path, 'rb') as audio_file:
                result = openai.Audio.transcribe(
                    model=node.whisper_model,
                    file=audio_file,
                    response_format="text"
                )
            return result
        except Exception as e:
            node.get_logger().error(f"Transcription failed: {str(e)}")
            return None
```

## Command Mapping Issues

### 1. Unrecognized Commands

**Symptoms:**
- Valid commands not being processed
- System responding with "command not understood"
- Pattern matching failing

**Possible Causes:**
- Insufficient command patterns
- Language variations not handled
- Preprocessing affecting recognition

**Solutions:**
```python
class RobustCommandMapper:
    """
    Enhanced command mapper with fuzzy matching
    """
    
    def __init__(self):
        # Extended command patterns with variations
        self.command_patterns = {
            'move_forward': [
                r'move forward',
                r'go forward', 
                r'forward',
                r'straight',
                r'step forward',
                r'go ahead',
                r'proceed forward'
            ],
            'move_backward': [
                r'move backward',
                r'go backward',
                r'backward',
                r'go back',
                r'back',
                r'reverse'
            ],
            'turn_left': [
                r'turn left',
                r'rotate left',
                r'left',
                r'pivot left',
                r'turn counter ?clockwise'
            ],
            'turn_right': [
                r'turn right',
                r'rotate right',
                r'right',
                r'pivot right',
                r'turn clockwise'
            ],
            'grasp': [
                r'pick up',
                r'grasp',
                r'grab',
                r'take',
                r'seize',
                r'lift'
            ]
        }
        
        # Add fuzzy matching capabilities
        self.fuzzy_threshold = 0.7  # 70% similarity required
    
    def fuzzy_match(self, text, pattern):
        """
        Perform fuzzy matching between text and pattern
        """
        import difflib
        
        # Split into words for comparison
        text_words = text.lower().split()
        pattern_words = pattern.lower().split()
        
        # Calculate similarity
        similarity = difflib.SequenceMatcher(None, text, pattern).ratio()
        
        return similarity >= self.fuzzy_threshold
    
    def parse_command_with_fallback(self, text):
        """
        Parse command with multiple fallback strategies
        """
        text_lower = text.lower().strip()
        
        # 1. Exact pattern matching
        for cmd_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                import re
                if re.search(pattern, text_lower):
                    return cmd_type, self.extract_parameters(text_lower, cmd_type)
        
        # 2. Fuzzy matching
        for cmd_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                if self.fuzzy_match(text_lower, pattern):
                    return cmd_type, self.extract_parameters(text_lower, cmd_type)
        
        # 3. Keyword-based matching
        keywords = {
            'forward': 'move_forward',
            'backward': 'move_backward', 
            'left': 'turn_left',
            'right': 'turn_right',
            'pick': 'grasp',
            'grasp': 'grasp',
            'take': 'grasp'
        }
        
        for keyword, cmd_type in keywords.items():
            if keyword in text_lower:
                return cmd_type, self.extract_parameters(text_lower, cmd_type)
        
        # 4. No match found
        return None, {}
    
    def extract_parameters(self, text, command_type):
        """
        Extract parameters for the command
        """
        params = {}
        
        # Extract numeric values
        import re
        numbers = re.findall(r'(\d+\.?\d*)', text)
        if numbers:
            # For now, take the first number as distance/duration
            try:
                params['value'] = float(numbers[0])
            except ValueError:
                pass
        
        # Extract object names
        objects = ['cup', 'ball', 'book', 'box', 'chair', 'table']
        for obj in objects:
            if obj in text:
                params['object'] = obj
                break
        
        # Extract colors
        colors = ['red', 'blue', 'green', 'yellow', 'white', 'black']
        for color in colors:
            if color in text:
                params['color'] = color
                break
        
        return params

# Usage example
mapper = RobustCommandMapper()
result = mapper.parse_command_with_fallback("Could you please go forwards a little bit?")
print(f"Parsed result: {result}")
```

### 2. Wrong Command Execution

**Symptoms:**
- Commands executing incorrectly
- Robot performing unintended actions
- Command parameters not applied correctly

**Possible Causes:**
- Parameter extraction errors
- Command ambiguity
- State management issues

**Solutions:**
```python
class SafeCommandExecutor:
    """
    Execute commands with safety checks
    """
    
    def __init__(self, node):
        self.node = node
        self.command_history = []
        self.max_command_history = 50
    
    def execute_with_validation(self, command_type, params):
        """
        Execute command with safety validation
        """
        # Validate command parameters
        if not self.validate_command(command_type, params):
            self.node.get_logger().warning(f"Invalid command parameters: {command_type}, {params}")
            return False
        
        # Check for potential conflicts with recent commands
        if self.would_conflict_with_recent(command_type, params):
            self.node.get_logger().warning(f"Command would conflict with recent commands: {command_type}")
            return False
        
        # Log command for history
        self.command_history.append({
            'type': command_type,
            'params': params,
            'timestamp': time.time()
        })
        
        # Maintain history size
        if len(self.command_history) > self.max_command_history:
            self.command_history = self.command_history[-self.max_command_history:]
        
        # Execute the validated command
        success = self.execute_command(command_type, params)
        
        if success:
            self.node.get_logger().info(f"Successfully executed: {command_type} with {params}")
        else:
            self.node.get_logger().error(f"Failed to execute: {command_type} with {params}")
        
        return success
    
    def validate_command(self, command_type, params):
        """
        Validate command parameters
        """
        # Check for required parameters
        if command_type == 'move_forward':
            if 'value' in params:
                distance = params['value']
                if distance <= 0 or distance > 10:  # Max 10 meters
                    return False
        
        elif command_type == 'turn_left' or command_type == 'turn_right':
            if 'value' in params:
                angle = params['value']
                if angle <= 0 or angle > 360:  # Angles should be 0-360
                    return False
        
        elif command_type == 'grasp':
            # Ensure object parameter is present for grasp commands
            if 'object' not in params and 'location' not in params:
                return False
        
        return True
    
    def would_conflict_with_recent(self, command_type, params):
        """
        Check if command would conflict with recently executed commands
        """
        # Example: don't move while grasping
        recent_grasps = [
            cmd for cmd in self.command_history 
            if cmd['type'] == 'grasp' and 
            time.time() - cmd['timestamp'] < 5.0  # Last 5 seconds
        ]
        
        if recent_grasps and command_type in ['move_forward', 'move_backward', 'turn_left', 'turn_right']:
            return True
        
        return False
    
    def execute_command(self, command_type, params):
        """
        Execute the actual command
        """
        # Implementation would depend on your specific robot interface
        # This is a simplified example
        try:
            if command_type == 'move_forward':
                distance = params.get('value', 1.0)
                self.move_robot(distance, 0, 0)  # x, y, theta
            elif command_type == 'turn_left':
                angle = params.get('value', 90)
                self.move_robot(0, 0, angle)  # x, y, theta
            elif command_type == 'grasp':
                obj = params.get('object', 'object')
                self.execute_grasp(obj)
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Command execution error: {str(e)}")
            return False
    
    def move_robot(self, x, y, theta):
        """
        Move robot by specified amounts (simplified)
        """
        # Implementation would publish to /cmd_vel or similar
        pass
    
    def execute_grasp(self, object_name):
        """
        Execute grasping action (simplified)
        """
        # Implementation would interface with manipulator
        pass
```

## System Performance Issues

### 1. High Latency

**Symptoms:**
- Long delay between command and action
- System appears unresponsive
- Timeout errors in command execution

**Possible Causes:**
- Network latency to API services
- Insufficient computing resources
- Blocking operations in command processing

**Solutions:**
```python
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor

class AsyncVoiceProcessor:
    """
    Asynchronous voice processing to reduce latency
    """
    
    def __init__(self, node):
        self.node = node
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.task_queue = asyncio.Queue()
        self.is_processing = False
    
    async def process_audio_async(self, audio_data):
        """
        Process audio asynchronously
        """
        # Add to processing queue
        await self.task_queue.put(audio_data)
        
        # Start processing if not already running
        if not self.is_processing:
            self.is_processing = True
            asyncio.create_task(self.process_queue())
        
        return True
    
    async def process_queue(self):
        """
        Process tasks from queue asynchronously
        """
        while not self.task_queue.empty() or self.is_processing:
            try:
                audio_data = await asyncio.wait_for(self.task_queue.get(), timeout=1.0)
                
                # Process in thread pool to avoid blocking
                loop = asyncio.get_event_loop()
                result = await loop.run_in_executor(
                    self.executor, 
                    self.transcribe_and_process, 
                    audio_data
                )
                
                if result:
                    self.node.get_logger().info(f"Processed transcription: {result}")
                
            except asyncio.TimeoutError:
                # Queue is empty, check if we should continue
                if self.task_queue.empty():
                    self.is_processing = False
                    break
                continue
    
    def transcribe_and_process(self, audio_data):
        """
        Transcribe and process audio (runs in thread pool)
        """
        try:
            # Save audio to temporary file for Whisper API
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                # Write audio data to temp file
                # ... implementation details ...
                temp_path = temp_file.name
            
            # Transcribe using Whisper API
            with open(temp_path, 'rb') as audio_file:
                result = openai.Audio.transcribe(
                    model=self.node.whisper_model,
                    file=audio_file
                )
            
            # Clean up temp file
            os.unlink(temp_path)
            
            # Process the transcription result
            if result and result.get('text'):
                text = result['text']
                # Parse and execute command (implementation would go here)
                return text
        
        except Exception as e:
            self.node.get_logger().error(f"Async transcription error: {str(e)}")
            return None

# Performance monitoring
class PerformanceMonitor:
    """
    Monitor system performance and identify bottlenecks
    """
    
    def __init__(self, node):
        self.node = node
        self.metrics = {
            'audio_process_time': [],
            'transcription_time': [],
            'command_parse_time': [],
            'command_execute_time': [],
            'api_call_time': []
        }
        
        # Start monitoring timer
        self.monitor_timer = node.create_timer(5.0, self.report_performance)
    
    def record_metric(self, metric_name, value):
        """
        Record a performance metric
        """
        if metric_name in self.metrics:
            self.metrics[metric_name].append(value)
            
            # Keep only last 100 measurements
            if len(self.metrics[metric_name]) > 100:
                self.metrics[metric_name] = self.metrics[metric_name][-100:]
    
    def report_performance(self):
        """
        Report performance metrics
        """
        for metric_name, values in self.metrics.items():
            if values:
                avg_time = sum(values) / len(values)
                max_time = max(values)
                self.node.get_logger().info(
                    f"{metric_name}: avg={avg_time:.3f}s, max={max_time:.3f}s, samples={len(values)}"
                )
                
                # Alert if performance is degrading
                if avg_time > 2.0:  # Threshold for concern
                    self.node.get_logger().warning(
                        f"Performance degradation detected in {metric_name}: {avg_time:.3f}s"
                    )
```

## Configuration and Environment Issues

### 1. Environment-Specific Problems

**Symptoms:**
- System works in one environment but not another
- Audio quality varies by location
- Network connectivity differs by deployment

**Solutions:**
```yaml
# Example adaptive configuration file
adaptive_voice_system:
  ros__parameters:
    # Environment-specific configurations
    default:
      whisper_model: 'whisper-1'
      confidence_threshold: 0.8
      noise_threshold: 0.01
    
    # For noisy environments like factories
    noisy_environment:
      confidence_threshold: 0.9  # Higher threshold
      enable_noise_reduction: true
      vad_threshold: 0.05       # Higher VAD threshold
    
    # For quiet environments like offices
    quiet_environment:
      confidence_threshold: 0.7  # Lower threshold
      enable_noise_reduction: false
      vad_threshold: 0.005      # Lower VAD threshold

# Configuration switching based on environment detection
def switch_environment_config(node, environment_type):
    """
    Switch configuration based on detected environment
    """
    config = node.get_parameter('adaptive_voice_system').value
    
    if environment_type in config:
        env_config = config[environment_type]
        
        # Update parameters dynamically
        node.set_parameters([
            rclpy.Parameter('confidence_threshold', value=env_config.get('confidence_threshold', 0.8)),
            rclpy.Parameter('vad_threshold', value=env_config.get('vad_threshold', 0.01)),
            rclpy.Parameter('enable_noise_reduction', value=env_config.get('enable_noise_reduction', True))
        ])
        
        node.get_logger().info(f"Switched to {environment_type} configuration")
```

## Diagnostic Tools

### 1. System Health Check

```python
class VoiceSystemDiagnostics:
    """
    Comprehensive diagnostic tools for voice systems
    """
    
    def __init__(self, node):
        self.node = node
        self.diagnostics = {}
    
    def run_full_diagnostic(self):
        """
        Run comprehensive diagnostic check
        """
        self.node.get_logger().info("Starting full diagnostic check...")
        
        results = {
            'audio_input': self.check_audio_input(),
            'api_connection': self.check_api_connection(),
            'command_mapping': self.check_command_mapping(),
            'permissions': self.check_permissions(),
            'network_latency': self.check_network_latency()
        }
        
        # Compile and report results
        self.report_diagnostics(results)
        return results
    
    def check_audio_input(self):
        """
        Check audio input functionality
        """
        try:
            import pyaudio
            
            pa = pyaudio.PyAudio()
            device_count = pa.get_device_count()
            
            if device_count == 0:
                return {'status': 'error', 'message': 'No audio devices found'}
            
            # Test default input device
            default_device = pa.get_default_input_device_info()
            if not default_device:
                return {'status': 'error', 'message': 'No default input device'}
            
            # Try to open and close the device
            stream = pa.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                frames_per_buffer=1024,
                input_device_index=default_device['index'],
                start=False  # Don't start stream for test
            )
            stream.close()
            
            pa.terminate()
            
            return {
                'status': 'ok',
                'message': f"Audio input OK: {default_device['name']}",
                'device_info': default_device
            }
            
        except Exception as e:
            return {'status': 'error', 'message': f"Audio input error: {str(e)}"}
    
    def check_api_connection(self):
        """
        Check connection to Whisper API
        """
        import openai
        import time
        
        start_time = time.time()
        try:
            # Make a simple API call to test connection
            result = openai.Model.list()
            api_time = time.time() - start_time
            
            return {
                'status': 'ok',
                'message': f"API connection OK",
                'response_time': round(api_time, 3)
            }
        except Exception as e:
            return {'status': 'error', 'message': f"API connection failed: {str(e)}"}
    
    def report_diagnostics(self, results):
        """
        Report diagnostic results
        """
        self.node.get_logger().info("DIAGNOSTIC RESULTS:")
        self.node.get_logger().info("=" * 50)
        
        for check_name, result in results.items():
            status_symbol = "✅" if result['status'] == 'ok' else "❌"
            self.node.get_logger().info(f"{status_symbol} {check_name}: {result['message']}")
        
        # Count errors
        errors = sum(1 for r in results.values() if r['status'] == 'error')
        self.node.get_logger().info("=" * 50)
        self.node.get_logger().info(f"Summary: {len(results) - errors} OK, {errors} errors")
```

## Preventive Maintenance

### 1. Regular Checks

```python
class PreventiveMaintenance:
    """
    Schedule regular checks to prevent issues
    """
    
    def __init__(self, node):
        self.node = node
        self.maintenance_timer = node.create_timer(3600.0, self.run_maintenance)  # Daily
    
    def run_maintenance(self):
        """
        Run scheduled maintenance tasks
        """
        self.node.get_logger().info("Running scheduled maintenance...")
        
        # Clean up old log files if needed
        self.cleanup_logs()
        
        # Check system resources
        self.check_system_resources()
        
        # Update statistics
        self.update_performance_stats()
        
        self.node.get_logger().info("Maintenance completed")
    
    def cleanup_logs(self):
        """
        Clean up old diagnostic logs
        """
        import os
        import glob
        from datetime import datetime, timedelta
        
        # Remove logs older than 7 days
        cutoff_date = datetime.now() - timedelta(days=7)
        
        log_files = glob.glob("/tmp/vla_voice_logs/*.log")
        for log_file in log_files:
            if datetime.fromtimestamp(os.path.getctime(log_file)) < cutoff_date:
                os.remove(log_file)
                self.node.get_logger().debug(f"Removed old log: {log_file}")
    
    def check_system_resources(self):
        """
        Check system resources and report if running low
        """
        import psutil
        
        # Check memory usage
        memory_percent = psutil.virtual_memory().percent
        if memory_percent > 80:
            self.node.get_logger().warning(f"High memory usage: {memory_percent}%")
        
        # Check disk space
        disk_percent = psutil.disk_usage('/').percent
        if disk_percent > 85:
            self.node.get_logger().warning(f"Low disk space: {100-disk_percent}% free")
    
    def update_performance_stats(self):
        """
        Update and log performance statistics
        """
        # In a real implementation, this would log performance statistics
        pass
```

This comprehensive troubleshooting guide provides solutions for common voice command issues in VLA systems, including audio input problems, recognition issues, command mapping errors, performance problems, and configuration challenges. The guide includes practical code examples and diagnostic tools that can be directly implemented in VLA systems.
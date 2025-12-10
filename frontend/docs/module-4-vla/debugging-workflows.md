# Debugging Workflows for Complex VLA System Issues

## Overview

This document provides comprehensive debugging workflows for identifying and resolving complex issues in Vision-Language-Action (VLA) systems. The workflows are designed to systematically troubleshoot problems spanning multiple components of the VLA architecture.

## Debugging Methodology

### 1. Systematic Approach
- **Isolate Components**: Test each component individually before integration
- **Check Dependencies**: Verify all system dependencies are functioning
- **Review Logs**: Examine logs in chronological order
- **Validate Data Flow**: Trace data from input to output through the system
- **Reproduce Issues**: Create repeatable test cases for identified problems

### 2. Core Debugging Principles
- **Start Simple**: Begin with basic functionality before complex features
- **Methodical Elimination**: Rule out potential causes systematically
- **Document Everything**: Keep detailed records of observations and fixes
- **Collaborative Approach**: Use team knowledge to tackle complex issues

## Common VLA System Issues and Workflows

### Issue 1: Voice Command Processing Failures

#### Symptoms
- Voice commands not recognized or processed
- Whisper API connection errors
- Audio input processing failures
- Delayed or inconsistent command interpretation

#### Debugging Workflow

**Step 1: Verify Audio Input**
```bash
# Check audio devices
arecord -l

# Test audio recording
arecord -D hw:0,0 -f cd test.wav
# Speak and press Ctrl+C to stop
```

**Step 2: Check Node Status**
```bash
# Check if voice pipeline node is running
ros2 node list | grep voice

# Check topic publications
ros2 topic list | grep audio
ros2 topic echo /audio_input  # Verify audio data is flowing
```

**Step 3: Examine Logs**
```bash
# Monitor voice pipeline logs
ros2 run vla_examples voice_pipeline 2>&1 | grep -E "(ERROR|WARN|INFO)"
```

**Step 4: Test API Connectivity**
```python
# Test API access separately
import os
from openai import OpenAI

client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
try:
    # Test with a simple request
    response = client.models.list()
    print("API connection successful:", response.data[0].id)
except Exception as e:
    print("API connection failed:", e)
```

**Step 5: Check Environment Variables**
```bash
echo $OPENAI_API_KEY
echo $PYTHONPATH
ros2 run vla_examples voice_pipeline
```

### Issue 2: Cognitive Planning Failures

#### Symptoms
- Action sequences not generated
- LLM API errors
- Context integration failures
- Incorrect action planning

#### Debugging Workflow

**Step 1: Verify LLM Service**
```bash
# Check environment variables
echo $OPENAI_API_KEY

# Test LLM service directly
python3 -c "
import os
from openai import OpenAI

client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
try:
    response = client.chat.completions.create(
        model='gpt-3.5-turbo',
        messages=[{'role': 'user', 'content': 'Hello'}],
        max_tokens=10
    )
    print('LLM service working:', response.choices[0].message.content)
except Exception as e:
    print('LLM service error:', e)
"
```

**Step 2: Check Cognitive Planner Node**
```bash
# Monitor cognitive planner status
ros2 node info /cognitive_planner_node

# Check topic connections
ros2 topic info /vla/natural_command
ros2 topic info /vla/action_sequence
```

**Step 3: Test Command Processing**
```bash
# Send test command
ros2 topic pub /vla/natural_command std_msgs/String "data: 'move forward 1 meter'"
```

**Step 4: Examine Planning Context**
```python
# Check if context is being received by the planner
# Add debug prints to cognitive planner to show:
# - Incoming commands
# - Environmental context
# - Generated action sequences
```

### Issue 3: Perception Module Failures

#### Symptoms
- Objects not detected
- Incorrect object classifications
- 3D position estimation errors
- Performance degradation

#### Debugging Workflow

**Step 1: Verify Camera Connection**
```bash
# Check camera topics
ros2 topic list | grep camera
ros2 topic echo /camera/image_raw --field data[0]  # Verify image data is flowing

# Check camera info
ros2 topic echo /camera/camera_info
```

**Step 2: Test Computer Vision Pipeline**
```python
# Test OpenCV functionality separately
python3 -c "
import cv2
import numpy as np

# Test basic CV operations
test_img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.rectangle(test_img, (100, 100), (200, 200), (255, 0, 0), 2)
print('OpenCV operations working')

# Test if models can be loaded
try:
    from ultralytics import YOLO
    model = YOLO('yolov8n.pt')
    print('YOLO model loaded successfully')
except Exception as e:
    print('YOLO model error:', e)
"
```

**Step 3: Monitor Perception Node**
```bash
# Check perception node status
ros2 node info /perception_module

# Monitor object detections
ros2 topic echo /vla/perception/objects
```

**Step 4: Validate 3D Estimation**
```python
# Debug 3D position estimation in perception module
# Add logging to show:
# - 2D bounding box coordinates
# - Camera intrinsic parameters
# - Calculated 3D positions
```

### Issue 4: System Integration Issues

#### Symptoms
- Components not communicating
- Timing issues between modules
- Message passing failures
- State synchronization problems

#### Debugging Workflow

**Step 1: Check ROS Network Configuration**
```bash
# Verify ROS domain ID
echo $ROS_DOMAIN_ID

# Check all nodes
ros2 node list

# Verify all topics
ros2 topic list
```

**Step 2: Monitor Message Flow**
```bash
# Monitor command flow
ros2 topic echo /vla/natural_command
ros2 topic echo /vla/action_sequence
ros2 topic echo /vla/perception/objects
```

**Step 3: Test Component Communication**
```bash
# Send message to each component and verify response
ros2 topic pub /vla/natural_command std_msgs/String "data: 'test command'"
# Monitor downstream topics for response
```

**Step 4: Check QoS Settings**
```python
# Verify QoS profiles match between publishers and subscribers
# Add debugging to show message delivery success/failure rates
```

## Advanced Debugging Techniques

### 1. Distributed Tracing System

Implement a distributed tracing system to track messages across components:

```python
# distributed_tracer.py
import time
import uuid
from dataclasses import dataclass
from typing import Dict, List
import json


@dataclass
class TraceEvent:
    """Represents a traced event in the system"""
    trace_id: str
    span_id: str
    parent_span_id: str
    name: str
    timestamp: float
    component: str
    action: str
    metadata: Dict


class DistributedTracer:
    """System for tracing messages across VLA components"""
    
    def __init__(self):
        self.events: List[TraceEvent] = []
        self.current_trace = None
    
    def start_trace(self, name: str, component: str, initial_metadata: Dict = None):
        """Start a new trace"""
        trace_id = str(uuid.uuid4())
        span_id = str(uuid.uuid4())
        
        event = TraceEvent(
            trace_id=trace_id,
            span_id=span_id,
            parent_span_id="",
            name=name,
            timestamp=time.time(),
            component=component,
            action="start_trace",
            metadata=initial_metadata or {}
        )
        
        self.events.append(event)
        self.current_trace = trace_id
        return trace_id, span_id
    
    def add_event(self, trace_id: str, name: str, component: str, 
                  action: str, metadata: Dict = None):
        """Add an event to an existing trace"""
        span_id = str(uuid.uuid4())
        
        event = TraceEvent(
            trace_id=trace_id,
            span_id=span_id,
            parent_span_id="",  # Could track parent-child relationships
            name=name,
            timestamp=time.time(),
            component=component,
            action=action,
            metadata=metadata or {}
        )
        
        self.events.append(event)
        return span_id
    
    def get_trace(self, trace_id: str) -> List[TraceEvent]:
        """Get all events for a specific trace"""
        return [e for e in self.events if e.trace_id == trace_id]
    
    def print_trace(self, trace_id: str):
        """Print a formatted trace"""
        events = self.get_trace(trace_id)
        events.sort(key=lambda x: x.timestamp)
        
        print(f"TRACE {trace_id}:")
        for event in events:
            elapsed = event.timestamp - events[0].timestamp
            print(f"  [{elapsed:.3f}s] {event.component}.{event.name}: {event.action}")
            if event.metadata:
                print(f"    Metadata: {event.metadata}")


# Example usage in nodes
tracer = DistributedTracer()

def process_command_with_trace(command: str):
    # Start trace
    trace_id, span_id = tracer.start_trace(
        name="command_processing", 
        component="cognitive_planner",
        initial_metadata={"command": command}
    )
    
    # Add intermediate events
    tracer.add_event(
        trace_id=trace_id,
        name="context_integration",
        component="cognitive_planner",
        action="complete",
        metadata={"objects_in_context": 5}
    )
    
    # Add final event
    tracer.add_event(
        trace_id=trace_id,
        name="action_generation",
        component="cognitive_planner",
        action="complete",
        metadata={"actions_generated": 3}
    )
    
    # Print trace for debugging
    tracer.print_trace(trace_id)
```

### 2. System Health Monitoring

Create a health monitoring system for continuous assessment:

```python
# health_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from vla_msgs.msg import SystemHealth
import psutil
import time
from datetime import datetime


class SystemHealthMonitor(Node):
    """Monitor overall system health"""
    
    def __init__(self):
        super().__init__('system_health_monitor')
        
        # Publishers
        self.health_publisher = self.create_publisher(
            SystemHealth,
            '/vla/system_health',
            10
        )
        
        self.cpu_publisher = self.create_publisher(
            Float64,
            '/system/cpu_usage',
            10
        )
        
        self.memory_publisher = self.create_publisher(
            Float64,
            '/system/memory_usage',
            10
        )
        
        # Timers for periodic health checks
        self.health_timer = self.create_timer(2.0, self.check_system_health)
        
        self.get_logger().info('System Health Monitor initialized')
    
    def check_system_health(self):
        """Check system resources and component status"""
        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        self.cpu_publisher.publish(Float64(data=cpu_percent))
        
        # Memory usage
        memory_percent = psutil.virtual_memory().percent
        self.memory_publisher.publish(Float64(data=memory_percent))
        
        # Disk usage
        disk_percent = psutil.disk_usage('/').percent
        
        # Component connectivity (simplified)
        components = {
            'voice_pipeline': self.check_component_status('voice_pipeline'),
            'cognitive_planner': self.check_component_status('cognitive_planner'),
            'perception_module': self.check_component_status('perception_module'),
            'path_planning': self.check_component_status('path_planning'),
            'manipulation_system': self.check_component_status('manipulation_system')
        }
        
        # Overall health status
        critical_failures = [k for k, v in components.items() if not v['status']]
        if critical_failures:
            overall_status = 'critical' if len(critical_failures) > 2 else 'warning'
        else:
            overall_status = 'healthy' if cpu_percent < 80 and memory_percent < 80 else 'degraded'
        
        # Create health message
        health_msg = SystemHealth()
        health_msg.header.stamp = self.get_clock().now().to_msg()
        health_msg.timestamp = time.time()
        health_msg.overall_status = overall_status
        health_msg.cpu_usage = cpu_percent
        health_msg.memory_usage = memory_percent
        health_msg.disk_usage = disk_percent
        health_msg.critical_failures = critical_failures
        health_msg.components_status = str(components)
        
        self.health_publisher.publish(health_msg)
        
        # Log warnings
        if overall_status != 'healthy':
            self.get_logger().warn(f'System health: {overall_status}, CPU: {cpu_percent}%, Memory: {memory_percent}%')
    
    def check_component_status(self, component_name):
        """Check if a component is responsive"""
        # This would actually check if the node is alive and responsive
        # For now, we'll return a mock status
        import random
        return {
            'status': random.choice([True, True, True, True, False]),  # 20% failure rate in mock
            'latency': random.uniform(0.01, 0.5),
            'errors': 0
        }


def main(args=None):
    rclpy.init(args=args)
    
    monitor = SystemHealthMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('System Health Monitor interrupted by user')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()
```

## Debugging Tools and Utilities

### 1. Message Inspector Tool

```python
# message_inspector.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, LaserScan
from vla_msgs.msg import ActionSequence, DetectedObjects
import json
import time
from datetime import datetime


class MessageInspector(Node):
    """Utility to inspect messages across topics"""
    
    def __init__(self):
        super().__init__('message_inspector')
        
        # Topic-specific subscribers
        self.command_subscriber = self.create_subscription(
            String,
            '/vla/natural_command',
            lambda msg: self.inspect_message('natural_command', msg),
            10
        )
        
        self.action_subscriber = self.create_subscription(
            ActionSequence,
            '/vla/action_sequence',
            lambda msg: self.inspect_message('action_sequence', msg),
            10
        )
        
        self.perception_subscriber = self.create_subscription(
            DetectedObjects,
            '/vla/perception/objects',
            lambda msg: self.inspect_message('perception_objects', msg),
            10
        )
        
        self.get_logger().info('Message Inspector initialized')

    def inspect_message(self, topic_name, message):
        """Inspect and log message details"""
        timestamp = self.get_clock().now().to_msg()
        
        print(f"\n{'='*50}")
        print(f"TOPIC: {topic_name}")
        print(f"RECEIVED: {datetime.fromtimestamp(timestamp.sec + timestamp.nanosec/1e9)}")
        print(f"TYPE: {type(message).__name__}")
        print("-" * 50)
        
        # Print message contents differently based on type
        if topic_name == 'natural_command':
            print(f"COMMAND: {message.data}")
        
        elif topic_name == 'action_sequence':
            print(f"REQUEST_ID: {message.request_id}")
            print(f"ORIGINAL_COMMAND: {message.command}")
            print(f"NUMBER_OF_ACTIONS: {len(message.actions)}")
            for i, action in enumerate(message.actions):
                print(f"  Action {i+1}: {action.type} - {action.description}")
        
        elif topic_name == 'perception_objects':
            print(f"NUMBER_OF_OBJECTS: {len(message.objects)}")
            for obj in message.objects:
                print(f"  Object: {obj.name}, Conf: {obj.confidence:.2f}")
                pos = obj.pose.position
                print(f"    Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        
        print(f"{'='*50}\n")


def main(args=None):
    rclpy.init(args=args)
    
    inspector = MessageInspector()
    
    try:
        rclpy.spin(inspector)
    except KeyboardInterrupt:
        inspector.get_logger().info('Message Inspector interrupted by user')
    finally:
        inspector.destroy_node()
        rclpy.shutdown()
```

### 2. Performance Profiler

```python
# performance_profiler.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from vla_msgs.msg import SystemHealth
import time
import threading
from collections import deque
import statistics


class PerformanceProfiler(Node):
    """Monitor and profile performance of VLA system components"""
    
    def __init__(self):
        super().__init__('performance_profiler')
        
        # Publishers for performance metrics
        self.latency_publisher = self.create_publisher(
            Float64,
            '/vla/performance/latency',
            10
        )
        
        self.throughput_publisher = self.create_publisher(
            Float64,
            '/vla/performance/throughput',
            10
        )
        
        # Storage for performance metrics
        self.latency_measurements = deque(maxlen=1000)
        self.throughput_measurements = deque(maxlen=1000)
        self.start_times = {}
        
        # Timers for periodic metrics reporting
        self.metrics_timer = self.create_timer(5.0, self.report_metrics)
        
        self.get_logger().info('Performance Profiler initialized')
    
    def start_measurement(self, measurement_id):
        """Start timing a specific process"""
        self.start_times[measurement_id] = time.time()
    
    def stop_measurement(self, measurement_id):
        """Stop timing and record measurement"""
        if measurement_id in self.start_times:
            elapsed = time.time() - self.start_times[measurement_id]
            self.latency_measurements.append(elapsed)
            
            # Publish latency
            latency_msg = Float64()
            latency_msg.data = elapsed
            self.latency_publisher.publish(latency_msg)
            
            # Remove from active timers
            del self.start_times[measurement_id]
    
    def record_throughput(self, items_processed, time_period):
        """Record throughput measurement"""
        throughput = items_processed / time_period
        self.throughput_measurements.append(throughput)
        
        # Publish throughput
        throughput_msg = Float64()
        throughput_msg.data = throughput
        self.throughput_publisher.publish(throughput_msg)
    
    def report_metrics(self):
        """Report performance metrics"""
        if not self.latency_measurements:
            return
        
        # Calculate statistics
        avg_latency = statistics.mean(self.latency_measurements)
        min_latency = min(self.latency_measurements)
        max_latency = max(self.latency_measurements)
        latency_stddev = statistics.stdev(self.latency_measurements) if len(self.latency_measurements) > 1 else 0
        
        avg_throughput = statistics.mean(self.throughput_measurements) if self.throughput_measurements else 0
        
        # Log metrics
        self.get_logger().info(f"PERFORMANCE METRICS:")
        self.get_logger().info(f"  Latency - Avg: {avg_latency:.3f}s, Min: {min_latency:.3f}s, Max: {max_latency:.3f}s, StdDev: {latency_stddev:.3f}s")
        self.get_logger().info(f"  Throughput - Avg: {avg_throughput:.2f} items/sec")
        self.get_logger().info(f"  Samples - Latency: {len(self.latency_measurements)}, Throughput: {len(self.throughput_measurements)}")


def main(args=None):
    rclpy.init(args=args)
    
    profiler = PerformanceProfiler()
    
    try:
        rclpy.spin(profiler)
    except KeyboardInterrupt:
        profiler.get_logger().info('Performance Profiler interrupted by user')
    finally:
        profiler.destroy_node()
        rclpy.shutdown()
```

## Troubleshooting Scenarios

### Scenario 1: Slow Response Time

**Problem**: The system takes too long to respond to commands.

**Debugging Steps**:
1. Use performance profiler to measure latency in each component
2. Check API connection speeds for LLM calls
3. Verify perception processing time
4. Measure network latencies if applicable

**Solution**:
- Implement caching for frequently requested information
- Use more efficient algorithms where possible
- Parallelize independent operations
- Optimize data transmission between components

### Scenario 2: Inconsistent Behavior

**Problem**: The system behaves differently with identical inputs.

**Debugging Steps**:
1. Add random seed setting for reproducible results
2. Check for race conditions in multi-threaded components
3. Verify state consistency across components
4. Log all relevant variables for debugging

**Solution**:
- Implement deterministic processing where needed
- Add proper locking mechanisms for shared resources
- Improve state synchronization between components

### Scenario 3: Memory Leaks

**Problem**: System memory usage increases over time.

**Debugging Steps**:
1. Monitor memory usage with system health monitor
2. Use memory profiling tools to identify leak sources
3. Check for circular references in data structures
4. Verify callbacks are properly removed

**Solution**:
- Implement proper cleanup procedures
- Limit cache sizes with LRU eviction
- Use weak references where appropriate
- Add memory monitoring alerts

## Best Practices for VLA System Debugging

### 1. Proactive Monitoring
- Implement health checks for all components
- Monitor resource usage continuously
- Set up alerts for performance degradation
- Log important system states

### 2. Modular Testing
- Test each component independently
- Use mock services for isolated testing
- Implement unit tests for core functions
- Create integration test suites

### 3. Comprehensive Logging
- Log at multiple levels (DEBUG, INFO, WARN, ERROR)
- Include timestamps and component identifiers
- Log both inputs and outputs for troubleshooting
- Use structured logging for easier analysis

### 4. Error Recovery
- Implement graceful degradation when components fail
- Design fallback mechanisms for critical functions
- Include automatic restart capabilities
- Provide clear error messages for users

## Conclusion

This debugging guide provides systematic approaches for identifying and resolving complex issues in Vision-Language-Action systems. The workflows emphasize methodical troubleshooting, component isolation, and comprehensive monitoring. By following these practices, developers can efficiently diagnose and fix issues in their VLA implementations.

The tools and utilities provided can be integrated into any VLA system to aid in ongoing maintenance and debugging efforts, ensuring robust and reliable operation.
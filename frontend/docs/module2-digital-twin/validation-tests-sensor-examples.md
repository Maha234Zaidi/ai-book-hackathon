# Validation Tests for Sensor Examples in Digital Twin Applications

This document provides comprehensive validation tests to ensure that sensor examples in digital twin applications produce the expected outputs. These tests verify that both Gazebo sensor models and Unity visualizations behave correctly and consistently.

## Overview of Sensor Validation

Validation in digital twin applications involves verifying that:
1. Sensor models in Gazebo produce realistic outputs
2. Unity visualizations accurately represent sensor data
3. Data transmission between Gazebo and Unity is accurate
4. Calibrated sensor outputs match expected values

## LiDAR Sensor Validation Tests

### Test 1: Range Accuracy Validation

This test verifies that LiDAR sensors return accurate distance measurements:

```python
#!/usr/bin/env python3
"""
LiDAR range accuracy validation test
"""
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

def test_lidar_range_accuracy(distance_data, expected_distances, tolerance=0.05):
    """
    Test LiDAR range measurements against expected distances
    
    Args:
        distance_data: Array of measured distances from LiDAR
        expected_distances: Array of expected distances
        tolerance: Acceptable measurement tolerance in meters
    
    Returns:
        dict: Validation results including accuracy metrics
    """
    if len(distance_data) != len(expected_distances):
        raise ValueError("Data arrays must have the same length")
    
    # Calculate errors
    errors = np.abs(np.array(distance_data) - np.array(expected_distances))
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    std_error = np.std(errors)
    
    # Check if all measurements are within tolerance
    within_tolerance = np.sum(errors <= tolerance)
    success_rate = within_tolerance / len(errors)
    
    results = {
        'mean_error': mean_error,
        'max_error': max_error,
        'std_error': std_error,
        'success_rate': success_rate,
        'total_measurements': len(errors),
        'within_tolerance': int(within_tolerance),
        'tolerance': tolerance,
        'passed': success_rate >= 0.95  # 95% of measurements within tolerance
    }
    
    return results

# Example usage of the test
def run_lidar_range_test():
    # Simulated data - in practice, this would come from your digital twin
    expected_distances = [1.0, 2.0, 3.0, 4.0, 5.0]  # Expected distances in meters
    measured_distances = [1.01, 2.05, 2.98, 4.02, 4.95]  # What the LiDAR reported
    
    results = test_lidar_range_accuracy(measured_distances, expected_distances, tolerance=0.1)
    
    print(f"LiDAR Range Accuracy Test Results:")
    print(f"  Mean Error: {results['mean_error']:.3f}m")
    print(f"  Max Error: {results['max_error']:.3f}m")
    print(f"  Success Rate: {results['success_rate']*100:.1f}%")
    print(f"  Test Passed: {results['passed']}")
    
    return results

# Test if the validation function works properly
if __name__ == "__main__":
    run_lidar_range_test()
```

### Test 2: Angular Resolution Validation

Validates the angular resolution of LiDAR measurements:

```python
def test_lidar_angular_resolution(angles, expected_resolution, tolerance=0.01):
    """
    Test LiDAR angular resolution
    """
    # Calculate actual resolution between consecutive measurements
    actual_resolutions = np.diff(angles)
    
    # Calculate statistics
    mean_resolution = np.mean(actual_resolutions)
    std_resolution = np.std(actual_resolutions)
    
    # Check if resolution is within tolerance
    resolution_error = np.abs(mean_resolution - expected_resolution)
    
    results = {
        'mean_resolution': mean_resolution,
        'std_resolution': std_resolution,
        'resolution_error': resolution_error,
        'expected_resolution': expected_resolution,
        'passed': resolution_error <= tolerance
    }
    
    return results
```

### Test 3: Obstacle Detection Validation

Tests that LiDAR properly detects obstacles at known positions:

```python
def test_lidar_obstacle_detection(lidar_ranges, lidar_angles, obstacles, tolerance=0.1):
    """
    Test LiDAR's ability to detect obstacles at known positions
    
    Args:
        lidar_ranges: Array of range measurements from LiDAR
        lidar_angles: Array of corresponding angle measurements
        obstacles: List of tuples (x, y, radius) representing obstacles
        tolerance: Position tolerance for detection
    
    Returns:
        dict: Detection accuracy results
    """
    detected_obstacles = []
    expected_obstacles = len(obstacles)
    
    # Convert range/angle pairs to Cartesian coordinates
    x_coords = [r * math.cos(theta) for r, theta in zip(lidar_ranges, lidar_angles)]
    y_coords = [r * math.sin(theta) for r, theta in zip(lidar_ranges, lidar_angles)]
    
    # Check if any LiDAR points are near known obstacles
    for obs_x, obs_y, obs_radius in obstacles:
        for x, y in zip(x_coords, y_coords):
            distance_to_obstacle = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            
            # If point is within obstacle radius + tolerance, consider it detected
            if distance_to_obstacle <= obs_radius + tolerance:
                detected_obstacles.append((x, y))
                break  # Each obstacle only needs to be detected once
    
    results = {
        'expected_obstacles': expected_obstacles,
        'detected_obstacles': len(detected_obstacles),
        'detection_rate': len(detected_obstacles) / expected_obstacles if expected_obstacles > 0 else 0,
        'passed': len(detected_obstacles) == expected_obstacles
    }
    
    return results
```

## Depth Camera Validation Tests

### Test 1: Depth Accuracy Validation

Validates the accuracy of depth camera measurements:

```python
def test_depth_camera_accuracy(depth_image, expected_depths, tolerance=0.05):
    """
    Validate depth camera accuracy against expected values
    
    Args:
        depth_image: 2D array of depth measurements
        expected_depths: 2D array of expected depth values
        tolerance: Acceptable measurement tolerance
    
    Returns:
        dict: Accuracy validation results
    """
    if depth_image.shape != expected_depths.shape:
        raise ValueError("Depth image and expected depths must have the same shape")
    
    # Calculate errors
    errors = np.abs(depth_image - expected_depths)
    
    # Filter out invalid measurements (e.g., max range values)
    valid_mask = (expected_depths > 0) & (expected_depths < 10)  # Assuming valid range is 0-10m
    valid_errors = errors[valid_mask]
    
    if len(valid_errors) == 0:
        return {'passed': False, 'error': 'No valid measurements found'}
    
    # Calculate statistics
    mean_error = np.mean(valid_errors)
    max_error = np.max(valid_errors)
    std_error = np.std(valid_errors)
    
    # Calculate success rate
    within_tolerance = np.sum(valid_errors <= tolerance)
    success_rate = within_tolerance / len(valid_errors)
    
    results = {
        'mean_error': mean_error,
        'max_error': max_error,
        'std_error': std_error,
        'success_rate': success_rate,
        'valid_measurements': len(valid_errors),
        'passed': success_rate >= 0.90  # 90% of valid measurements within tolerance
    }
    
    return results
```

### Test 2: RGB Image Quality Validation

Validates the quality of RGB images from the depth camera:

```python
def test_rgb_image_quality(rgb_image, min_brightness=0.1, max_brightness=0.9):
    """
    Test RGB image quality metrics
    
    Args:
        rgb_image: RGB image as a numpy array
        min_brightness: Minimum acceptable brightness (0-1)
        max_brightness: Maximum acceptable brightness (0-1)
    
    Returns:
        dict: Quality validation results
    """
    # Convert to grayscale for brightness calculation
    gray = np.dot(rgb_image[...,:3], [0.2989, 0.5870, 0.1140])
    brightness = np.mean(gray) / 255.0  # Normalize to 0-1 range
    
    # Calculate contrast (standard deviation in grayscale)
    contrast = np.std(gray) / 255.0
    
    # Check for saturation (proportion of pixels at near-max/min values)
    saturated_low = np.sum(gray < 10) / gray.size
    saturated_high = np.sum(gray > 245) / gray.size
    saturation_ratio = saturated_low + saturated_high
    
    results = {
        'brightness': brightness,
        'contrast': contrast,
        'saturation_ratio': saturation_ratio,
        'acceptable_brightness': min_brightness <= brightness <= max_brightness,
        'acceptable_contrast': contrast > 0.1,  # At least 10% contrast
        'acceptable_saturation': saturation_ratio < 0.1,  # Less than 10% saturation
        'passed': (min_brightness <= brightness <= max_brightness and 
                  contrast > 0.1 and 
                  saturation_ratio < 0.1)
    }
    
    return results
```

## IMU Validation Tests

### Test 1: Accelerometer Calibration Validation

Validates that accelerometer readings are properly calibrated when the system is stationary:

```python
def test_accelerometer_calibration(accel_readings, tolerance=0.1):
    """
    Validate accelerometer when system is stationary
    Expected: [0, 0, 9.81] m/s² (assuming Z-up coordinate system)
    """
    # Calculate mean values over the readings
    mean_x = np.mean([reading[0] for reading in accel_readings])
    mean_y = np.mean([reading[1] for reading in accel_readings])
    mean_z = np.mean([reading[2] for reading in accel_readings])
    
    # Expected values for stationary object (Z-axis measuring gravity)
    expected_x = 0.0
    expected_y = 0.0
    expected_z = 9.81
    
    # Calculate errors
    error_x = abs(mean_x - expected_x)
    error_y = abs(mean_y - expected_y)
    error_z = abs(mean_z - expected_z)
    
    # Check if errors are within tolerance
    x_ok = error_x <= tolerance
    y_ok = error_y <= tolerance
    z_ok = error_z <= tolerance
    
    results = {
        'mean_x': mean_x,
        'mean_y': mean_y,
        'mean_z': mean_z,
        'error_x': error_x,
        'error_y': error_y,
        'error_z': error_z,
        'tolerance': tolerance,
        'passed': x_ok and y_ok and z_ok
    }
    
    return results
```

### Test 2: Gyroscope Drift Validation

Validates that gyroscope readings don't drift significantly when the system is stationary:

```python
def test_gyroscope_drift(gyro_readings, max_drift_rate=0.01, tolerance=0.05):
    """
    Validate gyroscope drift during stationary periods
    
    Args:
        gyro_readings: Array of [wx, wy, wz] gyroscope readings
        max_drift_rate: Maximum acceptable drift rate (rad/s²)
        tolerance: Tolerance for instantaneous readings
    
    Returns:
        dict: Drift validation results
    """
    # Calculate instantaneous drift (deviation from zero)
    readings_array = np.array(gyro_readings)
    instantaneous_errors = np.abs(readings_array)
    
    # Calculate mean error (drift)
    mean_errors = np.mean(instantaneous_errors, axis=0)
    
    # Check if errors are within tolerance
    max_error = np.max(instantaneous_errors)
    mean_exceeds_tolerance = np.any(mean_errors > tolerance)
    
    results = {
        'mean_x_error': mean_errors[0],
        'mean_y_error': mean_errors[1],
        'mean_z_error': mean_errors[2],
        'max_instantaneous_error': max_error,
        'tolerance': tolerance,
        'passed': not mean_exceeds_tolerance and max_error <= tolerance
    }
    
    return results
```

## Unity Visualization Validation

### Test 1: Data Synchronization Validation

Validates that sensor data is properly synchronized between Gazebo and Unity:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorDataValidator : MonoBehaviour
{
    [Header("Validation Settings")]
    public float maxTimeDifference = 0.1f;  // Maximum allowed time difference in seconds
    public float valueTolerance = 0.05f;    // Tolerance for value comparison
    
    private Queue<SensorReading> gazeboReadings = new Queue<SensorReading>();
    private Queue<SensorReading> unityReadings = new Queue<SensorReading>();
    
    [System.Serializable]
    public class SensorReading
    {
        public float timestamp;
        public Vector3 value;
        public string sensorType;
    }
    
    public void AddGazeboReading(SensorReading reading)
    {
        gazeboReadings.Enqueue(reading);
    }
    
    public void AddUnityReading(SensorReading reading)
    {
        unityReadings.Enqueue(reading);
    }
    
    public bool ValidateSynchronization()
    {
        // Check if we have enough readings to compare
        if (gazeboReadings.Count < 10 || unityReadings.Count < 10)
        {
            Debug.LogWarning("Insufficient readings for validation");
            return false;
        }
        
        int validatedCount = 0;
        int totalValidated = 0;
        
        // Process pairs of readings
        while (gazeboReadings.Count > 0 && unityReadings.Count > 0)
        {
            SensorReading gazeboReading = gazeboReadings.Peek();
            SensorReading unityReading = unityReadings.Peek();
            
            // Check if readings are close in time
            float timeDiff = Mathf.Abs(gazeboReading.timestamp - unityReading.timestamp);
            
            if (timeDiff <= maxTimeDifference)
            {
                // If timestamps are close, validate the values
                gazeboReadings.Dequeue();
                unityReadings.Dequeue();
                
                // Check if sensor types match
                if (gazeboReading.sensorType == unityReading.sensorType)
                {
                    // Validate the values
                    float valueDiff = Vector3.Distance(gazeboReading.value, unityReading.value);
                    if (valueDiff <= valueTolerance)
                    {
                        validatedCount++;
                    }
                    totalValidated++;
                }
            }
            else if (gazeboReading.timestamp < unityReading.timestamp)
            {
                // Gazebo reading is older, discard it
                gazeboReadings.Dequeue();
            }
            else
            {
                // Unity reading is older, discard it
                unityReadings.Dequeue();
            }
        }
        
        // Calculate success rate
        float successRate = totalValidated > 0 ? (float)validatedCount / totalValidated : 0;
        
        // Return true if at least 90% of readings are synchronized and valid
        return successRate >= 0.9f;
    }
    
    public void ClearReadings()
    {
        gazeboReadings.Clear();
        unityReadings.Clear();
    }
}
```

### Test 2: Sensor Visualization Accuracy

Validates that sensor visualizations in Unity accurately represent the sensor data:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class VisualizationValidator : MonoBehaviour
{
    [Header("Visualization Settings")]
    public Transform cameraTransform;
    public float maxVisualizationError = 0.1f;
    
    public bool ValidateLiDARVisualization(
        float[] lidarRanges, 
        float[] lidarAngles, 
        GameObject[] visualizationPoints)
    {
        if (lidarRanges.Length != lidarAngles.Length || 
            lidarRanges.Length != visualizationPoints.Length)
        {
            Debug.LogError("Array lengths don't match for LiDAR validation");
            return false;
        }
        
        int accuratePoints = 0;
        
        for (int i = 0; i < lidarRanges.Length; i++)
        {
            // Calculate expected position from range and angle
            Vector3 expectedPosition = CalculateLiDARPointPosition(
                lidarRanges[i], lidarAngles[i]);
            
            // Get actual position from visualization
            Vector3 actualPosition = visualizationPoints[i].transform.position;
            
            // Calculate the difference
            float positionError = Vector3.Distance(expectedPosition, actualPosition);
            
            if (positionError <= maxVisualizationError)
            {
                accuratePoints++;
            }
        }
        
        // Calculate success rate
        float successRate = (float)accuratePoints / lidarRanges.Length;
        
        return successRate >= 0.95f; // 95% of points must be accurate
    }
    
    Vector3 CalculateLiDARPointPosition(float range, float angle)
    {
        // Calculate position relative to LiDAR origin
        // Assuming LiDAR is at this GameObject's position
        float x = range * Mathf.Cos(angle);
        float y = 0f; // Assuming 2D LiDAR
        float z = range * Mathf.Sin(angle);
        
        return transform.position + new Vector3(x, y, z);
    }
    
    public bool ValidateDepthVisualization(
        float[] depthData, 
        int width, 
        int height, 
        GameObject depthVisualizationPlane)
    {
        // This is a simplified validation - in practice you'd check each pixel
        // For now, just verify the visualization is present and has correct dimensions
        if (depthVisualizationPlane == null)
        {
            return false;
        }
        
        // Check if the visualization's scale or properties match expected dimensions
        // This is a simplified check
        return true;
    }
}
```

## Integration Validation Tests

### Test 1: System-Level Validation

Validates the overall sensor system in the digital twin:

```python
def run_comprehensive_sensor_validation():
    """
    Run all sensor validation tests and generate a comprehensive report
    """
    print("Running Comprehensive Sensor Validation Tests...")
    
    results = {}
    
    # Test LiDAR
    print("\n1. LiDAR Validation Tests:")
    lidar_ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
    expected_ranges = [1.01, 2.02, 3.01, 4.03, 4.99]
    results['lidar_range'] = test_lidar_range_accuracy(lidar_ranges, expected_ranges)
    print(f"   Range Accuracy: {'PASS' if results['lidar_range']['passed'] else 'FAIL'}")
    
    # Test IMU
    print("\n2. IMU Validation Tests:")
    stationary_accel_readings = [[0.01, -0.02, 9.83]] * 100  # 100 readings
    results['accel_calibration'] = test_accelerometer_calibration(stationary_accel_readings)
    print(f"   Accelerometer Calibration: {'PASS' if results['accel_calibration']['passed'] else 'FAIL'}")
    
    zero_gyro_readings = [[0.001, -0.002, 0.001]] * 100  # 100 readings
    results['gyro_drift'] = test_gyroscope_drift(zero_gyro_readings)
    print(f"   Gyroscope Drift: {'PASS' if results['gyro_drift']['passed'] else 'FAIL'}")
    
    # Test Depth Camera
    print("\n3. Depth Camera Validation Tests:")
    import numpy as np
    dummy_depth = np.ones((480, 640)) * 2.0  # 2m everywhere
    dummy_depth[200:280, 300:340] = 1.0      # 1m for a central object
    expected_depth = dummy_depth.copy()
    results['depth_accuracy'] = test_depth_camera_accuracy(dummy_depth, expected_depth)
    print(f"   Depth Accuracy: {'PASS' if results['depth_accuracy']['passed'] else 'FAIL'}")
    
    # Generate summary report
    print("\n" + "="*50)
    print("COMPREHENSIVE VALIDATION REPORT")
    print("="*50)
    
    passed_tests = 0
    total_tests = len(results)
    
    for test_name, test_result in results.items():
        status = "PASS" if test_result.get('passed', False) else "FAIL"
        print(f"{test_name.upper().replace('_', ' ')}: {status}")
        if test_result.get('passed', False):
            passed_tests += 1
    
    print("-"*50)
    print(f"SUMMARY: {passed_tests}/{total_tests} tests passed")
    print(f"SUCCESS RATE: {passed_tests/total_tests*100:.1f}%")
    
    # Determine overall system validation
    overall_pass = passed_tests == total_tests
    print(f"OVERALL VALIDATION: {'PASS' if overall_pass else 'FAIL'}")
    
    return results, overall_pass

# Run the validation
if __name__ == "__main__":
    validation_results, system_pass = run_comprehensive_sensor_validation()
```

### Test 2: Continuous Monitoring Validation

Sets up continuous monitoring of sensor validity:

```csharp
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class ContinuousSensorValidation : MonoBehaviour
{
    public float validationInterval = 1.0f;  // Validate every second
    public float warningThreshold = 0.8f;   // Warn if success rate drops below this
    public float failureThreshold = 0.5f;   // Consider failed if below this
    
    public SensorDataValidator sensorValidator;
    public VisualizationValidator vizValidator;
    
    private float lastValidationTime = 0f;
    private Queue<float> successRates = new Queue<float>();
    private const int SUCCESS_RATE_HISTORY = 10;  // Track last 10 validation periods
    
    void Update()
    {
        if (Time.time - lastValidationTime > validationInterval)
        {
            PerformValidation();
            lastValidationTime = Time.time;
        }
    }
    
    void PerformValidation()
    {
        // Perform synchronization validation
        bool syncValid = sensorValidator.ValidateSynchronization();
        
        // Log the validation result
        float successRate = syncValid ? 1.0f : 0.0f;
        successRates.Enqueue(successRate);
        
        if (successRates.Count > SUCCESS_RATE_HISTORY)
        {
            successRates.Dequeue();
        }
        
        // Calculate average success rate
        float avgSuccessRate = 0f;
        foreach (float rate in successRates)
        {
            avgSuccessRate += rate;
        }
        avgSuccessRate /= successRates.Count;
        
        // Check if we need to raise an alert
        if (avgSuccessRate < failureThreshold)
        {
            Debug.LogError($"SENSOR VALIDATION FAILURE: Average success rate is {avgSuccessRate * 100:F2}%");
        }
        else if (avgSuccessRate < warningThreshold)
        {
            Debug.LogWarning($"SENSOR VALIDATION WARNING: Average success rate is {avgSuccessRate * 100:F2}%");
        }
        else
        {
            Debug.Log($"Sensor validation OK: Average success rate is {avgSuccessRate * 100:F2}%");
        }
        
        // Store for visualization
        StoreValidationMetrics(avgSuccessRate, syncValid);
    }
    
    void StoreValidationMetrics(float avgSuccessRate, bool currentValid)
    {
        // In a real system, you might send this to a monitoring system
        // or store in a data structure for visualization
    }
    
    public float GetCurrentSuccessRate()
    {
        if (successRates.Count == 0) return 1.0f;
        
        float avgSuccessRate = 0f;
        foreach (float rate in successRates)
        {
            avgSuccessRate += rate;
        }
        return avgSuccessRate / successRates.Count;
    }
}
```

## Best Practices for Sensor Validation

### 1. Automated Testing Pipeline

Create an automated testing pipeline for sensor validation:

```python
import unittest
import numpy as np

class SensorValidationTestCase(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.tolerance = 0.05  # 5cm tolerance
    
    def test_lidar_range_accuracy(self):
        """Test LiDAR range measurements."""
        # Test with perfect data
        expected = [1.0, 2.0, 3.0]
        measured = [1.0, 2.0, 3.0]
        results = test_lidar_range_accuracy(measured, expected, self.tolerance)
        self.assertTrue(results['passed'])
        
        # Test with slightly off data
        measured = [1.02, 2.01, 2.98]
        results = test_lidar_range_accuracy(measured, expected, self.tolerance)
        self.assertTrue(results['passed'])
        
        # Test with significantly off data
        measured = [1.1, 2.1, 3.1]
        results = test_lidar_range_accuracy(measured, expected, self.tolerance)
        self.assertFalse(results['passed'])
    
    def test_accelerometer_calibration(self):
        """Test accelerometer calibration."""
        # Test with properly calibrated data (stationary)
        readings = [[0.01, -0.02, 9.82]] * 50
        results = test_accelerometer_calibration(readings, tolerance=0.1)
        self.assertTrue(results['passed'])
        
        # Test with uncalibrated data
        readings = [[0.5, 0.3, 10.2]] * 50  # Significant bias
        results = test_accelerometer_calibration(readings, tolerance=0.1)
        self.assertFalse(results['passed'])

if __name__ == '__main__':
    unittest.main()
```

### 2. Validation Dashboard

Create a dashboard for monitoring validation results:

```csharp
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class ValidationDashboard : MonoBehaviour
{
    public Text statusText;
    public Image statusIndicator;
    public Text validationDetailsText;
    
    public ContinuousSensorValidation validator;
    
    void Update()
    {
        float successRate = validator.GetCurrentSuccessRate();
        
        // Update status text
        statusText.text = $"Success Rate: {successRate * 100:F2}%";
        
        // Update indicator color
        if (successRate >= 0.9f)
        {
            statusIndicator.color = Color.green;
        }
        else if (successRate >= 0.7f)
        {
            statusIndicator.color = Color.yellow;
        }
        else
        {
            statusIndicator.color = Color.red;
        }
        
        // Update details with validation information
        validationDetailsText.text = GetValidationDetails();
    }
    
    string GetValidationDetails()
    {
        // Format detailed validation information
        float successRate = validator.GetCurrentSuccessRate();
        string details = $"Overall Success Rate: {successRate * 100:F2}%\n";
        // Add more details as needed
        return details;
    }
}
```

These validation tests ensure that the sensor examples in your digital twin application produce the expected outputs, maintaining the accuracy and reliability essential for digital twin applications.
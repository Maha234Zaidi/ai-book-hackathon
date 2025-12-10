# Sensor Calibration and Noise Modeling Techniques

This document details the techniques for calibrating sensors in digital twin applications and modeling their noise characteristics to ensure accurate simulations that match real-world sensor behavior.

## Introduction to Sensor Calibration

Sensor calibration is the process of determining and correcting for systematic errors in sensor measurements. In digital twin applications, accurate calibration is crucial for ensuring that the simulated sensor data matches the real-world behavior of the physical system being represented.

### Types of Sensor Errors

There are two main categories of sensor errors that need to be addressed through calibration:

1. **Systematic (Bias) Errors**: Consistent errors that affect all measurements in a predictable way.
2. **Random (Noise) Errors**: Unpredictable variations in measurements around the true value.

## Calibration Procedures for Different Sensors

### LiDAR Calibration

LiDAR sensors require several types of calibration to ensure accurate measurements:

#### 1. Range Calibration
Range calibration corrects for systematic biases in distance measurements:

```xml
<!-- Example of range calibration parameters in URDF/SDF -->
<gazebo reference="laser_link">
  <sensor type="ray" name="laser_scanner">
    <!-- Range calibration parameters -->
    <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
      <gaussian_noise>0.01</gaussian_noise>  <!-- 1cm standard deviation
      <bias_correction>0.005</bias_correction>  <!-- 5mm systematic bias
    </plugin>
  </sensor>
</gazebo>
```

#### 2. Angular Calibration
Angular calibration corrects for errors in the angular positioning of laser beams:

```yaml
# Example YAML configuration for LiDAR calibration
lidar_calibration:
  angular_offsets:
    -0.01, -0.008, -0.005, 0, 0.005, 0.008, 0.01  # Angular corrections in radians
  range_bias: 0.005  # Range correction in meters
  intensity_correction: 1.05  # Factor to adjust intensity readings
```

### Camera Calibration

Camera calibration is essential for accurate visual perception in digital twins:

#### 1. Intrinsic Calibration
Intrinsic parameters include focal length, principal point, and distortion coefficients:

```python
# Example Python code for camera calibration
import cv2
import numpy as np

def calibrate_camera(images, pattern_size):
    """
    Calibrate a camera using a chessboard pattern
    """
    # Prepare object points based on pattern size
    objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    
    # Arrays to store object points and image points from all images
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        
        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1))
            imgpoints.append(corners2)
    
    # Perform the calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    
    return mtx, dist, rvecs, tvecs

# Example usage
# camera_matrix, distortion_coeffs, rvecs, tvecs = calibrate_camera(image_list, (9, 6))
```

In SDF format, the camera calibration parameters would look like this:

```xml
<sensor type="camera" name="camera_sensor">
  <camera name="camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <distortion>
      <k1>-0.21</k1>
      <k2>0.25</k2>
      <k3>-0.001</k3>
      <p1>-0.0001</p1>
      <p2>-0.0001</p2>
      <center>0.5 0.5</center>
    </distortion>
  </camera>
</sensor>
```

#### 2. Extrinsic Calibration
Extrinsic parameters define the position and orientation of the camera relative to a reference frame:

```xml
<!-- In URDF, extrinsic calibration is defined as the pose of the camera link -->
<link name="camera_link">
  <pose>0.1 0.0 0.2 0.0 0.1 0.0</pose>  <!-- x, y, z, roll, pitch, yaw
</link>
```

### IMU Calibration

IMU calibration is critical for accurate motion estimation and control:

#### 1. Accelerometer Calibration
Accelerometer calibration involves determining scale factors and biases:

```c
// Example C code for accelerometer calibration
// Collect stationary readings (should be near [0, 0, 1] for z-axis up)
void calibrate_accelerometer(float* readings, int num_readings) {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for (int i = 0; i < num_readings; i++) {
        sum_x += readings[i*3];
        sum_y += readings[i*3 + 1];
        sum_z += readings[i*3 + 2];
    }
    
    // Calculate mean biases
    float bias_x = sum_x / num_readings;
    float bias_y = sum_y / num_readings;
    float bias_z = (sum_z / num_readings) - 9.81;  // Subtract gravity
    
    // Calculate scale factors relative to expected gravity
    float expected_magnitude = 9.81;
    float actual_magnitude = sqrt(
        (sum_x/num_readings) * (sum_x/num_readings) +
        (sum_y/num_readings) * (sum_y/num_readings) +
        (sum_z/num_readings) * (sum_z/num_readings)
    );
    
    float scale_factor = expected_magnitude / actual_magnitude;
    
    printf("Accel biases: [%f, %f, %f]\n", bias_x, bias_y, bias_z);
    printf("Scale factor: %f\n", scale_factor);
}
```

#### 2. Gyroscope Calibration
Gyroscope calibration typically involves estimating bias during stationary periods:

```python
# Example Python code for gyroscope calibration
import numpy as np

def calibrate_gyro_stationary(gyro_readings, sample_rate=100, duration=10):
    """
    Calibrate gyroscope using stationary measurements
    """
    # Use first N seconds of data assuming robot is stationary
    num_samples = min(len(gyro_readings), sample_rate * duration)
    stationary_data = gyro_readings[:num_samples]
    
    # Calculate mean as the bias
    bias_x = np.mean(stationary_data[:, 0])
    bias_y = np.mean(stationary_data[:, 1])
    bias_z = np.mean(stationary_data[:, 2])
    
    # Calculate standard deviation as noise estimate
    noise_std_x = np.std(stationary_data[:, 0])
    noise_std_y = np.std(stationary_data[:, 1])
    noise_std_z = np.std(stationary_data[:, 2])
    
    return [bias_x, bias_y, bias_z], [noise_std_x, noise_std_y, noise_std_z]

# Example usage:
# gyro_bias, gyro_noise_std = calibrate_gyro_stationary(gyro_data)
```

## Advanced Noise Modeling Techniques

### 1. Allan Variance Analysis

Allan variance is a powerful technique for characterizing different noise sources in sensors:

```python
# Example Python code for Allan variance analysis
import numpy as np
import matplotlib.pyplot as plt

def allan_variance(data, sample_rate=100):
    """
    Calculate Allan variance of sensor data to identify noise sources
    """
    # Convert to appropriate units if necessary
    # Process data for different cluster times
    max_cluster_points = len(data) // 2
    
    # Calculate for different cluster times
    cluster_times = []
    allan_devs = []
    
    for n in range(1, max_cluster_points):
        # Cluster time in seconds
        tau = n / sample_rate
        
        # Number of clusters
        N = len(data) // n
        
        # Average data in clusters
        clusters = []
        for i in range(N):
            start_idx = i * n
            end_idx = start_idx + n
            cluster_avg = np.mean(data[start_idx:end_idx])
            clusters.append(cluster_avg)
        
        # Calculate Allan deviation
        diffs = np.diff(clusters)
        allan_var = 0.5 * np.mean(diffs**2)
        allan_dev = np.sqrt(allan_var)
        
        cluster_times.append(tau)
        allan_devs.append(allan_dev)
    
    return np.array(cluster_times), np.array(allan_devs)

def plot_allan_variance(taus, allan_devs, sensor_type="gyro"):
    """
    Plot Allan variance results
    """
    plt.figure(figsize=(10, 6))
    plt.loglog(taus, allan_devs, 'b-', label='Allan Deviation')
    
    # Add reference lines for different noise types
    if sensor_type == "gyro":
        # Quantization noise: slope -1
        qn_slope = np.array([10**i for i in range(int(np.log10(taus[0])), int(np.log10(taus[-1])+1))])
        qn = [1e-6] * len(qn_slope)  # Example value
        plt.loglog(qn_slope, qn, '--', label='Quantization Noise (slope -1)')
        
        # Angle Random Walk: slope -0.5
        arw_slope = qn_slope
        arw = [1e-4 * t**(-0.5) for t in arw_slope]  # Example value
        plt.loglog(arw_slope, arw, '--', label='Angle Random Walk (slope -0.5)')
        
        # Bias Instability: slope 0
        bi_slope = qn_slope
        bi = [1e-5] * len(bi_slope)  # Example value
        plt.loglog(bi_slope, bi, '--', label='Bias Instability (slope 0)')
        
        # Rate Random Walk: slope 0.5
        rrw_slope = qn_slope
        rrw = [1e-4 * t**0.5 for t in rrw_slope]  # Example value
        plt.loglog(rrw_slope, rrw, '--', label='Rate Random Walk (slope 0.5)')
        
        # Angular Rate Ramp: slope 1
        arr_slope = qn_slope
        arr = [1e-4 * t**1 for t in arr_slope]  # Example value
        plt.loglog(arr_slope, arr, '--', label='Angular Rate Ramp (slope 1)')
    
    plt.xlabel('Cluster Time (s)')
    plt.ylabel('Allan Deviation')
    plt.title(f'Allan Variance Plot - {sensor_type.capitalize()}')
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage:
# taus, devs = allan_variance(gyro_x_data, sample_rate=100)
# plot_allan_variance(taus, devs, "gyro")
```

### 2. Auto-Correlation Analysis

Auto-correlation analysis can reveal time-dependent noise characteristics:

```python
# Example Python code for auto-correlation
import numpy as np
import matplotlib.pyplot as plt

def autocorrelation(data, max_lag=None):
    """
    Calculate the auto-correlation of a signal
    """
    if max_lag is None:
        max_lag = len(data) // 4  # Use quarter of data length
    
    autocorr = []
    mean_val = np.mean(data)
    
    for lag in range(max_lag):
        correlation = np.mean(
            (data[:len(data)-lag] - mean_val) * 
            (data[lag:] - mean_val)
        ) / np.var(data)
        autocorr.append(correlation)
    
    return np.array(autocorr)

def plot_autocorrelation(autocorr, title="Auto-correlation"):
    """
    Plot the auto-correlation function
    """
    lags = range(len(autocorr))
    
    plt.figure(figsize=(10, 6))
    plt.stem(lags[:50], autocorr[:50])  # Show first 50 lags
    plt.title(title)
    plt.xlabel('Lag')
    plt.ylabel('Correlation')
    plt.grid(True)
    plt.show()

# Example usage:
# autocorr = autocorrelation(gyro_data[:, 0])
# plot_autocorrelation(autocorr, "Gyroscope X-axis Auto-correlation")
```

## Implementing Calibration in Digital Twins

### Calibration Workflow in Unity

When using sensor data in Unity for visualization, you might need to apply calibration corrections:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorCalibration : MonoBehaviour
{
    [Header("IMU Calibration")]
    public Vector3 accelBias = Vector3.zero;
    public Vector3 gyroBias = Vector3.zero;
    public Vector3 accelScale = Vector3.one;
    public Vector3 gyroScale = Vector3.one;
    
    [Header("LiDAR Calibration")]
    public float lidarRangeBias = 0.0f;
    public float lidarAngularOffset = 0.0f;
    
    [Header("Camera Calibration")]
    public Matrix4x4 cameraIntrinsics = Matrix4x4.identity;
    public Matrix4x4 cameraDistortion = Matrix4x4.identity;
    
    // Apply calibration to raw sensor data
    public Vector3 CalibrateAccelerometer(Vector3 rawAccel)
    {
        // Apply scale and bias corrections
        Vector3 corrected = new Vector3(
            (rawAccel.x - accelBias.x) * accelScale.x,
            (rawAccel.y - accelBias.y) * accelScale.y,
            (rawAccel.z - accelBias.z) * accelScale.z
        );
        
        return corrected;
    }
    
    public Vector3 CalibrateGyroscope(Vector3 rawGyro)
    {
        // Apply scale and bias corrections
        Vector3 corrected = new Vector3(
            (rawGyro.x - gyroBias.x) * gyroScale.x,
            (rawGyro.y - gyroBias.y) * gyroScale.y,
            (rawGyro.z - gyroBias.z) * gyroScale.z
        );
        
        return corrected;
    }
    
    public float[] CalibrateLidarRange(float[] rawRanges)
    {
        // Apply range bias correction
        float[] correctedRanges = new float[rawRanges.Length];
        for (int i = 0; i < rawRanges.Length; i++)
        {
            correctedRanges[i] = rawRanges[i] + lidarRangeBias;
            
            // Apply angular correction if needed
            if (i < rawRanges.Length / 2)
            {
                correctedRanges[i] += lidarAngularOffset * Mathf.Sin(i * Mathf.PI / rawRanges.Length);
            }
            else
            {
                correctedRanges[i] += lidarAngularOffset * Mathf.Sin(i * Mathf.PI / rawRanges.Length);
            }
        }
        
        return correctedRanges;
    }
    
    void Start()
    {
        // Initialize calibration parameters
        InitializeCalibrationParameters();
    }
    
    void InitializeCalibrationParameters()
    {
        // Load calibration parameters from a configuration file or receive from ROS
        // This is a simplified example - in practice, you'd load from a file or ROS topic
        Debug.Log("Sensor calibration initialized with current parameters");
    }
}
```

### Dynamic Calibration Update

In real-time digital twins, you might want to update calibration parameters dynamically:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class DynamicCalibration : MonoBehaviour
{
    private Dictionary<string, CalibrationParams> sensorCalibrations;
    
    [System.Serializable]
    public class CalibrationParams
    {
        public Vector3 bias = Vector3.zero;
        public Vector3 scale = Vector3.one;
        public float noiseStdDev = 0.0f;
        public float correlationTime = 0.0f;
    }
    
    void Start()
    {
        sensorCalibrations = new Dictionary<string, CalibrationParams>();
        InitializeDefaultCalibrations();
    }
    
    void InitializeDefaultCalibrations()
    {
        // Example initialization with default values
        CalibrationParams imuCalib = new CalibrationParams
        {
            bias = new Vector3(0.01f, -0.02f, 0.015f),  // Small accelerometer biases
            scale = new Vector3(1.001f, 0.999f, 1.002f), // Slight scale errors
            noiseStdDev = 0.01f,  // 1cm standard deviation for IMU
            correlationTime = 100.0f
        };
        
        sensorCalibrations["imu"] = imuCalib;
    }
    
    public void UpdateCalibration(string sensorName, CalibrationParams newParams)
    {
        if (sensorCalibrations.ContainsKey(sensorName))
        {
            sensorCalibrations[sensorName] = newParams;
            Debug.Log($"Calibration updated for {sensorName}");
        }
        else
        {
            sensorCalibrations[sensorName] = newParams;
            Debug.LogWarning($"Added new calibration for unknown sensor: {sensorName}");
        }
    }
    
    public Vector3 ApplyCalibration(string sensorName, Vector3 rawReading)
    {
        if (sensorCalibrations.ContainsKey(sensorName))
        {
            CalibrationParams calib = sensorCalibrations[sensorName];
            
            // Apply bias correction
            Vector3 corrected = rawReading - calib.bias;
            
            // Apply scale correction
            corrected.x *= calib.scale.x;
            corrected.y *= calib.scale.y;
            corrected.z *= calib.scale.z;
            
            return corrected;
        }
        
        Debug.LogWarning($"No calibration found for {sensorName}");
        return rawReading;
    }
    
    // Method to receive calibration updates from ROS
    public void ReceiveCalibrationUpdate(string sensorName, Vector3 bias, Vector3 scale)
    {
        if (sensorCalibrations.ContainsKey(sensorName))
        {
            sensorCalibrations[sensorName].bias = bias;
            sensorCalibrations[sensorName].scale = scale;
            
            Debug.Log($"Received calibration update for {sensorName}");
        }
    }
}
```

## Noise Modeling in Unity

For visualization purposes, Unity can implement noise models to make sensor visualizations more realistic:

```csharp
using UnityEngine;

public class SensorNoiseModel : MonoBehaviour
{
    [Header("Noise Parameters")]
    public float gaussianNoiseStdDev = 0.01f;
    public float bias = 0.0f;
    public float biasDriftRate = 0.001f;
    public float correlationTime = 1.0f;
    
    private float currentBias = 0.0f;
    private float lastUpdateTimestamp = 0.0f;
    
    // Simple noise generation
    private System.Random random = new System.Random();
    
    void Start()
    {
        // Initialize with random bias
        currentBias = (float)(random.NextDouble() - 0.5) * biasDriftRate * 10;
        lastUpdateTimestamp = Time.time;
    }
    
    void Update()
    {
        // Update time-correlated drift
        float timeDelta = Time.time - lastUpdateTimestamp;
        if (timeDelta > 0)
        {
            // Simple first-order Gauss-Markov process for bias drift
            float decayFactor = Mathf.Exp(-timeDelta / correlationTime);
            float randomWalk = (float)(random.NextDouble() - 0.5) * biasDriftRate * timeDelta;
            
            currentBias = currentBias * decayFactor + randomWalk;
            lastUpdateTimestamp = Time.time;
        }
    }
    
    public float AddNoise(float measurement)
    {
        // Add Gaussian noise
        float gaussianNoise = GenerateGaussianNoise();
        
        // Combine all error sources
        float totalNoise = gaussianNoise + currentBias + bias;
        
        return measurement + totalNoise;
    }
    
    public Vector3 AddNoiseToVector3(Vector3 measurement)
    {
        return new Vector3(
            AddNoise(measurement.x),
            AddNoise(measurement.y),
            AddNoise(measurement.z)
        );
    }
    
    // Generate noise using Box-Muller transform
    private float GenerateGaussianNoise()
    {
        // Generate two independent random values in [0, 1]
        float u1 = (float)random.NextDouble();
        float u2 = (float)random.NextDouble();
        
        // Apply Box-Muller transform
        float magnitude = Mathf.Sqrt(-2.0f * Mathf.Log(u1));
        float z0 = magnitude * Mathf.Cos(2.0f * Mathf.PI * u2);
        
        // Scale by standard deviation
        return z0 * gaussianNoiseStdDev;
    }
    
    // Alternative simple random noise (less realistic but faster)
    public float AddSimpleNoise(float measurement)
    {
        float noise = (float)(random.NextDouble() - 0.5) * 2.0f * gaussianNoiseStdDev;
        return measurement + noise + currentBias + bias;
    }
}
```

## Best Practices for Sensor Calibration

### 1. Regular Calibration Updates
- Periodically recalibrate sensors during operation
- Use in-situ calibration techniques when possible
- Monitor calibration parameters for drift

### 2. Environmental Considerations
- Account for temperature, humidity, and vibration effects
- Implement environmental compensation where possible
- Characterize sensor behavior under different conditions

### 3. Validation and Testing
- Validate calibrations using ground truth data when available
- Test calibrated sensors in various operational scenarios
- Monitor residuals after calibration to assess quality

By properly calibrating sensors and modeling their noise characteristics, digital twin applications can provide more accurate and reliable representations of physical systems, enabling better testing, validation, and development of robotic applications.
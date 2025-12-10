# Sensor Simulation: LiDAR, Depth Cameras, and IMUs

Sensor simulation is a crucial component of digital twin systems, providing the perception capabilities that allow robots to understand their environment. This section covers how to simulate sensors like LiDAR, depth cameras, and IMUs in both Gazebo and Unity environments, ensuring that the digital twin accurately reflects the sensing capabilities of the physical system.

## Introduction to Sensor Simulation

In digital twin systems, sensor simulation plays a vital role in:
- Providing realistic perception data for algorithms
- Testing navigation and mapping systems
- Validating sensor fusion techniques
- Debugging perception pipelines

The three primary sensor types covered here are:
- **LiDAR** (Light Detection and Ranging): Precise distance measurement using laser pulses
- **Depth Cameras**: 3D imaging systems that capture distance information per pixel
- **IMUs** (Inertial Measurement Units): Sensors that measure acceleration and rotation

## LiDAR Simulation

### Understanding LiDAR in Gazebo

LiDAR sensors in Gazebo simulate the behavior of real LiDAR units, emitting laser beams and measuring the time it takes for them to return after hitting objects. This creates a "point cloud" of distance measurements around the robot.

### LiDAR Configuration in URDF/SDF

Here's an example of how to configure a LiDAR sensor in a URDF file:

```xml
<!-- Example: Hokuyo UTM-30LX LiDAR sensor configuration -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>                 <!-- Number of rays per 360 degree sweep -->
          <resolution>1</resolution>              <!-- Number of rays per degree -->
          <min_angle>-3.14159</min_angle>         <!-- -180 degrees in radians -->
          <max_angle>3.14159</max_angle>          <!-- 180 degrees in radians -->
        </horizontal>
        <!-- Vertical configuration (for 3D LiDAR) -->
        <!--
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle>         <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>          <!-- 15 degrees -->
        </vertical>
        -->
      </scan>
      <range>
        <min>0.1</min>                           <!-- Minimum detection range in meters -->
        <max>30.0</max>                           <!-- Maximum detection range in meters -->
        <resolution>0.01</resolution>             <!-- Distance resolution in meters -->
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>        <!-- Remap output topic -->
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_frame</frame_name>
    </plugin>
    <always_on>true</always_on>
    <update_rate>40</update_rate>                <!-- Update rate in Hz -->
    <visualize>true</visualize>                  <!-- Whether to visualize sensor rays -->
  </sensor>
</gazebo>
```

### Advanced LiDAR Configuration

For more sophisticated LiDAR configurations:

```xml
<!-- Example: 3D LiDAR configuration (like Velodyne VLP-16) -->
<gazebo reference="velodyne_link">
  <sensor type="ray" name="velodyne_sensor">
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>2</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.2</min>
        <max>100.0</max>
        <resolution>0.001</resolution>
      </range>
    </ray>
    
    <!-- Add noise to make the simulation more realistic -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- 5mm noise standard deviation -->
    </noise>
    
    <plugin name="velodyne_driver" filename="libgazebo_ros_velodyne_gpu_laser.so">
      <topicName>/velodyne_points</topicName>
      <frameName>velodyne_frame</frameName>
      <min_range>0.9</min_range>
      <max_range>130.0</max_range>
      <gaussian_noise>0.008</gaussian_noise>
    </plugin>
    
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>false</visualize>  <!-- Turn off visualization for performance -->
  </sensor>
</gazebo>
```

### LiDAR in Unity Visualization

To visualize LiDAR data in Unity, you can render the point cloud data received from Gazebo:

```csharp
// LidarPointCloudVisualizer.cs
using UnityEngine;
using System.Collections.Generic;

public class LidarPointCloudVisualizer : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    public bool visualizeLidar = true;
    public int maxPoints = 10000;
    public float pointSize = 0.05f;
    public Color pointColor = Color.red;
    public float maxVisualizationRange = 30.0f;
    
    [Header("Performance Settings")]
    public bool useObjectPooling = true;
    public int poolSize = 1000;
    
    private List<GameObject> pointPool;
    private List<GameObject> activePoints;
    private Mesh pointMesh;
    private Material pointMaterial;
    
    void Start()
    {
        InitializePointCloud();
    }
    
    void InitializePointCloud()
    {
        // Create a simple sphere mesh for each LiDAR point
        pointMesh = CreatePointMesh();
        
        // Create a material for the points
        pointMaterial = new Material(Shader.Find("Particles/Multiply")) 
        { 
            color = pointColor 
        };
        
        // Initialize object pooling if enabled
        if (useObjectPooling)
        {
            pointPool = new List<GameObject>();
            activePoints = new List<GameObject>();
            
            for (int i = 0; i < poolSize; i++)
            {
                GameObject point = CreatePointInstance();
                point.SetActive(false);
                pointPool.Add(point);
            }
        }
    }
    
    Mesh CreatePointMesh()
    {
        // Create a simple sphere mesh
        Mesh mesh = new Mesh();
        
        // For simplicity, we'll use a quad facing the camera
        // In reality, you might use a small sphere
        Vector3[] vertices = {
            new Vector3(-1, -1, 0),
            new Vector3(1, -1, 0),
            new Vector3(-1, 1, 0),
            new Vector3(1, 1, 0)
        };
        
        int[] triangles = { 0, 1, 2, 1, 3, 2 };
        
        Vector2[] uv = {
            new Vector2(0, 0),
            new Vector2(1, 0),
            new Vector2(0, 1),
            new Vector2(1, 1)
        };
        
        Vector3[] normals = {
            -Vector3.forward,
            -Vector3.forward,
            -Vector3.forward,
            -Vector3.forward
        };
        
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uv;
        mesh.normals = normals;
        
        return mesh;
    }
    
    GameObject CreatePointInstance()
    {
        GameObject point = new GameObject("LidarPoint");
        point.transform.SetParent(transform);
        
        MeshFilter meshFilter = point.AddComponent<MeshFilter>();
        meshFilter.mesh = pointMesh;
        
        MeshRenderer meshRenderer = point.AddComponent<MeshRenderer>();
        meshRenderer.material = pointMaterial;
        
        return point;
    }
    
    public void UpdateLidarPointCloud(List<Vector3> lidarPoints)
    {
        if (!visualizeLidar) return;
        
        ClearPreviousPoints();
        
        int pointsToShow = Mathf.Min(lidarPoints.Count, maxPoints);
        
        for (int i = 0; i < pointsToShow; i++)
        {
            Vector3 point = lidarPoints[i];
            
            // Only visualize points within our range
            if (point.magnitude <= maxVisualizationRange)
            {
                GameObject pointObj = GetOrCreatePoint();
                pointObj.transform.position = point;
                pointObj.transform.localScale = Vector3.one * pointSize;
                pointObj.SetActive(true);
                
                activePoints.Add(pointObj);
            }
        }
    }
    
    GameObject GetOrCreatePoint()
    {
        if (useObjectPooling)
        {
            foreach (GameObject point in pointPool)
            {
                if (!point.activeInHierarchy)
                {
                    return point;
                }
            }
            
            // Pool is exhausted, create a new one
            GameObject newPoint = CreatePointInstance();
            pointPool.Add(newPoint);
            return newPoint;
        }
        else
        {
            return CreatePointInstance();
        }
    }
    
    void ClearPreviousPoints()
    {
        if (useObjectPooling)
        {
            foreach (GameObject point in activePoints)
            {
                point.SetActive(false);
            }
            activePoints.Clear();
        }
        else
        {
            // Destroy all child objects
            foreach (Transform child in transform)
            {
                DestroyImmediate(child.gameObject);
            }
        }
    }
    
    // Alternative method using Graphics.DrawMesh for better performance
    public void RenderLidarPointCloudWithDrawMesh(List<Vector3> lidarPoints)
    {
        if (!visualizeLidar) return;
        
        Matrix4x4[] matrices = new Matrix4x4[Mathf.Min(lidarPoints.Count, maxPoints)];
        MaterialPropertyBlock properties = new MaterialPropertyBlock();
        
        int validPointCount = 0;
        
        for (int i = 0; i < lidarPoints.Count && i < maxPoints; i++)
        {
            Vector3 point = lidarPoints[i];
            
            // Only render points within our visualization range
            if (point.magnitude <= maxVisualizationRange)
            {
                Matrix4x4 pointMatrix = Matrix4x4.TRS(
                    point,
                    Quaternion.identity,
                    Vector3.one * pointSize
                );
                
                matrices[validPointCount] = pointMatrix;
                validPointCount++;
            }
        }
        
        // Batch render all points
        Graphics.DrawMeshInstanced(
            pointMesh,
            0,
            pointMaterial,
            matrices,
            validPointCount,
            properties,
            UnityEngine.Rendering.ShadowCastingMode.Off,
            false,
            gameObject.layer
        );
    }
    
    // Method to receive LiDAR data from ROS or other sources
    public void OnLidarDataReceived(float[] ranges, float angleMin, float angleMax, float angleIncrement)
    {
        List<Vector3> points = new List<Vector3>();
        
        // Convert ranges and angle information to 3D points
        for (int i = 0; i < ranges.Length; i++)
        {
            float angle = angleMin + i * angleIncrement;
            float range = ranges[i];
            
            // Skip invalid ranges
            if (range > 0 && !float.IsNaN(range) && !float.IsInfinity(range) && range <= maxVisualizationRange)
            {
                // Calculate 2D position (assuming LiDAR is level)
                float x = Mathf.Cos(angle) * range;
                float y = Mathf.Sin(angle) * range;
                
                // Create 3D point (z=0 for 2D LiDAR)
                Vector3 point = new Vector3(x, 0, y); // Unity: X-right, Y-up, Z-forward
                points.Add(point);
            }
        }
        
        UpdateLidarPointCloud(points);
    }
}
```

## Depth Camera Simulation

### Depth Camera Configuration in Gazebo

Depth cameras in Gazebo simulate RGB-D cameras like the Kinect or Intel RealSense, providing both color and depth information:

```xml
<!-- Example: Depth Camera configuration -->
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>30</update_rate>
    
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>    <!-- 60 degrees in radians -->
      <image>
        <format>R8G8B8</format>                 <!-- Color format -->
        <width>640</width>                       <!-- Image width -->
        <height>480</height>                     <!-- Image height -->
      </image>
      <clip>
        <near>0.1</near>                         <!-- Near clipping distance -->
        <far>10.0</far>                          <!-- Far clipping distance -->
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>                   <!-- Noise in meters -->
      </noise>
    </camera>
    
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.2</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <point_cloud_cutoff>0.3</point_cloud_cutoff>            <!-- Min depth for point cloud -->
      <point_cloud_cutoff_max>5.0</point_cloud_cutoff_max>    <!-- Max depth for point cloud -->
      <Cx_prime>0</Cx_prime>
      <Cx>320.5</Cx>                            <!-- Principal point X -->
      <Cy>240.5</Cy>                            <!-- Principal point Y -->
      <focal_length>320.0</focal_length>        <!-- Focal length in pixels -->
      <frame_name>camera_depth_frame</frame_name>
      <camera_name>camera</camera_name>
      
      <!-- ROS topic configuration -->
      <image_topic_name>/camera/image_raw</image_topic_name>
      <depth_image_topic_name>/camera/depth/image_raw</depth_image_topic_name>
      <point_cloud_topic_name>/camera/depth/points</point_cloud_topic_name>
      <camera_info_topic_name>/camera/camera_info</camera_info_topic_name>
    </plugin>
    
    <always_on>true</always_on>
    <visualize>false</visualize>
  </sensor>
</gazebo>
```

### Depth Camera Visualization in Unity

```csharp
// DepthCameraVisualizer.cs
using UnityEngine;
using System.Threading.Tasks;
using System.Collections.Generic;

public class DepthCameraVisualizer : MonoBehaviour
{
    [Header("Depth Camera Settings")]
    public int width = 640;
    public int height = 480;
    public float nearClip = 0.1f;
    public float farClip = 10.0f;
    public bool visualizeDepth = true;
    public bool visualizePointCloud = true;
    public float pointCloudResolution = 0.05f;  // Resolution in meters for point cloud sampling
    
    [Header("Display Options")]
    public Renderer imageDisplay;                 // For displaying the RGB image
    public Renderer depthDisplay;                 // For displaying the depth visualization
    public Material depthMaterial;                // Material for depth visualization
    public GameObject pointCloudParent;           // Parent for point cloud objects
    
    // Private members
    private Texture2D rgbTexture;
    private Texture2D depthTexture;
    private Color32[] rgbPixels;
    private float[] depthValues;
    private List<GameObject> pointCloudObjects;
    private Mesh pointMesh;
    private Material pointMaterial;
    
    void Start()
    {
        InitializeDepthCamera();
    }
    
    void InitializeDepthCamera()
    {
        // Initialize textures
        rgbTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        depthTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        
        // Initialize pixel arrays
        rgbPixels = new Color32[width * height];
        depthValues = new float[width * height];
        
        // Initialize point cloud
        pointCloudObjects = new List<GameObject>();
        pointMesh = CreatePointMesh();
        pointMaterial = new Material(Shader.Find("Unlit/Color")) { color = Color.blue };
        
        // Set up display materials
        if (imageDisplay != null && rgbTexture != null)
        {
            imageDisplay.material.mainTexture = rgbTexture;
        }
        
        if (depthDisplay != null && depthTexture != null)
        {
            depthDisplay.material.mainTexture = depthTexture;
            if (depthMaterial != null)
            {
                depthDisplay.material = depthMaterial;
            }
        }
    }
    
    Mesh CreatePointMesh()
    {
        // Create a small sphere for each depth point
        Mesh sphereMesh = new Mesh();
        CreateSphereMesh(sphereMesh, 0.01f, 8, 6); // Small sphere
        return sphereMesh;
    }
    
    void CreateSphereMesh(Mesh mesh, float radius, int segments, int rings)
    {
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        // Create vertices
        for (int ring = 0; ring <= rings; ring++)
        {
            float v = (float)ring / rings;
            float phi = v * Mathf.PI;

            for (int seg = 0; seg <= segments; seg++)
            {
                float u = (float)seg / segments;
                float theta = u * Mathf.PI * 2;

                float x = radius * Mathf.Sin(phi) * Mathf.Cos(theta);
                float y = radius * Mathf.Cos(phi);
                float z = radius * Mathf.Sin(phi) * Mathf.Sin(theta);

                vertices.Add(new Vector3(x, y, z));
            }
        }

        // Create triangles
        for (int ring = 0; ring < rings; ring++)
        {
            for (int seg = 0; seg < segments; seg++)
            {
                int current = ring * (segments + 1) + seg;
                int next = current + segments + 1;

                triangles.Add(current);
                triangles.Add(current + 1);
                triangles.Add(next + 1);

                triangles.Add(current);
                triangles.Add(next + 1);
                triangles.Add(next);
            }
        }

        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
    
    public void UpdateDepthCameraData(Color32[] rgbData, float[] depthData)
    {
        if (rgbData.Length != width * height || depthData.Length != width * height)
        {
            Debug.LogError("Depth camera data doesn't match expected dimensions");
            return;
        }
        
        // Update RGB texture
        if (rgbData != null)
        {
            rgbPixels = rgbData;
            rgbTexture.SetPixels32(rgbPixels);
            rgbTexture.Apply();
        }
        
        // Update depth texture and visualization
        if (depthData != null)
        {
            depthValues = depthData;
            UpdateDepthVisualization();
        }
        
        // Update point cloud if enabled
        if (visualizePointCloud)
        {
            UpdatePointCloud();
        }
    }
    
    void UpdateDepthVisualization()
    {
        // Convert depth values to a visual representation
        Color32[] depthColors = new Color32[width * height];
        
        for (int i = 0; i < depthValues.Length; i++)
        {
            float depth = depthValues[i];
            
            // Normalize depth value to 0-1 range for visualization
            float normalizedDepth = Mathf.InverseLerp(nearClip, farClip, depth);
            normalizedDepth = Mathf.Clamp01(normalizedDepth);
            
            // Create a color based on depth (blue for close, red for far)
            Color depthColor = Color.Lerp(Color.blue, Color.red, normalizedDepth);
            
            // Also add intensity based on depth certainty
            float intensity = 1.0f - Mathf.Pow(normalizedDepth, 0.5f);
            depthColor.a = intensity;
            
            depthColors[i] = depthColor;
        }
        
        depthTexture.SetPixels32(depthColors);
        depthTexture.Apply();
    }
    
    void UpdatePointCloud()
    {
        // Clear previous point cloud
        foreach (GameObject point in pointCloudObjects)
        {
            DestroyImmediate(point);
        }
        pointCloudObjects.Clear();
        
        if (pointCloudParent == null) return;
        
        // Sample depth points to create point cloud
        for (int y = 0; y < height; y += (int)(1.0f / pointCloudResolution))
        {
            for (int x = 0; x < width; x += (int)(1.0f / pointCloudResolution))
            {
                int idx = y * width + x;
                if (idx >= depthValues.Length) continue;
                
                float depth = depthValues[idx];
                
                // Skip invalid depth values
                if (depth <= 0 || depth > farClip || float.IsNaN(depth) || float.IsInfinity(depth))
                    continue;
                
                // Convert pixel coordinates to world coordinates
                Vector3 worldPoint = PixelToWorldSpace(x, y, depth);
                
                // Create point visualization
                GameObject pointObj = new GameObject($"DepthPoint_{x}_{y}");
                pointObj.transform.SetParent(pointCloudParent.transform);
                pointObj.transform.position = worldPoint;
                
                // Add mesh renderer
                MeshFilter meshFilter = pointObj.AddComponent<MeshFilter>();
                meshFilter.mesh = pointMesh;
                
                MeshRenderer meshRenderer = pointObj.AddComponent<MeshRenderer>();
                meshRenderer.material = pointMaterial;
                
                pointCloudObjects.Add(pointObj);
            }
        }
    }
    
    Vector3 PixelToWorldSpace(int pixelX, int pixelY, float depth)
    {
        // Convert pixel coordinates to normalized device coordinates
        float u = (pixelX - width * 0.5f) / (width * 0.5f);
        float v = (pixelY - height * 0.5f) / (height * 0.5f);
        
        // Apply camera intrinsic parameters
        // These should match the parameters used in Gazebo
        float fx_inv = width / (2 * Mathf.Tan(60 * Mathf.Deg2Rad / 2)); // Horizontal FOV: 60 deg
        float fy_inv = height / (2 * Mathf.Tan(48 * Mathf.Deg2Rad / 2)); // Vertical FOV: 48 deg
        
        // Calculate 3D point from depth
        float x = u * depth / fx_inv;
        float y = v * depth / fy_inv;
        float z = depth;
        
        // In Gazebo/ROS coordinates, Z is forward, Y is up, X is right
        // Convert to Unity coordinates: Z is forward, Y is up, X is right
        // Actually, in ROS: X-forward, Y-left, Z-up; in Unity: X-right, Y-up, Z-forward
        // So we need to transform: [x,y,z]_ROS -> [z,-x,y]_Unity
        return new Vector3(z, -x, y);
    }
    
    // Alternative method using compute shader for better performance on large point clouds
    public void UpdatePointCloudWithComputeShader()
    {
        // This would use a compute shader to generate the point cloud more efficiently
        // Implementation would involve sending depth texture to shader and letting the GPU
        // generate the point cloud vertices
        Debug.Log("Compute shader point cloud generation would be implemented here for performance");
    }
}
```

## IMU Simulation

### IMU Configuration in Gazebo

IMUs measure acceleration and angular velocity. Here's how to configure an IMU sensor in Gazebo:

```xml
<!-- Example: IMU sensor configuration -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>              <!-- 100 Hz update rate -->
    <visualize>false</visualize>
    <topic>__default_topic__</topic>
    
    <!-- IMU plugin configuration -->
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <topicName>imu/data</topicName>
      <bodyName>imu_link</bodyName>             <!-- Link to attach the IMU to -->
      <updateRateHZ>100.0</updateRateHZ>
      <gaussianNoise>0.01</gaussianNoise>        <!-- Noise level -->
      <xyzOffset>0 0 0</xyzOffset>               <!-- Offset from link origin -->
      <rpyOffset>0 0 0</rpyOffset>               <!-- Rotation offset -->
      <frameName>imu_frame</frameName>           <!-- Frame name for TF -->
    </plugin>
    
    <!-- Noise model for the IMU -->
    <noise type="gaussian">
      <angular_velocity>
        <x>
          <mean>0.0</mean>
          <stddev>0.001</stddev>                <!-- 1 mrad/s standard deviation for X-axis gyro -->
        </x>
        <y>
          <mean>0.0</mean>
          <stddev>0.001</stddev>                <!-- Y-axis gyro noise -->
        </y>
        <z>
          <mean>0.0</mean>
          <stddev>0.001</stddev>                <!-- Z-axis gyro noise -->
        </z>
      </angular_velocity>
      
      <linear_acceleration>
        <x>
          <mean>0.0</mean>
          <stddev>0.017</stddev>                <!-- 1.7 mg standard deviation for X-axis accelerometer -->
        </x>
        <y>
          <mean>0.0</mean>
          <stddev>0.017</stddev>                <!-- Y-axis accelerometer noise -->
        </y>
        <z>
          <mean>0.0</mean>
          <stddev>0.017</stddev>                <!-- Z-axis accelerometer noise -->
        </z>
      </linear_acceleration>
    </noise>
    
    <imu>
      <!-- Orientation is not needed, as it's calculated from angular velocity and acceleration -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>               <!-- 1.7 mg = 0.016669 m/s² -->
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### IMU Data Processing in Unity

While IMUs don't have a direct visual representation, their data can be used to visualize robot motion and orientation:

```csharp
// ImuVisualizer.cs
using UnityEngine;

public class ImuVisualizer : MonoBehaviour
{
    [Header("IMU Configuration")]
    public float linearAccelerationNoise = 0.017f;  // In m/s²
    public float angularVelocityNoise = 0.001f;     // In rad/s
    public float magneticFieldNoise = 0.0001f;      // In Tesla (if magnetometer simulated)
    
    [Header("Visualization Settings")]
    public bool showOrientation = true;
    public bool showAccelerationVector = true;
    public bool showAngularVelocity = true;
    public float vectorScale = 0.1f;
    
    [Header("Display Elements")]
    public GameObject orientationIndicator;         // Visual indicator for orientation
    public GameObject accelerationVector;            // Arrow for acceleration
    public GameObject angularVelocityVector;         // Arrow for angular velocity
    
    // IMU data buffers
    private Vector3 lastLinearAcceleration;
    private Vector3 currentLinearAcceleration;
    private Vector3 angularVelocity;
    private Vector3 magneticField;                   // If magnetometer data is available
    
    // Visualization helpers
    private LineRenderer accLineRenderer;
    private LineRenderer angVelLineRenderer;
    private Transform robotTransform;
    
    void Start()
    {
        InitializeImuVisualization();
    }
    
    void InitializeImuVisualization()
    {
        robotTransform = transform.parent != null ? transform.parent : transform;
        
        // Initialize visualization objects
        if (orientationIndicator == null)
        {
            orientationIndicator = CreateOrientationIndicator();
        }
        
        if (accelerationVector == null && showAccelerationVector)
        {
            accelerationVector = CreateVectorIndicator(Color.red, "Acceleration");
            accLineRenderer = accelerationVector.GetComponent<LineRenderer>();
        }
        
        if (angularVelocityVector == null && showAngularVelocity)
        {
            angularVelocityVector = CreateVectorIndicator(Color.blue, "AngularVelocity");
            angVelLineRenderer = angularVelocityVector.GetComponent<LineRenderer>();
        }
        
        // Initialize with zero values
        currentLinearAcceleration = Vector3.zero;
        angularVelocity = Vector3.zero;
        magneticField = Vector3.zero;
    }
    
    GameObject CreateOrientationIndicator()
    {
        GameObject indicator = GameObject.CreatePrimitive(PrimitiveType.Cube);
        indicator.name = "OrientationIndicator";
        indicator.transform.SetParent(transform);
        indicator.transform.localPosition = Vector3.zero;
        indicator.transform.localScale = Vector3.one * 0.1f;
        
        // Remove collider since it's just for visualization
        DestroyImmediate(indicator.GetComponent<BoxCollider>());
        
        return indicator;
    }
    
    GameObject CreateVectorIndicator(Color color, string name)
    {
        GameObject vectorObj = new GameObject(name);
        vectorObj.transform.SetParent(transform);
        
        LineRenderer lineRenderer = vectorObj.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Unlit/Color")) { color = color };
        lineRenderer.startWidth = 0.02f;
        lineRenderer.endWidth = 0.01f;
        lineRenderer.positionCount = 2;
        
        return vectorObj;
    }
    
    public void ProcessImuData(Vector3 linearAccel, Vector3 angVel, Vector3 magField = default)
    {
        // Apply noise to make the data more realistic
        currentLinearAcceleration = AddNoiseToVector(linearAccel, linearAccelerationNoise);
        angularVelocity = AddNoiseToVector(angVel, angularVelocityNoise);
        magneticField = magField == default ? magneticField : AddNoiseToVector(magField, magneticFieldNoise);
        
        // Update visualization
        UpdateVisualization();
    }
    
    Vector3 AddNoiseToVector(Vector3 input, float noiseLevel)
    {
        return new Vector3(
            input.x + Random.Range(-noiseLevel, noiseLevel),
            input.y + Random.Range(-noiseLevel, noiseLevel),
            input.z + Random.Range(-noiseLevel, noiseLevel)
        );
    }
    
    void UpdateVisualization()
    {
        // Update orientation indicator rotation
        if (showOrientation && orientationIndicator != null)
        {
            // This would typically come from integrating angular velocity over time
            // For this example, we'll apply a rotation based on angular velocity
            orientationIndicator.transform.rotation = transform.rotation * 
                Quaternion.Euler(angularVelocity * Mathf.Rad2Deg * Time.deltaTime * 10);
        }
        
        // Update acceleration vector visualization
        if (showAccelerationVector && accelerationVector != null)
        {
            UpdateVectorIndicator(accLineRenderer, currentLinearAcceleration * vectorScale, Color.red);
        }
        
        // Update angular velocity vector visualization
        if (showAngularVelocity && angularVelocityVector != null)
        {
            UpdateVectorIndicator(angVelLineRenderer, angularVelocity * vectorScale * 100, Color.blue);
        }
    }
    
    void UpdateVectorIndicator(LineRenderer lineRenderer, Vector3 vector, Color color)
    {
        if (lineRenderer == null) return;
        
        lineRenderer.SetPosition(0, transform.position);
        lineRenderer.SetPosition(1, transform.position + transform.rotation * vector);
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
    }
    
    // Integration methods for estimating pose from IMU data
    public Vector3 IntegrateAcceleration(Vector3 acceleration, float deltaTime)
    {
        // Simple integration: velocity = acceleration * time
        // In practice, you'd need to account for drift, gravity removal, etc.
        return acceleration * deltaTime;
    }
    
    public Quaternion IntegrateAngularVelocity(Vector3 angularVel, float deltaTime, Quaternion currentOrientation)
    {
        // Integrate angular velocity to estimate orientation change
        Vector3 rotation = angularVel * deltaTime;
        Quaternion deltaRotation = Quaternion.Euler(rotation * Mathf.Rad2Deg);
        
        return currentOrientation * deltaRotation;
    }
    
    // Method to visualize estimated trajectory from IMU integration
    public void VisualizeImuTrajectory()
    {
        // This would maintain a history of positions based on IMU integration
        // and visualize the estimated path
        Debug.Log("IMU trajectory visualization would be implemented here");
    }
    
    // Performance optimization methods for real-time processing
    public void ProcessImuDataOptimized(Vector3[] linearAccelArray, Vector3[] angVelArray, int count)
    {
        // Process multiple IMU readings at once for better performance
        for (int i = 0; i < count && i < linearAccelArray.Length && i < angVelArray.Length; i++)
        {
            ProcessImuData(linearAccelArray[i], angVelArray[i]);
        }
    }
}
```

## Sensor Fusion Example

Combining data from multiple sensors is often necessary in robotics:

```csharp
// SensorFusionVisualizer.cs
using UnityEngine;
using System.Collections.Generic;

public class SensorFusionVisualizer : MonoBehaviour
{
    [Header("Sensor Components")]
    public LidarPointCloudVisualizer lidarVisualizer;
    public DepthCameraVisualizer depthCameraVisualizer;
    public ImuVisualizer imuVisualizer;
    
    [Header("Fusion Settings")]
    public float fusionTrustThreshold = 0.9f;
    public bool visualizeFusedData = true;
    
    // Internal state
    private List<Vector3> fusedPointCloud = new List<Vector3>();
    private Vector3 estimatedPosition;
    private Quaternion estimatedOrientation;
    
    void Start()
    {
        InitializeSensorFusion();
    }
    
    void InitializeSensorFusion()
    {
        fusedPointCloud = new List<Vector3>();
        estimatedPosition = Vector3.zero;
        estimatedOrientation = Quaternion.identity;
    }
    
    public void FuseSensorData()
    {
        // Clear the fused point cloud
        fusedPointCloud.Clear();
        
        // Get data from individual sensors
        // In a real system, this would happen asynchronously
        // For this example, we'll simulate fusion of point clouds
        
        // In a real implementation, you'd:
        // 1. Transform LiDAR points to global frame using current pose estimate
        // 2. Transform depth camera points similarly
        // 3. Apply sensor fusion algorithm (Kalman filter, particle filter, etc.)
        // 4. Update pose estimate based on sensor data
        
        // For visualization purposes, let's simply merge the point clouds
        if (lidarVisualizer != null)
        {
            // Get LiDAR points and transform to world space
            // This is a simplified example
        }
        
        if (depthCameraVisualizer != null)
        {
            // Get depth camera points and transform to world space
            // This is a simplified example
        }
        
        // Update pose estimate based on IMU data
        if (imuVisualizer != null)
        {
            // Use IMU to adjust position/orientation estimates
            // This is a simplified example
        }
        
        // Visualize the fused data
        if (visualizeFusedData)
        {
            UpdateFusedVisualization();
        }
    }
    
    void UpdateFusedVisualization()
    {
        // Visualize fused point cloud
        if (lidarVisualizer != null)
        {
            lidarVisualizer.RenderLidarPointCloudWithDrawMesh(fusedPointCloud);
        }
        
        // Update other visualizations based on fused data
        transform.position = estimatedPosition;
        transform.rotation = estimatedOrientation;
    }
    
    // Method for debugging sensor correlation
    public void DebugSensorCorrelation()
    {
        Debug.Log("--- Sensor Fusion Debug Info ---");
        
        if (lidarVisualizer != null)
        {
            Debug.Log("LiDAR active points: [Implementation dependent]");
        }
        
        if (depthCameraVisualizer != null)
        {
            Debug.Log($"Depth camera resolution: {depthCameraVisualizer.width}x{depthCameraVisualizer.height}");
        }
        
        Debug.Log($"Estimated position: {estimatedPosition}");
        Debug.Log($"Estimated orientation: {estimatedOrientation.eulerAngles}");
        Debug.Log("--- End Debug Info ---");
    }
}
```

## Best Practices for Sensor Simulation

### 1. Realistic Noise Modeling
- Always add appropriate noise to sensor data
- Match noise characteristics to real sensors
- Consider drift in IMUs over time

### 2. Performance Considerations
- Use efficient rendering methods for large point clouds (Graphics.DrawMeshInstanced)
- Consider level-of-detail approaches for sensor visualization
- Optimize update rates for real-time performance

### 3. Validation Approaches
- Compare simulated sensor data to real sensor data
- Validate that algorithms behave similarly in simulation and reality
- Test edge cases and failure conditions

These sensor simulation techniques ensure that your digital twin system accurately represents the sensing capabilities of the physical system, enabling thorough testing and validation of perception algorithms.
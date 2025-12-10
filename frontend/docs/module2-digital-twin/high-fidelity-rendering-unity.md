# High-Fidelity Rendering in Unity

Unity provides the visualization component of our digital twin system, offering high-fidelity rendering that can accurately represent the physical system being simulated. This section covers how to create compelling visualizations that complement the physics simulation in Gazebo.

## Introduction to Unity Visualization

Unity is a powerful real-time 3D development platform that excels at creating high-quality visualizations. When combined with Gazebo's physics simulation, we can create digital twins that are both physically accurate and visually compelling.

### Key Rendering Components

1. **Lighting**: Realistic illumination and shadows
2. **Materials & Textures**: Surface properties and appearance
3. **Camera Systems**: Perspectives and viewport management
4. **Human-Robot Interaction**: UI elements and visualization tools

## Lighting in Unity

Lighting is crucial for creating realistic and informative visualizations in Unity. Proper lighting can help users understand spatial relationships and identify important elements in the digital twin.

### Types of Lighting

Unity supports several types of lights:
- **Directional Lights** (for simulating sun/moon)
- **Point Lights** (for bulbs, explosions)
- **Spot Lights** (for flashlights, car headlights)
- **Area Lights** (for soft lighting)

### Setting Up Lighting for Digital Twins

For digital twin applications, we typically want to simulate real-world lighting conditions:

```csharp
// Example: Creating and configuring lights for a digital twin environment
using UnityEngine;

public class DigitalTwinLighting : MonoBehaviour
{
    [Header("Lighting Configuration")]
    public float simulationTime = 12.0f; // 24-hour format (12.0 = noon)
    public float latitude = 40.0f;       // Geographic latitude
    public float longitude = -74.0f;     // Geographic longitude
    
    [Header("Light References")]
    public Light sunLight;
    public GameObject moonLight;
    public Light[] ambientLights;
    
    void Start()
    {
        SetupEnvironmentLighting();
        UpdateLightingForTime(simulationTime);
    }

    void SetupEnvironmentLighting()
    {
        // Configure the main directional light as the sun
        if (sunLight == null)
        {
            GameObject sunObj = new GameObject("DigitalTwinSun");
            sunLight = sunObj.AddComponent<Light>();
            sunLight.type = LightType.Directional;
            sunLight.color = Color.white;
            sunLight.intensity = 1.0f;
        }
        
        // Set up ambient lights for environment
        ConfigureAmbientLighting();
    }
    
    void ConfigureAmbientLighting()
    {
        // Set skybox-based ambient lighting
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = new Color(0.212f, 0.227f, 0.259f);
        RenderSettings.ambientEquatorColor = new Color(0.114f, 0.125f, 0.133f);
        RenderSettings.ambientGroundColor = new Color(0.047f, 0.043f, 0.035f);
    }
    
    public void UpdateLightingForTime(float hourOfDay)
    {
        // Calculate sun position based on time of day and geographic location
        float sunAngle = CalculateSunAngle(hourOfDay, latitude, longitude);
        
        // Position the sun light based on calculated angle
        Vector3 sunDirection = CalculateSunDirection(sunAngle);
        sunLight.transform.rotation = Quaternion.LookRotation(sunDirection);
        
        // Adjust intensity based on sun angle (sunrise/sunset is dimmer)
        float intensityFactor = CalculateIntensityFactor(sunAngle);
        sunLight.intensity = 1.0f * intensityFactor;
        
        // Update color temperature based on time of day
        UpdateLightColorForTime(hourOfDay);
    }
    
    float CalculateSunAngle(float hour, float lat, float lng)
    {
        // Simplified sun position calculation
        // In a real implementation, you'd use astronomical algorithms
        float hourAngle = (hour - 12.0f) * 15.0f; // 15 degrees per hour
        float declination = 23.45f * Mathf.Sin((hour - 81.0f) * 2 * Mathf.PI / 365.0f); // Simplified
        
        // Calculate sun elevation
        float latRad = lat * Mathf.PI / 180.0f;
        float elevation = Mathf.Asin(
            Mathf.Sin(declination * Mathf.PI / 180.0f) * Mathf.Sin(latRad) +
            Mathf.Cos(declination * Mathf.PI / 180.0f) * Mathf.Cos(latRad) * Mathf.Cos(hourAngle * Mathf.PI / 180.0f)
        ) * 180.0f / Mathf.PI;
        
        return elevation;
    }
    
    Vector3 CalculateSunDirection(float elevation)
    {
        // Calculate direction vector from elevation
        float angleRad = elevation * Mathf.PI / 180.0f;
        float y = Mathf.Sin(angleRad);
        float horizontal = Mathf.Cos(angleRad);
        
        // For simplified calculation, assume south direction
        float x = 0.0f;
        float z = -horizontal; // South is negative Z in Unity's coordinate system
        
        return new Vector3(x, y, z).normalized;
    }
    
    float CalculateIntensityFactor(float sunAngle)
    {
        // Adjust intensity based on sun elevation
        if (sunAngle < 0) return 0.1f; // Night or below horizon
        
        float normalizedAngle = Mathf.Clamp01((sunAngle + 30.0f) / 90.0f); // Clamp between -30° and 60°
        return Mathf.Lerp(0.2f, 1.0f, normalizedAngle);
    }
    
    void UpdateLightColorForTime(float hour)
    {
        Color lightColor;
        
        if (hour >= 6 && hour < 8) // Dawn
        {
            lightColor = new Color(1.0f, 0.7f, 0.4f, 1.0f);
        }
        else if (hour >= 8 && hour < 17) // Mid-day
        {
            lightColor = Color.white;
        }
        else if (hour >= 17 && hour < 19) // Dusk
        {
            lightColor = new Color(1.0f, 0.6f, 0.3f, 1.0f);
        }
        else // Night
        {
            lightColor = new Color(0.2f, 0.2f, 0.3f, 1.0f);
        }
        
        sunLight.color = lightColor;
    }
}
```

## Materials and Textures

Materials and textures give objects their visual appearance in Unity, defining how they interact with light and what they look like.

### Material Properties

Key properties of Unity materials include:
- **Albedo** (base color): The base color of the surface
- **Metallic**: How metallic the surface appears
- **Smoothness**: How smooth or rough the surface is
- **Normal maps**: Surface detail without geometry
- **Emission**: Light emitted by the surface

### Creating Robot-Specific Materials

For digital twin applications, materials should match the real-world robot's appearance:

```csharp
// Example: Creating materials for robot components
using UnityEngine;

[CreateAssetMenu(fileName = "RobotMaterialConfig", menuName = "Digital Twin/Material Configuration")]
public class RobotMaterialConfig : ScriptableObject
{
    [Header("Base Materials")]
    public Material robotChassisMaterial;
    public Material wheelMaterial;
    public Material sensorMaterial;
    public Material cableMaterial;
    
    [Header("Color Variants")]
    public Color primaryRobotColor = Color.blue;
    public Color secondaryRobotColor = Color.gray;
    public Color sensorColor = Color.black;
    
    [Header("Surface Properties")]
    public float robotMetallic = 0.2f;
    public float robotSmoothness = 0.5f;
    public float sensorMetallic = 0.9f;
    public float sensorSmoothness = 0.8f;
}

public class RobotMaterialManager : MonoBehaviour
{
    [SerializeField] private RobotMaterialConfig materialConfig;
    
    void Start()
    {
        ApplyRobotMaterials();
    }
    
    void ApplyRobotMaterials()
    {
        // Apply primary material to chassis
        if (materialConfig.robotChassisMaterial != null)
        {
            GameObject chassis = transform.Find("Chassis");
            if (chassis != null)
            {
                Renderer chassisRenderer = chassis.GetComponent<Renderer>();
                if (chassisRenderer != null)
                {
                    chassisRenderer.material = CreateCustomizedMaterial(
                        materialConfig.robotChassisMaterial, 
                        materialConfig.primaryRobotColor,
                        materialConfig.robotMetallic,
                        materialConfig.robotSmoothness
                    );
                }
            }
        }
        
        // Apply material to wheels
        if (materialConfig.wheelMaterial != null)
        {
            GameObject[] wheels = GameObject.FindGameObjectsWithTag("Wheel");
            foreach (GameObject wheel in wheels)
            {
                Renderer wheelRenderer = wheel.GetComponent<Renderer>();
                if (wheelRenderer != null)
                {
                    wheelRenderer.material = CreateCustomizedMaterial(
                        materialConfig.wheelMaterial,
                        materialConfig.secondaryRobotColor,
                        0.7f,  // Higher metallic for rubber
                        0.3f   // Lower smoothness for texture
                    );
                }
            }
        }
        
        // Apply material to sensors
        if (materialConfig.sensorMaterial != null)
        {
            GameObject[] sensors = GameObject.FindGameObjectsWithTag("Sensor");
            foreach (GameObject sensor in sensors)
            {
                Renderer sensorRenderer = sensor.GetComponent<Renderer>();
                if (sensorRenderer != null)
                {
                    sensorRenderer.material = CreateCustomizedMaterial(
                        materialConfig.sensorMaterial,
                        materialConfig.sensorColor,
                        materialConfig.sensorMetallic,
                        materialConfig.sensorSmoothness
                    );
                }
            }
        }
    }
    
    Material CreateCustomizedMaterial(Material baseMaterial, Color color, float metallic, float smoothness)
    {
        Material customMaterial = new Material(baseMaterial);
        customMaterial.color = color;
        
        // If using Standard shader, set metallic and smoothness
        if (baseMaterial.shader.name.Contains("Standard"))
        {
            customMaterial.SetFloat("_Metallic", metallic);
            customMaterial.SetFloat("_Smoothness", smoothness);
        }
        
        return customMaterial;
    }
}
```

## Importing Models from Gazebo

To maintain consistency between Gazebo physics and Unity visualization, you need to import models from Gazebo into Unity. This involves converting URDF models to formats Unity can understand.

### Conversion Process

1. **Export Gazebo models as standard 3D formats** (FBX, OBJ)
2. **Import into Unity with appropriate scaling**
3. **Apply materials that match the visual properties defined in URDF**

### Creating a Gazebo to Unity Model Pipeline

```csharp
// Example: Gazebo to Unity model pipeline
using UnityEngine;
using System.Collections;
using System.IO;

public class GazeboModelImporter : MonoBehaviour
{
    [Header("Model Conversion Settings")]
    public string gazeboModelPath = "C:/gazebo_models/";
    public string unityModelPath = "Assets/RobotModels/";
    public float scaleFactor = 1.0f; // Gazebo to Unity units conversion
    
    [Header("URDF Configuration")]
    public TextAsset urdfFile;
    public Material[] defaultMaterials;
    
    [System.Serializable]
    public class LinkMaterialMapping
    {
        public string linkName;
        public Material material;
    }
    
    public LinkMaterialMapping[] materialMappings;
    
    void Start()
    {
        if (urdfFile != null)
        {
            // Parse URDF and create Unity model
            StartCoroutine(ConvertURDFToUnity(urdfFile.text));
        }
    }
    
    IEnumerator ConvertURDFToUnity(string urdfContent)
    {
        // Simulate the parsing and conversion process
        yield return null;
        
        // Parse URDF content (in real implementation, you'd use a URDF parser)
        string[] lines = urdfContent.Split('\n');
        bool parsingLink = false;
        string currentLinkName = "";
        
        foreach (string line in lines)
        {
            if (line.Contains("<link name="))
            {
                parsingLink = true;
                int start = line.IndexOf('"') + 1;
                int end = line.IndexOf('"', start);
                currentLinkName = line.Substring(start, end - start);
            }
            else if (line.Contains("<visual>") && parsingLink)
            {
                // Process visual element for this link
                ProcessVisualElement(line, currentLinkName);
            }
            else if (line.Contains("</link>"))
            {
                parsingLink = false;
            }
            
            // Simulate processing time
            yield return new WaitForSeconds(0.01f);
        }
        
        Debug.Log("URDF to Unity model conversion complete");
    }
    
    void ProcessVisualElement(string line, string linkName)
    {
        // In a real implementation, this would extract geometry data
        // from the URDF visual element and create corresponding Unity objects
        
        // Create a primitive based on the geometry type found in URDF
        GameObject linkObject = CreateLinkVisual(linkName, "box", new Vector3(0.1f, 0.1f, 0.1f));
        
        // Apply appropriate material based on mapping
        ApplyMaterialToLink(linkObject, linkName);
    }
    
    GameObject CreateLinkVisual(string linkName, string geometryType, Vector3 dimensions)
    {
        GameObject linkObject = null;
        
        switch (geometryType.ToLower())
        {
            case "box":
                linkObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                linkObject.transform.localScale = dimensions * scaleFactor;
                break;
                
            case "cylinder":
                linkObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                // In Unity, cylinders are 1 unit tall and 1 unit in radius by default
                linkObject.transform.localScale = new Vector3(
                    dimensions.y * 2 * scaleFactor, // Diameter for width
                    dimensions.x * scaleFactor,     // Height for Unity cylinder
                    dimensions.y * 2 * scaleFactor  // Diameter for depth
                );
                // Rotate to align with URDF (Unity cylinder is vertical by default)
                linkObject.transform.Rotate(90, 0, 0);
                break;
                
            case "sphere":
                linkObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                linkObject.transform.localScale = Vector3.one * dimensions.x * 2 * scaleFactor;
                break;
                
            default:
                linkObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                linkObject.transform.localScale = dimensions * scaleFactor;
                break;
        }
        
        if (linkObject != null)
        {
            linkObject.name = linkName;
        }
        
        return linkObject;
    }
    
    void ApplyMaterialToLink(GameObject linkObject, string linkName)
    {
        Renderer linkRenderer = linkObject.GetComponent<Renderer>();
        if (linkRenderer == null)
        {
            linkRenderer = linkObject.AddComponent<MeshRenderer>();
        }
        
        // Find material mapping for this link
        Material assignedMaterial = null;
        foreach (LinkMaterialMapping mapping in materialMappings)
        {
            if (mapping.linkName == linkName)
            {
                assignedMaterial = mapping.material;
                break;
            }
        }
        
        if (assignedMaterial != null)
        {
            linkRenderer.material = assignedMaterial;
        }
        else
        {
            // Use default material if no specific mapping found
            linkRenderer.material = defaultMaterials.Length > 0 ? defaultMaterials[0] : new Material(Shader.Find("Standard"));
        }
    }
}
```

## Camera Systems

A well-designed camera system is essential for navigating and observing the digital twin. Unity provides various camera options and techniques.

### Common Camera Setups

- **Orbit cameras** for inspection
- **Fixed cameras** for monitoring
- **Dynamic cameras** that follow the robot
- **VR cameras** for immersive experiences

### Robotic Camera Controller

```csharp
// Example: Advanced camera system for robot monitoring
using UnityEngine;

public class RobotCameraController : MonoBehaviour
{
    [Header("Camera Configuration")]
    public Camera mainCamera;
    public Transform targetRobot;
    
    [Header("Camera Modes")]
    public CameraMode currentMode = CameraMode.Orbit;
    public enum CameraMode { Orbit, Follow, TopDown, Free, Fixed };
    
    [Header("Orbit Camera Settings")]
    public float orbitDistance = 5.0f;
    public float orbitHeight = 3.0f;
    public float orbitSensitivity = 2.0f;
    public float zoomSensitivity = 10.0f;
    public float minZoom = 1.0f;
    public float maxZoom = 20.0f;
    
    [Header("Follow Camera Settings")]
    public Vector3 followOffset = new Vector3(0, 2, -5);
    public float followSmoothness = 5.0f;
    
    [Header("Top-Down Camera Settings")]
    public float topDownHeight = 10.0f;
    
    // Internal state
    private float currentOrbitDistance;
    private Vector2 orbitRotation;
    private Vector3 velocity = Vector3.zero;
    private bool isInitialized = false;
    
    void Start()
    {
        InitializeCamera();
    }
    
    void InitializeCamera()
    {
        if (mainCamera == null)
            mainCamera = Camera.main;
            
        if (targetRobot == null)
            targetRobot = GameObject.FindGameObjectWithTag("Robot")?.transform;
            
        currentOrbitDistance = orbitDistance;
        orbitRotation = new Vector2(45, 30); // Default rotation
        isInitialized = true;
    }
    
    void LateUpdate()
    {
        if (!isInitialized || targetRobot == null)
            return;
            
        switch (currentMode)
        {
            case CameraMode.Orbit:
                HandleOrbitMode();
                break;
                
            case CameraMode.Follow:
                HandleFollowMode();
                break;
                
            case CameraMode.TopDown:
                HandleTopDownMode();
                break;
                
            case CameraMode.Free:
                HandleFreeMode();
                break;
                
            case CameraMode.Fixed:
                // No movement in fixed mode
                break;
        }
    }
    
    void HandleOrbitMode()
    {
        // Handle mouse input for rotation
        if (Input.GetMouseButton(1)) // Right mouse button
        {
            orbitRotation.y += Input.GetAxis("Mouse X") * orbitSensitivity;
            orbitRotation.x -= Input.GetAxis("Mouse Y") * orbitSensitivity;
            
            // Clamp vertical rotation
            orbitRotation.x = Mathf.Clamp(orbitRotation.x, -89, 89);
        }
        
        // Handle scroll for zoom
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        currentOrbitDistance -= scroll * zoomSensitivity;
        currentOrbitDistance = Mathf.Clamp(currentOrbitDistance, minZoom, maxZoom);
        
        // Calculate camera position
        float radX = orbitRotation.x * Mathf.PI / 180f;
        float radY = orbitRotation.y * Mathf.PI / 180f;
        
        float x = Mathf.Sin(radY) * Mathf.Cos(radX) * currentOrbitDistance;
        float y = Mathf.Sin(radX) * currentOrbitDistance;
        float z = Mathf.Cos(radY) * Mathf.Cos(radX) * currentOrbitDistance;
        
        Vector3 targetPosition = targetRobot.position - new Vector3(x, y, z);
        transform.position = targetPosition;
        
        // Look at target
        transform.LookAt(targetRobot.position);
    }
    
    void HandleFollowMode()
    {
        // Follow the robot with an offset
        Vector3 targetPos = targetRobot.position + targetRobot.TransformDirection(followOffset);
        transform.position = Vector3.SmoothDamp(transform.position, targetPos, ref velocity, 1.0f / followSmoothness);
        
        // Look slightly ahead of the robot
        Vector3 lookTarget = targetRobot.position + targetRobot.forward * 2.0f;
        transform.LookAt(lookTarget);
    }
    
    void HandleTopDownMode()
    {
        // Position camera directly above the target
        Vector3 targetPos = targetRobot.position;
        targetPos.y += topDownHeight;
        transform.position = targetPos;
        
        // Look straight down
        transform.rotation = Quaternion.Euler(90, 0, 0);
    }
    
    void HandleFreeMode()
    {
        // Free movement controls
        float moveSpeed = 10.0f;
        float fastMoveSpeed = 30.0f;
        
        float currentSpeed = Input.GetKey(KeyCode.LeftShift) ? fastMoveSpeed : moveSpeed;
        
        Vector3 movement = Vector3.zero;
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            movement += transform.forward;
        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            movement -= transform.forward;
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            movement -= transform.right;
        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            movement += transform.right;
        if (Input.GetKey(KeyCode.Q))
            movement -= transform.up;
        if (Input.GetKey(KeyCode.E))
            movement += transform.up;
            
        transform.position += movement.normalized * currentSpeed * Time.deltaTime;
        
        // Handle rotation
        if (Input.GetMouseButton(1))
        {
            float rotX = -Input.GetAxis("Mouse Y") * orbitSensitivity * 0.5f;
            float rotY = Input.GetAxis("Mouse X") * orbitSensitivity * 0.5f;
            
            transform.Rotate(rotX, rotY, 0, Space.Self);
        }
    }
    
    // Public methods to change camera mode
    public void SetCameraMode(CameraMode newMode)
    {
        currentMode = newMode;
        
        // Adjust camera properties based on new mode
        switch (newMode)
        {
            case CameraMode.Orbit:
                mainCamera.fieldOfView = 60f;
                break;
                
            case CameraMode.Follow:
                mainCamera.fieldOfView = 70f;
                break;
                
            case CameraMode.TopDown:
                mainCamera.fieldOfView = 60f;
                break;
                
            case CameraMode.Free:
                mainCamera.fieldOfView = 60f;
                break;
                
            case CameraMode.Fixed:
                mainCamera.fieldOfView = 60f;
                break;
        }
    }
    
    // Input handling for mode switching
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1)) SetCameraMode(CameraMode.Orbit);
        if (Input.GetKeyDown(KeyCode.Alpha2)) SetCameraMode(CameraMode.Follow);
        if (Input.GetKeyDown(KeyCode.Alpha3)) SetCameraMode(CameraMode.TopDown);
        if (Input.GetKeyDown(KeyCode.Alpha4)) SetCameraMode(CameraMode.Free);
        if (Input.GetKeyDown(KeyCode.Alpha5)) SetCameraMode(CameraMode.Fixed);
    }
}
```

## Human-Robot Interaction in Digital Twins

Creating effective interfaces for human-robot interaction is essential for digital twin applications, especially for monitoring, control, and debugging.

### UI Elements for Robot Control

```csharp
// Example: Robot control interface
using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class RobotControlInterface : MonoBehaviour
{
    [Header("Control Elements")]
    public Slider linearVelocitySlider;
    public Slider angularVelocitySlider;
    public Button moveForwardBtn;
    public Button moveBackwardBtn;
    public Button turnLeftBtn;
    public Button turnRightBtn;
    public Button emergencyStopBtn;
    
    [Header("Status Display")]
    public Text robotNameText;
    public Text positionText;
    public Text velocityText;
    public Text batteryLevelText;
    public Image batteryFillImage;
    
    [Header("ROS Connection")]
    public string robotName = "Robot1";
    public float maxLinearVelocity = 2.0f;
    public float maxAngularVelocity = 1.5f;
    
    // Robot state
    private Vector3 robotPosition;
    private Vector3 robotVelocity;
    private float batteryLevel = 100.0f;
    private bool emergencyStopActive = false;
    
    void Start()
    {
        SetupUI();
        StartCoroutine(UpdateRobotStatus());
    }
    
    void SetupUI()
    {
        // Set up control callbacks
        if (linearVelocitySlider != null)
            linearVelocitySlider.onValueChanged.AddListener(OnLinearVelocityChanged);
        
        if (angularVelocitySlider != null)
            angularVelocitySlider.onValueChanged.AddListener(OnAngularVelocityChanged);
        
        if (moveForwardBtn != null)
            moveForwardBtn.onClick.AddListener(() => SendVelocityCommand(maxLinearVelocity, 0));
        
        if (moveBackwardBtn != null)
            moveBackwardBtn.onClick.AddListener(() => SendVelocityCommand(-maxLinearVelocity, 0));
        
        if (turnLeftBtn != null)
            turnLeftBtn.onClick.AddListener(() => SendVelocityCommand(0, maxAngularVelocity));
        
        if (turnRightBtn != null)
            turnRightBtn.onClick.AddListener(() => SendVelocityCommand(0, -maxAngularVelocity));
        
        if (emergencyStopBtn != null)
            emergencyStopBtn.onClick.AddListener(ActivateEmergencyStop);
        
        // Initialize text displays
        if (robotNameText != null)
            robotNameText.text = robotName;
    }
    
    void OnLinearVelocityChanged(float value)
    {
        SendVelocityCommand(value * maxLinearVelocity, angularVelocitySlider.value * maxAngularVelocity);
    }
    
    void OnAngularVelocityChanged(float value)
    {
        SendVelocityCommand(linearVelocitySlider.value * maxLinearVelocity, value * maxAngularVelocity);
    }
    
    void SendVelocityCommand(float linear, float angular)
    {
        if (emergencyStopActive)
            return;
            
        // In a real implementation, this would send the command to the robot via ROS or another protocol
        Debug.Log($"Sending velocity command: Linear={linear:F2}, Angular={angular:F2}");
        
        // Update UI to show command was sent
        velocityText.text = $"Linear: {linear:F2} m/s, Angular: {angular:F2} rad/s";
    }
    
    void ActivateEmergencyStop()
    {
        emergencyStopActive = !emergencyStopActive;
        
        if (emergencyStopActive)
        {
            SendVelocityCommand(0, 0); // Stop the robot
            emergencyStopBtn.GetComponentInChildren<Text>().text = "EMERGENCY STOP (ACTIVE)";
            emergencyStopBtn.GetComponent<Image>().color = Color.red;
        }
        else
        {
            emergencyStopBtn.GetComponentInChildren<Text>().text = "EMERGENCY STOP";
            emergencyStopBtn.GetComponent<Image>().color = Color.red; // Keep red but inactive state
        }
    }
    
    IEnumerator UpdateRobotStatus()
    {
        while (true)
        {
            // In a real implementation, this would read actual robot status from ROS or similar
            UpdateRobotPosition(); // Simulated position update
            UpdateBatteryLevel();  // Simulated battery update
            
            yield return new WaitForSeconds(0.1f); // Update every 100ms
        }
    }
    
    void UpdateRobotPosition()
    {
        // This would come from actual robot pose
        // For simulation, we'll just create a simple movement pattern
        robotPosition += new Vector3(Mathf.Sin(Time.time) * 0.01f, 0, Mathf.Cos(Time.time) * 0.01f);
        
        if (positionText != null)
            positionText.text = $"Position: ({robotPosition.x:F2}, {robotPosition.y:F2}, {robotPosition.z:F2})";
    }
    
    void UpdateBatteryLevel()
    {
        // Simulate battery drain
        batteryLevel -= 0.01f;
        if (batteryLevel < 0) batteryLevel = 100.0f; // Wrap around for simulation
        
        if (batteryLevelText != null)
            batteryLevelText.text = $"Battery: {batteryLevel:F1}%";
            
        if (batteryFillImage != null)
            batteryFillImage.fillAmount = batteryLevel / 100.0f;
    }
    
    // Public method to update robot status from external source (e.g., ROS)
    public void UpdateRobotStatusExternal(Vector3 position, Vector3 velocity, float battery)
    {
        robotPosition = position;
        robotVelocity = velocity;
        batteryLevel = Mathf.Clamp01(battery);
        
        if (positionText != null)
            positionText.text = $"Position: ({position.x:F2}, {position.y:F2}, {position.z:F2})";
            
        if (velocityText != null)
            velocityText.text = $"Linear: {velocity.magnitude:F2} m/s";
            
        if (batteryLevelText != null)
            batteryLevelText.text = $"Battery: {batteryLevel:F1}%";
            
        if (batteryFillImage != null)
            batteryFillImage.fillAmount = batteryLevel;
    }
}
```

## Performance Optimization Tips

### For Unity Rendering

1. **Use Level of Detail (LOD)** systems for complex robot models
2. **Implement occlusion culling** for large environments
3. **Use efficient shaders** for real-time applications
4. **Optimize draw calls** by batching similar objects
5. **Use texture atlasing** to reduce texture swaps

## Hands-on Exercise 2: Creating a Visualization Scene

In this exercise, you'll create a Unity scene that visualizes a simple robot model with appropriate lighting and materials, and implement basic camera controls for the digital twin.

1. **Create a new Unity 3D project** named "DigitalTwinVisualization"
2. **Implement the DigitalTwinLighting** script to set up realistic lighting
3. **Create materials** for different robot components using the RobotMaterialManager
4. **Add the RobotCameraController** with multiple viewing modes
5. **Implement the RobotControlInterface** for basic interaction
6. **Test the scene** with different lighting conditions and camera angles

This high-fidelity rendering system, combined with Gazebo's physics simulation, creates a powerful digital twin that can be used for monitoring, debugging, and validating robotic systems.
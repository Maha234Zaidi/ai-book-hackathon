# Lighting Setup in Unity for Digital Twin Applications

This document provides detailed examples of lighting setup in Unity that matches the Gazebo environment to ensure visual consistency across both platforms in your digital twin application.

## Understanding Gazebo's Lighting System

Gazebo uses OGRE (Object-Oriented Graphics Rendering Engine) for rendering, which supports various lighting models. The key lighting parameters in Gazebo that should be matched in Unity include:

1. **Ambient Light**: The overall illumination level in the scene
2. **Directional Light**: Simulates sunlight or main light source direction
3. **Point Lights**: Localized light sources at specific positions
4. **Shadows**: Realistic shadow casting from objects

## Matching Gazebo Lighting Parameters in Unity

### 1. Ambient Light Configuration

In Gazebo, ambient light is defined in world files (SDF format). A typical Gazebo world file might define:

```xml
<world name="my_world">
  <ambient>0.4 0.4 0.4 1</ambient>  <!-- RGBA values -->
  <background>0.7 0.7 0.7 1</background>
</world>
```

In Unity, you can match this using:

```csharp
// Set ambient lighting to match Gazebo environment
RenderSettings.ambientLight = new Color(0.4f, 0.4f, 0.4f, 1.0f);
RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight; // For more nuanced ambient lighting

// You can also set different intensities for different ambient components
RenderSettings.ambientIntensity = 1.0f;
```

### 2. Directional Light Setup

Gazebo's directional light often simulates the sun. To match this in Unity:

```csharp
using UnityEngine;

public class GazeboLightMatching : MonoBehaviour
{
    [Header("Lighting Parameters")]
    public Color lightColor = new Color(1.0f, 0.95f, 0.9f, 1.0f);  // Slightly warm sunlight
    public float lightIntensity = 1.0f;
    public Vector3 lightDirection = new Vector3(-0.3f, -1.0f, -0.2f);  // Directional vector
    public float shadowStrength = 0.8f;

    void Start()
    {
        SetupDirectionalLight();
    }

    void SetupDirectionalLight()
    {
        // Create the main directional light
        GameObject mainLight = new GameObject("Main Directional Light");
        Light lightComponent = mainLight.AddComponent<Light>();
        
        // Configure the light properties
        lightComponent.type = LightType.Directional;
        lightComponent.color = lightColor;
        lightComponent.intensity = lightIntensity;
        lightComponent.shadows = LightShadows.Soft;
        lightComponent.shadowStrength = shadowStrength;
        
        // Set the rotation to match the light direction from Gazebo
        mainLight.transform.rotation = Quaternion.LookRotation(lightDirection);
    }
}
```

### 3. Advanced Lighting Configuration

For more complex scenarios, you might need multiple lights to match the Gazebo environment:

```csharp
using UnityEngine;

public class AdvancedGazeboLighting : MonoBehaviour
{
    [Header("Directional Light")]
    public Color directionalColor = Color.white;
    public float directionalIntensity = 1.0f;
    
    [Header("Point Lights")]
    public Color pointLightColor = Color.yellow;
    public float pointLightIntensity = 1.0f;
    public float pointLightRange = 10.0f;
    
    [Header("Environment")]
    public Color ambientSkyColor = new Color(0.2f, 0.2f, 0.2f, 1.0f);
    public Color ambientEquatorColor = new Color(0.2f, 0.2f, 0.2f, 1.0f);
    public Color ambientGroundColor = new Color(0.2f, 0.2f, 0.2f, 1.0f);

    void Start()
    {
        SetupEnvironmentLighting();
        SetupDirectionalLight();
        SetupPointLights();
    }

    void SetupEnvironmentLighting()
    {
        // Set ambient lighting modes
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = ambientSkyColor;
        RenderSettings.ambientEquatorColor = ambientEquatorColor;
        RenderSettings.ambientGroundColor = ambientGroundColor;
        RenderSettings.ambientIntensity = 1.0f;
    }

    void SetupDirectionalLight()
    {
        GameObject dirLight = new GameObject("Directional Light");
        Light light = dirLight.AddComponent<Light>();
        
        light.type = LightType.Directional;
        light.color = directionalColor;
        light.intensity = directionalIntensity;
        light.shadows = LightShadows.Soft;
        
        // Typical sun direction in Gazebo environments
        dirLight.transform.rotation = Quaternion.Euler(50, -120, 0);
    }

    void SetupPointLights()
    {
        // Create point lights at specific positions to match Gazebo setup
        // (typically these would be positioned based on your specific Gazebo world)
        CreatePointLight(new Vector3(5, 5, 0), pointLightColor, pointLightIntensity, pointLightRange);
        CreatePointLight(new Vector3(-5, 5, 0), pointLightColor, pointLightIntensity, pointLightRange);
    }

    GameObject CreatePointLight(Vector3 position, Color color, float intensity, float range)
    {
        GameObject lightGo = new GameObject("Point Light");
        lightGo.transform.position = position;
        
        Light light = lightGo.AddComponent<Light>();
        light.type = LightType.Point;
        light.color = color;
        light.intensity = intensity;
        light.range = range;
        light.shadows = LightShadows.Hard;
        
        return lightGo;
    }
}
```

## Practical Example: Matching a Standard Gazebo World

Here's a complete example that sets up Unity lighting to match a typical Gazebo world environment:

```csharp
using UnityEngine;

[ExecuteInEditMode]
public class GazeboWorldLighting : MonoBehaviour
{
    // This script should be attached to an empty GameObject in your Unity scene
    // It configures the lighting to match a standard Gazebo environment
    
    [Header("Gazebo World Parameters")]
    public Color gazeboAmbient = new Color(0.4f, 0.4f, 0.4f, 1.0f);
    public Color gazeboDirectional = Color.white;
    public float directionalIntensity = 0.8f;
    
    [Header("Light Positioning")]
    public Vector3 directionalEuler = new Vector3(50, -120, 0); // Rotation in Euler angles
    
    void Start()
    {
        ConfigureLighting();
    }

    void ConfigureLighting()
    {
        // Set ambient light to match Gazebo
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = gazeboAmbient;
        RenderSettings.ambientIntensity = 1.0f;
        
        // Create and configure directional light
        CreateDirectionalLight();
    }
    
    void CreateDirectionalLight()
    {
        // Check if we already have a directional light
        Light existingLight = FindObjectOfType<Light>();
        if (existingLight != null && existingLight.type == LightType.Directional)
        {
            // Update the existing light instead of creating a new one
            existingLight.color = gazeboDirectional;
            existingLight.intensity = directionalIntensity;
            existingLight.shadows = LightShadows.Soft;
            
            // Rotate based on the specified Euler angles
            existingLight.transform.rotation = Quaternion.Euler(directionalEuler);
            return;
        }
        
        // Create a new directional light
        GameObject lightObj = new GameObject("Gazebo Directional Light");
        Light light = lightObj.AddComponent<Light>();
        
        light.type = LightType.Directional;
        light.color = gazeboDirectional;
        light.intensity = directionalIntensity;
        light.shadows = LightShadows.Soft;
        
        // Rotate based on the specified Euler angles
        lightObj.transform.rotation = Quaternion.Euler(directionalEuler);
    }
}
```

## Verification Techniques

To verify that your Unity lighting matches the Gazebo environment:

1. **Visual Comparison**: Place similar objects in both environments and compare their appearance under the same lighting conditions.

2. **Color Matching**: Use color picking tools to ensure light color values are identical.

3. **Shadow Analysis**: Compare shadow directions and intensities between both environments.

4. **Reflection Analysis**: Check how reflective surfaces respond to lighting in both environments.

## Best Practices for Lighting Consistency

1. **Document Parameters**: Keep a record of all lighting parameters used in both Gazebo and Unity for future reference.

2. **Use Real-World Units**: When possible, use real-world lighting units (lux, candela) to ensure consistency across both systems.

3. **Calibration Process**: Establish a calibration process where you test and adjust lighting until the visual results match.

4. **Coordinate System Conversion**: Ensure that lighting directions account for coordinate system differences between Gazebo (X-forward) and Unity (Z-forward).

5. **Performance Considerations**: Balance visual accuracy with performance requirements, especially for real-time applications.

This setup ensures that your Unity visualization accurately reflects the lighting conditions of your Gazebo physics simulation, providing a consistent experience in your digital twin application.
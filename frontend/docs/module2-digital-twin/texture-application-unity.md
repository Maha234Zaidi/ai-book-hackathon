# Texture Application in Unity for Digital Twin Applications

This document provides detailed examples of texture application in Unity that maintains consistency with the visual properties defined in Gazebo through URDF files.

## Understanding Textures in Digital Twins

In a digital twin system, textures play a crucial role in ensuring visual consistency between the physics simulation in Gazebo and the rendering in Unity. Properly applied textures enhance the realism and recognition of objects, making the digital twin more effective for visualization and analysis.

## Texture Properties and Mapping

When applying textures in Unity for a digital twin, consider the following texture properties:

1. **Albedo/Diffuse Map**: The base color texture
2. **Normal Map**: Surface detail without additional geometry
3. **Metallic Map**: Defines how metallic different parts of the object appear
4. **Smoothness Map**: Controls how smooth or rough surfaces appear
5. **Occlusion Map**: Simulates ambient light occlusion
6. **Emission Map**: Defines self-illuminating areas

## Creating and Applying Textures

### Basic Texture Application

```csharp
using UnityEngine;

public class TextureApplication : MonoBehaviour
{
    public Texture2D albedoTexture;
    public Texture2D normalTexture;
    public float metallic = 0.0f;
    public float smoothness = 0.5f;
    
    void Start()
    {
        ApplyTextureToRenderer();
    }
    
    void ApplyTextureToRenderer()
    {
        // Get the renderer component
        Renderer renderer = GetComponent<Renderer>();
        
        if (renderer != null)
        {
            // Create a new material
            Material material = new Material(Shader.Find("Standard"));
            
            // Apply textures
            if (albedoTexture != null)
                material.mainTexture = albedoTexture;
                
            if (normalTexture != null)
                material.SetTexture("_BumpMap", normalTexture);
            
            // Set material properties
            material.SetFloat("_Metallic", metallic);
            material.SetFloat("_Smoothness", smoothness);
            
            // Apply the material to the renderer
            renderer.material = material;
        }
    }
}
```

### Advanced Texture Setup for URDF Consistency

```csharp
using UnityEngine;

public class URDFTextureSetup : MonoBehaviour
{
    [System.Serializable]
    public class TextureProperties
    {
        public string materialName;
        public Color albedoColor = Color.white;
        public Texture2D albedoTexture;
        public Texture2D normalTexture;
        public Texture2D metallicTexture;
        public float metallicValue = 0.0f;
        public float smoothnessValue = 0.5f;
        public float emissionIntensity = 0.0f;
    }
    
    public TextureProperties[] textureProperties;
    
    void Start()
    {
        ApplyURDFTextures();
    }
    
    void ApplyURDFTextures()
    {
        // Get all child renderers (for complex models with multiple materials)
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        
        foreach (Renderer renderer in renderers)
        {
            // Match renderer material name with defined properties
            foreach (TextureProperties properties in textureProperties)
            {
                if (renderer.name.ToLower().Contains(properties.materialName.ToLower()) ||
                    renderer.sharedMaterials.Length > 0 && 
                    renderer.sharedMaterials[0].name.ToLower().Contains(properties.materialName.ToLower()))
                {
                    ApplyTextureProperties(renderer, properties);
                    break; // Apply the first matching property
                }
            }
        }
    }
    
    void ApplyTextureProperties(Renderer renderer, TextureProperties properties)
    {
        // Create or get material
        Material material = renderer.sharedMaterial != null ? 
            new Material(renderer.sharedMaterial) : 
            new Material(Shader.Find("Standard"));
        
        // Apply albedo texture and color
        if (properties.albedoTexture != null)
        {
            material.mainTexture = properties.albedoTexture;
        }
        else
        {
            material.color = properties.albedoColor;
        }
        
        // Apply normal map if available
        if (properties.normalTexture != null)
        {
            material.SetTexture("_BumpMap", properties.normalTexture);
            material.EnableKeyword("_NORMALMAP");
        }
        
        // Apply metallic properties
        if (properties.metallicTexture != null)
        {
            material.SetTexture("_MetallicGlossMap", properties.metallicTexture);
        }
        else
        {
            material.SetFloat("_Metallic", properties.metallicValue);
        }
        
        // Apply smoothness
        material.SetFloat("_Smoothness", properties.smoothnessValue);
        
        // Apply emission if needed
        if (properties.emissionIntensity > 0)
        {
            material.EnableKeyword("_EMISSION");
            material.SetColor("_EmissionColor", properties.albedoColor * properties.emissionIntensity);
        }
        
        // Apply the material to the renderer
        renderer.material = material;
    }
}
```

## Texture Coordinate Mapping

When importing models from Gazebo, ensure proper texture coordinate mapping:

```csharp
using UnityEngine;

public class TextureCoordinateMapper : MonoBehaviour
{
    // Applies texture coordinate transformations to match between Gazebo and Unity
    public void MapTextureCoordinates()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            Mesh mesh = meshFilter.mesh;
            Vector2[] uvs = mesh.uv;
            
            // Example: Flip UV coordinates if needed based on import settings
            for (int i = 0; i < uvs.Length; i++)
            {
                // Unity and Gazebo might use different UV coordinate systems
                // This is a simplified example - actual mapping might be more complex
                uvs[i] = new Vector2(uvs[i].x, 1.0f - uvs[i].y);
            }
            
            mesh.uv = uvs;
        }
    }
}
```

## Creating Procedural Textures

Sometimes, you might need to generate textures procedurally for consistency:

```csharp
using UnityEngine;

public class ProceduralTextureGenerator : MonoBehaviour
{
    public int textureWidth = 64;
    public int textureHeight = 64;
    public Color baseColor = Color.gray;
    public float metallicValue = 0.0f;
    public float smoothnessValue = 0.5f;
    
    public Texture2D CreateProceduralTexture()
    {
        // Create a new texture
        Texture2D texture = new Texture2D(textureWidth, textureHeight);
        
        // Fill the texture with the base color
        for (int y = 0; y < textureHeight; y++)
        {
            for (int x = 0; x < textureWidth; x++)
            {
                texture.SetPixel(x, y, baseColor);
            }
        }
        
        texture.Apply();
        
        return texture;
    }
    
    public Material CreateProceduralMaterial()
    {
        // Create a new material with the procedural texture
        Material material = new Material(Shader.Find("Standard"));
        
        material.mainTexture = CreateProceduralTexture();
        material.SetColor("_Color", baseColor);
        material.SetFloat("_Metallic", metallicValue);
        material.SetFloat("_Smoothness", smoothnessValue);
        
        return material;
    }
}
```

## Best Practices for Texture Application

1. **Consistency with URDF**: Ensure textures match the visual properties defined in URDF files
2. **Efficient Textures**: Use appropriate texture sizes based on the application's performance requirements
3. **LOD Considerations**: Consider using different texture resolutions based on the distance from the camera
4. **Atlas Textures**: For objects with multiple materials, consider using texture atlases to improve performance
5. **Texture Compression**: Choose appropriate compression formats based on the target platform
6. **UV Mapping**: Verify proper UV mapping for all imported models

## Texture Optimization for Performance

```csharp
using UnityEngine;

public class TextureOptimization : MonoBehaviour
{
    public void OptimizeTextures()
    {
        Renderer renderer = GetComponent<Renderer>();
        if (renderer != null)
        {
            foreach (Material material in renderer.sharedMaterials)
            {
                // Reduce texture size based on distance for LOD systems
                if (material.mainTexture is Texture2D tex)
                {
                    // This is a placeholder - actual optimization might involve
                    // implementing LOD systems or texture streaming
                    Texture2D optimizedTexture = ReduceTextureSize(tex, 0.5f);
                    material.mainTexture = optimizedTexture;
                }
            }
        }
    }
    
    Texture2D ReduceTextureSize(Texture2D originalTexture, float scale)
    {
        // This is a simplified example - in practice, you might use more sophisticated
        // texture resizing techniques or Unity's built-in texture streaming
        int newWidth = Mathf.Max(1, Mathf.RoundToInt(originalTexture.width * scale));
        int newHeight = Mathf.Max(1, Mathf.RoundToInt(originalTexture.height * scale));
        
        RenderTexture rt = new RenderTexture(newWidth, newHeight, 24);
        RenderTexture.active = rt;
        
        Graphics.Blit(originalTexture, rt);
        
        Texture2D result = new Texture2D(newWidth, newHeight);
        result.ReadPixels(new Rect(0, 0, newWidth, newHeight), 0, 0);
        result.Apply();
        
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(rt);
        
        return result;
    }
}
```

## Loading Textures at Runtime

For dynamic texture loading to match Gazebo materials:

```csharp
using UnityEngine;
using System.Collections;

public class RuntimeTextureLoader : MonoBehaviour
{
    public string texturePath;  // Path to texture files
    
    // Coroutine to load texture asynchronously
    public IEnumerator LoadTextureAsync(string path)
    {
        // Using UnityWebRequest for newer Unity versions
        using (var request = UnityEngine.Networking.UnityWebRequestTexture.GetTexture(path))
        {
            yield return request.SendWebRequest();
            
            if (request.result == UnityEngine.Networking.UnityWebRequest.Result.Success)
            {
                Texture2D texture = ((UnityEngine.Networking.DownloadHandlerTexture)request.downloadHandler).texture;
                
                // Apply the loaded texture
                Renderer renderer = GetComponent<Renderer>();
                if (renderer != null)
                {
                    Material material = renderer.material;
                    material.mainTexture = texture;
                }
            }
            else
            {
                Debug.LogError($"Failed to load texture from {path}: {request.error}");
            }
        }
    }
}
```

These examples provide a foundation for applying textures in Unity that maintain consistency with visual properties defined in Gazebo URDF models, ensuring visual fidelity across your digital twin application.
# Importing Models from Gazebo to Unity for Consistent Visualization

This document details how to import models from Gazebo to Unity while maintaining visual and physical consistency between the two environments in your digital twin application.

## Overview of the Model Import Process

The process of importing models from Gazebo to Unity involves several steps:

1. Extracting and converting Gazebo models to standard 3D formats
2. Importing the model into Unity
3. Applying materials that match URDF visual properties
4. Configuring the model for Unity's physics and rendering systems
5. Validating consistency between Gazebo and Unity representations

## Understanding Gazebo Model Structure

Gazebo models are typically defined using the SDF (Simulation Description Format) and include:

- **Visual elements**: Define how the model looks in simulation
- **Collision elements**: Define how the model interacts physically
- **Inertial properties**: Define physical behavior
- **Material definitions**: Define visual appearance including colors and textures

The visual elements in Gazebo models often reference URDF (Unified Robot Description Format) files for robot models, which contain:

- **Geometry**: Shape and size specifications
- **Material**: Color and visual properties
- **Origin**: Position and orientation relative to parent links

## Extracting Models from Gazebo

### Method 1: Exporting from Gazebo GUI

1. Open Gazebo and load your world containing the model
2. Select the model you want to export
3. Right-click and select "Save As" to export the model
4. The model will be saved in SDF format with associated mesh files

### Method 2: Direct Access to Gazebo Model Directory

Gazebo models are typically stored in:
- Linux: `/usr/share/gazebo-<version>/models` or `~/.gazebo/models`
- The model structure looks like:

```
model_name/
├── model.config
├── model.sdf
└── meshes/
    ├── mesh1.dae
    ├── mesh2.stl
    └── ...
└── materials/
    ├── textures/
    │   └── texture1.png
    └── scripts/
        └── material.material
```

### Method 3: Converting from URDF to 3D Formats

Many Gazebo models originate from URDF files. You can extract model information directly from URDF:

```xml
<!-- Example URDF snippet -->
<link name="base_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot/meshes/base_link.dae"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot/meshes/base_link_collision.stl"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

## Converting Gazebo Models to Unity-Compatible Formats

### Supported 3D Formats in Unity

Unity supports several 3D model formats:
- **FBX** (recommended): Supports geometry, materials, textures, animations, and bones
- **DAE** (Collada): XML-based format that preserves most model properties
- **OBJ**: Basic geometry format (no materials or animations by default)
- **3DS**: Older format with limited features

### Converting Mesh Files

If your Gazebo model uses unsupported formats like STL, you'll need to convert them:

```bash
# Using MeshLab to convert STL to OBJ
meshlabserver -i input_model.stl -o output_model.obj

# Using Blender from command line to convert various formats to FBX
blender --background --python convert_script.py -- input_model.dae output_model.fbx
```

### Coordinate System Conversion

One of the most critical aspects is handling coordinate system differences:

- **Gazebo/ROS**: Right-handed coordinate system (X-forward, Y-left, Z-up)
- **Unity**: Left-handed coordinate system (X-right, Y-up, Z-forward)

Conversion formulas:
- Position: `(x, y, z)` in Gazebo → `(y, z, x)` in Unity
- Rotation: Convert Euler angles from Gazebo's coordinate frame to Unity's frame
- Scale: Both environments typically use meters as the unit of measurement

## Importing Models into Unity

### Step 1: Prepare Your Unity Project

1. Create a new Unity project or open an existing one
2. Create a folder structure in the Assets directory:
   ```
   Assets/
   ├── Models/
   ├── Materials/
   ├── Textures/
   └── ImportedGazeboModels/
   ```

### Step 2: Import the 3D Model

1. Place the converted model file in the `Assets/Models` folder
2. Unity will automatically import the model and generate:
   - Mesh files
   - Materials (if included in the format)
   - Animation files (if applicable)

### Step 3: Configure Model Import Settings

Select the imported model in the Project window and adjust import settings in the Inspector:

- **Scale Factor**: Set to 1 if both environments use meters (Gazebo and Unity default)
- **Mesh Compression**: Choose appropriate level (typically "Off" for precise models)
- **Read/Write Enabled**: Enable if you plan to modify the mesh at runtime
- **Import Materials**: Enable to import materials from the model file
- **Material Naming**: Use "By BaseTexture Name" for consistency

### Step 4: Apply Correct Materials

```csharp
using UnityEngine;

public class GazeboMaterialApplier : MonoBehaviour
{
    [System.Serializable]
    public class ColorMaterialMapping
    {
        public string gazeboColorName;
        public string gazeboRGBA;  // Format: "r g b a" (e.g., "0.0 1.0 0.0 1.0")
        public Material customMaterial;
    }

    public ColorMaterialMapping[] materialMappings;

    void Start()
    {
        ApplyMaterialsFromURDF();
    }

    void ApplyMaterialsFromURDF()
    {
        // Get all renderers in this object and its children
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        
        foreach (Renderer renderer in renderers)
        {
            // Apply material based on URDF color properties
            Material gazeboMaterial = FindMaterialByURDFColor(renderer);
            if (gazeboMaterial != null)
            {
                renderer.material = gazeboMaterial;
            }
        }
    }
    
    Material FindMaterialByURDFColor(Renderer renderer)
    {
        // This is a simplified example - in practice, you'd parse
        // material properties from URDF or SDF files
        foreach (ColorMaterialMapping mapping in materialMappings)
        {
            if (renderer.name.Contains(mapping.gazeboColorName) || 
                renderer.material.name.Contains(mapping.gazeboColorName))
            {
                return mapping.customMaterial;
            }
        }
        return renderer.material; // Return existing material if no match
    }
}
```

## Ensuring Consistency Between Environments

### Visual Consistency Checks

1. **Color Matching**:
   - Compare color values between Gazebo and Unity
   - Ensure lighting conditions match between environments
   - Verify that materials respond similarly to lighting

2. **Size Matching**:
   - Verify that models have the same dimensions in both environments
   - Check that scale factors are applied consistently

3. **Texture Alignment**:
   - Ensure textures map correctly to model surfaces
   - Verify that texture coordinates are properly applied

### Practical Example: Robot Model Import

Here's a complete workflow for importing a robot model:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class RobotModelImporter : MonoBehaviour
{
    [Header("Model Import Settings")]
    public string gazeboModelName = "my_robot";
    public float modelScaleFactor = 1.0f;  // Gazebo and Unity both use meters
    
    [Header("Material Properties")]
    public Material defaultMaterial;
    public Material wheelMaterial;
    public Material sensorMaterial;
    
    // Store references to different parts of the robot
    [Header("Robot Parts")]
    public Transform[] wheels;
    public Transform[] sensors;
    public Transform[] links;
    
    void Start()
    {
        // Apply transformations to match Gazebo coordinate system to Unity
        ApplyCoordinateSystemTransforms();
        
        // Apply appropriate materials based on URDF properties
        ApplyURDFMaterials();
        
        // Validate that all parts are properly positioned
        ValidateModelStructure();
    }
    
    void ApplyCoordinateSystemTransforms()
    {
        // Apply the transform to match coordinate systems
        // Gazebo: X-forward, Y-left, Z-up
        // Unity: X-right, Y-up, Z-forward
        
        // This is a simplified example - in practice, each joint and link
        // would need to be transformed according to coordinate system differences
        foreach (Transform child in transform)
        {
            // Apply appropriate rotation to match coordinate systems
            child.Rotate(0, 90, 0);  // Rotate to align Z-axis
        }
    }
    
    void ApplyURDFMaterials()
    {
        // Apply materials based on URDF specifications
        foreach (Transform wheel in wheels)
        {
            ApplyMaterialToTransform(wheel, wheelMaterial);
        }
        
        foreach (Transform sensor in sensors)
        {
            ApplyMaterialToTransform(sensor, sensorMaterial);
        }
    }
    
    void ApplyMaterialToTransform(Transform part, Material material)
    {
        Renderer renderer = part.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material = material;
        }
        
        // Handle multiple meshes per part
        Renderer[] childRenderers = part.GetComponentsInChildren<Renderer>();
        foreach (Renderer childRenderer in childRenderers)
        {
            if (childRenderer != renderer)  // Skip the parent renderer
            {
                childRenderer.material = material;
            }
        }
    }
    
    void ValidateModelStructure()
    {
        // Log warnings if important parts are missing
        if (wheels.Length == 0)
        {
            Debug.LogWarning("No wheels found in robot model. Check import settings.");
        }
        
        if (sensors.Length == 0)
        {
            Debug.LogWarning("No sensors found in robot model. Check import settings.");
        }
        
        Debug.Log($"Model validation complete. Found {links.Length} links, " +
                  $"{wheels.Length} wheels, and {sensors.Length} sensors.");
    }
}
```

## Common Issues and Solutions

### Issue 1: Incorrect Scaling

**Problem**: Model appears too large or small in Unity compared to Gazebo.

**Solution**: 
- Verify that both environments are using the same unit system (meters)
- Check that the Unity import scale factor is set correctly
- Apply uniform scaling if needed: `gameObject.transform.localScale = Vector3.one * scaleValue;`

### Issue 2: Texture Misalignment

**Problem**: Textures appear stretched or incorrectly positioned.

**Solution**:
- Check UV coordinates in your 3D modeling software before importing
- Verify that the conversion process preserved UV coordinates
- Reimport the model with different settings if needed

### Issue 3: Coordinate System Mismatch

**Problem**: Model appears rotated or positioned incorrectly.

**Solution**:
- Apply appropriate rotations to match coordinate systems
- Use the conversion formulas: Gazebo (x,y,z) → Unity (y,z,x) for positions
- Consider using a conversion script during import

## Best Practices

1. **Standardize Workflow**: Create a consistent process for all model imports to ensure uniformity.

2. **Documentation**: Keep detailed records of conversion parameters for each model.

3. **Validation**: Always validate imported models against the original Gazebo versions.

4. **Modular Approach**: Import complex robots as separate components that can be assembled in Unity.

5. **Material Library**: Maintain a library of materials that match common Gazebo/URDF material definitions.

6. **Version Control**: Store both original Gazebo models and Unity-converted models in version control.

7. **Performance Considerations**: Optimize model complexity for Unity's real-time rendering requirements.

By following these guidelines, you can successfully import Gazebo models into Unity while maintaining visual and behavioral consistency for your digital twin application.
# Validation Exercise: Photorealistic Simulation Setup

This exercise validates that users can successfully create a basic simulated environment with synthetic data generation capabilities as outlined in the photorealistic-simulation section.

## Exercise: Basic Photorealistic Simulation with Synthetic Data Generation

**Estimated Completion Time**: 50 minutes

**Learning Objectives**:
- Set up a photorealistic simulation environment in Isaac Sim
- Configure sensor models to generate synthetic data
- Generate labeled datasets from the simulation
- Validate the synthetic data quality

**Prerequisites**:
- Completion of Introduction to Isaac section
- Isaac Sim properly installed and configured
- Basic understanding of ROS 2 concepts
- Access to Isaac Sim's photorealistic assets

**Environment Requirements**:
- NVIDIA GPU with CUDA support
- Isaac Sim 2023.1 or later
- ROS 2 Humble Hawksbill
- Isaac ROS packages for data processing

## Introduction

This validation exercise confirms your ability to create a photorealistic simulation environment with synthetic data generation capabilities. You'll configure a scene with realistic lighting, materials, and sensor models to generate synthetic datasets useful for training AI models.

## Step-by-Step Instructions

### Step 1: Launch Isaac Sim in Photorealistic Mode
1. Open a terminal and navigate to your Isaac Sim directory:
```bash
cd ~/isaac-sim
```

2. Launch Isaac Sim with Kit plugins for photorealistic rendering:
```bash
./isaac-sim.sh --enable-omni-kit-plugin-wizard
```

3. In the Kit Plugin Wizard, ensure the following extensions are enabled:
   - omni.replicator
   - omni.kit.viewport.rtx
   - Isaac ROS Bridge

**Expected Result**: Isaac Sim launches with photorealistic rendering capabilities enabled.

### Step 2: Create a Photorealistic Environment
1. In Isaac Sim, go to "Quick Access" > "Isaac Examples" > "Environments" > "3D Furniture"
2. This will load a photorealistic environment with detailed textures and lighting

3. Add some dynamic elements:
   - Add a moving platform using "Create" > "Cube" (set as dynamic)
   - Adjust the lighting by modifying the dome light settings

**Expected Result**: A complex, photorealistic environment is loaded with realistic lighting and materials.

### Step 3: Set Up a Camera with Synthetic Data Generation
1. In the Stage panel, right-click and select "Create" > "Camera"
2. Position the camera to have a good view of the environment

3. Add Isaac Sim's Replicator extension to the camera:
   - Right-click on the camera prim in the Stage panel
   - Select "Add" > "Omniverse Replicator" > "RGB Camera"
   - Add additional sensors:
     - Depth sensor
     - Semantic segmentation sensor
     - Instance segmentation sensor

**Expected Result**: The camera is configured with multiple synthetic data generation capabilities.

### Step 4: Configure Synthetic Data Generation
1. In Isaac Sim, open the "Replicator" window via "Window" > "Omniverse" > "Replicator"

2. Create a basic generation script:
   - Add a camera capture node
   - Add annotation nodes (RGB, depth, semantic segmentation, instance segmentation)
   - Set output path to save synthetic data

3. Configure domain randomization:
   - Randomize lighting conditions
   - Randomize material properties of objects
   - Randomize camera positions (optional for this exercise)

**Expected Result**: Synthetic data generation is configured with domain randomization capabilities.

### Step 5: Run the Synthetic Data Generation
1. In Isaac Sim, press "Play" to start the simulation

2. Execute the synthetic data generation script:
   - You can do this through the Replicator UI or with a Python script

3. Capture at least 100 frames of synthetic data (RGB, depth, semantic segmentation)

**Expected Result**: Multiple synthetic data frames are saved to the specified output directory.

### Step 6: Validate Synthetic Data Quality
1. Navigate to your output directory and examine the generated data:
```bash
ls -la ~/output/synthetic_data/
```

2. Verify that multiple data types are present:
   - RGB images
   - Depth maps
   - Semantic segmentation masks
   - Instance segmentation masks

3. Use image viewing tools to inspect the quality of synthetic data:
   - Verify RGB images look photorealistic
   - Check depth maps are properly scaled
   - Confirm segmentation masks align with RGB images

**Expected Result**: High-quality synthetic data with proper alignment between different sensor modalities.

## Validation Checkpoints

### Checkpoint 1: After launching Isaac Sim
- Expected output: Isaac Sim opens with photorealistic rendering enabled
- Troubleshooting: If photorealistic mode doesn't enable, verify GPU and driver compatibility

### Checkpoint 2: After setting up environment
- Expected output: Photorealistic environment with detailed textures and lighting
- Troubleshooting: If textures don't appear, check Isaac Sim asset downloads

### Checkpoint 3: After configuring synthetic data generation
- Expected output: Replicator configured to capture multiple sensor types
- Troubleshooting: If Replicator doesn't appear, ensure plugins are enabled

### Checkpoint 4: After data generation
- Expected output: Directory with at least 100 synthetic data frames of various types
- Troubleshooting: If no data is generated, verify camera and sensor configurations

### Checkpoint 5: After data validation
- Expected output: Synthetic data of photorealistic quality with proper alignment
- Troubleshooting: If quality is poor, adjust lighting or domain randomization settings

## Expected Outcome

By the end of this exercise, you should have successfully:
- Created a photorealistic simulation environment
- Configured sensors to generate synthetic data
- Generated a dataset of synthetic RGB, depth, and segmentation data
- Validated the quality of the synthetic data

This demonstrates capability to create photorealistic simulations with synthetic data generation capabilities as required by the photorealistic simulation section.

## Assessment Criteria

- **Success**: Complete all steps, generate high-quality synthetic data, validate data properly
- **Partial Success**: Complete most steps but experience issues with data quality or quantity
- **Requires Review**: Complete setup but unable to generate or validate synthetic data

## Troubleshooting Section

### Issue 1: Isaac Sim fails to launch in photorealistic mode
- **Description**: Isaac Sim either doesn't start or runs in non-photorealistic mode
- **Solution**: Verify GPU has RTX capabilities, update graphics drivers, check CUDA compatibility
- **Prevention**: Ensure system meets minimum requirements before installation

### Issue 2: Replicator extension not available
- **Description**: Cannot find Replicator in the window menu or stage operations
- **Solution**: Check extension manager for 'omni.replicator' and ensure it's enabled
- **Prevention**: Verify complete Isaac Sim installation with all extensions

### Issue 3: No synthetic data generated
- **Description**: Output directory exists but is empty or contains corrupted files
- **Solution**: Check camera configuration, verify sensor attachments, review Replicator settings
- **Prevention**: Test with minimal configuration before adding complexity

### Issue 4: Poor data quality
- **Description**: Generated data doesn't appear photorealistic or annotations don't align properly
- **Solution**: Adjust lighting, verify camera parameters, check annotation accuracy settings
- **Prevention**: Start with simple scene before moving to complex environments

## Extension Activities (Optional)

1. Implement domain randomization techniques to increase dataset diversity
2. Generate synthetic data for a specific robotics task (e.g., object detection)
3. Validate synthetic vs. real data similarity using feature comparison
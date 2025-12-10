# Exercise 2: Creating Simulation Environments

**Estimated Completion Time**: 60 minutes

**Learning Objectives**:
- Configure a photorealistic simulation environment in Isaac Sim
- Integrate multiple sensor types in the simulation
- Generate synthetic data with RTX rendering
- Validate the quality of generated data

**Prerequisites**:
- Completion of Introduction to Isaac and Isaac Sim basics sections
- Isaac Sim with RTX capabilities installed
- Basic understanding of 3D environments
- GPU with RTX capabilities for optimal rendering

**Environment Requirements**:
- NVIDIA RTX GPU with 8GB+ VRAM
- Isaac Sim 2023.1 or later
- Isaac Sim Omniverse Replicator extension enabled

## Introduction

This exercise walks you through creating a complete photorealistic simulation environment with multiple sensor types. You'll configure a scene with RTX rendering, attach sensors, and generate synthetic data that could be used for training AI perception models.

## Step-by-Step Instructions

### Step 1: Launch Isaac Sim with RTX Rendering
1. Open a terminal and navigate to your Isaac Sim directory:
```bash
cd ~/isaac-sim
```

2. Launch Isaac Sim with RTX rendering enabled:
```bash
./isaac-sim.sh
```

3. Verify RTX rendering is active:
   - Go to "Window" > "Render Settings"
   - Confirm "RTX" is selected as the renderer
   - Check that the viewport shows realistic lighting

**Expected Result**: Isaac Sim launches with RTX rendering active, showing realistic lighting and reflections.

### Step 2: Create a Photorealistic Environment
1. In Isaac Sim, go to "Quick Access" > "Isaac Examples" > "Environments" > "3D Furniture"
2. This loads a complex, photorealistic environment with detailed furniture and lighting

3. Customize the environment:
   - Add a few random objects from the "Content" browser
   - Adjust the dome light to change environmental lighting
   - Position a spotlight to create more complex lighting scenarios

**Expected Result**: A complex, photorealistic environment with realistic lighting and materials.

### Step 3: Create and Configure a Robot with Sensors
1. In the "Quick Access" panel, go to "Isaac Examples" > "Robot Systems" > "Carter"

2. Add the Carter robot to your scene:
   - The robot should appear in the furniture environment
   - Verify it has proper physics properties

3. Add sensors to the robot:
   - Right-click on the robot's base in the Stage panel
   - Select "Add" > "Sensors" > "RGB Camera"
   - Position the camera at the front of the robot pointing forward
   - Add a LIDAR sensor at the top of the robot
   - Add an IMU sensor inside the robot's base

**Expected Result**: A robot equipped with multiple sensor types in the photorealistic environment.

### Step 4: Configure Synthetic Data Generation
1. Open the "Replicator" window via "Window" > "Omniverse" > "Replicator"

2. Create a basic generation graph:
   - Right-click in the Replicator Graph panel
   - Select "Create Node" > "Annotators" > "RGB"
   - Connect it to your camera
   - Add nodes for "Depth", "Semantic Segmentation", and "Instance Segmentation"

3. Set up the output paths for generated data:
   - Right-click and select "Create Node" > "Script" > "Write"
   - Configure the output directory (e.g., `~/isaac-sim-output/synthetic_data`)

**Expected Result**: A Replicator graph configured to generate multiple types of synthetic data from your robot's sensors.

### Step 5: Configure Domain Randomization
1. In the Replicator Graph, add domain randomization nodes:
   - Right-click and select "Create Node" > "Randomizers" > "Material Randomizer"
   - Connect it to randomize materials on environment objects
   - Add a "Light Randomizer" to vary lighting conditions

2. Configure the randomization parameters:
   - Randomize material colors between realistic ranges
   - Vary lighting intensity and color temperature
   - Add random object placement within safe bounds

**Expected Result**: Domain randomization parameters set up to enhance dataset robustness.

### Step 6: Run Simulation and Generate Data
1. Before running, set up the capture path:
   - Create a trigger node: "Create Node" > "Triggers" > "On Frame"
   - Set it to capture 50 frames (or a smaller number for testing)

2. Press the "Play" button in Isaac Sim to start the simulation

3. Execute the data generation:
   - The Replicator graph will capture data based on your trigger settings
   - Monitor the output directory for generated files

**Expected Result**: Multiple synthetic data files saved to the output directory with various annotations.

### Step 7: Validate Generated Data
1. Navigate to your output directory:
```bash
ls -la ~/isaac-sim-output/synthetic_data/
```

2. Examine the generated files:
   - RGB image files
   - Corresponding depth maps
   - Semantic and instance segmentation masks
   - Verify the files are correctly formatted and not corrupted

3. Visually inspect a few samples:
   - Open an RGB image and its corresponding segmentation mask
   - Verify that annotations align correctly
   - Check that depth values are reasonable

**Expected Result**: High-quality synthetic data with properly aligned annotations.

## Validation Checkpoints

### Checkpoint 1: After launching Isaac Sim with RTX
- Expected output: Isaac Sim with RTX rendering active
- Troubleshooting: If rendering doesn't look photorealistic, verify GPU compatibility

### Checkpoint 2: After creating photorealistic environment
- Expected output: Complex environment with realistic lighting and materials
- Troubleshooting: If environment assets don't load, check Isaac Sim installation

### Checkpoint 3: After adding sensors to robot
- Expected output: Robot with properly configured sensor attachments
- Troubleshooting: If sensors aren't appearing, verify Isaac Sim sensor extensions

### Checkpoint 4: After configuring Replicator graph
- Expected output: Graph showing data generation pipeline
- Troubleshooting: If Replicator doesn't function, ensure Omniverse Replicator extension is enabled

### Checkpoint 5: After data generation
- Expected output: Directory with synthetic data files
- Troubleshooting: If no files are generated, verify camera and sensor configurations

### Checkpoint 6: After data validation
- Expected output: Validated synthetic data with proper annotations
- Troubleshooting: If annotations don't align, check sensor calibration in simulation

## Expected Outcome

By the end of this exercise, you should have successfully:
- Created a photorealistic simulation environment with complex assets
- Configured a robot with multiple sensor types
- Set up synthetic data generation pipeline with annotations
- Generated and validated synthetic data suitable for AI training

This demonstrates the capability to create photorealistic simulations with synthetic data generation capabilities as required for robotics AI development.

## Assessment Criteria

- **Success**: Complete all steps, generate high-quality synthetic data, validate data alignment properly
- **Partial Success**: Complete most steps but experience issues with data quality or generation
- **Requires Review**: Complete setup but unable to generate or validate synthetic data

## Troubleshooting Section

### Issue 1: RTX rendering not available or performing poorly
- **Description**: Rendering appears basic or performance is very slow
- **Solution**: Verify GPU is RTX capable, update drivers, check VRAM usage
- **Prevention**: Confirm system meets Isaac Sim RTX requirements before starting

### Issue 2: Replicator extension not available
- **Description**: Cannot find Replicator in Window menu or Graph doesn't function
- **Solution**: Enable 'omni.replicator' extension in Extension Manager
- **Prevention**: Verify complete Isaac Sim installation with all extensions

### Issue 3: Sensors not appearing in Replicator graph
- **Description**: Cannot connect sensors to annotator nodes in Replicator
- **Solution**: Verify sensors are properly added and named in Stage panel
- **Prevention**: Follow Isaac Sim sensor integration documentation

### Issue 4: Poor synthetic data quality
- **Description**: Generated images appear low quality or annotations don't align
- **Solution**: Check camera calibration, lighting setup, and sensor positioning
- **Prevention**: Validate individual components before complex generation

### Issue 5: Domain randomization not working as expected
- **Description**: Randomization doesn't affect scene as expected
- **Solution**: Check randomizer node connections and parameter ranges
- **Prevention**: Test randomization with simple parameters first

## Extension Activities (Optional)

1. Implement more complex domain randomization (weather, time of day)
2. Add additional sensor types (thermal camera, multispectral)
3. Generate data for a specific robotics task (e.g., object detection in warehouses)
4. Create multiple environments and compare synthetic data quality
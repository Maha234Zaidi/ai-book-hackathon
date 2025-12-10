# Code Examples: Synthetic Data Generation in Isaac Sim

This document provides practical code examples for synthetic data generation in Isaac Sim, demonstrating how to configure and execute data generation pipelines with various sensor types and annotations.

## 1. Basic Synthetic Data Generation Setup

This example demonstrates how to set up a basic synthetic data generation pipeline:

```python
# basic_synthetic_data_generation.py
import omni
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import create_viewport_camera
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.replicator.core as rep

# Initialize simulation app
config = {"headless": False}
simulation_app = SimulationApp(config)

# Set up the world
world = World(stage_units_in_meters=1.0)

# Add a ground plane
world.scene.add_default_ground_plane()

# Create a simple scene with objects
from omni.isaac.core.objects import DynamicCuboid
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        name="cube",
        position=[0.0, 0.0, 1.0],
        size=0.5,
        color=[0.8, 0.2, 0.2]
    )
)

# Create a camera for data capture
camera = create_viewport_camera("/World/Camera", [1.0, 1.0, 1.0])
camera.set_world_pose(position=[1.0, 1.0, 1.0], orientation=[0.35, -0.35, -0.15, 0.85])

# Define the synthetic data generation using Replicator
with rep.new_layer():
    # Define the camera capture target
    camera_trigger = rep.create.trigger(
        prim_path="/Render/Variations/CameraTrigger",
        frequency=1,
        count=10  # Generate 10 frames
    )
    
    # Create render product for the camera
    with rep.create.render_product(camera.prim_path, [640, 480]):
        # Create RGB annotator
        rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annotator.attach([camera])
        
        # Create depth annotator
        depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        depth_annotator.attach([camera])
        
        # Create semantic segmentation annotator
        semantic_annotator = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
        semantic_annotator.attach([camera])

# Execute the data generation
with rep.trigger.on_frame(num_frames=10):
    pass

def main():
    # Reset and play the world
    world.reset()
    world.play()
    
    # Run simulation to completion
    for i in range(150):  # Run for 150 steps
        world.step(render=True)
    
    # Close the simulation
    simulation_app.close()

if __name__ == "__main__":
    main()
```

## 2. Advanced Domain Randomization Example

This example shows how to implement domain randomization to improve dataset robustness:

```python
# advanced_domain_randomization.py
import omni.replicator.core as rep
from pxr import Gf
import numpy as np

def setup_scene_randomization():
    """Set up domain randomization for the scene"""
    
    # Randomize materials
    def randomize_materials():
        # Get all cube prims in the scene
        cubes = rep.get.prims_from_path("/World/Cube", is_path_regex=True)
        
        with cubes:
            # Randomize color
            rep.randomizer.affect(
                cubes.prim.GetProperty("xformOp:translate"),
                rep.distribution.uniform((-2, -2, 0), (2, 2, 2)),
                expected_tokens=2
            )
        
        # Get all materials in the scene
        materials = rep.get.materials()
        
        with materials:
            # Randomize diffuse color
            rep.randomizer.affect(
                materials.prim.GetProperty("inputs:diffuse_tint"),
                rep.distribution.uniform((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
            )
            # Randomize roughness
            rep.randomizer.affect(
                materials.prim.GetProperty("inputs:roughness"),
                rep.distribution.uniform(0.0, 1.0)
            )
    
    # Randomize lights
    def randomize_lights():
        lights = rep.get.prim_at_path("/World/light")
        
        with lights:
            # Randomize intensity
            rep.randomizer.affect(
                lights.prim.GetProperty("inputs:intensity"),
                rep.distribution.uniform(1000, 5000)
            )
            # Randomize color temperature
            rep.randomizer.affect(
                lights.prim.GetProperty("inputs:color"),
                rep.distribution.uniform((0.8, 0.8, 0.9), (1.0, 1.0, 1.0))
            )
    
    # Randomize camera position
    def randomize_camera():
        camera = rep.get.prim_at_path("/World/Camera")
        
        with camera:
            # Randomize position
            rep.randomizer.affect(
                camera.prim.GetProperty("xformOp:translate"),
                rep.distribution.uniform((0.5, 0.5, 0.5), (2.0, 2.0, 2.0))
            )
    
    # Execute all randomization functions
    randomize_materials()
    randomize_lights()
    randomize_camera()

# Complete replication graph with domain randomization
def create_advanced_replication_graph():
    """Create a replication graph with domain randomization"""
    
    with rep.new_layer():
        # Define the camera capture target
        camera_trigger = rep.create.trigger(
            prim_path="/Render/Variations/CameraTrigger",
            frequency=1,
            count=50  # Generate 50 frames with randomization
        )
        
        # Create render product for the camera
        with rep.create.render_product("/World/Camera", [640, 480]):
            # Create various annotators
            rep.AnnotatorRegistry.get_annotator("rgb").attach([rep.get.prim_at_path("/World/Camera")])
            rep.AnnotatorRegistry.get_annotator("distance_to_camera").attach([rep.get.prim_at_path("/World/Camera")])
            rep.AnnotatorRegistry.get_annotator("semantic_segmentation").attach([rep.get.prim_at_path("/World/Camera")])
            rep.AnnotatorRegistry.get_annotator("instance_segmentation").attach([rep.get.prim_at_path("/World/Camera")])
    
    # Apply randomization on each frame
    with rep.trigger.on_frame(num_frames=50):
        setup_scene_randomization()
```

## 3. Multi-Sensor Data Generation

This example demonstrates how to generate synthetic data from multiple sensor types simultaneously:

```python
# multi_sensor_data_generation.py
import omni.replicator.core as rep
from omni.isaac.sensor import Camera
from omni.isaac.range_sensor import _range_sensor
from pxr import Gf

def create_multi_sensor_setup():
    """Set up multiple sensor types for synthetic data generation"""
    
    # Create RGB camera
    camera = Camera(
        prim_path="/World/RGB_Camera",
        position=[0.2, 0.0, 0.5],
        orientation=[0.0, 0.0, 0.0, 1.0]
    )
    camera.config_camera(
        resolution=(640, 480),
        focal_length=24.0,
        horizontal_aperture=20.956,
        clipping_range=(0.1, 100.0)
    )
    
    # Create depth camera
    depth_camera = Camera(
        prim_path="/World/Depth_Camera",
        position=[0.2, 0.0, 0.5],  # Same position as RGB
        orientation=[0.0, 0.0, 0.0, 1.0]
    )
    depth_camera.config_camera(
        resolution=(640, 480),
        focal_length=24.0,
        horizontal_aperture=20.956,
        clipping_range=(0.1, 100.0)
    )
    
    # Create LIDAR sensor
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
    lidar_interface.create_lidar(
        prim_path="/World/Lidar",
        translation=Gf.Vec3d(0.2, 0.0, 0.7),  # Slightly higher position
        orientation=Gf.Quatf(0.0, 0.0, 0.0, 1.0),
        config="Example_Rotary",
        translation_step=0.001,
        orientation_step=0.1
    )
    
    return camera, depth_camera, lidar_interface

def create_multi_sensor_replication_graph():
    """Create replication graph for multi-sensor data generation"""
    
    with rep.new_layer():
        # Create trigger for synchronized capture
        trigger = rep.create.trigger(
            prim_path="/Render/Variations/SensorTrigger",
            frequency=1,
            count=25  # Generate 25 synchronized frame sets
        )
        
        # RGB camera render product
        with rep.create.render_product("/World/RGB_Camera", [640, 480]):
            rep.AnnotatorRegistry.get_annotator("rgb").attach([rep.get.prim_at_path("/World/RGB_Camera")])
            rep.AnnotatorRegistry.get_annotator("distance_to_camera").attach([rep.get.prim_at_path("/World/RGB_Camera")])
        
        # Depth camera render product
        with rep.create.render_product("/World/Depth_Camera", [640, 480]):
            rep.AnnotatorRegistry.get_annotator("distance_to_camera").attach([rep.get.prim_at_path("/World/Depth_Camera")])
        
        # LIDAR data (handled differently from RGB/depth)
        # For LIDAR, we'd typically use Isaac Sim's range sensor interface
        # This is a simplified representation

    # Execute the replication graph
    with rep.trigger.on_frame(num_frames=25):
        # Apply any needed randomization
        pass

def main():
    """Main function to run multi-sensor data generation"""
    # Initialize sensors
    camera, depth_camera, lidar_interface = create_multi_sensor_setup()
    
    # Set up replication graph
    create_multi_sensor_replication_graph()
    
    # In a real implementation, you would:
    # 1. Start the Isaac Sim application
    # 2. Run the simulation with synchronized sensor capture
    # 3. Process and save the multi-modal data
    
    print("Multi-sensor synthetic data generation setup complete")
    print("RGB and depth data will be saved to specified output directory")
    print("LIDAR data will be captured and processed separately")
```

## 4. Synthetic Data Processing and Validation

This example shows how to process and validate the generated synthetic data:

```python
# synthetic_data_processing.py
import cv2
import numpy as np
import os
from PIL import Image

def validate_synthetic_data(data_dir):
    """Validate synthetic data quality and annotations"""
    
    # Check if directory exists
    if not os.path.exists(data_dir):
        raise ValueError(f"Data directory {data_dir} does not exist")
    
    # Get list of all files
    all_files = os.listdir(data_dir)
    
    # Separate different data types
    rgb_files = [f for f in all_files if f.endswith('.jpg') or f.endswith('.png')]
    depth_files = [f for f in all_files if 'depth' in f]
    seg_files = [f for f in all_files if 'seg' in f or 'semantic' in f]
    
    print(f"Found {len(rgb_files)} RGB images")
    print(f"Found {len(depth_files)} depth images")
    print(f"Found {len(seg_files)} segmentation masks")
    
    # Validate alignment between different modalities
    for rgb_file in rgb_files:
        base_name = os.path.splitext(rgb_file)[0]
        
        # Check if corresponding depth and segmentation files exist
        depth_file = f"{base_name}_depth.exr"  # Common format for depth
        seg_file = f"{base_name}_seg.png"      # Common format for segmentation
        
        if depth_file in all_files:
            # Load and validate depth data
            depth_path = os.path.join(data_dir, depth_file)
            # For EXR files, we'd typically use OpenEXR library
            print(f"Valid depth file: {depth_file}")
        else:
            print(f"Missing depth file for: {rgb_file}")
        
        if seg_file in all_files:
            # Load and validate segmentation data
            seg_path = os.path.join(data_dir, seg_file)
            seg_image = Image.open(seg_path)
            print(f"Valid segmentation file: {seg_file}")
        else:
            print(f"Missing segmentation file for: {rgb_file}")

def process_synthetic_dataset(data_dir, output_dir):
    """Process and organize synthetic dataset"""
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Create subdirectories for different data types
    for subdir in ['rgb', 'depth', 'segmentation', 'annotations']:
        subdir_path = os.path.join(output_dir, subdir)
        if not os.path.exists(subdir_path):
            os.makedirs(subdir_path)
    
    # Process each file and move to appropriate subdirectory
    for filename in os.listdir(data_dir):
        filepath = os.path.join(data_dir, filename)
        
        if any(ext in filename.lower() for ext in ['.jpg', '.png', '.jpeg']):
            # Determine if it's RGB, depth, or segmentation based on naming
            if 'depth' in filename.lower():
                dest = os.path.join(output_dir, 'depth', filename)
            elif 'seg' in filename.lower() or 'mask' in filename.lower():
                dest = os.path.join(output_dir, 'segmentation', filename)
            else:
                dest = os.path.join(output_dir, 'rgb', filename)
            
            print(f"Moving {filename} to {dest}")
            # In a real implementation, you would copy the file
            # os.rename(filepath, dest)

def calculate_data_statistics(data_dir):
    """Calculate statistics for synthetic dataset"""
    
    rgb_files = [f for f in os.listdir(data_dir) if f.endswith(('.jpg', '.png')) and 'depth' not in f.lower()]
    
    if not rgb_files:
        print("No RGB files found for statistics")
        return
    
    # Calculate basic statistics on a sample of images
    sample_size = min(10, len(rgb_files))  # Use up to 10 images for statistics
    sample_files = rgb_files[:sample_size]
    
    mean_brightness = []
    mean_contrast = []
    
    for filename in sample_files:
        filepath = os.path.join(data_dir, filename)
        # For this example, we'll just calculate brightness statistics
        # In practice, you might load the actual image data
        
        # Calculate mean brightness (simplified)
        # img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        # mean_brightness.append(np.mean(img))
        
        # Calculate contrast (simplified)
        # mean_contrast.append(np.std(img))
    
    print(f"Sample size: {sample_size}")
    print(f"Mean brightness range: {min(mean_brightness) if mean_brightness else 'N/A'} - {max(mean_brightness) if mean_brightness else 'N/A'}")
    print(f"Mean contrast range: {min(mean_contrast) if mean_contrast else 'N/A'} - {max(mean_contrast) if mean_contrast else 'N/A'}")

def main():
    """Main function to process synthetic data"""
    data_dir = "~/isaac-sim-output/synthetic_data"  # Update with actual path
    output_dir = "~/isaac-sim-output/processed_data"
    
    print("Validating synthetic data...")
    validate_synthetic_data(data_dir)
    
    print("\nProcessing synthetic dataset...")
    process_synthetic_dataset(data_dir, output_dir)
    
    print("\nCalculating dataset statistics...")
    calculate_data_statistics(data_dir)
    
    print("\nSynthetic data processing complete!")

if __name__ == "__main__":
    main()
```

## 5. Integration with Isaac ROS

This example demonstrates how to connect Isaac Sim synthetic data with Isaac ROS packages:

```python
# isaac_sim_ros_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np

class SyntheticDataBridge(Node):
    """Node to bridge synthetic data from Isaac Sim to ROS"""
    
    def __init__(self):
        super().__init__('synthetic_data_bridge')
        
        # Create publishers for different sensor modalities
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)
        
        # Create CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Timer to simulate data publishing (in real scenario, this would be driven by Isaac Sim)
        self.timer = self.create_timer(0.1, self.publish_data)  # 10Hz
        
        self.frame_counter = 0
        self.get_logger().info('Synthetic Data Bridge initialized')

    def publish_data(self):
        """Publish synthetic data to ROS topics"""
        
        # Simulate RGB image data generation
        # In a real implementation, this would receive data from Isaac Sim
        height, width = 640, 480
        
        # Create a simulated RGB image (replace with actual Isaac Sim data)
        rgb_image = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.header.frame_id = 'camera_rgb_optical_frame'
        
        # Create a simulated depth image (replace with actual Isaac Sim data)
        depth_image = np.random.uniform(0.1, 10.0, (height, width)).astype(np.float32)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        
        # Create camera info message
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        camera_info_msg.header.frame_id = 'camera_rgb_optical_frame'
        camera_info_msg.height = height
        camera_info_msg.width = width
        camera_info_msg.k = [240.0, 0.0, 320.0, 0.0, 240.0, 240.0, 0.0, 0.0, 1.0]  # Example intrinsics
        camera_info_msg.distortion_model = 'plumb_bob'
        camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        
        # Publish all messages
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(camera_info_msg)
        
        self.frame_counter += 1
        self.get_logger().info(f'Published frame {self.frame_counter}')

def main(args=None):
    rclpy.init(args=args)
    
    synthetic_data_bridge = SyntheticDataBridge()
    
    try:
        rclpy.spin(synthetic_data_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        synthetic_data_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

These examples demonstrate the key concepts of synthetic data generation in Isaac Sim, from basic setup to advanced domain randomization, multi-sensor integration, and ROS bridging. Each example includes comments explaining the purpose and can be adapted to specific use cases in robotics simulation and perception training.
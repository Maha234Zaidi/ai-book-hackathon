# Data Model: Digital Twin Module (Gazebo & Unity)

## Entities

### Digital Twin
- **Description**: A virtual representation of a physical system that simulates its behavior and characteristics
- **Properties**:
  - id: string (unique identifier)
  - name: string (name of the digital twin)
  - description: string (brief description of the twin)
  - physicalSystemType: string (type of physical system being represented)
  - creationDate: datetime (when the digital twin was created)
  - lastUpdated: datetime (when the twin was last updated)
  - status: enum (active, archived, underMaintenance)
- **Relationships**: 
  - Contains multiple PhysicsSimulation, RenderingConfiguration, SensorConfiguration
  - Referenced by User (for access control)

### Physics Simulation (Gazebo Component)
- **Description**: Configuration and state of the physics simulation environment
- **Properties**:
  - id: string (unique identifier)
  - twinId: string (reference to DigitalTwin)
  - gravity: Vector3 (gravitational force setting)
  - physicsEngine: string (type of physics engine used)
  - collisionDetection: boolean (whether collision detection is enabled)
  - simulationTimeStep: float (time step for simulation)
  - updateRate: float (updates per second)
  - frictionCoefficients: object (coefficients for different surfaces)
  - dampingParameters: object (damping settings)
- **Relationships**:
  - Belongs to DigitalTwin
  - Contains multiple URDFModel
  - Contains multiple SimulatedEnvironment

### Rendering Configuration (Unity Component)
- **Description**: Configuration for high-fidelity visualization in Unity
- **Properties**:
  - id: string (unique identifier)
  - twinId: string (reference to DigitalTwin)
  - lightingModel: string (type of lighting used)
  - textureQuality: enum (low, medium, high, ultra)
  - shadowQuality: enum (low, medium, high, ultra)
  - antiAliasing: string (anti-aliasing method)
  - renderResolution: string (resolution setting)
  - fpsTarget: integer (target frame rate)
- **Relationships**:
  - Belongs to DigitalTwin
  - Contains multiple UnityAsset
  - Contains multiple MaterialDefinition

### URDF Model
- **Description**: Unified Robot Description Format model used in Gazebo
- **Properties**:
  - id: string (unique identifier)
  - physicsSimId: string (reference to PhysicsSimulation)
  - name: string (name of the URDF model)
  - geometry: object (geometric properties)
  - kinematics: object (kinematic properties)
  - dynamics: object (dynamic properties)
  - visualProperties: object (visual representation)
  - collisionProperties: object (collision properties)
  - jointTypes: array (types of joints in the model)
  - materialProperties: object (material properties)
- **Relationships**:
  - Belongs to PhysicsSimulation
  - Referenced by SimulatedEnvironment

### Simulated Environment
- **Description**: The environment in which the physics simulation takes place
- **Properties**:
  - id: string (unique identifier)
  - physicsSimId: string (reference to PhysicsSimulation)
  - name: string (name of the environment)
  - dimensions: Vector3 (size of the environment)
  - lightingConditions: object (lighting in the environment)
  - obstacles: array (obstacles in the environment)
  - surfaces: array (surface types in the environment)
  - environmentalConditions: object (temperature, humidity, etc.)
- **Relationships**:
  - Belongs to PhysicsSimulation
  - Contains multiple URDFModel

### Sensor Configuration
- **Description**: Configuration for virtual sensors in the simulation
- **Properties**:
  - id: string (unique identifier)
  - twinId: string (reference to DigitalTwin)
  - sensorType: enum (lidar, depthCamera, imu, etc.)
  - name: string (name of the sensor)
  - position: Vector3 (position of the sensor)
  - orientation: Vector3 (orientation of the sensor)
  - noiseModel: object (configuration for sensor noise)
  - range: float (range of the sensor if applicable)
  - resolution: object (resolution settings for the sensor)
  - updateRate: integer (how frequently the sensor updates)
- **Relationships**:
  - Belongs to DigitalTwin
  - Belongs to PhysicsSimulation (for physics-based sensors)
  - Belongs to RenderingConfiguration (for visualization)

### Unity Asset
- **Description**: 3D assets and resources used in Unity visualization
- **Properties**:
  - id: string (unique identifier)
  - renderingConfigId: string (reference to RenderingConfiguration)
  - name: string (name of the asset)
  - assetType: enum (model, texture, animation, material, etc.)
  - fileReference: string (path to the asset file)
  - size: float (size of the asset file)
  - importSettings: object (settings used when importing the asset)
- **Relationships**:
  - Belongs to RenderingConfiguration
  - Referenced by MaterialDefinition

### Material Definition
- **Description**: Material properties for rendering in Unity
- **Properties**:
  - id: string (unique identifier)
  - renderingConfigId: string (reference to RenderingConfiguration)
  - name: string (name of the material)
  - shader: string (shader used for the material)
  - albedo: Color (base color of the material)
  - metallic: float (metallic property)
  - smoothness: float (smoothness property)
  - normalMap: string (reference to normal map texture)
  - emissionColor: Color (emission color if applicable)
- **Relationships**:
  - Belongs to RenderingConfiguration
  - Referenced by UnityAsset

## State Transitions

### Digital Twin States
- Draft → Active: When the digital twin is completed and ready for use
- Active → UnderMaintenance: When updates are being made to the twin
- UnderMaintenance → Active: When maintenance is completed
- Active → Archived: When the twin is no longer needed

### Simulation States
- Inactive → Running: When simulation is started
- Running → Paused: When simulation is paused
- Paused → Running: When simulation is resumed
- Running → Stopped: When simulation is stopped
# Data Model for Module 2: The Digital Twin (Gazebo & Unity)

## Key Entities

### DigitalTwin
- **ID**: Unique identifier for the digital twin
- **Name**: Name of the digital twin
- **Description**: Description of what the digital twin represents
- **Type**: Type of digital twin (e.g., "Humanoid Robot", "Environment", "Sensor")
- **CreatedDate**: Date when the digital twin was created
- **LastModified**: Date when the digital twin was last modified
- **Status**: Current status ("Draft", "Published", "Archived")

### GazeboSimulation
- **ID**: Unique identifier for the Gazebo simulation
- **DigitalTwinID**: Reference to the DigitalTwin entity
- **WorldFile**: Path to the world file used in the simulation
- **PhysicsEngine**: Physics engine used (e.g., "ODE", "Bullet")
- **Gravity**: Gravity settings (x, y, z components)
- **TimeStep**: Simulation time step
- **RealTimeFactor**: Real-time factor for the simulation

### UnityScene
- **ID**: Unique identifier for the Unity scene
- **DigitalTwinID**: Reference to the DigitalTwin entity
- **SceneFile**: Path to the Unity scene file
- **RenderingQuality**: Rendering quality setting ("Low", "Medium", "High", "Ultra")
- **LightingMode**: Lighting mode ("Baked", "Realtime", "Mixed")
- **TargetPlatform**: Target platform for the scene

### SensorSimulation
- **ID**: Unique identifier for the sensor simulation
- **DigitalTwinID**: Reference to the DigitalTwin entity
- **Type**: Type of sensor ("LiDAR", "DepthCamera", "IMU")
- **Configuration**: Configuration settings for the sensor
- **OutputFormat**: Format of the sensor output
- **Frequency**: Sampling frequency of the sensor
- **NoiseModel**: Noise model applied to the sensor data

### PhysicsProperty
- **ID**: Unique identifier for the physics property
- **EntityID**: Reference to the entity this property applies to (DigitalTwin, GazeboSimulation, etc.)
- **Mass**: Mass of the entity
- **Inertia**: Inertia tensor of the entity
- **Friction**: Friction coefficient
- **Restitution**: Restitution coefficient
- **Constraints**: Constraints applied to the entity

## Relationships

- DigitalTwin → GazeboSimulation (One-to-Many): A digital twin can have multiple Gazebo simulations
- DigitalTwin → UnityScene (One-to-Many): A digital twin can have multiple Unity scenes
- DigitalTwin → SensorSimulation (One-to-Many): A digital twin can have multiple sensor simulations
- GazeboSimulation → PhysicsProperty (One-to-Many): A Gazebo simulation can have multiple physics properties
- UnityScene → PhysicsProperty (One-to-Many): A Unity scene can have multiple physics properties
- SensorSimulation → PhysicsProperty (One-to-Many): A sensor simulation can have multiple physics properties

## Validation Rules

1. Every DigitalTwin must have at least one associated GazeboSimulation or UnityScene
2. Every GazeboSimulation must have a valid WorldFile path
3. Every UnityScene must have a valid SceneFile path
4. Every SensorSimulation must have a valid Type ("LiDAR", "DepthCamera", "IMU")
5. Every PhysicsProperty must have valid values for Mass, Inertia, Friction, and Restitution
6. Gravity values in GazeboSimulation must be within reasonable limits (-100 to 100 m/s²)
7. TimeStep in GazeboSimulation must be positive and less than 0.1 seconds
8. Frequency in SensorSimulation must be positive and less than 1000 Hz
9. NoiseModel in SensorSimulation must be a valid noise model for the sensor type

## State Transitions

- Draft → Published: When the digital twin is complete and ready for use
- Published → Archived: When the digital twin is no longer needed or has been replaced
- Archived → Draft: When the digital twin needs to be updated or revived

## Examples

```yaml
DigitalTwin:
  ID: "dt001"
  Name: "Humanoid Robot Digital Twin"
  Description: "Digital twin of a humanoid robot for testing and development"
  Type: "Humanoid Robot"
  CreatedDate: "2025-12-11"
  LastModified: "2025-12-11"
  Status: "Draft"

GazeboSimulation:
  ID: "gs001"
  DigitalTwinID: "dt001"
  WorldFile: "worlds/humanoid_robot.world"
  PhysicsEngine: "ODE"
  Gravity: "0 0 -9.8"
  TimeStep: 0.001
  RealTimeFactor: 1.0

UnityScene:
  ID: "us001"
  DigitalTwinID: "dt001"
  SceneFile: "Scenes/HumanoidRobot.unity"
  RenderingQuality: "High"
  LightingMode: "Mixed"
  TargetPlatform: "Windows"

SensorSimulation:
  ID: "ss001"
  DigitalTwinID: "dt001"
  Type: "LiDAR"
  Configuration: "range: 10m, resolution: 0.1deg"
  OutputFormat: "PointCloud"
  Frequency: 10
  NoiseModel: "Gaussian"

PhysicsProperty:
  ID: "pp001"
  EntityID: "dt001"
  Mass: 50.0
  Inertia: "1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0"
  Friction: 0.5
  Restitution: 0.3
  Constraints: "FixedJoint"
```
# Chapter 4: Unity Visualization for High-Fidelity Rendering and Human-Robot Interaction

## Introduction to Unity for Robotics

Unity is a powerful game engine that has emerged as a valuable tool for robotics visualization and simulation. While Gazebo focuses on physics-accurate simulation, Unity excels at high-fidelity rendering that closely mimics the real world in terms of lighting, materials, and visual appearance. For humanoid robot development, Unity provides:

- **Photorealistic rendering**: High-quality visuals with realistic lighting and materials
- **Advanced rendering features**: Global illumination, physically-based shading, post-processing
- **Virtual Reality support**: For immersive teleoperation and training
- **Human-Robot Interaction (HRI) studies**: Realistic visualization for HRI research
- **AI training environments**: Photorealistic environments for synthetic data generation

## Installing Unity for Robotics

### Setting up Unity Hub

Unity Hub is the recommended way to manage Unity installations:

1. Download Unity Hub from the [official Unity website](https://unity.com/download)
2. Install Unity Hub and create an account
3. Install a recent LTS version of Unity (e.g., 2022.3.14f1 LTS)
4. Install additional modules if needed (Android, iOS, etc.)

### Installing Unity Robotics Tools

Unity provides specific tools for robotics development:

1. **Unity Robotics Hub**: Centralized installation of robotics packages
2. **Unity Robotics Package**: Core functionality for robotics simulation
3. **ROS-TCP-Connector**: Connect Unity to ROS/ROS2
4. **Unity Computer Vision Package**: Tools for synthetic dataset generation

To add these to your Unity project:

1. In Unity's Package Manager (Window > Package Manager)
2. Select "My Registries" and add the Unity Robotics Registry
3. Install the Unity Robotics Package and related tools

## Unity-ROS/ROS2 Communication

### Setting Up ROS-TCP-Connector

The ROS-TCP-Connector package enables communication between Unity and ROS/ROS2 systems:

1. Install the ROS-TCP-Connector package in your Unity project
2. Add the ROSConnection GameObject to your scene
3. Configure the IP address and port for connection

```csharp
// Unity script to connect with ROS2
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    // Start with connection to ROS2
    void Start()
    {
        // Get the ROS connection static instance
        ROSConnection.instance = GetComponent<ROSConnection>();
        
        // Connect to ROS2 using TCP
        ROSConnection.instance.Initialize("127.0.0.1", 10000);
    }
    
    // Example: Send data to ROS2
    void SendDataToROS2()
    {
        // Create a message object
        var message = new Unity.Robotics.ROS_TCP_Connection.MessageSupport.std_msgs.StringMsg();
        message.data = "Hello from Unity!";
        
        // Send the message to a ROS2 topic
        ROSConnection.instance.Send<string>("unity_message", message.data);
    }
    
    // Example: Receive data from ROS2
    void OnMessageReceived(string messageData)
    {
        Debug.Log("Received from ROS2: " + messageData);
        // Process the message in Unity
    }
}
```

### ROS2 Message Support

Unity supports a wide range of ROS2 message types:

- Standard messages: String, Int32, Float64, Bool
- Geometry: Point, Pose, Twist, Transform
- Sensors: LaserScan, Image, Imu, JointState
- Custom messages: Can be generated from .msg files

## Creating a Humanoid Robot Model in Unity

### Importing Assets

Unity supports several ways to import robot models:

1. **URDF Importer**: Use the Unity Robotics URDF Importer package to directly import URDF files
2. **FBX/OBJ Import**: Import CAD models exported from SolidWorks, Blender, etc.
3. **Manual Construction**: Build models from primitive shapes

### Using the URDF Importer

The URDF Importer package makes it easy to bring ROS-compatible robot models into Unity:

```csharp
using Unity.Robotics.URDFImport;
using UnityEngine;

public class HumanoidRobotSetup : MonoBehaviour
{
    // Load and instantiate a robot from URDF
    public void LoadRobotFromURDF(string urdfPath)
    {
        // Load the URDF and create a robot GameObject
        GameObject robotGO = URDFRobotExtensions.CreateRobot(urdfPath);
        
        // Position the robot in the scene
        robotGO.transform.position = Vector3.zero;
        robotGO.transform.rotation = Quaternion.identity;
        
        // You can now add Unity-specific components to the robot
        AddRobotComponents(robotGO);
    }
    
    void AddRobotComponents(GameObject robot)
    {
        // Add Rigidbody components to robot links for physics simulation
        foreach(Transform child in robot.transform)
        {
            // Add Rigidbody if it's a physical link
            if (child.GetComponent<Rigidbody>() == null)
            {
                child.gameObject.AddComponent<Rigidbody>();
            }
            
            // Add colliders if needed
            if (child.GetComponent<Collider>() == null)
            {
                // Add appropriate collider based on visual geometry
                AddColliderBasedOnGeometry(child.gameObject);
            }
        }
    }
    
    void AddColliderBasedOnGeometry(GameObject link)
    {
        // This is a simplified example - real implementation would analyze
        // the visual geometry to add appropriate colliders
        link.AddComponent<BoxCollider>();
    }
}
```

### Robot Hierarchy and Joints

Unlike Gazebo, Unity handles joints differently. You might need to create custom joint controllers:

```csharp
using UnityEngine;

public class UnityJointController : MonoBehaviour
{
    [Header("Joint Configuration")]
    public ArticulationBody jointBody;  // Unity's equivalent to physics joints
    public float minAngle = -45f;
    public float maxAngle = 45f;
    public float maxForce = 1000f;
    
    [Header("ROS Communication")]
    public string jointName;  // Name to match ROS joint names
    public float jointPosition = 0f;
    
    void Start()
    {
        SetupJoint();
    }
    
    void SetupJoint()
    {
        if (jointBody == null)
        {
            Debug.LogError("No Articulation Body assigned to joint: " + jointName);
            return;
        }
        
        // Configure the joint limits
        ArticulationDrive drive = jointBody.xDrive;
        drive.lowerLimit = minAngle;
        drive.upperLimit = maxAngle;
        drive.forceLimit = maxForce;
        drive.damping = 10f;  // Add some damping to stabilize
        jointBody.xDrive = drive;
        
        // Lock other degrees of freedom
        jointBody.immovable = false;
        jointBody.linearLockX = ArticulationDofLock.Locked;
        jointBody.linearLockY = ArticulationDofLock.Locked;
        jointBody.linearLockZ = ArticulationDofLock.Locked;
        
        // For revolute joints, lock the twist and swing
        jointBody.twistLock = ArticulationDofLock.LimitedMotion;
        jointBody.swingYLock = ArticulationDofLock.Locked;
        jointBody.swingZLock = ArticulationDofLock.Locked;
    }
    
    public void SetJointPosition(float angle)
    {
        // Update the joint position
        jointPosition = Mathf.Clamp(angle, minAngle, maxAngle);
        
        // Apply the target position to the articulation body
        ArticulationDrive drive = jointBody.xDrive;
        drive.target = jointPosition;
        jointBody.xDrive = drive;
    }
    
    public float GetJointPosition()
    {
        return jointBody.jointPosition.x; // For x-axis rotation
    }
}
```

## High-Fidelity Rendering Techniques

### Physically-Based Rendering (PBR)

Unity's PBR pipeline provides realistic materials:

```csharp
using UnityEngine;

public class RobotMaterialManager : MonoBehaviour
{
    // Robot material properties
    [Header("Material Configuration")]
    public Material metalMaterial;
    public Material rubberMaterial;
    public Material screenMaterial;
    
    [Header("Visual Effects")]
    public Light robotHeadlight;
    public ParticleSystem indicatorParticles;
    
    void Start()
    {
        ApplyMaterialsToRobot();
        SetupVisualEffects();
    }
    
    void ApplyMaterialsToRobot()
    {
        // Find all renderers in the robot hierarchy
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        
        foreach(Renderer renderer in renderers)
        {
            // Assign appropriate materials based on part type
            string partName = renderer.gameObject.name.ToLower();
            
            if (partName.Contains("arm") || partName.Contains("leg"))
            {
                renderer.material = metalMaterial;
            }
            else if (partName.Contains("foot") || partName.Contains("hand"))
            {
                renderer.material = rubberMaterial;
            }
            else if (partName.Contains("sensor") || partName.Contains("camera"))
            {
                renderer.material = screenMaterial;
            }
            
            // Configure rendering properties
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
            renderer.receiveShadows = true;
        }
    }
    
    void SetupVisualEffects()
    {
        // Configure robot headlight
        if (robotHeadlight != null)
        {
            robotHeadlight.type = LightType.Spot;
            robotHeadlight.spotAngle = 60f;
            robotHeadlight.color = Color.blue;
            robotHeadlight.intensity = 2f;
        }
        
        // Configure indicator particles
        if (indicatorParticles != null)
        {
            var main = indicatorParticles.main;
            main.startColor = Color.green;
            main.startSize = 0.1f;
            main.maxParticles = 100;
        }
    }
}
```

### Advanced Lighting for Realism

Unity's lighting system can create highly realistic environments:

```csharp
using UnityEngine;

public class AdvancedLightingSetup : MonoBehaviour
{
    [Header("Lighting Configuration")]
    public Light mainDirectionalLight;
    public ReflectionProbe environmentReflection;
    public Skybox skyMaterial;
    
    [Header("Environment")]
    public GameObject[] environmentObjects;
    
    void Start()
    {
        SetupLighting();
        ConfigureEnvironment();
    }
    
    void SetupLighting()
    {
        // Configure main directional light (sun)
        if (mainDirectionalLight != null)
        {
            mainDirectionalLight.type = LightType.Directional;
            mainDirectionalLight.color = Color.white;
            mainDirectionalLight.intensity = 1.0f;
            
            // Set realistic sun angle
            mainDirectionalLight.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        }
        
        // Configure reflection probe for realistic reflections
        if (environmentReflection != null)
        {
            environmentReflection.mode = ReflectionProbeMode.Realtime;
            environmentReflection.refreshMode = ReflectionProbeRefreshMode.EveryFrame;
        }
        
        // Set up skybox
        RenderSettings.skybox = skyMaterial;
        RenderSettings.ambientLight = new Color(0.2f, 0.2f, 0.2f);
    }
    
    void ConfigureEnvironment()
    {
        // Add environmental objects for realism
        foreach(GameObject obj in environmentObjects)
        {
            if (obj != null)
            {
                // Add realistic materials to environment objects
                Renderer[] envRenderers = obj.GetComponentsInChildren<Renderer>();
                foreach(Renderer renderer in envRenderers)
                {
                    // Apply PBR materials suitable for the environment
                    AssignEnvironmentMaterial(renderer);
                }
            }
        }
    }
    
    void AssignEnvironmentMaterial(Renderer renderer)
    {
        // Assign appropriate materials based on object type
        string objName = renderer.gameObject.name.ToLower();
        
        if (objName.Contains("wall") || objName.Contains("floor"))
        {
            renderer.material = CreatePBRMaterial(Color.gray, 0.5f, 0.1f);
        }
        else if (objName.Contains("table") || objName.Contains("desk"))
        {
            renderer.material = CreatePBRMaterial(new Color(0.8f, 0.6f, 0.4f), 0.3f, 0.2f);
        }
        else
        {
            renderer.material = CreatePBRMaterial(Color.white, 0.2f, 0.05f);
        }
    }
    
    Material CreatePBRMaterial(Color albedo, float metallic, float smoothness)
    {
        Material mat = new Material(Shader.Find("Standard"));
        mat.SetColor("_Color", albedo);
        mat.SetFloat("_Metallic", metallic);
        mat.SetFloat("_Smoothness", smoothness);
        return mat;
    }
}
```

## Human-Robot Interaction (HRI) in Unity

### Creating Interactive Spaces

Unity is particularly well-suited for HRI research due to its ability to create immersive environments:

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;

public class HumanRobotInteractionController : MonoBehaviour
{
    [Header("Interaction Components")]
    public Camera hriCamera;  // Camera for human perspective
    public GameObject robotObject;
    public GameObject interactionUI;
    
    [Header("HRI Parameters")]
    public float interactionDistance = 3.0f;
    public LayerMask interactableLayers;
    
    [Header("ROS Communication")]
    public string robotCommandTopic = "/hri_commands";
    
    void Update()
    {
        HandleInteractions();
        UpdateInteractionUI();
    }
    
    void HandleInteractions()
    {
        // Raycast to detect what the user is looking at
        RaycastHit hit;
        Vector3 rayOrigin = hriCamera.transform.position;
        Vector3 rayDirection = hriCamera.transform.forward;
        
        if (Physics.Raycast(rayOrigin, rayDirection, out hit, interactionDistance, interactableLayers))
        {
            // Highlight the object being looked at
            HighlightObject(hit.collider.gameObject);
            
            // Check for interaction input
            if (Input.GetMouseButtonDown(0))
            {
                ProcessInteraction(hit.collider.gameObject);
            }
        }
    }
    
    void HighlightObject(GameObject obj)
    {
        // Add temporary highlight effect
        Renderer renderer = obj.GetComponent<Renderer>();
        if (renderer != null)
        {
            // Store original material to restore later
            if (!obj.CompareTag("Highlighted"))
            {
                obj.tag = "Highlighted";
                StartCoroutine(RemoveHighlight(obj, renderer));
            }
        }
    }
    
    System.Collections.IEnumerator RemoveHighlight(GameObject obj, Renderer renderer)
    {
        // Change material temporarily
        Color originalColor = renderer.material.color;
        renderer.material.color = Color.yellow;
        
        yield return new WaitForSeconds(0.5f);
        
        // Restore original color
        renderer.material.color = originalColor;
        obj.tag = "Untagged";
    }
    
    void ProcessInteraction(GameObject interactedObject)
    {
        // Determine what type of interaction to send to ROS
        string interactionType = DetermineInteractionType(interactedObject);
        
        // Send interaction command to ROS
        SendInteractionToROS(interactionType);
    }
    
    string DetermineInteractionType(GameObject obj)
    {
        string objName = obj.name.ToLower();
        
        if (objName.Contains("button") || objName.Contains("switch"))
        {
            return "BUTTON_PRESS";
        }
        else if (objName.Contains("door"))
        {
            return "DOOR_OPEN";
        }
        else if (objName.Contains("object"))
        {
            return "OBJECT_PICKUP";
        }
        else
        {
            return "GENERAL_INTERACTION";
        }
    }
    
    void SendInteractionToROS(string interactionType)
    {
        // Create and send ROS message
        var interactionMsg = new InteractionCommand();
        interactionMsg.type = interactionType;
        interactionMsg.timestamp = System.DateTime.Now.ToString();
        
        // Send through ROS connection
        ROSConnection.instance.Send<InteractionCommand>(robotCommandTopic, interactionMsg);
    }
    
    void UpdateInteractionUI()
    {
        // Update UI elements based on interaction state
        if (interactionUI != null)
        {
            // Show/hide UI based on proximity to robot
            float distToRobot = Vector3.Distance(transform.position, robotObject.transform.position);
            interactionUI.SetActive(distToRobot < interactionDistance);
        }
    }
}

[System.Serializable]
public class InteractionCommand
{
    public string type;
    public string timestamp;
    public string[] parameters; // Additional parameters for the interaction
}
```

### VR Integration

For immersive HRI experiences, Unity supports VR platforms:

```csharp
#if UNITY_EDITOR || UNITY_STANDALONE
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
#endif

public class VRHumanRobotInterface : MonoBehaviour
{
    [Header("VR Components")]
    public XRNode inputSource;  // LeftHand or RightHand controllers
    public InputDevice device;
    
    [Header("Robot Interaction")]
    public GameObject robot;
    public Transform vrSpace;
    
    void Start()
    {
        SetupVR();
    }
    
    void SetupVR()
    {
#if UNITY_EDITOR || UNITY_STANDALONG
        // Get the input device for the specified controller
        List<InputDevice> devices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(inputSource, devices);
        
        if (devices.Count > 0)
        {
            device = devices[0];
            Debug.Log("Connected device: " + device.name);
        }
#endif
    }
    
    void Update()
    {
        HandleVRInput();
    }
    
    void HandleVRInput()
    {
        if (device.isValid)
        {
            // Get controller rotation and position
            Vector3 position;
            Quaternion rotation;
            
            if (device.TryGetFeatureValue(CommonUsages.devicePosition, out position) &&
                device.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation))
            {
                // Update controller position/rotation in VR space
                transform.position = vrSpace.TransformPoint(position);
                transform.rotation = vrSpace.rotation * rotation;
            }
            
            // Handle button presses
            bool triggerPressed = false;
            if (device.TryGetFeatureValue(CommonUsages.triggerButton, out triggerPressed) && triggerPressed)
            {
                TriggerPressed();
            }
        }
    }
    
    void TriggerPressed()
    {
        // Raycast to find what's in front of the controller
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, 5.0f))
        {
            if (hit.collider.CompareTag("RobotPart"))
            {
                // Interact with robot part
                RobotPartInteraction(hit.collider.gameObject);
            }
        }
    }
    
    void RobotPartInteraction(GameObject robotPart)
    {
        Debug.Log("Interacting with: " + robotPart.name);
        // Send interaction command to robot via ROS
        SendInteractionToRobot(robotPart.name);
    }
    
    void SendInteractionToRobot(string partName)
    {
        // Send message to ROS
        var interactionMsg = new VRInteractionMessage();
        interactionMsg.part = partName;
        interactionMsg.hand = inputSource.ToString();
        
        ROSConnection.instance.Send<VRInteractionMessage>("/vr_interaction", interactionMsg);
    }
}

[System.Serializable]
public class VRInteractionMessage
{
    public string part;
    public string hand;
    public float pressure; // If applicable
}
```

## Sensor Simulation in Unity

### Camera Simulation

Unity can simulate camera sensors for computer vision applications:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROS_TCP_Connection.MessageSupport.sensor_msgs;

public class SimulatedCamera : MonoBehaviour
{
    [Header("Camera Configuration")]
    public Camera sensorCamera;
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float fov = 60f; // Field of view in degrees
    
    [Header("ROS Configuration")]
    public string imageTopic = "/camera/image_raw";
    public string cameraInfoTopic = "/camera/camera_info";
    
    [Header("Performance")]
    public float captureInterval = 0.1f; // seconds between captures
    private float lastCaptureTime;
    
    // Render texture for camera capture
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    
    void Start()
    {
        SetupCamera();
        SetupRenderTexture();
        lastCaptureTime = Time.time;
    }
    
    void SetupCamera()
    {
        if (sensorCamera == null)
            sensorCamera = GetComponent<Camera>();
            
        // Configure camera parameters to match real-world specs
        sensorCamera.fieldOfView = fov;
        sensorCamera.aspect = (float)imageWidth / imageHeight;
    }
    
    void SetupRenderTexture()
    {
        // Create render texture to capture camera output
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        
        // Create a temporary texture to read from the render texture
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        
        // Assign render texture to camera
        sensorCamera.targetTexture = renderTexture;
    }
    
    void Update()
    {
        // Capture image at specified interval
        if (Time.time - lastCaptureTime > captureInterval)
        {
            CaptureAndPublishImage();
            lastCaptureTime = Time.time;
        }
    }
    
    void CaptureAndPublishImage()
    {
        // Set the active render texture
        RenderTexture.active = renderTexture;
        
        // Read pixels from the render texture
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();
        
        // Convert to byte array for ROS message
        byte[] imageData = texture2D.EncodeToPNG();
        
        // Create ROS image message
        var imageMsg = new ImageMsg();
        imageMsg.header.stamp = new TimeStamp(Time.time);
        imageMsg.header.frame_id = transform.name; // Use transform name as frame ID
        imageMsg.height = (uint)imageHeight;
        imageMsg.width = (uint)imageWidth;
        imageMsg.encoding = "rgb8"; // Or "rgba8" depending on format
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(imageWidth * 3); // 3 bytes per pixel for RGB
        imageMsg.data = imageData;
        
        // Publish the image to ROS
        ROSConnection.instance.Send<ImageMsg>(imageTopic, imageMsg);
        
        // Also publish camera info
        PublishCameraInfo();
    }
    
    void PublishCameraInfo()
    {
        // Create and populate camera info message
        var cameraInfoMsg = new CameraInfoMsg();
        cameraInfoMsg.header.stamp = new TimeStamp(Time.time);
        cameraInfoMsg.header.frame_id = transform.name;
        cameraInfoMsg.width = (uint)imageWidth;
        cameraInfoMsg.height = (uint)imageHeight;
        
        // Calculate intrinsic camera parameters
        float fx = (imageWidth / 2.0f) / Mathf.Tan(Mathf.Deg2Rad * fov / 2.0f);
        float fy = (imageHeight / 2.0f) / Mathf.Tan(Mathf.Deg2Rad * fov / 2.0f);
        float cx = imageWidth / 2.0f;
        float cy = imageHeight / 2.0f;
        
        // Intrinsic camera matrix (3x3)
        cameraInfoMsg.K = new double[] { 
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1
        };
        
        // Distortion coefficients (assume none for simplicity)
        cameraInfoMsg.D = new double[] { 0, 0, 0, 0, 0 };
        
        // Publish camera info
        ROSConnection.instance.Send<CameraInfoMsg>(cameraInfoTopic, cameraInfoMsg);
    }
}
```

## Integration with Real Robotics Workflows

### Unity as a Visualization Layer

Often, Unity is used as a visualization layer on top of Gazebo simulations:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROS_TCP_Connection.MessageSupport;

public class UnityVisualizationLayer : MonoBehaviour
{
    [Header("Connection Settings")]
    public string rosMasterUri = "http://localhost:11311";
    public string robotStateTopic = "/joint_states";
    
    [Header("Robot Configuration")]
    public GameObject robotModel;
    public string[] jointNames;  // Names must match ROS joint names
    public Transform[] jointTransforms;  // Corresponding transforms in Unity model
    
    private JointStateMsg lastJointStates;
    
    void Start()
    {
        // Subscribe to joint states from ROS
        ROSConnection.instance.Subscribe<JointStateMsg>(robotStateTopic, JointStateCallback);
    }
    
    void JointStateCallback(JointStateMsg jointStates)
    {
        lastJointStates = jointStates;
        UpdateRobotVisualization();
    }
    
    void UpdateRobotVisualization()
    {
        if (lastJointStates == null || jointNames.Length != jointTransforms.Length)
            return;
            
        // Update each joint based on ROS joint states
        for (int i = 0; i < jointNames.Length; i++)
        {
            string jointName = jointNames[i];
            Transform jointTransform = jointTransforms[i];
            
            // Find the corresponding joint state from ROS
            int rosJointIndex = System.Array.IndexOf(lastJointStates.name, jointName);
            
            if (rosJointIndex >= 0 && rosJointIndex < lastJointStates.position.Length)
            {
                // Get the joint position (in radians for revolute joints)
                float jointPosition = (float)lastJointStates.position[rosJointIndex];
                
                // Apply the rotation to the Unity joint transform
                UpdateJointTransform(jointTransform, jointPosition);
            }
        }
    }
    
    void UpdateJointTransform(Transform jointTransform, float jointAngle)
    {
        // Apply rotation based on joint type
        // For revolute joints, rotate around the joint axis (often Z-axis)
        jointTransform.localRotation = Quaternion.Euler(0, 0, Mathf.Rad2Deg * jointAngle);
    }
    
    // Helper method to get joint position by name
    public float GetJointPosition(string jointName)
    {
        if (lastJointStates != null)
        {
            int index = System.Array.IndexOf(lastJointStates.name, jointName);
            if (index >= 0 && index < lastJointStates.position.Length)
            {
                return (float)lastJointStates.position[index];
            }
        }
        return 0f; // Return 0 if not found
    }
    
    // Helper method to get all joint positions
    public System.Collections.Generic.Dictionary<string, float> GetAllJointPositions()
    {
        var positions = new System.Collections.Generic.Dictionary<string, float>();
        
        if (lastJointStates != null)
        {
            for (int i = 0; i < lastJointStates.name.Length; i++)
            {
                if (i < lastJointStates.position.Length)
                {
                    positions.Add(lastJointStates.name[i], (float)lastJointStates.position[i]);
                }
            }
        }
        
        return positions;
    }
}
```

## Performance Optimization

When creating high-fidelity Unity environments for robotics, performance optimization is crucial:

```csharp
using UnityEngine;

public class PerformanceOptimizer : MonoBehaviour
{
    [Header("LOD Settings")]
    public int lodCount = 3;
    public float[] lodDistances = { 10f, 30f, 60f };
    
    [Header("Occlusion Culling")]
    public bool useOcclusionCulling = true;
    
    [Header("Quality Settings")]
    public int targetFrameRate = 60;
    public int maxTextureSize = 2048;
    public bool useAnisotropicFiltering = true;
    
    void Start()
    {
        OptimizeForRobotics();
    }
    
    void OptimizeForRobotics()
    {
        // Set frame rate
        Application.targetFrameRate = targetFrameRate;
        
        // Optimize graphics quality
        QualitySettings.vSyncCount = 0; // Disable V-Sync for consistent timing
        
        // Optimize texture streaming
        QualitySettings.streamingMipmapsActive = true;
        QualitySettings.streamingMipmapsMaxLevelReduction = 2;
        QualitySettings.streamingMipmapsMemoryBudget = 512.0f; // MB
        
        // Configure occlusion culling
        if (useOcclusionCulling)
        {
            StaticOcclusionCulling.Compute();
        }
        
        // Optimize physics for robotics simulation
        Physics.defaultSolverIterations = 6; // Reduce solver iterations
        Physics.defaultSolverVelocityIterations = 1; // Reduce velocity iterations
        
        // Enable batching
        Graphics.activeTier = GraphicsTier.Tier1; // Target mid-range hardware
    }
    
    void Update()
    {
        // Monitor performance and adjust dynamically if needed
        MonitorPerformance();
    }
    
    void MonitorPerformance()
    {
        float frameTime = Time.deltaTime;
        float fps = 1.0f / frameTime;
        
        // Log performance metrics
        if (Time.frameCount % 100 == 0) // Every 100 frames
        {
            Debug.Log($"Current FPS: {fps:F2}, Frame Time: {frameTime * 1000:F1} ms");
        }
        
        // Adaptively adjust quality based on performance
        if (fps < targetFrameRate * 0.8f)
        {
            // Performance too low, consider reducing quality
            ReduceQualityForPerformance();
        }
    }
    
    void ReduceQualityForPerformance()
    {
        // Temporarily reduce rendering quality if needed
        // This could include disabling some visual effects, 
        // reducing shadow resolution, etc.
        DynamicBatching.enabled = true;
        StaticBatchingUtility.Combine(gameObject);
    }
}
```

## Best Practices for Unity in Robotics

1. **Start Simple**: Begin with basic geometry before adding complex materials and lighting
2. **Optimize Early**: Maintain good frame rates for real-time robotics applications
3. **Match Real-World Specs**: Configure sensors and cameras to match real hardware
4. **Modular Design**: Create reusable components for different robot types
5. **Validation**: Compare Unity sensor output with real sensors when possible

## Chapter Summary

Unity provides a powerful platform for high-fidelity visualization of humanoid robots, particularly for HRI studies and photorealistic environments. By combining Unity's advanced rendering capabilities with ROS/ROS2 communication, researchers can create compelling simulation environments that closely match real-world appearance. The integration of Unity with robotics workflows enables new possibilities for visualization, data synthesis, and HRI research that complement traditional physics-based simulators like Gazebo.

## Learning Objectives

After completing this chapter, the reader should be able to:
- Install and configure Unity for robotics applications
- Set up communication between Unity and ROS2
- Create realistic humanoid robot models in Unity
- Implement high-fidelity rendering techniques
- Develop human-robot interaction scenarios in Unity
- Optimize Unity performance for real-time robotics applications
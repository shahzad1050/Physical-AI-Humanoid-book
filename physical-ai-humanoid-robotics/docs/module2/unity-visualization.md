# Unity for Robot Visualization

## Introduction to Unity for Robotics

Unity is a powerful 3D development platform that has gained significant traction in robotics for creating high-fidelity visualizations, simulations, and digital twins. While traditionally used for gaming, Unity's real-time rendering capabilities, physics engine, and extensibility make it an excellent choice for robotics applications.

## Why Unity for Robotics?

### Advantages
- **High-Quality Rendering**: State-of-the-art graphics for photorealistic simulation
- **Real-time Performance**: Optimized for interactive applications
- **Extensive Asset Library**: Thousands of 3D models, materials, and environments
- **Cross-Platform Support**: Deploy to multiple platforms and devices
- **Active Community**: Large developer community with extensive resources
- **Flexible Integration**: APIs for connecting with external systems

### Use Cases in Robotics
- **High-fidelity simulation** for perception system training
- **Digital twin visualization** for monitoring real robots
- **Virtual reality (VR) interfaces** for robot teleoperation
- **Training environments** for reinforcement learning
- **Public demonstrations** and educational tools

## Unity Robotics Setup

### Installation Requirements

1. **Unity Hub**: Download from Unity's official website
2. **Unity Editor**: Install version 2021.3 LTS or later
3. **Unity Robotics Hub**: Extension for robotics-specific tools

### Installing Unity Robotics Tools

```bash
# Through Unity Package Manager:
# Window → Package Manager → Unity Registry
# Install:
# - Unity Robotics Hub
# - ROS-TCP-Connector
# - Unity Perception
# - ML-Agents (for AI training)
```

## Unity ROS Integration

### ROS-TCP-Connector

The ROS-TCP-Connector package enables communication between Unity and ROS 2:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<UInt64Msg>("robot_command");
    }

    void Update()
    {
        // Send a message to ROS
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ros.Publish("robot_command", new UInt64Msg(1));
        }
    }
}
```

### Setting Up the Connection

1. **Start ROS 2 bridge** in your terminal:
```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

2. **Configure Unity** to connect to the bridge:
```csharp
// In Unity, configure the ROS connection
using Unity.Robotics.ROSTCPConnector;

public class ROSConnectionSetup : MonoBehaviour
{
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    void Start()
    {
        ROSConnection.instance = ROSConnection.GetOrCreateInstance();
        ROSConnection.instance.RosIPAddress = rosIPAddress;
        ROSConnection.instance.RosPort = rosPort;
    }
}
```

## Creating Robot Models in Unity

### Importing Robot Models

Unity supports several 3D model formats:
- **FBX**: Recommended for complex models with animations
- **OBJ**: Simple geometry import
- **DAE**: Collada format for interchange
- **USD**: Universal Scene Description (Unity native)

### Robot Structure in Unity

```csharp
using UnityEngine;

public class RobotModel : MonoBehaviour
{
    [Header("Robot Joints")]
    public Transform baseLink;
    public Transform joint1;
    public Transform joint2;
    public Transform endEffector;

    [Header("Joint Limits")]
    public float joint1Min = -90f;
    public float joint1Max = 90f;
    public float joint2Min = -45f;
    public float joint2Max = 45f;

    // Forward kinematics calculation
    public Vector3 CalculateEndEffectorPosition()
    {
        return endEffector.position;
    }

    // Inverse kinematics (simplified)
    public void SetJointAngles(float angle1, float angle2)
    {
        angle1 = Mathf.Clamp(angle1, joint1Min, joint1Max);
        angle2 = Mathf.Clamp(angle2, joint2Min, joint2Max);

        joint1.localEulerAngles = new Vector3(0, 0, angle1);
        joint2.localEulerAngles = new Vector3(0, 0, angle2);
    }
}
```

### Joint Control System

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointController : MonoBehaviour
{
    ROSConnection ros;
    public string jointStateTopic = "/joint_states";
    public Transform[] joints;
    public string[] jointNames;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        for (int i = 0; i < jointNames.Length; i++)
        {
            int jointIndex = System.Array.IndexOf(jointState.name, jointNames[i]);
            if (jointIndex >= 0 && jointIndex < jointState.position.Length)
            {
                // Apply joint position to Unity transform
                joints[i].localEulerAngles = new Vector3(0, 0, (float)jointState.position[jointIndex] * Mathf.Rad2Deg);
            }
        }
    }
}
```

## Sensor Simulation in Unity

### Camera Sensors

Unity's built-in cameras can simulate various types of vision sensors:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class UnityCameraSensor : MonoBehaviour
{
    Camera cam;
    ROSConnection ros;
    public string imageTopic = "/camera/image_raw";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float updateRate = 30.0f;

    RenderTexture renderTexture;
    Texture2D texture2D;

    void Start()
    {
        cam = GetComponent<Camera>();
        ros = ROSConnection.GetOrCreateInstance();

        // Create render texture for camera
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        cam.targetTexture = renderTexture;

        InvokeRepeating("CaptureAndSendImage", 0, 1.0f / updateRate);
    }

    void CaptureAndSendImage()
    {
        // Set the active render texture
        RenderTexture.active = renderTexture;

        // Read pixels from the render texture
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();

        // Convert to ROS message format (simplified)
        byte[] imageData = texture2D.EncodeToJPG();

        // Send image via ROS (implementation depends on message format)
        // ros.Publish(imageTopic, imageMsg);
    }
}
```

### LIDAR Simulation

Unity can simulate LIDAR sensors using raycasting:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

public class UnityLidar : MonoBehaviour
{
    public int horizontalSamples = 720;
    public int verticalSamples = 1;
    public float minAngle = -Mathf.PI;
    public float maxAngle = Mathf.PI;
    public float maxRange = 30.0f;
    public LayerMask detectionMask = -1; // All layers

    ROSConnection ros;
    public string scanTopic = "/scan";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        InvokeRepeating("SimulateLidar", 0, 0.1f); // 10Hz
    }

    void SimulateLidar()
    {
        List<float> ranges = new List<float>();

        float angleIncrement = (maxAngle - minAngle) / horizontalSamples;

        for (int i = 0; i < horizontalSamples; i++)
        {
            float angle = minAngle + i * angleIncrement;

            Vector3 direction = new Vector3(
                Mathf.Cos(angle),
                0,
                Mathf.Sin(angle)
            );

            RaycastHit hit;
            if (Physics.Raycast(transform.position, transform.TransformDirection(direction),
                              out hit, maxRange, detectionMask))
            {
                ranges.Add(hit.distance);
            }
            else
            {
                ranges.Add(maxRange); // No obstacle detected
            }
        }

        // Publish ranges to ROS (simplified)
        // ros.Publish(scanTopic, laserScanMsg);
    }
}
```

## Unity Perception Package

The Unity Perception package provides tools for generating synthetic training data:

### Semantic Segmentation

```csharp
using UnityEngine;
using Unity.Perception.GroundTruth;
using Unity.Perception.GroundTruth.LabelManagement;

public class SemanticSegmentationSetup : MonoBehaviour
{
    void Start()
    {
        // Assign semantic labels to objects
        AssignSemanticLabels();
    }

    void AssignSemanticLabels()
    {
        GameObject[] objects = GameObject.FindGameObjectsWithTag("SemanticObject");

        foreach (GameObject obj in objects)
        {
            var labeler = obj.GetComponent<SemanticLabeler>();
            if (labeler == null)
                labeler = obj.AddComponent<SemanticLabeler>();

            // Assign labels based on object type
            if (obj.tag == "Robot")
                labeler.labelId = 1;
            else if (obj.tag == "Obstacle")
                labeler.labelId = 2;
            else if (obj.tag == "Ground")
                labeler.labelId = 3;
        }
    }
}
```

### Dataset Generation

```csharp
using UnityEngine;
using Unity.Perception.GroundTruth;
using System.Collections;

public class DatasetGenerator : MonoBehaviour
{
    public int datasetSize = 1000;
    public float captureInterval = 0.5f;

    void Start()
    {
        StartCoroutine(GenerateDataset());
    }

    IEnumerator GenerateDataset()
    {
        for (int i = 0; i < datasetSize; i++)
        {
            // Move objects to random positions
            RandomizeScene();

            // Capture ground truth data
            CaptureGroundTruth();

            yield return new WaitForSeconds(captureInterval);
        }
    }

    void RandomizeScene()
    {
        // Randomly position objects in the scene
        GameObject[] sceneObjects = GameObject.FindGameObjectsWithTag("RandomObject");

        foreach (GameObject obj in sceneObjects)
        {
            Vector3 randomPos = new Vector3(
                Random.Range(-5f, 5f),
                Random.Range(0f, 2f),
                Random.Range(-5f, 5f)
            );

            obj.transform.position = randomPos;

            // Random rotation
            obj.transform.rotation = Random.rotation;
        }
    }

    void CaptureGroundTruth()
    {
        // Trigger capture of all ground truth data
        // This would typically save images and annotations
    }
}
```

## Digital Twin Implementation

### Real-time Synchronization

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

public class DigitalTwin : MonoBehaviour
{
    ROSConnection ros;
    public string tfTopic = "/tf";
    public string odomTopic = "/odom";

    // Store robot state
    Vector3 realPosition;
    Quaternion realRotation;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(odomTopic, OnOdometryReceived);
        ros.Subscribe<TFMessage>(tfTopic, OnTFReceived);
    }

    void OnOdometryReceived(OdometryMsg odom)
    {
        // Update digital twin with real robot pose
        realPosition = new Vector3(
            (float)odom.pose.pose.position.x,
            (float)odom.pose.pose.position.y,
            (float)odom.pose.pose.position.z
        );

        realRotation = new Quaternion(
            (float)odom.pose.pose.orientation.x,
            (float)odom.pose.pose.orientation.y,
            (float)odom.pose.pose.orientation.z,
            (float)odom.pose.pose.orientation.w
        );

        // Apply to Unity object
        transform.position = realPosition;
        transform.rotation = realRotation;
    }

    void OnTFReceived(TFMessage tf)
    {
        // Handle transform updates for multi-link robots
        foreach (var transform in tf.transforms)
        {
            // Update corresponding Unity object
            UpdateLinkPose(transform.child_frame_id, transform.transform);
        }
    }

    void UpdateLinkPose(string frameId, TransformMsg tf)
    {
        GameObject linkObject = GameObject.Find(frameId);
        if (linkObject != null)
        {
            linkObject.transform.position = new Vector3(
                (float)tf.translation.x,
                (float)tf.translation.y,
                (float)tf.translation.z
            );

            linkObject.transform.rotation = new Quaternion(
                (float)tf.rotation.x,
                (float)tf.rotation.y,
                (float)tf.rotation.z,
                (float)tf.rotation.w
            );
        }
    }
}
```

## Performance Optimization

### Level of Detail (LOD)

```csharp
using UnityEngine;

public class RobotLOD : MonoBehaviour
{
    public float[] lodDistances = {10f, 30f, 50f};
    public GameObject[] lodObjects;

    Camera mainCamera;

    void Start()
    {
        mainCamera = Camera.main;
        UpdateLOD();
    }

    void Update()
    {
        UpdateLOD();
    }

    void UpdateLOD()
    {
        if (mainCamera == null) return;

        float distance = Vector3.Distance(transform.position, mainCamera.transform.position);

        for (int i = 0; i < lodDistances.Length; i++)
        {
            if (distance <= lodDistances[i])
            {
                ActivateLOD(i);
                return;
            }
        }

        // Use lowest detail if beyond all distances
        ActivateLOD(lodObjects.Length - 1);
    }

    void ActivateLOD(int lodIndex)
    {
        for (int i = 0; i < lodObjects.Length; i++)
        {
            if (lodObjects[i] != null)
            {
                lodObjects[i].SetActive(i == lodIndex);
            }
        }
    }
}
```

### Occlusion Culling

Enable Unity's built-in occlusion culling for better performance:

```csharp
// In Unity Editor: Window → Rendering → Occlusion Culling
// Mark static objects as "Occluder Static" or "Occludee Static"
```

## VR Integration for Teleoperation

### VR Teleoperation Interface

```csharp
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;

public class VRTeleoperation : MonoBehaviour
{
    ROSConnection ros;
    public string commandTopic = "/teleop_command";

    InputDevice leftController;
    InputDevice rightController;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Initialize XR devices
        var devices = new List<InputDevice>();
        InputDevices.GetDevices(devices);

        foreach (var device in devices)
        {
            if (device.role == InputDeviceRole.LeftHanded)
                leftController = device;
            else if (device.role == InputDeviceRole.RightHanded)
                rightController = device;
        }
    }

    void Update()
    {
        // Get controller inputs
        if (leftController.isValid)
        {
            // Get trigger value
            float triggerValue;
            if (leftController.TryGetFeatureValue(CommonUsages.trigger, out triggerValue))
            {
                // Send command based on trigger
                SendTeleoperationCommand(triggerValue);
            }
        }
    }

    void SendTeleoperationCommand(float value)
    {
        // Publish command to ROS
        // ros.Publish(commandTopic, commandMsg);
    }
}
```

## Best Practices

### 1. Scene Organization
- Use meaningful names for GameObjects
- Group related objects under parent GameObjects
- Use tags and layers for efficient selection

### 2. Resource Management
- Use object pooling for frequently instantiated objects
- Implement proper garbage collection
- Optimize textures and 3D models for performance

### 3. Network Communication
- Implement proper error handling for ROS connections
- Use appropriate update rates to avoid network congestion
- Consider data compression for large messages

### 4. Physics Settings
- Use appropriate physics settings for your application
- Balance accuracy with performance requirements
- Test with various scenarios to ensure stability

## Integration Workflow

### Development Pipeline
```
[ROS 2 Nodes] ↔ [ROS-TCP-Endpoint] ↔ [Unity Scene] ↔ [Visualization]
```

1. **Design Phase**: Create 3D models and scenes in Unity
2. **Integration Phase**: Connect Unity to ROS 2 using TCP connector
3. **Testing Phase**: Validate data flow and visualization accuracy
4. **Optimization Phase**: Improve performance and visual quality
5. **Deployment Phase**: Package for target platform

## Troubleshooting Common Issues

### Connection Problems
```csharp
// Verify ROS connection
void CheckConnection()
{
    if (ROSConnection.instance == null)
    {
        Debug.LogError("ROS Connection not initialized!");
        return;
    }

    if (!ROSConnection.instance.IsConnected())
    {
        Debug.LogWarning("Not connected to ROS bridge");
    }
}
```

### Performance Issues
- Reduce polygon count of 3D models
- Use texture atlasing to reduce draw calls
- Implement frustum culling for distant objects
- Optimize shader complexity

## Summary

Unity provides a powerful platform for robotics visualization and simulation, offering high-quality graphics, real-time performance, and extensive integration capabilities. When combined with ROS 2, Unity enables the creation of sophisticated digital twins, training environments, and visualization tools that bridge the gap between simulation and reality.

The Unity Robotics ecosystem continues to evolve with new tools and capabilities, making it an increasingly attractive option for robotics researchers and developers who need high-fidelity visualization and simulation capabilities.

In the next section, we'll create a practical lab exercise to apply these concepts.
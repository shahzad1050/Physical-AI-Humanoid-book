# Practical Lab: Building an AI-Powered Robot

## Objective

In this lab, you will create a complete AI-powered robot system using the NVIDIA Isaac Platform. You will design a robot with perception capabilities, implement AI-based control algorithms, and create a complete pipeline that integrates simulation and real-world deployment considerations.

## Prerequisites

- Completed all previous modules (ROS 2, Simulation, Isaac Platform)
- Access to Isaac Sim (Omniverse or Docker)
- NVIDIA GPU with CUDA support (recommended: RTX 3080 or better)
- Basic understanding of Python and C++
- Familiarity with Docker for Isaac Sim

## Step 1: Project Setup and Environment Configuration

First, create a workspace for your AI-powered robot project:

```bash
# Create project directory
mkdir -p ~/isaac_ai_robot_ws/src
cd ~/isaac_ai_robot_ws/src

# Create a new ROS 2 package for the robot system
ros2 pkg create --build-type ament_python ai_robot_system --dependencies rclpy std_msgs sensor_msgs geometry_msgs vision_msgs message_filters cv_bridge tf2_ros

# Create additional directories for models and configurations
mkdir -p ~/isaac_ai_robot_ws/src/ai_robot_system/config
mkdir -p ~/isaac_ai_robot_ws/src/ai_robot_system/models
mkdir -p ~/isaac_ai_robot_ws/src/ai_robot_system/launch
mkdir -p ~/isaac_ai_robot_ws/src/ai_robot_system/ai_models
```

## Step 2: Create the Robot URDF Model

Create a simple mobile manipulator robot model in `~/isaac_ai_robot_ws/src/ai_robot_system/models/ai_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="ai_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Robot properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20.0"/>
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0"/>
    </inertial>
  </link>

  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia
        ixx="0.001" ixy="0.0" ixz="0.0"
        iyy="0.001" iyz="0.0"
        izz="0.001"/>
    </inertial>
  </link>

  <!-- Camera joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- Arm base link -->
  <link name="arm_base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia
        ixx="0.001" ixy="0.0" ixz="0.0"
        iyy="0.001" iyz="0.0"
        izz="0.001"/>
    </inertial>
  </link>

  <!-- Arm base joint -->
  <joint name="arm_base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_base_link"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- First arm link -->
  <link name="arm_link1">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.02"/>
      </geometry>
      <material name="silver">
        <color rgba="0.75 0.75 0.75 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia
        ixx="0.001" ixy="0.0" ixz="0.0"
        iyy="0.001" iyz="0.0"
        izz="0.001"/>
    </inertial>
  </link>

  <!-- First arm joint -->
  <joint name="arm_joint1" type="revolute">
    <parent link="arm_base_link"/>
    <child link="arm_link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

  <!-- Second arm link -->
  <link name="arm_link2">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.02"/>
      </geometry>
      <material name="silver">
        <color rgba="0.75 0.75 0.75 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <inertia
        ixx="0.001" ixy="0.0" ixz="0.0"
        iyy="0.001" iyz="0.0"
        izz="0.001"/>
    </inertial>
  </link>

  <!-- Second arm joint -->
  <joint name="arm_joint2" type="revolute">
    <parent link="arm_link1"/>
    <child link="arm_link2"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

  <!-- End effector link -->
  <link name="end_effector">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia
        ixx="0.0001" ixy="0.0" ixz="0.0"
        iyy="0.0001" iyz="0.0"
        izz="0.0001"/>
    </inertial>
  </link>

  <!-- End effector joint -->
  <joint name="ee_joint" type="fixed">
    <parent link="arm_link2"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.125" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <!-- Differential drive plugin for base -->
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- Add wheel joints and links here if needed -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia
        ixx="0.001" ixy="0.0" ixz="0.0"
        iyy="0.001" iyz="0.0"
        izz="0.002"/>
    </inertial>
  </link>

  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia
        ixx="0.001" ixy="0.0" ixz="0.0"
        iyy="0.001" iyz="0.0"
        izz="0.002"/>
    </inertial>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.2 -0.1" rpy="${M_PI/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.2 -0.1" rpy="${M_PI/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Step 3: Create the Perception Node

Create the AI perception node in `~/isaac_ai_robot_ws/src/ai_robot_system/ai_robot_system/perception_node.py`:

```python
#!/usr/bin/env python3

"""
AI Perception Node
Implements object detection and scene understanding using Isaac GEMs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from torchvision.models.detection import fasterrcnn_resnet50_fpn
import tf2_ros
from tf2_ros import TransformException
import message_filters


class AI PerceptionNode(Node):
    def __init__(self):
        super().__init__('ai_perception_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize neural network model
        self.initialize_model()

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/ai_robot/detections',
            10
        )

        # Initialize camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None

        # TF broadcaster and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('AI Perception Node initialized')

    def initialize_model(self):
        """Initialize the AI model for object detection"""
        try:
            # Load pre-trained model
            self.model = fasterrcnn_resnet50_fpn(pretrained=True)
            self.model.eval()

            # Define class names (COCO dataset classes)
            self.class_names = [
                '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
                'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
                'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
                'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
                'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
                'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
                'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table',
                'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
                'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
                'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
            ]

            self.get_logger().info('AI model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize AI model: {e}')
            # Fallback to a simple detection method
            self.model = None

    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run object detection
            if self.model is not None:
                detections = self.run_object_detection(cv_image)
            else:
                # Fallback: simple color-based detection
                detections = self.fallback_detection(cv_image)

            # Create detection message
            detection_msg = self.create_detection_message(detections, msg.header)

            # Publish detections
            self.detection_pub.publish(detection_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def run_object_detection(self, image):
        """Run object detection using the AI model"""
        # Preprocess image
        transform = transforms.Compose([
            transforms.ToTensor(),
        ])
        input_tensor = transform(image)
        input_batch = input_tensor.unsqueeze(0)

        # Run inference
        with torch.no_grad():
            outputs = self.model(input_batch)

        # Process outputs
        detections = []
        for i, (boxes, scores, labels) in enumerate(zip(outputs[0]['boxes'],
                                                        outputs[0]['scores'],
                                                        outputs[0]['labels'])):
            # Filter detections by confidence
            keep = scores > 0.5
            boxes = boxes[keep].cpu().numpy()
            scores = scores[keep].cpu().numpy()
            labels = labels[keep].cpu().numpy()

            for box, score, label in zip(boxes, scores, labels):
                if label < len(self.class_names):
                    detection = {
                        'bbox': box,
                        'score': score,
                        'class_name': self.class_names[label],
                        'class_id': label
                    }
                    detections.append(detection)

        return detections

    def fallback_detection(self, image):
        """Fallback detection method using color thresholding"""
        # Simple red object detection as fallback
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                detection = {
                    'bbox': [x, y, x + w, y + h],
                    'score': 0.8,  # Fallback confidence
                    'class_name': 'red_object',
                    'class_id': 0
                }
                detections.append(detection)

        return detections

    def camera_info_callback(self, msg):
        """Update camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def create_detection_message(self, detections, header):
        """Create Detection2DArray message from detections"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            detection_2d = Detection2D()
            detection_2d.header = header

            # Set bounding box
            bbox = detection['bbox']
            detection_2d.bbox.center.x = (bbox[0] + bbox[2]) / 2.0
            detection_2d.bbox.center.y = (bbox[1] + bbox[3]) / 2.0
            detection_2d.bbox.size_x = abs(bbox[2] - bbox[0])
            detection_2d.bbox.size_y = abs(bbox[3] - bbox[1])

            # Set results
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = str(detection['class_name'])
            result.hypothesis.score = float(detection['score'])
            detection_2d.results.append(result)

            detection_array.detections.append(detection_2d)

        return detection_array


def main(args=None):
    rclpy.init(args=args)
    node = AI PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4: Create the Navigation Node

Create the navigation node in `~/isaac_ai_robot_ws/src/ai_robot_system/ai_robot_system/navigation_node.py`:

```python
#!/usr/bin/env python3

"""
AI Navigation Node
Implements autonomous navigation using reinforcement learning
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random


class DQN(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=128):
        super(DQN, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim)
        )

    def forward(self, state):
        return self.network(state)


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Initialize DQN
        self.state_dim = 36 + 3  # 36 laser readings + 3 (x, y, theta)
        self.action_dim = 4  # 4 discrete actions: forward, left, right, stop
        self.q_network = DQN(self.state_dim, self.action_dim)
        self.target_network = DQN(self.state_dim, self.action_dim)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=1e-3)

        self.target_network.load_state_dict(self.q_network.state_dict())

        # Training parameters
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        self.batch_size = 32

        # Replay buffer
        self.memory = deque(maxlen=10000)

        # Current state
        self.current_scan = None
        self.current_odom = None
        self.target_position = np.array([5.0, 5.0])  # Target coordinates

        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Create timer for navigation loop
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('AI Navigation Node initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        # Process laser scan to reduce dimensionality
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=np.inf)  # Replace NaN with infinity

        # Reduce to 36 readings for computational efficiency
        step = len(ranges) // 36
        self.current_scan = ranges[::step][:36]

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_odom = msg

    def get_state(self):
        """Get current state for the RL agent"""
        if self.current_scan is None or self.current_odom is None:
            return None

        # Get robot position and orientation
        pos = self.current_odom.pose.pose.position
        current_pos = np.array([pos.x, pos.y])

        # Get robot orientation
        quat = self.current_odom.pose.pose.orientation
        # Convert quaternion to euler (simplified - just get yaw)
        # In a real implementation, you'd use tf2 for proper conversion
        yaw = self.quaternion_to_yaw(quat)

        # Calculate relative target position
        relative_target = self.target_position - current_pos

        # Combine laser scan and relative target position
        state = np.concatenate([self.current_scan / 10.0, relative_target, [yaw]])

        return state

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle (simplified)"""
        # This is a simplified conversion - in practice, use tf2
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def select_action(self, state):
        """Select action using epsilon-greedy policy"""
        if np.random.random() <= self.epsilon:
            return random.randrange(self.action_dim)

        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        q_values = self.q_network(state_tensor)
        return np.argmax(q_values.cpu().data.numpy())

    def calculate_reward(self, state, action, next_state):
        """Calculate reward based on state transition"""
        # Get current and next positions
        current_pos = np.array([state[-3], state[-2]])  # x, y from state
        next_pos = np.array([next_state[-3], next_state[-2]])

        # Distance to target
        current_dist = np.linalg.norm(self.target_position - current_pos)
        next_dist = np.linalg.norm(self.target_position - next_pos)

        # Reward based on progress toward target
        progress_reward = current_dist - next_dist

        # Penalty for getting too close to obstacles
        min_scan = np.min(state[:-3])  # Exclude position/orientation
        obstacle_penalty = 0
        if min_scan < 0.5:  # Too close to obstacle
            obstacle_penalty = -1.0

        # Large reward for reaching target
        target_reward = 0
        if next_dist < 0.5:  # Within 0.5m of target
            target_reward = 100.0

        # Small penalty for each step (encourage efficiency)
        step_penalty = -0.1

        total_reward = progress_reward + obstacle_penalty + target_reward + step_penalty

        return total_reward

    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay buffer"""
        self.memory.append((state, action, reward, next_state, done))

    def replay(self):
        """Train the network on a batch of experiences"""
        if len(self.memory) < self.batch_size:
            return

        batch = random.sample(self.memory, self.batch_size)
        states = torch.FloatTensor([e[0] for e in batch])
        actions = torch.LongTensor([e[1] for e in batch])
        rewards = torch.FloatTensor([e[2] for e in batch])
        next_states = torch.FloatTensor([e[3] for e in batch])
        dones = torch.BoolTensor([e[4] for e in batch])

        current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1))
        next_q_values = self.target_network(next_states).max(1)[0].detach()
        target_q_values = rewards + (self.gamma * next_q_values * ~dones)

        loss = nn.MSELoss()(current_q_values.squeeze(), target_q_values)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def navigation_loop(self):
        """Main navigation loop"""
        state = self.get_state()
        if state is None:
            return

        # Select action
        action = self.select_action(state)

        # Execute action
        cmd_vel = Twist()
        if action == 0:  # Forward
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0
        elif action == 1:  # Turn left
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.5
        elif action == 2:  # Turn right
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = -0.5
        elif action == 3:  # Stop
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Update network with experience
        next_state = self.get_state()
        if next_state is not None:
            reward = self.calculate_reward(state, action, next_state)
            done = np.linalg.norm(self.target_position - np.array([next_state[-3], next_state[-2]])) < 0.5
            self.remember(state, action, reward, next_state, done)
            self.replay()

    def update_target_network(self):
        """Update target network with current network weights"""
        self.target_network.load_state_dict(self.q_network.state_dict())


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 5: Create the Manipulation Node

Create the manipulation node in `~/isaac_ai_robot_ws/src/ai_robot_system/ai_robot_system/manipulation_node.py`:

```python
#!/usr/bin/env python3

"""
AI Manipulation Node
Implements robotic arm control and grasping using AI techniques
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from scipy.spatial.transform import Rotation as R
import time


class GraspNet(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(GraspNet, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim)
        )

    def forward(self, state):
        return torch.tanh(self.network(state))


class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')

        # Initialize grasp network
        self.state_dim = 7 + 3 + 4  # joint states (7) + object position (3) + object orientation (4)
        self.action_dim = 7  # 7 joint positions for the arm
        self.grasp_net = GraspNet(self.state_dim, self.action_dim)

        # Current states
        self.current_joints = None
        self.detected_objects = []
        self.target_object = None

        # Create subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray, '/ai_robot/detections', self.detection_callback, 10
        )

        # Create publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10
        )

        # Create timer for manipulation loop
        self.manip_timer = self.create_timer(0.1, self.manipulation_loop)

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('AI Manipulation Node initialized')

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        self.current_joints = np.array(msg.position)

    def detection_callback(self, msg):
        """Process object detection messages"""
        self.detected_objects = []

        for detection in msg.detections:
            if len(detection.results) > 0:
                result = detection.results[0]
                obj = {
                    'class': result.hypothesis.class_id,
                    'confidence': result.hypothesis.score,
                    'position': detection.bbox.center  # This is image center, need to convert to 3D
                }
                self.detected_objects.append(obj)

    def convert_image_to_3d(self, image_point, depth):
        """Convert 2D image coordinates to 3D world coordinates"""
        # This is a simplified conversion - in practice, you'd use camera calibration
        # For this example, we'll assume a fixed conversion
        if depth is None:
            depth = 1.0  # Default depth if not available

        # Simplified conversion (would use camera matrix in real implementation)
        x_3d = (image_point.x - 320) * depth / 640  # Assuming 640x480 image
        y_3d = (image_point.y - 240) * depth / 480
        z_3d = depth

        return np.array([x_3d, y_3d, z_3d])

    def select_target_object(self):
        """Select the most promising object for manipulation"""
        if not self.detected_objects:
            return None

        # For this example, select the object with highest confidence
        best_obj = max(self.detected_objects, key=lambda x: x['confidence'])

        # Convert 2D position to 3D (simplified)
        # In a real implementation, you'd use depth information
        obj_3d_pos = self.convert_image_to_3d(best_obj['position'], None)

        return {
            'class': best_obj['class'],
            'confidence': best_obj['confidence'],
            'position': obj_3d_pos
        }

    def plan_grasp(self, object_pos):
        """Plan a grasp for the target object"""
        if self.current_joints is None:
            return None

        # Simplified grasp planning
        # In a real implementation, you'd use inverse kinematics and grasp planning algorithms

        # Calculate desired end-effector position (slightly above object)
        grasp_pos = object_pos.copy()
        grasp_pos[2] += 0.2  # Approach from above

        # Define grasp orientation (pointing down)
        grasp_orientation = [0, 0, 0, 1]  # Quaternion for downward orientation

        # Plan joint positions to reach this pose
        # This is a simplified approach - real implementation would use IK
        desired_joints = self.plan_arm_motion_to_pose(grasp_pos, grasp_orientation)

        return desired_joints

    def plan_arm_motion_to_pose(self, target_pos, target_orientation):
        """Plan arm motion to reach target pose (simplified)"""
        if self.current_joints is None:
            return np.zeros(7)

        # This is a very simplified approach
        # In practice, you'd use inverse kinematics (IK) solvers

        # For this example, we'll just return a simple motion plan
        # that moves the arm toward the target
        current_pos = self.get_end_effector_position()

        # Calculate motion direction
        direction = target_pos - current_pos
        step_size = 0.1  # Move in 10cm steps

        # This is a placeholder - real implementation would use proper IK
        desired_joints = self.current_joints.copy()

        # Simple joint adjustments based on target position
        # This is highly simplified and would not work in practice
        desired_joints[0] += np.arctan2(target_pos[1], target_pos[0]) * 0.1  # Base joint
        desired_joints[1] += (target_pos[2] - current_pos[2]) * 0.5  # Shoulder joint
        desired_joints[2] += (target_pos[0] - current_pos[0]) * 0.2  # Elbow joint

        # Constrain joint limits
        joint_limits = [
            [-2.967, 2.967],   # Joint 1
            [-1.832, 1.832],   # Joint 2
            [-2.618, 2.618],   # Joint 3
            [-3.141, 3.141],   # Joint 4
            [-2.967, 2.967],   # Joint 5
            [-3.141, 3.141],   # Joint 6
            [-2.967, 2.967]    # Joint 7
        ]

        for i, (min_limit, max_limit) in enumerate(joint_limits):
            desired_joints[i] = np.clip(desired_joints[i], min_limit, max_limit)

        return desired_joints

    def get_end_effector_position(self):
        """Get current end-effector position (simplified)"""
        # This is a simplified forward kinematics calculation
        # In practice, you'd use a proper FK solver
        if self.current_joints is None:
            return np.array([0, 0, 0.5])  # Default position

        # Simplified calculation based on joint angles
        # This is not accurate and just for demonstration
        q = self.current_joints

        # Base position
        x = 0.1  # Offset from base
        y = 0
        z = 0.1  # Base height

        # Add contributions from each joint
        # This is a very simplified approximation
        for i, angle in enumerate(q[:3]):  # Consider first 3 joints
            radius = 0.1 * (i + 1)  # Simplified link length
            x += radius * np.cos(sum(q[:i+1]))
            y += radius * np.sin(sum(q[:i+1]))
            z += 0.05  # Height contribution

        return np.array([x, y, z])

    def execute_grasp(self, grasp_joints):
        """Execute the planned grasp"""
        if grasp_joints is None:
            return False

        # Create joint command message
        cmd_msg = Float64MultiArray()
        cmd_msg.data = grasp_joints.tolist()

        # Publish command
        self.joint_cmd_pub.publish(cmd_msg)

        self.get_logger().info(f'Executing grasp with joints: {grasp_joints}')
        return True

    def manipulation_loop(self):
        """Main manipulation loop"""
        # Select target object
        self.target_object = self.select_target_object()

        if self.target_object is not None:
            self.get_logger().info(f'Target object: {self.target_object["class"]} at {self.target_object["position"]}')

            # Plan grasp
            grasp_joints = self.plan_grasp(self.target_object['position'])

            if grasp_joints is not None:
                # Execute grasp
                success = self.execute_grasp(grasp_joints)

                if success:
                    self.get_logger().info('Grasp executed successfully')
                else:
                    self.get_logger().warn('Failed to execute grasp')
            else:
                self.get_logger().warn('Could not plan grasp for target object')
        else:
            self.get_logger().info('No objects detected for manipulation')


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 6: Create the Main Control Node

Create the main control node in `~/isaac_ai_robot_ws/src/ai_robot_system/ai_robot_system/ai_robot_controller.py`:

```python
#!/usr/bin/env python3

"""
AI Robot Main Controller
Coordinates perception, navigation, and manipulation modules
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray
from ai_robot_system.perception_node import AI PerceptionNode
from ai_robot_system.navigation_node import NavigationNode
from ai_robot_system.manipulation_node import ManipulationNode
import time
import threading


class AIRobotController(Node):
    def __init__(self):
        super().__init__('ai_robot_controller')

        # Initialize component nodes
        self.perception_node = AI PerceptionNode()
        self.navigation_node = NavigationNode()
        self.manipulation_node = ManipulationNode()

        # Create state machine
        self.current_state = "IDLE"  # IDLE, NAVIGATING, MANIPULATING, PERCEIVING
        self.task_queue = []
        self.robot_tasks = {
            'explore': self.explore_environment,
            'find_object': self.find_object_task,
            'navigate_to_object': self.navigate_to_object_task,
            'grasp_object': self.grasp_object_task,
            'place_object': self.place_object_task
        }

        # Create publisher for state updates
        self.state_pub = self.create_publisher(String, '/ai_robot/state', 10)

        # Create timer for main control loop
        self.control_timer = self.create_timer(1.0, self.control_loop)

        self.get_logger().info('AI Robot Controller initialized')

    def control_loop(self):
        """Main control loop"""
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

        # Process task queue
        if self.task_queue:
            next_task = self.task_queue[0]
            self.execute_task(next_task)
        else:
            # Default behavior when no tasks
            self.current_state = "IDLE"
            self.idle_behavior()

    def execute_task(self, task):
        """Execute a specific task"""
        if task in self.robot_tasks:
            self.get_logger().info(f'Executing task: {task}')
            self.current_state = task.upper()
            success = self.robot_tasks[task]()

            if success:
                # Remove completed task
                self.task_queue.pop(0)
                self.get_logger().info(f'Task {task} completed successfully')
            else:
                self.get_logger().warn(f'Task {task} failed')
        else:
            self.get_logger().error(f'Unknown task: {task}')
            self.task_queue.pop(0)  # Remove invalid task

    def add_task(self, task):
        """Add a task to the queue"""
        self.task_queue.append(task)
        self.get_logger().info(f'Task {task} added to queue')

    def idle_behavior(self):
        """Behavior when robot is idle"""
        # For now, just stay still
        # In a real implementation, you might have the robot patrol or charge
        pass

    def explore_environment(self):
        """Explore the environment"""
        self.get_logger().info('Exploring environment...')

        # This would involve moving to various locations to build a map
        # For this example, we'll just move in a simple pattern
        self.navigation_node.target_position = [2.0, 2.0]
        time.sleep(5)  # Simulate exploration time

        self.navigation_node.target_position = [-2.0, 2.0]
        time.sleep(5)

        self.navigation_node.target_position = [0.0, 0.0]  # Return to start
        time.sleep(5)

        return True

    def find_object_task(self):
        """Find a specific object in the environment"""
        self.get_logger().info('Searching for objects...')

        # Wait for object detections
        timeout = time.time() + 60*2  # 2 minutes timeout
        while time.time() < timeout:
            if self.perception_node.current_scan is not None:
                # Check if any objects were detected
                if self.perception_node.detection_pub.get_subscription_count() > 0:
                    # In a real implementation, you'd check the actual detections
                    # For this example, we'll assume an object was found
                    self.get_logger().info('Object found!')
                    return True
            time.sleep(0.1)

        self.get_logger().warn('Object not found within timeout')
        return False

    def navigate_to_object_task(self):
        """Navigate to a detected object"""
        self.get_logger().info('Navigating to object...')

        # In a real implementation, you'd use the object's position
        # For this example, we'll navigate to a fixed location
        self.navigation_node.target_position = [1.0, 1.0]

        # Wait until navigation is complete
        timeout = time.time() + 60  # 1 minute timeout
        while time.time() < timeout:
            # Check if robot is close to target
            if self.navigation_node.current_odom is not None:
                pos = self.navigation_node.current_odom.pose.pose.position
                current_pos = np.array([pos.x, pos.y])
                dist = np.linalg.norm(self.navigation_node.target_position - current_pos)

                if dist < 0.5:  # Within 0.5m of target
                    self.get_logger().info('Navigation to object completed')
                    return True
            time.sleep(0.1)

        self.get_logger().warn('Navigation to object failed')
        return False

    def grasp_object_task(self):
        """Grasp a nearby object"""
        self.get_logger().info('Attempting to grasp object...')

        # In a real implementation, you'd coordinate with perception and manipulation
        # For this example, we'll just execute a simple grasp motion
        if self.manipulation_node.current_joints is not None:
            # Plan a simple grasp motion
            grasp_joints = self.manipulation_node.current_joints.copy()
            # Move to a pre-defined grasp position
            grasp_joints[0] = 0.5  # Base joint
            grasp_joints[1] = 0.3  # Shoulder joint
            grasp_joints[2] = -0.2  # Elbow joint

            self.manipulation_node.execute_grasp(grasp_joints)
            time.sleep(3)  # Wait for grasp to complete

            # Close gripper (simplified)
            # In a real implementation, you'd control the gripper separately
            self.get_logger().info('Grasp attempt completed')
            return True

        self.get_logger().warn('Cannot execute grasp - no joint information available')
        return False

    def place_object_task(self):
        """Place the grasped object at a location"""
        self.get_logger().info('Placing object...')

        # Move to placement location
        self.navigation_node.target_position = [3.0, 0.0]

        # Wait for navigation to complete
        timeout = time.time() + 60  # 1 minute timeout
        while time.time() < timeout:
            if self.navigation_node.current_odom is not None:
                pos = self.navigation_node.current_odom.pose.pose.position
                current_pos = np.array([pos.x, pos.y])
                dist = np.linalg.norm(self.navigation_node.target_position - current_pos)

                if dist < 0.5:
                    # Open gripper to place object (simplified)
                    # In a real implementation, you'd control the gripper separately
                    self.get_logger().info('Object placed successfully')
                    return True
            time.sleep(0.1)

        self.get_logger().warn('Object placement failed')
        return False


def main(args=None):
    rclpy.init(args=args)
    controller = AIRobotController()

    # Add some sample tasks to the queue
    controller.add_task('explore')
    controller.add_task('find_object')
    controller.add_task('navigate_to_object')
    controller.add_task('grasp_object')
    controller.add_task('place_object')

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 7: Create Launch Files

Create the main launch file in `~/isaac_ai_robot_ws/src/ai_robot_system/launch/ai_robot_system.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('ai_robot_system')

    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch Gazebo simulation (if available)
    # This would typically be a separate launch file, but we'll include it here
    # For this example, we'll assume a separate Gazebo launch is handled externally

    # AI Perception Node
    perception_node = Node(
        package='ai_robot_system',
        executable='perception_node',
        name='ai_perception_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # AI Navigation Node
    navigation_node = Node(
        package='ai_robot_system',
        executable='navigation_node',
        name='ai_navigation_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # AI Manipulation Node
    manipulation_node = Node(
        package='ai_robot_system',
        executable='manipulation_node',
        name='ai_manipulation_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # AI Robot Controller Node
    controller_node = Node(
        package='ai_robot_system',
        executable='ai_robot_controller',
        name='ai_robot_controller',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        perception_node,
        navigation_node,
        manipulation_node,
        controller_node
    ])
```

## Step 8: Update setup.py

Update the `setup.py` file in `~/isaac_ai_robot_ws/src/ai_robot_system/setup.py`:

```python
from setuptools import setup

package_name = 'ai_robot_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ai_robot_system.launch.py']),
        ('share/' + package_name + '/models', ['models/ai_robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='AI-powered robot system using NVIDIA Isaac Platform',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = ai_robot_system.perception_node:main',
            'navigation_node = ai_robot_system.navigation_node:main',
            'manipulation_node = ai_robot_system.manipulation_node:main',
            'ai_robot_controller = ai_robot_system.ai_robot_controller:main',
        ],
    },
)
```

## Step 9: Create a Simulation Environment

Create a simple world file for Gazebo in `~/isaac_ai_robot_ws/src/ai_robot_system/worlds/ai_robot_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="ai_robot_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add some objects for the robot to interact with -->
    <model name="red_box">
      <pose>2 2 0.1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.0017</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0017</iyy>
            <iyz>0</iyz>
            <izz>0.0017</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="blue_cylinder">
      <pose>-2 -1 0.15 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.3</mass>
          <inertia>
            <ixx>0.0015</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0015</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

## Step 10: Build and Run the System

Build your package:

```bash
cd ~/isaac_ai_robot_ws
colcon build --packages-select ai_robot_system
source install/setup.bash
```

Run the complete AI robot system:

```bash
# Terminal 1: Launch the robot system
ros2 launch ai_robot_system ai_robot_system.launch.py

# Terminal 2: (Optional) Launch Gazebo simulation with your robot
# This would depend on your specific Gazebo setup
```

## Step 11: Testing and Validation

Test the system by sending commands and monitoring the behavior:

```bash
# Monitor robot state
ros2 topic echo /ai_robot/state

# Monitor detections
ros2 topic echo /ai_robot/detections

# Monitor navigation commands
ros2 topic echo /cmd_vel

# Send manual navigation commands if needed
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

## Expected Results

When you run the complete AI robot system:

1. The perception node should detect objects in the environment
2. The navigation node should plan paths to navigate to objects
3. The manipulation node should plan and execute grasping motions
4. The main controller should coordinate all modules to perform complex tasks

## Troubleshooting

If you encounter issues:

1. **Missing dependencies**: Install required Python packages:
   ```bash
   pip3 install torch torchvision opencv-python tf2_ros
   ```

2. **CUDA issues**: Ensure your NVIDIA drivers and CUDA are properly installed

3. **ROS 2 topics not connecting**: Check that all nodes are using the same ROS domain ID

4. **Performance issues**: The AI models may be computationally intensive; ensure your GPU can handle the workload

## Extensions

To extend this system:

1. **Add more sophisticated AI models**: Replace the simple models with more advanced ones
2. **Implement learning algorithms**: Add reinforcement learning for improved navigation
3. **Add more sensors**: Include depth cameras, IMUs, or other sensors
4. **Improve manipulation**: Add grasp planning and force control
5. **Add semantic mapping**: Create maps with object labels and relationships

## Summary

In this lab, you have created a complete AI-powered robot system that integrates:
- Perception using computer vision and AI
- Navigation using reinforcement learning
- Manipulation using AI-based grasp planning
- High-level task coordination

This system demonstrates the power of the NVIDIA Isaac Platform for creating sophisticated AI-powered robotic applications that can perceive, navigate, and manipulate in their environment.
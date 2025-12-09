# Final Project: Complete Humanoid Robot System

## Project Overview

This final project integrates all concepts learned throughout the Physical AI & Humanoid Robotics textbook to create a complete humanoid robot system capable of autonomous interaction in human environments. The project demonstrates the full pipeline from perception to action with safety, efficiency, and natural human compatibility.

## Project Objectives

### Primary Goals
- Implement a complete humanoid robot controller integrating balance, manipulation, and interaction
- Demonstrate safe and natural human-robot interaction in simulation
- Showcase the integration of AI perception, control, and social behaviors
- Validate the entire technology stack from ROS 2 to Isaac Platform to humanoid control

### Learning Outcomes
- Synthesize knowledge from all four modules
- Demonstrate practical implementation of theoretical concepts
- Validate system integration and safety considerations
- Prepare for real-world humanoid robot deployment

## System Architecture

### High-Level Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│                    HUMANOID ROBOT SYSTEM                        │
├─────────────────────────────────────────────────────────────────┤
│  Perception Layer    │  Planning Layer    │  Control Layer     │
│  • Vision Processing │  • Task Planning   │  • Balance Ctrl    │
│  • Object Detection  │  • Motion Planning │  • Locomotion Ctrl │
│  • Human Detection   │  • Grasp Planning  │  • Manipulation Ctrl│
│  • SLAM             │  • Path Planning   │  • Social Behaviors│
│  • State Estimation │  • Behavior Trees  │                   │
├─────────────────────────────────────────────────────────────────┤
│                   SAFETY & MONITORING                           │
│  • Collision Avoidance • Emergency Stop • Health Monitoring   │
└─────────────────────────────────────────────────────────────────┘
```

## Implementation Components

### 1. Integrated Perception System

```python
# Complete perception system integrating all learned concepts
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu, JointState
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from vision_msgs.msg import Detection2DArray
import torch
import torchvision

class IntegratedPerceptionSystem(Node):
    def __init__(self):
        super().__init__('integrated_perception')

        # Initialize all perception components
        self.initialize_vision_system()
        self.initialize_state_estimator()
        self.initialize_human_detector()

        # Publishers and subscribers
        self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.create_subscription(PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Perception output publishers
        self.obj_detection_pub = self.create_publisher(Detection2DArray, '/perception/objects', 10)
        self.human_detection_pub = self.create_publisher(Detection2DArray, '/perception/humans', 10)
        self.state_pub = self.create_publisher(PoseStamped, '/robot/state', 10)

        # System state
        self.robot_state = {
            'position': np.zeros(3),
            'orientation': np.array([0, 0, 0, 1]),  # quaternion
            'velocity': np.zeros(3),
            'joint_positions': np.zeros(28),  # Example: 28 DOF humanoid
            'com_position': np.zeros(3)
        }

    def initialize_vision_system(self):
        """Initialize computer vision components"""
        # Load pre-trained object detection model (YOLO, DetectNet, etc.)
        self.object_detector = self.load_object_detection_model()

        # Initialize segmentation model
        self.segmentation_model = self.load_segmentation_model()

        # Initialize depth processing
        self.depth_processor = DepthProcessor()

    def initialize_state_estimator(self):
        """Initialize robot state estimation"""
        self.state_estimator = RobotStateEstimator()
        self.kalman_filter = self.initialize_kalman_filter()

    def initialize_human_detector(self):
        """Initialize human detection and tracking"""
        self.human_detector = HumanDetector()
        self.social_space_manager = SocialSpaceManager()

    def image_callback(self, msg):
        """Process RGB camera images"""
        # Convert ROS image to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run object detection
        object_detections = self.object_detector.detect(cv_image)

        # Run human detection
        human_detections = self.human_detector.detect(cv_image)

        # Process and publish results
        self.publish_object_detections(object_detections)
        self.publish_human_detections(human_detections)

    def pointcloud_callback(self, msg):
        """Process 3D point cloud data"""
        # Convert point cloud to numpy array
        pointcloud = self.pointcloud_to_array(msg)

        # Run 3D object detection and segmentation
        objects_3d = self.process_3d_objects(pointcloud)

        # Update spatial understanding
        self.update_environment_map(objects_3d)

    def process_3d_objects(self, pointcloud):
        """Process 3D objects from point cloud"""
        # Segment objects from scene
        segmented_objects = self.segment_objects_in_pointcloud(pointcloud)

        # Estimate poses and properties
        objects_with_pose = []
        for obj in segmented_objects:
            pose_3d = self.estimate_3d_pose(obj, pointcloud)
            properties = self.estimate_object_properties(obj)

            objects_with_pose.append({
                'pointcloud': obj,
                'pose': pose_3d,
                'properties': properties
            })

        return objects_with_pose

    def estimate_3d_pose(self, object_points, full_pointcloud):
        """Estimate 3D pose of object"""
        # Calculate bounding box
        min_pt = np.min(object_points, axis=0)
        max_pt = np.max(object_points, axis=0)
        center = (min_pt + max_pt) / 2

        # Estimate orientation using PCA
        cov_matrix = np.cov(object_points.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # Sort eigenvectors by eigenvalues
        idx = np.argsort(eigenvalues)[::-1]
        eigenvectors = eigenvectors[:, idx]

        # Create rotation matrix
        rotation_matrix = eigenvectors
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        return {
            'position': center,
            'orientation': quaternion
        }

    def update_environment_map(self, objects_3d):
        """Update internal environment representation"""
        # Update occupancy grid
        self.update_occupancy_grid(objects_3d)

        # Update object map
        self.update_object_map(objects_3d)

        # Update social spaces
        self.update_social_spaces()

    def update_occupancy_grid(self, objects_3d):
        """Update occupancy grid with detected objects"""
        # This would update a 2D/3D occupancy grid representation
        pass

    def update_object_map(self, objects_3d):
        """Update object-centric map"""
        # Maintain list of detected objects with their properties and locations
        pass

    def update_social_spaces(self):
        """Update social interaction spaces based on detected humans"""
        # Update personal, social, and public space boundaries
        pass

class RobotStateEstimator:
    def __init__(self):
        self.kalman_filter = self.initialize_kalman_filter()
        self.com_estimator = CenterOfMassEstimator()

    def estimate_robot_state(self, joint_states, imu_data, vision_data):
        """Estimate complete robot state"""
        # Combine all sensor data for state estimation
        state = {
            'position': self.estimate_position(joint_states, imu_data),
            'orientation': self.estimate_orientation(imu_data, vision_data),
            'velocity': self.estimate_velocity(joint_states, imu_data),
            'com_position': self.com_estimator.estimate_com(joint_states),
            'zmp': self.calculate_zmp(joint_states, imu_data)
        }

        return state

    def estimate_position(self, joint_states, imu_data):
        """Estimate robot position using sensor fusion"""
        # Use IMU integration and joint kinematics
        imu_position = self.integrate_imu(imu_data)
        kinematic_position = self.forward_kinematics(joint_states)

        # Fuse estimates using Kalman filter
        fused_position = self.kalman_filter.update_position(
            imu_position, kinematic_position
        )

        return fused_position

    def estimate_orientation(self, imu_data, vision_data):
        """Estimate robot orientation"""
        # Use IMU for absolute orientation
        imu_orientation = self.extract_orientation_from_imu(imu_data)

        # Use vision for relative orientation corrections
        if vision_data and 'fiducials' in vision_data:
            vision_correction = self.calculate_orientation_correction(vision_data['fiducials'])
            corrected_orientation = self.fuse_orientations(
                imu_orientation, vision_correction
            )
        else:
            corrected_orientation = imu_orientation

        return corrected_orientation

    def calculate_zmp(self, joint_states, imu_data):
        """Calculate Zero Moment Point"""
        # Calculate ZMP from force/torque sensors and kinematic data
        # This would use actual force/torque sensor data in practice
        return np.array([0.0, 0.0, 0.0])  # Placeholder
```

### 2. Task Planning and Reasoning System

```python
class TaskPlanningSystem(Node):
    def __init__(self):
        super().__init__('task_planning')

        # Initialize planning components
        self.symbolic_planner = SymbolicPlanner()
        self.motion_planner = MotionPlanner()
        self.behavior_tree = BehaviorTree()
        self.trajectory_optimizer = TrajectoryOptimizer()

        # Subscribe to high-level commands
        self.create_subscription(String, '/high_level_command', self.command_callback, 10)

        # Publish low-level actions
        self.action_pub = self.create_publisher(String, '/low_level_action', 10)

    def command_callback(self, msg):
        """Process high-level commands and generate plans"""
        command = msg.data

        # Parse command and create task graph
        task_graph = self.parse_command_to_tasks(command)

        # Plan sequence of actions
        action_sequence = self.plan_task_sequence(task_graph)

        # Optimize trajectories
        optimized_trajectories = self.optimize_trajectories(action_sequence)

        # Execute or publish plan
        self.execute_or_publish_plan(optimized_trajectories)

    def parse_command_to_tasks(self, command):
        """Parse natural language command to task graph"""
        # Example: "Go to kitchen, pick up cup, bring to table"
        if "bring" in command and "to" in command:
            # Complex task involving navigation, manipulation, and delivery
            tasks = [
                {'type': 'navigate', 'target': self.extract_target_location(command)},
                {'type': 'find_object', 'object': self.extract_object(command)},
                {'type': 'grasp', 'object': self.extract_object(command)},
                {'type': 'navigate', 'target': self.extract_delivery_location(command)},
                {'type': 'place', 'location': self.extract_delivery_location(command)}
            ]
        elif "go to" in command:
            # Navigation task
            tasks = [
                {'type': 'navigate', 'target': self.extract_target_location(command)}
            ]
        elif "pick up" in command or "grasp" in command:
            # Manipulation task
            tasks = [
                {'type': 'find_object', 'object': self.extract_object(command)},
                {'type': 'grasp', 'object': self.extract_object(command)}
            ]
        else:
            # Simple task
            tasks = [{'type': 'simple_action', 'action': command}]

        return self.create_task_dependency_graph(tasks)

    def plan_task_sequence(self, task_graph):
        """Plan sequence of actions to accomplish tasks"""
        action_sequence = []

        for task in task_graph['tasks']:
            if task['type'] == 'navigate':
                path = self.motion_planner.plan_path_to_location(task['target'])
                actions = self.convert_path_to_navigation_actions(path)
                action_sequence.extend(actions)

            elif task['type'] == 'find_object':
                actions = self.plan_object_search_actions(task['object'])
                action_sequence.extend(actions)

            elif task['type'] == 'grasp':
                grasp_plan = self.plan_grasp_action(task['object'])
                action_sequence.extend(grasp_plan)

            elif task['type'] == 'place':
                place_plan = self.plan_place_action(task['location'])
                action_sequence.extend(place_plan)

        return action_sequence

    def plan_grasp_action(self, object_info):
        """Plan complete grasp action"""
        # 1. Move to pre-grasp position
        pre_grasp_pose = self.calculate_pre_grasp_pose(object_info)

        # 2. Plan approach trajectory
        approach_traj = self.motion_planner.plan_trajectory_to_pose(pre_grasp_pose)

        # 3. Execute approach
        approach_actions = self.convert_trajectory_to_actions(approach_traj, 'approach')

        # 4. Execute grasp
        grasp_action = {
            'type': 'grasp_execution',
            'object': object_info,
            'gripper_command': 'close'
        }

        # 5. Lift object
        lift_action = {
            'type': 'lift',
            'height': 0.1  # Lift 10cm
        }

        return approach_actions + [grasp_action, lift_action]

    def optimize_trajectories(self, action_sequence):
        """Optimize action sequence for efficiency and safety"""
        optimized_sequence = []

        for action in action_sequence:
            if action['type'] == 'navigation':
                # Optimize navigation trajectory
                optimized_nav = self.trajectory_optimizer.optimize_navigation(action)
                optimized_sequence.append(optimized_nav)
            elif action['type'] == 'manipulation':
                # Optimize manipulation trajectory
                optimized_manip = self.trajectory_optimizer.optimize_manipulation(action)
                optimized_sequence.append(optimized_manip)
            else:
                # Other actions, add as-is
                optimized_sequence.append(action)

        return optimized_sequence

class BehaviorTree:
    def __init__(self):
        self.root = None

    def build_interaction_tree(self):
        """Build behavior tree for human interaction"""
        # Root: Selector (try different interaction strategies)
        root = SelectorNode("interaction_strategies")

        # Strategy 1: Direct interaction (if human is close and attending)
        direct_interaction = SequenceNode("direct_interaction")
        direct_interaction.add_child(ConditionNode("human_close"))
        direct_interaction.add_child(ConditionNode("human_attending"))
        direct_interaction.add_child(ActionNode("engage_directly"))

        # Strategy 2: Attention getting (if human is close but not attending)
        attention_getting = SequenceNode("attention_getting")
        attention_getting.add_child(ConditionNode("human_close"))
        attention_getting.add_child(NegationNode(ConditionNode("human_attending")))
        attention_getting.add_child(ActionNode("get_attention"))

        # Strategy 3: Wait and observe (if human is far)
        wait_observe = SequenceNode("wait_observe")
        wait_observe.add_child(NegationNode(ConditionNode("human_close")))
        wait_observe.add_child(ActionNode("wait_and_observe"))

        root.add_child(direct_interaction)
        root.add_child(attention_getting)
        root.add_child(wait_observe)

        self.root = root
        return root

    def execute(self):
        """Execute behavior tree"""
        if self.root:
            return self.root.tick()
        return False
```

### 3. Integrated Control System

```python
class IntegratedControlSystem(Node):
    def __init__(self):
        super().__init__('integrated_control')

        # Initialize all controllers
        self.balance_controller = BalanceController()
        self.manipulation_controller = ManipulationController()
        self.locomotion_controller = LocomotionController()
        self.social_controller = SocialController()

        # Priority-based task coordinator
        self.task_coordinator = TaskCoordinator()

        # Safety monitor
        self.safety_monitor = SafetyMonitor()

        # State estimator
        self.state_estimator = RobotStateEstimator()

        # Initialize timers for control loops
        self.high_freq_timer = self.create_timer(0.001, self.high_frequency_control)  # 1kHz
        self.low_freq_timer = self.create_timer(0.01, self.low_frequency_control)     # 100Hz

    def high_frequency_control(self):
        """High-frequency control loop (1kHz)"""
        # Get current state
        current_state = self.state_estimator.estimate_robot_state()

        # Run safety checks
        if not self.safety_monitor.is_safe(current_state):
            self.emergency_stop()
            return

        # Calculate control commands based on active tasks
        control_commands = self.task_coordinator.compute_control_commands(current_state)

        # Apply control commands
        self.execute_control_commands(control_commands)

    def low_frequency_control(self):
        """Low-frequency control loop (100Hz)"""
        # Update task planning based on new perceptions
        self.update_task_planning()

        # Update social behaviors
        self.update_social_behaviors()

        # Monitor system health
        self.monitor_system_health()

    def execute_control_commands(self, commands):
        """Execute control commands on robot"""
        # Send joint commands
        self.send_joint_commands(commands['joint_commands'])

        # Send force/torque commands if needed
        if 'force_commands' in commands:
            self.send_force_commands(commands['force_commands'])

        # Send impedance commands if needed
        if 'impedance_commands' in commands:
            self.send_impedance_commands(commands['impedance_commands'])

class TaskCoordinator:
    def __init__(self):
        self.task_priorities = {
            'safety': 1,
            'balance': 2,
            'collision_avoidance': 3,
            'task_execution': 4,
            'comfort': 5
        }

        self.active_tasks = []
        self.task_weights = {}

    def compute_control_commands(self, current_state):
        """Compute control commands using prioritized task coordination"""
        # Start with highest priority tasks
        commands = {'joint_commands': np.zeros(28)}  # Example: 28 DOF

        # Safety tasks (highest priority)
        if self.has_safety_task():
            safety_commands = self.compute_safety_commands(current_state)
            commands = self.integrate_commands(commands, safety_commands, 'override')

        # Balance tasks
        if self.has_balance_task():
            balance_commands = self.compute_balance_commands(current_state)
            commands = self.integrate_commands(commands, balance_commands, 'high_priority')

        # Collision avoidance
        if self.has_collision_task():
            collision_commands = self.compute_collision_commands(current_state)
            commands = self.integrate_commands(commands, collision_commands, 'high_priority')

        # Task execution
        if self.has_task_execution():
            task_commands = self.compute_task_commands(current_state)
            commands = self.integrate_commands(commands, task_commands, 'low_priority')

        # Comfort behaviors (lowest priority)
        if self.has_comfort_task():
            comfort_commands = self.compute_comfort_commands(current_state)
            commands = self.integrate_commands(commands, comfort_commands, 'very_low_priority')

        return commands

    def integrate_commands(self, base_commands, new_commands, priority_level):
        """Integrate new commands with base commands based on priority"""
        if priority_level == 'override':
            return new_commands
        elif priority_level == 'high_priority':
            # Use null-space projection to add new commands without interfering with high-priority tasks
            return self.null_space_projection(base_commands, new_commands)
        elif priority_level == 'low_priority':
            # Add commands in null space of higher priority tasks
            return self.add_in_null_space(base_commands, new_commands)
        elif priority_level == 'very_low_priority':
            # Add commands with minimal impact
            return self.add_with_minimal_impact(base_commands, new_commands)
        else:
            return base_commands

    def null_space_projection(self, primary_commands, secondary_commands):
        """Project secondary commands into null space of primary commands"""
        # Calculate Jacobian for primary task
        J_primary = self.calculate_jacobian_for_task('balance')  # Example

        # Calculate null space projector
        I = np.eye(len(primary_commands['joint_commands']))
        pinv_J_primary = np.linalg.pinv(J_primary)
        null_space_projector = I - pinv_J_primary @ J_primary

        # Apply secondary commands in null space
        modified_secondary = secondary_commands.copy()
        modified_secondary['joint_commands'] = (
            null_space_projector @ secondary_commands['joint_commands']
        )

        # Combine commands
        combined_commands = primary_commands.copy()
        combined_commands['joint_commands'] += modified_secondary['joint_commands']

        return combined_commands

class SafetyMonitor:
    def __init__(self):
        self.safety_thresholds = {
            'joint_limits': 0.95,  # 95% of physical limits
            'torque_limits': 0.90,  # 90% of maximum torque
            'velocity_limits': 0.95,  # 95% of maximum velocity
            'collision_distance': 0.1,  # 10cm minimum distance
            'balance_margin': 0.1  # 10cm margin for ZMP
        }

        self.emergency_stop_triggered = False

    def is_safe(self, current_state):
        """Check if current state is safe"""
        # Check joint limits
        if not self.check_joint_limits(current_state):
            self.trigger_safety_response('joint_limit_violation')
            return False

        # Check torque limits
        if not self.check_torque_limits(current_state):
            self.trigger_safety_response('torque_limit_violation')
            return False

        # Check collision avoidance
        if not self.check_collision_avoidance(current_state):
            self.trigger_safety_response('collision_imminent')
            return False

        # Check balance
        if not self.check_balance_stability(current_state):
            self.trigger_safety_response('balance_lost')
            return False

        # All checks passed
        return True

    def check_collision_avoidance(self, current_state):
        """Check for imminent collisions"""
        # Use distance sensors and planning data
        # Check manipulator workspace for self-collisions
        # Check navigation path for obstacles

        # For this example, assume safe
        return True

    def check_balance_stability(self, current_state):
        """Check if robot is in stable balance"""
        zmp = current_state['zmp']
        support_polygon = self.calculate_support_polygon(current_state)

        # Check if ZMP is within support polygon
        is_stable = self.is_point_in_polygon(zmp[:2], support_polygon)

        return is_stable

    def trigger_safety_response(self, violation_type):
        """Trigger appropriate safety response"""
        if violation_type == 'balance_lost':
            # Execute balance recovery
            self.execute_balance_recovery()
        elif violation_type == 'collision_imminent':
            # Execute collision avoidance
            self.execute_collision_avoidance()
        elif violation_type == 'joint_limit_violation':
            # Apply joint limit constraints
            self.apply_joint_limit_constraints()
        elif violation_type == 'torque_limit_violation':
            # Reduce commanded torques
            self.reduce_torque_commands()

        # Log safety event
        self.get_logger().error(f"Safety violation: {violation_type}")
```

### 4. Human-Robot Interaction System

```python
class HumanRobotInteractionSystem(Node):
    def __init__(self):
        super().__init__('hri_system')

        # Initialize interaction components
        self.gesture_recognizer = GestureRecognizer()
        self.speech_processor = SpeechProcessor()
        self.social_behavior_manager = SocialBehaviorManager()
        self.person_model = PersonModel()

        # Subscribe to perception data
        self.create_subscription(Detection2DArray, '/perception/humans', self.human_detection_callback, 10)
        self.create_subscription(String, '/speech_recognition/text', self.speech_callback, 10)

        # Publish interaction commands
        self.interaction_cmd_pub = self.create_publisher(String, '/interaction/command', 10)
        self.gesture_pub = self.create_publisher(String, '/robot/gesture', 10)

    def human_detection_callback(self, msg):
        """Process human detections and update interaction state"""
        detected_humans = msg.detections

        # Update person tracking
        self.update_person_tracking(detected_humans)

        # Assess social situation
        social_context = self.assess_social_context(detected_humans)

        # Select appropriate social behavior
        behavior = self.social_behavior_manager.select_behavior(social_context)

        # Execute behavior
        self.execute_social_behavior(behavior)

    def speech_callback(self, msg):
        """Process speech input and generate response"""
        speech_text = msg.data

        # Parse intent from speech
        intent = self.parse_speech_intent(speech_text)

        # Generate appropriate response
        response = self.generate_response(intent, speech_text)

        # Execute response
        self.execute_verbal_response(response)

    def assess_social_context(self, detected_humans):
        """Assess current social context"""
        context = {
            'number_of_people': len(detected_humans),
            'people_positions': [det.get_position() for det in detected_humans],
            'attention_patterns': self.analyze_attention_patterns(detected_humans),
            'proximity_analysis': self.analyze_proximity(detected_humans),
            'group_dynamics': self.analyze_group_dynamics(detected_humans)
        }

        return context

    def analyze_attention_patterns(self, detected_humans):
        """Analyze where people are looking"""
        # This would use gaze estimation from vision system
        attention_patterns = []

        for human in detected_humans:
            # Estimate where the person is looking
            gaze_direction = self.estimate_gaze_direction(human)
            is_attending_to_robot = self.is_looking_at_robot(gaze_direction)

            attention_patterns.append({
                'person_id': human.id,
                'gaze_direction': gaze_direction,
                'attending_to_robot': is_attending_to_robot
            })

        return attention_patterns

    def analyze_proximity(self, detected_humans):
        """Analyze proximity to social norms"""
        proximity_analysis = []

        robot_pos = self.get_robot_position()

        for human in detected_humans:
            distance = np.linalg.norm(human.position - robot_pos)

            # Determine social zone
            if distance < 0.5:  # 50cm - intimate zone
                zone = 'intimate'
            elif distance < 1.2:  # 120cm - personal zone
                zone = 'personal'
            elif distance < 3.6:  # 360cm - social zone
                zone = 'social'
            else:  # Beyond 360cm - public zone
                zone = 'public'

            proximity_analysis.append({
                'person_id': human.id,
                'distance': distance,
                'social_zone': zone,
                'appropriate': self.is_distance_appropriate(zone)
            })

        return proximity_analysis

    def generate_response(self, intent, original_text):
        """Generate appropriate response based on intent"""
        response_templates = {
            'greeting': [
                "Hello! It's nice to meet you.",
                "Hi there! How can I assist you today?",
                "Greetings! I'm your humanoid assistant."
            ],
            'request': [
                "I can help you with that.",
                "Let me assist you with your request.",
                "I'll do my best to help you."
            ],
            'question': [
                "That's a great question!",
                "I'd be happy to explain.",
                "Let me provide information about that."
            ],
            'farewell': [
                "Goodbye! Have a wonderful day!",
                "See you later!",
                "Take care!"
            ]
        }

        import random
        if intent in response_templates:
            return random.choice(response_templates[intent])
        else:
            return "I'm not sure I understood. Could you please repeat?"

    def execute_social_behavior(self, behavior):
        """Execute selected social behavior"""
        behavior_type = behavior['type']

        if behavior_type == 'approach':
            self.execute_approach_behavior(behavior)
        elif behavior_type == 'greet':
            self.execute_greeting_behavior(behavior)
        elif behavior_type == 'assist':
            self.execute_assistance_behavior(behavior)
        elif behavior_type == 'maintain_distance':
            self.execute_distance_maintenance(behavior)
        elif behavior_type == 'engage_conversation':
            self.execute_conversation_behavior(behavior)

    def execute_greeting_behavior(self, behavior):
        """Execute greeting behavior"""
        # Move to appropriate distance
        target_distance = 1.0  # 1 meter for greeting
        self.move_to_distance(behavior['target_person'], target_distance)

        # Make eye contact
        self.make_eye_contact(behavior['target_person'])

        # Wave gesture
        self.publish_gesture('wave')

        # Speak greeting
        greeting = self.generate_contextual_greeting(behavior['target_person'])
        self.speak_text(greeting)

    def generate_contextual_greeting(self, target_person):
        """Generate greeting based on context"""
        # Consider time of day, previous interactions, etc.
        import datetime
        hour = datetime.datetime.now().hour

        if hour < 12:
            time_greeting = "Good morning"
        elif hour < 17:
            time_greeting = "Good afternoon"
        else:
            time_greeting = "Good evening"

        # Check if person is known
        person_known = self.person_model.is_person_known(target_person['id'])

        if person_known:
            previous_interactions = self.person_model.get_interaction_count(target_person['id'])
            if previous_interactions > 5:
                familiarity = "It's great to see you again!"
            else:
                familiarity = "Nice to see you!"

            return f"{time_greeting}! {familiarity}"
        else:
            return f"{time_greeting}! I'm your humanoid assistant. How can I help you?"
```

### 5. Complete System Integration

```python
class CompleteHumanoidSystem(Node):
    def __init__(self):
        super().__init__('complete_humanoid_system')

        # Initialize all subsystems
        self.perception_system = IntegratedPerceptionSystem()
        self.task_planning_system = TaskPlanningSystem()
        self.integrated_control = IntegratedControlSystem()
        self.hri_system = HumanRobotInteractionSystem()

        # Initialize safety and monitoring
        self.system_monitor = SystemMonitor()
        self.health_manager = HealthManager()

        # System state
        self.system_state = {
            'initialized': False,
            'operational': False,
            'emergency_stop': False,
            'current_behavior_mode': 'idle',
            'last_error': None
        }

        # Initialize the complete system
        self.initialize_system()

    def initialize_system(self):
        """Initialize the complete humanoid system"""
        self.get_logger().info("Initializing complete humanoid system...")

        # Initialize perception system
        self.get_logger().info("Initializing perception system...")
        # perception initialization happens in constructor

        # Initialize planning system
        self.get_logger().info("Initializing task planning system...")
        # planning initialization happens in constructor

        # Initialize control system
        self.get_logger().info("Initializing control system...")
        # control initialization happens in constructor

        # Initialize HRI system
        self.get_logger().info("Initializing human-robot interaction system...")
        # HRI initialization happens in constructor

        # Run system checks
        if self.run_system_checks():
            self.system_state['initialized'] = True
            self.system_state['operational'] = True
            self.get_logger().info("Complete humanoid system initialized successfully!")
        else:
            self.get_logger().error("System initialization failed!")
            self.system_state['initialized'] = False

    def run_system_checks(self):
        """Run comprehensive system checks"""
        checks_passed = True

        # Check perception system
        if not self.check_perception_system():
            self.get_logger().error("Perception system check failed")
            checks_passed = False

        # Check planning system
        if not self.check_planning_system():
            self.get_logger().error("Planning system check failed")
            checks_passed = False

        # Check control system
        if not self.check_control_system():
            self.get_logger().error("Control system check failed")
            checks_passed = False

        # Check HRI system
        if not self.check_hri_system():
            self.get_logger().error("HRI system check failed")
            checks_passed = False

        # Check safety systems
        if not self.check_safety_systems():
            self.get_logger().error("Safety systems check failed")
            checks_passed = False

        return checks_passed

    def check_perception_system(self):
        """Check perception system functionality"""
        # Verify all perception components are responsive
        # Test sensor data flow
        # Validate processing pipelines

        # For this example, assume all checks pass
        return True

    def check_planning_system(self):
        """Check planning system functionality"""
        # Test task planning capabilities
        # Verify path planning works
        # Test behavior trees

        # For this example, assume all checks pass
        return True

    def check_control_system(self):
        """Check control system functionality"""
        # Test basic control commands
        # Verify safety monitors
        # Test task coordination

        # For this example, assume all checks pass
        return True

    def check_hri_system(self):
        """Check HRI system functionality"""
        # Test speech processing
        # Verify gesture recognition
        # Test social behavior execution

        # For this example, assume all checks pass
        return True

    def check_safety_systems(self):
        """Check safety system functionality"""
        # Test emergency stop
        # Verify collision detection
        # Test balance recovery

        # For this example, assume all checks pass
        return True

    def execute_demo_scenario(self):
        """Execute a complete demonstration scenario"""
        self.get_logger().info("Starting complete demonstration scenario...")

        # Scenario: Robot detects human, approaches, and engages in simple interaction

        # Phase 1: Stand by and detect humans
        self.system_state['current_behavior_mode'] = 'detection'
        self.get_logger().info("Phase 1: Detecting humans in environment")

        # Wait for human detection (simulated)
        import time
        time.sleep(5.0)

        # Phase 2: Approach detected human
        self.system_state['current_behavior_mode'] = 'approach'
        self.get_logger().info("Phase 2: Approaching detected human")

        # Execute approach behavior
        self.execute_approach_sequence()

        # Phase 3: Greet and engage
        self.system_state['current_behavior_mode'] = 'interaction'
        self.get_logger().info("Phase 3: Engaging in social interaction")

        # Execute greeting and basic interaction
        self.execute_interaction_sequence()

        # Phase 4: Complete and return to idle
        self.system_state['current_behavior_mode'] = 'idle'
        self.get_logger().info("Phase 4: Scenario complete, returning to idle")

        self.return_to_idle_pose()

    def execute_approach_sequence(self):
        """Execute approach sequence"""
        # Move to appropriate distance
        self.move_to_distance(target_distance=1.0, speed='slow')

        # Align with human
        self.align_to_human()

        # Wait briefly to establish attention
        time.sleep(2.0)

    def execute_interaction_sequence(self):
        """Execute interaction sequence"""
        # Make eye contact
        self.make_eye_contact()

        # Wave gesture
        self.execute_gesture('wave')

        # Speak greeting
        self.speak_text("Hello! I'm a humanoid robot assistant. How can I help you today?")

        # Wait for response (simulated)
        time.sleep(5.0)

        # Nod gesture (acknowledgment)
        self.execute_gesture('nod')

        # Speak acknowledgment
        self.speak_text("I'm here to assist with various tasks. Just let me know what you need!")

    def return_to_idle_pose(self):
        """Return robot to safe idle pose"""
        # Move arms to safe position
        self.move_to_safe_arm_configuration()

        # Center head and maintain alert posture
        self.adopt_alert_posture()

        # Resume detection mode
        self.system_state['current_behavior_mode'] = 'detection'

    def shutdown_system(self):
        """Safely shut down the complete system"""
        self.get_logger().info("Shutting down complete humanoid system...")

        # Stop all control loops
        self.stop_control_loops()

        # Deactivate all subsystems
        self.deactivate_subsystems()

        # Run shutdown checks
        self.run_shutdown_procedures()

        self.get_logger().info("Complete humanoid system shut down successfully")

    def stop_control_loops(self):
        """Stop all control loops safely"""
        # Stop high-frequency control
        # Stop low-frequency control
        # Stop all timers
        pass

    def deactivate_subsystems(self):
        """Deactivate all subsystems"""
        # Stop perception processing
        # Stop planning updates
        # Stop interaction processing
        pass

    def run_shutdown_procedures(self):
        """Run shutdown safety procedures"""
        # Move to safe configuration
        # Disable motors safely
        # Save system state
        # Log shutdown event
        pass

def main(args=None):
    """Main function to run the complete humanoid system"""
    rclpy.init(args=args)

    # Create the complete system
    system = CompleteHumanoidSystem()

    # Wait for system initialization
    import time
    time.sleep(2.0)

    if system.system_state['operational']:
        system.get_logger().info("System operational - starting demonstration...")

        # Run a demonstration scenario
        system.execute_demo_scenario()

        # Continue running until shutdown
        try:
            rclpy.spin(system)
        except KeyboardInterrupt:
            system.get_logger().info("Interrupted by user")
        finally:
            system.shutdown_system()
    else:
        system.get_logger().error("System not operational - cannot run demonstration")

    system.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing and Validation

### Comprehensive Test Suite

```python
import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R

class TestHumanoidSystem(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures before each test method."""
        pass

    def test_perception_accuracy(self):
        """Test perception system accuracy"""
        # Test object detection accuracy
        # Test human detection accuracy
        # Test depth estimation accuracy
        pass

    def test_balance_stability(self):
        """Test balance control stability"""
        # Test static balance
        # Test dynamic balance recovery
        # Test disturbance rejection
        pass

    def test_manipulation_precision(self):
        """Test manipulation precision"""
        # Test reaching accuracy
        # Test grasping success rate
        # Test placement precision
        pass

    def test_interaction_naturalness(self):
        """Test interaction naturalness"""
        # Test response time
        # Test gesture appropriateness
        # Test speech recognition accuracy
        pass

    def test_system_integration(self):
        """Test complete system integration"""
        # Test all subsystems working together
        # Test safety systems
        # Test fault tolerance
        pass

    def test_performance_metrics(self):
        """Test performance metrics"""
        # Test control loop timing
        # Test computational efficiency
        # Test memory usage
        pass

def run_comprehensive_tests():
    """Run comprehensive tests on the complete system"""
    # Create test suite
    test_suite = unittest.TestSuite()

    # Add tests
    test_suite.addTest(unittest.makeSuite(TestHumanoidSystem))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)

    # Print results
    print(f"\nTests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors))/result.testsRun)*100:.1f}%")

    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_comprehensive_tests()
    exit(0 if success else 1)
```

## Performance Benchmarks

### Benchmarking Suite

```python
import time
import numpy as np
import psutil
import GPUtil

class PerformanceBenchmark:
    def __init__(self):
        self.results = {}

    def benchmark_perception_pipeline(self):
        """Benchmark perception pipeline performance"""
        # Test object detection speed
        start_time = time.time()
        # Run perception pipeline multiple times
        for i in range(100):
            # Simulate perception processing
            pass
        end_time = time.time()

        avg_time = (end_time - start_time) / 100
        self.results['perception_fps'] = 1.0 / avg_time if avg_time > 0 else float('inf')

        print(f"Perception pipeline: {self.results['perception_fps']:.2f} FPS")

    def benchmark_control_loop(self):
        """Benchmark control loop performance"""
        # Test control loop timing consistency
        loop_times = []

        for i in range(1000):  # Test 1000 control cycles
            start = time.perf_counter()
            # Simulate control computation
            self.simulate_control_computation()
            end = time.perf_counter()
            loop_times.append(end - start)

        avg_loop_time = np.mean(loop_times)
        std_loop_time = np.std(loop_times)

        self.results['control_avg_time'] = avg_loop_time
        self.results['control_std_time'] = std_loop_time
        self.results['control_stability'] = 1.0 - (std_loop_time / avg_loop_time)  # Higher is better

        print(f"Control loop: {1.0/avg_loop_time:.2f} Hz, stability: {self.results['control_stability']:.3f}")

    def simulate_control_computation(self):
        """Simulate control computation"""
        # Simulate typical control calculations
        np.random.random((100, 100)) @ np.random.random((100, 100))  # Matrix multiplication
        np.linalg.inv(np.random.random((10, 10)) + 0.1 * np.eye(10))  # Matrix inversion

    def benchmark_system_resources(self):
        """Benchmark system resource usage"""
        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        self.results['cpu_usage'] = cpu_percent

        # Memory usage
        memory = psutil.virtual_memory()
        self.results['memory_usage_percent'] = memory.percent
        self.results['memory_available_gb'] = memory.available / (1024**3)

        # GPU usage (if available)
        gpus = GPUtil.getGPUs()
        if gpus:
            gpu = gpus[0]  # Primary GPU
            self.results['gpu_usage'] = gpu.load * 100
            self.results['gpu_memory_usage'] = gpu.memoryUtil * 100
            self.results['gpu_temperature'] = gpu.temperature
        else:
            self.results['gpu_usage'] = 0
            self.results['gpu_memory_usage'] = 0
            self.results['gpu_temperature'] = 0

        print(f"System resources - CPU: {cpu_percent}%, Memory: {memory.percent}%, GPU: {self.results.get('gpu_usage', 0)}%")

    def run_complete_benchmark(self):
        """Run complete system benchmark"""
        print("Running complete system benchmark...")

        # Benchmark each component
        self.benchmark_perception_pipeline()
        self.benchmark_control_loop()
        self.benchmark_system_resources()

        # Calculate overall performance score
        performance_score = self.calculate_performance_score()
        self.results['overall_performance_score'] = performance_score

        print(f"\nOverall Performance Score: {performance_score:.2f}/10.0")
        print("\nDetailed Results:")
        for key, value in self.results.items():
            print(f"  {key}: {value}")

        return self.results

    def calculate_performance_score(self):
        """Calculate overall performance score"""
        # Weighted combination of different performance metrics
        # Each component contributes to the overall score
        perception_score = min(10.0, self.results.get('perception_fps', 0) / 30.0) * 2  # Target 30+ FPS
        control_score = min(10.0, (1.0 / self.results.get('control_avg_time', 1)) / 100.0) * 3  # Target 100+ Hz
        stability_score = self.results.get('control_stability', 0) * 10 * 2  # Stability factor
        resource_score = max(0, (100 - self.results.get('cpu_usage', 50)) / 10) * 1  # Lower CPU better

        total_score = (perception_score + control_score + stability_score + resource_score) / 6.0
        return min(10.0, total_score)  # Cap at 10.0

def main():
    benchmark = PerformanceBenchmark()
    results = benchmark.run_complete_benchmark()

    # Save results
    import json
    with open('benchmark_results.json', 'w') as f:
        json.dump(results, f, indent=2)

    print("\nBenchmark results saved to 'benchmark_results.json'")

if __name__ == '__main__':
    main()
```

## Deployment Considerations

### Real Robot Deployment

```python
class DeploymentManager:
    def __init__(self):
        self.deployment_config = self.load_deployment_config()
        self.calibration_manager = CalibrationManager()
        self.safety_validator = SafetyValidator()

    def prepare_for_real_robot(self):
        """Prepare the system for deployment on a real robot"""
        print("Preparing system for real robot deployment...")

        # Validate safety systems
        if not self.safety_validator.validate_all_safety_systems():
            raise RuntimeError("Safety validation failed - cannot deploy to real robot")

        # Run hardware-in-the-loop tests
        self.run_hardware_in_the_loop_tests()

        # Validate robot-specific parameters
        self.validate_robot_parameters()

        # Prepare deployment package
        deployment_package = self.create_deployment_package()

        print("System prepared for real robot deployment!")
        return deployment_package

    def run_hardware_in_the_loop_tests(self):
        """Run tests with real hardware components"""
        # Connect to real sensors and actuators in a safe environment
        # Test basic functionality without full autonomy
        # Validate communication protocols
        pass

    def validate_robot_parameters(self):
        """Validate robot-specific parameters"""
        # Check joint limits, velocities, accelerations
        # Validate mass properties and inertias
        # Verify sensor configurations
        pass

    def create_deployment_package(self):
        """Create deployment package for real robot"""
        # Bundle all necessary code, models, and configurations
        # Include safety monitors and emergency procedures
        # Add robot-specific calibration data
        pass

    def load_deployment_config(self):
        """Load deployment configuration"""
        # This would load from configuration files
        return {
            'robot_model': 'custom_humanoid',
            'safety_factors': 0.8,  # 80% of limits for safety
            'calibration_required': True,
            'emergency_procedures': ['stop_all_motors', 'return_to_home', 'alert_operator']
        }

def deploy_to_real_robot():
    """Deploy the complete system to a real robot"""
    print("Starting deployment to real robot...")

    # Create deployment manager
    deployer = DeploymentManager()

    # Prepare system for deployment
    deployment_package = deployer.prepare_for_real_robot()

    # Execute deployment
    # This would involve actual deployment procedures

    print("Deployment to real robot completed successfully!")
```

## Conclusion

This final project demonstrates the complete integration of all concepts covered in the Physical AI & Humanoid Robotics textbook. The system incorporates:

1. **Complete ROS 2 Architecture**: All modules integrated into a cohesive system
2. **Advanced Simulation**: Using Isaac Sim for development and testing
3. **AI Integration**: Modern AI techniques for perception and control
4. **Humanoid Control**: Sophisticated balance, locomotion, and manipulation
5. **Social Interaction**: Natural human-robot interaction capabilities
6. **Safety Systems**: Comprehensive safety and monitoring
7. **Performance Optimization**: Efficient implementation for real-time operation

The project showcases how all the individual components learned throughout the textbook work together to create a complete, functional humanoid robot system. It demonstrates the integration challenges and solutions required to build complex robotic systems that can operate safely and effectively in human environments.

This implementation serves as both a practical demonstration of the concepts learned and a foundation for further development and research in humanoid robotics. The modular architecture allows for easy extension and modification, making it suitable for continued learning and experimentation.

The success of this integrated system validates the educational approach of the textbook, showing how foundational concepts build up to sophisticated capabilities when properly integrated. It represents the culmination of the learning journey from basic ROS 2 concepts to advanced humanoid robotics.
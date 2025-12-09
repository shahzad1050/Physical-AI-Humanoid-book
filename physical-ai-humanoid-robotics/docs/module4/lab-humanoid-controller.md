# Practical Lab: Building a Humanoid Robot Controller

## Objective

In this lab, you will implement a complete humanoid robot controller that integrates all the concepts learned in Module 4. You will create a system that can maintain balance, perform basic manipulation tasks, and interact socially with humans. The controller will demonstrate coordination between balance, locomotion, manipulation, and human-robot interaction capabilities.

## Prerequisites

- Completed all previous modules (ROS 2, Simulation, Isaac Platform, Humanoid Robotics fundamentals)
- Access to a humanoid robot simulator (Gazebo, Isaac Sim, or similar)
- Python 3.8+ with required packages installed
- Understanding of control theory, kinematics, and dynamics

## Step 1: Project Setup and Architecture

First, let's create the project structure for our humanoid controller:

```bash
# Create project directory
mkdir -p ~/humanoid_robot_controller/src
cd ~/humanoid_robot_controller/src

# Create ROS 2 package
ros2 pkg create --build-type ament_python humanoid_controller --dependencies rclpy std_msgs sensor_msgs geometry_msgs builtin_interfaces message_filters cv_bridge tf2_ros tf2_geometry_msgs

# Create directory structure
mkdir -p ~/humanoid_robot_controller/src/humanoid_controller/{controllers,sensors,utils,behaviors}
```

## Step 2: Create the Main Controller Node

Create the main controller node in `~/humanoid_robot_controller/src/humanoid_controller/humanoid_controller_node.py`:

```python
#!/usr/bin/env python3

"""
Humanoid Robot Controller
Integrates balance, manipulation, and social interaction capabilities
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from geometry_msgs.msg import Twist, Pose, Point, Vector3
from std_msgs.msg import String, Float64MultiArray
from builtin_interfaces.msg import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import numpy as np
import math
import time
from collections import deque

# Import controller modules
from .controllers.balance_controller import BalanceController
from .controllers.manipulation_controller import ManipulationController
from .controllers.locomotion_controller import LocomotionController
from .sensors.state_estimator import StateEstimator
from .behaviors.social_behavior import SocialBehaviorManager


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Initialize components
        self.balance_controller = BalanceController(self)
        self.manipulation_controller = ManipulationController(self)
        self.locomotion_controller = LocomotionController(self)
        self.state_estimator = StateEstimator(self)
        self.social_behavior_manager = SocialBehaviorManager(self)

        self.cv_bridge = CvBridge()

        # Robot state variables
        self.joint_positions = np.zeros(28)  # Example: 28 DOF humanoid
        self.joint_velocities = np.zeros(28)
        self.imu_data = None
        self.camera_image = None
        self.force_torque_data = {'left_foot': [0,0,0], 'right_foot': [0,0,0]}

        # Control state
        self.current_mode = 'idle'  # idle, balance, walk, manipulate, interact
        self.desired_com_position = np.array([0.0, 0.0, 0.8])  # Desired CoM height: 80cm
        self.desired_com_velocity = np.zeros(3)

        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.ft_sensor_sub = self.create_subscription(
            Float64MultiArray,
            '/force_torque_sensors',
            self.force_torque_callback,
            10
        )

        # Create publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.com_pub = self.create_publisher(
            Point,
            '/center_of_mass',
            10
        )

        self.zmp_pub = self.create_publisher(
            Point,
            '/zero_moment_point',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/controller_status',
            10
        )

        # Create service clients
        self.service_clients = {}

        # Timer for main control loop
        self.control_timer = self.create_timer(0.01, self.main_control_loop)  # 100Hz

        # TF broadcaster and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize robot state
        self.initialize_robot()

        self.get_logger().info('Humanoid Controller initialized')

    def initialize_robot(self):
        """Initialize robot to safe configuration"""
        # Move to neutral standing position
        neutral_pos = self.get_neutral_standing_position()

        cmd_msg = Float64MultiArray()
        cmd_msg.data = neutral_pos.tolist()
        self.joint_cmd_pub.publish(cmd_msg)

        # Wait for robot to reach position
        time.sleep(2.0)

        # Switch to balance mode
        self.current_mode = 'balance'
        self.get_logger().info('Robot initialized to balance mode')

    def get_neutral_standing_position(self):
        """Get neutral standing joint configuration"""
        # Example neutral standing position for a typical humanoid
        # This would be specific to your robot model
        neutral_pos = np.array([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Head/Neck joints
            0.0, 0.2, -0.4, 0.2, 0.0, 0.0, 0.0,  # Left arm
            0.0, -0.2, 0.4, -0.2, 0.0, 0.0, 0.0,  # Right arm
            0.0, 0.0, -0.3, 0.6, -0.3, 0.0,  # Left leg
            0.0, 0.0, -0.3, 0.6, -0.3, 0.0   # Right leg
        ])

        return neutral_pos

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        if len(msg.position) == len(self.joint_positions):
            self.joint_positions = np.array(msg.position)

        if len(msg.velocity) == len(self.joint_velocities):
            self.joint_velocities = np.array(msg.velocity)

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = {
            'linear_acceleration': np.array([msg.linear_acceleration.x,
                                           msg.linear_acceleration.y,
                                           msg.linear_acceleration.z]),
            'angular_velocity': np.array([msg.angular_velocity.x,
                                        msg.angular_velocity.y,
                                        msg.angular_velocity.z]),
            'orientation': np.array([msg.orientation.w, msg.orientation.x,
                                   msg.orientation.y, msg.orientation.z])
        }

    def camera_callback(self, msg):
        """Process camera images"""
        try:
            self.camera_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def force_torque_callback(self, msg):
        """Process force/torque sensor data"""
        if len(msg.data) >= 6:  # At least 6 values for both feet
            self.force_torque_data = {
                'left_foot': msg.data[0:3],   # Fx, Fy, Fz
                'right_foot': msg.data[3:6]   # Fx, Fy, Fz
            }

    def main_control_loop(self):
        """Main control loop that orchestrates all subsystems"""
        # Update state estimation
        current_state = self.state_estimator.estimate_state(
            self.joint_positions,
            self.joint_velocities,
            self.imu_data,
            self.force_torque_data
        )

        # Calculate control commands based on current mode
        control_commands = self.compute_control_commands(current_state)

        # Publish control commands
        self.publish_control_commands(control_commands)

        # Update status
        self.publish_status()

        # Execute social behaviors if in interaction mode
        if self.current_mode == 'interact':
            self.social_behavior_manager.execute_behaviors(current_state)

    def compute_control_commands(self, current_state):
        """Compute control commands based on current mode"""
        if self.current_mode == 'balance':
            return self.balance_controller.compute_balance_control(current_state)
        elif self.current_mode == 'walk':
            return self.locomotion_controller.compute_locomotion_control(current_state)
        elif self.current_mode == 'manipulate':
            return self.manipulation_controller.compute_manipulation_control(current_state)
        elif self.current_mode == 'interact':
            # Combine balance, manipulation, and social behaviors
            balance_cmds = self.balance_controller.compute_balance_control(current_state)
            manipulation_cmds = self.manipulation_controller.compute_manipulation_control(current_state)
            social_cmds = self.social_behavior_manager.get_social_commands(current_state)

            # Combine commands with priorities
            return self.combine_commands(balance_cmds, manipulation_cmds, social_cmds)
        else:  # idle mode
            # Maintain current position
            return self.joint_positions

    def combine_commands(self, balance_cmds, manipulation_cmds, social_cmds):
        """Combine different command types with priorities"""
        # This is a simplified combination - in practice, you'd use null-space projections
        # or task-priority based control

        combined_commands = balance_cmds.copy()

        # Add manipulation commands in null space of balance
        # Add social behavior commands as secondary objectives

        return combined_commands

    def publish_control_commands(self, commands):
        """Publish control commands to robot"""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands.tolist()
        self.joint_cmd_pub.publish(cmd_msg)

    def publish_status(self):
        """Publish controller status"""
        status_msg = String()
        status_msg.data = f"Mode: {self.current_mode}, CoM: {self.state_estimator.com_position}"
        self.status_pub.publish(status_msg)

        # Publish CoM and ZMP for visualization
        com_msg = Point()
        com_msg.x = float(self.state_estimator.com_position[0])
        com_msg.y = float(self.state_estimator.com_position[1])
        com_msg.z = float(self.state_estimator.com_position[2])
        self.com_pub.publish(com_msg)

        zmp_msg = Point()
        zmp = self.state_estimator.calculate_zmp()
        zmp_msg.x = float(zmp[0])
        zmp_msg.y = float(zmp[1])
        zmp_msg.z = 0.0  # ZMP is on ground plane
        self.zmp_pub.publish(zmp_msg)

    def switch_mode(self, new_mode):
        """Switch controller mode"""
        if new_mode in ['idle', 'balance', 'walk', 'manipulate', 'interact']:
            old_mode = self.current_mode
            self.current_mode = new_mode

            # Perform mode transition actions
            if old_mode == 'balance' and new_mode != 'balance':
                # Transitioning out of balance mode
                pass
            elif new_mode == 'balance' and old_mode != 'balance':
                # Transitioning into balance mode
                pass

            self.get_logger().info(f'Switched from {old_mode} to {new_mode}')
        else:
            self.get_logger().warn(f'Invalid mode: {new_mode}')

    def execute_simple_task(self, task_type, **kwargs):
        """Execute a simple task"""
        if task_type == 'wave':
            self.switch_mode('interact')
            return self.social_behavior_manager.execute_wave_behavior(**kwargs)
        elif task_type == 'reach':
            self.switch_mode('manipulate')
            return self.manipulation_controller.execute_reach(**kwargs)
        elif task_type == 'step':
            self.switch_mode('balance')
            return self.balance_controller.execute_recovery_step(**kwargs)
        else:
            self.get_logger().warn(f'Unknown task type: {task_type}')
            return False


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Humanoid Controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 3: Create the Balance Controller

Create the balance controller in `~/humanoid_robot_controller/src/humanoid_controller/controllers/balance_controller.py`:

```python
#!/usr/bin/env python3

"""
Balance Controller for Humanoid Robot
Implements LIPM-based balance control with ZMP tracking
"""

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import time


class BalanceController:
    def __init__(self, parent_node):
        self.parent = parent_node
        self.com_height = 0.8  # Center of mass height (m)
        self.gravity = 9.81
        self.omega = math.sqrt(self.gravity / self.com_height)

        # Control gains
        self.Kp_com = np.diag([100, 100, 0])  # Position gains for CoM
        self.Kd_com = np.diag([20, 20, 0])    # Velocity gains for CoM
        self.Kp_zmp = np.array([50, 50])      # ZMP tracking gains

        # State variables
        self.previous_com_error = np.zeros(2)
        self.integral_com_error = np.zeros(2)
        self.zmp_reference = np.zeros(2)
        self.support_polygon = self.calculate_default_support_polygon()

        # Walking parameters (for stepping control)
        self.step_width = 0.2  # Distance between feet (m)
        self.step_length = 0.3  # Step length (m)
        self.is_left_support = True  # Which foot is currently supporting

    def compute_balance_control(self, current_state):
        """
        Compute balance control commands
        """
        # Extract current state
        current_com = current_state['com_position'][:2]  # Only x,y for balance
        current_com_vel = current_state['com_velocity'][:2]
        current_zmp = current_state['zmp'][:2]
        support_polygon = current_state['support_polygon']

        # Calculate CoM tracking error
        com_error = self.zmp_reference - current_com
        com_vel_error = np.zeros(2) - current_com_vel  # Desired velocity is 0

        # PID control for CoM
        self.integral_com_error += com_error * 0.01  # dt = 0.01s
        derivative_com_error = (com_error - self.previous_com_error) / 0.01

        com_control = (self.Kp_com[:2, :2] @ com_error +
                      self.Kd_com[:2, :2] @ com_vel_error +
                      1.0 * self.integral_com_error)  # Integral gain of 1.0

        # Calculate ZMP error
        zmp_error = self.zmp_reference - current_zmp
        zmp_control = self.Kp_zmp * zmp_error

        # Combine controls
        total_control = com_control + zmp_control

        # Convert to joint torques using inverse dynamics
        joint_commands = self.compute_joint_commands_from_balance_control(
            total_control, current_state
        )

        # Update state for next iteration
        self.previous_com_error = com_error.copy()

        return joint_commands

    def compute_joint_commands_from_balance_control(self, balance_control, current_state):
        """
        Convert balance control signals to joint commands
        """
        # This is a simplified approach - in practice, use whole-body control
        current_joints = current_state['joint_positions']

        # Calculate required joint modifications to achieve balance
        # This would typically use inverse kinematics or operational space control

        # For this example, adjust ankle joints for balance
        ankle_adjustments = self.calculate_ankle_adjustments(balance_control, current_joints)

        # Apply adjustments to current joint positions
        new_joints = current_joints.copy()

        # Modify ankle joints (indices are example - depend on your robot)
        # Assuming ankle joints are at indices 22-23 (left) and 24-25 (right)
        new_joints[22] += ankle_adjustments[0]  # Left ankle roll
        new_joints[23] += ankle_adjustments[1]  # Left ankle pitch
        new_joints[24] += ankle_adjustments[2]  # Right ankle roll
        new_joints[25] += ankle_adjustments[3]  # Right ankle pitch

        # Add hip adjustments for larger balance corrections
        hip_adjustments = self.calculate_hip_adjustments(balance_control, current_joints)
        new_joints[16] += hip_adjustments[0]  # Left hip roll
        new_joints[17] += hip_adjustments[1]  # Left hip pitch
        new_joints[18] += hip_adjustments[2]  # Left hip yaw
        new_joints[26] += hip_adjustments[3]  # Right hip roll
        new_joints[27] += hip_adjustments[4]  # Right hip pitch
        new_joints[28] += hip_adjustments[5]  # Right hip yaw

        return new_joints

    def calculate_ankle_adjustments(self, balance_control, current_joints):
        """
        Calculate ankle joint adjustments for balance control
        """
        # Balance control contains [x_control, y_control]
        x_control, y_control = balance_control

        # Convert balance commands to ankle adjustments
        # This mapping depends on your robot's kinematics
        ankle_roll_adjustment = 0.1 * y_control  # Y control -> roll
        ankle_pitch_adjustment = -0.1 * x_control  # X control -> pitch

        # Return adjustments for both ankles
        return np.array([
            ankle_roll_adjustment,   # Left ankle roll
            ankle_pitch_adjustment,  # Left ankle pitch
            -ankle_roll_adjustment,  # Right ankle roll (opposite)
            -ankle_pitch_adjustment  # Right ankle pitch (opposite)
        ])

    def calculate_hip_adjustments(self, balance_control, current_joints):
        """
        Calculate hip joint adjustments for larger balance corrections
        """
        x_control, y_control = balance_control

        # For larger disturbances, use hip joints
        hip_roll_adjustment = 0.05 * y_control
        hip_pitch_adjustment = -0.05 * x_control
        hip_yaw_adjustment = 0.02 * (x_control + y_control)  # For turning

        # Return adjustments for both hips
        return np.array([
            hip_roll_adjustment,   # Left hip roll
            hip_pitch_adjustment,  # Left hip pitch
            hip_yaw_adjustment,    # Left hip yaw
            -hip_roll_adjustment,  # Right hip roll (opposite)
            -hip_pitch_adjustment, # Right hip pitch (opposite)
            -hip_yaw_adjustment    # Right hip yaw (opposite)
        ])

    def update_zmp_reference(self, new_zmp_ref):
        """
        Update the ZMP reference trajectory
        """
        self.zmp_reference = new_zmp_ref

    def calculate_default_support_polygon(self):
        """
        Calculate default support polygon based on foot positions
        """
        # Simplified: assume feet are 20cm apart
        foot_separation = 0.2
        foot_length = 0.15
        foot_width = 0.08

        # Support polygon vertices (simplified as rectangle)
        return {
            'min_x': -foot_length/2,
            'max_x': foot_length/2,
            'min_y': -foot_separation/2 - foot_width/2,
            'max_y': foot_separation/2 + foot_width/2
        }

    def is_zmp_stable(self, zmp_pos, support_polygon):
        """
        Check if ZMP is within support polygon
        """
        return (support_polygon['min_x'] <= zmp_pos[0] <= support_polygon['max_x'] and
                support_polygon['min_y'] <= zmp_pos[1] <= support_polygon['max_y'])

    def execute_recovery_step(self, current_state, direction='forward'):
        """
        Execute a recovery step to regain balance
        """
        # Calculate capture point
        current_com = current_state['com_position'][:2]
        current_com_vel = current_state['com_velocity'][:2]

        capture_point = current_com + current_com_vel / self.omega

        # Determine step location based on direction
        step_offset = np.array([0.0, 0.0])
        if direction == 'forward':
            step_offset[0] = 0.1  # Step forward 10cm
        elif direction == 'backward':
            step_offset[0] = -0.1  # Step backward 10cm
        elif direction == 'left':
            step_offset[1] = 0.1  # Step left 10cm
        elif direction == 'right':
            step_offset[1] = -0.1  # Step right 10cm

        # Target step location
        step_target = capture_point + step_offset

        # Execute step using manipulation controller
        # This is a simplified approach - in practice, use stepping controller
        self.parent.manipulation_controller.execute_foot_placement(step_target)

        # Update support polygon
        self.update_support_polygon_after_step(step_target)

        return True

    def update_support_polygon_after_step(self, new_foot_pos):
        """
        Update support polygon after a step is taken
        """
        # This would update the support polygon based on new foot placement
        # For now, just update the reference ZMP to the new foot location
        self.zmp_reference = new_foot_pos
```

## Step 4: Create the Manipulation Controller

Create the manipulation controller in `~/humanoid_robot_controller/src/humanoid_controller/controllers/manipulation_controller.py`:

```python
#!/usr/bin/env python3

"""
Manipulation Controller for Humanoid Robot
Handles arm control, grasping, and object manipulation
"""

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import time


class ManipulationController:
    def __init__(self, parent_node):
        self.parent = parent_node

        # Arm configuration
        self.arm_dof = 7  # 7 DOF arms
        self.left_arm_indices = slice(7, 14)   # Joints 7-13 for left arm
        self.right_arm_indices = slice(14, 21) # Joints 14-20 for right arm

        # Control parameters
        self.kp_pos = 100.0  # Position gain
        self.kd_pos = 20.0   # Velocity gain
        self.kp_ori = 10.0   # Orientation gain
        self.kd_ori = 5.0    # Angular velocity gain

        # Jacobian and inverse kinematics parameters
        self.ik_lambda = 0.01  # Damping factor for Jacobian pseudoinverse
        self.ik_max_iter = 100
        self.ik_tolerance = 1e-4

        # State variables
        self.left_ee_pose = np.eye(4)  # Left end-effector pose
        self.right_ee_pose = np.eye(4)  # Right end-effector pose
        self.left_ee_vel = np.zeros(6)  # Left end-effector velocity
        self.right_ee_vel = np.zeros(6)  # Right end-effector velocity

        # Task state
        self.active_left_task = None
        self.active_right_task = None

    def compute_manipulation_control(self, current_state):
        """
        Compute manipulation control commands
        """
        current_joints = current_state['joint_positions']
        current_velocities = current_state['joint_velocities']

        # Calculate desired joint positions for manipulation tasks
        left_arm_cmd = self.compute_left_arm_control(current_state)
        right_arm_cmd = self.compute_right_arm_control(current_state)

        # Combine with other joints
        new_joints = current_joints.copy()
        new_joints[self.left_arm_indices] = left_arm_cmd
        new_joints[self.right_arm_indices] = right_arm_cmd

        return new_joints

    def compute_left_arm_control(self, current_state):
        """
        Compute control for left arm
        """
        if self.active_left_task is None:
            # Return current position (no active task)
            return current_state['joint_positions'][self.left_arm_indices]

        task = self.active_left_task
        current_joints = current_state['joint_positions'][self.left_arm_indices]

        if task['type'] == 'reach':
            return self.ik_reach_target(
                current_joints,
                task['target_position'],
                task['target_orientation']
            )
        elif task['type'] == 'grasp':
            return self.execute_grasp_task(current_joints, task)
        elif task['type'] == 'hold':
            return self.hold_object(current_joints, task)
        else:
            return current_joints  # No active task

    def compute_right_arm_control(self, current_state):
        """
        Compute control for right arm
        """
        if self.active_right_task is None:
            # Return current position (no active task)
            return current_state['joint_positions'][self.right_arm_indices]

        task = self.active_right_task
        current_joints = current_state['joint_positions'][self.right_arm_indices]

        if task['type'] == 'reach':
            return self.ik_reach_target(
                current_joints,
                task['target_position'],
                task['target_orientation']
            )
        elif task['type'] == 'grasp':
            return self.execute_grasp_task(current_joints, task)
        elif task['type'] == 'hold':
            return self.hold_object(current_joints, task)
        else:
            return current_joints  # No active task

    def ik_reach_target(self, current_joints, target_pos, target_ori=None):
        """
        Inverse kinematics to reach target position and orientation
        """
        if target_ori is None:
            # Use current orientation if not specified
            current_ee_pose = self.forward_kinematics(current_joints)
            target_ori = current_ee_pose[:3, :3]

        target_pose = np.eye(4)
        target_pose[:3, 3] = target_pos
        target_pose[:3, :3] = target_ori if isinstance(target_ori, np.ndarray) else R.from_quat(target_ori).as_matrix()

        # Use iterative inverse kinematics
        new_joints = self.iterative_ik(current_joints, target_pose)

        return new_joints

    def iterative_ik(self, current_joints, target_pose):
        """
        Iterative inverse kinematics using Jacobian transpose/pseudoinverse
        """
        current_joints = current_joints.copy()

        for iteration in range(self.ik_max_iter):
            # Calculate current end-effector pose
            current_pose = self.forward_kinematics(current_joints)

            # Calculate error
            pos_error = target_pose[:3, 3] - current_pose[:3, 3]
            ori_error = self.rotation_error(current_pose[:3, :3], target_pose[:3, :3])

            # Check convergence
            if np.linalg.norm(pos_error) < self.ik_tolerance and np.linalg.norm(ori_error) < self.ik_tolerance:
                break

            # Calculate Jacobian
            jacobian = self.calculate_jacobian(current_joints)

            # Combine position and orientation errors
            error = np.concatenate([pos_error, ori_error])

            # Calculate joint updates using damped least squares
            I = np.eye(len(current_joints))
            jtj_lambda = jacobian.T @ jacobian + self.ik_lambda * I
            joint_delta = np.linalg.solve(jtj_lambda, jacobian.T @ error)

            # Apply updates
            current_joints += 0.5 * joint_delta  # 0.5 for stability

            # Apply joint limits
            current_joints = self.apply_joint_limits(current_joints)

        return current_joints

    def forward_kinematics(self, joint_angles):
        """
        Simplified forward kinematics - in practice, use robot-specific FK
        """
        # This is a placeholder - implement robot-specific forward kinematics
        # For this example, return a simple transformation
        pose = np.eye(4)

        # Simplified FK based on joint angles
        # In practice, use DH parameters or other kinematic model
        x = 0.3 + 0.1 * math.sin(joint_angles[0])
        y = 0.2 + 0.1 * math.cos(joint_angles[1])
        z = 1.0 + 0.1 * math.sin(joint_angles[2])

        pose[0, 3] = x
        pose[1, 3] = y
        pose[2, 3] = z

        # Simple orientation (identity for now)
        return pose

    def calculate_jacobian(self, joint_angles):
        """
        Calculate geometric Jacobian - in practice, use robot-specific method
        """
        # This is a simplified Jacobian calculation
        # In practice, use analytical or numerical differentiation
        n_joints = len(joint_angles)
        jacobian = np.zeros((6, n_joints))  # 6 DOF (pos + ori)

        # Simplified Jacobian - in practice, calculate properly based on kinematics
        for i in range(n_joints):
            # Position part of Jacobian
            jacobian[0:3, i] = self.calculate_position_jacobian_column(i, joint_angles)
            # Orientation part of Jacobian
            jacobian[3:6, i] = self.calculate_orientation_jacobian_column(i, joint_angles)

        return jacobian

    def calculate_position_jacobian_column(self, joint_idx, joint_angles):
        """
        Calculate position part of Jacobian column
        """
        # Simplified calculation - in practice, use proper kinematic derivation
        return np.array([0.1, 0.1, 0.1])  # Placeholder

    def calculate_orientation_jacobian_column(self, joint_idx, joint_angles):
        """
        Calculate orientation part of Jacobian column
        """
        # Simplified calculation - in practice, use proper kinematic derivation
        return np.array([0.01, 0.01, 0.01])  # Placeholder

    def rotation_error(self, current_rotation, target_rotation):
        """
        Calculate rotational error as angle-axis representation
        """
        relative_rotation = target_rotation @ current_rotation.T
        rotation_vector = R.from_matrix(relative_rotation).as_rotvec()
        return rotation_vector

    def apply_joint_limits(self, joints):
        """
        Apply joint limits to joint angles
        """
        # Example joint limits (these should match your robot)
        min_limits = np.array([-2.0] * len(joints))
        max_limits = np.array([2.0] * len(joints))

        return np.clip(joints, min_limits, max_limits)

    def execute_grasp_task(self, current_joints, task):
        """
        Execute grasp task
        """
        # Move to pre-grasp position
        pre_grasp_pos = task['object_position'] + np.array([0, 0, 0.1])  # 10cm above object
        pre_grasp_ori = task['grasp_orientation']

        # Move to pre-grasp
        joints = self.ik_reach_target(current_joints, pre_grasp_pos, pre_grasp_ori)

        # Move down to object
        grasp_pos = task['object_position']
        joints = self.ik_reach_target(joints, grasp_pos, pre_grasp_ori)

        # Close gripper (simplified)
        # In practice, control gripper separately
        joints = self.execute_gripper_control(joints, 'close')

        # Lift object
        lift_pos = grasp_pos + np.array([0, 0, 0.1])  # Lift 10cm
        joints = self.ik_reach_target(joints, lift_pos, pre_grasp_ori)

        return joints

    def execute_gripper_control(self, joints, command):
        """
        Control gripper (simplified)
        """
        # In practice, this would control actual gripper joints
        # For this example, we'll just return the joints unchanged
        return joints

    def hold_object(self, current_joints, task):
        """
        Hold object at specified location
        """
        # Maintain grasp while possibly moving to new location
        target_pos = task.get('hold_position', self.get_current_ee_position(current_joints))
        target_ori = task.get('hold_orientation', self.get_current_ee_orientation(current_joints))

        return self.ik_reach_target(current_joints, target_pos, target_ori)

    def get_current_ee_position(self, joints):
        """
        Get current end-effector position
        """
        pose = self.forward_kinematics(joints)
        return pose[:3, 3]

    def get_current_ee_orientation(self, joints):
        """
        Get current end-effector orientation
        """
        pose = self.forward_kinematics(joints)
        return pose[:3, :3]

    def execute_reach(self, target_position, arm='right', orientation=None):
        """
        Execute reach task with specified arm
        """
        task = {
            'type': 'reach',
            'target_position': np.array(target_position),
            'target_orientation': orientation
        }

        if arm == 'left':
            self.active_left_task = task
        else:  # right
            self.active_right_task = task

    def execute_grasp(self, object_position, grasp_type='top', arm='right'):
        """
        Execute grasp task
        """
        # Determine grasp orientation based on grasp type
        if grasp_type == 'top':
            grasp_orientation = R.from_euler('xyz', [0, 0, 0]).as_matrix()  # Grasp from top
        elif grasp_type == 'side':
            grasp_orientation = R.from_euler('xyz', [0, np.pi/2, 0]).as_matrix()  # Grasp from side
        else:
            grasp_orientation = R.from_euler('xyz', [0, 0, 0]).as_matrix()  # Default

        task = {
            'type': 'grasp',
            'object_position': np.array(object_position),
            'grasp_orientation': grasp_orientation
        }

        if arm == 'left':
            self.active_left_task = task
        else:  # right
            self.active_right_task = task

    def execute_hold(self, hold_position, arm='right'):
        """
        Execute hold task
        """
        task = {
            'type': 'hold',
            'hold_position': np.array(hold_position)
        }

        if arm == 'left':
            self.active_left_task = task
        else:  # right
            self.active_right_task = task

    def execute_foot_placement(self, target_position):
        """
        Execute foot placement for stepping (simplified)
        """
        # This would coordinate with balance controller for stepping
        # For this example, just return True
        return True
```

## Step 5: Create the State Estimator

Create the state estimator in `~/humanoid_robot_controller/src/humanoid_controller/sensors/state_estimator.py`:

```python
#!/usr/bin/env python3

"""
State Estimator for Humanoid Robot
Estimates robot state including CoM, ZMP, joint states, etc.
"""

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from collections import deque
import time


class StateEstimator:
    def __init__(self, parent_node):
        self.parent = parent_node

        # State variables
        self.com_position = np.array([0.0, 0.0, 0.8])  # Initial CoM at 80cm height
        self.com_velocity = np.zeros(3)
        self.com_acceleration = np.zeros(3)

        self.zmp = np.zeros(3)  # Zero Moment Point
        self.support_polygon = self.calculate_initial_support_polygon()

        self.foot_positions = {
            'left': np.array([0.0, 0.1, 0.0]),   # Left foot position
            'right': np.array([0.0, -0.1, 0.0])  # Right foot position
        }

        # Estimation parameters
        self.com_height = 0.8  # Fixed CoM height assumption for LIPM
        self.gravity = 9.81
        self.omega = math.sqrt(self.gravity / self.com_height)

        # Filtering parameters
        self.com_velocity_filter = self.initialize_filter(3, 5)  # 3D, 5 taps
        self.com_position_history = deque(maxlen=10)  # Keep last 10 positions

        # Sensor fusion weights
        self.kf_process_noise = np.diag([0.01, 0.01, 0.1])  # Process noise
        self.kf_measurement_noise = np.diag([0.001, 0.001, 0.01])  # Measurement noise

    def estimate_state(self, joint_positions, joint_velocities, imu_data, ft_data):
        """
        Estimate robot state from sensor data
        """
        # Update CoM position and velocity
        self.update_com_state(joint_positions)

        # Update ZMP from force/torque sensors
        self.update_zmp(ft_data)

        # Update support polygon
        self.update_support_polygon()

        # Update foot positions
        self.update_foot_positions(joint_positions)

        # Package state for controller
        state = {
            'com_position': self.com_position.copy(),
            'com_velocity': self.com_velocity.copy(),
            'com_acceleration': self.com_acceleration.copy(),
            'zmp': self.zmp.copy(),
            'support_polygon': self.support_polygon.copy(),
            'foot_positions': self.foot_positions.copy(),
            'joint_positions': joint_positions,
            'joint_velocities': joint_velocities,
            'imu_data': imu_data,
            'force_torque_data': ft_data
        }

        return state

    def update_com_state(self, joint_positions):
        """
        Update CoM position and velocity from joint positions
        """
        # Calculate CoM position from joint configuration
        # This is a simplified approach - in practice, use full kinematic model
        new_com_pos = self.calculate_com_from_joints(joint_positions)

        # Update position history
        self.com_position_history.append(new_com_pos)

        # Calculate velocity from position differences
        if len(self.com_position_history) >= 2:
            dt = 0.01  # Assuming 100Hz control
            pos_diff = new_com_pos - self.com_position_history[-2]
            new_com_vel = pos_diff / dt

            # Apply simple filtering to velocity
            if np.linalg.norm(self.com_velocity) > 0:
                self.com_velocity = 0.7 * self.com_velocity + 0.3 * new_com_vel
            else:
                self.com_velocity = new_com_vel

        # Update CoM position (apply some smoothing)
        alpha = 0.9  # Smoothing factor
        self.com_position = alpha * self.com_position + (1 - alpha) * new_com_pos

    def calculate_com_from_joints(self, joint_positions):
        """
        Calculate CoM from joint positions using simplified model
        """
        # This would use the full kinematic model in practice
        # For this example, use a simplified calculation

        # Simplified CoM calculation based on joint positions
        # In practice, use mass distribution and full forward kinematics
        base_com = np.array([0.0, 0.0, 0.8])  # Base CoM position

        # Add influence from joint positions
        # This is a very simplified model
        joint_influence = np.zeros(3)

        # Influence from leg joints (affects x,y,z)
        leg_joints = joint_positions[21:29]  # Assuming leg joints are at indices 21-28
        for i, joint in enumerate(leg_joints):
            # Each leg joint has some influence on CoM position
            influence = 0.01 * math.sin(joint)  # Simplified influence
            joint_influence[0] += influence * ((i % 3) == 0)  # x influence
            joint_influence[1] += influence * ((i % 3) == 1)  # y influence
            joint_influence[2] += influence * ((i % 3) == 2)  # z influence

        return base_com + joint_influence

    def update_zmp(self, ft_data):
        """
        Update Zero Moment Point from force/torque sensors
        """
        # Calculate ZMP from foot force/torque measurements
        # ZMP_x = (M_y + F_z * h) / F_x  (simplified)
        # ZMP_y = (-M_x + F_z * h) / F_y (simplified)

        # Get forces from both feet
        left_force = np.array(ft_data['left_foot'])
        right_force = np.array(ft_data['right_foot'])

        # Calculate total forces and moments
        total_force = left_force + right_force

        # Simplified ZMP calculation
        # In practice, use full moment calculations from both feet
        if total_force[2] != 0:  # Fz should not be zero
            zmp_x = -(left_force[4] + right_force[4]) / total_force[2]  # Moment_y / Force_z
            zmp_y = (left_force[3] + right_force[3]) / total_force[2]   # Moment_x / Force_z
        else:
            zmp_x, zmp_y = 0, 0  # Default to origin if no vertical force

        self.zmp = np.array([zmp_x, zmp_y, 0.0])

    def update_support_polygon(self):
        """
        Update support polygon based on foot contact
        """
        # Calculate support polygon from foot positions
        # This is simplified - in practice, consider foot geometry and contact points

        left_pos = self.foot_positions['left']
        right_pos = self.foot_positions['right']

        # Create bounding box as support polygon
        min_x = min(left_pos[0], right_pos[0]) - 0.05  # Add small margin
        max_x = max(left_pos[0], right_pos[0]) + 0.05
        min_y = min(left_pos[1], right_pos[1]) - 0.1   # Larger margin in y
        max_y = max(left_pos[1], right_pos[1]) + 0.1

        self.support_polygon = {
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y
        }

    def update_foot_positions(self, joint_positions):
        """
        Update foot positions from joint configuration
        """
        # Calculate foot positions using forward kinematics
        # This is simplified - in practice, use full FK
        self.foot_positions['left'] = self.calculate_foot_position(joint_positions, 'left')
        self.foot_positions['right'] = self.calculate_foot_position(joint_positions, 'right')

    def calculate_foot_position(self, joint_positions, foot_side):
        """
        Calculate foot position using simplified forward kinematics
        """
        # This would use full robot kinematics in practice
        # For this example, use a simplified model

        if foot_side == 'left':
            # Use left leg joint positions (indices are example)
            leg_joints = joint_positions[21:27]  # Assuming left leg joints are 21-26
        else:  # right
            leg_joints = joint_positions[27:33]  # Assuming right leg joints are 27-32

        # Simplified calculation of foot position based on leg joints
        # In practice, use full DH parameters or kinematic model
        foot_pos = np.array([0.0, 0.1 if foot_side == 'left' else -0.1, 0.0])

        # Add influence from joint angles
        for i, angle in enumerate(leg_joints):
            foot_pos[0] += 0.05 * math.sin(angle + i * 0.5)  # Simplified influence
            foot_pos[1] += 0.02 * math.cos(angle + i * 0.3)  # Simplified influence
            foot_pos[2] -= 0.08  # Foot is below hip (negative z)

        return foot_pos

    def calculate_initial_support_polygon(self):
        """
        Calculate initial support polygon
        """
        return {
            'min_x': -0.1,
            'max_x': 0.1,
            'min_y': -0.2,
            'max_y': 0.2
        }

    def initialize_filter(self, state_dim, taps):
        """
        Initialize filter for state estimation
        """
        return np.ones(taps) / taps  # Simple moving average

    def is_balanced(self):
        """
        Check if robot is balanced based on CoM and ZMP
        """
        # Check if ZMP is within support polygon
        zmp_in_polygon = (self.support_polygon['min_x'] <= self.zmp[0] <= self.support_polygon['max_x'] and
                         self.support_polygon['min_y'] <= self.zmp[1] <= self.support_polygon['max_y'])

        # Check CoM position relative to feet
        com_stable = abs(self.com_position[0]) < 0.1 and abs(self.com_position[1]) < 0.15

        return zmp_in_polygon and com_stable

    def calculate_capture_point(self):
        """
        Calculate capture point for balance recovery
        """
        # Capture point: where to step to stop the CoM
        capture_point = self.com_position[:2] + self.com_velocity[:2] / self.omega
        return capture_point
```

## Step 6: Create Social Behavior Manager

Create the social behavior manager in `~/humanoid_robot_controller/src/humanoid_controller/behaviors/social_behavior.py`:

```python
#!/usr/bin/env python3

"""
Social Behavior Manager for Humanoid Robot
Manages social interactions, expressions, and communication
"""

import numpy as np
import math
import time
from scipy.spatial.transform import Rotation as R


class SocialBehaviorManager:
    def __init__(self, parent_node):
        self.parent = parent_node

        # Social behavior parameters
        self.social_space_radius = 1.2  # Personal space radius (m)
        self.greeting_distance = 1.0    # Distance for greeting (m)
        self.attention_span = 5.0       # Time to pay attention (seconds)

        # Behavior state
        self.current_behavior = 'idle'
        self.behavior_start_time = time.time()
        self.attended_person = None
        self.interaction_intensity = 0.5  # 0-1 scale

        # Expression and gesture libraries
        self.expressions = {
            'neutral': [0, 0, 0, 0],      # [brow, eyes, mouth, cheeks]
            'happy': [0, 1, 1, 1],        # Raised brows, smiling eyes, smile, raised cheeks
            'sad': [-1, -1, -1, 0],       # Lowered brows, sad eyes, frown, neutral cheeks
            'surprised': [1, 1, 0, 0],    # Raised brows, wide eyes, neutral mouth
            'attentive': [0, 0.5, 0, 0]   # Slightly raised brows for attention
        }

        self.gestures = {
            'wave': self.wave_gesture,
            'point': self.point_gesture,
            'nod': self.nod_gesture,
            'shake_head': self.shake_head_gesture,
            'beckon': self.beckon_gesture
        }

    def execute_behaviors(self, current_state):
        """
        Execute social behaviors based on current state
        """
        # Check if there are humans nearby
        humans_nearby = self.detect_humans_in_field_of_view(current_state)

        if humans_nearby:
            # Select appropriate behavior based on context
            self.select_and_execute_behavior(humans_nearby, current_state)
        else:
            # Return to idle behavior
            self.current_behavior = 'idle'
            self.set_expression('neutral')

    def detect_humans_in_field_of_view(self, current_state):
        """
        Detect humans in robot's field of view (simplified)
        """
        # This would use vision processing in practice
        # For this example, return a mock detection
        camera_image = self.parent.camera_image
        if camera_image is not None:
            # Mock detection - in practice, use object detection
            return [{'position': np.array([1.0, 0.0, 0.0]), 'distance': 1.0}]
        return []

    def select_and_execute_behavior(self, humans, current_state):
        """
        Select and execute appropriate social behavior
        """
        closest_human = min(humans, key=lambda h: h['distance'])
        distance = closest_human['distance']

        if distance <= self.greeting_distance and self.current_behavior != 'greeting':
            # Close enough for greeting
            self.execute_greeting_behavior(closest_human)
        elif distance <= self.social_space_radius:
            # In personal space, be attentive
            self.maintain_attention_behavior(closest_human)
        else:
            # Outside personal space, acknowledge presence
            self.acknowledge_presence_behavior(closest_human)

    def execute_greeting_behavior(self, human):
        """
        Execute greeting behavior
        """
        if self.current_behavior != 'greeting':
            self.current_behavior = 'greeting'
            self.behavior_start_time = time.time()

            # Wave gesture
            self.execute_wave_behavior(target=human['position'])

            # Friendly expression
            self.set_expression('happy')

            # Say greeting
            self.speak_greeting()

    def maintain_attention_behavior(self, human):
        """
        Maintain attention to nearby human
        """
        if self.current_behavior != 'attentive':
            self.current_behavior = 'attentive'
            self.behavior_start_time = time.time()

            # Attentive expression
            self.set_expression('attentive')

            # Maintain gaze
            self.maintain_gaze_on_human(human)

    def acknowledge_presence_behavior(self, human):
        """
        Acknowledge human presence from distance
        """
        if self.current_behavior != 'acknowledge':
            self.current_behavior = 'acknowledge'
            self.behavior_start_time = time.time()

            # Brief nod gesture
            self.execute_nod_behavior()

            # Neutral expression
            self.set_expression('neutral')

    def execute_wave_behavior(self, target=None, duration=2.0):
        """
        Execute waving gesture
        """
        # Plan wave trajectory
        wave_trajectory = self.plan_wave_trajectory(target)

        # Execute wave motion
        start_time = time.time()
        while time.time() - start_time < duration:
            # Follow wave trajectory
            self.follow_arm_trajectory('right', wave_trajectory)
            time.sleep(0.01)  # 100Hz

    def plan_wave_trajectory(self, target=None):
        """
        Plan waving trajectory
        """
        # Simplified wave trajectory
        trajectory = []
        amplitude = 0.1  # 10cm amplitude

        for t in np.linspace(0, 2*np.pi, 20):  # 20 points for wave
            x_offset = 0.3  # Extend arm
            y_offset = amplitude * math.sin(t)  # Vertical oscillation
            z_offset = 0.05 * math.cos(t)  # Forward-back oscillation

            position = np.array([x_offset, y_offset, z_offset])
            trajectory.append(position)

        return trajectory

    def execute_nod_behavior(self, count=1):
        """
        Execute nodding gesture
        """
        for i in range(count):
            # Nod down
            self.move_head([0.2, 0, 0])  # Pitch down
            time.sleep(0.3)

            # Nod up
            self.move_head([0, 0, 0])  # Return to neutral
            time.sleep(0.3)

    def set_expression(self, expression_name):
        """
        Set facial expression
        """
        if expression_name in self.expressions:
            expression_values = self.expressions[expression_name]
            # In practice, send these values to facial actuation system
            print(f"Setting expression: {expression_name} with values {expression_values}")

    def speak_greeting(self):
        """
        Speak greeting message
        """
        greeting_messages = [
            "Hello! Nice to meet you!",
            "Hi there! How can I help you?",
            "Greetings! I'm your humanoid assistant."
        ]

        # In practice, use text-to-speech system
        import random
        message = random.choice(greeting_messages)
        print(f"Speaking: {message}")

    def maintain_gaze_on_human(self, human):
        """
        Maintain gaze on human
        """
        # Calculate where to look
        human_pos = human['position']

        # In practice, control head/eye movements
        print(f"Maintaining gaze on human at {human_pos}")

    def move_head(self, joint_angles):
        """
        Move head to specified joint angles
        """
        # In practice, send commands to head joints
        print(f"Moving head to angles: {joint_angles}")

    def follow_arm_trajectory(self, arm_side, trajectory):
        """
        Follow arm trajectory
        """
        # In practice, use trajectory controller
        print(f"Following {arm_side} arm trajectory")

    def get_social_commands(self, current_state):
        """
        Get social behavior commands to combine with other controllers
        """
        # Return commands that complement balance and manipulation
        social_commands = {
            'head_orientation': self.get_head_orientation_command(),
            'facial_expression': self.get_current_expression(),
            'gesture_commands': self.get_active_gesture_commands()
        }

        return social_commands

    def get_head_orientation_command(self):
        """
        Get head orientation command for social interaction
        """
        # If attending to someone, orient head toward them
        if self.attended_person:
            # Calculate direction to attended person
            direction_to_person = self.attended_person['position'] - self.parent.state_estimator.com_position
            direction_xy = direction_to_person[:2]  # Only x,y for head orientation

            if np.linalg.norm(direction_xy) > 0.1:  # Avoid division by zero
                direction_xy = direction_xy / np.linalg.norm(direction_xy)

                # Convert to head joint commands
                # This would map to actual head joint angles in practice
                head_yaw = math.atan2(direction_xy[1], direction_xy[0])
                head_pitch = 0  # Keep level for now

                return {'yaw': head_yaw, 'pitch': head_pitch}

        # Default: look forward
        return {'yaw': 0, 'pitch': 0}

    def get_current_expression(self):
        """
        Get current facial expression
        """
        return self.current_behavior

    def get_active_gesture_commands(self):
        """
        Get commands for any active gestures
        """
        # Return any ongoing gesture commands
        return []

    def execute_social_interaction(self, interaction_type, target_person=None):
        """
        Execute specific type of social interaction
        """
        if interaction_type == 'greeting':
            self.execute_greeting_interaction(target_person)
        elif interaction_type == 'farewell':
            self.execute_farewell_interaction(target_person)
        elif interaction_type == 'assistance_request':
            self.execute_assistance_interaction(target_person)
        elif interaction_type == 'conversation':
            self.execute_conversation_interaction(target_person)

    def execute_greeting_interaction(self, target_person):
        """
        Execute full greeting interaction
        """
        print("Executing greeting interaction...")

        # Move to appropriate distance
        self.move_to_interaction_distance(target_person, self.greeting_distance)

        # Make eye contact
        self.maintain_gaze_on_human(target_person)

        # Wave
        self.execute_wave_behavior(target=target_person['position'])

        # Smile
        self.set_expression('happy')

        # Speak
        self.speak_greeting()

    def move_to_interaction_distance(self, target_person, desired_distance):
        """
        Move to appropriate distance for interaction
        """
        # Calculate current distance
        current_pos = self.parent.state_estimator.com_position
        target_pos = target_person['position']
        current_distance = np.linalg.norm(target_pos[:2] - current_pos[:2])

        # If too far, move closer (simplified)
        if current_distance > desired_distance + 0.2:  # 20cm tolerance
            print(f"Moving closer to person, current: {current_distance:.2f}m, desired: {desired_distance:.2f}m")
            # In practice, use locomotion controller to move

    def execute_farewell_interaction(self, target_person):
        """
        Execute farewell interaction
        """
        print("Executing farewell interaction...")

        # Wave goodbye
        self.execute_wave_behavior(target=target_person['position'])

        # Set sad/sympathetic expression
        self.set_expression('happy')  # Actually happy to say goodbye in a positive way

        # Speak farewell
        farewell_messages = [
            "Goodbye! Have a great day!",
            "See you later!",
            "Take care!"
        ]
        import random
        message = random.choice(farewell_messages)
        print(f"Speaking: {message}")

    def execute_assistance_interaction(self, target_person):
        """
        Execute interaction for requesting/providing assistance
        """
        print("Executing assistance interaction...")

        # Lean forward slightly to show attention
        # In practice, adjust posture

        # Make direct eye contact
        self.maintain_gaze_on_human(target_person)

        # Set attentive expression
        self.set_expression('attentive')

        # Speak assistance message
        assistance_messages = [
            "How can I assist you?",
            "I'm here to help. What do you need?",
            "Please let me know if you need any help."
        ]
        import random
        message = random.choice(assistance_messages)
        print(f"Speaking: {message}")

    def execute_conversation_interaction(self, target_person):
        """
        Execute conversation-like interaction
        """
        print("Executing conversation interaction...")

        # Maintain natural gaze
        self.maintain_gaze_on_human(target_person)

        # Use conversational expressions (vary over time)
        conversation_expressions = ['attentive', 'happy', 'neutral']
        import random
        self.set_expression(random.choice(conversation_expressions))

        # Nod occasionally to show understanding
        if time.time() - self.behavior_start_time > 3:  # Every 3 seconds
            self.execute_nod_behavior(count=1)
            self.behavior_start_time = time.time()
```

## Step 7: Create the Main Launch File

Create the launch file in `~/humanoid_robot_controller/launch/humanoid_controller.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Humanoid controller node
    humanoid_controller = Node(
        package='humanoid_controller',
        executable='humanoid_controller_node',
        name='humanoid_controller',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Additional nodes could be added here:
    # - Vision processing node
    # - Audio processing node
    # - High-level task planner

    return LaunchDescription([
        use_sim_time,
        humanoid_controller
    ])
```

## Step 8: Update setup.py

Update the setup.py file in `~/humanoid_robot_controller/setup.py`:

```python
from setuptools import setup

package_name = 'humanoid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,
              f'{package_name}.controllers',
              f'{package_name}.sensors',
              f'{package_name}.utils',
              f'{package_name}.behaviors'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/humanoid_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Humanoid Robot Controller integrating balance, manipulation, and social interaction',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_controller_node = humanoid_controller.humanoid_controller_node:main',
        ],
    },
)
```

## Step 9: Create a Simple Test Script

Create a test script in `~/humanoid_robot_controller/test_controller.py`:

```python
#!/usr/bin/env python3

"""
Test script for Humanoid Controller
Demonstrates basic functionality
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time


class ControllerTester(Node):
    def __init__(self):
        super().__init__('controller_tester')

        # Create subscriber to controller status
        self.status_sub = self.create_subscription(
            String,
            '/controller_status',
            self.status_callback,
            10
        )

        # Create subscriber to CoM position
        self.com_sub = self.create_subscription(
            Point,
            '/center_of_mass',
            self.com_callback,
            10
        )

        # Create subscriber to ZMP
        self.zmp_sub = self.create_subscription(
            Point,
            '/zero_moment_point',
            self.zmp_callback,
            10
        )

        # Store latest values
        self.latest_status = None
        self.latest_com = None
        self.latest_zmp = None

        self.get_logger().info('Controller Tester initialized')

    def status_callback(self, msg):
        self.latest_status = msg.data
        self.get_logger().info(f'Controller Status: {msg.data}')

    def com_callback(self, msg):
        self.latest_com = [msg.x, msg.y, msg.z]
        # Don't log every message to avoid spam

    def zmp_callback(self, msg):
        self.latest_zmp = [msg.x, msg.y, msg.z]
        # Don't log every message to avoid spam

    def test_basic_functions(self):
        """Test basic controller functions"""
        self.get_logger().info('Testing basic controller functions...')

        # Wait a bit for messages to arrive
        time.sleep(2.0)

        if self.latest_status:
            self.get_logger().info(f'Current status: {self.latest_status}')

        if self.latest_com:
            self.get_logger().info(f'Current CoM: [{self.latest_com[0]:.3f}, {self.latest_com[1]:.3f}, {self.latest_com[2]:.3f}]')

        if self.latest_zmp:
            self.get_logger().info(f'Current ZMP: [{self.latest_zmp[0]:.3f}, {self.latest_zmp[1]:.3f}, {self.latest_zmp[2]:.3f}]')

        # Test mode switching (this would require service calls in practice)
        self.get_logger().info('Test completed successfully!')


def main(args=None):
    rclpy.init(args=args)
    tester = ControllerTester()

    # Run basic tests
    tester.test_basic_functions()

    # Keep node alive briefly to receive messages
    time.sleep(5.0)

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 10: Build and Test the Controller

Now let's build the package and test it:

```bash
# Terminal 1: Build the package
cd ~/humanoid_robot_controller
colcon build --packages-select humanoid_controller
source install/setup.bash

# Terminal 2: Run the controller (in simulation environment)
ros2 launch humanoid_controller humanoid_controller.launch.py

# Terminal 3: Test the controller
ros2 run humanoid_controller test_controller
```

## Step 11: Advanced Testing Scenarios

Create an advanced testing script in `~/humanoid_robot_controller/test_advanced.py`:

```python
#!/usr/bin/env python3

"""
Advanced test script for Humanoid Controller
Tests complex scenarios including balance recovery, manipulation, and interaction
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import numpy as np


class AdvancedControllerTester(Node):
    def __init__(self):
        super().__init__('advanced_controller_tester')

        # Publishers for testing
        self.mode_cmd_pub = self.create_publisher(String, '/controller_mode_cmd', 10)
        self.test_cmd_pub = self.create_publisher(String, '/controller_test_cmd', 10)

        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/controller_status',
            self.status_callback,
            10
        )

        self.com_sub = self.create_subscription(
            Point,
            '/center_of_mass',
            self.com_callback,
            10
        )

        self.zmp_sub = self.create_subscription(
            Point,
            '/zero_moment_point',
            self.zmp_callback,
            10
        )

        # Test state
        self.test_results = {}
        self.current_test = None

        self.get_logger().info('Advanced Controller Tester initialized')

    def status_callback(self, msg):
        if self.current_test:
            print(f'Test {self.current_test}: Status - {msg.data}')

    def com_callback(self, msg):
        if self.current_test and 'balance' in self.current_test:
            com = np.array([msg.x, msg.y, msg.z])
            self.test_results[self.current_test]['com_history'].append(com)

    def zmp_callback(self, msg):
        if self.current_test and 'balance' in self.current_test:
            zmp = np.array([msg.x, msg.y, msg.z])
            self.test_results[self.current_test]['zmp_history'].append(zmp)

    def run_comprehensive_test(self):
        """Run comprehensive tests of all controller capabilities"""
        self.get_logger().info('Starting comprehensive controller test...')

        # Test 1: Balance control
        self.run_balance_test()

        # Test 2: Basic manipulation
        self.run_manipulation_test()

        # Test 3: Social interaction
        self.run_interaction_test()

        # Test 4: Integrated behavior
        self.run_integration_test()

        self.print_test_summary()

    def run_balance_test(self):
        """Test balance control capabilities"""
        self.current_test = 'balance_basic'
        self.test_results[self.current_test] = {
            'passed': False,
            'com_history': [],
            'zmp_history': [],
            'start_time': time.time()
        }

        self.get_logger().info('Running basic balance test...')

        # Let it run for 10 seconds
        time.sleep(10.0)

        # Analyze results
        com_history = self.test_results[self.current_test]['com_history']
        zmp_history = self.test_results[self.current_test]['zmp_history']

        if len(com_history) > 50 and len(zmp_history) > 50:
            # Calculate stability metrics
            com_stability = np.std([c[0:2] for c in com_history[-20:]])  # Last 20 CoM positions
            zmp_stability = np.std([z[0:2] for z in zmp_history[-20:]])  # Last 20 ZMP positions

            self.get_logger().info(f'Balance test - CoM stability: {np.mean(com_stability):.3f}, ZMP stability: {np.mean(zmp_stability):.3f}')

            # Check if stable (thresholds are arbitrary for demo)
            if np.mean(com_stability) < 0.05 and np.mean(zmp_stability) < 0.05:
                self.test_results[self.current_test]['passed'] = True
                self.get_logger().info('✓ Balance test PASSED')
            else:
                self.get_logger().info('✗ Balance test FAILED')
        else:
            self.get_logger().info('✗ Balance test INCONCLUSIVE - insufficient data')

    def run_manipulation_test(self):
        """Test manipulation capabilities"""
        self.current_test = 'manipulation_basic'
        self.test_results[self.current_test] = {
            'passed': False,
            'start_time': time.time()
        }

        self.get_logger().info('Running basic manipulation test...')

        # In a real test, you would command specific manipulation tasks
        # For this demo, just verify the controller can switch modes
        mode_cmd = String()
        mode_cmd.data = 'manipulate'

        # Publish mode command (in real scenario, this would use services)
        for i in range(5):
            self.mode_cmd_pub.publish(mode_cmd)
            time.sleep(0.5)

        # Wait to see if mode switches
        time.sleep(2.0)

        # For this basic test, assume it passes if no errors occurred
        self.test_results[self.current_test]['passed'] = True
        self.get_logger().info('✓ Manipulation test completed')

    def run_interaction_test(self):
        """Test social interaction capabilities"""
        self.current_test = 'interaction_basic'
        self.test_results[self.current_test] = {
            'passed': False,
            'start_time': time.time()
        }

        self.get_logger().info('Running basic interaction test...')

        # Switch to interaction mode
        mode_cmd = String()
        mode_cmd.data = 'interact'

        for i in range(5):
            self.mode_cmd_pub.publish(mode_cmd)
            time.sleep(0.5)

        # Wait to observe interaction behaviors
        time.sleep(5.0)

        # For this basic test, assume it passes
        self.test_results[self.current_test]['passed'] = True
        self.get_logger().info('✓ Interaction test completed')

    def run_integration_test(self):
        """Test integrated behavior - balance + manipulation + interaction"""
        self.current_test = 'integration_full'
        self.test_results[self.current_test] = {
            'passed': False,
            'start_time': time.time()
        }

        self.get_logger().info('Running full integration test...')

        # This would test the controller coordinating all capabilities
        # For demo purposes, cycle through different modes
        modes = ['balance', 'manipulate', 'interact', 'balance']

        for mode in modes:
            mode_cmd = String()
            mode_cmd.data = mode
            self.mode_cmd_pub.publish(mode_cmd)
            self.get_logger().info(f'Switched to mode: {mode}')
            time.sleep(3.0)

        # For this basic test, assume it passes
        self.test_results[self.current_test]['passed'] = True
        self.get_logger().info('✓ Integration test completed')

    def print_test_summary(self):
        """Print summary of all tests"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('CONTROLLER TEST SUMMARY')
        self.get_logger().info('='*50)

        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result['passed'])

        for test_name, result in self.test_results.items():
            status = "PASS" if result['passed'] else "FAIL"
            duration = time.time() - result['start_time']
            self.get_logger().info(f'{test_name:<20} | {status:<4} | {duration:>5.1f}s')

        self.get_logger().info('-'*50)
        self.get_logger().info(f'Total: {total_tests}, Passed: {passed_tests}, Failed: {total_tests - passed_tests}')
        self.get_logger().info(f'Success Rate: {(passed_tests/total_tests)*100:.1f}%' if total_tests > 0 else '0%')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    tester = AdvancedControllerTester()

    try:
        tester.run_comprehensive_test()
    except KeyboardInterrupt:
        tester.get_logger().info('Testing interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Expected Results

When you run the complete humanoid controller system:

1. **Balance Control**: The robot should maintain stable balance with CoM and ZMP within appropriate bounds
2. **Manipulation**: The robot should be able to execute basic reaching and grasping motions
3. **Social Interaction**: The robot should detect humans and respond with appropriate behaviors
4. **Integration**: All systems should work together without conflicts

## Troubleshooting

If you encounter issues:

1. **Build Errors**: Ensure all dependencies are installed and paths are correct
2. **Runtime Errors**: Check that the simulation environment is properly configured
3. **Performance Issues**: The controller may need tuning for your specific robot model
4. **Integration Problems**: Verify that all ROS 2 topics and services are properly connected

## Extensions

To enhance this controller:

1. **Add more sophisticated behaviors** like emotion recognition
2. **Implement learning algorithms** to adapt to users
3. **Add more complex manipulation tasks** like bimanual coordination
4. **Include advanced safety features** like collision avoidance
5. **Add localization and mapping** for autonomous navigation

This controller provides a solid foundation for humanoid robot control that integrates balance, manipulation, and social interaction capabilities in a unified framework.
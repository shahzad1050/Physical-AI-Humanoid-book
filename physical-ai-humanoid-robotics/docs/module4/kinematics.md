# Humanoid Robot Kinematics

## Introduction to Humanoid Kinematics

Kinematics is the study of motion without considering the forces that cause the motion. In humanoid robotics, kinematics is crucial for understanding and controlling the complex movements of robots with human-like structures. Humanoid robots typically have 20-50+ degrees of freedom (DOF), making their kinematic analysis significantly more complex than simpler robotic systems.

## Humanoid Robot Structure and Joint Configuration

### Typical Humanoid DOF Distribution

A standard humanoid robot typically has the following joint distribution:

```
Total DOF: ~30-40 (varies by platform)

Head: 2-3 DOF
├── Neck pitch/yaw (2 DOF) or neck pitch/yaw/roll (3 DOF)

Arms (each): 6-7 DOF
├── Shoulder: 3 DOF (pitch/yaw/roll)
├── Elbow: 1 DOF (pitch)
├── Wrist: 2-3 DOF (pitch/yaw + roll)
└── Hand: 0-16 DOF (varies greatly by design)

Torso: 0-3 DOF
├── Waist: 0-3 DOF (pitch/yaw/roll)

Legs (each): 6-7 DOF
├── Hip: 3 DOF (pitch/yaw/roll)
├── Knee: 1 DOF (pitch)
└── Ankle: 2-3 DOF (pitch/yaw + roll)
```

### Common Humanoid Topologies

Different humanoid platforms have different topologies:

- **Nao (Aldebaran)**: 25 DOF, compact design
- **HUBO (KAIST)**: 41 DOF, full human-like structure
- **HRP-4 (AIST)**: 37 DOF, adult-sized humanoid
- **ATLAS (Boston Dynamics)**: 28 DOF, optimized for specific tasks

## Forward Kinematics

Forward kinematics calculates the position and orientation of the end-effector given the joint angles. For humanoid robots, this involves multiple end-effectors (hands, feet).

### Mathematical Representation

For a humanoid robot with n joints, the forward kinematics for any link i can be expressed as:

```
T_i = T_0 * A_1(θ_1) * A_2(θ_2) * ... * A_i(θ_i)
```

Where:
- `T_i` is the transformation matrix of link i relative to the base
- `T_0` is the base transformation
- `A_j(θ_j)` is the transformation matrix of joint j

### Denavit-Hartenberg (DH) Convention

The DH convention is commonly used to define coordinate frames for each joint:

```
A_i = [cos(θ_i)   -sin(θ_i)*cos(α_i)   sin(θ_i)*sin(α_i)   a_i*cos(θ_i)]
      [sin(θ_i)    cos(θ_i)*cos(α_i)  -cos(θ_i)*sin(α_i)   a_i*sin(θ_i)]
      [0           sin(α_i)            cos(α_i)            d_i        ]
      [0           0                   0                   1          ]
```

Where:
- `θ_i` is the joint angle
- `d_i` is the link offset
- `a_i` is the link length
- `α_i` is the link twist

### Example: 6-DOF Arm Forward Kinematics

```python
import numpy as np
from math import sin, cos

def dh_transform(theta, d, a, alpha):
    """Calculate DH transformation matrix"""
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_arm(joint_angles):
    """Calculate forward kinematics for a 6-DOF arm"""
    # DH parameters for a typical 6-DOF arm
    dh_params = [
        (joint_angles[0], 0.1, 0, -np.pi/2),  # Joint 1
        (joint_angles[1], 0, 0.3, 0),         # Joint 2
        (joint_angles[2], 0, 0, -np.pi/2),    # Joint 3
        (joint_angles[3], 0.25, 0, np.pi/2),  # Joint 4
        (joint_angles[4], 0, 0, -np.pi/2),    # Joint 5
        (joint_angles[5], 0.08, 0, 0)         # Joint 6
    ]

    T = np.eye(4)  # Identity transformation

    for theta, d, a, alpha in dh_params:
        A = dh_transform(theta, d, a, alpha)
        T = np.dot(T, A)

    return T

def get_end_effector_pose(joint_angles):
    """Get position and orientation of end-effector"""
    T = forward_kinematics_arm(joint_angles)

    # Extract position
    position = T[:3, 3]

    # Extract orientation (as rotation matrix)
    orientation = T[:3, :3]

    return position, orientation
```

### Humanoid Multi-Chain Forward Kinematics

For humanoid robots, we need to calculate forward kinematics for multiple kinematic chains:

```python
class HumanoidKinematics:
    def __init__(self):
        # Define DH parameters for each chain
        self.left_arm_dh = self.define_arm_dh()
        self.right_arm_dh = self.define_arm_dh()
        self.left_leg_dh = self.define_leg_dh()
        self.right_leg_dh = self.define_leg_dh()

    def define_arm_dh(self):
        """Define DH parameters for arm"""
        # Simplified 7-DOF arm (shoulder + elbow + wrist)
        return [
            (0, 0.1, 0, -np.pi/2),      # Shoulder pitch
            (0, 0, 0.3, 0),             # Shoulder yaw
            (0, 0, 0, 0),               # Shoulder roll
            (0, 0.25, 0, np.pi/2),      # Elbow
            (0, 0, 0, -np.pi/2),        # Wrist pitch
            (0, 0.08, 0, 0),            # Wrist yaw
            (0, 0, 0, 0)                # Wrist roll
        ]

    def define_leg_dh(self):
        """Define DH parameters for leg"""
        # Simplified 6-DOF leg (hip + knee + ankle)
        return [
            (0, 0.1, 0, -np.pi/2),      # Hip roll
            (0, 0, 0, -np.pi/2),        # Hip pitch
            (0, 0, 0.4, 0),             # Hip yaw
            (0, 0.35, 0, 0),            # Knee
            (0, 0, 0, -np.pi/2),        # Ankle pitch
            (0, 0.05, 0, 0)             # Ankle roll
        ]

    def forward_kinematics_arm(self, joint_angles, dh_params):
        """Calculate forward kinematics for arm"""
        if len(joint_angles) != len(dh_params):
            raise ValueError("Joint angles and DH parameters must have same length")

        T = np.eye(4)

        for i, (theta, d, a, alpha) in enumerate(dh_params):
            A = dh_transform(theta + joint_angles[i], d, a, alpha)
            T = np.dot(T, A)

        return T

    def get_all_end_effectors(self, joint_angles):
        """Get positions of all end-effectors"""
        # Extract joint angles for each chain
        left_arm_joints = joint_angles[0:7]
        right_arm_joints = joint_angles[7:14]
        left_leg_joints = joint_angles[14:20]
        right_leg_joints = joint_angles[20:26]

        # Calculate end-effector positions
        left_hand_pose = self.forward_kinematics_arm(left_arm_joints, self.left_arm_dh)
        right_hand_pose = self.forward_kinematics_arm(right_arm_joints, self.right_arm_dh)
        left_foot_pose = self.forward_kinematics_arm(left_leg_joints, self.left_leg_dh)
        right_foot_pose = self.forward_kinematics_arm(right_leg_joints, self.right_leg_dh)

        return {
            'left_hand': left_hand_pose,
            'right_hand': right_hand_pose,
            'left_foot': left_foot_pose,
            'right_foot': right_foot_pose
        }
```

## Inverse Kinematics

Inverse kinematics (IK) is the process of determining the joint angles required to achieve a desired end-effector position and orientation. This is more challenging than forward kinematics and often has multiple solutions or no solutions.

### Mathematical Formulation

For a given end-effector pose `T_d`, we want to find joint angles `θ` such that:

```
f(θ) = T_d
```

Where `f` represents the forward kinematics function.

### Analytical vs. Numerical Methods

#### Analytical IK

Analytical solutions exist for simple kinematic chains (like 6-DOF arms with specific geometries):

```python
def analytical_ik_6dof(position, orientation):
    """
    Analytical inverse kinematics for a 6-DOF arm with specific geometry
    Assumes spherical wrist (last 3 joints intersect at a point)
    """
    px, py, pz = position

    # Calculate wrist center position
    # Assuming tool frame offset
    tool_length = 0.1
    wrist_center_x = px - tool_length * orientation[0, 2]
    wrist_center_y = py - tool_length * orientation[1, 2]
    wrist_center_z = pz - tool_length * orientation[2, 2]

    # Calculate first three joints (position)
    theta1 = np.arctan2(wrist_center_y, wrist_center_x)

    # Calculate distance from joint 2 to wrist center
    r = np.sqrt(wrist_center_x**2 + wrist_center_y**2)
    d = wrist_center_z - 0.1  # height of joint 2

    # Calculate remaining joint angles using geometric approach
    # This is simplified - full solution would be more complex
    theta2 = np.arctan2(d, r)
    theta3 = 0  # Simplified

    # Calculate last three joints (orientation) using wrist approach
    theta4, theta5, theta6 = calculate_wrist_angles(orientation, theta1, theta2, theta3)

    return [theta1, theta2, theta3, theta4, theta5, theta6]

def calculate_wrist_angles(orientation, theta1, theta2, theta3):
    """Calculate wrist joint angles for desired orientation"""
    # Calculate rotation matrix from first 3 joints
    # Then solve for wrist joint angles
    # Implementation depends on specific arm geometry
    pass
```

#### Numerical IK

For complex kinematic chains like humanoid robots, numerical methods are typically used:

```python
import scipy.optimize as opt

class NumericalIK:
    def __init__(self, robot_kinematics):
        self.kinematics = robot_kinematics

    def ik_objective(self, joint_angles, target_pose, chain_dh):
        """Objective function for IK optimization"""
        current_pose = self.kinematics.forward_kinematics_arm(joint_angles, chain_dh)

        # Calculate position error
        pos_error = np.linalg.norm(target_pose[:3, 3] - current_pose[:3, 3])

        # Calculate orientation error (using rotation matrix difference)
        R_current = current_pose[:3, :3]
        R_target = target_pose[:3, :3]

        # Frobenius norm of rotation difference
        R_diff = R_target - R_current
        rot_error = np.linalg.norm(R_diff, 'fro')

        # Weighted combination of position and orientation errors
        total_error = pos_error + 0.1 * rot_error

        return total_error

    def solve_ik(self, target_pose, initial_guess, chain_dh, joint_limits=None):
        """Solve inverse kinematics using numerical optimization"""
        if joint_limits is None:
            # Default joint limits (±π for all joints)
            joint_limits = [(-np.pi, np.pi)] * len(initial_guess)

        # Define bounds
        bounds = opt.Bounds(
            [lim[0] for lim in joint_limits],
            [lim[1] for lim in joint_limits]
        )

        # Solve optimization problem
        result = opt.minimize(
            self.ik_objective,
            initial_guess,
            args=(target_pose, chain_dh),
            method='L-BFGS-B',
            bounds=bounds
        )

        if result.success:
            return result.x
        else:
            raise Exception(f"IK solution failed: {result.message}")
```

### Jacobian-Based Methods

The Jacobian matrix relates joint velocities to end-effector velocities:

```
v_e = J(θ) * θ̇
```

Where:
- `v_e` is the end-effector velocity vector
- `J(θ)` is the Jacobian matrix
- `θ̇` is the joint velocity vector

```python
def calculate_jacobian(robot_kinematics, joint_angles, link_index):
    """Calculate geometric Jacobian for a given link"""
    n = len(joint_angles)
    J = np.zeros((6, n))  # 6 DOF (3 position, 3 orientation)

    # Get current transformation for the link
    T_current = robot_kinematics.forward_kinematics_arm(joint_angles[:link_index+1],
                                                        robot_kinematics.get_dh_params()[:link_index+1])
    current_pos = T_current[:3, 3]
    current_rot = T_current[:3, :3]

    # Calculate Jacobian columns
    for i in range(n):
        if i <= link_index:
            # Get joint axis in world coordinates
            T_to_joint = robot_kinematics.forward_kinematics_arm(joint_angles[:i+1],
                                                                robot_kinematics.get_dh_params()[:i+1])
            joint_axis_world = T_to_joint[:3, :3] @ np.array([0, 0, 1])  # Assuming z-axis joints
            joint_pos = T_to_joint[:3, 3]

            # Position part of Jacobian
            J[:3, i] = np.cross(joint_axis_world, current_pos - joint_pos)

            # Orientation part of Jacobian
            J[3:, i] = joint_axis_world

    return J

def jacobian_ik_step(jacobian, error, alpha=0.5):
    """Calculate joint angle update using Jacobian pseudoinverse"""
    # Use damped least squares to avoid singularities
    damping = 0.01
    I = np.eye(jacobian.shape[1])

    # Damped pseudoinverse
    J_pinv = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + damping**2 * I)

    # Calculate joint angle update
    delta_theta = alpha * J_pinv @ error

    return delta_theta
```

## Humanoid-Specific Kinematic Considerations

### Redundancy Resolution

Humanoid robots often have redundant DOF (more joints than necessary for a task). This redundancy can be resolved using various techniques:

```python
def resolve_redundancy(joint_angles, joint_centers, weights):
    """
    Resolve redundancy by moving toward joint centers
    Uses null-space projection to maintain end-effector position
    """
    # Calculate joint centering objective
    centering_term = np.zeros_like(joint_angles)

    for i in range(len(joint_angles)):
        centering_term[i] = weights[i] * (joint_centers[i] - joint_angles[i])

    return centering_term

def null_space_projection(jacobian, joint_angles, joint_centers, weights):
    """Project redundancy resolution into null space of Jacobian"""
    J = jacobian
    I = np.eye(J.shape[1])

    # Calculate pseudoinverse
    J_pinv = np.linalg.pinv(J)

    # Calculate null space projection matrix
    null_space_proj = I - J_pinv @ J

    # Calculate redundancy resolution term
    redundancy_term = resolve_redundancy(joint_angles, joint_centers, weights)

    # Project into null space
    null_space_motion = null_space_proj @ redundancy_term

    return null_space_motion
```

### Whole-Body Kinematics

Humanoid robots require whole-body kinematic solutions that consider all kinematic chains simultaneously:

```python
class WholeBodyKinematics:
    def __init__(self):
        self.kinematics = HumanoidKinematics()
        self.support_polygon = None

    def solve_whole_body_ik(self, tasks, current_angles, weights=None):
        """
        Solve whole-body inverse kinematics with multiple tasks
        Tasks: list of (end_effector_name, target_pose, priority)
        """
        if weights is None:
            weights = np.ones(len(current_angles))  # Default weights

        # Define optimization problem with multiple tasks
        def objective(joint_angles):
            total_error = 0

            for task_name, target_pose, priority in tasks:
                if task_name == 'left_hand':
                    current_pose = self.kinematics.forward_kinematics_arm(
                        joint_angles[0:7], self.kinematics.left_arm_dh
                    )
                elif task_name == 'right_hand':
                    current_pose = self.kinematics.forward_kinematics_arm(
                        joint_angles[7:14], self.kinematics.right_arm_dh
                    )
                elif task_name == 'left_foot':
                    current_pose = self.kinematics.forward_kinematics_arm(
                        joint_angles[14:20], self.kinematics.left_leg_dh
                    )
                elif task_name == 'right_foot':
                    current_pose = self.kinematics.forward_kinematics_arm(
                        joint_angles[20:26], self.kinematics.right_leg_dh
                    )

                # Calculate task error
                pos_error = np.linalg.norm(target_pose[:3, 3] - current_pose[:3, 3])
                total_error += priority * pos_error

            # Add joint centering term
            centering_error = 0.01 * np.sum(weights * (joint_angles - 0)**2)
            total_error += centering_error

            return total_error

        # Solve optimization problem
        result = opt.minimize(
            objective,
            current_angles,
            method='L-BFGS-B'
        )

        return result.x if result.success else current_angles
```

## Kinematic Constraints and Limitations

### Joint Limits

Joint limits must be respected in all kinematic calculations:

```python
def enforce_joint_limits(joint_angles, min_limits, max_limits):
    """Enforce joint limits on calculated joint angles"""
    limited_angles = np.copy(joint_angles)

    for i in range(len(joint_angles)):
        limited_angles[i] = np.clip(joint_angles[i], min_limits[i], max_limits[i])

    return limited_angles

def check_joint_limits(joint_angles, min_limits, max_limits):
    """Check if joint angles are within limits"""
    for i in range(len(joint_angles)):
        if joint_angles[i] < min_limits[i] or joint_angles[i] > max_limits[i]:
            return False
    return True
```

### Singularity Avoidance

Singular configurations should be avoided as they lead to loss of DOF:

```python
def calculate_condition_number(jacobian):
    """Calculate condition number to detect singularities"""
    U, s, Vh = np.linalg.svd(jacobian)
    return s[0] / s[-1] if s[-1] != 0 else float('inf')

def is_near_singular(jacobian, threshold=100):
    """Check if Jacobian is near singular"""
    cond_num = calculate_condition_number(jacobian)
    return cond_num > threshold

def add_singular_perturbation(joint_angles, jacobian, target_pose):
    """Add small perturbation to move away from singular configuration"""
    if is_near_singular(jacobian):
        # Add small random perturbation
        perturbation = np.random.normal(0, 0.01, size=joint_angles.shape)
        return joint_angles + perturbation
    return joint_angles
```

## Implementation Example: Humanoid Arm Control

```python
class HumanoidArmController:
    def __init__(self, side='right'):
        self.side = side
        self.kinematics = HumanoidKinematics()
        self.current_angles = np.zeros(7)  # 7 DOF arm
        self.joint_limits = [(-2.967, 2.967)] * 7  # Example limits

        # Joint center positions for redundancy resolution
        self.joint_centers = np.array([0, 0, 0, 0, 0, 0, 0])
        self.centering_weights = np.array([0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05])

    def move_to_pose(self, target_position, target_orientation):
        """Move arm to target pose"""
        # Create target pose transformation matrix
        target_pose = np.eye(4)
        target_pose[:3, 3] = target_position
        target_pose[:3, :3] = target_orientation

        # Solve inverse kinematics
        try:
            new_angles = self.solve_ik(target_pose)

            # Apply joint limits
            new_angles = enforce_joint_limits(new_angles,
                                            [lim[0] for lim in self.joint_limits],
                                            [lim[1] for lim in self.joint_limits])

            # Update current angles
            self.current_angles = new_angles

            return True, new_angles
        except Exception as e:
            print(f"Failed to move to pose: {e}")
            return False, self.current_angles

    def solve_ik(self, target_pose):
        """Solve inverse kinematics for target pose"""
        # Use numerical method with initial guess
        initial_guess = self.current_angles

        # Set up optimization with multiple objectives
        def objective(angles):
            # Calculate current pose
            current_pose = self.kinematics.forward_kinematics_arm(
                angles,
                self.kinematics.right_arm_dh if self.side == 'right' else self.kinematics.left_arm_dh
            )

            # Position error
            pos_error = np.linalg.norm(target_pose[:3, 3] - current_pose[:3, 3])

            # Orientation error
            R_diff = target_pose[:3, :3] - current_pose[:3, :3]
            rot_error = np.linalg.norm(R_diff, 'fro')

            # Joint centering (redundancy resolution)
            centering_error = np.sum(self.centering_weights * (angles - self.joint_centers)**2)

            # Total objective
            total_error = pos_error + 0.1 * rot_error + 0.01 * centering_error

            return total_error

        # Solve optimization
        result = opt.minimize(
            objective,
            initial_guess,
            method='L-BFGS-B',
            bounds=self.joint_limits
        )

        if result.success:
            return result.x
        else:
            raise Exception(f"IK solution failed: {result.message}")

    def get_current_pose(self):
        """Get current end-effector pose"""
        current_pose = self.kinematics.forward_kinematics_arm(
            self.current_angles,
            self.kinematics.right_arm_dh if self.side == 'right' else self.kinematics.left_arm_dh
        )

        position = current_pose[:3, 3]
        orientation = current_pose[:3, :3]

        return position, orientation
```

## Best Practices for Humanoid Kinematics

### 1. Modular Design

Design kinematic functions to be modular and reusable:

```python
# Separate forward and inverse kinematics into distinct modules
# Allow for easy swapping of algorithms
# Provide consistent interfaces
```

### 2. Error Handling

Always include proper error handling for kinematic calculations:

```python
def safe_ik_solution(target_pose, current_angles):
    """Safely solve IK with error handling"""
    try:
        solution = solve_ik(target_pose, current_angles)

        # Verify solution
        if verify_ik_solution(target_pose, solution):
            return solution
        else:
            raise ValueError("IK solution verification failed")
    except Exception as e:
        print(f"IK error: {e}")
        return current_angles  # Return current angles if failed
```

### 3. Performance Optimization

For real-time humanoid control, optimize kinematic calculations:

```python
# Use efficient numerical libraries (NumPy, SciPy)
# Cache transformation matrices when possible
# Use analytical solutions when available
# Consider approximate methods for real-time applications
```

## Summary

Humanoid robot kinematics is a complex but fundamental aspect of humanoid robotics. Understanding both forward and inverse kinematics is essential for controlling these multi-DOF systems effectively. The redundancy inherent in humanoid robots provides both opportunities (for obstacle avoidance, joint centering) and challenges (multiple solutions, singularity avoidance) that must be carefully managed.

Key takeaways from this section include:

1. **Forward kinematics** provides the mathematical foundation for determining end-effector positions from joint angles
2. **Inverse kinematics** is typically solved using numerical methods for complex humanoid structures
3. **Redundancy resolution** is crucial for managing the extra DOF in humanoid robots
4. **Constraint handling** for joint limits, singularities, and other physical constraints is essential
5. **Whole-body coordination** requires considering all kinematic chains simultaneously

In the next section, we'll explore the dynamics and control aspects of humanoid robots, which build upon the kinematic foundation established here.
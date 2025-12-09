# Balance and Postural Control

## Introduction to Balance and Postural Control

Balance and postural control are fundamental capabilities for humanoid robots, enabling them to maintain stability while performing various tasks. Unlike wheeled or tracked robots, humanoid robots must actively control their posture to maintain balance, especially during dynamic movements like walking, running, or manipulation tasks.

## Fundamentals of Postural Control

### Postural Control System Components

The postural control system in humanoid robots typically consists of three main components:

1. **Sensors**: IMU, force/torque sensors, joint encoders, vision systems
2. **State Estimation**: Estimating robot pose, velocity, and external disturbances
3. **Control Algorithms**: Generating appropriate motor commands to maintain balance

### Balance Control Strategies

Humanoid robots employ several balance control strategies:

1. **Ankle Strategy**: Adjusting ankle torques to maintain balance (for small disturbances)
2. **Hip Strategy**: Using hip movements to control balance (for medium disturbances)
3. **Stepping Strategy**: Taking a step to recover balance (for large disturbances)
4. **Arm Strategy**: Using arm movements to assist balance

## State Estimation for Balance

### Center of Mass (CoM) Estimation

Accurate CoM estimation is crucial for balance control:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class CenterOfMassEstimator:
    def __init__(self, robot_model):
        self.model = robot_model
        self.link_masses = robot_model.link_masses
        self.link_poses = {}

    def estimate_com_position(self, joint_angles):
        """
        Estimate Center of Mass position from joint angles
        """
        total_mass = sum(self.link_masses.values())

        com_numerator = np.zeros(3)

        # Calculate transforms for each link
        transforms = self.calculate_link_transforms(joint_angles)

        for link_name, mass in self.link_masses.items():
            if link_name in transforms:
                # Get link CoM position in world frame
                link_transform = transforms[link_name]
                local_com = self.model.get_link_com_offset(link_name)

                # Transform local CoM to world frame
                world_com = link_transform[:3, :3] @ local_com + link_transform[:3, 3]

                com_numerator += mass * world_com

        com_position = com_numerator / total_mass
        return com_position

    def estimate_com_velocity(self, joint_angles, joint_velocities):
        """
        Estimate CoM velocity using Jacobian method
        """
        J_com = self.calculate_com_jacobian(joint_angles)
        com_velocity = J_com @ joint_velocities
        return com_velocity

    def estimate_com_acceleration(self, joint_angles, joint_velocities, joint_accelerations):
        """
        Estimate CoM acceleration
        """
        J_com = self.calculate_com_jacobian(joint_angles)
        J_com_dot = self.calculate_com_jacobian_derivative(joint_angles, joint_velocities)

        com_acceleration = J_com @ joint_accelerations + J_com_dot @ joint_velocities
        return com_acceleration

    def calculate_link_transforms(self, joint_angles):
        """
        Calculate transforms for all links
        """
        transforms = {}

        # This would use forward kinematics to calculate all link transforms
        # Implementation depends on robot kinematic structure
        for link_name in self.model.link_names:
            transform = self.model.forward_kinematics(joint_angles, link_name)
            transforms[link_name] = transform

        return transforms

    def calculate_com_jacobian(self, joint_angles):
        """
        Calculate Jacobian matrix for CoM
        """
        n_joints = len(joint_angles)
        J_com = np.zeros((3, n_joints))

        # Calculate CoM Jacobian using individual link contributions
        total_mass = sum(self.link_masses.values())

        transforms = self.calculate_link_transforms(joint_angles)

        for i, (link_name, mass) in enumerate(self.link_masses.items()):
            if link_name in transforms:
                # Get link Jacobian
                J_link = self.model.get_link_jacobian(joint_angles, link_name)

                # Weight by mass ratio
                J_com += (mass / total_mass) * J_link[:3, :]  # Only position part

        return J_com

    def calculate_com_jacobian_derivative(self, joint_angles, joint_velocities):
        """
        Calculate time derivative of CoM Jacobian
        """
        # This is a simplified approximation
        # Full implementation would require more complex calculations
        n_joints = len(joint_angles)
        J_com_dot = np.zeros((3, n_joints))

        # Approximate using finite differences
        h = 1e-6
        J_plus = self.calculate_com_jacobian(joint_angles + h * joint_velocities)
        J_minus = self.calculate_com_jacobian(joint_angles - h * joint_velocities)

        J_com_dot = (J_plus - J_minus) / (2 * h)

        return J_com_dot
```

### State Estimation with Sensor Fusion

```python
class StateEstimator:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.state = np.zeros(13)  # [pos(3), quat(4), vel(3), omega(3)]
        self.covariance = np.eye(13) * 0.1
        self.gravity = np.array([0, 0, -9.81])

        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # Measurement noise
        self.R_imu = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])  # acc, gyro
        self.R_foot = np.diag([0.001, 0.001, 0.001])  # position

    def predict(self, control_input):
        """
        Prediction step of EKF
        """
        # Extract state variables
        pos = self.state[0:3]
        quat = self.state[3:7]  # [w, x, y, z]
        vel = self.state[7:10]
        omega = self.state[10:13]  # Angular velocity

        # Normalize quaternion
        quat = quat / np.linalg.norm(quat)

        # Convert angular velocity to quaternion derivative
        omega_quat = np.array([0, omega[0], omega[1], omega[2]])
        quat_dot = 0.5 * self.quaternion_multiply(quat, omega_quat)

        # State transition model
        new_pos = pos + vel * self.dt
        new_quat = quat + quat_dot * self.dt
        new_quat = new_quat / np.linalg.norm(new_quat)  # Renormalize

        # Acceleration from control and gravity
        linear_acc = control_input['linear_acc'] if 'linear_acc' in control_input else np.zeros(3)
        new_vel = vel + (linear_acc + self.gravity) * self.dt
        new_omega = omega + control_input.get('angular_acc', np.zeros(3)) * self.dt

        # Update state
        self.state = np.hstack([new_pos, new_quat, new_vel, new_omega])

        # Update covariance (simplified)
        F = self.calculate_jacobian()
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update_with_imu(self, measured_acc, measured_gyro):
        """
        Update state estimate with IMU measurements
        """
        # Predicted measurements
        quat = self.state[3:7]
        R_world_imu = self.quaternion_to_rotation_matrix(quat)

        # Predicted acceleration (in world frame)
        linear_acc_world = self.state[7:10]  # Linear acceleration
        predicted_acc = R_world_imu.T @ (linear_acc_world - self.gravity)

        # Predicted angular velocity
        predicted_gyro = self.state[10:13]

        predicted_measurement = np.hstack([predicted_acc, predicted_gyro])
        measured_measurement = np.hstack([measured_acc, measured_gyro])

        # Innovation
        innovation = measured_measurement - predicted_measurement

        # Measurement matrix
        H = self.get_imu_measurement_matrix()

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_imu

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ innovation

        # Update covariance
        I = np.eye(len(self.state))
        self.covariance = (I - K @ H) @ self.covariance

    def update_with_foot_contact(self, foot_position, is_left_foot=True):
        """
        Update state estimate with foot contact information
        """
        # Foot position should be at ground level (z=0) when in contact
        if is_left_foot:
            predicted_foot_pos = self.get_left_foot_position()
        else:
            predicted_foot_pos = self.get_right_foot_position()

        # Measurement residual
        residual = foot_position - predicted_foot_pos

        # Simplified update - only correct position part
        self.state[0:3] = self.state[0:3] + 0.1 * residual  # Simple correction

    def get_left_foot_position(self):
        """
        Get left foot position from current state
        """
        # This would use forward kinematics with current joint angles
        # Simplified as an example
        return self.state[0:3] + np.array([0, 0.1, -0.8])  # Approximate

    def get_right_foot_position(self):
        """
        Get right foot position from current state
        """
        # Simplified as an example
        return self.state[0:3] + np.array([0, -0.1, -0.8])  # Approximate

    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def quaternion_to_rotation_matrix(self, quat):
        """
        Convert quaternion to rotation matrix
        """
        w, x, y, z = quat

        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

        return R

    def calculate_jacobian(self):
        """
        Calculate state transition Jacobian
        """
        # Simplified Jacobian matrix
        F = np.eye(13)

        # Position with respect to velocity
        F[0:3, 7:10] = np.eye(3) * self.dt

        # Velocity with respect to acceleration
        F[7:10, 7:10] = np.eye(3)  # Basic integration

        # Quaternion with respect to angular velocity (simplified)
        F[3:7, 10:13] = 0.5 * self.dt  # Very simplified

        return F

    def get_imu_measurement_matrix(self):
        """
        Get measurement matrix for IMU
        """
        # Simplified - maps state to IMU measurements
        H = np.zeros((6, 13))

        # Accelerometer measures linear acceleration + gravity
        H[0:3, 7:10] = np.eye(3)  # Linear acceleration affects accelerometer

        # Gyroscope measures angular velocity
        H[3:6, 10:13] = np.eye(3)  # Angular velocity affects gyroscope

        return H
```

## Balance Control Algorithms

### Linear Inverted Pendulum Model (LIPM)

The Linear Inverted Pendulum Model is widely used for balance control:

```python
class LIPMController:
    def __init__(self, com_height=0.8, gravity=9.81):
        self.h = com_height  # CoM height
        self.g = gravity
        self.omega = np.sqrt(self.g / self.h)

        # Control gains
        self.K_pos = 10.0
        self.K_vel = 2.0 * np.sqrt(10.0)  # Critical damping

    def calculate_zmp_from_com(self, com_pos, com_vel):
        """
        Calculate ZMP from CoM state using LIPM
        """
        # ZMP = CoM - (h/g) * CoM_acceleration
        # For LIPM: CoM_acceleration = omega^2 * (CoM - ZMP)
        # So: ZMP = CoM - (1/omega^2) * CoM_acceleration
        # But we need a control law to determine CoM_acceleration

        # Simplified control law: ZMP = reference + Kp*pos_error + Kd*vel_error
        zmp_ref = np.array([0.0, 0.0])  # Reference ZMP (typically under feet center)

        pos_error = com_pos[:2] - zmp_ref
        vel_error = com_vel[:2]

        zmp_cmd = zmp_ref + self.K_pos * pos_error + self.K_vel * vel_error

        return zmp_cmd

    def calculate_com_reference(self, zmp_ref, current_com, current_com_vel, dt=0.01):
        """
        Calculate CoM reference trajectory from ZMP reference
        """
        # Inverted dynamics: CoM_ddot = g/h * (CoM - ZMP)
        com_acc_ref = (self.g / self.h) * (current_com[:2] - zmp_ref)

        # Integrate to get velocity and position references
        com_vel_ref = current_com_vel[:2] + com_acc_ref * dt
        com_pos_ref = current_com[:2] + com_vel_ref * dt

        return com_pos_ref, com_vel_ref, com_acc_ref

    def generate_balance_trajectory(self, current_state, target_zmp, duration=1.0, dt=0.01):
        """
        Generate balance recovery trajectory
        """
        timesteps = int(duration / dt)

        com_trajectory = np.zeros((timesteps, 3))
        com_velocity = np.zeros((timesteps, 3))
        com_acceleration = np.zeros((timesteps, 3))

        current_com = current_state['com_pos']
        current_com_vel = current_state['com_vel']

        for i in range(timesteps):
            t = i * dt

            # Interpolate ZMP reference
            alpha = t / duration
            current_zmp_ref = alpha * target_zmp + (1 - alpha) * current_state['current_zmp'][:2]

            # Calculate CoM reference
            com_pos_ref, com_vel_ref, com_acc_ref = self.calculate_com_reference(
                current_zmp_ref, current_com, current_com_vel, dt
            )

            # Update for next iteration
            current_com[:2] = com_pos_ref
            current_com_vel[:2] = com_vel_ref

            com_trajectory[i, :2] = com_pos_ref
            com_velocity[i, :2] = com_vel_ref
            com_acceleration[i, :2] = com_acc_ref

            # Maintain height
            com_trajectory[i, 2] = self.h
            com_velocity[i, 2] = 0
            com_acceleration[i, 2] = 0

        return com_trajectory, com_velocity, com_acceleration

    def calculate_capture_point(self, com_pos, com_vel):
        """
        Calculate capture point - where to step to stop the CoM
        """
        capture_point = com_pos[:2] + com_vel[:2] / self.omega
        return capture_point
```

### Cart-Table Model

The Cart-Table model is another approach for balance control:

```python
class CartTableController:
    def __init__(self, com_height=0.8, gravity=9.81, cart_mass=1.0):
        self.h = com_height
        self.g = gravity
        self.m = cart_mass  # Effective mass of CoM
        self.omega = np.sqrt(self.g / self.h)

        # Control gains
        self.Kp_com = np.diag([100, 100])  # CoM position
        self.Kd_com = np.diag([20, 20])    # CoM velocity
        self.Kp_foot = np.diag([50, 50])   # Foot position
        self.Kd_foot = np.diag([10, 10])   # Foot velocity

    def compute_cart_table_control(self, current_state, desired_state):
        """
        Compute control using Cart-Table model
        """
        # Current state
        com_pos = current_state['com_pos'][:2]
        com_vel = current_state['com_vel'][:2]
        foot_pos = current_state['foot_pos'][:2]
        foot_vel = current_state['foot_vel'][:2]

        # Desired state
        des_com_pos = desired_state['com_pos'][:2]
        des_com_vel = desired_state['com_vel'][:2]
        des_foot_pos = desired_state['foot_pos'][:2]
        des_foot_vel = desired_state['foot_vel'][:2]

        # Position and velocity errors
        com_pos_error = des_com_pos - com_pos
        com_vel_error = des_com_vel - com_vel
        foot_pos_error = des_foot_pos - foot_pos
        foot_vel_error = des_foot_vel - foot_vel

        # Control law: F = Kp*pos_error + Kd*vel_error
        com_control = self.Kp_com @ com_pos_error + self.Kd_com @ com_vel_error
        foot_control = self.Kp_foot @ foot_pos_error + self.Kd_foot @ foot_vel_error

        # Combine controls
        total_control = com_control + foot_control

        return total_control
```

## Multi-Strategy Balance Control

### Hierarchical Balance Control

```python
class HierarchicalBalanceController:
    def __init__(self, robot_model):
        self.model = robot_model
        self.lipm_controller = LIPMController(com_height=0.8)
        self.state_estimator = StateEstimator()
        self.footstep_planner = FootstepPlanner()

        # Balance strategy thresholds
        self.ankle_threshold = 0.05  # 5cm
        self.hip_threshold = 0.15    # 15cm
        self.step_threshold = 0.3    # 30cm

        # Current strategy
        self.current_strategy = "ankle"
        self.support_foot = "left"  # Current support foot

    def select_balance_strategy(self, current_state):
        """
        Select appropriate balance strategy based on disturbance magnitude
        """
        com_pos = current_state['com_pos']
        zmp_pos = current_state['zmp_pos']
        support_polygon = current_state['support_polygon']

        # Calculate distance from ZMP to support polygon boundary
        zmp_x, zmp_y = zmp_pos[0], zmp_pos[1]

        dist_to_boundary_x = min(
            abs(zmp_x - support_polygon['min_x']),
            abs(zmp_x - support_polygon['max_x'])
        )

        dist_to_boundary_y = min(
            abs(zmp_y - support_polygon['min_y']),
            abs(zmp_y - support_polygon['max_y'])
        )

        min_dist_to_boundary = min(dist_to_boundary_x, dist_to_boundary_y)

        # Select strategy based on distance
        if min_dist_to_boundary > self.step_threshold:
            strategy = "ankle"  # Small disturbance
        elif min_dist_to_boundary > self.hip_threshold:
            strategy = "hip"    # Medium disturbance
        elif min_dist_to_boundary > self.ankle_threshold:
            strategy = "step"   # Large disturbance - prepare to step
        else:
            strategy = "emergency"  # Very large disturbance

        return strategy

    def execute_balance_strategy(self, strategy, current_state):
        """
        Execute the selected balance strategy
        """
        if strategy == "ankle":
            return self.ankle_strategy(current_state)
        elif strategy == "hip":
            return self.hip_strategy(current_state)
        elif strategy == "step":
            return self.step_strategy(current_state)
        elif strategy == "emergency":
            return self.emergency_strategy(current_state)
        else:
            return self.ankle_strategy(current_state)  # Default

    def ankle_strategy(self, current_state):
        """
        Ankle strategy: adjust ankle torques to maintain balance
        """
        # Calculate desired CoM position based on ZMP control
        com_pos = current_state['com_pos']
        com_vel = current_state['com_vel']

        # Use LIPM to calculate required ZMP
        desired_zmp = self.lipm_controller.calculate_zmp_from_com(com_pos, com_vel)

        # Convert ZMP to ankle torques
        ankle_torques = self.zmp_to_ankle_torques(desired_zmp, current_state)

        return {
            'joint_torques': self.distribute_ankle_torques(ankle_torques),
            'strategy': 'ankle',
            'zmp_ref': desired_zmp
        }

    def hip_strategy(self, current_state):
        """
        Hip strategy: use hip movements to control balance
        """
        # Calculate CoM control to move ZMP back to safe region
        com_pos = current_state['com_pos']
        com_vel = current_state['com_vel']

        # Calculate capture point
        capture_point = self.lipm_controller.calculate_capture_point(com_pos, com_vel)

        # Generate CoM trajectory to move capture point toward safe area
        safe_area_center = self.calculate_safe_area_center(current_state)

        # Create trajectory to move CoM toward safe area
        com_trajectory = self.generate_com_trajectory_to_safe_area(
            com_pos, safe_area_center
        )

        # Calculate required joint torques
        joint_torques = self.com_trajectory_to_joint_torques(com_trajectory, current_state)

        return {
            'joint_torques': joint_torques,
            'strategy': 'hip',
            'com_trajectory': com_trajectory
        }

    def step_strategy(self, current_state):
        """
        Step strategy: plan and execute a step to recover balance
        """
        # Calculate capture point
        com_pos = current_state['com_pos']
        com_vel = current_state['com_vel']

        capture_point = self.lipm_controller.calculate_capture_point(com_pos, com_vel)

        # Plan step to capture point
        next_support_foot_pos = self.plan_step_to_capture_point(
            capture_point, current_state
        )

        # Generate foot trajectory
        foot_trajectory = self.generate_foot_trajectory_to_point(
            next_support_foot_pos, current_state
        )

        # Execute step while maintaining balance
        control_commands = self.execute_step_with_balance(
            foot_trajectory, current_state
        )

        # Update support foot
        self.support_foot = "right" if self.support_foot == "left" else "left"

        return {
            'joint_torques': control_commands['joint_torques'],
            'foot_trajectory': foot_trajectory,
            'strategy': 'step',
            'next_support_foot': self.support_foot
        }

    def emergency_strategy(self, current_state):
        """
        Emergency strategy: prepare for fall or extreme measures
        """
        # Safety measures: bend knees, extend arms, prepare for impact
        emergency_torques = self.calculate_emergency_torques(current_state)

        return {
            'joint_torques': emergency_torques,
            'strategy': 'emergency',
            'safety_mode': True
        }

    def zmp_to_ankle_torques(self, desired_zmp, current_state):
        """
        Convert ZMP command to ankle torques
        """
        # Simplified conversion - in practice, this involves more complex calculations
        # relating ZMP deviation to required ankle torques
        current_zmp = current_state['current_zmp']

        zmp_error = desired_zmp - current_zmp[:2]

        # Proportional control to generate ankle torques
        ankle_torque_x = 500 * zmp_error[0]  # N-m per m error
        ankle_torque_y = 500 * zmp_error[1]  # N-m per m error

        return np.array([ankle_torque_x, ankle_torque_y])

    def distribute_ankle_torques(self, ankle_torques):
        """
        Distribute ankle torques to joint controllers
        """
        # This would convert ankle torques to individual joint torques
        # based on robot kinematics and control allocation
        n_joints = self.model.n_joints
        joint_torques = np.zeros(n_joints)

        # Simplified mapping - in practice, use full kinematic model
        # Map ankle torques to ankle joint torques
        ankle_indices = self.get_ankle_joint_indices()

        for i, idx in enumerate(ankle_indices):
            if i < len(ankle_torques):
                joint_torques[idx] = ankle_torques[i]

        return joint_torques

    def get_ankle_joint_indices(self):
        """
        Get indices of ankle joints in the joint array
        """
        # This depends on the specific robot joint ordering
        # Example: assuming ankle joints are at specific indices
        return [10, 11, 24, 25]  # Example indices for left and right ankle

    def calculate_safe_area_center(self, current_state):
        """
        Calculate center of safe area based on support polygon
        """
        support_polygon = current_state['support_polygon']

        safe_center_x = (support_polygon['min_x'] + support_polygon['max_x']) / 2
        safe_center_y = (support_polygon['min_y'] + support_polygon['max_y']) / 2

        return np.array([safe_center_x, safe_center_y, current_state['com_pos'][2]])

    def generate_com_trajectory_to_safe_area(self, current_com, safe_target):
        """
        Generate CoM trajectory to move toward safe area
        """
        # Use 5th order polynomial for smooth trajectory
        duration = 0.5  # 500ms to reach safe area
        dt = 0.01
        steps = int(duration / dt)

        trajectory = np.zeros((steps, 3))

        for i in range(3):  # For each axis
            start_pos = current_com[i]
            end_pos = safe_target[i]

            # 5th order polynomial coefficients
            coeffs = self.calculate_5th_order_coefficients(
                start_pos, 0, 0,  # start pos, vel, acc
                end_pos, 0, 0     # end pos, vel, acc
            )

            for j in range(steps):
                t = j * dt / duration  # normalized time [0, 1]
                trajectory[j, i] = (
                    coeffs[0] +
                    coeffs[1] * t +
                    coeffs[2] * t**2 +
                    coeffs[3] * t**3 +
                    coeffs[4] * t**4 +
                    coeffs[5] * t**5
                )

        return trajectory

    def calculate_5th_order_coefficients(self, x0, v0, a0, x1, v1, a1):
        """
        Calculate coefficients for 5th order polynomial
        """
        A = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, 1, 1, 1, 1, 1],
            [0, 1, 2, 3, 4, 5],
            [0, 0, 2, 6, 12, 20]
        ])

        b = np.array([x0, v0, a0, x1, v1, a1])
        coeffs = np.linalg.solve(A, b)

        return coeffs

    def plan_step_to_capture_point(self, capture_point, current_state):
        """
        Plan step location to capture point
        """
        current_pos = current_state['base_pos'][:2]

        # Calculate step direction and distance
        step_vector = capture_point - current_pos
        step_distance = np.linalg.norm(step_vector)

        if step_distance > 0:
            step_direction = step_vector / step_distance
            # Limit step size to maximum step length
            max_step_length = 0.4  # 40cm maximum step
            actual_step_length = min(step_distance, max_step_length)

            target_pos = current_pos + actual_step_length * step_direction
        else:
            target_pos = current_pos

        # Add small margin for stability
        target_pos[2] = 0  # Ground level

        return target_pos

    def generate_foot_trajectory_to_point(self, target_pos, current_state):
        """
        Generate foot trajectory to step location
        """
        current_foot_pos = current_state[f'{self.support_foot}_foot_pos']

        # Use polynomial trajectory
        duration = 0.4  # 400ms for step
        dt = 0.01
        steps = int(duration / dt)

        trajectory = np.zeros((steps, 6))  # Position and velocity for x, y, z

        for i in range(3):  # x, y, z
            if i < 2:  # Horizontal movement
                start_pos = current_foot_pos[i]
                end_pos = target_pos[i]
            else:  # Vertical movement
                start_pos = current_foot_pos[i]
                end_pos = target_pos[i] if i == 2 else start_pos

                # Add foot lift for step
                if np.linalg.norm(target_pos[:2] - current_foot_pos[:2]) > 0.05:
                    # Significant step - lift foot
                    end_pos = current_foot_pos[i] + 0.1  # Lift 10cm

            # Calculate trajectory
            coeffs = self.calculate_5th_order_coefficients(
                start_pos, 0, 0,  # start pos, vel, acc
                end_pos, 0, 0     # end pos, vel, acc
            )

            for j in range(steps):
                t = j * dt / duration
                trajectory[j, i] = (
                    coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 +
                    coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5
                )

        # Calculate velocities (derivatives)
        for i in range(3):
            for j in range(steps):
                if j > 0:
                    trajectory[j, i+3] = (trajectory[j, i] - trajectory[j-1, i]) / dt
                else:
                    trajectory[j, i+3] = 0  # Initial velocity

        return trajectory

    def execute_step_with_balance(self, foot_trajectory, current_state):
        """
        Execute step while maintaining balance
        """
        # This would coordinate the step with balance control
        # For now, return a simplified control command
        n_joints = self.model.n_joints
        joint_torques = np.zeros(n_joints)

        # Add balance torques while executing step
        balance_torques = self.calculate_balance_torques(current_state)
        step_torques = self.foot_trajectory_to_joint_torques(foot_trajectory[0])

        # Combine torques
        joint_torques = balance_torques + 0.5 * step_torques

        return {'joint_torques': joint_torques}

    def calculate_balance_torques(self, current_state):
        """
        Calculate torques for balance maintenance
        """
        # Use current balance strategy to calculate required torques
        com_pos = current_state['com_pos']
        com_vel = current_state['com_vel']

        desired_zmp = self.lipm_controller.calculate_zmp_from_com(com_pos, com_vel)
        ankle_torques = self.zmp_to_ankle_torques(desired_zmp, current_state)

        return self.distribute_ankle_torques(ankle_torques)

    def foot_trajectory_to_joint_torques(self, foot_state):
        """
        Convert desired foot state to joint torques
        """
        # This would use inverse dynamics or operational space control
        n_joints = self.model.n_joints
        return np.zeros(n_joints)  # Placeholder

    def calculate_emergency_torques(self, current_state):
        """
        Calculate emergency safety torques
        """
        n_joints = self.model.n_joints
        emergency_torques = np.zeros(n_joints)

        # Example: bend knees to lower CoM, extend arms for protection
        knee_indices = self.get_knee_joint_indices()
        arm_indices = self.get_arm_joint_indices()

        # Knee bending (flex joints)
        for idx in knee_indices:
            emergency_torques[idx] = 10  # Torque to flex knees

        # Arm extension for protection
        for idx in arm_indices:
            emergency_torques[idx] = 5  # Protective arm positioning

        return emergency_torques

    def get_knee_joint_indices(self):
        """
        Get indices of knee joints
        """
        # Example indices - would depend on specific robot
        return [8, 22]  # Left and right knee

    def get_arm_joint_indices(self):
        """
        Get indices of arm joints
        """
        # Example indices - would depend on specific robot
        return [2, 3, 4, 16, 17, 18]  # Both arm joints
```

## Advanced Balance Techniques

### Model Predictive Control (MPC) for Balance

```python
class MPCBalanceController:
    def __init__(self, prediction_horizon=20, dt=0.01):
        self.N = prediction_horizon  # Prediction horizon
        self.dt = dt
        self.Q = np.diag([100, 100, 10, 1, 1, 1])  # State cost (CoM pos/vel, ZMP)
        self.R = np.diag([1, 1, 0.1, 0.1])         # Control cost (joint velocities/torques)
        self.P = np.diag([500, 500, 50, 5, 5, 5])  # Terminal cost

    def solve_balance_mpc(self, current_state, reference_trajectory):
        """
        Solve MPC problem for balance control
        """
        # This would typically use a QP solver like OSQP or CVXOPT
        # For this example, we'll outline the structure

        # System matrices for LIPM (Linear Inverted Pendulum Model)
        A = self.get_system_matrix()
        B = self.get_input_matrix()

        # State: [com_x, com_y, com_x_dot, com_y_dot]
        current_x = np.array([
            current_state['com_pos'][0],
            current_state['com_pos'][1],
            current_state['com_vel'][0],
            current_state['com_vel'][1]
        ])

        # Initialize optimization variables
        X = np.zeros((4, self.N + 1))  # State trajectory
        U = np.zeros((2, self.N))      # Control trajectory (ZMP commands)

        X[:, 0] = current_x  # Initial state

        # Prediction loop
        for k in range(self.N):
            # Predict next state: x_{k+1} = A*x_k + B*u_k
            X[:, k+1] = A @ X[:, k] + B @ U[:, k]

            # Compute control based on reference tracking
            if k < len(reference_trajectory):
                ref_state = reference_trajectory[k]
            else:
                ref_state = reference_trajectory[-1]  # Hold last reference

            # Simple LQR control law (in practice, solve full QP)
            U[:, k] = self.compute_lqr_control(X[:, k], ref_state)

        # Return first control in sequence
        return U[:, 0]

    def get_system_matrix(self):
        """
        Get system matrix A for LIPM
        """
        # For LIPM: [ẍ] = ω² * [x - zmp_x]
        #          [ÿ] = ω² * [y - zmp_y]
        # Discretized form
        omega = 3.5  # Example value (sqrt(g/h))
        dt = self.dt

        A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [omega**2 * dt, 0, 1, 0],
            [0, omega**2 * dt, 0, 1]
        ])

        return A

    def get_input_matrix(self):
        """
        Get input matrix B for LIPM
        """
        omega = 3.5
        dt = self.dt

        B = np.array([
            [0, 0],
            [0, 0],
            [-omega**2 * dt, 0],
            [0, -omega**2 * dt]
        ])

        return B

    def compute_lqr_control(self, state, reference):
        """
        Compute LQR control law
        """
        # Solve Riccati equation offline, then apply feedback law
        # For this example, use simple gains
        Q = np.eye(4) * 10
        R_inv = 1.0

        # Approximate LQR gain
        K = np.array([[1, 0, 2, 0], [0, 1, 0, 2]])  # [k_x, k_y, k_xdot, k_ydot] for each dim

        error = reference[:4] - state
        control = R_inv * K @ error

        return control
```

### Compliance and Impedance Control

```python
class ComplianceBalanceController:
    def __init__(self, robot_model):
        self.model = robot_model

        # Impedance parameters for different joints
        self.impedance_params = {
            'ankle': {'M': 1.0, 'D': 10.0, 'K': 100.0},  # Mass, Damping, Stiffness
            'hip': {'M': 2.0, 'D': 15.0, 'K': 80.0},
            'knee': {'M': 1.5, 'D': 12.0, 'K': 90.0}
        }

    def compute_compliance_control(self, desired_pose, current_pose, external_force, joint_type='ankle'):
        """
        Compute compliance control for balance
        """
        params = self.impedance_params[joint_type]

        # Calculate pose error
        pose_error = desired_pose - current_pose

        # Calculate desired acceleration based on impedance model
        # M*ẍ + D*ẋ + K*x = F_external
        desired_acc = (external_force - params['D'] * pose_error[3:] - params['K'] * pose_error[:3]) / params['M']

        return desired_acc

    def adaptive_compliance(self, current_state, disturbance_level):
        """
        Adjust compliance based on disturbance level
        """
        # Increase compliance (decrease stiffness) for larger disturbances
        # to allow more flexible response
        base_params = self.impedance_params.copy()

        if disturbance_level > 0.5:  # Large disturbance
            # Decrease stiffness, increase damping for more compliance
            for joint_type in base_params:
                base_params[joint_type]['K'] *= 0.5  # Reduce stiffness
                base_params[joint_type]['D'] *= 1.5  # Increase damping
        elif disturbance_level < 0.1:  # Small disturbance
            # Increase stiffness for precise control
            for joint_type in base_params:
                base_params[joint_type]['K'] *= 1.5  # Increase stiffness

        return base_params
```

## Balance Recovery and Disturbance Handling

### Disturbance Observer

```python
class DisturbanceObserver:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.filter_coeff = 0.1  # Low-pass filter coefficient
        self.estimated_disturbance = np.zeros(6)  # 3D force + 3D torque
        self.disturbance_history = []

    def estimate_disturbance(self, measured_force_torque, expected_force_torque):
        """
        Estimate external disturbances using the difference between
        measured and expected forces/torques
        """
        # Calculate disturbance
        disturbance = measured_force_torque - expected_force_torque

        # Low-pass filter to reduce noise
        self.estimated_disturbance = (
            self.filter_coeff * disturbance +
            (1 - self.filter_coeff) * self.estimated_disturbance
        )

        # Store for history
        self.disturbance_history.append(self.estimated_disturbance.copy())
        if len(self.disturbance_history) > 100:  # Keep last 100 samples
            self.disturbance_history.pop(0)

        return self.estimated_disturbance

    def classify_disturbance(self, disturbance):
        """
        Classify the type and magnitude of disturbance
        """
        magnitude = np.linalg.norm(disturbance)

        if magnitude < 5:  # Small disturbance
            return {'type': 'small', 'magnitude': magnitude, 'direction': disturbance/magnitude if magnitude > 0 else np.zeros(6)}
        elif magnitude < 20:  # Medium disturbance
            return {'type': 'medium', 'magnitude': magnitude, 'direction': disturbance/magnitude if magnitude > 0 else np.zeros(6)}
        else:  # Large disturbance
            return {'type': 'large', 'magnitude': magnitude, 'direction': disturbance/magnitude if magnitude > 0 else np.zeros(6)}

    def predict_disturbance_effect(self, disturbance, robot_state):
        """
        Predict the effect of disturbance on robot balance
        """
        # Use simplified model to predict CoM displacement
        # This would involve more complex dynamics in practice
        com_inertia = 70  # 70kg effective mass
        predicted_com_acc = disturbance[:3] / com_inertia  # Only linear forces affect CoM directly

        # Calculate time to reach critical state
        current_com_vel = robot_state['com_vel']
        critical_velocity = 0.5  # m/s - velocity threshold for concern

        if np.linalg.norm(predicted_com_acc) > 0:
            time_to_critical = (critical_velocity - np.linalg.norm(current_com_vel)) / np.linalg.norm(predicted_com_acc)
        else:
            time_to_critical = float('inf')

        return {
            'predicted_acceleration': predicted_com_acc,
            'time_to_critical': time_to_critical,
            'risk_level': 'low' if time_to_critical > 1.0 else 'medium' if time_to_critical > 0.3 else 'high'
        }
```

### Balance Recovery Planning

```python
class BalanceRecoveryPlanner:
    def __init__(self, robot_model):
        self.model = robot_model
        self.capturability_map = self.precompute_capturability()

    def precompute_capturability(self):
        """
        Precompute capturability regions for different CoM states
        """
        # This would involve offline computation of capturability
        # for different CoM positions and velocities
        return {
            'max_recoverable_velocity': 0.8,  # m/s
            'max_recoverable_distance': 0.5,  # m
            'capture_time_constant': 0.3    # s
        }

    def plan_recovery_action(self, current_state, disturbance_info):
        """
        Plan balance recovery action based on current state and disturbance
        """
        com_pos = current_state['com_pos']
        com_vel = current_state['com_vel']

        # Calculate capture point
        omega = np.sqrt(9.81 / 0.8)  # sqrt(g/h) assuming 80cm CoM height
        capture_point = com_pos[:2] + com_vel[:2] / omega

        # Determine if recovery is possible
        if self.is_recoverable(capture_point, current_state):
            return self.plan_stable_recovery(current_state, capture_point)
        else:
            return self.plan_robust_recovery(current_state, capture_point)

    def is_recoverable(self, capture_point, current_state):
        """
        Check if balance recovery is possible from current state
        """
        # Check if capture point is within reachable area
        current_pos = current_state['base_pos'][:2]
        distance_to_capture = np.linalg.norm(capture_point - current_pos)

        # Check velocity is not too high
        com_velocity_norm = np.linalg.norm(current_state['com_vel'])

        # Recovery is possible if capture point is within reasonable distance
        # and velocity is below threshold
        return (distance_to_capture < self.capturability_map['max_recoverable_distance'] and
                com_velocity_norm < self.capturability_map['max_recoverable_velocity'])

    def plan_stable_recovery(self, current_state, capture_point):
        """
        Plan stable recovery when capture point is reachable
        """
        # Plan step to capture point
        step_target = self.adjust_step_for_capture_point(current_state, capture_point)

        # Generate smooth transition
        recovery_plan = {
            'type': 'stable_recovery',
            'step_target': step_target,
            'step_timing': 'next_opportunity',
            'balance_strategy': 'step_to_capture',
            'duration': 0.6  # 600ms for recovery step
        }

        return recovery_plan

    def plan_robust_recovery(self, current_state, capture_point):
        """
        Plan robust recovery for challenging situations
        """
        # If capture point is not reachable, plan alternative strategy
        current_pos = current_state['base_pos'][:2]

        # Find nearest safe location within reach
        direction_to_capture = capture_point - current_pos
        distance_to_capture = np.linalg.norm(direction_to_capture)

        if distance_to_capture > 0:
            direction = direction_to_capture / distance_to_capture
            max_reachable_distance = 0.4  # Maximum step length
            safe_target = current_pos + min(distance_to_capture, max_reachable_distance) * direction
        else:
            safe_target = current_pos

        recovery_plan = {
            'type': 'robust_recovery',
            'step_target': safe_target,
            'step_timing': 'immediate',
            'balance_strategy': 'max_effort_stabilization',
            'additional_actions': ['arm_swing', 'hip_moment'],
            'duration': 0.8  # Allow more time for challenging recovery
        }

        return recovery_plan

    def adjust_step_for_capture_point(self, current_state, capture_point):
        """
        Adjust step target to account for capture point and stability
        """
        current_pos = current_state['base_pos'][:2]

        # Calculate desired step direction
        desired_direction = capture_point - current_pos
        desired_distance = np.linalg.norm(desired_direction)

        if desired_distance > 0:
            desired_direction = desired_direction / desired_distance
        else:
            desired_direction = np.array([1, 0])  # Default direction

        # Limit step distance to maximum comfortable step
        max_step_length = 0.35  # 35cm maximum step
        actual_distance = min(desired_distance, max_step_length)

        # Add small safety margin
        target_pos = current_pos + actual_distance * desired_direction

        # Ensure target is on solid ground (simplified)
        target_pos[2] = 0  # Ground level

        return target_pos
```

## Implementation Example: Integrated Balance Controller

```python
class IntegratedBalanceController:
    def __init__(self, robot_model):
        self.model = robot_model
        self.state_estimator = StateEstimator()
        self.hierarchical_controller = HierarchicalBalanceController(robot_model)
        self.disturbance_observer = DisturbanceObserver()
        self.recovery_planner = BalanceRecoveryPlanner(robot_model)
        self.mpc_controller = MPCBalanceController()

        # Balance performance metrics
        self.balance_metrics = {
            'zmp_error_integral': 0,
            'com_deviation_integral': 0,
            'control_effort': 0,
            'recovery_count': 0
        }

    def balance_control_step(self, sensor_data, dt=0.01):
        """
        Main balance control step
        """
        # 1. Update state estimate
        current_state = self.update_state_estimate(sensor_data)

        # 2. Estimate disturbances
        disturbance_info = self.estimate_disturbances(current_state, sensor_data)

        # 3. Select balance strategy
        strategy = self.hierarchical_controller.select_balance_strategy(current_state)

        # 4. Execute balance strategy
        control_output = self.hierarchical_controller.execute_balance_strategy(
            strategy, current_state
        )

        # 5. Apply MPC if needed for enhanced performance
        if self.should_use_mpc(disturbance_info):
            mpc_correction = self.mpc_controller.solve_balance_mpc(
                current_state, self.generate_mpc_reference(current_state)
            )
            control_output['joint_torques'] += self.mpc_to_joint_torques(mpc_correction)

        # 6. Update performance metrics
        self.update_balance_metrics(current_state, control_output)

        return control_output

    def update_state_estimate(self, sensor_data):
        """
        Update state estimate from sensor data
        """
        # This would integrate sensor data using the state estimator
        # For now, return a simplified state
        return {
            'com_pos': sensor_data.get('com_pos', np.array([0, 0, 0.8])),
            'com_vel': sensor_data.get('com_vel', np.array([0, 0, 0])),
            'com_acc': sensor_data.get('com_acc', np.array([0, 0, 0])),
            'zmp_pos': sensor_data.get('zmp_pos', np.array([0, 0, 0])),
            'base_pos': sensor_data.get('base_pos', np.array([0, 0, 0, 1, 0, 0, 0])),  # [pos, quat]
            'base_vel': sensor_data.get('base_vel', np.array([0, 0, 0, 0, 0, 0])),  # [lin_vel, ang_vel]
            'joint_positions': sensor_data.get('joint_positions', np.zeros(28)),
            'joint_velocities': sensor_data.get('joint_velocities', np.zeros(28)),
            'left_foot_pos': sensor_data.get('left_foot_pos', np.array([0, 0.1, 0])),
            'right_foot_pos': sensor_data.get('right_foot_pos', np.array([0, -0.1, 0])),
            'support_polygon': self.calculate_support_polygon(sensor_data)
        }

    def estimate_disturbances(self, current_state, sensor_data):
        """
        Estimate external disturbances
        """
        # Calculate expected vs measured forces
        expected_forces = self.calculate_expected_forces(current_state)
        measured_forces = sensor_data.get('force_torque', np.zeros(6))

        disturbance = self.disturbance_observer.estimate_disturbance(
            measured_forces, expected_forces
        )

        return self.disturbance_observer.classify_disturbance(disturbance)

    def calculate_expected_forces(self, current_state):
        """
        Calculate expected forces based on robot dynamics
        """
        # Simplified calculation
        total_weight = 70 * 9.81  # 70kg robot
        return np.array([0, 0, -total_weight, 0, 0, 0])  # Gravity + no external forces

    def calculate_support_polygon(self, sensor_data):
        """
        Calculate support polygon from foot positions
        """
        left_foot = sensor_data.get('left_foot_pos', np.array([0, 0.1, 0]))
        right_foot = sensor_data.get('right_foot_pos', np.array([0, -0.1, 0]))

        return {
            'min_x': min(left_foot[0], right_foot[0]) - 0.02,  # Add small margin
            'max_x': max(left_foot[0], right_foot[0]) + 0.02,
            'min_y': min(left_foot[1], right_foot[1]) - 0.02,
            'max_y': max(left_foot[1], right_foot[1]) + 0.02
        }

    def should_use_mpc(self, disturbance_info):
        """
        Determine if MPC should be used based on disturbance level
        """
        return disturbance_info['magnitude'] > 10  # Use MPC for significant disturbances

    def generate_mpc_reference(self, current_state):
        """
        Generate reference trajectory for MPC
        """
        # Generate a short reference trajectory for MPC
        reference_trajectory = []
        for i in range(10):  # 10 steps
            t = i * 0.01
            # Simple reference: return to nominal CoM position
            ref_com = current_state['com_pos'].copy()
            ref_com[0] *= np.exp(-t)  # Exponentially decay to zero
            ref_com[1] *= np.exp(-t)
            reference_trajectory.append(ref_com)

        return reference_trajectory

    def mpc_to_joint_torques(self, mpc_control):
        """
        Convert MPC control to joint torques
        """
        # This would use inverse dynamics or other methods
        n_joints = self.model.n_joints
        return np.zeros(n_joints) * 0.1  # Small correction torques

    def update_balance_metrics(self, current_state, control_output):
        """
        Update balance performance metrics
        """
        # Calculate ZMP error
        zmp_error = np.linalg.norm(current_state['zmp_pos'][:2])
        self.balance_metrics['zmp_error_integral'] += zmp_error * 0.01

        # Calculate CoM deviation from nominal
        com_deviation = np.linalg.norm(current_state['com_pos'][:2])
        self.balance_metrics['com_deviation_integral'] += com_deviation * 0.01

        # Calculate control effort
        control_effort = np.sum(np.abs(control_output['joint_torques']))
        self.balance_metrics['control_effort'] += control_effort * 0.01

    def get_balance_performance(self):
        """
        Get current balance performance metrics
        """
        return self.balance_metrics.copy()
```

## Simulation and Testing

### Balance Control Testing Framework

```python
class BalanceControlTester:
    def __init__(self, robot_model, controller):
        self.model = robot_model
        self.controller = controller
        self.simulator = self.initialize_simulator()

    def initialize_simulator(self):
        """
        Initialize physics simulator for balance testing
        """
        return {
            'gravity': 9.81,
            'timestep': 0.001,  # Fine-grained simulation
            'total_time': 10.0,  # 10 seconds simulation
            'disturbance_times': [2.0, 4.0, 6.0, 8.0],  # Times to apply disturbances
            'disturbance_magnitudes': [50, 100, 75, 125]  # Force magnitudes (N)
        }

    def test_static_balance(self, initial_state):
        """
        Test static balance maintenance
        """
        print("Testing static balance...")

        # Apply no external disturbances
        simulation_data = self.run_simulation(initial_state, disturbances=[])

        # Analyze results
        stability_score = self.analyze_static_balance(simulation_data)

        return {
            'stability_score': stability_score,
            'max_com_deviation': np.max(np.abs(simulation_data['com_trajectory'][:, :2])),
            'average_zmp_error': np.mean(simulation_data['zmp_errors']),
            'success': stability_score > 0.8
        }

    def test_dynamic_balance(self, walking_pattern):
        """
        Test balance during dynamic motion (walking)
        """
        print("Testing dynamic balance during walking...")

        # Start with walking pattern
        initial_state = self.generate_walking_start_state()

        # Apply periodic disturbances during walking
        disturbances = self.generate_walking_disturbances()

        simulation_data = self.run_simulation(initial_state, disturbances)

        # Analyze results
        walking_stability = self.analyze_walking_balance(simulation_data)

        return {
            'walking_stability': walking_stability,
            'step_success_rate': self.calculate_step_success(simulation_data),
            'average_balance_effort': self.calculate_balance_effort(simulation_data),
            'success': walking_stability > 0.7
        }

    def test_disturbance_recovery(self, disturbance_magnitude=100):
        """
        Test recovery from external disturbances
        """
        print(f"Testing disturbance recovery (magnitude: {disturbance_magnitude}N)...")

        initial_state = self.generate_balanced_state()

        # Apply single large disturbance
        disturbances = [{'time': 1.0, 'force': [disturbance_magnitude, 0, 0], 'duration': 0.1}]

        simulation_data = self.run_simulation(initial_state, disturbances)

        # Analyze recovery performance
        recovery_performance = self.analyze_disturbance_recovery(simulation_data)

        return {
            'recovery_time': recovery_performance['recovery_time'],
            'recovery_accuracy': recovery_performance['accuracy'],
            'recovery_energy': recovery_performance['energy'],
            'success': recovery_performance['success']
        }

    def run_simulation(self, initial_state, disturbances):
        """
        Run full balance control simulation
        """
        sim_data = {
            'time': [],
            'states': [],
            'controls': [],
            'com_trajectory': [],
            'zmp_trajectory': [],
            'zmp_errors': [],
            'joint_torques': [],
            'disturbance_applied': []
        }

        state = initial_state.copy()
        sim_time = 0.0
        dt = 0.01  # Control timestep

        while sim_time < self.simulator['total_time']:
            # Apply disturbances if scheduled
            current_disturbance = self.get_current_disturbance(disturbances, sim_time)
            if current_disturbance is not None:
                state = self.apply_disturbance(state, current_disturbance)
                sim_data['disturbance_applied'].append((sim_time, current_disturbance))

            # Get sensor data (with noise simulation)
            sensor_data = self.generate_sensor_data(state, sim_time)

            # Run balance controller
            control_output = self.controller.balance_control_step(sensor_data, dt)

            # Apply control to robot dynamics
            state = self.update_robot_dynamics(state, control_output['joint_torques'], dt)

            # Log data
            sim_data['time'].append(sim_time)
            sim_data['states'].append(state.copy())
            sim_data['controls'].append(control_output.copy())
            sim_data['com_trajectory'].append(state['com_pos'].copy())
            sim_data['zmp_trajectory'].append(state['zmp_pos'].copy())

            # Calculate ZMP error
            support_polygon = state['support_polygon']
            zmp_pos = state['zmp_pos'][:2]
            zmp_error = self.calculate_zmp_error(zmp_pos, support_polygon)
            sim_data['zmp_errors'].append(zmp_error)

            sim_data['joint_torques'].append(control_output['joint_torques'].copy())

            sim_time += dt

        return sim_data

    def get_current_disturbance(self, disturbances, current_time):
        """
        Get disturbance to apply at current time
        """
        for dist in disturbances:
            if dist['time'] <= current_time < dist['time'] + dist.get('duration', 0.1):
                return dist
        return None

    def apply_disturbance(self, state, disturbance):
        """
        Apply external disturbance to robot state
        """
        # Apply force to CoM
        force = np.array(disturbance['force'])
        mass = 70  # 70kg robot
        acceleration = force / mass

        # Update CoM velocity
        state['com_vel'][:3] += acceleration * 0.01  # Apply for 1 control cycle

        return state

    def generate_sensor_data(self, state, time):
        """
        Generate sensor data with realistic noise
        """
        # Add realistic sensor noise
        noise_level = 0.001
        com_pos_noisy = state['com_pos'] + np.random.normal(0, noise_level, 3)
        com_vel_noisy = state['com_vel'] + np.random.normal(0, noise_level*10, 3)

        return {
            'com_pos': com_pos_noisy,
            'com_vel': com_vel_noisy,
            'zmp_pos': state['zmp_pos'],
            'base_pos': state['base_pos'],
            'base_vel': state['base_vel'],
            'joint_positions': state['joint_positions'] + np.random.normal(0, 0.0005, len(state['joint_positions'])),
            'joint_velocities': state['joint_velocities'],
            'left_foot_pos': state['left_foot_pos'],
            'right_foot_pos': state['right_foot_pos'],
            'force_torque': np.random.normal(0, 0.1, 6)  # Simulated force/torque noise
        }

    def update_robot_dynamics(self, state, joint_torques, dt):
        """
        Update robot dynamics with applied torques
        """
        # Simplified dynamics update
        # In practice, this would use full rigid body dynamics
        new_state = state.copy()

        # Apply joint torques and update joint velocities/positions
        joint_acc = joint_torques / 1.0  # Simplified inertia
        new_state['joint_velocities'] += joint_acc * dt
        new_state['joint_positions'] += new_state['joint_velocities'] * dt

        # Update CoM based on new configuration
        new_state['com_pos'] = self.model.calculate_com_position(new_state['joint_positions'])
        new_state['com_vel'] = self.model.calculate_com_velocity(
            new_state['joint_positions'], new_state['joint_velocities']
        )

        return new_state

    def calculate_zmp_error(self, zmp_pos, support_polygon):
        """
        Calculate ZMP error relative to support polygon
        """
        # Distance to closest boundary of support polygon
        errors = [
            abs(zmp_pos[0] - support_polygon['min_x']),
            abs(zmp_pos[0] - support_polygon['max_x']),
            abs(zmp_pos[1] - support_polygon['min_y']),
            abs(zmp_pos[1] - support_polygon['max_y'])
        ]

        return min(errors)  # Distance to nearest boundary

    def analyze_static_balance(self, sim_data):
        """
        Analyze static balance performance
        """
        zmp_errors = np.array(sim_data['zmp_errors'])
        com_deviations = np.array([np.linalg.norm(state['com_pos'][:2]) for state in sim_data['states']])

        # Calculate stability metrics
        avg_zmp_error = np.mean(zmp_errors)
        max_zmp_error = np.max(zmp_errors)
        avg_com_deviation = np.mean(com_deviations)

        # Stability score (lower is better)
        stability_score = 1.0 / (1.0 + avg_zmp_error + avg_com_deviation)

        return stability_score

    def analyze_walking_balance(self, sim_data):
        """
        Analyze balance during walking
        """
        # Calculate metrics specific to walking balance
        pass

    def calculate_step_success(self, sim_data):
        """
        Calculate step success rate
        """
        pass

    def calculate_balance_effort(self, sim_data):
        """
        Calculate average balance control effort
        """
        joint_torques = np.array(sim_data['joint_torques'])
        avg_effort = np.mean(np.abs(joint_torques))
        return avg_effort

    def analyze_disturbance_recovery(self, sim_data):
        """
        Analyze disturbance recovery performance
        """
        # Find when robot recovered to stable state
        zmp_errors = np.array(sim_data['zmp_errors'])
        com_trajectory = np.array(sim_data['com_trajectory'])

        # Find recovery time (when ZMP error returns to acceptable range)
        recovery_threshold = 0.05  # 5cm from support polygon edge
        recovery_time_idx = None

        for i, error in enumerate(zmp_errors):
            if error < recovery_threshold:
                recovery_time_idx = i
                break

        if recovery_time_idx is not None:
            recovery_time = sim_data['time'][recovery_time_idx]
            recovery_accuracy = zmp_errors[recovery_time_idx]
        else:
            recovery_time = self.simulator['total_time']  # Did not recover
            recovery_accuracy = np.max(zmp_errors)

        # Calculate energy used for recovery
        joint_torques = np.array(sim_data['joint_torques'])
        recovery_energy = np.sum(np.abs(joint_torques[:recovery_time_idx])) if recovery_time_idx else float('inf')

        return {
            'recovery_time': recovery_time,
            'accuracy': recovery_accuracy,
            'energy': recovery_energy,
            'success': recovery_time < self.simulator['total_time'] * 0.8  # Recovered in time
        }
```

## Summary

Balance and postural control are critical capabilities for humanoid robots, enabling them to maintain stability during both static and dynamic activities. The key components covered in this section include:

1. **State Estimation**: Accurate estimation of robot state using sensor fusion
2. **Balance Control Strategies**: Ankle, hip, and stepping strategies for different disturbance levels
3. **Mathematical Models**: LIPM, Cart-Table, and other models for balance control
4. **Advanced Techniques**: MPC, compliance control, and disturbance handling
5. **Recovery Planning**: Proactive planning for balance recovery

The balance control system must be able to:
- Rapidly detect balance disturbances
- Select appropriate control strategies based on disturbance magnitude
- Coordinate multiple joints to maintain stability
- Plan and execute recovery actions when necessary
- Adapt to changing conditions and environments

Effective balance control requires careful integration of these components, along with extensive testing and tuning to achieve stable and robust performance. The next section will cover manipulation and grasping techniques for humanoid robots.
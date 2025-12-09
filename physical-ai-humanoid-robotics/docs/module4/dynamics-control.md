# Dynamics and Control

## Introduction to Humanoid Robot Dynamics

Dynamics in humanoid robotics deals with the forces and torques that cause motion, as opposed to kinematics which only describes motion. For humanoid robots, dynamics is crucial for understanding how the robot responds to control inputs, external forces, and environmental interactions. Proper dynamic modeling and control are essential for stable walking, manipulation, and interaction with the environment.

## Mathematical Foundations of Dynamics

### Newton-Euler Formulation

The Newton-Euler equations form the basis for rigid body dynamics:

**Translational motion:**
```
F = ma
```

**Rotational motion:**
```
τ = Iα + ω × (Iω)
```

Where:
- `F` is the force vector
- `m` is the mass
- `a` is the linear acceleration
- `τ` is the torque vector
- `I` is the inertia tensor
- `α` is the angular acceleration
- `ω` is the angular velocity

### Lagrangian Formulation

For complex multi-body systems like humanoid robots, the Lagrangian formulation is often more convenient:

```
L = T - V
```

Where:
- `L` is the Lagrangian
- `T` is the kinetic energy
- `V` is the potential energy

The equations of motion are given by the Euler-Lagrange equations:

```
d/dt(∂L/∂q̇) - ∂L/∂q = τ
```

Where:
- `q` is the vector of generalized coordinates (joint angles)
- `q̇` is the vector of generalized velocities (joint velocities)
- `τ` is the vector of generalized forces (joint torques)

### Generalized Equation of Motion

For a humanoid robot with `n` degrees of freedom, the equation of motion can be written as:

```
M(q)q̈ + C(q,q̇)q̇ + g(q) = τ + J^T(q)F_ext
```

Where:
- `M(q)` is the mass/inertia matrix
- `C(q,q̇)` is the Coriolis and centrifugal matrix
- `g(q)` is the gravity vector
- `τ` is the vector of joint torques
- `J(q)` is the Jacobian matrix
- `F_ext` is the vector of external forces

## Derivation of Humanoid Dynamics

### Mass Matrix (M(q))

The mass matrix represents the inertial properties of the robot:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def calculate_mass_matrix(robot_kinematics, joint_angles, link_masses, link_inertias):
    """
    Calculate mass matrix using the Composite Rigid Body Algorithm (CRBA)
    """
    n = len(joint_angles)
    M = np.zeros((n, n))

    # Calculate transforms for each link
    transforms = []
    for i in range(n):
        # Calculate transform from base to link i
        T = robot_kinematics.forward_kinematics_partial(joint_angles[:i+1], i)
        transforms.append(T)

    # For each pair of joints, calculate their inertial coupling
    for i in range(n):
        for j in range(i, n):
            # Calculate the spatial inertia of the subtree from joint j
            subtree_inertia = calculate_subtree_inertia(j, transforms, link_masses, link_inertias)

            # Calculate the Jacobian for joint i
            Ji = calculate_jacobian_link(robot_kinematics, joint_angles, i)

            # Calculate the Jacobian for joint j
            Jj = calculate_jacobian_link(robot_kinematics, joint_angles, j)

            # Calculate the coupling term
            M[i, j] = Ji.T @ subtree_inertia @ Jj

            # Matrix is symmetric
            if i != j:
                M[j, i] = M[i, j]

    return M

def calculate_subtree_inertia(link_idx, transforms, link_masses, link_inertias):
    """
    Calculate the composite inertia of a subtree starting from link_idx
    """
    # This is a simplified version - full implementation would be more complex
    total_inertia = np.zeros((6, 6))  # Spatial inertia (3 linear + 3 angular)

    # Add inertia of the link itself
    link_transform = transforms[link_idx]
    link_inertia_6d = spatial_inertia(link_masses[link_idx], link_inertias[link_idx], link_transform)

    total_inertia += link_inertia_6d

    # Add inertias of all child links (simplified)
    # In a full implementation, this would recursively include all descendant links

    return total_inertia

def spatial_inertia(mass, inertia_3d, transform):
    """
    Convert 3D inertia to 6D spatial inertia at a given transform
    """
    # Extract rotation and translation from transform
    R = transform[:3, :3]
    p = transform[:3, 3]

    # Convert 3D inertia to spatial inertia
    # This is a simplified representation
    spatial_I = np.zeros((6, 6))

    # Mass term
    spatial_I[0:3, 0:3] = mass * np.eye(3)

    # Inertia term
    I_rotated = R @ inertia_3d @ R.T
    spatial_I[3:6, 3:6] = I_rotated

    return spatial_I
```

### Coriolis and Centrifugal Matrix (C(q,q̇))

The Coriolis and centrifugal forces arise from velocity-dependent terms:

```python
def calculate_coriolis_matrix(robot_kinematics, joint_angles, joint_velocities, link_masses):
    """
    Calculate Coriolis and centrifugal matrix
    """
    n = len(joint_angles)
    C = np.zeros((n, n))

    # Use Christoffel symbols of the first kind
    M = calculate_mass_matrix(robot_kinematics, joint_angles, link_masses,
                              [np.eye(3) for _ in link_masses])  # Simplified inertia

    for i in range(n):
        for j in range(n):
            c_sum = 0
            for k in range(n):
                # Christoffel symbol: Γ^i_jk = 0.5 * (∂M_ik/∂q_j + ∂M_jk/∂q_i - ∂M_ij/∂q_k)
                gamma = 0.5 * (partial_derivative_M(M, i, k, j, joint_angles) +
                              partial_derivative_M(M, j, k, i, joint_angles) -
                              partial_derivative_M(M, i, j, k, joint_angles))
                c_sum += gamma * joint_velocities[k]
            C[i, j] = c_sum

    return C

def partial_derivative_M(M, i, j, k, joint_angles, h=1e-6):
    """
    Numerical approximation of partial derivative of mass matrix
    """
    # This is a simplified placeholder - in practice, analytical derivatives are preferred
    angles_plus = joint_angles.copy()
    angles_plus[k] += h
    M_plus = calculate_mass_matrix_partial(angles_plus)  # Simplified function

    angles_minus = joint_angles.copy()
    angles_minus[k] -= h
    M_minus = calculate_mass_matrix_partial(angles_minus)  # Simplified function

    return (M_plus[i, j] - M_minus[i, j]) / (2 * h)
```

### Gravity Vector (g(q))

The gravity vector accounts for gravitational forces acting on each link:

```python
def calculate_gravity_vector(robot_kinematics, joint_angles, link_masses, gravity=[0, 0, -9.81]):
    """
    Calculate gravity vector for the robot
    """
    n = len(joint_angles)
    g = np.zeros(n)

    gravity_vec = np.array(gravity)

    for i in range(n):
        # Calculate Jacobian for link i
        Ji = calculate_jacobian_link(robot_kinematics, joint_angles, i)

        # Calculate position of link i's center of mass
        pos_com = calculate_link_com_position(robot_kinematics, joint_angles, i)

        # Calculate gravitational force on this link
        F_gravity = link_masses[i] * gravity_vec

        # Project gravitational force onto joint i
        g[i] = Ji[:3, i].T @ F_gravity  # Only linear part affects joint torque

    return g

def calculate_link_com_position(robot_kinematics, joint_angles, link_idx):
    """
    Calculate the position of the center of mass of a link
    """
    # This would use forward kinematics to find link COM position
    T = robot_kinematics.forward_kinematics_partial(joint_angles[:link_idx+1], link_idx)
    # Add offset from joint to COM
    com_offset = np.array([0, 0, 0])  # Simplified - would come from URDF
    return T[:3, 3] + T[:3, :3] @ com_offset
```

## Control Methods for Humanoid Robots

### Computed Torque Control (Inverse Dynamics Control)

Computed torque control linearizes the robot dynamics:

```python
class ComputedTorqueController:
    def __init__(self, robot_model):
        self.model = robot_model
        self.Kp = np.diag([100, 100, 100, 50, 50, 50])  # Proportional gains
        self.Kd = np.diag([20, 20, 20, 10, 10, 10])     # Derivative gains

    def compute_control_torque(self, q, qd, q_desired, qd_desired, qdd_desired):
        """
        Compute control torque using computed torque method
        """
        # Calculate current dynamics
        M = self.model.mass_matrix(q)
        C = self.model.coriolis_matrix(q, qd)
        g = self.model.gravity_vector(q)

        # Calculate control law
        # τ = M(q) * (qdd_d + Kd*(qd_d - qd) + Kp*(q_d - q)) + C(q,qd)*qd + g(q)
        tracking_error = q_desired - q
        velocity_error = qd_desired - qd

        feedforward_term = M @ qdd_desired
        feedback_term = M @ (self.Kd @ velocity_error + self.Kp @ tracking_error)
        coriolis_gravity_term = C @ qd + g

        tau = feedforward_term + feedback_term + coriolis_gravity_term

        return tau
```

### Operational Space Control

Operational space control allows control in task space (end-effector space):

```python
class OperationalSpaceController:
    def __init__(self, robot_model):
        self.model = robot_model

    def compute_task_space_control(self, q, qd, x_desired, xd_desired, xdd_desired,
                                   task_jacobian, task_inertia):
        """
        Compute control in operational space
        """
        # Calculate joint-space mass matrix
        M = self.model.mass_matrix(q)

        # Calculate operational space inertia
        # Lambda = (J * M^(-1) * J^T)^(-1)
        J = task_jacobian
        Lambda = np.linalg.inv(J @ np.linalg.inv(M) @ J.T)

        # Calculate operational space Coriolis and gravity terms
        # This is simplified - full implementation is more complex
        h_op = self.calculate_operational_coriolis_gravity(q, qd, J, M)

        # Calculate operational space error
        x_error = x_desired - self.get_current_task_position(q)
        xd_error = xd_desired - self.get_current_task_velocity(q, qd)

        # Control law in operational space
        # F_task = Lambda * (xdd_d + Kd*xd_error + Kp*x_error) + h_op
        Kp = 100 * np.eye(len(x_desired))  # Position gains
        Kd = 20 * np.eye(len(x_desired))   # Velocity gains

        F_task = (Lambda @ (xdd_desired + Kd @ xd_error + Kp @ x_error) + h_op)

        # Convert to joint torques
        # τ = J^T * F_task + N^T * τ_null
        # where N is the null space projector
        tau = J.T @ F_task

        # Add null space motion to avoid joint limits
        tau += self.compute_null_space_motion(q, qd, M, J)

        return tau

    def calculate_operational_coriolis_gravity(self, q, qd, J, M):
        """
        Calculate Coriolis and gravity terms in operational space
        """
        # This is a simplified representation
        # Full implementation requires careful calculation of dJ/dt
        return np.zeros(len(J))

    def compute_null_space_motion(self, q, qd, M, J):
        """
        Compute motion in null space to achieve secondary objectives
        """
        # Null space projector: N = I - M^(-1) * J^T * Lambda * J
        Lambda = np.linalg.inv(J @ np.linalg.inv(M) @ J.T)
        N = np.eye(len(q)) - np.linalg.inv(M) @ J.T @ Lambda @ J

        # Desired null space motion (e.g., toward joint centers)
        q_center = np.zeros(len(q))  # Joint center positions
        K_null = 1.0  # Null space gain
        q_null_desired = K_null * (q_center - q)

        # Apply null space motion
        tau_null = N.T @ q_null_desired

        return tau_null
```

### Impedance Control

Impedance control makes the robot behave like a mechanical system with desired stiffness, damping, and inertia:

```python
class ImpedanceController:
    def __init__(self, desired_mass, desired_damping, desired_stiffness):
        self.M_d = desired_mass      # Desired inertia
        self.D_d = desired_damping   # Desired damping
        self.K_d = desired_stiffness # Desired stiffness

    def compute_impedance_control(self, x, xd, x_desired, xd_desired, xdd_desired,
                                  jacobian, mass_matrix):
        """
        Compute impedance control law
        """
        # Calculate position and velocity errors
        x_error = x_desired - x
        xd_error = xd_desired - xd

        # Impedance control law in task space
        # M_d * (xdd_d - xdd) + D_d * (xd_d - xd) + K_d * (x_d - x) = F_ext
        F_impedance = (self.M_d @ (xdd_desired - self.calculate_xdd(x, xd)) +
                      self.D_d @ xd_error +
                      self.K_d @ x_error)

        # Convert to joint space
        tau = jacobian.T @ F_impedance

        return tau

    def calculate_xdd(self, x, xd, dt=0.01):
        """
        Calculate acceleration from position and velocity
        """
        # This would use numerical differentiation or state estimation
        return np.zeros_like(x)
```

## Walking Pattern Generation

### Zero Moment Point (ZMP) Based Control

ZMP is crucial for stable bipedal walking:

```python
class ZMPController:
    def __init__(self, robot_height, gravity=9.81):
        self.h = robot_height  # Height of center of mass
        self.g = gravity

    def calculate_zmp_from_com(self, com_pos, com_vel, com_acc):
        """
        Calculate ZMP from center of mass trajectory
        """
        x_com, y_com = com_pos[:2]
        x_com_dot, y_com_dot = com_vel[:2]
        x_com_ddot, y_com_ddot = com_acc[:2]

        # ZMP equations (simplified for 2D)
        zmp_x = x_com - (self.h / self.g) * x_com_ddot
        zmp_y = y_com - (self.h / self.g) * y_com_ddot

        return np.array([zmp_x, zmp_y, 0])

    def generate_com_trajectory(self, zmp_trajectory, initial_com, duration, dt=0.01):
        """
        Generate CoM trajectory from desired ZMP trajectory
        """
        timesteps = int(duration / dt)
        com_trajectory = np.zeros((timesteps, 3))
        com_velocity = np.zeros((timesteps, 3))
        com_acceleration = np.zeros((timesteps, 3))

        # Initial conditions
        com_pos = initial_com.copy()
        com_vel = np.zeros(3)

        for i in range(timesteps):
            t = i * dt

            # Get desired ZMP at current time
            zmp_desired = self.interpolate_zmp_trajectory(zmp_trajectory, t)

            # Calculate CoM acceleration from ZMP
            # Using inverted pendulum model: (ẍ, ÿ) = g/h * (com - zmp)
            com_acc[0] = (self.g / self.h) * (com_pos[0] - zmp_desired[0])
            com_acc[1] = (self.g / self.h) * (com_pos[1] - zmp_desired[1])
            com_acc[2] = 0  # Assume constant height

            # Integrate to get velocity and position
            com_vel += com_acc * dt
            com_pos += com_vel * dt

            com_trajectory[i] = com_pos.copy()
            com_velocity[i] = com_vel.copy()
            com_acceleration[i] = com_acc.copy()

        return com_trajectory, com_velocity, com_acceleration

    def interpolate_zmp_trajectory(self, zmp_trajectory, time):
        """
        Interpolate ZMP trajectory at given time
        """
        # Simplified linear interpolation
        return zmp_trajectory[0]  # Placeholder
```

### Preview Control

Preview control uses future ZMP reference to improve tracking:

```python
class PreviewController:
    def __init__(self, robot_height, preview_horizon=2.0, dt=0.01):
        self.h = robot_height
        self.preview_horizon = preview_horizon
        self.dt = dt
        self.N = int(preview_horizon / dt)

        # Calculate preview control gain
        self.K_preview = self.calculate_preview_gain()

    def calculate_preview_gain(self):
        """
        Calculate preview control gain matrix
        """
        # This involves solving Riccati equations and calculating gains
        # Simplified implementation
        A_c = np.array([[0, 1], [self.g/self.h, 0]])  # Continuous system matrix
        B_c = np.array([[0], [-self.g/self.h]])      # Continuous input matrix

        # Discretize the system
        A_d = np.eye(2) + A_c * self.dt
        B_d = B_c * self.dt

        # For simplicity, return a basic gain matrix
        # Full implementation would solve the discrete-time Riccati equation
        return np.array([1.0, 0.1])

    def compute_control(self, current_state, future_zmp_reference):
        """
        Compute control using preview of future ZMP reference
        """
        # current_state = [com_x, com_x_dot]
        # future_zmp_reference = array of future ZMP values

        # Implement preview control law
        control = 0

        # Feedback term
        feedback_gain = np.array([10, 2])  # [position, velocity gains]
        tracking_error = current_state  # Simplified
        control += feedback_gain @ tracking_error

        # Preview term (sum of future reference weighted by preview gains)
        for k in range(min(self.N, len(future_zmp_reference))):
            preview_gain = self.calculate_preview_gain_for_step(k)
            control += preview_gain * future_zmp_reference[k]

        return control

    def calculate_preview_gain_for_step(self, k):
        """
        Calculate preview gain for k steps ahead
        """
        # This would come from the solution of the Riccati equation
        # Simplified implementation
        return 0.01 * np.exp(-k * 0.1)  # Decaying gain
```

## Balance Control

### Linear Inverted Pendulum Model (LIPM)

The LIPM is commonly used for humanoid balance control:

```python
class LIPMController:
    def __init__(self, com_height, gravity=9.81):
        self.h = com_height
        self.g = gravity
        self.omega = np.sqrt(self.g / self.h)

    def calculate_capture_point(self, com_pos, com_vel):
        """
        Calculate capture point for balance recovery
        """
        # Capture point: where to step to stop the CoM
        capture_point = com_pos + com_vel / self.omega
        return capture_point

    def generate_balance_trajectory(self, current_com, current_com_vel, target_com, duration, dt=0.01):
        """
        Generate CoM trajectory for balance control
        """
        timesteps = int(duration / dt)
        trajectory = np.zeros((timesteps, 3))

        for i in range(timesteps):
            t = i * dt
            progress = t / duration

            # Exponential decay toward target
            remaining_time = duration - t
            decay_factor = np.exp(-self.omega * remaining_time)

            # Calculate current CoM position
            current_pos = (1 - decay_factor) * target_com + decay_factor * current_com + \
                         decay_factor / self.omega * current_com_vel

            trajectory[i] = current_pos

        return trajectory
```

### Whole-Body Control

For complex humanoid robots, whole-body control considers all tasks simultaneously:

```python
class WholeBodyController:
    def __init__(self, robot_model):
        self.model = robot_model
        self.n_joints = robot_model.n_joints
        self.n_contacts = 4  # Assuming 2 feet contacts (3 DOF each = 6, simplified to 4)

    def compute_whole_body_control(self, tasks, contact_points, external_forces=None):
        """
        Compute whole-body control considering multiple tasks and contacts
        """
        # tasks: list of (task_type, task_value, priority, weight)
        # contact_points: points where robot contacts environment

        # Formulate as optimization problem:
        # min ||Ax - b||^2 + reg_term
        # subject to: Cx = d (equality constraints)

        # Build task hierarchy
        A_tasks, b_tasks = self.build_task_jacobians(tasks)
        A_contacts, b_contacts = self.build_contact_constraints(contact_points)

        # Combine all constraints
        A_total = np.vstack([A_tasks, A_contacts])
        b_total = np.hstack([b_tasks, b_contacts])

        # Solve using weighted least squares
        weights = self.calculate_task_weights(tasks)
        W = np.diag(weights)

        # Regularized solution
        reg_param = 0.001
        I_reg = reg_param * np.eye(A_total.shape[1])

        # Solve: (A^T * W * A + I_reg) * x = A^T * W * b
        AtWA = A_total.T @ W @ A_total + I_reg
        AtWb = A_total.T @ W @ b_total

        solution = np.linalg.solve(AtWA, AtWb)

        return solution

    def build_task_jacobians(self, tasks):
        """
        Build Jacobian matrix for all tasks
        """
        total_rows = sum(len(task[1]) for task in tasks)  # Size of all task errors
        A = np.zeros((total_rows, self.n_joints + 6 * self.n_contacts))  # +6 for contact forces
        b = np.zeros(total_rows)

        row_idx = 0
        for task_type, task_value, priority, weight in tasks:
            if task_type == "end_effector_pose":
                jacobian = self.get_end_effector_jacobian(task_value['link_name'])
                task_error = self.calculate_pose_error(task_value['desired'], task_value['current'])

            elif task_type == "com_position":
                jacobian = self.get_com_jacobian()
                task_error = task_value['desired'] - task_value['current']

            elif task_type == "posture":
                jacobian = self.get_posture_jacobian()
                task_error = task_value['desired'] - task_value['current']

            # Add to matrices
            A[row_idx:row_idx+len(task_error), :self.n_joints] = jacobian
            b[row_idx:row_idx+len(task_error)] = task_error
            row_idx += len(task_error)

        return A, b

    def build_contact_constraints(self, contact_points):
        """
        Build equality constraints for contact points
        """
        # No sliding, no penetration constraints
        n_constraints = len(contact_points) * 3  # 3 DOF per contact point
        A = np.zeros((n_constraints, self.n_joints + 6 * len(contact_points)))
        b = np.zeros(n_constraints)

        for i, contact in enumerate(contact_points):
            # Contact point should have zero velocity (no sliding)
            # and appropriate force constraints
            start_row = i * 3
            contact_jacobian = self.get_contact_jacobian(contact['link_name'], contact['point'])

            # Velocity constraint: J * qdot = 0
            A[start_row:start_row+3, :self.n_joints] = contact_jacobian

        return A, b

    def calculate_task_weights(self, tasks):
        """
        Calculate weights based on task priorities
        """
        total_size = sum(len(task[1]) for task in tasks)
        weights = np.ones(total_size)
        current_idx = 0

        for task_type, task_value, priority, weight in tasks:
            task_size = len(task_value) if isinstance(task_value, (list, np.ndarray)) else 3
            weights[current_idx:current_idx+task_size] = weight
            current_idx += task_size

        return weights
```

## Control Implementation Example

```python
class HumanoidBalanceController:
    def __init__(self, robot_model):
        self.model = robot_model
        self.zmp_controller = ZMPController(robot_height=0.8)
        self.lipm_controller = LIPMController(com_height=0.8)
        self.whole_body_controller = WholeBodyController(robot_model)

        # State estimators
        self.state_estimator = self.initialize_state_estimator()

    def control_step(self, sensor_data, dt):
        """
        Main control step for humanoid balance
        """
        # 1. State estimation
        current_state = self.estimate_state(sensor_data)

        # 2. Balance planning
        desired_zmp = self.plan_balance(current_state)

        # 3. Trajectory generation
        com_trajectory = self.generate_com_trajectory(desired_zmp, current_state['com'])

        # 4. Whole-body control
        control_commands = self.compute_control_commands(
            current_state, com_trajectory, sensor_data
        )

        # 5. Apply control
        self.send_control_commands(control_commands)

        return control_commands

    def estimate_state(self, sensor_data):
        """
        Estimate robot state from sensor data
        """
        # Combine IMU, joint encoders, force/torque sensors
        com_pos = self.estimate_com_position(sensor_data)
        com_vel = self.estimate_com_velocity(sensor_data)
        com_acc = self.estimate_com_acceleration(sensor_data)

        joint_positions = sensor_data['joint_positions']
        joint_velocities = sensor_data['joint_velocities']

        # Foot contact states
        left_foot_contact = sensor_data['left_foot_force'] > 10  # Threshold
        right_foot_contact = sensor_data['right_foot_force'] > 10

        return {
            'com': com_pos,
            'com_vel': com_vel,
            'com_acc': com_acc,
            'joint_positions': joint_positions,
            'joint_velocities': joint_velocities,
            'left_foot_contact': left_foot_contact,
            'right_foot_contact': right_foot_contact
        }

    def plan_balance(self, current_state):
        """
        Plan balance strategy based on current state
        """
        # Calculate current ZMP
        current_zmp = self.zmp_controller.calculate_zmp_from_com(
            current_state['com'],
            current_state['com_vel'],
            current_state['com_acc']
        )

        # Calculate capture point
        capture_point = self.lipm_controller.calculate_capture_point(
            current_state['com'][:2],
            current_state['com_vel'][:2]
        )

        # Determine balance strategy
        support_polygon = self.calculate_support_polygon(current_state)

        if not self.is_zmp_in_support(current_zmp, support_polygon):
            # Generate recovery ZMP trajectory
            recovery_zmp = self.generate_recovery_trajectory(current_zmp, capture_point)
            return recovery_zmp
        else:
            # Maintain current balance
            return current_zmp

    def generate_recovery_trajectory(self, current_zmp, capture_point):
        """
        Generate ZMP trajectory for balance recovery
        """
        # Move ZMP toward capture point to initiate recovery step
        step_duration = 0.8  # 800ms step
        dt = 0.01
        timesteps = int(step_duration / dt)

        recovery_trajectory = np.zeros((timesteps, 3))

        for i in range(timesteps):
            t = i * dt
            # Smooth transition from current ZMP to capture point
            alpha = 0.5 * (1 - np.cos(np.pi * t / step_duration))  # S-curve
            recovery_trajectory[i] = (1 - alpha) * current_zmp + alpha * np.append(capture_point, [0])

        return recovery_trajectory

    def compute_control_commands(self, current_state, com_trajectory, sensor_data):
        """
        Compute final control commands
        """
        # Define tasks with priorities
        tasks = [
            # High priority: maintain CoM trajectory
            ('com_position', {
                'desired': com_trajectory[0],  # Next desired CoM position
                'current': current_state['com']
            }, 1, 100.0),

            # Medium priority: maintain foot positions
            ('left_foot_position', {
                'desired': self.get_left_foot_target(),
                'current': self.get_current_left_foot_position()
            }, 2, 50.0),

            # Low priority: joint posture
            ('posture', {
                'desired': self.get_nominal_joint_positions(),
                'current': current_state['joint_positions']
            }, 3, 1.0)
        ]

        # Define contact constraints
        contact_points = [
            {'link_name': 'left_foot', 'point': [0, 0, 0]},
            {'link_name': 'right_foot', 'point': [0, 0, 0]}
        ]

        # Compute whole-body control
        control_solution = self.whole_body_controller.compute_whole_body_control(
            tasks, contact_points
        )

        # Extract joint torques (first n_joints elements)
        joint_torques = control_solution[:self.model.n_joints]

        # Extract contact forces (remaining elements)
        contact_forces = control_solution[self.model.n_joints:]

        return {
            'joint_torques': joint_torques,
            'contact_forces': contact_forces
        }

    def send_control_commands(self, control_commands):
        """
        Send control commands to robot hardware
        """
        # Interface with robot's low-level controllers
        # This would typically use ROS messages, EtherCAT, or other communication protocols
        joint_torques = control_commands['joint_torques']

        # Apply joint limits and safety checks
        limited_torques = self.apply_joint_limits(joint_torques)

        # Send to hardware
        self.robot_interface.send_joint_commands(limited_torques)

    def apply_joint_limits(self, torques):
        """
        Apply safety limits to control commands
        """
        max_torque = 100.0  # Example limit (Nm)
        return np.clip(torques, -max_torque, max_torque)
```

## Advanced Control Techniques

### Model Predictive Control (MPC)

MPC is particularly useful for humanoid robots due to its ability to handle constraints:

```python
class ModelPredictiveController:
    def __init__(self, prediction_horizon=10, dt=0.01):
        self.N = prediction_horizon
        self.dt = dt
        self.Q = np.eye(6) * 10  # State cost matrix
        self.R = np.eye(3) * 0.1  # Control cost matrix
        self.P = np.eye(6) * 50  # Terminal cost matrix

    def solve_mpc(self, current_state, reference_trajectory):
        """
        Solve MPC optimization problem
        """
        # This would typically use a QP solver like OSQP or CVXOPT
        # For this example, we'll outline the structure

        # Predict system evolution over horizon
        predicted_states = []
        control_sequence = []

        state = current_state.copy()

        for k in range(self.N):
            # Calculate optimal control for this step
            ref_k = reference_trajectory[k] if k < len(reference_trajectory) else reference_trajectory[-1]

            # Simple LQR solution (in practice, this would be a constrained QP)
            control = self.calculate_lqr_control(state, ref_k)

            # Apply control and predict next state
            state = self.predict_next_state(state, control)
            predicted_states.append(state.copy())
            control_sequence.append(control.copy())

        return control_sequence[0]  # Return first control in sequence

    def calculate_lqr_control(self, state, reference):
        """
        Calculate LQR control law
        """
        # Solve Riccati equation offline, then apply feedback law
        # u = -K * (x - x_ref)
        error = state - reference
        K = self.calculate_lqr_gain()  # Pre-computed offline
        control = -K @ error
        return control

    def calculate_lqr_gain(self):
        """
        Calculate LQR gain matrix (typically done offline)
        """
        # Solve: A^T * P * A - P - A^T * P * B * (R + B^T * P * B)^(-1) * B^T * P * A + Q = 0
        # Return: K = (R + B^T * P * B)^(-1) * B^T * P * A
        return np.array([[1, 2, 1, 0.5, 0.1, 0.1]])  # Placeholder
```

## Control Architecture

### Hierarchical Control Structure

Humanoid robots typically use a hierarchical control structure:

```
┌─────────────────────────────────────────┐
│           High-Level Planner            │
│  (Walking pattern, trajectory planning) │
├─────────────────────────────────────────┤
│         Pattern Generator               │
│     (ZMP, CoM, foot placement)         │
├─────────────────────────────────────────┤
│         Feedback Controller             │
│     (Balance, impedance control)        │
├─────────────────────────────────────────┤
│         Joint Servo Level               │
│      (PID, current control)             │
└─────────────────────────────────────────┘
```

## Simulation and Testing

### Simulation Environment

```python
class HumanoidControlSimulator:
    def __init__(self, robot_model):
        self.model = robot_model
        self.controller = HumanoidBalanceController(robot_model)
        self.dt = 0.01

    def simulate_control_step(self, current_state):
        """
        Simulate one control step
        """
        # Get sensor data from simulation
        sensor_data = self.get_sensor_data(current_state)

        # Apply control
        commands = self.controller.control_step(sensor_data, self.dt)

        # Update simulation with commands
        next_state = self.update_dynamics(current_state, commands['joint_torques'])

        return next_state, commands

    def get_sensor_data(self, state):
        """
        Simulate sensor readings
        """
        # Add realistic sensor noise
        joint_pos = state['joint_positions'] + np.random.normal(0, 0.001, size=state['joint_positions'].shape)
        joint_vel = state['joint_velocities'] + np.random.normal(0, 0.01, size=state['joint_velocities'].shape)

        # IMU simulation
        imu_data = self.simulate_imu(state)

        return {
            'joint_positions': joint_pos,
            'joint_velocities': joint_vel,
            'imu_data': imu_data,
            'force_torque_data': self.simulate_force_sensors(state)
        }

    def update_dynamics(self, current_state, joint_torques):
        """
        Update robot dynamics with control torques
        """
        # Apply joint torques and simulate forward dynamics
        # This would use numerical integration of the equations of motion
        # using methods like Runge-Kutta or Euler integration

        # Simplified example using Euler integration
        M = self.model.mass_matrix(current_state['joint_positions'])
        C = self.model.coriolis_matrix(current_state['joint_positions'], current_state['joint_velocities'])
        g = self.model.gravity_vector(current_state['joint_positions'])

        # M*qdd + C*qd + g = tau
        joint_acc = np.linalg.solve(M, joint_torques - C @ current_state['joint_velocities'] - g)

        # Integrate to get new state
        new_joint_velocities = current_state['joint_velocities'] + joint_acc * self.dt
        new_joint_positions = current_state['joint_positions'] + new_joint_velocities * self.dt

        return {
            'joint_positions': new_joint_positions,
            'joint_velocities': new_joint_velocities
        }
```

## Summary

Dynamics and control form the core of humanoid robot functionality. The complex multi-body dynamics of humanoid robots require sophisticated control approaches that can handle:

1. **Multi-constraint optimization**: Balancing multiple objectives simultaneously
2. **Real-time computation**: Meeting strict timing requirements for stability
3. **Disturbance rejection**: Handling external forces and environmental interactions
4. **Safety considerations**: Ensuring stable and safe operation

Key techniques covered in this section include:

- **Inverse dynamics control**: Linearizing robot dynamics for precise control
- **Operational space control**: Controlling tasks in end-effector space
- **Impedance control**: Achieving compliant behavior for safe interaction
- **ZMP-based walking**: Ensuring stable bipedal locomotion
- **Whole-body control**: Coordinating all robot degrees of freedom
- **Balance recovery**: Maintaining stability under disturbances

The next section will focus specifically on bipedal locomotion, which builds upon these dynamic control foundations to enable humanoid robots to walk effectively.
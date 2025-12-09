# Bipedal Locomotion

## Introduction to Bipedal Locomotion

Bipedal locomotion is one of the most challenging and fascinating aspects of humanoid robotics. Unlike wheeled or tracked robots, bipedal robots must dynamically balance on two legs while walking, requiring sophisticated control algorithms and careful mechanical design. The ability to walk like humans enables humanoid robots to navigate human environments effectively and interact with human-designed infrastructure.

## Fundamentals of Human Walking

### Gait Cycle

Human walking consists of a repeating gait cycle with two main phases:

1. **Stance Phase (60% of cycle)**: The foot is in contact with the ground
   - Initial contact
   - Loading response
   - Mid stance
   - Terminal stance
   - Pre-swing

2. **Swing Phase (40% of cycle)**: The foot is off the ground, moving forward
   - Initial swing
   - Mid swing
   - Terminal swing

### Key Biomechanical Principles

Human walking is efficient due to several biomechanical principles:

- **Passive Dynamics**: Energy-efficient motion using pendulum-like mechanics
- **Ground Reaction Forces**: Proper force application for forward progression
- **Center of Mass (CoM) Movement**: Controlled CoM trajectory for stability
- **Ankle Strategy**: Using ankle torques for balance and propulsion

## Walking Pattern Generation

### Center of Mass Trajectory

The Center of Mass (CoM) trajectory is crucial for stable walking:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

class CoMTrajectoryGenerator:
    def __init__(self, walking_height=0.8, step_length=0.3, step_time=0.8):
        self.h = walking_height  # CoM height
        self.step_length = step_length
        self.step_time = step_time
        self.g = 9.81  # Gravity

    def generate_3d_com_trajectory(self, steps, start_pos=np.array([0, 0, 0.8])):
        """
        Generate 3D CoM trajectory for walking
        """
        dt = 0.01  # 100Hz control rate
        total_time = steps * self.step_time
        timesteps = int(total_time / dt)

        # Initialize trajectory arrays
        com_trajectory = np.zeros((timesteps, 3))
        com_velocity = np.zeros((timesteps, 3))
        com_acceleration = np.zeros((timesteps, 3))

        # Generate trajectories for each axis
        t = np.linspace(0, total_time, timesteps)

        # X-axis: forward progression with periodic motion
        x_pos = np.zeros(timesteps)
        for i in range(steps):
            step_start = i * self.step_time
            step_end = (i + 1) * self.step_time
            mask = (t >= step_start) & (t < step_end)
            if np.any(mask):
                # Smooth progression using 5th order polynomial
                phase = (t[mask] - step_start) / self.step_time
                x_progress = self.step_length * self.smooth_step(phase)
                x_pos[mask] = i * self.step_length + x_progress

        # Y-axis: lateral sway for stability (double support phase)
        y_pos = self.generate_lateral_sway(t, steps)

        # Z-axis: vertical oscillation (pelvic motion)
        z_pos = start_pos[2] + 0.02 * np.sin(2 * np.pi * t / self.step_time)  # Small vertical oscillation

        com_trajectory[:, 0] = x_pos
        com_trajectory[:, 1] = y_pos
        com_trajectory[:, 2] = z_pos

        # Calculate velocities and accelerations
        for i in range(3):
            com_velocity[:, i] = np.gradient(com_trajectory[:, i], dt)
            com_acceleration[:, i] = np.gradient(com_velocity[:, i], dt)

        return com_trajectory, com_velocity, com_acceleration

    def smooth_step(self, t):
        """
        5th order polynomial for smooth motion profile
        """
        return 6*t**5 - 15*t**4 + 10*t**3

    def generate_lateral_sway(self, t, steps):
        """
        Generate lateral sway for stability
        """
        y_pos = np.zeros_like(t)
        step_frequency = 1.0 / self.step_time

        for i in range(steps):
            step_start = i * self.step_time
            step_end = (i + 1) * self.step_time
            mask = (t >= step_start) & (t < step_end)

            if np.any(mask):
                phase = (t[mask] - step_start) / self.step_time
                # Lateral sway: shift CoM toward stance leg
                if i % 2 == 0:  # Right leg stance (left foot moves)
                    y_offset = -0.05 * np.sin(np.pi * phase)  # Move toward stance leg
                else:  # Left leg stance (right foot moves)
                    y_offset = 0.05 * np.sin(np.pi * phase)   # Move toward stance leg

                y_pos[mask] = y_offset

        return y_pos

    def plot_trajectory(self, com_trajectory):
        """
        Plot the CoM trajectory
        """
        t = np.linspace(0, len(com_trajectory) * 0.01, len(com_trajectory))

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

        ax1.plot(t, com_trajectory[:, 0])
        ax1.set_ylabel('X Position (m)')
        ax1.set_title('CoM Trajectory - X Axis')

        ax2.plot(t, com_trajectory[:, 1])
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title('CoM Trajectory - Y Axis')

        ax3.plot(t, com_trajectory[:, 2])
        ax3.set_ylabel('Z Position (m)')
        ax3.set_xlabel('Time (s)')
        ax3.set_title('CoM Trajectory - Z Axis')

        plt.tight_layout()
        plt.show()
```

### Footstep Planning

Footstep planning determines where and when the feet should be placed:

```python
class FootstepPlanner:
    def __init__(self, step_width=0.2, step_length=0.3, max_step_width=0.4):
        self.step_width = step_width
        self.step_length = step_length
        self.max_step_width = max_step_width

    def plan_forward_walk(self, num_steps, start_pose=np.array([0, 0, 0])):
        """
        Plan footsteps for forward walking
        """
        footsteps = []

        current_pose = start_pose.copy()

        for i in range(num_steps):
            # Determine which foot to step with
            is_left_foot = (i % 2) == 0  # Start with left foot

            # Calculate step position
            step_x = current_pose[0] + self.step_length
            step_y = current_pose[1] + (self.step_width if is_left_foot else -self.step_width)
            step_theta = current_pose[2]  # Maintain heading

            foot_pose = np.array([step_x, step_y, step_theta])
            footsteps.append({
                'position': foot_pose,
                'is_left': is_left_foot,
                'step_number': i
            })

            # Update current pose for next step
            current_pose[0] = step_x

        return footsteps

    def plan_turning_walk(self, turn_angle, num_steps, start_pose=np.array([0, 0, 0])):
        """
        Plan footsteps for turning motion
        """
        footsteps = []
        current_pose = start_pose.copy()

        angle_per_step = turn_angle / num_steps

        for i in range(num_steps):
            is_left_foot = (i % 2) == 0

            # Calculate turning arc
            turn_radius = self.step_length / (2 * np.sin(angle_per_step / 2)) if angle_per_step != 0 else float('inf')

            if turn_radius != float('inf'):
                # Calculate new position after turning
                step_x = current_pose[0] + self.step_length * np.cos(current_pose[2] + angle_per_step/2)
                step_y = current_pose[1] + self.step_length * np.sin(current_pose[2] + angle_per_step/2)
            else:
                # Straight line if no turn
                step_x = current_pose[0] + self.step_length * np.cos(current_pose[2])
                step_y = current_pose[1] + self.step_length * np.sin(current_pose[2])

            step_theta = current_pose[2] + angle_per_step

            # Adjust foot placement for turning
            if is_left_foot:
                # Left foot placement during right turn (or vice versa)
                step_y += self.step_width * np.cos(angle_per_step/2)
            else:
                step_y -= self.step_width * np.cos(angle_per_step/2)

            foot_pose = np.array([step_x, step_y, step_theta])
            footsteps.append({
                'position': foot_pose,
                'is_left': is_left_foot,
                'step_number': i
            })

            current_pose = np.array([step_x, step_y, step_theta])

        return footsteps

    def plan_sideways_walk(self, num_steps, start_pose=np.array([0, 0, 0])):
        """
        Plan footsteps for sideways walking (lateral movement)
        """
        footsteps = []
        current_pose = start_pose.copy()

        for i in range(num_steps):
            is_left_foot = (i % 2) == 0

            # For sideways walking, step in Y direction
            step_x = current_pose[0]
            step_y = current_pose[1] + (self.step_width if is_left_foot else -self.step_width)
            step_theta = current_pose[2]

            # Alternate feet placement for stability
            if i > 0:
                # Adjust the other foot to maintain balance
                if is_left_foot:
                    step_y += 0.1  # Small offset for left foot
                else:
                    step_y -= 0.1  # Small offset for right foot

            foot_pose = np.array([step_x, step_y, step_theta])
            footsteps.append({
                'position': foot_pose,
                'is_left': is_left_foot,
                'step_number': i
            })

            current_pose = foot_pose.copy()

        return footsteps

    def check_footstep_feasibility(self, footstep, obstacles=None):
        """
        Check if a footstep is feasible given obstacles
        """
        if obstacles is None:
            obstacles = []

        # Check for collisions with obstacles
        for obs in obstacles:
            if self.distance_to_obstacle(footstep['position'][:2], obs) < 0.1:  # 10cm clearance
                return False, "Footstep collides with obstacle"

        # Check if footstep is within reachable workspace
        if abs(footstep['position'][1]) > self.max_step_width:
            return False, "Footstep exceeds maximum lateral reach"

        return True, "Footstep is feasible"

    def distance_to_obstacle(self, point, obstacle):
        """
        Calculate distance from point to obstacle
        """
        # Simplified distance calculation (assuming circular obstacles)
        center = obstacle['center']
        radius = obstacle.get('radius', 0.1)
        return np.linalg.norm(point - center[:2]) - radius
```

## Zero Moment Point (ZMP) Based Walking

### ZMP Fundamentals

The Zero Moment Point is a critical concept in bipedal walking control:

```python
class ZMPController:
    def __init__(self, com_height=0.8, gravity=9.81):
        self.h = com_height
        self.g = gravity
        self.omega = np.sqrt(self.g / self.h)

    def calculate_zmp_from_com(self, com_pos, com_vel, com_acc):
        """
        Calculate ZMP from CoM state
        """
        x_com, y_com = com_pos[0], com_pos[1]
        x_com_dot, y_com_dot = com_vel[0], com_vel[1]
        x_com_ddot, y_com_ddot = com_acc[0], com_acc[1]

        # ZMP equations
        zmp_x = x_com - (self.h / self.g) * x_com_ddot
        zmp_y = y_com - (self.h / self.g) * y_com_ddot

        return np.array([zmp_x, zmp_y, 0.0])

    def calculate_com_from_zmp(self, zmp_trajectory, initial_com, initial_com_vel):
        """
        Calculate CoM trajectory from ZMP reference using inverted pendulum model
        """
        dt = 0.01  # Control timestep
        n_points = len(zmp_trajectory)

        com_pos = np.zeros((n_points, 3))
        com_vel = np.zeros((n_points, 3))
        com_acc = np.zeros((n_points, 3))

        # Initialize with given conditions
        com_pos[0] = initial_com
        com_vel[0] = initial_com_vel

        for i in range(1, n_points):
            # Inverted pendulum dynamics: com_ddot = g/h * (com - zmp)
            com_acc[i, 0] = (self.g / self.h) * (com_pos[i-1, 0] - zmp_trajectory[i-1, 0])
            com_acc[i, 1] = (self.g / self.h) * (com_pos[i-1, 1] - zmp_trajectory[i-1, 1])
            com_acc[i, 2] = 0  # Assume constant height

            # Integrate to get velocity and position
            com_vel[i] = com_vel[i-1] + com_acc[i] * dt
            com_pos[i] = com_pos[i-1] + com_vel[i] * dt

        return com_pos, com_vel, com_acc

    def generate_zmp_trajectory(self, footsteps, step_time=0.8, dt=0.01):
        """
        Generate ZMP trajectory following footsteps
        """
        n_steps = len(footsteps)
        step_samples = int(step_time / dt)
        total_samples = n_steps * step_samples

        zmp_trajectory = np.zeros((total_samples, 3))

        for i, footstep in enumerate(footsteps):
            start_idx = i * step_samples
            end_idx = min((i + 1) * step_samples, total_samples)

            # ZMP should be under the supporting foot
            support_foot_pos = footstep['position'][:2]

            # Smooth transition from previous support foot to new support foot
            if i > 0:
                prev_footstep = footsteps[i-1]
                prev_support_pos = prev_footstep['position'][:2]

                for j in range(start_idx, end_idx):
                    if j < start_idx + step_samples // 4:  # Double support phase
                        # Interpolate between old and new support foot
                        alpha = (j - start_idx) / (step_samples // 4)
                        zmp_trajectory[j, :2] = (1 - alpha) * prev_support_pos + alpha * support_foot_pos
                    else:  # Single support phase
                        zmp_trajectory[j, :2] = support_foot_pos
            else:
                # First step - start at initial support foot
                zmp_trajectory[start_idx:end_idx, :2] = support_foot_pos

            # Maintain ZMP at foot level (z=0)
            zmp_trajectory[start_idx:end_idx, 2] = 0.0

        return zmp_trajectory

    def calculate_support_polygon(self, left_foot_pos, right_foot_pos):
        """
        Calculate support polygon from foot positions
        """
        # For two feet, support polygon is the convex hull of both feet
        # Simplified as a rectangle encompassing both feet
        min_x = min(left_foot_pos[0], right_foot_pos[0])
        max_x = max(left_foot_pos[0], right_foot_pos[0])
        min_y = min(left_foot_pos[1], right_foot_pos[1])
        max_y = max(left_foot_pos[1], right_foot_pos[1])

        # Add some margin for stability
        margin = 0.02  # 2cm margin
        return {
            'min_x': min_x - margin,
            'max_x': max_x + margin,
            'min_y': min_y - margin,
            'max_y': max_y + margin
        }

    def is_zmp_stable(self, zmp_pos, support_polygon):
        """
        Check if ZMP is within support polygon
        """
        return (support_polygon['min_x'] <= zmp_pos[0] <= support_polygon['max_x'] and
                support_polygon['min_y'] <= zmp_pos[1] <= support_polygon['max_y'])
```

### Preview Control for ZMP Tracking

Preview control improves ZMP tracking by considering future reference:

```python
class PreviewController:
    def __init__(self, com_height=0.8, preview_time=2.0, dt=0.01):
        self.h = com_height
        self.preview_time = preview_time
        self.dt = dt
        self.g = 9.81
        self.omega = np.sqrt(self.g / self.h)

        # Calculate preview control gains
        self.K_x, self.K_v, self.K_preview = self.calculate_preview_gains()

    def calculate_preview_gains(self):
        """
        Calculate preview control gains by solving Riccati equation
        """
        # System matrices for inverted pendulum
        A = np.array([[0, 1], [self.omega**2, 0]])  # State matrix
        B = np.array([0, -self.omega**2])          # Input matrix

        # Discretize system
        I = np.eye(2)
        A_d = I + A * self.dt + (A @ A) * (self.dt**2) / 2  # Approximate discretization
        B_d = B * self.dt + (A @ B) * (self.dt**2) / 2

        # Design parameters
        Q = np.array([[100, 0], [0, 10]])  # State cost
        R = 1.0                           # Control cost

        # Solve discrete-time Riccati equation (simplified approach)
        # In practice, this would use more sophisticated methods
        P = np.array([[10, 0], [0, 1]])  # Solution to Riccati equation

        # Feedback gain
        K = (B_d.T @ P @ A_d) / (R + B_d.T @ P @ B_d)

        # Preview gains calculation (simplified)
        preview_steps = int(self.preview_time / self.dt)
        preview_gains = np.zeros(preview_steps)

        # Calculate preview gains based on system dynamics
        for i in range(preview_steps):
            k = i * self.dt
            preview_gains[i] = self.omega * np.exp(-self.omega * k) * self.dt

        return K[0], K[1], preview_gains

    def compute_control(self, current_com, current_com_vel, zmp_reference):
        """
        Compute control using preview of future ZMP reference
        """
        current_state = np.array([current_com[0], current_com_vel[0]])  # Simplified to x-axis
        reference_zmp = zmp_reference[0]  # X component of ZMP reference

        # Current error
        zmp_current = current_com[0] - (self.h / self.g) * current_com_vel[0]  # Simplified ZMP calculation
        error = reference_zmp - zmp_current

        # Feedback control
        feedback_control = self.K_x * (reference_zmp - current_com[0]) + \
                          self.K_v * (0 - current_com_vel[0])  # Assuming desired velocity is 0

        # Preview control (simplified)
        preview_control = 0
        if len(zmp_reference) > 1:
            for i, future_ref in enumerate(zmp_reference[1:]):
                if i < len(self.K_preview):
                    preview_control += self.K_preview[i] * future_ref

        total_control = feedback_control + preview_control

        # Convert to CoM acceleration reference
        com_acc_ref = self.omega**2 * (current_com[0] - reference_zmp) + total_control

        return com_acc_ref

    def generate_zmp_reference(self, footsteps, step_time=0.8):
        """
        Generate ZMP reference trajectory from footsteps
        """
        dt = self.dt
        step_samples = int(step_time / dt)
        n_steps = len(footsteps)

        # Create ZMP reference that follows footsteps with smooth transitions
        zmp_ref = np.zeros((n_steps * step_samples, 2))

        for i, footstep in enumerate(footsteps):
            start_idx = i * step_samples
            end_idx = min((i + 1) * step_samples, len(zmp_ref))

            # Position of supporting foot
            foot_pos = footstep['position'][:2]

            # Smooth transition between footsteps
            for j in range(start_idx, end_idx):
                if j < start_idx + step_samples // 5:  # 20% double support
                    # If not first step, interpolate with previous foot position
                    if i > 0:
                        prev_pos = footsteps[i-1]['position'][:2]
                        alpha = (j - start_idx) / (step_samples // 5)
                        zmp_ref[j] = (1 - alpha) * prev_pos + alpha * foot_pos
                    else:
                        zmp_ref[j] = foot_pos
                else:
                    zmp_ref[j] = foot_pos

        return zmp_ref
```

## Walking Pattern Generators

### Foot Trajectory Generation

```python
class FootTrajectoryGenerator:
    def __init__(self, foot_height=0.1, swing_time_ratio=0.3):
        self.foot_height = foot_height  # Maximum foot lift height
        self.swing_time_ratio = swing_time_ratio  # Ratio of step time for swing phase

    def generate_foot_trajectory(self, start_pos, end_pos, step_time=0.8, dt=0.01):
        """
        Generate complete foot trajectory for one step
        """
        total_samples = int(step_time / dt)
        trajectory = np.zeros((total_samples, 6))  # Position and velocity (x, y, z)

        # Calculate swing and stance phases
        swing_samples = int(total_samples * self.swing_time_ratio)
        stance_samples = total_samples - swing_samples

        # Generate trajectory using 5th order polynomial for smooth motion
        for i in range(3):  # For x, y, z
            if i < 2:  # X and Y (horizontal movement)
                # Only move during swing phase
                swing_traj = self.generate_5th_order_polynomial(
                    start_pos[i], 0, 0,  # Start: pos, vel, acc
                    end_pos[i], 0, 0,    # End: pos, vel, acc
                    swing_samples
                )

                # Combine with stance phase (no movement)
                full_traj = np.zeros(total_samples)
                full_traj[:swing_samples] = swing_traj
                full_traj[swing_samples:] = end_pos[i]  # Hold position during stance

            else:  # Z (vertical movement)
                # Foot lifts during swing, stays low during stance
                swing_z = self.generate_vertical_profile(swing_samples)
                stance_z = np.full(stance_samples, start_pos[2])  # Maintain ground contact height

                full_traj = np.concatenate([swing_z, stance_z])

            trajectory[:, i] = full_traj

        # Calculate velocities (numerical differentiation)
        for i in range(3):
            trajectory[:, i + 3] = np.gradient(trajectory[:, i], dt)

        return trajectory

    def generate_5th_order_polynomial(self, x0, v0, a0, x1, v1, a1, n_points):
        """
        Generate 5th order polynomial trajectory
        """
        t = np.linspace(0, 1, n_points)

        # Coefficients for 5th order polynomial: x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
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

        # Evaluate polynomial
        traj = (coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 +
                coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5)

        return traj

    def generate_vertical_profile(self, swing_samples):
        """
        Generate vertical foot trajectory for swing phase
        """
        # Create a smooth profile that lifts foot and puts it down
        t = np.linspace(0, 1, swing_samples)

        # Use 5th order polynomial for smooth lift and place
        lift_height = self.foot_height
        start_z = 0  # Foot starts at ground level
        end_z = 0    # Foot ends at ground level

        # Generate polynomial coefficients for vertical motion
        A = np.array([
            [1, 0, 0, 0, 0, 0],    # Start position
            [0, 1, 0, 0, 0, 0],    # Start velocity
            [0, 0, 2, 0, 0, 0],    # Start acceleration
            [1, 1, 1, 1, 1, 1],    # End position
            [0, 1, 2, 3, 4, 5],    # End velocity
            [0, 0, 2, 6, 12, 20]   # End acceleration
        ])

        b = np.array([start_z, 0, 0, end_z, 0, 0])  # Zero velocity and acceleration at start/end
        coeffs = np.linalg.solve(A, b)

        # Evaluate polynomial for vertical trajectory
        z_traj = (coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 +
                 coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5)

        # Scale to achieve desired lift height
        max_z = np.max(z_traj)
        if max_z > 0:
            z_traj = z_traj * (lift_height / max_z)

        return z_traj

    def generate_ankle_trajectory(self, foot_trajectory):
        """
        Generate ankle trajectory from foot trajectory
        """
        # For simplicity, ankle trajectory follows foot trajectory
        # In reality, ankle trajectory would consider foot orientation and ground contact
        return foot_trajectory
```

## Walking Controllers

### Joint Space Walking Controller

```python
class WalkingController:
    def __init__(self, robot_model, zmp_controller, preview_controller):
        self.model = robot_model
        self.zmp_controller = zmp_controller
        self.preview_controller = preview_controller
        self.foot_generator = FootTrajectoryGenerator()

        # Walking parameters
        self.step_length = 0.3
        self.step_width = 0.2
        self.step_time = 0.8
        self.com_height = 0.8

        # Initialize walking state
        self.current_step = 0
        self.is_left_support = True  # Start with left foot as support
        self.walking_state = "double_support"  # Initial state

    def compute_walking_control(self, current_state, dt=0.01):
        """
        Compute walking control for current state
        """
        # 1. Update ZMP and CoM based on current state
        com_pos = current_state['com_pos']
        com_vel = current_state['com_vel']
        com_acc = current_state['com_acc']

        current_zmp = self.zmp_controller.calculate_zmp_from_com(com_pos, com_vel, com_acc)

        # 2. Plan next footsteps
        next_footsteps = self.plan_next_footsteps(current_state)

        # 3. Generate ZMP reference
        zmp_reference = self.preview_controller.generate_zmp_reference(next_footsteps)

        # 4. Compute CoM control using preview control
        com_control = self.preview_controller.compute_control(com_pos, com_vel, zmp_reference[0])

        # 5. Generate foot trajectories
        left_foot_traj, right_foot_traj = self.generate_foot_trajectories(
            current_state, next_footsteps
        )

        # 6. Compute joint space control
        joint_torques = self.compute_joint_control(
            current_state, com_control, left_foot_traj, right_foot_traj
        )

        # 7. Update walking state
        self.update_walking_state(current_state)

        return {
            'joint_torques': joint_torques,
            'com_control': com_control,
            'zmp_reference': zmp_reference[0],
            'current_zmp': current_zmp
        }

    def plan_next_footsteps(self, current_state):
        """
        Plan next few footsteps based on current state and desired walking direction
        """
        # For this example, plan a simple forward walk
        # In practice, this would consider terrain, obstacles, and walking goals
        current_pos = current_state['base_pos'][:2]  # X, Y position
        current_heading = current_state['base_pos'][2]  # Yaw angle

        footsteps = []
        n_future_steps = 3  # Plan 3 steps ahead

        for i in range(n_future_steps):
            step_x = current_pos[0] + (i + 1) * self.step_length * np.cos(current_heading)
            step_y = current_pos[1] + (i + 1) * self.step_length * np.sin(current_heading)

            # Alternate feet
            is_left = (self.current_step + i) % 2 == 0
            if is_left:
                step_y += self.step_width / 2 * np.cos(current_heading + np.pi/2)
            else:
                step_y += self.step_width / 2 * np.cos(current_heading - np.pi/2)

            step_theta = current_heading  # Maintain heading

            footsteps.append({
                'position': np.array([step_x, step_y, step_theta]),
                'is_left': is_left,
                'step_number': self.current_step + i
            })

        return footsteps

    def generate_foot_trajectories(self, current_state, footsteps):
        """
        Generate complete foot trajectories for next steps
        """
        # Get current foot positions
        left_foot_pos = current_state['left_foot_pos']
        right_foot_pos = current_state['right_foot_pos']

        # Determine which foot is swing foot
        swing_foot_pos = left_foot_pos if not self.is_left_support else right_foot_pos
        stance_foot_pos = right_foot_pos if not self.is_left_support else left_foot_pos

        # Generate trajectory for swing foot
        next_footstep = footsteps[0]  # Next target footstep
        swing_trajectory = self.foot_generator.generate_foot_trajectory(
            swing_foot_pos, next_footstep['position'], self.step_time
        )

        # Stance foot remains in place (for now)
        stance_trajectory = np.tile(stance_foot_pos, (len(swing_trajectory), 1))

        if self.is_left_support:
            left_foot_traj = stance_trajectory
            right_foot_traj = swing_trajectory
        else:
            left_foot_traj = swing_trajectory
            right_foot_traj = stance_trajectory

        return left_foot_traj[0], right_foot_traj[0]  # Return first point for current control

    def compute_joint_control(self, current_state, com_control, left_foot_ref, right_foot_ref):
        """
        Compute joint space control using operational space control
        """
        # Get current joint positions and velocities
        q = current_state['joint_positions']
        qd = current_state['joint_velocities']

        # Compute Jacobians for feet and CoM
        left_foot_jac = self.model.get_jacobian('left_foot', q)
        right_foot_jac = self.model.get_jacobian('right_foot', q)
        com_jac = self.model.get_com_jacobian(q)

        # Mass matrix
        M = self.model.mass_matrix(q)

        # Desired accelerations
        left_foot_acc_des = self.compute_foot_acceleration(left_foot_ref)
        right_foot_acc_des = self.compute_foot_acceleration(right_foot_ref)
        com_acc_des = np.array([com_control, 0, 0])  # Simplified - only x-direction control

        # Stack task equations
        # For simplicity, we'll focus on CoM control and one foot (stance foot)
        if self.is_left_support:
            # Control right foot (swing) and CoM
            task_jac = np.vstack([right_foot_jac, com_jac[:3, :]])  # Only position part of CoM
            task_acc_des = np.hstack([left_foot_acc_des[:3], com_acc_des[:3]])  # Only position
        else:
            # Control left foot (swing) and CoM
            task_jac = np.vstack([left_foot_jac, com_jac[:3, :]])
            task_acc_des = np.hstack([right_foot_acc_des[:3], com_acc_des[:3]])

        # Operational space control law
        Lambda = np.linalg.inv(task_jac @ np.linalg.inv(M) @ task_jac.T)
        J_bar = np.linalg.inv(M) @ task_jac.T @ Lambda  # Dynamically consistent inverse

        # Compute task-space forces
        Kp_pos = 100 * np.eye(task_acc_des.shape[0])  # Position gains
        Kd_vel = 20 * np.eye(task_acc_des.shape[0])   # Velocity gains

        task_error = self.compute_task_error(q, task_jac)  # Simplified
        task_vel_error = self.compute_task_velocity_error(q, qd, task_jac)  # Simplified

        F_task = Lambda @ (task_acc_des + Kp_pos @ task_error + Kd_vel @ task_vel_error)

        # Convert to joint torques
        tau = task_jac.T @ F_task

        # Add null-space motion to maintain posture
        tau += self.compute_null_space_motion(q, qd, M, task_jac)

        return tau

    def compute_foot_acceleration(self, foot_ref):
        """
        Compute desired foot acceleration from reference trajectory
        """
        # Simplified - in practice, this would use trajectory derivatives
        return np.zeros(6)  # [xdot, ydot, zdot, xddot, yddot, zddot]

    def compute_task_error(self, q, jacobian):
        """
        Compute task space position error
        """
        # Simplified implementation
        return np.zeros(jacobian.shape[0])

    def compute_task_velocity_error(self, q, qd, jacobian):
        """
        Compute task space velocity error
        """
        # Simplified implementation
        return np.zeros(jacobian.shape[0])

    def compute_null_space_motion(self, q, qd, M, task_jac):
        """
        Compute motion in null space to achieve secondary objectives
        """
        # Null space projector
        J_bar = np.linalg.pinv(task_jac)  # Pseudo-inverse
        I = np.eye(M.shape[0])
        N = I - J_bar @ task_jac

        # Desired null space motion (e.g., toward nominal posture)
        q_nominal = np.array([0, 0.1, -0.2, 0.1, 0, 0, 0] * 2)  # Simplified nominal pose
        null_motion = 0.1 * (q_nominal - q)

        return N.T @ null_motion

    def update_walking_state(self, current_state):
        """
        Update walking state based on phase and foot contacts
        """
        # Simplified state machine
        time_in_step = current_state.get('time_in_step', 0)
        step_phase = time_in_step / self.step_time

        if step_phase < 0.1:  # First 10% of step
            self.walking_state = "double_support"
        elif step_phase < 0.9:  # Middle 80% of step
            self.walking_state = "single_support"
        else:  # Last 10% of step
            self.walking_state = "double_support"

        # Update support foot
        if step_phase > 0.5 and self.walking_state == "single_support":
            self.is_left_support = not self.is_left_support  # Switch support foot
            self.current_step += 1
```

## Advanced Walking Techniques

### Walking on Uneven Terrain

```python
class UnevenTerrainWalker:
    def __init__(self, base_walker):
        self.base_walker = base_walker
        self.terrain_estimator = self.initialize_terrain_estimator()

    def initialize_terrain_estimator(self):
        """
        Initialize terrain estimation system
        """
        return {
            'height_map': np.zeros((100, 100)),  # Local height map
            'obstacle_map': np.zeros((100, 100)),  # Obstacle map
            'roughness_map': np.zeros((100, 100))  # Surface roughness
        }

    def adapt_walking_for_terrain(self, current_state, terrain_data):
        """
        Adapt walking pattern based on terrain characteristics
        """
        # 1. Update terrain model
        self.update_terrain_model(terrain_data)

        # 2. Adjust footstep placement
        adjusted_footsteps = self.adjust_footsteps_for_terrain(
            current_state, terrain_data
        )

        # 3. Modify ZMP reference
        modified_zmp_ref = self.modify_zmp_for_terrain(
            current_state, terrain_data
        )

        # 4. Adjust walking parameters
        modified_params = self.adjust_walking_parameters(
            terrain_data
        )

        # 5. Generate adapted control
        control_commands = self.base_walker.compute_walking_control(
            current_state
        )

        # 6. Apply terrain-specific modifications
        adapted_control = self.apply_terrain_adaptations(
            control_commands, terrain_data
        )

        return adapted_control

    def update_terrain_model(self, terrain_data):
        """
        Update internal terrain model with new sensor data
        """
        # Update height map with new LIDAR or vision data
        # Update obstacle map
        # Update roughness estimates
        pass

    def adjust_footsteps_for_terrain(self, current_state, terrain_data):
        """
        Adjust footstep locations based on terrain characteristics
        """
        original_footsteps = self.base_walker.plan_next_footsteps(current_state)

        adjusted_footsteps = []
        for footstep in original_footsteps:
            original_pos = footstep['position']

            # Check terrain at original location
            terrain_height = self.get_terrain_height(original_pos[:2])
            terrain_roughness = self.get_terrain_roughness(original_pos[:2])
            has_obstacle = self.has_obstacle_at(original_pos[:2])

            adjusted_pos = original_pos.copy()

            if has_obstacle:
                # Find alternative location
                adjusted_pos = self.find_safe_alternative(footstep, terrain_data)
            elif terrain_roughness > 0.05:  # Too rough
                # Adjust foot orientation or position
                adjusted_pos = self.adjust_for_roughness(footstep, terrain_data)
            elif abs(terrain_height - current_state['base_pos'][2]) > 0.1:  # Height change > 10cm
                # Adjust step height or approach
                adjusted_pos = self.adjust_for_height_change(footstep, terrain_data)

            adjusted_footsteps.append({
                'position': adjusted_pos,
                'is_left': footstep['is_left'],
                'step_number': footstep['step_number']
            })

        return adjusted_footsteps

    def modify_zmp_for_terrain(self, current_state, terrain_data):
        """
        Modify ZMP reference for terrain adaptation
        """
        # On slopes, adjust ZMP to maintain stability
        # On rough terrain, use more conservative ZMP bounds
        # On stairs, adjust for step height
        pass

    def adjust_walking_parameters(self, terrain_data):
        """
        Adjust walking parameters based on terrain
        """
        params = {
            'step_length': self.step_length,
            'step_width': self.step_width,
            'step_time': self.step_time,
            'foot_lift_height': self.foot_generator.foot_height
        }

        # Adjust for different terrain types
        terrain_type = self.classify_terrain(terrain_data)

        if terrain_type == 'slope_up':
            params['step_length'] *= 0.8  # Shorter steps on uphill
            params['step_time'] *= 1.2   # Slower on uphill
        elif terrain_type == 'slope_down':
            params['step_length'] *= 0.9  # Shorter steps on downhill
            params['step_time'] *= 1.1   # More cautious downhill
        elif terrain_type == 'rough':
            params['step_time'] *= 1.3   # Slower on rough terrain
            params['foot_lift_height'] *= 1.5  # Higher foot lift
        elif terrain_type == 'stairs':
            # Special stair climbing gait
            pass

        return params

    def apply_terrain_adaptations(self, control_commands, terrain_data):
        """
        Apply terrain-specific modifications to control commands
        """
        # Add ankle adjustments for uneven terrain
        # Modify hip height for step climbing
        # Adjust balance strategy for different surfaces
        return control_commands

    def get_terrain_height(self, position):
        """
        Get terrain height at given position
        """
        # Interpolate from height map
        return 0.0

    def get_terrain_roughness(self, position):
        """
        Get terrain roughness at given position
        """
        # Calculate from local height variations
        return 0.0

    def has_obstacle_at(self, position):
        """
        Check if there's an obstacle at given position
        """
        return False

    def find_safe_alternative(self, footstep, terrain_data):
        """
        Find safe alternative footstep location
        """
        # Search in local neighborhood for safe placement
        return footstep['position']
```

## Walking Stability and Recovery

### Disturbance Recovery

```python
class DisturbanceRecovery:
    def __init__(self, robot_model):
        self.model = robot_model
        self.capturability = self.calculate_capturability()

    def calculate_capturability(self):
        """
        Calculate capturability region for the robot
        """
        # The capturability region defines where the robot can step to stop
        # This is based on the Linear Inverted Pendulum Model
        com_height = 0.8  # Example CoM height
        g = 9.81
        omega = np.sqrt(g / com_height)

        # Maximum step time for recovery
        max_time = 2.0  # seconds

        # Maximum recoverable velocity
        max_velocity = 0.5  # m/s

        return {
            'max_distance': omega * max_time,  # Maximum distance to capture point
            'max_velocity': max_velocity,
            'omega': omega
        }

    def detect_disturbance(self, current_state, threshold=0.1):
        """
        Detect if robot is experiencing a disturbance
        """
        zmp_pos = current_state['zmp']
        support_polygon = current_state['support_polygon']

        # Check if ZMP is outside support polygon
        zmp_stable = self.model.zmp_controller.is_zmp_stable(zmp_pos, support_polygon)

        # Check CoM velocity (too fast indicates disturbance)
        com_velocity_norm = np.linalg.norm(current_state['com_vel'])

        # Check joint accelerations (sudden changes indicate disturbance)
        joint_acc_norm = np.linalg.norm(current_state['joint_acc'])

        disturbance_detected = not zmp_stable or \
                              com_velocity_norm > self.capturability['max_velocity'] or \
                              joint_acc_norm > threshold

        return disturbance_detected

    def compute_recovery_strategy(self, current_state):
        """
        Compute recovery strategy based on current state
        """
        com_pos = current_state['com_pos'][:2]  # X, Y only
        com_vel = current_state['com_vel'][:2]

        # Calculate capture point
        capture_point = com_pos + com_vel / self.capturability['omega']

        # Determine recovery type based on capturability
        distance_to_capture = np.linalg.norm(capture_point - current_state['current_foot_pos'][:2])

        if distance_to_capture < 0.3:  # Can step to capture point
            return self.step_to_capture_point(current_state, capture_point)
        elif distance_to_capture < 0.6:  # Extended step possible
            return self.extended_step_strategy(current_state, capture_point)
        else:  # May need more aggressive recovery
            return self.emergency_recovery(current_state)

    def step_to_capture_point(self, current_state, capture_point):
        """
        Plan step to capture point for recovery
        """
        current_pos = current_state['base_pos'][:2]

        # Plan step toward capture point
        step_direction = capture_point - current_pos
        step_distance = np.linalg.norm(step_direction)

        if step_distance > 0:
            step_direction = step_direction / step_distance
            # Limit step size to maximum step length
            step_length = min(step_distance, 0.4)  # Max 40cm step
            target_pos = current_pos + step_length * step_direction
        else:
            target_pos = current_pos

        return {
            'type': 'capture_point_step',
            'target_position': target_pos,
            'step_timing': 'immediate'
        }

    def extended_step_strategy(self, current_state, capture_point):
        """
        Strategy for when capture point is slightly out of reach
        """
        # Use hip shift, arm swing, or extended step
        return {
            'type': 'extended_strategy',
            'target_position': capture_point,
            'modifications': ['hip_shift', 'arm_swing'],
            'timing': 'next_step'
        }

    def emergency_recovery(self, current_state):
        """
        Emergency recovery when falling is imminent
        """
        # Strategies: knee bending, arm extension, fall preparation
        return {
            'type': 'emergency',
            'actions': ['bend_knees', 'extend_arms', 'prepare_fall'],
            'priority': 'highest'
        }

    def execute_recovery(self, recovery_strategy, current_state):
        """
        Execute the recovery strategy
        """
        if recovery_strategy['type'] == 'capture_point_step':
            # Modify next footstep to target capture point
            return self.modify_footstep_for_recovery(
                current_state, recovery_strategy['target_position']
            )
        elif recovery_strategy['type'] == 'extended_strategy':
            # Apply additional stabilization
            return self.apply_extended_stabilization(
                current_state, recovery_strategy
            )
        elif recovery_strategy['type'] == 'emergency':
            # Execute emergency procedures
            return self.execute_emergency_procedures(
                current_state, recovery_strategy
            )

        return None
```

## Walking Simulation and Testing

### Walking Simulation Environment

```python
class WalkingSimulator:
    def __init__(self, robot_model, controller):
        self.model = robot_model
        self.controller = controller
        self.dt = 0.01  # 100Hz control rate

    def simulate_walking_step(self, initial_state, steps=10):
        """
        Simulate walking for specified number of steps
        """
        state = initial_state.copy()
        simulation_data = {
            'time': [],
            'com_trajectory': [],
            'zmp_trajectory': [],
            'joint_angles': [],
            'joint_torques': [],
            'foot_positions': []
        }

        for step in range(steps):
            step_start_time = len(simulation_data['time']) * self.dt

            # Simulate one step duration
            step_duration = 0.8  # 800ms per step
            step_samples = int(step_duration / self.dt)

            for i in range(step_samples):
                current_time = step_start_time + i * self.dt

                # Get sensor data from current state
                sensor_data = self.get_sensor_data(state)

                # Compute control commands
                control_output = self.controller.compute_walking_control(sensor_data, self.dt)

                # Apply control to robot dynamics
                state = self.apply_control_and_update_dynamics(state, control_output['joint_torques'])

                # Log simulation data
                simulation_data['time'].append(current_time)
                simulation_data['com_trajectory'].append(state['com_pos'].copy())
                simulation_data['zmp_trajectory'].append(
                    self.model.zmp_controller.calculate_zmp_from_com(
                        state['com_pos'], state['com_vel'], state['com_acc']
                    )
                )
                simulation_data['joint_angles'].append(state['joint_positions'].copy())
                simulation_data['joint_torques'].append(control_output['joint_torques'].copy())
                simulation_data['foot_positions'].append({
                    'left': state['left_foot_pos'].copy(),
                    'right': state['right_foot_pos'].copy()
                })

                # Check for stability
                if self.is_falling(state):
                    print(f"Robot fell at time {current_time:.2f}s")
                    break

        return simulation_data

    def get_sensor_data(self, state):
        """
        Simulate sensor data from robot state
        """
        # Add realistic sensor noise
        noisy_com_pos = state['com_pos'] + np.random.normal(0, 0.001, 3)
        noisy_com_vel = state['com_vel'] + np.random.normal(0, 0.01, 3)

        # Simulate IMU data
        imu_data = self.simulate_imu(state)

        # Simulate joint encoders with noise
        noisy_joint_pos = state['joint_positions'] + np.random.normal(0, 0.0005, len(state['joint_positions']))

        return {
            'com_pos': noisy_com_pos,
            'com_vel': noisy_com_vel,
            'joint_positions': noisy_joint_pos,
            'joint_velocities': state['joint_velocities'],
            'imu_data': imu_data,
            'force_torque': self.simulate_force_sensors(state)
        }

    def apply_control_and_update_dynamics(self, state, joint_torques):
        """
        Apply control torques and update robot dynamics
        """
        # Get current state
        q = state['joint_positions']
        qd = state['joint_velocities']

        # Get dynamic parameters
        M = self.model.mass_matrix(q)
        C = self.model.coriolis_matrix(q, qd)
        g = self.model.gravity_vector(q)

        # Compute joint accelerations: M*qdd + C*qd + g = tau
        qdd = np.linalg.solve(M, joint_torques - C @ qd - g)

        # Integrate to get new velocities and positions
        new_qd = qd + qdd * self.dt
        new_q = q + new_qd * self.dt

        # Update state
        new_state = state.copy()
        new_state['joint_positions'] = new_q
        new_state['joint_velocities'] = new_qd

        # Update CoM based on new joint configuration
        new_state['com_pos'] = self.model.calculate_com_position(new_q)
        new_state['com_vel'] = self.model.calculate_com_velocity(new_q, new_qd)
        new_state['com_acc'] = (new_state['com_vel'] - state['com_vel']) / self.dt

        return new_state

    def simulate_imu(self, state):
        """
        Simulate IMU data
        """
        # Calculate body acceleration and angular velocity
        return {
            'linear_acceleration': np.array([0, 0, -9.81]) + np.random.normal(0, 0.01, 3),
            'angular_velocity': state['base_angular_vel'] + np.random.normal(0, 0.001, 3),
            'orientation': state['base_orientation']
        }

    def simulate_force_sensors(self, state):
        """
        Simulate force/torque sensors in feet
        """
        # Calculate ground reaction forces based on contact model
        left_contact = self.is_foot_in_contact(state['left_foot_pos'], state['com_pos'])
        right_contact = self.is_foot_in_contact(state['right_foot_pos'], state['com_pos'])

        # Simplified force calculation
        total_weight = 70 * 9.81  # 70kg robot

        if left_contact and right_contact:
            # Double support - distribute weight
            left_force = total_weight / 2
            right_force = total_weight / 2
        elif left_contact:
            # Left foot support
            left_force = total_weight
            right_force = 0
        elif right_contact:
            # Right foot support
            left_force = 0
            right_force = total_weight
        else:
            # No contact - robot is falling
            left_force = 0
            right_force = 0

        return {
            'left_foot': [0, 0, left_force],  # Fx, Fy, Fz
            'right_foot': [0, 0, right_force]
        }

    def is_foot_in_contact(self, foot_pos, com_pos):
        """
        Simple contact detection
        """
        # Check if foot is below or at ground level
        return foot_pos[2] <= 0.01  # 1cm above ground considered contact

    def is_falling(self, state):
        """
        Check if robot is falling
        """
        # Check if CoM is too far from support polygon
        # Check if joint limits are exceeded
        # Check if robot orientation is too tilted
        base_orientation = state['base_orientation']
        roll, pitch, yaw = self.quaternion_to_euler(base_orientation)

        return abs(roll) > 0.5 or abs(pitch) > 0.5  # 30 degrees threshold

    def quaternion_to_euler(self, quat):
        """
        Convert quaternion to Euler angles
        """
        # Simplified conversion
        w, x, y, z = quat
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw

    def analyze_walking_performance(self, simulation_data):
        """
        Analyze walking performance metrics
        """
        metrics = {}

        # Calculate walking speed
        com_trajectory = np.array(simulation_data['com_trajectory'])
        total_distance = np.sum(np.sqrt(np.sum(np.diff(com_trajectory[:, :2], axis=0)**2, axis=1)))
        total_time = simulation_data['time'][-1] - simulation_data['time'][0]
        avg_speed = total_distance / total_time if total_time > 0 else 0

        # Calculate ZMP tracking error
        zmp_trajectory = np.array(simulation_data['zmp_trajectory'])
        # Calculate how well ZMP stays within support polygon
        # (This would require support polygon data for each timestep)

        # Calculate energy efficiency
        joint_torques = np.array(simulation_data['joint_torques'])
        energy_consumption = np.sum(np.abs(joint_torques)) * self.dt

        metrics = {
            'average_speed': avg_speed,
            'total_distance': total_distance,
            'energy_efficiency': energy_consumption,
            'stability_score': self.calculate_stability_score(simulation_data),
            'step_consistency': self.calculate_step_consistency(simulation_data)
        }

        return metrics

    def calculate_stability_score(self, simulation_data):
        """
        Calculate stability score based on ZMP tracking
        """
        # Simplified stability calculation
        return 0.8  # Placeholder

    def calculate_step_consistency(self, simulation_data):
        """
        Calculate consistency of step patterns
        """
        # Simplified consistency calculation
        return 0.9  # Placeholder
```

## Summary

Bipedal locomotion is one of the most challenging aspects of humanoid robotics, requiring sophisticated control algorithms that integrate:

1. **Pattern Generation**: Creating stable CoM and foot trajectories
2. **ZMP Control**: Ensuring the Zero Moment Point stays within the support polygon
3. **Balance Control**: Maintaining stability during dynamic motion
4. **Terrain Adaptation**: Adjusting gait for different surfaces and obstacles
5. **Disturbance Recovery**: Handling unexpected forces and maintaining stability

The key techniques covered in this section include:

- **CoM trajectory generation** using inverted pendulum models
- **Footstep planning** for various walking patterns
- **ZMP-based control** for stable walking
- **Preview control** for improved ZMP tracking
- **Advanced techniques** for uneven terrain and disturbance recovery

Successful bipedal locomotion requires careful coordination of all these elements, along with real-time state estimation and control adaptation. The next section will focus specifically on balance and postural control, which is essential for stable bipedal locomotion.
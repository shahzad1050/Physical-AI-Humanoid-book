# Practical Lab: Creating Your First Robot Simulation

## Objective

In this lab, you will create a complete robot simulation environment using Gazebo and ROS 2. You will design a simple mobile robot, create its URDF model, set up a Gazebo world, and implement basic control and sensing capabilities.

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Gazebo (Harmonic or compatible version)
- Basic knowledge of URDF and ROS 2 concepts
- Terminal/command line familiarity

## Step 1: Create a New ROS 2 Package

First, create a workspace and package for your simulation:

```bash
# Create workspace directory
mkdir -p ~/ros2_simulation_ws/src
cd ~/ros2_simulation_ws/src

# Create a new package for the robot simulation
ros2 pkg create --build-type ament_python robot_simulation_pkg --dependencies rclpy std_msgs geometry_msgs sensor_msgs gazebo_ros_pkgs gazebo_ros
```

## Step 2: Create the Robot URDF Model

Create the directory structure for your robot model:

```bash
# Create URDF directory
mkdir -p ~/ros2_simulation_ws/src/robot_simulation_pkg/urdf

# Create meshes directory
mkdir -p ~/ros2_simulation_ws/src/robot_simulation_pkg/meshes
```

Create the main robot URDF file `~/ros2_simulation_ws/src/robot_simulation_pkg/urdf/my_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Robot base -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia
        ixx="0.425" ixy="0.0" ixz="0.0"
        iyy="0.425" iyz="0.0"
        izz="0.9"/>
    </inertial>
  </link>

  <!-- Left wheel -->
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

  <!-- Right wheel -->
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

  <!-- Castor wheel (for stability) -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
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

  <!-- Joints -->
  <joint name="base_to_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.25 -0.05" rpy="${M_PI/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.25 -0.05" rpy="${M_PI/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="base_to_caster" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.2 0 -0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <!-- Differential drive plugin -->
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>base_to_left_wheel</left_joint>
      <right_joint>base_to_right_wheel</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>

  <!-- Gazebo materials and properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="wheel_left">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_right">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="caster_wheel">
    <material>Gazebo/Grey</material>
  </gazebo>
</robot>
```

## Step 3: Create a Gazebo World File

Create the worlds directory and a simple world file:

```bash
mkdir -p ~/ros2_simulation_ws/src/robot_simulation_pkg/worlds
```

Create `~/ros2_simulation_ws/src/robot_simulation_pkg/worlds/simple_room.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_room">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Simple room with walls -->
    <model name="wall_1">
      <pose>-5 0 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_2">
      <pose>5 0 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_3">
      <pose>0 -5 2.5 0 0 1.5707</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_4">
      <pose>0 5 2.5 0 0 1.5707</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add some obstacles -->
    <model name="obstacle_1">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="obstacle_2">
      <pose>-2 -2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
```

## Step 4: Create a Robot State Publisher Node

Create a Python node to publish the robot state:

Create `~/ros2_simulation_ws/src/robot_simulation_pkg/robot_simulation_pkg/robot_state_publisher.py`:

```python
#!/usr/bin/env python3

"""
Robot State Publisher Node
Publishes joint states for visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer for publishing
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

        # Initialize joint positions
        self.wheel_left_pos = 0.0
        self.wheel_right_pos = 0.0

        self.get_logger().info('Robot State Publisher started')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = ['base_to_left_wheel', 'base_to_right_wheel']
        msg.position = [self.wheel_left_pos, self.wheel_right_pos]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]

        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish joint states
        self.joint_pub.publish(msg)

        # Broadcast transforms
        self.broadcast_transforms()

        # Update wheel positions (simulated motion)
        self.wheel_left_pos += 0.01
        self.wheel_right_pos += 0.01

    def broadcast_transforms(self):
        # Publish transform for base_link to odom (if available)
        try:
            # This would come from odometry in a real system
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().warn(f'Could not broadcast transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()

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

## Step 5: Create a Simple Controller Node

Create `~/ros2_simulation_ws/src/robot_simulation_pkg/robot_simulation_pkg/simple_controller.py`:

```python
#!/usr/bin/env python3

"""
Simple Controller Node
Subscribes to velocity commands and publishes them to the robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Create subscriber for velocity commands
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create subscriber for laser scan (for obstacle detection)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Create publisher for velocity commands (in case we need to modify them)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

        # Parameters
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # Initialize variables
        self.scan_data = None

        self.get_logger().info('Simple Controller initialized')

    def cmd_vel_callback(self, msg):
        # Process the command (in this simple case, just log it)
        self.get_logger().info(f'Received command: linear={msg.linear.x}, angular={msg.angular.z}')

        # Check for obstacles and modify command if needed
        if self.scan_data is not None:
            modified_msg = self.avoid_obstacles(msg)
            self.cmd_pub.publish(modified_msg)
        else:
            self.cmd_pub.publish(msg)

    def scan_callback(self, msg):
        # Store the latest scan data
        self.scan_data = msg

    def avoid_obstacles(self, original_cmd):
        # Create a new command message
        new_cmd = Twist()

        # Check if there are obstacles in front
        front_ranges = []
        for i, distance in enumerate(self.scan_data.ranges):
            if not math.isnan(distance) and not math.isinf(distance):
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                # Check front 60 degrees
                if -math.pi/6 < angle < math.pi/6:
                    front_ranges.append(distance)

        if front_ranges and min(front_ranges) < self.safety_distance:
            # Obstacle detected, stop and turn
            new_cmd.linear.x = 0.0
            new_cmd.angular.z = self.max_angular_speed
            self.get_logger().warn(f'Obstacle detected at {min(front_ranges):.2f}m, turning!')
        else:
            # Path is clear, follow original command but with speed limits
            new_cmd.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, original_cmd.linear.x))
            new_cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, original_cmd.angular.z))

        return new_cmd


def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()

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

## Step 6: Create Launch Files

Create the launch directory:

```bash
mkdir -p ~/ros2_simulation_ws/src/robot_simulation_pkg/launch
```

Create `~/ros2_simulation_ws/src/robot_simulation_pkg/launch/robot_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('robot_simulation_pkg')

    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    world = DeclareLaunchArgument(
        'world',
        default_value='simple_room.world',
        description='Choose one of the world files from `/robot_simulation_pkg/worlds`'
    )

    # Launch Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gz', 'sim', '-r', PathJoinSubstitution([pkg_dir, 'worlds', LaunchConfiguration('world')])],
        output='screen'
    )

    # Launch Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open(PathJoinSubstitution([pkg_dir, 'urdf', 'my_robot.urdf']).perform({})).read()
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Robot State Publisher node (custom)
    custom_robot_state_publisher = Node(
        package='robot_simulation_pkg',
        executable='robot_state_publisher',
        name='custom_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Simple controller node
    simple_controller = Node(
        package='robot_simulation_pkg',
        executable='simple_controller',
        name='simple_controller',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'safety_distance': 0.5},
            {'max_linear_speed': 0.5},
            {'max_angular_speed': 1.0}
        ]
    )

    return LaunchDescription([
        use_sim_time,
        world,
        gzserver,
        # gzclient,  # Uncomment if you want the GUI
        robot_state_publisher,
        spawn_entity,
        custom_robot_state_publisher,
        simple_controller
    ])
```

## Step 7: Update setup.py

Update the `setup.py` file to include entry points:

```python
from setuptools import setup

package_name = 'robot_simulation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/my_robot.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/simple_room.world']),
        ('share/' + package_name + '/launch', ['launch/robot_simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot simulation package for lab exercise',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_publisher = robot_simulation_pkg.robot_state_publisher:main',
            'simple_controller = robot_simulation_pkg.simple_controller:main',
        ],
    },
)
```

## Step 8: Build and Run the Simulation

Build your package:

```bash
cd ~/ros2_simulation_ws
colcon build --packages-select robot_simulation_pkg
source install/setup.bash
```

Run the simulation:

```bash
# Terminal 1: Launch the simulation
ros2 launch robot_simulation_pkg robot_simulation.launch.py

# Terminal 2: Send velocity commands (after simulation is running)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

## Step 9: Test and Monitor

Monitor various topics to see the simulation in action:

```bash
# Check joint states
ros2 topic echo /joint_states

# Check odometry
ros2 topic echo /odom

# Check laser scan data
ros2 topic echo /scan

# Check TF transforms
ros2 run tf2_tools view_frames
```

## Step 10: Advanced Testing

Try different commands to see how the robot responds:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}' --times 10

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}' --times 10

# Square pattern
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}' --times 10
sleep 2
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}' --times 10
```

## Expected Results

When you run the simulation:

1. The Gazebo environment should start with your robot model
2. The robot should appear in the simple room world with walls and obstacles
3. When you send velocity commands, the robot should move in Gazebo
4. The robot should publish odometry data showing its position and orientation
5. The obstacle avoidance controller should respond to laser scan data

## Troubleshooting

If you encounter issues:

1. **Model not appearing**: Check that the URDF is valid and the spawn command is correct
2. **No movement**: Verify that the differential drive plugin is properly configured
3. **TF errors**: Ensure the robot_state_publisher is running
4. **Gazebo crashes**: Try reducing physics complexity or increasing resources

## Extensions

To extend this simulation:

1. **Add sensors**: Include camera, IMU, or other sensor models in the URDF
2. **Improve obstacle avoidance**: Implement more sophisticated navigation algorithms
3. **Add more complex environments**: Create larger, more detailed worlds
4. **Implement SLAM**: Add mapping and localization capabilities

## Summary

In this lab, you have successfully:
- Created a complete robot model in URDF format
- Set up a Gazebo simulation environment
- Implemented a basic control system with obstacle avoidance
- Connected ROS 2 nodes to the simulation
- Tested the system with various commands

This provides a foundation for more complex robotic simulation scenarios and demonstrates the integration between ROS 2 and Gazebo for robotics development.
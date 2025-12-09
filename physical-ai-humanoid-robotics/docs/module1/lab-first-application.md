# Practical Lab: Building Your First ROS 2 Application

## Objective

In this lab, you will create a complete ROS 2 application that demonstrates the core concepts of nodes, topics, publishers, subscribers, and parameters. By the end of this lab, you will have built a simple robot controller that publishes sensor data and responds to commands.

## Prerequisites

- ROS 2 installed (Humble Hawksbill or later recommended)
- Python 3.8 or higher
- Basic Python programming knowledge
- Terminal/command line familiarity

## Step 1: Create a New ROS 2 Package

First, create a new workspace and package for your application:

```bash
# Create workspace directory
mkdir -p ~/ros2_labs/src
cd ~/ros2_labs/src

# Create a new package
ros2 pkg create --build-type ament_python robot_controller_pkg --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

## Step 2: Create the Publisher Node

Create a file `~/ros2_labs/src/robot_controller_pkg/robot_controller_pkg/sensor_publisher.py`:

```python
#!/usr/bin/env python3

"""
Sensor Publisher Node
Publishes simulated sensor data (laser scan and IMU)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math
import random


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publishers
        self.laser_pub = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_level', 10)

        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('sensor_range', 10.0)

        # Get parameter values
        self.rate = self.get_parameter('publish_rate').value
        self.range = self.get_parameter('sensor_range').value

        # Create timer
        self.timer = self.create_timer(1.0 / self.rate, self.publish_sensor_data)

        self.get_logger().info(f'Sensor publisher started with rate: {self.rate}Hz')

    def publish_sensor_data(self):
        # Publish laser scan
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser_frame'
        laser_msg.angle_min = -math.pi / 2
        laser_msg.angle_max = math.pi / 2
        laser_msg.angle_increment = math.pi / 180  # 1 degree
        laser_msg.time_increment = 0.0
        laser_msg.scan_time = 1.0 / self.rate
        laser_msg.range_min = 0.1
        laser_msg.range_max = self.range

        # Generate simulated laser data
        num_readings = int((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1
        laser_msg.ranges = []
        for i in range(num_readings):
            # Simulate some obstacles at specific angles
            angle = laser_msg.angle_min + i * laser_msg.angle_increment
            distance = self.range  # Default: no obstacle
            if -0.5 < angle < 0.5:  # Front obstacle
                distance = 2.0 + random.uniform(-0.2, 0.2)
            elif 0.8 < abs(angle) < 1.2:  # Side obstacles
                distance = 3.0 + random.uniform(-0.3, 0.3)
            laser_msg.ranges.append(distance)

        self.laser_pub.publish(laser_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_frame'

        # Simulate slight orientation changes
        imu_msg.orientation.x = random.uniform(-0.01, 0.01)
        imu_msg.orientation.y = random.uniform(-0.01, 0.01)
        imu_msg.orientation.z = random.uniform(-0.01, 0.01)
        imu_msg.orientation.w = 1.0  # Normalize

        # Simulate small angular velocities
        imu_msg.angular_velocity.x = random.uniform(-0.001, 0.001)
        imu_msg.angular_velocity.y = random.uniform(-0.001, 0.001)
        imu_msg.angular_velocity.z = random.uniform(-0.001, 0.001)

        # Simulate linear accelerations
        imu_msg.linear_acceleration.x = random.uniform(-0.1, 0.1)
        imu_msg.linear_acceleration.y = random.uniform(-0.1, 0.1)
        imu_msg.linear_acceleration.z = 9.8 + random.uniform(-0.1, 0.1)  # Gravity

        self.imu_pub.publish(imu_msg)

        # Publish battery level
        battery_msg = Float32()
        battery_msg.data = max(0.0, 100.0 - (self.get_clock().now().nanoseconds / 1e9) % 100.0)
        self.battery_pub.publish(battery_msg)


def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 3: Create the Subscriber Node

Create a file `~/ros2_labs/src/robot_controller_pkg/robot_controller_pkg/command_subscriber.py`:

```python
#!/usr/bin/env python3

"""
Command Subscriber Node
Subscribes to sensor data and publishes commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32
import math


class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')

        # Create publisher for robot commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscribers
        self.laser_sub = self.create_subscription(LaserScan, 'laser_scan', self.laser_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.battery_sub = self.create_subscription(Float32, 'battery_level', self.battery_callback, 10)

        # Declare parameters
        self.declare_parameter('safety_distance', 1.0)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)

        # Get parameter values
        self.safety_distance = self.get_parameter('safety_distance').value
        self.max_linear = self.get_parameter('max_linear_velocity').value
        self.max_angular = self.get_parameter('max_angular_velocity').value

        # Initialize sensor data storage
        self.laser_data = None
        self.imu_data = None
        self.battery_level = 100.0

        # Create timer for command generation
        self.timer = self.create_timer(0.1, self.generate_command)

        self.get_logger().info('Command subscriber initialized')

    def laser_callback(self, msg):
        self.laser_data = msg
        self.get_logger().debug(f'Laser data received, {len(msg.ranges)} ranges')

    def imu_callback(self, msg):
        self.imu_data = msg
        # Log orientation (simplified)
        self.get_logger().debug(f'IMU orientation: ({msg.orientation.x:.3f}, {msg.orientation.y:.3f})')

    def battery_callback(self, msg):
        self.battery_level = msg.data
        if self.battery_level < 20.0:
            self.get_logger().warn(f'Low battery: {self.battery_level:.1f}%')

    def generate_command(self):
        if self.laser_data is None:
            return  # Wait for laser data

        # Create command message
        cmd_msg = Twist()

        # Check for obstacles in front
        front_ranges = []
        for i, distance in enumerate(self.laser_data.ranges):
            angle = self.laser_data.angle_min + i * self.laser_data.angle_increment
            if -0.5 < angle < 0.5:  # Front 60-degree sector
                if not math.isnan(distance) and distance < self.safety_distance:
                    front_ranges.append(distance)

        if front_ranges and min(front_ranges) < self.safety_distance:
            # Obstacle detected, turn away
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.max_angular if min(front_ranges) < self.safety_distance * 0.7 else self.max_angular / 2
            self.get_logger().info(f'Obstacle detected! Distance: {min(front_ranges):.2f}, turning')
        else:
            # Clear path, move forward
            cmd_msg.linear.x = self.max_linear
            cmd_msg.angular.z = 0.0

        # Publish command
        self.cmd_pub.publish(cmd_msg)

        # Log current state
        self.get_logger().info(
            f'Cmd: linear={cmd_msg.linear.x:.2f}, angular={cmd_msg.angular.z:.2f}, '
            f'battery={self.battery_level:.1f}%'
        )


def main(args=None):
    rclpy.init(args=args)
    command_subscriber = CommandSubscriber()

    try:
        rclpy.spin(command_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        command_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4: Create a Launch File

Create a launch directory and file:

```bash
mkdir -p ~/ros2_labs/src/robot_controller_pkg/launch
```

Create `~/ros2_labs/src/robot_controller_pkg/launch/robot_system.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Sensor publishing rate'
    )

    safety_distance = DeclareLaunchArgument(
        'safety_distance',
        default_value='1.0',
        description='Minimum safe distance'
    )

    return LaunchDescription([
        use_sim_time,
        publish_rate,
        safety_distance,

        # Sensor publisher node
        Node(
            package='robot_controller_pkg',
            executable='sensor_publisher',
            name='sensor_publisher',
            parameters=[
                {'publish_rate': LaunchConfiguration('publish_rate')},
                {'sensor_range': 10.0}
            ],
            output='screen'
        ),

        # Command subscriber node
        Node(
            package='robot_controller_pkg',
            executable='command_subscriber',
            name='command_subscriber',
            parameters=[
                {'safety_distance': LaunchConfiguration('safety_distance')},
                {'max_linear_velocity': 0.5},
                {'max_angular_velocity': 1.0}
            ],
            output='screen'
        )
    ])
```

## Step 5: Update setup.py

Update the `setup.py` file in your package to include entry points:

```python
from setuptools import setup

package_name = 'robot_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot controller package for lab exercise',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = robot_controller_pkg.sensor_publisher:main',
            'command_subscriber = robot_controller_pkg.command_subscriber:main',
        ],
    },
)
```

## Step 6: Build and Run

Build your package:

```bash
cd ~/ros2_labs
colcon build --packages-select robot_controller_pkg
source install/setup.bash
```

Run the system:

```bash
# Option 1: Run the launch file
ros2 launch robot_controller_pkg robot_system.launch.py

# Option 2: Run nodes separately
# Terminal 1:
ros2 run robot_controller_pkg sensor_publisher

# Terminal 2:
ros2 run robot_controller_pkg command_subscriber
```

## Step 7: Monitor the System

In another terminal, monitor the topics:

```bash
# List active topics
ros2 topic list

# Echo laser scan data
ros2 topic echo /laser_scan

# Echo robot commands
ros2 topic echo /cmd_vel

# Check parameters
ros2 param list /command_subscriber
```

## Step 8: Experiment with Parameters

Try changing parameters while the system is running:

```bash
# Change safety distance
ros2 param set /command_subscriber safety_distance 2.0

# Change publish rate
ros2 param set /sensor_publisher publish_rate 20.0
```

## Expected Results

When you run the system:

1. The sensor publisher should publish laser scan, IMU, and battery data
2. The command subscriber should process the sensor data and generate appropriate commands
3. When obstacles are detected in front of the robot, it should turn to avoid them
4. The robot should continue moving forward when the path is clear
5. Low battery warnings should appear when the simulated battery level drops

## Troubleshooting

If you encounter issues:

1. **Package not found**: Make sure you sourced the setup file after building
2. **Node not launching**: Check that entry points are correctly defined in setup.py
3. **No communication**: Verify that both nodes are on the same ROS domain
4. **Permission errors**: Ensure your Python files have execute permissions

## Summary

In this lab, you have:
- Created a ROS 2 package with multiple nodes
- Implemented publisher and subscriber nodes
- Used parameters for configuration
- Created a launch file to run the complete system
- Monitored and modified parameters at runtime

This exercise demonstrates the core concepts of ROS 2: nodes, topics, parameters, and launch files. You've built a simple but complete robotic system that processes sensor data and generates appropriate commands based on that data.
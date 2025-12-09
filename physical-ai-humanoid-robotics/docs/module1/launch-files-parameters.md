# Launch Files and Parameter Management

## Launch Files

Launch files in ROS 2 allow you to start multiple nodes with a single command and configure them with specific parameters. They are written in Python using the `launch` library.

### Basic Launch File Structure

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='talker',
            name='talker_node',
            parameters=[
                {'frequency': 1.0},
                {'robot_name': 'my_robot'}
            ],
            remappings=[
                ('/chatter', '/custom_chatter')
            ],
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='listener',
            name='listener_node',
            output='screen'
        )
    ])
```

### Advanced Launch File Features

#### Conditional Launch

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    debug_mode = LaunchConfiguration('debug')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )

    # Conditional node
    debug_node = Node(
        package='my_robot_package',
        executable='debug_node',
        condition=IfCondition(debug_mode)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_debug,
        debug_node
    ])
```

#### Including Other Launch Files

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include another launch file
    other_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('other_package'),
                'launch',
                'other_launch.py'
            )
        )
    )

    return LaunchDescription([
        other_launch_file
    ])
```

### Launch File Parameters

Launch files can accept parameters that can be passed at runtime:

```bash
# Launch with parameters
ros2 launch my_robot_package my_launch.py use_sim_time:=true debug:=true
```

## Parameter Management

### Parameter Files (YAML)

Parameters can be stored in YAML files for easy management:

```yaml
# config/my_robot_params.yaml
my_robot_node:
  ros__parameters:
    # Basic parameters
    robot_name: "my_robot"
    frequency: 10.0
    max_velocity: 1.0

    # Nested parameters
    controller:
      kp: 1.0
      ki: 0.1
      kd: 0.05

    # Array parameters
    joint_names: ["joint1", "joint2", "joint3"]

    # Namespace parameters
    sensors:
      lidar_enabled: true
      camera_enabled: false
```

### Loading Parameters from YAML

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get path to parameter file
    params_file = os.path.join(
        get_package_share_directory('my_robot_package'),
        'config',
        'my_robot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='my_robot_node',
            name='my_robot_node',
            parameters=[params_file],
            output='screen'
        )
    ])
```

### Dynamic Parameter Reconfiguration

ROS 2 allows parameters to be changed at runtime:

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter


class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with descriptors
        self.declare_parameter(
            'velocity',
            0.5,
            ParameterDescriptor(
                name='velocity',
                type=ParameterType.PARAMETER_DOUBLE,
                description='Robot velocity',
                additional_constraints='Must be between 0.0 and 2.0',
                floating_point_range=[0.0, 2.0]
            )
        )

        # Create parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'velocity' and param.value.double_value > 2.0:
                return SetParameters.Response(
                    successful=False,
                    reason='Velocity must be <= 2.0'
                )
        return SetParameters.Response(successful=True)
```

## Command Line Parameter Tools

### Setting Parameters

```bash
# Set a parameter on a running node
ros2 param set /my_node frequency 2.0

# Get parameter value
ros2 param get /my_node frequency

# List all parameters for a node
ros2 param list /my_node

# Load parameters from file
ros2 param load /my_node /path/to/params.yaml

# Save parameters to file
ros2 param dump /my_node --output /path/to/params.yaml
```

## Parameter Best Practices

### 1. Organize Parameters Logically

```yaml
# Good organization
robot_controller:
  ros__parameters:
    # Control parameters
    control:
      frequency: 100.0
      max_acceleration: 2.0

    # Hardware parameters
    hardware:
      encoder_resolution: 4096
      motor_limits:
        max_current: 10.0
        max_voltage: 24.0
```

### 2. Use Appropriate Parameter Types

```python
# Declare parameters with appropriate types
self.declare_parameter('int_param', 42, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
self.declare_parameter('double_param', 3.14, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
self.declare_parameter('string_param', 'hello', ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
self.declare_parameter('bool_param', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
self.declare_parameter('array_param', [1, 2, 3], ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))
```

### 3. Validate Parameters

```python
def validate_parameters(self):
    velocity = self.get_parameter('velocity').value
    if velocity < 0.0 or velocity > 2.0:
        self.get_logger().error(f'Velocity {velocity} is out of range [0.0, 2.0]')
        return False
    return True
```

## Practical Example: Complete Launch System

Here's a complete example showing how to set up a robot system with launch files and parameters:

### Directory Structure
```
my_robot_package/
├── launch/
│   └── robot_system.launch.py
├── config/
│   ├── robot_params.yaml
│   └── controller_params.yaml
└── my_robot_package/
    ├── robot_controller.py
    └── sensor_processor.py
```

### robot_params.yaml
```yaml
robot_controller:
  ros__parameters:
    robot_name: "my_robot"
    control_frequency: 50.0
    max_velocity: 1.0
    safety:
      max_distance: 0.5
      emergency_stop: false

sensor_processor:
  ros__parameters:
    sensor_frequency: 30.0
    filter_enabled: true
    calibration:
      offset_x: 0.0
      offset_y: 0.0
```

### robot_system.launch.py
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('my_robot_package'),
            'config',
            'robot_params.yaml'
        ),
        description='Path to parameters file'
    )

    # Get launch configuration
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        params_file_arg,
        LogInfo(msg=['Loading parameters from: ', params_file]),

        # Robot controller node
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            parameters=[params_file],
            output='screen',
            respawn=True
        ),

        # Sensor processor node
        Node(
            package='my_robot_package',
            executable='sensor_processor',
            name='sensor_processor',
            parameters=[params_file],
            output='screen',
            respawn=True
        )
    ])
```

## Summary

Launch files and parameter management are crucial for configuring and running complex robotic systems. They allow you to:

1. Start multiple nodes with a single command
2. Configure nodes with parameters at launch time
3. Enable conditional launching based on arguments
4. Manage parameters in organized YAML files
5. Dynamically reconfigure parameters during runtime

Understanding these concepts will help you build more maintainable and configurable robotic applications.
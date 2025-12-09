# Gazebo Environment Setup

## Overview of Gazebo

Gazebo is a 3D simulation environment that enables accurate and efficient testing of robotics systems. It provides high-fidelity physics simulation, realistic rendering, and convenient programmatic interfaces. Gazebo is widely used in the robotics community and integrates seamlessly with ROS 2.

## Installing Gazebo

### Prerequisites

Before installing Gazebo, ensure you have ROS 2 installed. Gazebo Harmonic is the recommended version that works well with ROS 2 Humble Hawksbill.

### Installation on Ubuntu

```bash
# Add the Gazebo repository
sudo apt install software-properties-common
sudo add-apt-repository ppa:openrobotics/gazebo
sudo apt update

# Install Gazebo
sudo apt install gazebo
```

### Installation via Package Manager (Alternative)

```bash
# Install Gazebo and ROS 2 integration packages
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-dev
```

### Installation on Other Platforms

For macOS or Windows, refer to the official Gazebo documentation for platform-specific installation instructions.

## Gazebo Components

### 1. gz-sim (formerly Ignition Gazebo)

The core simulation engine that handles physics, rendering, and simulation management.

### 2. gz-gui (formerly Ignition GUI)

The user interface framework that provides the visual interface for interacting with simulations.

### 3. gz-fuel (formerly Ignition Fuel)

An online repository for sharing robot models, worlds, and other simulation assets.

## Basic Gazebo Concepts

### Worlds

Worlds define the environment in which robots operate. They include:
- Terrain and static objects
- Lighting conditions
- Physics parameters
- Robot spawn locations

Example world file (`example.world`):
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
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
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Models

Models represent robots, objects, and other entities in the simulation. They include:
- Physical properties (mass, inertia, friction)
- Visual appearance
- Sensors and actuators
- Joints and kinematic chains

## Running Gazebo

### Basic Launch

```bash
# Launch Gazebo with the default empty world
gz sim

# Launch Gazebo with a specific world file
gz sim -r my_world.sdf

# Launch Gazebo in verbose mode for debugging
gz sim -v 4 my_world.sdf
```

### Gazebo GUI Elements

When Gazebo launches, you'll see several interface elements:

1. **Main 3D View**: The primary simulation visualization
2. **World Info Panel**: Shows simulation time and world information
3. **Entity Tree**: Hierarchical view of all objects in the world
4. **Tools Panel**: Access to various simulation tools
5. **Time Panel**: Controls for simulation time and playback

## Gazebo with ROS 2 Integration

### Installation of ROS 2 Gazebo Packages

```bash
# Install ROS 2 Gazebo integration packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Launching Gazebo through ROS 2

```python
# Example launch file to start Gazebo with ROS 2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # World file argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Choose one of the world files from `/gazebo_ros_pkgs/gazebo_ros/worlds`'
    )

    # Launch Gazebo
    gzserver = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=[LaunchConfiguration('world'), '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient = Node(
        package='gazebo_ros',
        executable='gzclient',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gzserver,
        gzclient
    ])
```

## Gazebo Simulation Parameters

### Physics Engine Configuration

Gazebo supports multiple physics engines:
- **ODE** (Open Dynamics Engine): Default, good balance of speed and accuracy
- **Bullet**: Good for real-time applications
- **DART**: Advanced physics simulation
- **Simbody**: Multibody dynamics

Physics parameters can be configured in the world file:

```xml
<world name="default">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

### Common Physics Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| max_step_size | Maximum simulation time step | 0.001s |
| real_time_factor | Target simulation speed | 1.0 (real-time) |
| real_time_update_rate | Updates per second | 1000 Hz |

## Working with Gazebo Models

### Finding Models

Gazebo comes with a library of pre-built models:

```bash
# List available models
ls /usr/share/gazebo-*/models/

# Or check your local model path
echo $GAZEBO_MODEL_PATH
```

### Adding Custom Models

1. Create a model directory: `~/.gazebo/models/my_robot/`
2. Add a `model.config` file describing the model
3. Add the SDF model file
4. Include visual and collision meshes if needed

Example `model.config`:
```xml
<?xml version="1.0"?>
<model>
  <name>My Robot</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A custom robot model</description>
</model>
```

## Gazebo Services and Topics

Gazebo provides various ROS 2 services and topics for programmatic control:

### Common Services

- `/gazebo/set_model_state`: Set pose and velocity of a model
- `/gazebo/get_model_state`: Get current state of a model
- `/gazebo/spawn_entity`: Spawn a new entity in the simulation
- `/gazebo/delete_entity`: Remove an entity from the simulation

### Common Topics

- `/clock`: Simulation time (when using sim time)
- `/gazebo/model_states`: States of all models in the simulation

## Performance Optimization

### Graphics Settings

For better performance on less powerful hardware:

```bash
# Launch with software rendering (slower but more compatible)
MESA_GL_VERSION_OVERRIDE=3.3 gz sim

# Or disable GUI for headless operation
gz sim -s my_world.sdf
```

### Physics Settings

Adjust physics parameters for your specific needs:
- Increase `max_step_size` for better performance (less accuracy)
- Decrease `real_time_update_rate` for less CPU usage
- Use simpler collision geometries when possible

## Troubleshooting Common Issues

### Gazebo Won't Start

```bash
# Check if Gazebo is properly installed
gz --version

# Check for OpenGL issues
glxinfo | grep "OpenGL version"

# Clear Gazebo cache if needed
rm -rf ~/.gazebo
```

### Performance Issues

- Close other applications to free up resources
- Reduce world complexity
- Adjust graphics quality settings
- Use simpler physics parameters

### ROS 2 Connection Issues

```bash
# Check if ROS 2 nodes are communicating
ros2 topic list

# Verify Gazebo plugins are loaded
# Look for libgazebo_ros_init.so in the launch output
```

## Summary

Gazebo provides a powerful and flexible simulation environment for robotics development. Proper setup is crucial for effective simulation work. Understanding the core concepts, components, and integration with ROS 2 will enable you to create sophisticated simulation environments for testing and validating your robotic systems.

In the next section, we'll explore URDF and SDF formats, which are essential for defining robot models in Gazebo.
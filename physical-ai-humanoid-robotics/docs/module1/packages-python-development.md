# ROS 2 Packages and Python Development

## Package Structure

A ROS 2 package is a reusable, self-contained unit of software. The basic structure of a ROS 2 package is:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata
├── setup.py                # Python package setup
├── setup.cfg               # Installation configuration
├── my_robot_package/       # Python module
│   ├── __init__.py
│   ├── publisher_member_function.py
│   └── subscriber_member_function.py
└── test/                   # Test files
    └── test_copyright.py
    └── test_flake8.py
    └── test_pep257.py
```

## Creating a Python Package

### Using colcon

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python my_robot_package
```

### package.xml

The `package.xml` file contains metadata about the package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Example ROS 2 package in Python</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### setup.py

The `setup.py` file defines how the Python package should be built and installed:

```python
from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Example ROS 2 package in Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_robot_package.publisher_member_function:main',
            'listener = my_robot_package.subscriber_member_function:main',
        ],
    },
)
```

## Python Node Development

### Basic Node Structure

```python
#!/usr/bin/env python3

"""
Example ROS 2 Python node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Parameter Management

ROS 2 provides a parameter system for configuration:

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType


class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)

        # Get parameter values
        self.frequency = self.get_parameter('frequency').value
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value

        self.get_logger().info(
            f'Parameters - Frequency: {self.frequency}, '
            f'Robot: {self.robot_name}, Max Vel: {self.max_velocity}'
        )
```

## Testing in Python

### Unit Tests

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from my_robot_package.my_node import MyNode


class TestMyNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MyNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_node_creation(self):
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'my_node')

    def test_parameter_declaration(self):
        self.assertTrue(self.node.has_parameter('frequency'))
```

### Running Tests

```bash
# Run all tests
colcon test

# Run specific tests
colcon test --packages-select my_robot_package

# View test results
colcon test-result --all
```

## Best Practices for Python Development

1. **Code Style**: Follow PEP 8 guidelines
2. **Documentation**: Use docstrings for all public methods
3. **Type Hints**: Use type hints for better code clarity
4. **Error Handling**: Implement proper exception handling
5. **Logging**: Use the ROS 2 logging system
6. **Resource Management**: Properly clean up resources

### Example with Type Hints and Error Handling

```python
from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.publisher: Optional[rclpy.publisher.Publisher] = None
        self.setup_publisher()

    def setup_publisher(self) -> None:
        try:
            self.publisher = self.create_publisher(String, 'topic', 10)
            self.get_logger().info('Publisher created successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to create publisher: {e}')
            raise

    def safe_publish(self, message: str) -> bool:
        if self.publisher is not None:
            msg = String()
            msg.data = message
            try:
                self.publisher.publish(msg)
                return True
            except Exception as e:
                self.get_logger().error(f'Failed to publish message: {e}')
                return False
        else:
            self.get_logger().warning('Publisher not initialized')
            return False
```

## Building and Running

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
```

### Source the Workspace

```bash
source install/setup.bash
```

### Run the Node

```bash
ros2 run my_robot_package talker
```

## Summary

This section covered the fundamentals of creating and managing ROS 2 packages with Python. We explored the package structure, configuration files, and best practices for Python development in the ROS 2 ecosystem. Understanding these concepts is essential for building maintainable and robust robotic applications.
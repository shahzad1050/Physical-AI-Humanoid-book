# ROS 2 Architecture and Concepts

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is the next generation of the Robot Operating System, designed to address the needs of commercial robotics applications. Unlike its predecessor, ROS 2 is built from the ground up to support real-time systems, security, and multi-robot systems.

## Core Architecture

### DDS (Data Distribution Service)

ROS 2 is built on top of DDS (Data Distribution Service), a middleware standard that provides a publisher-subscriber communication pattern. DDS handles the underlying network communication, serialization, and discovery mechanisms.

```
┌─────────────┐          ┌─────────────┐          ┌─────────────┐
│   Node A    │          │             │          │   Node B    │
│             │◄─────────┤   DDS       │─────────►│             │
│ Publisher   │          │   Middleware│          │ Subscriber  │
└─────────────┘          └─────────────┘          └─────────────┘
```

### Nodes

A node is the fundamental unit of computation in ROS 2. It's a process that performs computation and communicates with other nodes through ROS 2 communication primitives.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal_node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Communication Primitives

ROS 2 provides four main communication patterns:

1. **Topics**: Asynchronous publisher-subscriber communication
2. **Services**: Synchronous request-response communication
3. **Actions**: Asynchronous request-response with feedback and goal preemption
4. **Parameters**: Key-value storage for configuration

## Quality of Service (QoS)

QoS profiles allow you to configure how data is published and subscribed, including reliability, durability, and liveliness policies.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Configure QoS for real-time communication
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

## Security in ROS 2

ROS 2 includes built-in security features:
- Authentication: Verifying identity of nodes
- Authorization: Controlling what nodes can do
- Encryption: Protecting data in transit

## Summary

ROS 2 provides a robust architecture for building complex robotic systems. Its DDS-based communication layer, combined with security features and real-time capabilities, makes it suitable for commercial robotics applications.

In the next section, we'll explore the different communication primitives in detail.
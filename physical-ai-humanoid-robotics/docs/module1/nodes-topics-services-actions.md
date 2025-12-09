# Nodes, Topics, Services, and Actions

## Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system.

### Creating a Node

```python
import rclpy
from rclpy.node import Node

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Node Lifecycle

Nodes in ROS 2 have a well-defined lifecycle with states:
- Unconfigured
- Inactive
- Active
- Finalized

## Topics and Publishers/Subscribers

Topics enable asynchronous communication between nodes using a publish-subscribe pattern.

### Publisher Example

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello from publisher'
        self.publisher.publish(msg)
```

### Subscriber Example

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services

Services provide synchronous request-response communication between nodes.

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

Actions are used for long-running tasks that provide feedback and can be canceled.

### Action Server

```python
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Communication Patterns Comparison

| Pattern | Type | Use Case | Example |
|---------|------|----------|---------|
| Topics | Asynchronous | Continuous data streams | Sensor data, robot pose |
| Services | Synchronous | Request-response | Map loading, coordinate transformation |
| Actions | Asynchronous with feedback | Long-running tasks | Navigation, trajectory execution |

## Best Practices

1. **Node Design**: Keep nodes focused on a single responsibility
2. **Topic Naming**: Use descriptive, consistent naming conventions
3. **QoS Settings**: Match QoS settings between publishers and subscribers
4. **Error Handling**: Implement proper error handling and logging
5. **Resource Management**: Properly clean up resources in destructors

## Summary

Understanding the different communication patterns in ROS 2 is crucial for designing effective robotic systems. Topics are ideal for continuous data streams, services for request-response interactions, and actions for long-running tasks with feedback.
---
sidebar_position: 3
---

# Nodes, Topics & Services

Learn how ROS 2 nodes communicate through topics and services to build distributed robot systems.

## Nodes: The Building Blocks

A **node** is an independent process performing a specific task. Well-designed nodes follow the Single Responsibility Principle.

### Creating a Node

```python
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('Robot controller started!')

def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Node Lifecycle

Nodes can have managed lifecycles for production systems:
- `Unconfigured`: Initial state
- `Inactive`: Configured but not active
- `Active`: Fully operational
- `Finalized`: Shutting down

## Topics: Publish-Subscribe Communication

Topics enable asynchronous, decoupled communication.

### Publishing to a Topic

```python
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0
    
    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1
```

### Subscribing to a Topic

```python
class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Services: Request-Response Pattern

Services provide synchronous request-response communication.

### Creating a Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Calling a Service

```python
class ClientNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.send_request(3, 5)
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

## Custom Messages

Define custom message types for your robot:

### Creating a Message

Create `msg/RobotStatus.msg`:
```
string robot_name
float32 battery_level
bool is_moving
int32 error_code
```

### Using Custom Messages

```python
from my_robot_msgs.msg import RobotStatus

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
    
    def publish_status(self):
        msg = RobotStatus()
        msg.robot_name = 'Humanoid-01'
        msg.battery_level = 85.5
        msg.is_moving = True
        msg.error_code = 0
        self.publisher.publish(msg)
```

## Quality of Service (QoS)

Control communication reliability and performance:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For sensor data (best effort, volatile)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# For commands (reliable, volatile)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

self.sensor_sub = self.create_subscription(
    LaserScan, '/scan', callback, sensor_qos)
```

## Best Practices

1. **Name Nodes Descriptively**: Use clear, hierarchical names
2. **Namespace Your Topics**: Organize topics logically (`/robot1/camera/image`)
3. **Choose Appropriate QoS**: Match QoS to your data characteristics
4. **Handle Errors Gracefully**: Always check for service availability
5. **Log Appropriately**: Use debug/info/warn/error levels correctly

## Next Steps

- Learn about [Python Integration](/docs/module1/python-rclpy) patterns
- Study [URDF Modeling](/docs/module1/urdf) for robot description
- Complete Lab 1: Build a multi-node sensor system

---

*Understanding nodes, topics, and services is fundamental to building any ROS 2 system.*

---
sidebar_position: 4
---

# Python Integration with rclpy

Master Python robotics development using rclpy, the Python client library for ROS 2.

## Why Python for Robotics?

**Advantages**:
- Rapid prototyping and development
- Rich ecosystem (NumPy, OpenCV, TensorFlow)
- Easy integration with AI/ML frameworks
- Readable, maintainable code

**Trade-offs**:
- Slower than C++ for compute-intensive tasks
- GIL (Global Interpreter Lock) limits multithreading
- Higher memory usage

## rclpy Basics

### Initialization Pattern

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create node
    node = MyRobotNode()
    
    try:
        # Spin (process callbacks)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Class Structure

```python
class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        
        # Parameters
        self.declare_parameter('robot_name', 'default_robot')
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'status', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Timers
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Services
        self.reset_srv = self.create_service(
            Trigger, 'reset', self.reset_callback)
    
    def cmd_callback(self, msg):
        # Handle commands
        pass
    
    def timer_callback(self):
        # Periodic tasks
        pass
    
    def reset_callback(self, request, response):
        # Service handler
        response.success = True
        return response
```

## Advanced Patterns

### Multithreading with Executors

```python
from rclpy.executors import MultiThreadedExecutor
import threading

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
    
    def scan_callback(self, msg):
        # CPU-intensive processing
        processed_data = self.process_scan(msg)

def main():
    rclpy.init()
    
    # Use multiple threads
    executor = MultiThreadedExecutor(num_threads=4)
    
    node1 = SensorNode()
    node2 = ControlNode()
    
    executor.add_node(node1)
    executor.add_node(node2)
    
    executor.spin()
```

### Parameter Handling

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # Declare parameters with defaults
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('enable_logging', True)
        
        # Get parameter values
        rate = self.get_parameter('update_rate').value
        name = self.get_parameter('robot_name').value
        
        # Add parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'update_rate':
                self.get_logger().info(f'Update rate changed to {param.value}')
        return SetParametersResult(successful=True)
```

### Actions (Long-Running Tasks)

```python
from rclpy.action import ActionServer, ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci',
            execute_callback=self.execute_callback)
    
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            # Check if goal is cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            
            # Compute next number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + 
                feedback_msg.partial_sequence[i-1])
            
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            
            await asyncio.sleep(1)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```

## Integration with NumPy and OpenCV

### Image Processing

```python
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.publisher = self.create_publisher(
            Image, '/camera/image_processed', 10)
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Process image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Convert back to ROS Image
        edge_msg = self.bridge.cv2_to_imgmsg(edges, 'mono8')
        edge_msg.header = msg.header
        
        # Publish
        self.publisher.publish(edge_msg)
```

### Point Cloud Processing

```python
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pc_callback, 10)
    
    def pc_callback(self, msg):
        # Convert to NumPy array
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), 
                                                skip_nans=True)))
        
        # Process points
        distances = np.linalg.norm(points, axis=1)
        close_points = points[distances < 2.0]
        
        self.get_logger().info(f'Found {len(close_points)} points within 2m')
```

## Best Practices

### 1. Proper Error Handling

```python
def safe_callback(self, msg):
    try:
        result = self.process_message(msg)
    except ValueError as e:
        self.get_logger().error(f'Invalid data: {e}')
        return
    except Exception as e:
        self.get_logger().fatal(f'Unexpected error: {e}')
        raise
```

### 2. Resource Management

```python
def __del__(self):
    # Cleanup resources
    if hasattr(self, 'camera'):
        self.camera.release()
```

### 3. Logging Levels

```python
self.get_logger().debug('Detailed debugging information')
self.get_logger().info('Normal operation info')
self.get_logger().warn('Warning message')
self.get_logger().error('Recoverable error')
self.get_logger().fatal('Fatal error, shutting down')
```

### 4. Package Structure

```
my_robot_package/
├── my_robot_package/
│   ├── __init__.py
│   ├── robot_node.py
│   ├── utils.py
│   └── config.py
├── launch/
│   └── robot.launch.py
├── config/
│   └── params.yaml
├── test/
│   └── test_robot.py
├── package.xml
└── setup.py
```

## Testing

### Unit Testing

```python
import unittest
from my_robot_package.robot_node import RobotController

class TestRobotController(unittest.TestCase):
    def test_initialization(self):
        rclpy.init()
        node = RobotController()
        self.assertIsNotNone(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
```

## Next Steps

- Study [URDF Modeling](/docs/module1/urdf) for robot description
- Complete Lab 2: Build an image processing pipeline
- Explore [Gazebo Simulation](/docs/module2/gazebo)

---

*Python and rclpy provide a powerful, flexible platform for robotics development.*

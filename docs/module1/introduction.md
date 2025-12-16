---
sidebar_position: 1
---

# ROS 2 Introduction

Robot Operating System 2 (ROS 2) is the backbone of modern robotics software development. This chapter introduces the architecture, philosophy, and core concepts that make ROS 2 the industry standard.

## What is ROS 2?

ROS 2 is **not** an operating system despite its name. It's a flexible **middleware framework** that provides:

- **Communication Infrastructure**: Message passing between distributed processes
- **Hardware Abstraction**: Uniform interfaces for diverse sensors and actuators
- **Development Tools**: Build systems, debugging utilities, and visualization
- **Package Ecosystem**: Thousands of pre-built libraries for common robotics tasks

### Evolution from ROS 1

ROS 1 served the robotics community for over a decade, but faced limitations:

| Limitation | Impact | ROS 2 Solution |
|-----------|--------|---------------|
| Single master node | Single point of failure | Distributed discovery (DDS) |
| No real-time support | Unsafe for critical systems | Real-time compatible middleware |
| TCP-only | High latency | UDP, shared memory options |
| No security | Vulnerable to attacks | DDS security (SROS2) |
| Linux-focused | Limited portability | Cross-platform (Windows, macOS) |

ROS 2 addresses these with a complete architectural redesign built on **Data Distribution Service (DDS)**.

## Core Architecture

### DDS Middleware

ROS 2 uses DDS (Data Distribution Service) as its communication layer:

```
Application Layer (Your Code)
        ↓
    rclpy / rclcpp (ROS Client Libraries)
        ↓
    rcl (ROS Core Library)
        ↓
    rmw (ROS Middleware Interface)
        ↓
    DDS Implementation (FastDDS, CycloneDDS, etc.)
        ↓
    Network Transport (UDP/TCP/Shared Memory)
```

**Key DDS Features**:
- **Publish-Subscribe**: Decoupled, scalable communication
- **Quality of Service**: Reliability, durability, and latency control
- **Automatic Discovery**: No central broker needed
- **Type Safety**: Strong typing of messages

### Computational Graph

ROS 2 systems form a **computational graph**:

```
 [Camera Node] ──image──> [Object Detector]
                              │
                          detections
                              ↓
 [Planner] <───────────> [Controller] ──cmd──> [Motor Node]
              path              ↑
                                │
                           odom [Odometry]
```

**Nodes**: Independent processes that perform computation  
**Topics**: Named channels for asynchronous data streams  
**Services**: Synchronous request-reply interactions  
**Actions**: Long-running tasks with feedback and cancellation  

## Installation

### Prerequisites

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Architecture**: x86_64 (ARM64 also supported)
- **Python**: 3.10+

### Install ROS 2 Humble

```bash
# Set up sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

### Environment Setup

Add to your `~/.bashrc`:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Optional: Auto-completion
eval "$(register-python-argcomplete3 ros2)"
```

### Verify Installation

```bash
source ~/.bashrc
ros2 --version  # Should show Humble version

# Run demo
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

You should see the listener receiving messages from the talker.

## First ROS 2 Program

### Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Simple Publisher (Python)

Create `~/ros2_ws/src/minimal_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run It

```bash
python3 minimal_publisher.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /topic
```

## Key Concepts

### Nodes

A **node** is a single computational process. Best practices:

- **Single Responsibility**: Each node should do one thing well
- **Configurable**: Use parameters for flexibility
- **Discoverable**: Choose meaningful node names
- **Robust**: Handle errors gracefully

### Topics

**Topics** enable anonymous publish-subscribe communication:

```python
# Publisher
self.pub = self.create_publisher(String, '/robot/status', 10)

# Subscriber
self.sub = self.create_subscription(String, '/robot/status', 
                                     self.callback, 10)
```

**When to use topics**:
- Continuous data streams (sensor readings)
- One-to-many communication (broadcasts)
- Fire-and-forget messaging

### Quality of Service (QoS)

QoS policies control communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.pub = self.create_publisher(String, '/topic', qos)
```

**Common QoS profiles**:
- **Sensor Data**: Best effort, volatile (fast, lossy)
- **Parameters**: Reliable, transient local (persistent)
- **Services**: Reliable, volatile (guaranteed delivery)

## ROS 2 CLI Tools

Essential command-line tools:

```bash
# List nodes
ros2 node list

# Node information
ros2 node info /my_node

# List topics
ros2 topic list

# Topic info and echo
ros2 topic info /my_topic
ros2 topic echo /my_topic

# Publish from command line
ros2 topic pub /my_topic std_msgs/String "data: 'Hello'"

# Service calls
ros2 service call /my_service std_srvs/Trigger

# Parameter operations
ros2 param list
ros2 param get /my_node my_parameter
ros2 param set /my_node my_parameter 42
```

## Next Steps

Now that you understand ROS 2 basics:

1. **Explore**: Try the [demo nodes](https://docs.ros.org/en/humble/Tutorials.html)
2. **Build**: Create your first package in the next section
3. **Learn**: Study [Nodes and Topics](/docs/module1/nodes-topics) in detail
4. **Practice**: Complete Lab 1 exercises

## Additional Resources

- [Official ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Articles](https://design.ros2.org/)
- [ROS Discourse Forum](https://discourse.ros.org/)
- [ROS 2 GitHub](https://github.com/ros2)

---

*ROS 2 provides the foundation for everything we'll build in this course. Take time to understand these concepts—they'll serve you throughout your robotics career.*

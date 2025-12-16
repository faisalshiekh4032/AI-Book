---
sidebar_position: 2
---

# ROS 2 Architecture Deep Dive

Understanding ROS 2's layered architecture is crucial for debugging issues, optimizing performance, and making informed design decisions.

## Layered Architecture

ROS 2 is built as a stack of abstraction layers:

```
┌─────────────────────────────────────┐
│     Application Code (Your Nodes)   │
├─────────────────────────────────────┤
│  rclpy / rclcpp (Client Libraries)  │
├─────────────────────────────────────┤
│      rcl (ROS Client Library)       │
├─────────────────────────────────────┤
│    rmw (ROS Middleware Interface)   │
├─────────────────────────────────────┤
│   DDS Implementations (FastDDS,     │
│     CycloneDDS, Connext DDS)        │
├─────────────────────────────────────┤
│       Operating System / Network    │
└─────────────────────────────────────┘
```

Each layer serves a specific purpose and can be understood independently.

## Layer-by-Layer Breakdown

### Application Layer

Your robot code using Python (rclpy) or C++ (rclcpp).

**Key Features**:
- High-level APIs for nodes, topics, services, actions
- Pythonic/idiomatic interfaces
- Language-specific best practices

**Example**:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Your robot logic here
```

### Client Libraries (rclpy/rclcpp)

Language-specific implementations of ROS 2 concepts.

**Responsibilities**:
- Memory management (especially in C++)
- Callback execution
- Timer management
- Executor threading models

**Python (rclpy)**:
- Easier to learn and prototype
- Slower execution (GIL limitations)
- Less control over memory

**C++ (rclcpp)**:
- Maximum performance
- Manual memory management required
- Better for real-time systems

### ROS Client Library (rcl)

Language-agnostic C library implementing core ROS 2 logic.

**Why it exists**: 
- Avoid duplicating logic across language bindings
- Provides consistent behavior
- Easier to maintain and test

**What it does**:
- Manages lifecycle of nodes, publishers, subscribers
- Handles discovery and graph changes
- Implements QoS policy logic
- Coordinates with rmw layer

### ROS Middleware Interface (rmw)

Abstract interface that allows ROS 2 to work with different DDS implementations.

**Benefits**:
- **Vendor Neutrality**: Switch DDS without changing code
- **Performance Tuning**: Choose DDS based on requirements
- **Future-Proofing**: New middleware can be added

**Available implementations**:
- **FastDDS**: Default, good all-around performance
- **CycloneDDS**: Low latency, embedded-friendly
- **Connext DDS**: Commercial, enterprise features
- **zenoh**: Experimental, for edge/IoT

**Switching DDS**:
```bash
# Use CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Verify
ros2 doctor --report
```

### DDS Layer

Data Distribution Service (OMG standard) handles actual networking.

**Core Concepts**:
- **Global Data Space**: All participants share a virtual data space
- **Decentralized**: No broker or master
- **Automatic Discovery**: Publishers and subscribers find each other
- **Quality of Service**: Granular control over delivery

**DDS vs. Traditional Messaging**:

| Feature | Traditional (e.g., RabbitMQ) | DDS |
|---------|------------------------------|-----|
| Architecture | Broker-based | Peer-to-peer |
| Discovery | Manual configuration | Automatic |
| Latency | Higher (broker hop) | Lower (direct) |
| Real-time | Not guaranteed | Yes |
| Fault Tolerance | Broker SPOF | Fully distributed |

## Communication Patterns

### Publish-Subscribe (Topics)

Decoupled, asynchronous, one-to-many:

```python
# Publisher
self.pub = self.create_publisher(Image, '/camera/image', 10)
self.pub.publish(image_msg)

# Subscribers (multiple)
self.sub1 = self.create_subscription(Image, '/camera/image', callback1, 10)
self.sub2 = self.create_subscription(Image, '/camera/image', callback2, 10)
```

**Use cases**:
- Sensor data streams
- State broadcasts
- Telemetry

**Pros**: Scalable, loose coupling  
**Cons**: No delivery confirmation

### Request-Response (Services)

Synchronous, client-server, one-to-one:

```python
# Server
self.srv = self.create_service(AddTwoInts, '/add_two_ints', self.handle_request)

def handle_request(self, request, response):
    response.sum = request.a + request.b
    return response

# Client
self.client = self.create_client(AddTwoInts, '/add_two_ints')
future = self.client.call_async(request)
```

**Use cases**:
- Configuration queries
- One-off computations
- Mode changes

**Pros**: Guaranteed response, simple  
**Cons**: Blocking, no progress feedback

### Goal-Oriented (Actions)

Asynchronous, long-running, with feedback and cancellation:

```python
# Server
self._action_server = ActionServer(
    self, Fibonacci, 'fibonacci',
    execute_callback=self.execute_callback
)

def execute_callback(self, goal_handle):
    # Long-running task
    for i in range(goal_handle.request.order):
        # Send feedback
        feedback_msg.partial_sequence.append(...)
        goal_handle.publish_feedback(feedback_msg)
    
    # Return result
    return result

# Client
self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
goal_handle = await self._action_client.send_goal_async(goal_msg, 
                                                         feedback_callback=...)
result = await goal_handle.get_result_async()
```

**Use cases**:
- Navigation goals
- Manipulation tasks
- Multi-step operations

**Pros**: Cancellable, progress updates  
**Cons**: More complex implementation

## Executor Models

Executors control how callbacks are invoked.

### Single-Threaded Executor

**Default behavior**:
```python
rclpy.spin(node)  # Blocks until shutdown
```

**Characteristics**:
- **Serial execution**: One callback at a time
- **Simple**: No thread safety concerns
- **Blocking**: Long callbacks delay others

**When to use**: Simple robots with low callback frequency

### MultiThreadedExecutor

**Usage**:
```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
```

**Characteristics**:
- **Parallel**: Multiple callbacks simultaneously
- **Complex**: Requires thread-safe code
- **Higher throughput**: Better CPU utilization

**When to use**: High-frequency sensors, computationally intensive tasks

### Custom Executors

You can create custom executors for specific patterns:

```python
class PriorityExecutor(Executor):
    def spin_once(self, timeout_sec=None):
        # Custom scheduling logic
        high_priority_work = self.get_high_priority_work()
        if high_priority_work:
            high_priority_work.execute()
        else:
            low_priority_work = self.get_low_priority_work()
            if low_priority_work:
                low_priority_work.execute()
```

## Discovery Mechanism

ROS 2 uses automatic discovery—no master node like ROS 1.

### Discovery Process

1. **Participant Announcement**: Node joins DDS domain
2. **Endpoint Discovery**: Advertises publishers/subscribers
3. **Matching**: DDS matches publishers with subscribers
4. **Connection**: Direct communication established

**Under the hood**:
```
Node A starts → Sends multicast announcement
              ← Node B receives, responds with own info
              → Nodes exchange endpoint information
              → Topic types matched
              → Direct data channel established
```

### DDS Domains

Domains isolate ROS 2 systems:

```bash
# Terminal 1
export ROS_DOMAIN_ID=0
ros2 run demo_nodes_cpp talker

# Terminal 2  
export ROS_DOMAIN_ID=0  # Same domain - can see talker
ros2 run demo_nodes_cpp listener

# Terminal 3
export ROS_DOMAIN_ID=1  # Different domain - cannot see talker
ros2 node list  # Empty
```

**Use cases**:
- Multiple robot teams in one space
- Isolating development environments
- Network segmentation

**Domain range**: 0-101 (reserved), 102-232 (available)

## Quality of Service (QoS) Policies

QoS policies control communication characteristics.

### Key Policies

**Reliability**:
- `RELIABLE`: Guaranteed delivery (TCP-like)
- `BEST_EFFORT`: May drop messages (UDP-like)

**Durability**:
- `VOLATILE`: Only send to active subscribers
- `TRANSIENT_LOCAL`: Late-joiners get last message

**History**:
- `KEEP_LAST(depth)`: Buffer last N messages
- `KEEP_ALL`: Buffer all (until memory limit)

**Liveliness**:
- `AUTOMATIC`: DDS manages
- `MANUAL_BY_TOPIC`: Application signals

### QoS Profiles

Pre-configured combinations:

```python
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# For high-frequency sensors (best effort, volatile)
self.create_subscription(LaserScan, '/scan', callback, 
                        qos_profile_sensor_data)

# For critical commands (reliable, volatile)
self.create_subscription(Twist, '/cmd_vel', callback, 
                        qos_profile_system_default)
```

### QoS Compatibility

Publishers and subscribers must have **compatible** QoS:

| Publisher | Subscriber | Compatible? |
|-----------|-----------|-------------|
| RELIABLE | RELIABLE | ✅ Yes |
| RELIABLE | BEST_EFFORT | ✅ Yes |
| BEST_EFFORT | RELIABLE | ❌ No |
| BEST_EFFORT | BEST_EFFORT | ✅ Yes |

**Debugging QoS issues**:
```bash
ros2 topic info /my_topic --verbose
# Shows QoS of all publishers and subscribers
```

## Performance Considerations

### Intra-Process Communication

When publisher and subscriber are in the same process, ROS 2 can optimize:

```python
rclpy.init()
node = Node('my_node', 
            enable_intra_process_comms=True)  # Enable optimization
```

**Benefits**:
- Zero-copy message passing
- No serialization overhead
- Lower latency

**Limitations**:
- Only works within single process
- Both publisher and subscriber must enable it

### Shared Memory Transport

For large messages (images, point clouds), use shared memory:

```xml
<!-- FastDDS profile -->
<data_writer profile_name="large_data">
    <qos>
        <publishMode>
            <kind>ASYNCHRONOUS</kind>
        </publishMode>
    </qos>
    <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
</data_writer>
```

### Network Tuning

For multi-machine setups:

```bash
# Increase UDP buffer sizes (Linux)
sudo sysctl -w net.core.rmem_max=8388608
sudo sysctl -w net.core.wmem_max=8388608
```

## Debugging Architecture Issues

### Common Problems

**1. Nodes not discovering each other**
```bash
# Check domain ID
echo $ROS_DOMAIN_ID

# Check network interface
ros2 doctor --report

# Monitor discovery traffic
ros2 run rclcpp list_nodes
```

**2. QoS mismatch**
```bash
ros2 topic info /my_topic --verbose
# Look for "incompatible QoS" warnings
```

**3. Slow communication**
```bash
# Benchmark topic throughput
ros2 topic hz /my_topic
ros2 topic bw /my_topic

# Profile with tools
ros2 run rclcpp benchmark_publisher
```

## Summary

Key takeaways:

- **Layered Design**: Each layer abstracts complexity
- **rmw Interface**: Enables DDS vendor flexibility
- **Multiple Patterns**: Topics, services, actions for different use cases
- **QoS Control**: Fine-tune reliability, latency, and throughput
- **No Master**: Fully distributed via DDS discovery

Understanding this architecture helps you:
- Debug inter-node communication issues
- Optimize performance
- Make informed design choices
- Integrate with non-ROS systems

## Next Steps

- Read about [Nodes and Topics](/docs/module1/nodes-topics) in detail
- Learn [Python Integration](/docs/module1/python-rclpy) patterns
- Study [URDF Modeling](/docs/module1/urdf) for robot description

## Further Reading

- [ROS 2 Design Decisions](https://design.ros2.org/)
- [DDS-ROS Mapping](https://design.ros2.org/articles/mapping_dds_types.html)
- [QoS Best Practices](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

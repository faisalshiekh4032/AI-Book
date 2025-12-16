---
sidebar_position: 3
---

# Weekly Schedule

This 13-week course is structured into four main modules plus a capstone project. Each week builds on previous knowledge, progressing from fundamentals to advanced autonomous systems.

## Course Timeline Overview

```
Weeks 1-2:  Introduction & Foundations
Weeks 3-5:  Module 1 - ROS 2 Fundamentals
Weeks 6-7:  Module 2 - Digital Twin Simulation
Weeks 8-10: Module 3 - NVIDIA Isaac Platform
Weeks 11-12: Module 4 - Vision-Language-Action
Week 13:    Capstone Project & Presentations
```

---

## Weeks 1-2: Introduction & Foundations

### Week 1: Physical AI Concepts

**Learning Objectives**:
- Understand embodied intelligence vs. digital AI
- Recognize the unique challenges of physical systems
- Survey the landscape of humanoid robotics

**Topics**:
- What is Physical AI?
- Why humanoid robots matter
- Current state of the industry (Boston Dynamics, Figure AI, Tesla Bot)
- Course tools overview (ROS 2, Gazebo, Isaac, GPT)

**Activities**:
- Install Ubuntu 22.04 LTS (dual-boot or dedicated machine)
- Set up development environment
- Join course Discord community
- Complete pre-assessment quiz

**Deliverables**:
- Working Ubuntu installation
- Development tools configured (Python, Git, VS Code)

### Week 2: Sensors & Perception

**Learning Objectives**:
- Understand robotic sensor types and their applications
- Learn coordinate frames and transformations
- Grasp real-time data processing requirements

**Topics**:
- Sensor systems: LIDAR, cameras, IMUs, force/torque sensors
- Computer vision basics for robotics
- Sensor fusion fundamentals
- Introduction to coordinate transforms (tf2)

**Activities**:
- Visualize sensor data from sample datasets
- Implement basic camera calibration
- Explore RealSense camera demos

**Deliverables**:
- Lab report on sensor characteristics
- Calibration results

---

## Module 1: ROS 2 Fundamentals (Weeks 3-5)

### Week 3: ROS 2 Architecture

**Learning Objectives**:
- Understand the ROS 2 architecture and DDS middleware
- Create and run basic ROS 2 nodes
- Use command-line tools for debugging

**Topics**:
- ROS 2 vs. ROS 1 improvements
- Nodes, topics, and the pub-sub pattern
- Quality of Service (QoS) policies
- ROS 2 CLI tools (ros2 node, topic, service)

**Activities**:
- Install ROS 2 Humble
- Create your first publisher/subscriber pair
- Visualize topic data with rqt
- Monitor node graph with rqt_graph

**Deliverables**:
- **Lab 1**: Temperature sensor publisher and monitor subscriber

### Week 4: Services, Actions & Messages

**Learning Objectives**:
- Implement request-response patterns with services
- Use actions for long-running tasks
- Define custom message types

**Topics**:
- ROS 2 services for synchronous communication
- Actions for cancellable, feedback-driven tasks
- Custom .msg and .srv definitions
- Package creation and building (colcon)

**Activities**:
- Build a service-based calculator node
- Implement an action server for robot movement
- Create custom message types

**Deliverables**:
- **Lab 2**: Motion control action server with progress feedback

### Week 5: URDF & Launch Files

**Learning Objectives**:
- Model robots using URDF
- Write launch files for multi-node systems
- Use parameters for configuration

**Topics**:
- Unified Robot Description Format (URDF)
- Xacro for modular robot descriptions
- Launch files in Python
- Parameter management and dynamic reconfigure

**Activities**:
- Create URDF model for a simple robot
- Write launch file to start multiple nodes
- Visualize robot in RViz2

**Deliverables**:
- **Module 1 Project**: Multi-node robot control system with custom URDF

---

## Module 2: Digital Twin & Simulation (Weeks 6-7)

### Week 6: Gazebo Simulation

**Learning Objectives**:
- Set up simulation environments in Gazebo
- Simulate physics and sensors
- Bridge Gazebo with ROS 2

**Topics**:
- Gazebo Classic vs. Gazebo Sim (Ignition)
- SDF (Simulation Description Format)
- Physics engines and collision detection
- Sensor plugins (camera, LIDAR, IMU)

**Activities**:
- Build a custom environment in Gazebo
- Spawn robot from URDF
- Implement sensor-based navigation
- Test robot behaviors in simulation

**Deliverables**:
- **Lab 3**: Simulated robot with camera and LIDAR navigation

### Week 7: Unity & Advanced Simulation

**Learning Objectives**:
- Integrate Unity for high-fidelity visualization
- Understand sim-to-real transfer challenges
- Generate synthetic training data

**Topics**:
- Unity Robotics Hub
- ROS-Unity communication
- Photorealistic rendering for perception training
- Domain randomization techniques

**Activities**:
- Set up Unity with ROS 2
- Create a warehouse environment
- Collect synthetic image datasets

**Deliverables**:
- **Module 2 Project**: Simulated robot navigating complex environment

---

## Module 3: NVIDIA Isaac Platform (Weeks 8-10)

### Week 8: Isaac Sim Introduction

**Learning Objectives**:
- Set up NVIDIA Isaac Sim
- Create photorealistic simulations
- Generate synthetic data for AI training

**Topics**:
- Omniverse platform overview
- Isaac Sim interface and USD format
- Physics simulation with PhysX
- Synthetic data generation (SDG)

**Activities**:
- Install Isaac Sim
- Import robot models
- Set up camera sensors for SDG
- Generate training datasets

**Deliverables**:
- **Lab 4**: Synthetic dataset for object detection

### Week 9: Isaac ROS & Perception

**Learning Objectives**:
- Deploy GPU-accelerated perception pipelines
- Implement Visual SLAM
- Use hardware-accelerated ROS 2 nodes

**Topics**:
- Isaac ROS GEMs (reusable modules)
- Visual SLAM with Isaac ROS
- DNN inference on Jetson
- Performance optimization

**Activities**:
- Install Isaac ROS on workstation
- Run Visual SLAM demo
- Deploy perception pipeline on Jetson
- Benchmark performance

**Deliverables**:
- **Lab 5**: Real-time object detection with Isaac ROS

### Week 10: Navigation & Planning

**Learning Objectives**:
- Use Nav2 for autonomous navigation
- Plan paths in complex environments
- Handle dynamic obstacles

**Topics**:
- Nav2 architecture and plugins
- Costmap configuration
- Path planning algorithms (A*, DWB)
- Behavior trees for decision logic

**Activities**:
- Configure Nav2 for your robot
- Test navigation in Isaac Sim
- Implement dynamic obstacle avoidance
- Tune navigation parameters

**Deliverables**:
- **Module 3 Project**: Autonomous navigation in simulated warehouse

---

## Module 4: Vision-Language-Action (Weeks 11-12)

### Week 11: Voice Control Integration

**Learning Objectives**:
- Integrate speech recognition with ROS 2
- Implement natural language command processing
- Build voice-controlled robot behaviors

**Topics**:
- OpenAI Whisper for speech-to-text
- Natural language understanding basics
- Intent recognition and entity extraction
- Voice command safety considerations

**Activities**:
- Set up Whisper with ROS 2
- Create voice command parser
- Map commands to robot actions
- Test voice-controlled navigation

**Deliverables**:
- **Lab 6**: Voice-controlled robot ("Go to kitchen", "Pick up box")

### Week 12: LLM-Based Planning

**Learning Objectives**:
- Use large language models for task planning
- Implement cognitive planning architecture
- Build multimodal robot systems

**Topics**:
- GPT-4 for robot task planning
- Prompt engineering for robotics
- Multimodal AI (vision + language)
- Error recovery and replanning

**Activities**:
- Integrate GPT-4 with ROS 2
- Implement task decomposition
- Add vision-based verification
- Build closed-loop planning system

**Deliverables**:
- **Module 4 Project**: Multimodal autonomous assistant

---

## Week 13: Capstone Project

### Autonomous Humanoid Assistant

**Project Requirements**:
- Accept natural language voice commands
- Plan multi-step tasks using LLMs
- Navigate indoor environment autonomously
- Detect and manipulate objects
- Operate in Isaac Sim (real hardware optional)

**Weekly Milestones**:
- **Days 1-2**: Project planning and architecture
- **Days 3-5**: Core integration and testing
- **Days 6-7**: Polish, documentation, and video
- **Final Day**: Project presentations

**Deliverables**:
- Source code repository (GitHub)
- Demo video (90 seconds)
- Technical documentation
- Live presentation (invited participants)

**Presentation Format**:
- 5-minute demo
- 5-minute technical Q&A
- 2-minute feedback

---

## Time Management Tips

### Weekly Study Plan

**Weekdays** (2 hours/day):
- Monday-Wednesday: Lectures and reading
- Thursday-Friday: Labs and exercises

**Weekends** (5-6 hours):
- Saturday: Project work
- Sunday: Review and catch-up

### Staying on Track

- ✅ Set calendar reminders for deadlines
- ✅ Join weekly study groups on Discord
- ✅ Ask questions early and often
- ✅ Start projects when assigned, not before deadline
- ✅ Maintain a learning journal

### If You Fall Behind

1. Prioritize current week's lab
2. Skip optional readings temporarily
3. Ask for help in Discord
4. Attend office hours
5. Consider extension (communicate early)

---

*This schedule is designed to balance theoretical learning with hands-on practice. Adjust your pace as needed, but maintain consistent progress toward the capstone.*

---
sidebar_position: 5
---

# URDF: Robot Description Format

Learn to model robots using URDF (Unified Robot Description Format), the standard for describing robot kinematics and geometry in ROS 2.

## What is URDF?

URDF is an XML format that describes:
- Robot structure (links and joints)
- Visual appearance
- Collision geometry
- Physical properties (mass, inertia)
- Sensor placement

## Basic URDF Structure

### Simple Robot Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Joint connecting wheel to base -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0.2 0.225 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

</robot>
```

## URDF Components

### Links

Links represent rigid bodies:

```xml
<link name="arm_link">
  <!-- Visual representation -->
  <visual>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="1.0"/>
    </geometry>
    <material name="silver">
      <color rgba="0.75 0.75 0.75 1"/>
    </material>
  </visual>
  
  <!-- Collision geometry (simpler than visual) -->
  <collision>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.06" length="1.0"/>
    </geometry>
  </collision>
  
  <!-- Physical properties -->
  <inertial>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <mass value="2.5"/>
    <inertia ixx="0.1" ixy="0" ixz="0" 
             iyy="0.1" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

### Joints

Joints connect links and define motion:

**Joint Types**:
- `fixed`: No motion
- `revolute`: Rotation with limits
- `continuous`: Unlimited rotation
- `prismatic`: Linear sliding
- `floating`: 6 DOF
- `planar`: 2D motion

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
</joint>
```

## Humanoid Robot URDF

### Torso and Head

```xml
<robot name="humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" 
               iyy="0.6" iyz="0" izz="0.3"/>
    </inertial>
  </link>
  
  <!-- Neck -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" velocity="2.0" lower="-0.785" upper="0.785"/>
  </joint>
  
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Xacro: Modular URDF

Xacro extends URDF with macros and variables for reusability.

### Basic Xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
  
  <!-- Properties (constants) -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_length" value="0.05"/>
  
  <!-- Macros (reusable components) -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
        </geometry>
      </visual>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0.2 ${reflect*0.225} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>
  
  <!-- Use macros -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

</robot>
```

### Converting Xacro to URDF

```bash
ros2 run xacro xacro my_robot.urdf.xacro > my_robot.urdf
```

## Visualizing in RViz2

### Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf', 'robot.urdf')
    
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
```

## Adding Sensors

### Camera

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>camera</namespace>
        <remapping>image_raw:=image_raw</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>10.0</max>
      </range>
    </ray>
    <plugin name="scan" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Best Practices

1. **Use Xacro**: Avoid repetition, use macros for repeated components
2. **Separate Collision**: Use simpler geometry for collision than visual
3. **Accurate Inertia**: Calculate or measure real inertial properties
4. **Consistent Naming**: Use clear, hierarchical link/joint names
5. **Validate**: Use `check_urdf` to verify your URDF

```bash
check_urdf my_robot.urdf
```

## Common Issues

### Problem: Robot Explodes in Simulation

**Cause**: Missing or incorrect inertial properties  
**Solution**: Add proper mass and inertia to all links

### Problem: Robot Sinks Through Ground

**Cause**: Missing collision geometry  
**Solution**: Add collision tags to all links

### Problem: Joints Don't Move

**Cause**: Joint limits too restrictive or wrong axis  
**Solution**: Check limit values and axis direction

## Next Steps

- Test your URDF in [Gazebo Simulation](/docs/module2/gazebo)
- Learn about [Digital Twins](/docs/module2/overview)
- Complete Lab 3: Create a humanoid robot URDF

---

*URDF is the foundation for simulation and visualization in ROS 2. Master it to build accurate robot models.*

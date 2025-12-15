# Chapter 3: Modeling Humanoid Robots with URDF

## Introduction to URDF

**URDF (Unified Robot Description Format)** is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its kinematic structure, dynamics, visual appearance, and collision properties. URDF is essential for robot simulation, visualization, and control in ROS-based systems.

URDF enables:
- Robot model definition for simulation environments
- Visualization in tools like RViz
- Collision detection and physics simulation
- Kinematic chain definition for motion planning
- Integration with robot state publishers

## URDF Structure and Components

### Basic URDF Elements

A URDF file consists of several key elements:

1. **Robot**: The root element that contains the entire robot description
2. **Links**: Rigid bodies that represent physical parts of the robot
3. **Joints**: Connections between links that define how they can move relative to each other
4. **Materials**: Visual properties like color and texture
5. **Transmissions**: Define how actuators connect to joints

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Define the base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

## Creating a Simple Humanoid Robot Model

Let's build a basic humanoid robot model step by step:

### 1. Defining the Torso

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="white">
      <color rgba="1.0 1.0 1.0 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

### 2. Adding a Head

```xml
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <material name="skin">
      <color rgba="0.8 0.6 0.4 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
  </inertial>
</link>

<!-- Joint connecting head to torso -->
<joint name="neck_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.35" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
</joint>
```

### 3. Adding Arms

```xml
<!-- Left Arm -->
<link name="left_upper_arm">
  <visual>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <material name="arm_color">
      <color rgba="0.7 0.7 0.7 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
  </inertial>
</link>

<link name="left_lower_arm">
  <visual>
    <geometry>
      <cylinder length="0.25" radius="0.04"/>
    </geometry>
    <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    <material name="arm_color"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.25" radius="0.04"/>
    </geometry>
    <origin xyz="0 0 -0.125" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="0.8"/>
    <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
  </inertial>
</link>

<!-- Joints for left arm -->
<joint name="left_shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>

<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

### 4. Adding Legs

```xml
<!-- Left Leg -->
<link name="left_upper_leg">
  <visual>
    <geometry>
      <cylinder length="0.4" radius="0.06"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <material name="leg_color">
      <color rgba="0.6 0.6 0.6 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.4" radius="0.06"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002"/>
  </inertial>
</link>

<link name="left_lower_leg">
  <visual>
    <geometry>
      <cylinder length="0.35" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.175" rpy="0 0 0"/>
    <material name="leg_color"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.35" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.175" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
  </inertial>
</link>

<link name="left_foot">
  <visual>
    <geometry>
      <box size="0.15 0.08 0.05"/>
    </geometry>
    <material name="foot_color">
      <color rgba="0.4 0.4 0.4 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.15 0.08 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
  </inertial>
</link>

<!-- Joints for left leg -->
<joint name="left_hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_leg"/>
  <origin xyz="-0.05 0 -0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
</joint>

<joint name="left_knee_joint" type="revolute">
  <parent link="left_upper_leg"/>
  <child link="left_lower_leg"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="0.0" effort="20.0" velocity="1.0"/>
</joint>

<joint name="left_ankle_joint" type="revolute">
  <parent link="left_lower_leg"/>
  <child link="left_foot"/>
  <origin xyz="0 0 -0.35" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
</joint>
```

## Complete Humanoid Robot URDF

Here's a complete example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <material name="skin">
    <color rgba="0.8 0.6 0.4 1.0"/>
  </material>
  <material name="arm_color">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>
  <material name="leg_color">
    <color rgba="0.6 0.6 0.6 1.0"/>
  </material>
  <material name="foot_color">
    <color rgba="0.4 0.4 0.4 1.0"/>
  </material>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="arm_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="arm_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Right Arm (symmetric) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="arm_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="arm_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="leg_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <material name="leg_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="foot_color"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.05 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.0" effort="20.0" velocity="1.0"/>
  </joint>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Right Leg (symmetric) -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="leg_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <material name="leg_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="foot_color"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.05 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.0" effort="20.0" velocity="1.0"/>
  </joint>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Fixed joint to define the base of the robot -->
  <link name="base_footprint">
    <visual>
      <geometry>
        <cylinder radius="0.01" length="0.01"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.01" length="0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0001"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="torso"/>
    <origin xyz="0 0 0.45" rpy="0 0 0"/>
  </joint>
</robot>
```

## Exercises

### Exercise 1: Create a Simple Robot Model
1. Create a URDF file for a simple wheeled robot with a base, two wheels, and a caster
2. Validate the URDF file
3. Visualize the robot in RViz

Example wheeled robot URDF:
```xml
<?xml version="1.0"?>
<robot name="wheeled_robot">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.02" radius="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.02" radius="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.1 0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.02" radius="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.02" radius="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.1 -0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Caster wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.15 0 -0.06" rpy="0 0 0"/>
  </joint>
</robot>
```

### Exercise 2: Extend the Humanoid Model
1. Add sensors to the humanoid model (e.g., a camera on the head)
2. Add additional joints to make the model more articulated
3. Validate and visualize the extended model

### Exercise 3: Integration with ROS 2
1. Create a launch file that loads your URDF model
2. Set up the robot state publisher to visualize joint movements
3. Test the integration with a simple joint controller
```

## Visualization and Validation

### Validating URDF Files

Before using your URDF model, validate it for correctness:

```bash
# Install the check_urdf tool if not already installed
sudo apt-get install ros-humble-urdfdom-py

# Validate your URDF file
check_urdf /path/to/your/robot.urdf
```

### Visualizing in RViz

To visualize your URDF model in RViz:

1. Launch RViz
2. Add a RobotModel display
3. Set the Robot Description parameter to the name of your robot description parameter (usually "robot_description")

### Launching with Robot State Publisher

Create a launch file to visualize your robot:

```python
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('your_package_name'),
        'urdf',
        'simple_humanoid.urdf'
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    return LaunchDescription([
        robot_state_publisher
    ])
```

## Advanced URDF Features

### Using Xacro for Complex Models

Xacro (XML Macros) allows you to create more maintainable URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_length" value="0.5" />
  <xacro:property name="torso_radius" value="0.15" />

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side parent xyz rpy">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <material name="arm_color"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:arm side="left" parent="torso" xyz="0.15 0 0.1" rpy="0 0 0"/>
  <xacro:arm side="right" parent="torso" xyz="-0.15 0 0.1" rpy="0 0 0"/>

</robot>
```

## Integration with ROS 2

### Robot State Publisher

The Robot State Publisher node is crucial for integrating your URDF model with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing transforms
        self.timer = self.create_timer(0.05, self.publish_transforms)

        self.joint_state = JointState()

    def joint_state_callback(self, msg):
        """Update joint state with latest values"""
        self.joint_state = msg

    def publish_transforms(self):
        """Publish transforms for all joints"""
        # Create transforms for each joint based on current joint states
        # This is where you'd create TransformStamped messages for each joint
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

### Exercise 1: Create a Simple Robot Model
1. Create a URDF file for a simple wheeled robot with a base, two wheels, and a caster
2. Validate the URDF file
3. Visualize the robot in RViz

### Exercise 2: Extend the Humanoid Model
1. Add sensors to the humanoid model (e.g., a camera on the head)
2. Add additional joints to make the model more articulated
3. Validate and visualize the extended model

### Exercise 3: Integration with ROS 2
1. Create a launch file that loads your URDF model
2. Set up the robot state publisher to visualize joint movements
3. Test the integration with a simple joint controller

## References and Citations

1. [URDF/XML Format Documentation](http://wiki.ros.org/urdf/XML) - Official ROS URDF XML specification
2. [ROS 2 URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) - ROS 2 specific URDF tutorials
3. [Xacro Documentation](http://wiki.ros.org/xacro) - XML Macros for URDF
4. [Robot State Publisher](https://github.com/ros2/robot_state_publisher) - Package for publishing robot state transforms
5. [Gazebo Robot Simulation](https://gazebosim.org/tutorials?tut=ros2_overview) - Integration with simulation environments

## Summary

In this chapter, you've learned how to model humanoid robots using URDF. You've seen the basic structure of URDF files, created a complete humanoid model, learned about validation and visualization, and explored advanced features like Xacro. You've also learned how to integrate URDF models with ROS 2 systems using the robot state publisher. These skills enable you to create realistic robot models for simulation, visualization, and control in your robotic applications.
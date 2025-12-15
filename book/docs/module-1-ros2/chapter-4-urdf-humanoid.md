# Chapter 4: Understanding URDF for Humanoid Robots

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial, visual, and collision characteristics.

For humanoid robots, URDF is particularly important as it captures the complex kinematic structure of the robot, enabling accurate simulation, visualization, and control.

## Basic Structure of URDF

A URDF file contains the following main elements:

1. **Robot element**: The root element that defines the robot
2. **Link elements**: Represent rigid parts of the robot
3. **Joint elements**: Define how links are connected
4. **Visual elements**: Define how links appear in visualization
5. **Collision elements**: Define collision boundaries for simulation
6. **Inertial elements**: Define mass and inertial properties

Here's a simple example:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
  </link>

  <!-- A leg link connected to the base -->
  <link name="leg_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.3" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.3" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5" />
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.004" />
    </inertial>
  </link>

  <!-- Joint connecting base to leg -->
  <joint name="base_to_leg" type="fixed">
    <parent link="base_link" />
    <child link="leg_link" />
    <origin xyz="0 0 -0.15" rpy="0 0 0" />
  </joint>
</robot>
```

## Complete Humanoid Robot URDF Model File

Here's a more comprehensive URDF model for a humanoid robot with a head, torso, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">

  <!-- Materials definition -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 1.0 0.0 1.0"/>
  </material>
  <material name="yellow">
    <color rgba="1.0 1.0 0.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link: Pelvis -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.2 0.2" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.2 0.2" />
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="8.0" />
      <origin xyz="0 0 0.25" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0" />
      <geometry>
        <box size="0.25 0.15 0.5" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0" />
      <geometry>
        <box size="0.25 0.15 0.5" />
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting pelvis to torso -->
  <joint name="pelvis_torso_joint" type="fixed">
    <parent link="base_link" />
    <child link="torso" />
    <origin xyz="0 0 0.2" rpy="0 0 0" />
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0.08" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.08" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.16" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.08" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.16" />
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.5" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Left arm - shoulder -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.05" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="left_shoulder" />
    <origin xyz="0.1 0.15 0.25" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Left arm - upper arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.04" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.04" />
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder" />
    <child link="left_upper_arm" />
    <origin xyz="0 0 -0.05" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-2.35" upper="0.0" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Left arm - lower arm -->
  <link name="left_lower_arm">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.12" />
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.12" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.24" radius="0.035" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.24" radius="0.035" />
      </geometry>
    </collision>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_upper_arm" />
    <child link="left_lower_arm" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
  </joint>

  <!-- Left hand -->
  <link name="left_hand">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 -0.05" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0" />
      <geometry>
        <box size="0.1 0.08 0.1" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0" />
      <geometry>
        <box size="0.1 0.08 0.1" />
      </geometry>
    </collision>
  </link>

  <joint name="left_hand_joint" type="fixed">
    <parent link="left_lower_arm" />
    <child link="left_hand" />
    <origin xyz="0 0 -0.12" rpy="0 0 0" />
  </joint>

  <!-- Right arm (symmetric to left) -->
  <link name="right_shoulder">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.05" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="right_shoulder" />
    <origin xyz="0.1 -0.15 0.25" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.04" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.04" />
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder" />
    <child link="right_upper_arm" />
    <origin xyz="0 0 -0.05" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-2.35" upper="0.0" effort="10.0" velocity="1.0" />
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.12" />
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.12" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.24" radius="0.035" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.24" radius="0.035" />
      </geometry>
    </collision>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_upper_arm" />
    <child link="right_lower_arm" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
  </joint>

  <link name="right_hand">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 -0.05" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0" />
      <geometry>
        <box size="0.1 0.08 0.1" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0" />
      <geometry>
        <box size="0.1 0.08 0.1" />
      </geometry>
    </collision>
  </link>

  <joint name="right_hand_joint" type="fixed">
    <parent link="right_lower_arm" />
    <child link="right_hand" />
    <origin xyz="0 0 -0.12" rpy="0 0 0" />
  </joint>

  <!-- Left leg - hip -->
  <link name="left_hip">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.05" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="left_hip" />
    <origin xyz="-0.05 0.08 -0.1" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="20.0" velocity="1.0" />
  </joint>

  <!-- Left leg - upper leg -->
  <link name="left_upper_leg">
    <inertial>
      <mass value="3.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip" />
    <child link="left_upper_leg" />
    <origin xyz="0 0 -0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="0.0" upper="2.3" effort="20.0" velocity="1.0" />
  </joint>

  <!-- Left leg - lower leg -->
  <link name="left_lower_leg">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.045" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.045" />
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_upper_leg" />
    <child link="left_lower_leg" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Left foot -->
  <link name="left_foot">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0.05 0 -0.02" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.1 0.04" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.1 0.04" />
      </geometry>
    </collision>
  </link>

  <joint name="left_foot_joint" type="fixed">
    <parent link="left_lower_leg" />
    <child link="left_foot" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
  </joint>

  <!-- Right leg (symmetric to left) -->
  <link name="right_hip">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.05" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="right_hip" />
    <origin xyz="-0.05 -0.08 -0.1" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="20.0" velocity="1.0" />
  </joint>

  <link name="right_upper_leg">
    <inertial>
      <mass value="3.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip" />
    <child link="right_upper_leg" />
    <origin xyz="0 0 -0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="0.0" upper="2.3" effort="20.0" velocity="1.0" />
  </joint>

  <link name="right_lower_leg">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.045" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.045" />
      </geometry>
    </collision>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_upper_leg" />
    <child link="right_lower_leg" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <link name="right_foot">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0.05 0 -0.02" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.1 0.04" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.1 0.04" />
      </geometry>
    </collision>
  </link>

  <joint name="right_foot_joint" type="fixed">
    <parent link="right_lower_leg" />
    <child link="right_foot" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
  </joint>

  <!-- Sensors and other elements can be added here -->
</robot>
```

## Visual and Collision Elements in URDF

Visual elements define how the robot appears in visualization tools like RViz2, while collision elements define boundaries for physics simulation:

```xml
<!-- Visual element -->
<visual>
  <origin xyz="0 0 0" rpy="0 0 0" />
  <geometry>
    <box size="0.1 0.1 0.1" />
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1" />
  </material>
</visual>

<!-- Collision element -->
<collision>
  <origin xyz="0 0 0" rpy="0 0 0" />
  <geometry>
    <box size="0.1 0.1 0.1" />
  </geometry>
</collision>
```

## Joint Definitions for Humanoid Robot

Joints in a humanoid robot define the connection between links and specify degrees of freedom:

1. **Fixed joints**: No movement (like connecting a sensor to a link)
2. **Revolute joints**: Rotational movement around a single axis (like elbows, knees)
3. **Continuous joints**: Like revolute but unlimited rotation (like a wheel)
4. **Prismatic joints**: Linear movement along an axis
5. **Floating joints**: 6 degrees of freedom (rarely used in humanoid robots)
6. **Planar joints**: Movement on a plane (rarely used in humanoid robots)

For our humanoid robot, we primarily use revolute joints with specified limits to match human-like range of motion:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm" />
  <child link="lower_arm" />
  <origin xyz="0 0 -0.3" rpy="0 0 0" />
  <axis xyz="0 1 0" /> <!-- Rotation around Y-axis -->
  <limit lower="-2.35" upper="0.0" effort="10.0" velocity="1.0" />
  <dynamics damping="0.1" friction="0.0" />
</joint>
```

## Material Definitions and Colors

Material definitions in URDF allow for consistent coloring and appearance across visualization tools:

```xml
<material name="blue">
  <color rgba="0.0 0.0 1.0 1.0"/>
</material>
<material name="red">
  <color rgba="1.0 0.0 0.0 1.0"/>
</material>
<material name="light_grey">
  <color rgba="0.8 0.8 0.8 1.0"/>
</material>
```

## URDF Validation and Visualization Techniques

### Validating URDF Files

To validate a URDF file, you can:

1. Use the `check_urdf` command:
   ```bash
   check_urdf /path/to/robot.urdf
   ```

2. Use the `urdfdom` parser to check for XML syntax errors

3. Load the URDF in RViz2 to visually inspect the robot model

### Visualizing URDF in RViz2

To visualize your URDF in RViz2:

1. Create a launch file to publish the robot state:
   ```xml
   <launch>
     <node pkg="robot_state_publisher" executable="robot_state_publisher" name="robot_state_publisher">
       <param name="robot_description" value="$(find my_robot_description)/urdf/robot.urdf" />
     </node>
   </launch>
   ```

2. Add RobotModel display in RViz2 and set the robot description topic

### Testing URDF with Gazebo

To test your URDF with Gazebo:

1. Add Gazebo-specific extensions to your URDF:
   ```xml
   <gazebo reference="link_name">
     <material>Gazebo/Blue</material>
   </gazebo>
   ```

2. Include physics properties in your links and joints for realistic simulation

## Testing URDF Model in RViz and Gazebo Simulation

To test your URDF model:

1. Create a simple launch file:

   ```python
   #!/usr/bin/env python3
   from launch import LaunchDescription
   from launch.substitutions import Command
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory
   import os


   def generate_launch_description():
       pkg_share = get_package_share_directory('my_robot_description')
       urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid_robot.urdf')
       
       return LaunchDescription([
           Node(
               package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               output='screen',
               parameters=[{
                   'robot_description': Command(['xacro ', urdf_file]).to_string()
               }]
           )
       ])
   ```

2. Run the launch file:
   ```bash
   ros2 launch my_robot_description display.launch.py
   ```

3. Visualize in RViz2 by adding the RobotModel display and setting the topic to `/robot_description`

## URDF Best Practices and Common Patterns

### File Organization
- Keep URDF files modular using xacro macros for complex robots
- Use separate .xacro files for different parts of the robot
- Include common materials and constants in separate files

### Kinematic Chains
- Design consistent joint naming conventions
- Consider the kinematic structure for inverse kinematics solvers
- Plan for future sensor integration points

### Mass and Inertia Properties
- Calculate mass and inertia properties accurately
- Use CAD tools to calculate these properties if available
- For simple shapes, use standard formulas

```xml
<!-- Example of a properly calculated inertia -->
<inertial>
  <mass value="1.0" />
  <origin xyz="0 0 0" />
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02" />
</inertial>
```

## Learning Objectives

After completing this chapter, you should be able to:
- Understand the structure and components of URDF files
- Create complete humanoid robot URDF models with proper kinematic chains
- Define visual, collision, and inertial elements for robots
- Implement proper joint definitions for humanoid robot kinematics
- Validate URDF models and visualize them in RViz
- Apply URDF best practices for humanoid robot design

## Chapter Summary

URDF is essential for representing humanoid robots in ROS 2. This chapter covered the complete structure of URDF files, demonstrated how to create a full humanoid robot model, explained the different elements needed for visualization and simulation, and provided best practices for humanoid robot URDF design. Understanding URDF is crucial for any roboticist working with humanoid robots, as it forms the basis for simulation, visualization, and control.
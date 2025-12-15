# Chapter 6: Complete Digital Twin Implementation Example

## Bringing It All Together

This chapter demonstrates a complete Digital Twin implementation for a humanoid robot, integrating all concepts covered in Module 2. We'll build a comprehensive simulation environment that combines physics simulation, high-fidelity rendering, and sensor simulation to create a realistic digital representation of a physical humanoid robot.

## Complete Implementation Architecture

### System Overview

Our complete Digital Twin system will consist of:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          HUMANOID ROBOT DIGITAL TWIN                           │
├─────────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐            │
│  │   PHYSICS       │    │   RENDERING     │    │   SENSORS       │            │
│  │   SIMULATION    │    │   ENGINE        │    │   SIMULATION    │            │
│  │   (Gazebo)      │    │   (Unity)       │    │   (Various)     │            │
│  │                 │    │                 │    │                 │            │
│  │ • Accurate      │    │ • Photorealistic│    │ • LiDAR         │            │
│  │   physics       │    │   rendering     │    │ • Depth Camera  │            │
│  │ • Collision     │    │ • Materials     │    │ • IMU           │            │
│  │   detection     │    │ • Lighting      │    │ • Others        │            │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘            │
│                                    │                                           │
│                                    ▼                                           │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                    ROS 2 COMMUNICATION LAYER                          │ │
│  │  • Topics for sensor data, commands, status                            │ │
│  │  • Services for configuration and control                              │ │
│  │  • Actions for long-running tasks                                      │ │
│  │  • Parameters for system configuration                                 │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                    │                                           │
│                                    ▼                                           │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                   AI/CONTROL ALGORITHMS                                 │ │
│  │  • Perception pipelines                                                │ │
│  │  • Navigation and path planning                                        │ │
│  │  • Balance control                                                     │ │
│  │  • Manipulation planning                                               │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Creating the Complete Robot Model

Let's start with a comprehensive URDF for our humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="complete_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include common materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="silver">
    <color rgba="0.75 0.75 0.75 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.42 0.0 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.87 0.84 0.70 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base/Pelvis Link -->
  <link name="base_link">
    <inertial>
      <mass value="15.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.15" ixy="0.0" ixz="0.0" iyy="0.25" iyz="0.0" izz="0.18" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.25 0.2" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.25 0.2" />
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="12.0" />
      <origin xyz="0 0 0.25" />
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0" />
      <geometry>
        <box size="0.25 0.2 0.5" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0" />
      <geometry>
        <box size="0.25 0.2 0.5" />
      </geometry>
    </collision>
  </link>

  <joint name="pelvis_torso_joint" type="fixed">
    <parent link="base_link" />
    <child link="torso" />
    <origin xyz="0.0 0.0 0.2" rpy="0 0 0" />
  </joint>

  <!-- Head with sensors -->
  <link name="head">
    <inertial>
      <mass value="2.5" />
      <origin xyz="0 0 0.08" />
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02" />
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

  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0.0 0 0.5" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
    <dynamics damping="0.5" friction="0.1" />
  </joint>

  <!-- Depth camera in head -->
  <link name="camera_link">
    <inertial>
      <mass value="0.1" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <box size="0.05 0.05 0.05" />
      </geometry>
      <material name="black" />
    </visual>
    <collision>
      <origin xyz="0 0 0" />
      <geometry>
        <box size="0.05 0.05 0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head" />
    <child link="camera_link" />
    <origin xyz="0.1 0 0.08" rpy="0 0 0" /> <!-- Offset from head center -->
  </joint>

  <!-- IMU sensor in head -->
  <link name="imu_link" />

  <joint name="imu_joint" type="fixed">
    <parent link="head" />
    <child link="imu_link" />
    <origin xyz="0 0 0.15" /> <!-- On top of head -->
  </joint>

  <!-- LiDAR mount on head -->
  <link name="lidar_mount">
    <inertial>
      <mass value="0.2" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.05" />
      </geometry>
      <material name="black" />
    </visual>
    <collision>
      <origin xyz="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="lidar_mount_joint" type="fixed">
    <parent link="head" />
    <child link="lidar_mount" />
    <origin xyz="0.15 0 0.08" rpy="0 0 0" />
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_yaw" type="revolute">
    <parent link="torso" />
    <child link="left_shoulder" />
    <origin xyz="0.13 0.12 0.25" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="15.0" velocity="1.0" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.012" ixy="0.0" ixz="0.0" iyy="0.012" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="left_shoulder" />
    <child link="left_upper_arm" />
    <origin xyz="0.0 0.0 -0.05" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="15.0" velocity="1.0" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.12" />
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.12" />
      <geometry>
        <cylinder length="0.24" radius="0.045" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" />
      <geometry>
        <cylinder length="0.24" radius="0.045" />
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm" />
    <child link="left_lower_arm" />
    <origin xyz="0.0 0.0 -0.3" />
    <axis xyz="0 1 0" />
    <limit lower="-2.35" upper="0.0" effort="10.0" velocity="2.0" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <link name="left_hand">
    <inertial>
      <mass value="0.8" />
      <origin xyz="0.05 0 -0.02" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.12 0.08 0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.12 0.08 0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm" />
    <child link="left_hand" />
    <origin xyz="0.0 0.0 -0.24" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
    <dynamics damping="0.5" friction="0.05" />
  </joint>

  <!-- Right Arm (mirror of left) -->
  <link name="right_shoulder">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_yaw" type="revolute">
    <parent link="torso" />
    <child link="right_shoulder" />
    <origin xyz="0.13 -0.12 0.25" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="15.0" velocity="1.0" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.012" ixy="0.0" ixz="0.0" iyy="0.012" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="right_shoulder" />
    <child link="right_upper_arm" />
    <origin xyz="0.0 0.0 -0.05" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="15.0" velocity="1.0" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.12" />
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.12" />
      <geometry>
        <cylinder length="0.24" radius="0.045" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" />
      <geometry>
        <cylinder length="0.24" radius="0.045" />
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm" />
    <child link="right_lower_arm" />
    <origin xyz="0.0 0.0 -0.3" />
    <axis xyz="0 1 0" />
    <limit lower="-2.35" upper="0.0" effort="10.0" velocity="2.0" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <link name="right_hand">
    <inertial>
      <mass value="0.8" />
      <origin xyz="0.05 0 -0.02" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.12 0.08 0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.12 0.08 0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_lower_arm" />
    <child link="right_hand" />
    <origin xyz="0.0 0.0 -0.24" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
    <dynamics damping="0.5" friction="0.05" />
  </joint>

  <!-- Left Leg -->
  <link name="left_hip">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.05" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_yaw" type="revolute">
    <parent link="base_link" />
    <child link="left_hip" />
    <origin xyz="-0.08 0.08 -0.1" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="30.0" velocity="1.0" />
    <dynamics damping="2.0" friction="0.2" />
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_pitch" type="revolute">
    <parent link="left_hip" />
    <child link="left_thigh" />
    <origin xyz="0.0 0.0 -0.05" />
    <axis xyz="0 1 0" />
    <limit lower="-0.3" upper="2.2" effort="30.0" velocity="1.0" />
    <dynamics damping="2.0" friction="0.2" />
  </joint>

  <link name="left_shin">
    <inertial>
      <mass value="4.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.06" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.055" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.055" />
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh" />
    <child link="left_shin" />
    <origin xyz="0.0 0.0 -0.4" />
    <axis xyz="0 1 0" />
    <limit lower="0.0" upper="2.3" effort="30.0" velocity="1.0" />
    <dynamics damping="2.0" friction="0.2" />
  </joint>

  <link name="left_foot">
    <inertial>
      <mass value="1.2" />
      <origin xyz="0.05 0 -0.02" />
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.003" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.15 0.08 0.04" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.15 0.08 0.04" />
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin" />
    <child link="left_foot" />
    <origin xyz="0.0 0.0 -0.4" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="15.0" velocity="1.0" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <!-- Right Leg (mirror of left) -->
  <link name="right_hip">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.05" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" />
      <geometry>
        <cylinder length="0.1" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_yaw" type="revolute">
    <parent link="base_link" />
    <child link="right_hip" />
    <origin xyz="-0.08 -0.08 -0.1" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="30.0" velocity="1.0" />
    <dynamics damping="2.0" friction="0.2" />
  </joint>

  <link name="right_thigh">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_pitch" type="revolute">
    <parent link="right_hip" />
    <child link="right_thigh" />
    <origin xyz="0.0 0.0 -0.05" />
    <axis xyz="0 1 0" />
    <limit lower="-0.3" upper="2.2" effort="30.0" velocity="1.0" />
    <dynamics damping="2.0" friction="0.2" />
  </joint>

  <link name="right_shin">
    <inertial>
      <mass value="4.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.06" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.055" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.055" />
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh" />
    <child link="right_shin" />
    <origin xyz="0.0 0.0 -0.4" />
    <axis xyz="0 1 0" />
    <limit lower="0.0" upper="2.3" effort="30.0" velocity="1.0" />
    <dynamics damping="2.0" friction="0.2" />
  </joint>

  <link name="right_foot">
    <inertial>
      <mass value="1.2" />
      <origin xyz="0.05 0 -0.02" />
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.003" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.15 0.08 0.04" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.15 0.08 0.04" />
      </geometry>
    </collision>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_shin" />
    <child link="right_foot" />
    <origin xyz="0.0 0.0 -0.4" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="15.0" velocity="1.0" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <!-- Gazebo-specific configurations -->
  
  <!-- Depth Camera Configuration -->
  <gazebo reference="camera_link">
    <sensor name="depth_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <robotNamespace>/humanoid_robot</robotNamespace>
        <cameraName>head_camera</cameraName>
        <imageTopicName>/camera/rgb/image_raw</imageTopicName>
        <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
        <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
        <cameraInfoTopicName>/camera/rgb/camera_info</cameraInfoTopicName>
        <depthImageCameraInfoTopicName>/camera/depth/camera_info</depthImageCameraInfoTopicName>
        <frameName>camera_link</frameName>
        <baseline>0.1</baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
      </plugin>
    </sensor>
  </gazebo>

  <!-- LiDAR Configuration -->
  <gazebo reference="lidar_mount">
    <sensor name="humanoid_lidar" type="ray">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle> <!-- -180 degrees -->
            <max_angle>3.14159</max_angle>  <!-- 180 degrees -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
        <ros>
          <namespace>/humanoid_robot</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs::LaserScan</output_type>
        <frame_name>lidar_mount</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU Configuration -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.0000075</bias_mean>
              <bias_stddev>0.0000008</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.0000075</bias_mean>
              <bias_stddev>0.0000008</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.0000075</bias_mean>
              <bias_stddev>0.0000008</bias_stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.1</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.1</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.1</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/humanoid_robot</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Physics configuration -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>
</robot>
```

## Simulation Environment Setup

### World File for Complete Digital Twin

Now let's create a comprehensive world file that includes our humanoid robot in a realistic environment:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="humanoid_laboratory">
    <!-- Include default models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add laboratory furniture -->
    <model name="table_1">
      <pose>2 1 0 0 0 0</pose>
      <link name="table_base">
        <pose>0 0 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.0 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>5.8333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>11.4583</iyy>
            <iyz>0</iyz>
            <izz>15.8333</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <model name="chair_1">
      <pose>2.5 1.2 0 0 0 1.57</pose>
      <link name="chair_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.6 0.6 1.0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>0.4167</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.4167</iyy>
            <iyz>0</iyz>
            <izz>0.4167</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Add a simple humanoid figure for comparison -->
    <include>
      <uri>model://complete_humanoid</uri>
      <pose>1 0 1 0 0 0</pose> <!-- Start 1m above ground to avoid collision -->
    </include>
    
    <!-- Physics engine configuration -->
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.000001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    
    <!-- Light sources for Unity-style rendering -->
    <light name="main_light" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>10</range>
      </attenuation>
      <direction>-0.3 -0.3 -1</direction>
    </light>
    
    <light name="fill_light" type="point">
      <pose>5 5 2 0 0 0</pose>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.2</constant>
        <linear>0.1</linear>
        <quadratic>0.05</quadratic>
      </attenuation>
    </light>
  </world>
</sdf>
```

## Complete System Integration Example

Now let's create a comprehensive ROS 2 node that integrates all sensor data streams:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo, Imu, PointCloud2, JointState
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float32
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import PoseStamped
import numpy as np
import math
from collections import deque


class DigitalTwinIntegrator(Node):
    """
    Complete integration node that brings together all sensor data streams
    from the Digital Twin humanoid robot simulation
    """
    def __init__(self):
        super().__init__('digital_twin_integrator')
        
        # Initialize buffers for sensor data synchronization
        self.lidar_buffer = deque(maxlen=3)
        self.camera_buffer = deque(maxlen=3)
        self.imu_buffer = deque(maxlen=10)
        self.joint_buffer = deque(maxlen=5)
        self.odom_buffer = deque(maxlen=5)
        
        # Latest sensor readings
        self.latest_lidar = None
        self.latest_camera = None
        self.latest_imu = None
        self.latest_joints = None
        self.latest_odom = None
        
        # Publishers for integrated data
        self.integrated_data_pub = self.create_publisher(
            String, 
            '/humanoid_robot/integrated_sensor_data', 
            10
        )
        self.safety_status_pub = self.create_publisher(
            String, 
            '/humanoid_robot/safety_status', 
            10
        )
        self.environment_map_pub = self.create_publisher(
            OccupancyGrid, 
            '/humanoid_robot/environment_map', 
            10
        )
        
        # Subscribers for all sensor streams
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.lidar_callback,
            10
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/humanoid_robot/camera/rgb/image_raw',
            self.camera_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/humanoid_robot/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/humanoid_robot/joint_states',
            self.joint_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/humanoid_robot/odom',
            self.odom_callback,
            10
        )
        
        # Timer for main integration loop
        self.integration_timer = self.create_timer(0.1, self.integrate_sensor_data)  # 10 Hz
        
        # TF listener for transforming sensor data
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Robot state tracking
        self.robot_pose = None
        self.balance_state = "STABLE"
        self.safety_score = 1.0  # 1.0 = safe, 0.0 = unsafe
        
        # Integration parameters
        self.declare_parameter('integration_window', 0.1)  # seconds
        self.declare_parameter('safety_distance_threshold', 0.5)  # meters
        self.declare_parameter('max_tilt_threshold', 0.3)  # radians
        self.declare_parameter('balance_recover_threshold', 0.15)  # radians
        
        self.get_logger().info('Digital Twin Integrator initialized')

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        self.latest_lidar = msg
        self.lidar_buffer.append(msg)
        
    def camera_callback(self, msg):
        """Process RGB camera data"""
        self.latest_camera = msg
        self.camera_buffer.append(msg)
        
    def depth_callback(self, msg):
        """Process depth camera data"""
        # Use depth data combined with RGB for 3D perception
        pass
        
    def imu_callback(self, msg):
        """Process IMU data for balance and orientation tracking"""
        self.latest_imu = msg
        self.imu_buffer.append(msg)
        
        # Check balance state
        self.update_balance_state(msg)
        
    def joint_callback(self, msg):
        """Process joint state data"""
        self.latest_joints = msg
        self.joint_buffer.append(msg)
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.latest_odom = msg
        self.odom_buffer.append(msg)
        self.robot_pose = msg.pose.pose
        
    def update_balance_state(self, imu_msg):
        """
        Update the robot's balance state based on IMU data
        """
        # Extract orientation from quaternion
        orientation = imu_msg.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Convert to roll/pitch/yaw
        import tf_transformations as tf
        euler = tf.euler_from_quaternion(q)
        roll, pitch, yaw = euler
        
        # Calculate tilt angle
        tilt_angle = math.sqrt(roll*roll + pitch*pitch)
        
        # Update balance state based on tilt
        if tilt_angle > self.get_parameter('max_tilt_threshold').value:
            self.balance_state = "UNSTABLE"
            self.safety_score = max(0.0, 1.0 - (tilt_angle / self.get_parameter('max_tilt_threshold').value))
        elif tilt_angle > self.get_parameter('balance_recover_threshold').value:
            self.balance_state = "RECOVERING"
            self.safety_score = max(0.2, 1.0 - (tilt_angle / self.get_parameter('max_tilt_threshold').value))
        else:
            self.balance_state = "STABLE"
            self.safety_score = 1.0
    
    def integrate_sensor_data(self):
        """
        Main integration function that combines all sensor data streams
        """
        if not all([self.latest_lidar, self.latest_camera, self.latest_imu, 
                   self.latest_joints, self.latest_odom]):
            self.get_logger().debug('Waiting for all sensor data to arrive...')
            return
        
        # Create integrated data message
        integrated_data = {
            'timestamp': self.get_clock().now().to_msg(),
            'balance_state': self.balance_state,
            'safety_score': self.safety_score,
            'robot_pose': self.robot_pose,
            'joint_positions': dict(zip(self.latest_joints.name, self.latest_joints.position)),
            'obstacle_distances': self.extract_obstacle_data(self.latest_lidar),
            'environment_map': self.build_environment_map(self.latest_lidar, self.latest_odom),
            'orientation': self.extract_orientation(self.latest_imu)
        }
        
        # Publish integrated data
        data_msg = String()
        data_msg.data = str(integrated_data)  # In practice, you'd use a custom message type
        self.integrated_data_pub.publish(data_msg)
        
        # Publish safety status
        safety_msg = String()
        safety_msg.data = f"STATE:{self.balance_state}, SCORE:{self.safety_score:.2f}"
        self.safety_status_pub.publish(safety_msg)
        
        # Publish environment map
        env_map = self.build_environment_map(self.latest_lidar, self.latest_odom)
        self.environment_map_pub.publish(env_map)
        
        # Perform integration-specific tasks
        self.perform_safety_checks(integrated_data)
        self.update_navigation_with_sensor_data(integrated_data)
        self.log_integration_summary(integrated_data)

    def extract_obstacle_data(self, lidar_msg):
        """
        Extract obstacle information from LiDAR data
        """
        ranges = np.array(lidar_msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) == 0:
            return {'min_distance': float('inf'), 'front_clear': True}
        
        min_distance = np.min(valid_ranges)
        
        # Check if path in front is clear
        front_sector_size = int(len(ranges) * 0.1)  # 10% of field of view
        front_start = len(ranges) // 2 - front_sector_size // 2
        front_end = len(ranges) // 2 + front_sector_size // 2
        
        front_distances = ranges[front_start:front_end]
        valid_front_distances = front_distances[np.isfinite(front_distances)]
        
        front_clear = len(valid_front_distances) == 0 or np.min(valid_front_distances) > 0.5  # 50cm threshold
        
        return {
            'min_distance': min_distance,
            'front_clear': front_clear,
            'obstacle_count': len(valid_ranges)
        }

    def build_environment_map(self, lidar_msg, odom_msg):
        """
        Build a simple occupancy grid from LiDAR data
        """
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import Pose
        
        # Create a basic occupancy grid based on LiDAR data
        grid = OccupancyGrid()
        grid.header.stamp = lidar_msg.header.stamp
        grid.header.frame_id = "map"
        
        # Define grid parameters
        resolution = 0.1  # 10cm resolution
        width = 200  # 20m x 20m grid
        height = 200
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        
        # Set origin to robot's current position
        grid.info.origin.position.x = odom_msg.pose.pose.position.x - (width * resolution / 2)
        grid.info.origin.position.y = odom_msg.pose.pose.position.y - (height * resolution / 2)
        
        # Initialize all cells as unknown (-1)
        grid.data = [-1] * (width * height)
        
        # Process LiDAR ranges to populate the grid
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment
        
        robot_x = odom_msg.pose.pose.position.x
        robot_y = odom_msg.pose.pose.position.y
        
        for i, range_val in enumerate(lidar_msg.ranges):
            if not np.isfinite(range_val):
                continue
                
            angle = angle_min + i * angle_increment
            
            # Calculate world coordinates of obstacle
            world_x = robot_x + range_val * math.cos(angle + odom_msg.pose.pose.orientation.z)  # Simplified
            world_y = robot_y + range_val * math.sin(angle + odom_msg.pose.pose.orientation.z)  # Simplified
            
            # Convert to grid coordinates
            grid_x = int((world_x - grid.info.origin.position.x) / resolution)
            grid_y = int((world_y - grid.info.origin.position.y) / resolution)
            
            # Check bounds
            if 0 <= grid_x < width and 0 <= grid_y < height:
                grid.data[grid_y * width + grid_x] = 100  # Occupied
        
        return grid

    def extract_orientation(self, imu_msg):
        """
        Extract orientation data from IMU
        """
        return {
            'roll': self.quaternion_to_roll(imu_msg.orientation),
            'pitch': self.quaternion_to_pitch(imu_msg.orientation),
            'yaw': self.quaternion_to_yaw(imu_msg.orientation)
        }

    def quaternion_to_roll(self, q):
        """Convert quaternion to roll angle"""
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        return math.atan2(sinr_cosp, cosr_cosp)

    def quaternion_to_pitch(self, q):
        """Convert quaternion to pitch angle"""
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            return math.copysign(math.pi / 2, sinp)
        return math.asin(sinp)

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def perform_safety_checks(self, integrated_data):
        """
        Perform safety checks based on integrated sensor data
        """
        # Check for critical safety conditions
        if integrated_data['safety_score'] < 0.3:
            self.get_logger().error('CRITICAL SAFETY VIOLATION! EMERGENCY STOP!')
            # In a real system, this would trigger emergency procedures
        elif integrated_data['safety_score'] < 0.6:
            self.get_logger().warn('SAFETY THRESHOLD CROSSED! EXERCISING CAUTION!')
        else:
            self.get_logger().info('SAFETY STATUS: NOMINAL')
            
        # Check obstacle distances
        if integrated_data['obstacle_data']['min_distance'] < 0.3:  # 30cm
            self.get_logger().warn(
                f'CLOSE OBSTACLE DETECTED: {integrated_data["obstacle_data"]["min_distance"]:.2f}m'
            )
    
    def update_navigation_with_sensor_data(self, integrated_data):
        """
        Update navigation system with integrated sensor data
        """
        # This would typically send data to the navigation system
        # based on the integrated perception of the environment
        pass
    
    def log_integration_summary(self, integrated_data):
        """
        Log a summary of the integration for monitoring
        """
        summary = f"""
        DT INTEGRATION SUMMARY:
        Balance: {integrated_data['balance_state']}
        Safety: {integrated_data['safety_score']:.2f}
        Obstacles: {integrated_data['obstacle_data']['obstacle_count']}
        Min Dist: {integrated_data['obstacle_data']['min_distance']:.2f}m
        Front Clear: {integrated_data['obstacle_data']['front_clear']}
        Pos: ({integrated_data['robot_pose'].position.x:.2f}, 
              {integrated_data['robot_pose'].position.y:.2f})
        """
        self.get_logger().debug(summary)


def main(args=None):
    rclpy.init(args=args)
    integrator = DigitalTwinIntegrator()
    
    try:
        rclpy.spin(integrator)
    except KeyboardInterrupt:
        integrator.get_logger().info('Shutting down Digital Twin Integrator')
    finally:
        integrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Validation and Testing of the Complete System

### Unit Tests for Digital Twin Components

```python
import unittest
import rclpy
from sensor_msgs.msg import LaserScan, Imu
import math
import numpy as np


class TestDigitalTwinIntegration(unittest.TestCase):
    """
    Test suite for the complete Digital Twin implementation
    """
    def setUp(self):
        rclpy.init()
        self.integrator = DigitalTwinIntegrator()
    
    def tearDown(self):
        self.integrator.destroy_node()
        rclpy.shutdown()
    
    def test_lidar_integration(self):
        """
        Test LiDAR data processing and integration
        """
        # Create mock LiDAR message with some obstacles
        msg = LaserScan()
        msg.ranges = [1.0, 1.5, 2.0, 2.5, 3.0] * 72  # 720 total readings
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / len(msg.ranges)
        
        # Process the message
        self.integrator.lidar_callback(msg)
        
        # Verify processing
        self.assertIsNotNone(self.integrator.latest_lidar)
        self.assertEqual(len(self.integrator.latest_lidar.ranges), 360)
        
        # Check obstacle extraction
        obstacle_data = self.integrator.extract_obstacle_data(msg)
        self.assertLess(obstacle_data['min_distance'], 1.1)
        self.assertTrue(obstacle_data['front_clear'])
    
    def test_imu_balance_tracking(self):
        """
        Test IMU-based balance state tracking
        """
        # Test stable orientation
        msg = Imu()
        msg.orientation.w = 1.0  # Perfect upright orientation
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        
        self.integrator.imu_callback(msg)
        self.assertEqual(self.integrator.balance_state, "STABLE")
        self.assertAlmostEqual(self.integrator.safety_score, 1.0, places=1)
        
        # Test tilted orientation
        msg.orientation.w = 0.968  # ~15 degree tilt (cos(15°))
        msg.orientation.z = 0.258  # ~15 degree tilt (sin(15°))
        
        self.integrator.imu_callback(msg)
        # With 15 degree tilt, should still be in recovering state (threshold is 0.3 radians ~17 degrees)
        self.assertIn(self.integrator.balance_state, ["STABLE", "RECOVERING"])
        
    def test_environment_mapping(self):
        """
        Test environment map generation from LiDAR data
        """
        # Create a LiDAR message with simulated obstacles
        lidar_msg = LaserScan()
        lidar_msg.ranges = [1.0] * 360  # Obstacle at 1m in all directions
        lidar_msg.angle_min = -math.pi
        lidar_msg.angle_increment = 2 * math.pi / 360
        
        # Create an odometry message
        from nav_msgs.msg import Odometry
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        
        # Test environment map generation
        env_map = self.integrator.build_environment_map(lidar_msg, odom_msg)
        
        # Verify map properties
        self.assertEqual(env_map.info.resolution, 0.1)
        self.assertEqual(env_map.info.width, 200)
        self.assertEqual(env_map.info.height, 200)
        
        # Verify that the map has some occupied cells
        occupied_cells = sum(1 for val in env_map.data if val == 100)
        self.assertGreater(occupied_cells, 0)

    def test_sensor_integration(self):
        """
        Test the complete sensor integration pipeline
        """
        # Create mock data for all sensors
        lidar_msg = LaserScan()
        lidar_msg.ranges = [2.0] * 360
        lidar_msg.angle_min = -math.pi
        lidar_msg.angle_increment = 2 * math.pi / 360
        lidar_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        
        imu_msg = Imu()
        imu_msg.orientation.w = 1.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        
        # Process all sensor data
        self.integrator.lidar_callback(lidar_msg)
        self.integrator.imu_callback(imu_msg)
        
        # The integration timer should handle the integration
        # In a real test, we'd wait for the timer to execute
        

def run_validation_tests():
    """
    Run all validation tests for the Digital Twin implementation
    """
    test_suite = unittest.TestSuite()
    test_suite.addTest(unittest.makeSuite(TestDigitalTwinIntegration))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_validation_tests()
    print(f"\nIntegration tests: {'PASSED' if success else 'FAILED'}")
    exit(0 if success else 1)
```

## Performance Optimization and Validation

### Performance Benchmarks

For the complete Digital Twin system, we should establish performance benchmarks:

1. **Sensor Processing Latency**: All sensor data should be processed within 100ms
2. **Perception Accuracy**: LiDAR and depth camera should match real-world measurements within 5%
3. **Balance Estimation**: IMU-based balance estimation should update at least 10Hz
4. **Integration Throughput**: System should handle all sensor streams simultaneously without drops

## Chapter Summary

This chapter presented a complete implementation example of a Digital Twin system for humanoid robots that integrates all key sensor types:

- **LiDAR**: For environment mapping and obstacle detection
- **Depth Cameras**: For 3D perception and object recognition
- **IMUs**: For balance control and orientation tracking

The implementation demonstrates how these sensor types work together in a unified system, with proper ROS 2 communication patterns and data integration techniques. The example includes:

1. A complete humanoid robot model with appropriate sensor mounts
2. Gazebo simulation environment with realistic physics
3. ROS 2 nodes for processing and integrating sensor data
4. Validation and testing procedures for the integrated system

The Digital Twin approach allows for safe, cost-effective development and testing of humanoid robot capabilities before physical deployment, with simulation fidelity sufficient to transfer learned behaviors to real robots.

## Learning Objectives

After completing this chapter, students should be able to:
1. Integrate multiple sensor types in a unified Digital Twin system
2. Process and fuse data from LiDAR, depth cameras, and IMUs
3. Implement safety checks based on sensor fusion
4. Build environment maps from sensor data
5. Validate sensor simulation accuracy against real-world requirements
6. Create comprehensive testing frameworks for multi-sensor systems
7. Optimize performance for real-time sensor processing
8. Design architectures that facilitate transfer from simulation to reality
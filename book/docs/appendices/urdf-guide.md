# URDF Guide

This guide provides a comprehensive reference for Unified Robot Description Format (URDF), with focus on humanoid robot applications.

## What is URDF?

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial, visual, and collision characteristics.

## Basic URDF Structure

A URDF file has a robot as the root element, containing links, joints, and other optional elements:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define the rigid parts of the robot -->
  <link name="base_link">
    <!-- Inertial properties for physics simulation -->
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    
    <!-- Visual properties for visualization -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
      <material name="blue" />
    </visual>
    
    <!-- Collision properties for physics simulation -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
    </collision>
  </link>
  
  <!-- Joints define how links are connected -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link" />
    <child link="child_link" />
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>
  
  <link name="child_link">
    <!-- Child link definition -->
  </link>
</robot>
```

## Links

Links represent the rigid parts of a robot. Each link can have multiple child elements:

### Inertial Properties
The `<inertial>` element defines the inertial properties of the link for physics simulation:

```xml
<inertial>
  <mass value="1.0" />
  <origin xyz="0 0 0.1" rpy="0 0 0" />
  <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
</inertial>
```

- `mass`: The mass of the link in kilograms
- `origin`: The pose of the inertial reference frame relative to the link frame
- `inertia`: The 3x3 symmetric inertia matrix in the inertia reference frame

### Visual Properties
The `<visual>` element defines how the link appears in visualization:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0" />
  <geometry>
    <box size="0.1 0.1 0.1" />
  </geometry>
  <material name="blue" />
</visual>
```

- `origin`: The pose of the visual reference frame relative to the link frame
- `geometry`: The shape of the visual element (box, cylinder, sphere, mesh)
- `material`: The material properties (color, texture)

### Collision Properties
The `<collision>` element defines the collision boundaries for physics simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0" />
  <geometry>
    <box size="0.1 0.1 0.1" />
  </geometry>
</collision>
```

### Geometric Shapes
URDF supports several basic geometric shapes:

#### Box
```xml
<geometry>
  <box size="0.1 0.2 0.3" />
</geometry>
```

#### Cylinder
```xml
<geometry>
  <cylinder radius="0.05" length="0.1" />
</geometry>
```

#### Sphere
```xml
<geometry>
  <sphere radius="0.05" />
</geometry>
```

#### Mesh
```xml
<geometry>
  <mesh filename="package://my_package/meshes/link_model.stl" scale="1.0 1.0 1.0" />
</geometry>
```

## Joints

Joints define how links are connected and how they can move relative to each other. There are several joint types:

### Fixed Joint
No movement between links:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0.1" rpy="0 0 0" />
</joint>
```

### Revolute Joint
Rotation around a single axis:

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  <dynamics damping="0.1" friction="0.0" />
</joint>
```

### Continuous Joint
Like revolute but unlimited rotation:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
  <dynamics damping="0.1" friction="0.0" />
</joint>
```

### Prismatic Joint
Linear movement along an axis:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
  <limit lower="0.0" upper="0.5" effort="10.0" velocity="1.0" />
</joint>
```

### Joint Elements
- `parent`: The name of the parent link
- `child`: The name of the child link
- `origin`: The transform from the parent link to the joint frame
- `axis`: The joint axis in the joint frame
- `limit`: Joint limits (not for continuous joints)
- `dynamics`: Joint dynamics properties like damping and friction

## Materials

Materials define the visual appearance of links:

```xml
<material name="blue">
  <color rgba="0.0 0.0 1.0 1.0" />
</material>

<material name="red">
  <color rgba="1.0 0.0 0.0 1.0" />
</material>

<material name="my_material">
  <color rgba="0.5 0.5 0.5 1.0" />
  <texture filename="package://my_package/materials/textures/texture.png" />
</material>
```

## Complete Humanoid Robot Example

Here's a complete URDF for a simplified humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0" />
  </material>
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0" />
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0" />
  </material>

  <!-- Base link / Pelvis -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
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
      <mass value="8.0" />
      <origin xyz="0 0 0.25" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
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

  <!-- Torso joint -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link" />
    <child link="torso" />
    <origin xyz="0 0 0.2" rpy="0 0 0" />
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.15" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.15" />
      </geometry>
    </collision>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.5" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="5.0" velocity="1.0" />
  </joint>

  <!-- Left arm -->
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

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="left_upper_arm" />
    <origin xyz="0.13 0.1 0.2" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

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

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm" />
    <child link="left_lower_arm" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-2.0" upper="0.0" effort="10.0" velocity="1.0" />
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

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm" />
    <child link="left_hand" />
    <origin xyz="0 0 -0.24" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.0" upper="1.0" effort="5.0" velocity="1.0" />
  </joint>

  <!-- Right arm (similar to left, mirrored) -->
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

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="right_upper_arm" />
    <origin xyz="0.13 -0.1 0.2" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
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

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm" />
    <child link="right_lower_arm" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-2.0" upper="0.0" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Right hand -->
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

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_lower_arm" />
    <child link="right_hand" />
    <origin xyz="0 0 -0.24" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.0" upper="1.0" effort="5.0" velocity="1.0" />
  </joint>

  <!-- Left leg -->
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

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="left_upper_leg" />
    <origin xyz="-0.08 0.08 -0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="20.0" velocity="1.0" />
  </joint>

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

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg" />
    <child link="left_lower_leg" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="0.0" upper="2.3" effort="20.0" velocity="1.0" />
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

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg" />
    <child link="left_foot" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Right leg (similar to left, mirrored) -->
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

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="right_upper_leg" />
    <origin xyz="-0.08 -0.08 -0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="20.0" velocity="1.0" />
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

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg" />
    <child link="right_lower_leg" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="0.0" upper="2.3" effort="20.0" velocity="1.0" />
  </joint>

  <!-- Right foot -->
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

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg" />
    <child link="right_foot" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>
</robot>
```

## Using Xacro for Complex URDFs

Xacro (XML Macros) is an XML macro language that extends URDF with features like constants, properties, macros, and mathematical expressions:

### Basic Xacro Example
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.08" />

  <!-- Define a macro for wheels -->
  <xacro:macro name="wheel" params="prefix x y z">
    <link name="${prefix}_wheel">
      <inertial>
        <mass value="1.0" />
        <origin xyz="0 0 0" />
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}" />
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}" />
        </geometry>
      </collision>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link" />
      <child link="${prefix}_wheel" />
      <origin xyz="${x} ${y} ${z}" rpy="0 0 0" />
      <axis xyz="0 1 0" />
    </joint>
  </xacro:macro>

  <!-- Use the macro to create wheels -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2" />
      </geometry>
    </visual>
  </link>

  <xacro:wheel prefix="front_left" x="0.2" y="0.2" z="0" />
  <xacro:wheel prefix="front_right" x="0.2" y="-0.2" z="0" />
  <xacro:wheel prefix="back_left" x="-0.2" y="0.2" z="0" />
  <xacro:wheel prefix="back_right" x="-0.2" y="-0.2" z="0" />
</robot>
```

## URDF Best Practices

1. **Use consistent naming**: Use descriptive names with consistent prefixes (e.g., "left_shoulder_joint", "right_shoulder_joint")

2. **Organize complex URDFs with Xacro**: Use macros and properties to reduce repetition

3. **Accurate inertial parameters**: Get mass and inertia values from CAD software or calculate them properly

4. **Appropriate joint limits**: Set realistic limits based on physical constraints

5. **Proper origin definitions**: Define origins carefully to ensure correct robot kinematics

6. **Separate visual and collision models**: Use simpler collision models for computational efficiency

## Validating URDF Files

### Using check_urdf command:
```bash
check_urdf /path/to/robot.urdf
```

### Using xacro if using macros:
```bash
xacro /path/to/robot.urdf.xacro > /tmp/expanded.urdf
check_urdf /tmp/expanded.urdf
```

## Simulation-Specific Elements

### Gazebo Plugins:
```xml
<gazebo reference="left_wheel">
  <mu1>0.1</mu1>
  <mu2>0.1</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>

<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</gazebo>
```

## Common URDF Issues and Solutions

1. **Self-collisions**: Adjust collision models to prevent parts that normally touch from colliding

2. **Gravity turn**: Add fixed joints with small offsets to prevent floating joints from moving due to numerical errors

3. **Kinematic loops**: URDF doesn't support closed kinematic chains directly - use separate mechanisms

4. **Inertia calculation**: Use CAD tools or online calculators for accurate inertia values

## Visualization Tools

- **RViz2**: Visualize URDF models in ROS2
- **Gazebo**: Physics simulation with visualization
- **URDF Viewer**: Standalone tools to view URDF files
- **Blender**: Import URDF with appropriate plugins

This guide provides the fundamentals needed to create and work with URDF files for humanoid robots. For complex robots, consider using Xacro to make your URDFs more maintainable and reusable.
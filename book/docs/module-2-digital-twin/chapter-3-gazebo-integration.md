# Chapter 3: Gazebo Integration for Robotics Simulation with ROS 2

## Introduction to Gazebo for Robotics

Gazebo is a physics-based 3D simulator that provides realistic virtual environments for robotics research and development. It integrates seamlessly with ROS 2, making it an essential tool for developing and testing humanoid robots within the ROS ecosystem. Gazebo provides:

- High-performance physics engines (ODE, Bullet, Simbody)
- Realistic sensors (cameras, LiDAR, IMU, GPS, etc.)
- Dynamic scene rendering with support for lighting and shadows
- Extension via plugins for custom behaviors
- Realistic environment simulation with support for indoor and outdoor scenarios

### Why Gazebo for Humanoid Robots?

For humanoid robots specifically, Gazebo offers:

1. **Accurate Physics Simulation**: Essential for simulating walking, balance, and manipulation
2. **Complex Environments**: Multi-story buildings, furniture, and obstacles
3. **Sensor Simulation**: Cameras, LiDAR, IMU, and force/torque sensors
4. **Realistic Contact Physics**: Critical for bipedal locomotion and manipulation
5. **ROS 2 Integration**: Direct communication with ROS 2 nodes

## Setting Up Gazebo with ROS 2

### Installation Requirements

For Ubuntu 22.04 with ROS 2 Humble Hawksbill:

```bash
# Install Gazebo Fortress (or Garden, depending on your needs)
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Install Gazebo simulator separately
wget https://osrf-distributions.s3.amazonaws.com/gazebo/releases/gazebo11_11.14.0_amd64.deb
sudo dpkg -i gazebo11_11.14.0_amd64.deb

# Install additional dependencies
sudo apt-get install -y libgazebo11-dev gazebo11-common
```

### Basic Gazebo-ROS 2 Communication

Gazebo communicates with ROS 2 through the `gazebo_ros_pkgs`, which provide plugins for:
- Publishing simulation state to ROS topics
- Subscribing to ROS topics to control simulated robots
- Serving ROS services for world control
- Action interfaces for complex interactions

## Creating a Robot Model for Gazebo

### URDF to SDF Conversion

While Gazebo natively uses SDF (Simulation Description Format), ROS 2 workflows often use URDF. The `gazebo_ros` package handles the conversion:

```xml
<!-- In your URDF, add Gazebo-specific extensions -->
<robot name="humanoid_robot">
  <!-- Your robot's links and joints -->
  
  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>
  
  <!-- Sensor plugins for each sensor -->
  <gazebo reference="head_camera">
    <sensor name="head_camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.3962634</horizontal_fov> <!-- 80 degrees in radians -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>head_camera_optical_frame</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>100</max_depth>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Basic Gazebo Integration Example

Let's create a complete example that shows how to integrate a simple robot model with Gazebo and ROS 2.

### Robot URDF with Gazebo Extensions

```xml
<?xml version="1.0"?>
<robot name="simple_gazebo_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link: Pelvis -->
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

  <!-- Gazebo-specific configurations -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_gazebo_humanoid</robotNamespace>
    </plugin>
  </gazebo>

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
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <!-- Add Gazebo extensions for joints that need control -->
  <gazebo reference="left_hip_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
</robot>
```

### World File for Simulation

Create a simple world file to test the robot:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Physics engine configuration -->
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    
    <!-- Place our robot in the world -->
    <model name="simple_gazebo_humanoid">
      <include>
        <uri>model://simple_gazebo_humanoid</uri>
      </include>
      <pose>0 0 1 0 0 0</pose> <!-- Start 1 meter above ground to avoid collision -->
    </model>
    
    <!-- Add some simple obstacles -->
    <model name="block_1">
      <pose>2 0 0.2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.016667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.016667</iyy>
            <iyz>0</iyz>
            <izz>0.016667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Launching Gazebo with ROS 2

### Launch File

To automate the process of launching Gazebo with our robot, we create a launch file:

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )
    
    # Get URDF via xacro
    robot_description = Command(['xacro ', PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'simple_gazebo_humanoid.urdf.xacro'
    ])])
    
    # Robot State Publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Gazebo node
    gazebo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_gazebo_humanoid',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '1.0'  # Start above ground to avoid collision
        ],
        output='screen'
    )
    
    # Gazebo server and client
    gazebo_server = Node(
        package='gazebo_ros',
        executable='gzserver',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    gazebo_client = Node(
        package='gazebo_ros',
        executable='gzclient',
        condition=LaunchConfiguration('gui'),
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        gazebo,
    ])
```

## Basic Robot Control in Gazebo

### Publishing Joint Commands

To control the robot in Gazebo, we can publish to the appropriate ROS 2 topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class SimpleHumanoidController(Node):
    def __init__(self):
        super().__init__('simple_humanoid_controller')
        
        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray, 
            '/simple_gazebo_humanoid/forward_position_controller/commands', 
            10
        )
        
        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/simple_gazebo_humanoid/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer to send commands
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.joint_positions = {}
        self.command_index = 0
    
    def joint_state_callback(self, msg):
        """Update internal joint position tracking"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
    
    def control_loop(self):
        """Send periodic joint commands"""
        # Create a periodic command for the neck joint
        cmd_msg = Float64MultiArray()
        
        # For this example, we'll just move the neck joint back and forth
        neck_pos = 0.3 * math.sin(self.command_index * 0.1)
        cmd_msg.data = [neck_pos]  # Only controlling the neck joint
        
        self.joint_cmd_publisher.publish(cmd_msg)
        self.command_index += 1


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleHumanoidController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced Gazebo Features for Humanoid Robots

### Sensor Integration

For humanoid robots, integrating sensors is crucial. Here's how to add different types of sensors:

```xml
<!-- IMU sensor -->
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
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
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/simple_gazebo_humanoid</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>

<!-- LiDAR sensor -->
<gazebo reference="head">
  <sensor name="lidar_sensor" type="ray">
    <pose>0.1 0 0 0 0 0</pose> <!-- Offset from head center -->
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.5708</min_angle> <!-- -90 degrees in radians -->
          <max_angle>1.5708</max_angle>   <!-- 90 degrees in radians -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/simple_gazebo_humanoid</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Working with Controllers

To properly control a humanoid robot in Gazebo, we typically use the ROS 2 control framework:

### Controller Configuration (config/controllers.yaml)
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_controller:
      type: joint_state_controller/JointStateController

    neck_position_controller:
      type: position_controllers/JointPositionController

joint_state_controller:
  ros__parameters:
    type: joint_state_controller/JointStateController
    publish_rate: 50

neck_position_controller:
  ros__parameters:
    type: position_controllers/JointPositionController
    joint: neck_joint
```

## Debugging Gazebo-ROS Issues

Common issues and solutions:

1. **Robot falls through the ground:**
   - Check that collision geometries are defined
   - Verify mass and inertia parameters
   - Ensure physics engine is running

2. **Controllers not working:**
   - Check controller manager is running
   - Verify configuration files are correct
   - Ensure joint names match between URDF and controller configs

3. **No TF transforms:**
   - Ensure robot_state_publisher is running
   - Check that joint states are being published

## Best Practices for Gazebo Integration

1. **Start Simple**: Begin with a basic model before adding complex features
2. **Validate URDF**: Use `check_urdf` to validate before simulation
3. **Tune Physics**: Adjust step size and solver parameters for stability
4. **Use Proper Inertias**: Calculate realistic inertia values for stable simulation
5. **Add Sensors Gradually**: Test each sensor individually before complex configurations

## Chapter Summary

Gazebo provides a powerful simulation environment for humanoid robots when properly integrated with ROS 2. Through the use of URDF extensions, appropriate world files, launch files, and controllers, we can create realistic simulation environments that closely mirror real-world behavior. Understanding the Gazebo-ROS integration is essential for developing and testing humanoid robots in a safe, controlled environment before physical implementation.

## Learning Objectives

After completing this chapter, the reader should be able to:
- Understand the role of Gazebo in the ROS 2 ecosystem
- Configure basic Gazebo integration for a humanoid robot
- Implement sensors and controllers in Gazebo
- Troubleshoot common Gazebo-ROS integration issues
- Create appropriate launch files for Gazebo simulation
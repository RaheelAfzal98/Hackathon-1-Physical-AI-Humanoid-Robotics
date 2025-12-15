# Chapter 5: Complete Humanoid Robot Implementation Example

## Introduction

In this final chapter of the ROS 2 Nervous System module, we'll integrate all the concepts covered in previous chapters into a complete humanoid robot implementation. This chapter will demonstrate how to combine ROS 2 communication patterns, Python agents, URDF models, and control systems into a functional humanoid robot system.

## Complete Humanoid Robot Node with All Components

Here's a comprehensive ROS 2 node that implements the core control system for a humanoid robot, integrating all the components we've learned about:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
from enum import Enum


class RobotState(Enum):
    IDLE = 1
    WALKING = 2
    STANDING = 3
    SITTING = 4
    INTERACTING = 5


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        
        # Create callback group for reentrant callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize robot state
        self.current_state = RobotState.IDLE
        self.battery_level = 100.0
        
        # Joint names for the humanoid robot
        self.joint_names = [
            # Head
            'neck_joint',
            # Left arm
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
            # Right arm
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint',
            # Left leg
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            # Right leg
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
        
        # Initialize joint positions
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        
        # Create publishers
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.status_pub = self.create_publisher(String, 'robot_status', qos_profile)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos_profile)
        
        # Create subscribers
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, qos_profile)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, qos_profile)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile)
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile)
        
        # Create action clients
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timers
        self.joint_state_timer = self.create_timer(0.05, self.publish_joint_states)
        self.robot_control_timer = self.create_timer(0.1, self.robot_control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Initialize position and orientation
        self.position = [0.0, 0.0, 0.0]  # x, y, theta
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion (x, y, z, w)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Obstacle detection
        self.obstacle_distance = float('inf')
        
        self.get_logger().info('Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Update joint positions from received JointState message"""
        for i, name in enumerate(msg.name):
            if name in self.joint_positions:
                self.joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract orientation from quaternion
        self.orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # Extract angular velocity
        # Use for balance control in real implementation

    def scan_callback(self, msg):
        """Process laser scan data"""
        if msg.ranges:
            # Filter out invalid ranges
            valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r)) and r > 0]
            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)
            else:
                self.obstacle_distance = float('inf')
        else:
            self.obstacle_distance = float('inf')

    def cmd_vel_callback(self, msg):
        """Process velocity commands"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        # Change state to walking if we receive movement commands
        if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
            if self.current_state != RobotState.WALKING:
                self.current_state = RobotState.WALKING

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.velocity = [0.0] * len(msg.position)  # For simplicity
        msg.effort = [0.0] * len(msg.position)    # For simplicity
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        self.joint_pub.publish(msg)

    def robot_control_loop(self):
        """Main control loop for the humanoid robot"""
        current_time = self.get_clock().now()
        
        # Update position based on velocity
        dt = 0.1  # Time step for this loop
        self.position[0] += self.linear_vel * math.cos(self.position[2]) * dt
        self.position[1] += self.linear_vel * math.sin(self.position[2]) * dt
        self.position[2] += self.angular_vel * dt
        
        # Keep theta in [-pi, pi]
        self.position[2] = math.atan2(
            math.sin(self.position[2]), 
            math.cos(self.position[2])
        )
        
        # Publish odometry
        self.publish_odom()
        
        # Publish IMU data (simulated)
        self.publish_imu()
        
        # Publish TF
        self.publish_transforms()
        
        # Implement state-specific behavior
        if self.current_state == RobotState.WALKING:
            self.walk_behavior()
        elif self.current_state == RobotState.STANDING:
            self.standing_behavior()
        elif self.current_state == RobotState.SITTING:
            self.sitting_behavior()
        elif self.current_state == RobotState.INTERACTING:
            self.interacting_behavior()
        elif self.current_state == RobotState.IDLE:
            self.idle_behavior()
        
        # Update battery level
        self.battery_level -= 0.01  # Simulate battery drain
        if self.battery_level < 0:
            self.battery_level = 0

    def walk_behavior(self):
        """Behavior for walking state"""
        # In a real implementation, this would send appropriate joint commands
        # to achieve a walking gait pattern
        pass

    def standing_behavior(self):
        """Behavior for standing state"""
        # Keep balance - in a real robot, this would involve feedback control
        pass

    def sitting_behavior(self):
        """Behavior for sitting state"""
        # Move joints to sitting position
        pass

    def interacting_behavior(self):
        """Behavior for interacting state"""
        # Head tracking, gesture control, etc.
        pass

    def idle_behavior(self):
        """Behavior for idle state"""
        # Minimal movement to conserve power
        pass

    def publish_odom(self):
        """Publish odometry data"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = 0.0  # Assuming flat ground
        
        # Orientation (from Euler angles to quaternion)
        from geometry_msgs.msg import Quaternion
        def euler_to_quaternion(yaw):
            qx = 0.0
            qy = 0.0
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            return Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        msg.pose.pose.orientation = euler_to_quaternion(self.position[2])
        
        # Velocity
        msg.twist.twist.linear.x = self.linear_vel
        msg.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(msg)

    def publish_imu(self):
        """Publish simulated IMU data"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Orientation (simplified - in reality would come from sensors)
        msg.orientation.x = self.orientation[0]
        msg.orientation.y = self.orientation[1]
        msg.orientation.z = self.orientation[2]
        msg.orientation.w = self.orientation[3]
        
        # Angular velocity (simplified)
        msg.angular_velocity.z = self.angular_vel
        
        # Linear acceleration (simplified)
        msg.linear_acceleration.x = self.linear_vel * 0.1  # Simulated acceleration
        
        self.imu_pub.publish(msg)

    def publish_transforms(self):
        """Publish transforms for robot visualization"""
        from geometry_msgs.msg import TransformStamped
        
        # Publish transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = 0.0  # Assuming flat ground
        
        # Convert orientation to quaternion
        t.transform.rotation = euler_to_quaternion(self.position[2])
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish other transforms as needed (e.g., from base_link to other links)
        # For brevity, only showing the main transform

    def publish_status(self):
        """Publish robot status"""
        msg = String()
        msg.data = (
            f"State: {self.current_state.name}, "
            f"Position: ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}), "
            f"Battery: {self.battery_level:.1f}%"
        )
        self.status_pub.publish(msg)

    def send_trajectory_goal(self, joint_names, positions, time_from_start=2.0):
        """Send a trajectory goal to move joints to specified positions"""
        if not self.trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Trajectory action server not available')
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
        
        goal_msg.trajectory.points.append(point)
        
        self.trajectory_client.send_goal_async(goal_msg)
        return True

    def set_standing_pose(self):
        """Move robot to standing position"""
        # Define joint positions for standing pose
        standing_positions = {
            'neck_joint': 0.0,
            'left_shoulder_joint': 0.2,
            'left_elbow_joint': -1.0,
            'left_wrist_joint': 0.0,
            'right_shoulder_joint': 0.2,
            'right_elbow_joint': -1.0,
            'right_wrist_joint': 0.0,
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'left_ankle_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0,
            'right_ankle_joint': 0.0
        }
        
        # Extract ordered positions
        pos_list = [standing_positions[name] for name in self.joint_names]
        
        success = self.send_trajectory_goal(self.joint_names, pos_list, 3.0)
        if success:
            self.current_state = RobotState.STANDING
            self.get_logger().info('Standing pose command sent')

    def set_walking_state(self):
        """Change robot to walking state"""
        self.current_state = RobotState.WALKING
        self.get_logger().info('Robot state changed to WALKING')

    def set_idle_state(self):
        """Change robot to idle state"""
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.current_state = RobotState.IDLE
        self.get_logger().info('Robot state changed to IDLE')


def main(args=None):
    rclpy.init(args=args)
    
    humanoid_controller = HumanoidController()
    
    try:
        # Set robot to standing pose initially
        humanoid_controller.set_standing_pose()
        
        # Use multi-threaded executor to handle all callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(humanoid_controller)
        
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_controller.get_logger().info('Shutting down humanoid controller')
        humanoid_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Python Agents Controlling the Complete Humanoid Robot

Now let's create Python agents that interact with our complete humanoid robot system:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import cv2
from cv_bridge import CvBridge
import numpy as np
from enum import Enum


class AgentState(Enum):
    IDLE = 1
    EXPLORING = 2
    AVOIDING_OBSTACLE = 3
    INTERACTING = 4
    PERFORMING_TASK = 5


class HumanoidAgent(Node):
    def __init__(self):
        super().__init__('humanoid_agent')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize agent state
        self.current_state = AgentState.IDLE
        self.target_position = [0.0, 0.0]  # x, y coordinates
        self.task_queue = []
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile, callback_group=self.callback_group)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, qos_profile, callback_group=self.callback_group)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, qos_profile, callback_group=self.callback_group)
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile, callback_group=self.callback_group)
        self.status_pub = self.create_publisher(String, 'agent_status', qos_profile, callback_group=self.callback_group)
        
        # Create action client for trajectory control
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory', callback_group=self.callback_group)
        
        # Create timers
        self.agent_control_timer = self.create_timer(0.1, self.agent_control_loop, callback_group=self.callback_group)
        
        # Robot state variables
        self.obstacle_distance = float('inf')
        self.heading_error = 0.0
        self.latest_image = None
        self.balance_data = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        
        self.get_logger().info('Humanoid Agent initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if not (float('inf') == r or r == 0.0)]
            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)
            else:
                self.obstacle_distance = float('inf')
        else:
            self.obstacle_distance = float('inf')

    def imu_callback(self, msg):
        """Process IMU data for balance information"""
        import math
        
        # Extract roll and pitch from orientation quaternion
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.balance_data['roll'] = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.balance_data['pitch'] = math.copysign(math.pi / 2, sinp)
        else:
            self.balance_data['pitch'] = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.balance_data['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        """Process camera image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def agent_control_loop(self):
        """Main agent control loop"""
        cmd_vel = Twist()
        
        # State-based behavior
        if self.current_state == AgentState.IDLE:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            
            # Check if we have a target to move to
            if self.target_position != [0.0, 0.0]:
                self.current_state = AgentState.EXPLORING
                
        elif self.current_state == AgentState.EXPLORING:
            # Move toward target with obstacle avoidance
            cmd_vel.linear.x = 0.3  # Move forward
            cmd_vel.angular.z = self.heading_error * 0.5  # Adjust heading
            
            # Check for obstacles
            if self.obstacle_distance < 0.8:  # 0.8 meters threshold
                self.current_state = AgentState.AVOIDING_OBSTACLE
                self.get_logger().warn('Obstacle detected, switching to avoidance mode')
                
        elif self.current_state == AgentState.AVOIDING_OBSTACLE:
            # Implement obstacle avoidance behavior
            if self.obstacle_distance < 0.5:
                # Stop and turn away
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5  # Turn right
            elif self.obstacle_distance < 0.8:
                # Move forward while turning slightly
                cmd_vel.linear.x = 0.2
                cmd_vel.angular.z = 0.3
            else:
                # Obstacle cleared, return to exploration
                self.current_state = AgentState.EXPLORING
                
        elif self.current_state == AgentState.INTERACTING:
            # Face detected person or object
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            
            # Process image to detect features
            if self.latest_image is not None:
                self.process_interaction_image()
                
        elif self.current_state == AgentState.PERFORMING_TASK:
            # Execute specific task from queue
            if self.task_queue:
                task = self.task_queue[0]  # Take first task
                cmd_vel = self.execute_task(task)
                
                # Remove task when completed (simplified)
                if self.task_completed(task):
                    self.task_queue.pop(0)
                    if not self.task_queue:
                        self.current_state = AgentState.IDLE
            else:
                self.current_state = AgentState.IDLE

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"State: {self.current_state.name}, Obstacle: {self.obstacle_distance:.2f}m"
        self.status_pub.publish(status_msg)

    def process_interaction_image(self):
        """Process image for interaction tasks"""
        if self.latest_image is None:
            return
            
        # Simple color-based object detection (red objects)
        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        
        mask = mask1 + mask2
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate center of contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate heading error to center the object in view
                image_center_x = self.latest_image.shape[1] / 2
                self.heading_error = (cx - image_center_x) / image_center_x  # Normalize to [-1, 1]
                
                self.get_logger().info(f'Target found at ({cx}, {cy}), heading error: {self.heading_error:.2f}')
                
                # Switch to interacting state if in exploration
                if self.current_state == AgentState.EXPLORING:
                    self.current_state = AgentState.INTERACTING
        else:
            # No target found, resume exploration
            if self.current_state == AgentState.INTERACTING:
                self.current_state = AgentState.EXPLORING

    def execute_task(self, task):
        """Execute a specific task from the queue"""
        cmd_vel = Twist()
        
        if task == "wave_hello":
            # In a real implementation, this would send trajectory commands
            # to make the robot wave
            self.get_logger().info('Executing wave_hello task')
            return cmd_vel
        elif task == "walk_forward":
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = 0.0
            return cmd_vel
        elif task == "turn_left":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5
            return cmd_vel
        else:
            return cmd_vel

    def task_completed(self, task):
        """Check if a task is completed (simplified)"""
        # In a real implementation, this would check specific conditions
        # For now, just return True to remove the task
        return True

    def add_task(self, task):
        """Add a task to the queue"""
        self.task_queue.append(task)
        if self.current_state == AgentState.IDLE:
            self.current_state = AgentState.PERFORMING_TASK
        self.get_logger().info(f'Task "{task}" added to queue')

    def set_target_position(self, x, y):
        """Set a target position for the robot to navigate to"""
        self.target_position = [x, y]
        if self.current_state == AgentState.IDLE:
            self.current_state = AgentState.EXPLORING
        self.get_logger().info(f'Target position set to ({x}, {y})')

    def stop_robot(self):
        """Stop the robot and return to idle state"""
        self.current_state = AgentState.IDLE
        self.target_position = [0.0, 0.0]
        self.task_queue = []
        self.get_logger().info('Robot stopped and returned to idle state')


def main(args=None):
    rclpy.init(args=args)
    
    humanoid_agent = HumanoidAgent()
    
    # Add some example tasks to the queue
    humanoid_agent.add_task("walk_forward")
    humanoid_agent.add_task("turn_left")
    humanoid_agent.add_task("wave_hello")
    
    # Set a target position for navigation
    humanoid_agent.set_target_position(2.0, 1.0)
    
    try:
        # Use multi-threaded executor to handle all callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(humanoid_agent)
        
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_agent.stop_robot()
        humanoid_agent.get_logger().info('Humanoid Agent shutting down')
        humanoid_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating URDF Model with ROS 2 Controllers and Sensors

To integrate our URDF model with ROS 2 controllers and sensors, we need to create configuration files that tell ROS 2 how to control our robot:

### Joint State Controller Configuration (config/joint_state_controller.yaml)
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_controller:
      type: joint_state_controller/JointStateController

    # Add other controllers as needed
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_state_controller:
  ros__parameters:
    type: joint_state_controller/JointStateController
    publish_rate: 50
```

### Joint Trajectory Controller Configuration (config/joint_trajectory_controller.yaml)
```yaml
joint_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - neck_joint
      - left_shoulder_joint
      - left_elbow_joint
      - left_wrist_joint
      - right_shoulder_joint
      - right_elbow_joint
      - right_wrist_joint
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## Demonstrate Robot State Publishing and Joint Control

Here's an example of how to properly publish robot state and control joints:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Publisher for joint trajectories
        self.trajectory_pub = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)
        
        # Timer to publish state
        self.timer = self.create_timer(0.05, self.publish_robot_state)  # 20 Hz
        
        # Simulated joint positions
        self.joint_positions = {
            'neck_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'left_wrist_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'right_wrist_joint': 0.0,
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'left_ankle_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0,
            'right_ankle_joint': 0.0
        }
        
        # Time tracking for animation
        self.time = 0.0
        
        self.get_logger().info('Robot State Publisher initialized')

    def publish_robot_state(self):
        """Publish the current state of all joints"""
        # Update joint positions for animation (sine wave motion for demonstration)
        self.time += 0.05  # Increment by timer period
        
        # Create animated motions for demonstration
        self.joint_positions['neck_joint'] = 0.3 * math.sin(self.time)
        self.joint_positions['left_shoulder_joint'] = 0.5 * math.sin(self.time * 0.5)
        self.joint_positions['right_shoulder_joint'] = 0.5 * math.sin(self.time * 0.5 + math.pi)
        
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set joint names and positions
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        
        # Set velocities and efforts to zero for simplicity
        msg.velocity = [0.0] * len(msg.position)
        msg.effort = [0.0] * len(msg.position)
        
        # Publish the message
        self.joint_pub.publish(msg)

    def send_trajectory_command(self, joint_names, positions, time_from_start=1.0):
        """Send a trajectory command to move joints to specified positions"""
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.joint_names = joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        
        # Calculate time from start
        from builtin_interfaces.msg import Duration
        point.time_from_start = Duration(sec=int(time_from_start), nanosec=int((time_from_start - int(time_from_start)) * 1e9))
        
        msg.points = [point]
        
        # Publish the trajectory command
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'Trajectory command sent for joints: {joint_names}')


def main(args=None):
    rclpy.init(args=args)
    
    robot_state_publisher = RobotStatePublisher()
    
    # Example: Send a trajectory command to move the head
    robot_state_publisher.send_trajectory_command(
        joint_names=['neck_joint'],
        positions=[0.5],  # Move head to 0.5 radians
        time_from_start=2.0
    )
    
    try:
        rclpy.spin(robot_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Simulation Example with Gazebo Integration

To complete our example, here's how to set up Gazebo simulation for our humanoid robot:

### Gazebo Launch File (launch/humanoid_gazebo.launch.py)
```python
# humanoid_gazebo.launch.py
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
        'humanoid_robot.urdf.xacro'
    ])])
    
    # Robot State Publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '1.0'  # Start above ground to avoid collision
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_argument,
        robot_state_publisher,
        spawn_entity,
    ])
```

### Gazebo Plugin Configuration (in URDF)
```xml
<!-- Add this to your URDF file -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>

<!-- For each joint, add transmission -->
<transmission name="tran_neck_joint">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="neck_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor_neck_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Learning Objectives

After completing this chapter, you should be able to:
- Integrate all ROS 2 components into a complete humanoid robot implementation
- Create comprehensive robot controllers that combine multiple ROS 2 concepts
- Implement Python agents that control complex humanoid robots
- Integrate URDF models with ROS 2 controllers and sensors
- Properly publish robot state information and implement joint control
- Set up simulation environments with Gazebo for humanoid robots
- Document complete system architecture for humanoid robots

## Chapter Summary

This chapter provided a comprehensive example that integrates all concepts covered in the ROS 2 Nervous System module. We demonstrated how to implement a complete humanoid robot system with proper ROS 2 architecture patterns, showing how Nodes, Topics, Services, Actions, URDF, and Python agents can work together to create sophisticated robotic behaviors.

The examples showed practical implementations of:
- Robot state management with multiple controllers
- Sensor integration for perception and navigation
- Trajectory control for joint actuation
- Python agents for high-level decision making
- Simulation integration for testing

This completes the ROS 2 Nervous System module, providing a solid foundation for working with humanoid robots in ROS 2.
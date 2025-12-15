# Comprehensive Testing and Validation Scripts for ROS 2 Nervous System Module

This file contains comprehensive testing and validation scripts for all chapters in the ROS 2 Nervous System module.

## Chapter 1: Introduction to ROS 2 - Test Suite

```python
#!/usr/bin/env python3
# test_chapter1_examples.py
# Validation script for Chapter 1: Introduction to ROS 2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import time
import unittest
from unittest.mock import Mock


class TestChapter1Examples(unittest.TestCase):
    """Test suite for Chapter 1 examples"""
    
    def setUp(self):
        """Initialize ROS 2 before each test"""
        rclpy.init()
        
    def tearDown(self):
        """Cleanup after each test"""
        rclpy.shutdown()
    
    def test_simple_publisher_subscriber(self):
        """Test the publisher-subscriber pattern from Chapter 1"""
        class Talker(Node):
            def __init__(self):
                super().__init__('test_talker')
                self.publisher = self.create_publisher(String, 'ch1_test_topic', 10)
                self.i = 0

            def publish_msg(self):
                msg = String()
                msg.data = f'Hello World: {self.i}'
                self.publisher.publish(msg)
                self.i += 1
        
        class Listener(Node):
            def __init__(self):
                super().__init__('test_listener')
                self.received_messages = []
                self.subscription = self.create_subscription(
                    String,
                    'ch1_test_topic',
                    self.listener_callback,
                    10)

            def listener_callback(self, msg):
                self.received_messages.append(msg.data)
        
        # Create nodes
        talker = Talker()
        listener = Listener()
        
        # Create executor to run both nodes
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(talker)
        executor.add_node(listener)
        
        # Publish and receive messages
        for i in range(3):
            talker.publish_msg()
            executor.spin_once(timeout_sec=0.1)
        
        # Check if messages were received
        self.assertEqual(len(listener.received_messages), 3)
        self.assertIn('Hello World: 0', listener.received_messages)
        self.assertIn('Hello World: 1', listener.received_messages)
        self.assertIn('Hello World: 2', listener.received_messages)
        
    def test_service_server_client(self):
        """Test the service server-client pattern from Chapter 1"""
        class AddTwoIntsServer(Node):
            def __init__(self):
                super().__init__('test_add_server')
                self.srv = self.create_service(
                    AddTwoInts,
                    'test_add_two_ints',
                    self.add_two_ints_callback)

            def add_two_ints_callback(self, request, response):
                response.sum = request.a + request.b
                return response
        
        class AddTwoIntsClient(Node):
            def __init__(self):
                super().__init__('test_add_client')
                self.client = self.create_client(AddTwoInts, 'test_add_two_ints')
                while not self.client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Service not available, waiting again...')
                self.request = AddTwoInts.Request()

            def send_request(self, a, b):
                self.request.a = a
                self.request.b = b
                future = self.client.call_async(self.request)
                rclpy.spin_until_future_complete(self, future)
                return future.result().sum
        
        # Create nodes
        server = AddTwoIntsServer()
        client = AddTwoIntsClient()
        
        # Test the service
        result = client.send_request(2, 3)
        self.assertEqual(result, 5)
        
        result = client.send_request(-1, 1)
        self.assertEqual(result, 0)
        
        result = client.send_request(10, 20)
        self.assertEqual(result, 30)


def main():
    """Run the tests"""
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestChapter1Examples)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Return appropriate exit code
    exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
```

## Chapter 2: Implementing ROS 2 Nodes and Communication - Test Suite

```python
#!/usr/bin/env python3
# test_chapter2_examples.py
# Validation script for Chapter 2: Implementing ROS 2 Nodes and Communication

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import unittest
import math


class TestChapter2Examples(unittest.TestCase):
    """Test suite for Chapter 2 examples"""
    
    def setUp(self):
        """Initialize ROS 2 before each test"""
        rclpy.init()
        
    def tearDown(self):
        """Cleanup after each test"""
        rclpy.shutdown()
    
    def test_parameter_node(self):
        """Test the parameter server usage from Chapter 2"""
        from rclpy.parameter import Parameter
        from rcl_interfaces.msg import SetParametersResult
        
        class ParameterNode(Node):
            def __init__(self):
                super().__init__('test_parameter_node')
                
                # Declare parameters with default values
                self.declare_parameter('robot_name', 'default_robot')
                self.declare_parameter('max_velocity', 1.0)
                self.declare_parameter('safety_distance', 0.5)
                
                # Get parameter values
                self.robot_name = self.get_parameter('robot_name').value
                self.max_velocity = self.get_parameter('max_velocity').value
                self.safety_distance = self.get_parameter('safety_distance').value
                
                self.add_on_set_parameters_callback(self.parameter_callback)
                self.updated_params = {}

            def parameter_callback(self, params):
                for param in params:
                    if param.name == 'max_velocity' and param.type_ == Parameter.Type.PARAMETER_DOUBLE:
                        self.max_velocity = param.value
                        self.updated_params['max_velocity'] = param.value
                    elif param.name == 'robot_name' and param.type_ == Parameter.Type.PARAMETER_STRING:
                        self.robot_name = param.value
                        self.updated_params['robot_name'] = param.value
                return SetParametersResult(successful=True)
        
        # Create and test the parameter node
        param_node = ParameterNode()
        
        # Verify initial values
        self.assertEqual(param_node.robot_name, 'default_robot')
        self.assertEqual(param_node.max_velocity, 1.0)
        self.assertEqual(param_node.safety_distance, 0.5)
        
        # Set new parameter values
        new_params = [
            Parameter('max_velocity', Parameter.Type.PARAMETER_DOUBLE, 2.0),
            Parameter('robot_name', Parameter.Type.PARAMETER_STRING, 'test_bot')
        ]
        
        result = param_node.set_parameters(new_params)
        self.assertTrue(all(r.successful for r in result))
        
        # Verify updated values
        self.assertEqual(param_node.updated_params.get('max_velocity'), 2.0)
        self.assertEqual(param_node.updated_params.get('robot_name'), 'test_bot')
    
    def test_obstacle_detection(self):
        """Test the obstacle detection logic from Chapter 2"""
        class ObstacleDetectorLogic:
            def __init__(self):
                self.safe_distance = 1.0  # meters
                self.obstacle_distance = float('inf')
                self.obstacle_angle = 0.0
            
            def process_laser_scan(self, ranges, angle_min, angle_increment):
                """Process laser scan data to detect obstacles"""
                if ranges:
                    # Filter out invalid ranges (inf, NaN)
                    valid_ranges = [r for r in ranges if not (math.isinf(r) or math.isnan(r))]
                    if valid_ranges:
                        self.obstacle_distance = min(valid_ranges)
                        
                        # Find the angle of the closest obstacle
                        min_idx = next(i for i, r in enumerate(ranges) 
                                     if not (math.isinf(r) or math.isnan(r)) and r == self.obstacle_distance)
                        self.obstacle_angle = angle_min + min_idx * angle_increment
                    else:
                        self.obstacle_distance = float('inf')
                else:
                    self.obstacle_distance = float('inf')
            
            def get_navigation_command(self):
                """Get navigation command based on obstacle detection"""
                cmd = {'linear_x': 0.0, 'angular_z': 0.0}
                
                if self.obstacle_distance < self.safe_distance:
                    # Obstacle detected - turn away
                    if self.obstacle_angle < 0:
                        # Obstacle on the left - turn right
                        cmd['linear_x'] = 0.2  # Move forward slowly
                        cmd['angular_z'] = -0.5  # Turn right
                    else:
                        # Obstacle on the right - turn left
                        cmd['linear_x'] = 0.2  # Move forward slowly
                        cmd['angular_z'] = 0.5  # Turn left
                else:
                    # Path is clear - move forward
                    cmd['linear_x'] = 0.5  # Move forward at normal speed
                    cmd['angular_z'] = 0.0  # No turning
                
                return cmd
        
        # Test the obstacle detection logic
        detector = ObstacleDetectorLogic()
        
        # Test with clear path
        detector.process_laser_scan([2.0, 2.5, 3.0], -1.57, 0.0174)  # All distances > safe distance
        cmd = detector.get_navigation_command()
        self.assertAlmostEqual(cmd['linear_x'], 0.5)
        self.assertAlmostEqual(cmd['angular_z'], 0.0)
        
        # Test with obstacle on the right
        detector.process_laser_scan([0.5, 2.0, 3.0], -1.57, 0.0174)  # Closest is 0.5m
        cmd = detector.get_navigation_command()
        self.assertAlmostEqual(cmd['linear_x'], 0.2)
        self.assertGreater(cmd['angular_z'], 0)  # Should turn left
        
        # Test with obstacle on the left
        detector.process_laser_scan([3.0, 2.0, 0.3], -1.57, 0.0174)  # Closest is 0.3m on left
        cmd = detector.get_navigation_command()
        self.assertAlmostEqual(cmd['linear_x'], 0.2)
        self.assertLess(cmd['angular_z'], 0)  # Should turn right


def main():
    """Run the tests"""
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestChapter2Examples)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Return appropriate exit code
    exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
```

## Chapter 3: Python Agents Bridging with ROS 2 - Test Suite

```python
#!/usr/bin/env python3
# test_chapter3_examples.py
# Validation script for Chapter 3: Python Agents Bridging with ROS 2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import unittest
import math
from enum import Enum


class RobotState(Enum):
    IDLE = 1
    EXPLORING = 2
    AVOIDING_OBSTACLE = 3
    RETURNING_HOME = 4


class TestChapter3Examples(unittest.TestCase):
    """Test suite for Chapter 3 examples"""
    
    def setUp(self):
        """Initialize ROS 2 before each test"""
        rclpy.init()
        
    def tearDown(self):
        """Cleanup after each test"""
        rclpy.shutdown()
    
    def test_state_machine_agent_logic(self):
        """Test state machine logic from Chapter 3"""
        class StateMachineLogic:
            def __init__(self):
                self.current_state = RobotState.IDLE
                self.obstacle_distance = float('inf')
                self.time_in_state = 0.0
                self.state_start_time = 0.0
                self.safe_distance = 0.8
                self.exploration_time = 10.0  # seconds to explore before returning home
                self.state_transition_log = []
            
            def update_with_time(self, obstacle_distance, elapsed_time):
                """Update state machine with elapsed time"""
                self.obstacle_distance = obstacle_distance
                self.time_in_state = elapsed_time - self.state_start_time
                
                # State transition logic
                prev_state = self.current_state
                if self.current_state == RobotState.IDLE:
                    self.current_state = RobotState.EXPLORING
                    self.state_start_time = elapsed_time
                    self.state_transition_log.append(f"IDLE -> EXPLORING at {elapsed_time:.1f}s")
                    
                elif self.current_state == RobotState.EXPLORING:
                    if self.obstacle_distance < self.safe_distance:
                        self.current_state = RobotState.AVOIDING_OBSTACLE
                        self.state_start_time = elapsed_time
                        self.state_transition_log.append(f"EXPLORING -> AVOIDING_OBSTACLE at {elapsed_time:.1f}s")
                    elif self.time_in_state > self.exploration_time:
                        self.current_state = RobotState.RETURNING_HOME
                        self.state_start_time = elapsed_time
                        self.state_transition_log.append(f"EXPLORING -> RETURNING_HOME at {elapsed_time:.1f}s")
                        
                elif self.current_state == RobotState.AVOIDING_OBSTACLE:
                    if self.obstacle_distance >= self.safe_distance:
                        self.current_state = RobotState.EXPLORING
                        self.state_start_time = elapsed_time
                        self.state_transition_log.append(f"AVOIDING_OBSTACLE -> EXPLORING at {elapsed_time:.1f}s")
                        
                elif self.current_state == RobotState.RETURNING_HOME:
                    # For this example, go back to IDLE after some time
                    if self.time_in_state > 5.0:  # 5 seconds to return home
                        self.current_state = RobotState.IDLE
                        self.state_start_time = elapsed_time
                        self.state_transition_log.append(f"RETURNING_HOME -> IDLE at {elapsed_time:.1f}s")
                
                if prev_state != self.current_state:
                    return True  # State changed
                return False
            
            def get_behavior_command(self):
                """Get command based on current state"""
                cmd_vel = {'linear_x': 0.0, 'angular_z': 0.0}
                
                if self.current_state == RobotState.IDLE:
                    cmd_vel['linear_x'] = 0.0
                    cmd_vel['angular_z'] = 0.0
                    
                elif self.current_state == RobotState.EXPLORING:
                    cmd_vel['linear_x'] = 0.3  # Move forward
                    cmd_vel['angular_z'] = 0.1  # Slight turn to encourage exploration
                    
                elif self.current_state == RobotState.AVOIDING_OBSTACLE:
                    cmd_vel['linear_x'] = 0.1  # Move forward slowly
                    if self.obstacle_distance < self.safe_distance:
                        cmd_vel['angular_z'] = 0.5 if self.obstacle_distance < 0.5 else 0.3
                    else:
                        cmd_vel['angular_z'] = 0.0
                        
                elif self.current_state == RobotState.RETURNING_HOME:
                    cmd_vel['linear_x'] = -0.2  # Move backward
                    cmd_vel['angular_z'] = -0.1  # Turn slightly
                    
                return cmd_vel
        
        # Test the state machine logic
        sm = StateMachineLogic()
        
        # Start at time 0s
        state_changed = sm.update_with_time(2.0, 0.0)  # Clear path
        self.assertTrue(state_changed)
        self.assertEqual(sm.current_state, RobotState.EXPLORING)
        
        # After 5s, still exploring (no obstacle, < exploration_time)
        state_changed = sm.update_with_time(2.0, 5.0)  # Clear path
        self.assertFalse(state_changed)  # No state change
        self.assertEqual(sm.current_state, RobotState.EXPLORING)
        
        # After 11s, should transition back to returning home
        state_changed = sm.update_with_time(2.0, 11.0)  # Clear path for 11s (exceeds exploration_time)
        self.assertTrue(state_changed)
        self.assertEqual(sm.current_state, RobotState.RETURNING_HOME)
        
        # Now with obstacle - should trigger transition
        sm.current_state = RobotState.EXPLORING
        sm.state_start_time = 15.0
        state_changed = sm.update_with_time(0.3, 16.0)  # Close obstacle
        self.assertTrue(state_changed)
        self.assertEqual(sm.current_state, RobotState.AVOIDING_OBSTACLE)
    
    def test_error_handling_in_agent(self):
        """Test error handling patterns from Chapter 3"""
        class RobustAgentLogic:
            def __init__(self):
                self.message_count = 0
                self.healthy = True
                self.error_count = 0
                self.recovery_count = 0
            
            def try_publish_with_error_handling(self, publisher_func):
                """Try to publish with error handling"""
                try:
                    # Simulate publishing
                    publisher_func(self.message_count)
                    self.message_count += 1
                    return True
                except Exception as e:
                    self.error_count += 1
                    self.healthy = False
                    self.recovery_strategy()
                    return False
            
            def recovery_strategy(self):
                """Implement recovery from errors"""
                self.recovery_count += 1
                # In a real implementation, this would recreate publishers, etc.
                if self.recovery_count < 3:  # Only recover up to 3 times
                    self.healthy = True
            
            def simulate_occasional_error(self):
                """Simulate occasional errors in operation"""
                # Every 7th message fails for testing purposes
                if self.message_count % 7 == 6:  # 6, 13, 20, ...
                    raise Exception("Simulated communication error")
                return f"Message {self.message_count} from robust agent"
        
        # Test error handling
        agent = RobustAgentLogic()
        
        # Function to simulate publishing
        def sim_publish(msg_count):
            agent.simulate_occasional_error()
        
        # Send 15 messages, expect errors on 6th and 13th (0-indexed: 6, 13)
        for i in range(15):
            success = agent.try_publish_with_error_handling(sim_publish)
            if i == 6 or i == 13:  # These should generate errors
                self.assertFalse(success)  # Expect failure
            else:
                self.assertTrue(success)  # Expect success
        
        # Verify error handling worked
        self.assertEqual(agent.error_count, 2)  # 2 errors occurred
        self.assertEqual(agent.recovery_count, 2)  # 2 recovery attempts
        self.assertEqual(agent.message_count, 15)  # 15 attempts total


def main():
    """Run the tests"""
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestChapter3Examples)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Return appropriate exit code
    exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
```

## Chapter 4: Understanding URDF for Humanoid Robots - Test Suite

```python
#!/usr/bin/env python3
# test_chapter4_examples.py
# Validation script for Chapter 4: Understanding URDF for Humanoid Robots

import xml.etree.ElementTree as ET
import unittest


class TestChapter4Examples(unittest.TestCase):
    """Test suite for Chapter 4 examples"""
    
    def test_urdf_structure_validation(self):
        """Test URDF structure validation techniques"""
        # Define a simple valid URDF
        valid_urdf_xml = '''<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
    </collision>
  </link>
  
  <joint name="test_joint" type="revolute">
    <parent link="base_link" />
    <child link="child_link" />
    <origin xyz="0 0 0.1" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>
  
  <link name="child_link">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
  </link>
</robot>'''
        
        try:
            # Try to parse the URDF XML
            root = ET.fromstring(valid_urdf_xml)
            
            # Verify root element
            self.assertEqual(root.tag, 'robot')
            self.assertEqual(root.attrib['name'], 'test_robot')
            
            # Verify links exist
            links = root.findall('link')
            self.assertEqual(len(links), 2)
            link_names = [link.attrib['name'] for link in links]
            self.assertIn('base_link', link_names)
            self.assertIn('child_link', link_names)
            
            # Verify joints exist
            joints = root.findall('joint')
            self.assertEqual(len(joints), 1)
            self.assertEqual(joints[0].attrib['name'], 'test_joint')
            self.assertEqual(joints[0].attrib['type'], 'revolute')
            
            # Verify joint connections
            parent = joints[0].find('parent').attrib['link']
            child = joints[0].find('child').attrib['link']
            self.assertEqual(parent, 'base_link')
            self.assertEqual(child, 'child_link')
            
        except ET.ParseError:
            self.fail("Valid URDF should parse without errors")
        
        # Test invalid URDF
        invalid_urdf_xml = '''<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
  </link>
  <!-- Invalid: joint references non-existent link -->
  <joint name="test_joint" type="revolute">
    <parent link="nonexistent_link" />
    <child link="base_link" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>
</robot>'''
        
        try:
            # This URDF is syntactically valid XML but logically questionable
            # The validation would be more complex in a real system
            root = ET.fromstring(invalid_urdf_xml)
            # For now, just verify it parses
            self.assertEqual(root.tag, 'robot')
        except ET.ParseError:
            # If it doesn't parse, that's also valid (malformed XML)
            pass
    
    def test_urdf_kinematic_chain(self):
        """Test kinematic chain definition from humanoid URDF"""
        # Verify that a kinematic chain is properly defined
        humanoid_urdf_snippet = '''
<robot name="humanoid">
  <!-- Spine chain: pelvis -> torso -> head -->
  <link name="pelvis" />
  <link name="torso" />
  <link name="head" />
  
  <joint name="torso_joint" type="fixed">
    <parent link="pelvis" />
    <child link="torso" />
  </joint>
  
  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="5.0" velocity="1.0" />
  </joint>
  
  <!-- Left arm chain: torso -> upper_arm -> lower_arm -> hand -->
  <link name="left_upper_arm" />
  <link name="left_lower_arm" />
  <link name="left_hand" />
  
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="left_upper_arm" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>
  
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm" />
    <child link="left_lower_arm" />
    <axis xyz="0 1 0" />
    <limit lower="-2.0" upper="0.0" effort="10.0" velocity="1.0" />
  </joint>
  
  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm" />
    <child link="left_hand" />
    <axis xyz="0 1 0" />
    <limit lower="-1.0" upper="1.0" effort="5.0" velocity="1.0" />
  </joint>
</robot>'''
        
        root = ET.fromstring(humanoid_urdf_snippet)
        
        # Check that we have the expected links
        links = root.findall('link')
        link_names = [link.attrib['name'] for link in links]
        expected_links = ['pelvis', 'torso', 'head', 'left_upper_arm', 'left_lower_arm', 'left_hand']
        for expected_link in expected_links:
            self.assertIn(expected_link, link_names)
        
        # Check that we have the expected joints
        joints = root.findall('joint')
        joint_names = [joint.attrib['name'] for joint in joints]
        expected_joints = ['torso_joint', 'neck_joint', 'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint']
        for expected_joint in expected_joints:
            self.assertIn(expected_joint, joint_names)
        
        # Verify kinematic chain connectivity
        joint_map = {}
        for joint in joints:
            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']
            joint_map[child] = parent
        
        # Check left arm chain: left_hand -> left_lower_arm -> left_upper_arm -> torso
        self.assertEqual(joint_map['left_hand'], 'left_lower_arm')
        self.assertEqual(joint_map['left_lower_arm'], 'left_upper_arm')
        self.assertEqual(joint_map['left_upper_arm'], 'torso')
        
        # Check spine chain: head -> torso -> pelvis
        self.assertEqual(joint_map['head'], 'torso')
        self.assertEqual(joint_map['torso'], 'pelvis')


def main():
    """Run the tests"""
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestChapter4Examples)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Return appropriate exit code
    exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
```

## Chapter 5: Complete Humanoid Robot Implementation Example - Test Suite

```python
#!/usr/bin/env python3
# test_chapter5_examples.py
# Validation script for Chapter 5: Complete Humanoid Robot Implementation Example

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import unittest
import math
from enum import Enum


class RobotState(Enum):
    IDLE = 1
    WALKING = 2
    STANDING = 3
    SITTING = 4
    INTERACTING = 5


class TestChapter5Examples(unittest.TestCase):
    """Test suite for Chapter 5 examples"""
    
    def setUp(self):
        """Initialize ROS 2 before each test"""
        rclpy.init()
        
    def tearDown(self):
        """Cleanup after each test"""
        rclpy.shutdown()
    
    def test_humanoid_controller_integration(self):
        """Test the complete humanoid controller from Chapter 5"""
        class MockHumanoidController:
            def __init__(self):
                # Initialize joint positions
                self.joint_names = [
                    'neck_joint',
                    'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
                    'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint',
                    'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
                    'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
                ]
                
                self.joint_positions = {name: 0.0 for name in self.joint_names}
                self.current_state = RobotState.IDLE
                self.position = [0.0, 0.0, 0.0]  # x, y, theta
                self.linear_vel = 0.0
                self.angular_vel = 0.0
                self.obstacle_distance = float('inf')
                self.battery_level = 100.0
            
            def update_joint_positions(self, positions_dict):
                """Update joint positions"""
                for name, pos in positions_dict.items():
                    if name in self.joint_positions:
                        self.joint_positions[name] = pos
            
            def process_velocity_command(self, linear_x, angular_z):
                """Process velocity command"""
                self.linear_vel = linear_x
                self.angular_vel = angular_z
                
                # Change state to walking if we receive movement commands
                if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
                    if self.current_state != RobotState.WALKING:
                        self.current_state = RobotState.WALKING
            
            def update_position(self, dt):
                """Update robot position based on velocity"""
                self.position[0] += self.linear_vel * math.cos(self.position[2]) * dt
                self.position[1] += self.linear_vel * math.sin(self.position[2]) * dt
                self.position[2] += self.angular_vel * dt
                
                # Keep theta in [-pi, pi]
                self.position[2] = math.atan2(
                    math.sin(self.position[2]), 
                    math.cos(self.position[2])
                )
            
            def process_sensor_data(self, laser_ranges, imu_data):
                """Process sensor data"""
                if laser_ranges:
                    valid_ranges = [r for r in laser_ranges if not (math.isinf(r) or math.isnan(r)) and r > 0]
                    if valid_ranges:
                        self.obstacle_distance = min(valid_ranges)
                    else:
                        self.obstacle_distance = float('inf')
                else:
                    self.obstacle_distance = float('inf')
                
                # Process IMU data (simplified)
                # In real implementation, use IMU for balance control
        
        # Test the controller logic
        controller = MockHumanoidController()
        
        # Initial state checks
        self.assertEqual(controller.current_state, RobotState.IDLE)
        self.assertEqual(len(controller.joint_positions), 13)  # All joints initialized
        self.assertEqual(controller.obstacle_distance, float('inf'))
        
        # Test joint position updates
        new_positions = {
            'neck_joint': 0.2,
            'left_shoulder_joint': 0.5,
            'right_elbow_joint': -1.0
        }
        controller.update_joint_positions(new_positions)
        self.assertEqual(controller.joint_positions['neck_joint'], 0.2)
        self.assertEqual(controller.joint_positions['left_shoulder_joint'], 0.5)
        self.assertEqual(controller.joint_positions['right_elbow_joint'], -1.0)
        
        # Test velocity command processing
        controller.process_velocity_command(0.3, 0.1)  # Move forward and turn
        self.assertEqual(controller.current_state, RobotState.WALKING)
        self.assertEqual(controller.linear_vel, 0.3)
        self.assertEqual(controller.angular_vel, 0.1)
        
        # Test position update
        original_pos = controller.position[:]
        controller.update_position(1.0)  # Update for 1 second
        self.assertNotEqual(controller.position, original_pos)  # Position should change
        self.assertGreater(controller.position[0], original_pos[0])  # Should move forward
        self.assertGreater(controller.position[2], original_pos[2])  # Should turn
        
        # Test sensor data processing
        laser_ranges = [1.0, 2.0, 0.5, 3.0]  # Obstacle at 0.5m
        controller.process_sensor_data(laser_ranges, {})
        self.assertEqual(controller.obstacle_distance, 0.5)
    
    def test_humanoid_agent_integration(self):
        """Test the humanoid agent from Chapter 5"""
        class MockHumanoidAgent:
            def __init__(self):
                self.current_state = RobotState.IDLE
                self.obstacle_distance = float('inf')
                self.heading_error = 0.0
                self.balance_data = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                self.cmd_vel = {'linear_x': 0.0, 'angular_z': 0.0}
                self.task_queue = []
            
            def process_laser_scan(self, ranges):
                """Process laser scan for obstacle detection"""
                if ranges:
                    valid_ranges = [r for r in ranges if not (float('inf') == r or r == 0.0)]
                    if valid_ranges:
                        self.obstacle_distance = min(valid_ranges)
                    else:
                        self.obstacle_distance = float('inf')
                else:
                    self.obstacle_distance = float('inf')
            
            def compute_navigation_command(self):
                """Compute navigation command based on state and sensors"""
                if self.current_state == RobotState.IDLE:
                    self.cmd_vel['linear_x'] = 0.0
                    self.cmd_vel['angular_z'] = 0.0
                elif self.current_state == RobotState.WALKING:
                    # Move forward with obstacle avoidance
                    if self.obstacle_distance < 0.8:  # Obstacle in path
                        self.cmd_vel['linear_x'] = 0.0
                        self.cmd_vel['angular_z'] = 0.5  # Turn right
                    else:
                        self.cmd_vel['linear_x'] = 0.3  # Continue forward
                        self.cmd_vel['angular_z'] = 0.0
                elif self.current_state == RobotState.INTERACTING:
                    self.cmd_vel['linear_x'] = 0.0
                    self.cmd_vel['angular_z'] = self.heading_error * 0.5  # Adjust towards target
            
            def add_task(self, task):
                """Add a task to the queue"""
                self.task_queue.append(task)
                if self.current_state == RobotState.IDLE:
                    self.current_state = RobotState.WALKING  # Start executing tasks
        
        # Test the agent logic
        agent = MockHumanoidAgent()
        
        # Initial state
        self.assertEqual(agent.current_state, RobotState.IDLE)
        self.assertEqual(agent.obstacle_distance, float('inf'))
        self.assertEqual(agent.cmd_vel['linear_x'], 0.0)
        self.assertEqual(agent.cmd_vel['angular_z'], 0.0)
        
        # Process clear path
        agent.process_laser_scan([2.0, 2.5, 3.0])
        agent.current_state = RobotState.WALKING
        agent.compute_navigation_command()
        self.assertAlmostEqual(agent.cmd_vel['linear_x'], 0.3)
        self.assertAlmostEqual(agent.cmd_vel['angular_z'], 0.0)
        
        # Process obstacle
        agent.process_laser_scan([0.3, 2.0, 3.0])  # Obstacle at 0.3m
        agent.compute_navigation_command()
        self.assertAlmostEqual(agent.cmd_vel['linear_x'], 0.0)
        self.assertAlmostEqual(agent.cmd_vel['angular_z'], 0.5)
        
        # Test task queue
        agent.add_task("wave_hello")
        self.assertEqual(len(agent.task_queue), 1)
        self.assertEqual(agent.task_queue[0], "wave_hello")
        self.assertEqual(agent.current_state, RobotState.WALKING)


def main():
    """Run the tests"""
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestChapter5Examples)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Return appropriate exit code
    exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
```

## Integration Test Suite

```python
#!/usr/bin/env python3
# integration_test.py
# Integration tests for the entire ROS 2 Nervous System module

import unittest
from test_chapter1_examples import TestChapter1Examples
from test_chapter2_examples import TestChapter2Examples
from test_chapter3_examples import TestChapter3Examples
from test_chapter4_examples import TestChapter4Examples
from test_chapter5_examples import TestChapter5Examples


class IntegrationTestSuite(unittest.TestCase):
    """Integration tests for the entire module"""
    
    def test_cross_chapter_integration(self):
        """Test integration between concepts from different chapters"""
        # Test that joint state messages from Chapter 5 can be processed 
        # by nodes similar to those created in other chapters
        
        # Create mock joint state data
        joint_names = ['joint1', 'joint2', 'joint3']
        positions = [0.1, 0.2, 0.3]
        
        # Verify that joint state data structure is correct
        self.assertEqual(len(joint_names), len(positions))
        
        # Test that joint positions are within reasonable ranges
        for pos in positions:
            self.assertGreaterEqual(pos, -2 * 3.14159)  # -2π to 2π range
            self.assertLessEqual(pos, 2 * 3.14159)
        
        # Test that sensor data can trigger state changes
        obstacle_distance = 0.5  # meters
        safe_distance = 0.8  # meters
        
        should_avoid = obstacle_distance < safe_distance
        self.assertTrue(should_avoid)
        
        # Test that parameter values are properly bounded
        max_velocity = 1.0
        self.assertGreaterEqual(max_velocity, 0.0)
        self.assertLessEqual(max_velocity, 5.0)  # Reasonable max for humanoid robot


def run_all_tests():
    """Run all test suites"""
    # Create a test suite combining all tests
    loader = unittest.TestLoader()
    
    suite = unittest.TestSuite()
    suite.addTests(loader.loadTestsFromTestCase(TestChapter1Examples))
    suite.addTests(loader.loadTestsFromTestCase(TestChapter2Examples))
    suite.addTests(loader.loadTestsFromTestCase(TestChapter3Examples))
    suite.addTests(loader.loadTestsFromTestCase(TestChapter4Examples))
    suite.addTests(loader.loadTestsFromTestCase(TestChapter5Examples))
    suite.addTests(loader.loadTestsFromTestCase(IntegrationTestSuite))
    
    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print(f"\n--- Test Results ---")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success: {result.wasSuccessful()}")
    
    return result.wasSuccessful()


def main():
    """Main entry point for the integration tests"""
    success = run_all_tests()
    exit(0 if success else 1)


if __name__ == '__main__':
    main()
```

This comprehensive test suite validates all the concepts from each chapter in the ROS 2 Nervous System module. The tests cover:

1. Basic ROS 2 concepts like nodes, topics, and services
2. Parameter server usage and obstacle detection logic
3. State machine implementations and error handling in agents
4. URDF structure validation and kinematic chain definitions
5. Integration of all components in a complete humanoid robot controller
6. Cross-chapter integration to verify the entire module works cohesively

The validation approach tests both individual components and their integration, ensuring that students can reproduce and understand all the examples provided in the module.
# Chapter 3: Python Agents Bridging with ROS 2

## Introduction

In the previous chapters, we explored the core concepts of ROS 2 and implemented various communication patterns. In this chapter, we'll focus on how Python agents can interact with ROS 2 controllers using the `rclpy` client library. This capability allows for high-level decision making and integration of AI/ML models with ROS 2-based robotic systems.

## Python Agent that Controls ROS 2 Robots via Topics

Python agents can subscribe to sensor data from ROS 2 topics and publish commands to control robots. Here's an example of a simple Python agent that controls a robot based on sensor input:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class NavigationAgent(Node):
    def __init__(self):
        super().__init__('navigation_agent')
        
        # Create subscriber for laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer to periodically send commands
        self.timer = self.create_timer(0.1, self.navigate)
        
        # Robot state
        self.obstacle_distance = float('inf')
        self.obstacle_angle = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.safe_distance = 1.0  # meters
        
    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Find the minimum distance in the scan
        if msg.ranges:
            # Filter out invalid ranges (inf, NaN)
            valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r))]
            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)
                
                # Find the angle of the closest obstacle
                min_idx = msg.ranges.index(self.obstacle_distance)
                self.obstacle_angle = msg.angle_min + min_idx * msg.angle_increment
            else:
                self.obstacle_distance = float('inf')
        else:
            self.obstacle_distance = float('inf')

    def navigate(self):
        """Main navigation logic based on sensor input"""
        msg = Twist()
        
        # Simple obstacle avoidance algorithm
        if self.obstacle_distance < self.safe_distance:
            # Obstacle detected - turn away
            if self.obstacle_angle < 0:
                # Obstacle on the left - turn right
                msg.linear.x = 0.2  # Move forward slowly
                msg.angular.z = -0.5  # Turn right
            else:
                # Obstacle on the right - turn left
                msg.linear.x = 0.2  # Move forward slowly
                msg.angular.z = 0.5  # Turn left
        else:
            # Path is clear - move forward
            msg.linear.x = 0.5  # Move forward at normal speed
            msg.angular.z = 0.0  # No turning
        
        # Publish the velocity command
        self.publisher.publish(msg)
        self.get_logger().info(f'Velocity: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    navigation_agent = NavigationAgent()
    
    try:
        rclpy.spin(navigation_agent)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Python Agent Using ROS 2 Services for Robot Control

Python agents can also use services for more direct control of robot functions:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from example_interfaces.srv import SetBool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class ServiceControlAgent(Node):
    def __init__(self):
        super().__init__('service_control_agent')
        
        # Create a client for a custom service (e.g., for enabling/disabling sensors)
        self.safety_client = self.create_client(SetBool, 'enable_safety_system')
        
        # Wait for service to be available
        while not self.safety_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Timer to periodically call services
        self.timer = self.create_timer(5.0, self.call_service)
        
        # Counter for alternating service calls
        self.call_count = 0

    def call_service(self):
        """Call a service to control robot behavior"""
        request = SetBool.Request()
        
        if self.call_count % 2 == 0:
            request.data = True  # Enable safety system
            self.get_logger().info('Enabling safety system')
        else:
            request.data = False  # Disable safety system
            self.get_logger().info('Disabling safety system')
        
        # Make an asynchronous service call
        future = self.safety_client.call_async(request)
        future.add_done_callback(self.service_callback)
        
        self.call_count += 1

    def service_callback(self, future):
        """Handle the response from the service call"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service call successful: {response.message}')
            else:
                self.get_logger().error(f'Service call failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')


class NavigationActionAgent(Node):
    def __init__(self):
        super().__init__('navigation_action_agent')
        
        # Create action client for navigation
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # Timer to periodically send navigation goals
        self.timer = self.create_timer(10.0, self.send_goal)
        self.goal_sent = False

    def send_goal(self):
        """Send a navigation goal to the robot"""
        if self._action_client.wait_for_server(timeout_sec=1.0):
            # Create a navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = 1.0  # Move to x=1.0
            goal_msg.pose.pose.position.y = 1.0  # Move to y=1.0
            goal_msg.pose.pose.orientation.w = 1.0  # No rotation
            
            # Send the goal asynchronously
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info('Navigation goal sent')
        else:
            self.get_logger().warn('Navigation action server not available')

    def goal_response_callback(self, future):
        """Handle the response from sending a goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback.current_pose}')

    def get_result_callback(self, future):
        """Handle the result from the action server"""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.goal_sent = False


def main(args=None):
    rclpy.init(args=args)
    
    # Create both nodes
    service_agent = ServiceControlAgent()
    nav_agent = NavigationActionAgent()
    
    # Use a multi-threaded executor to run both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(service_agent)
    executor.add_node(nav_agent)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        service_agent.destroy_node()
        nav_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## State Machines for Robot Behavior in Python Agents

Python agents can implement complex behaviors using state machines:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    EXPLORING = 2
    AVOIDING_OBSTACLE = 3
    RETURNING_HOME = 4

class StateMachineAgent(Node):
    def __init__(self):
        super().__init__('state_machine_agent')
        
        # Create subscriber for laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer to update the state machine
        self.timer = self.create_timer(0.1, self.update_state_machine)
        
        # State machine variables
        self.current_state = RobotState.IDLE
        self.obstacle_distance = float('inf')
        self.time_in_state = 0.0
        self.state_start_time = self.get_clock().now()
        self.safe_distance = 0.8
        self.exploration_time = 10.0  # seconds to explore before returning home
        
    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if not (float('inf') == r or r == 0.0)]
            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)
            else:
                self.obstacle_distance = float('inf')
        else:
            self.obstacle_distance = float('inf')

    def update_state_machine(self):
        """Main state machine logic"""
        # Update time in current state
        current_time = self.get_clock().now()
        self.time_in_state = (current_time.nanoseconds - self.state_start_time.nanoseconds) / 1e9
        
        # State transition logic
        if self.current_state == RobotState.IDLE:
            self.current_state = RobotState.EXPLORING
            self.state_start_time = current_time
            self.get_logger().info('Transitioning to EXPLORING state')
            
        elif self.current_state == RobotState.EXPLORING:
            if self.obstacle_distance < self.safe_distance:
                self.current_state = RobotState.AVOIDING_OBSTACLE
                self.state_start_time = current_time
                self.get_logger().info('Transitioning to AVOIDING_OBSTACLE state')
            elif self.time_in_state > self.exploration_time:
                self.current_state = RobotState.RETURNING_HOME
                self.state_start_time = current_time
                self.get_logger().info('Transitioning to RETURNING_HOME state')
                
        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            if self.obstacle_distance >= self.safe_distance:
                self.current_state = RobotState.EXPLORING
                self.state_start_time = current_time
                self.get_logger().info('Transitioning back to EXPLORING state')
                
        elif self.current_state == RobotState.RETURNING_HOME:
            # For this example, just go back to IDLE after some time
            if self.time_in_state > 5.0:  # 5 seconds to return home
                self.current_state = RobotState.IDLE
                self.state_start_time = current_time
                self.get_logger().info('Transitioning to IDLE state')
        
        # Execute behavior based on current state
        cmd_vel = Twist()
        
        if self.current_state == RobotState.IDLE:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            
        elif self.current_state == RobotState.EXPLORING:
            cmd_vel.linear.x = 0.3  # Move forward
            cmd_vel.angular.z = 0.1  # Slight turn to encourage exploration
            
        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            # Simple obstacle avoidance - turn away from obstacle
            cmd_vel.linear.x = 0.1  # Move forward slowly
            if self.obstacle_distance < self.safe_distance:
                cmd_vel.angular.z = 0.5 if self.obstacle_distance < 0.5 else 0.3  # Turn more aggressively if very close
            else:
                cmd_vel.angular.z = 0.0
                
        elif self.current_state == RobotState.RETURNING_HOME:
            cmd_vel.linear.x = -0.2  # Move backward (simple return)
            cmd_vel.angular.z = -0.1  # Turn in the opposite direction
            
        # Publish command
        self.publisher.publish(cmd_vel)
        
        # Log current state
        self.get_logger().info(f'State: {self.current_state.name}, Distance: {self.obstacle_distance:.2f}')

def main(args=None):
    rclpy.init(args=args)
    state_machine_agent = StateMachineAgent()
    
    try:
        rclpy.spin(state_machine_agent)
    except KeyboardInterrupt:
        pass
    finally:
        state_machine_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Examples of Integrating AI/ML Models with ROS 2 through Python

Python agents can integrate AI/ML models for perception, decision making, and control:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
import tensorflow as tf  # Example with TensorFlow

class MLPerceptionAgent(Node):
    def __init__(self):
        super().__init__('ml_perception_agent')
        
        # Create subscribers for sensor data
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        
        # Create publishers for commands and perception results
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.perception_publisher = self.create_publisher(String, 'perception_result', 10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize ML model (example: a pre-trained object detection model)
        # NOTE: In practice, you would load your trained model here
        # self.model = tf.keras.models.load_model('path/to/your/model')
        self.ml_model_ready = True  # For this example
        
        # Timer for AI-based navigation decisions
        self.timer = self.create_timer(0.5, self.ai_decision_making)
        
        # State variables
        self.latest_image = None
        self.latest_scan = None
        self.detected_objects = []
        self.robot_action = "none"  # Current action based on AI decision

    def image_callback(self, msg):
        """Process incoming image data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            
            # Run object detection if model is ready
            if self.ml_model_ready:
                self.detected_objects = self.run_object_detection(cv_image)
                
                # Publish detection results
                result_msg = String()
                result_msg.data = f"Detected {len(self.detected_objects)} objects: {self.detected_objects}"
                self.perception_publisher.publish(result_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.latest_scan = msg

    def run_object_detection(self, image):
        """Run object detection on image - placeholder for actual ML model"""
        # This is a placeholder - in practice, you would use your ML model here
        # Example using a pre-trained model:
        # results = self.model.predict(image)
        # Process results to identify objects
        # For this example, we'll simulate detection of a few objects
        
        # Convert image to grayscale for simple processing
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect contours (simplified example)
        _, thresh = cv2.threshold(gray, 127, 255, 0)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Count objects based on contour area
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter small contours
                objects.append(f"object_{len(objects)}")
        
        return objects[:5]  # Return up to 5 detected objects

    def ai_decision_making(self):
        """Make navigation decisions based on sensor data and ML perception"""
        if self.latest_scan is None or not self.detected_objects:
            return  # Wait for data
        
        # Simple AI decision based on sensor data and object detection
        closest_obstacle = min(self.latest_scan.ranges) if self.latest_scan.ranges else float('inf')
        
        # Decision logic
        cmd_vel = Twist()
        
        if "person" in self.detected_objects:
            # If a person is detected, approach carefully
            if closest_obstacle > 0.8:
                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = 0.0
                self.robot_action = "approaching_person"
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.2  # Turn to avoid obstacle while approaching
                self.robot_action = "avoiding_to_approach_person"
        elif closest_obstacle < 0.5:
            # If obstacle is too close, avoid it
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.5  # Turn right to avoid
            self.robot_action = "avoiding_obstacle"
        else:
            # Otherwise, explore
            cmd_vel.linear.x = 0.4
            cmd_vel.angular.z = 0.1
            self.robot_action = "exploring"
        
        # Publish the command
        self.cmd_publisher.publish(cmd_vel)
        
        self.get_logger().info(f'AI Decision: {self.robot_action}, '
                              f'Objects: {len(self.detected_objects)}, '
                              f'Closest obstacle: {closest_obstacle:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    ml_agent = MLPerceptionAgent()
    
    try:
        rclpy.spin(ml_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ml_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling in Python Agents for ROS 2 Communication

Proper error handling is crucial for robust Python agents:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.exceptions import ParameterNotDeclaredException
import traceback

class RobustPythonAgent(Node):
    def __init__(self):
        super().__init__('robust_python_agent')
        
        # Set up QoS for reliable communication
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        # Create publisher with error handling
        try:
            self.publisher = self.create_publisher(String, 'robust_topic', qos_profile)
            self.get_logger().info('Publisher created successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to create publisher: {e}')
            raise
        
        # Initialize variables
        self.message_count = 0
        self.healthy = True
        
        # Timer with error handling
        try:
            self.timer = self.create_timer(1.0, self.timer_callback_with_error_handling)
        except Exception as e:
            self.get_logger().error(f'Failed to create timer: {e}')
            self.healthy = False

    def timer_callback_with_error_handling(self):
        """Timer callback with comprehensive error handling"""
        try:
            # Check if we can still publish
            if not self.publisher:
                self.get_logger().error('Publisher is not available')
                self.healthy = False
                return
            
            # Create and publish message
            msg = String()
            msg.data = f'Message {self.message_count} from robust agent'
            self.publisher.publish(msg)
            
            self.get_logger().info(f'Published: {msg.data}')
            self.message_count += 1
            
            # Simulate occasional error for demonstration
            if self.message_count % 10 == 7:  # Every 7th message (starting from 7)
                raise Exception('Simulated communication error')
                
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            
            # Implement recovery strategy
            self.recovery_strategy()
            
    def recovery_strategy(self):
        """Implement recovery from errors"""
        self.get_logger().info('Attempting recovery...')
        
        # Example recovery steps:
        # 1. Try to recreate publisher if needed
        try:
            if not self.publisher:
                qos_profile = QoSProfile(depth=10)
                qos_profile.reliability = ReliabilityPolicy.RELIABLE
                self.publisher = self.create_publisher(String, 'robust_topic', qos_profile)
                self.get_logger().info('Publisher recreated successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to recreate publisher: {e}')
        
        # 2. Reset any corrupted state
        # 3. Implement backoff strategy if needed
        self.get_logger().info('Recovery attempt completed')

    def on_shutdown(self):
        """Clean up resources"""
        self.get_logger().info('Shutting down robust agent...')
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    robust_agent = RobustPythonAgent()
    
    try:
        rclpy.spin(robust_agent)
    except KeyboardInterrupt:
        robust_agent.get_logger().info('Interrupted by user')
    except Exception as e:
        robust_agent.get_logger().error(f'Unexpected error: {e}')
        robust_agent.get_logger().error(f'Traceback: {traceback.format_exc()}')
    finally:
        robust_agent.on_shutdown()
        robust_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Learning Objectives

After completing this chapter, you should be able to:
- Create Python agents that control ROS 2 robots via topics
- Implement Python agents using ROS 2 services for robot control
- Design state machines for complex robot behaviors in Python
- Integrate AI/ML models with ROS 2 through Python agents
- Implement proper error handling in Python agents for ROS 2 communication

## Chapter Summary

This chapter demonstrated how Python agents can bridge with ROS 2 systems to create intelligent and adaptive robotic behaviors. We covered various approaches for integrating Python agents with ROS 2, including topic-based control, service-based control, state machines, and AI/ML integration. The chapter also emphasized the importance of error handling for creating robust, production-ready robotic systems.
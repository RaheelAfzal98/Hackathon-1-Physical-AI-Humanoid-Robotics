# Chapter 2: Implementing ROS 2 Nodes and Communication

## Introduction

In Chapter 1, we introduced the fundamental concepts of ROS 2: Nodes, Topics, and Services. In this chapter, we'll dive deeper into implementing these concepts with practical examples. You'll learn how to create more complex nodes and establish communication through Topics and Services.

## More Complex Publisher-Subscriber Examples with Custom Messages

In Chapter 1, we saw a basic publisher-subscriber example using the built-in `std_msgs/String` message type. Now, let's create custom messages for more complex data structures.

### Creating Custom Messages

To create a custom message, you need to define a `.msg` file in your package's `msg` directory. Let's create a custom message for robot position data:

```
# In msg/RobotPosition.msg
float64 x
float64 y
float64 theta
string robot_name
```

### Publisher with Custom Message

```python
import rclpy
from rclpy.node import Node
from your_package_msgs.msg import RobotPosition  # Your custom message

class RobotPositionPublisher(Node):
    def __init__(self):
        super().__init__('robot_position_publisher')
        self.publisher_ = self.create_publisher(RobotPosition, 'robot_position', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = RobotPosition()
        msg.x = 1.0 + self.i * 0.1
        msg.y = 2.0 + self.i * 0.05
        msg.theta = 0.0
        msg.robot_name = 'turtlebot1'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={msg.x}, y={msg.y}, robot={msg.robot_name}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    robot_position_publisher = RobotPositionPublisher()
    
    try:
        rclpy.spin(robot_position_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_position_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber with Custom Message

```python
import rclpy
from rclpy.node import Node
from your_package_msgs.msg import RobotPosition

class RobotPositionSubscriber(Node):
    def __init__(self):
        super().__init__('robot_position_subscriber')
        self.subscription = self.create_subscription(
            RobotPosition,
            'robot_position',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: x={msg.x}, y={msg.y}, robot={msg.robot_name}')

def main(args=None):
    rclpy.init(args=args)
    robot_position_subscriber = RobotPositionSubscriber()
    
    try:
        rclpy.spin(robot_position_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        robot_position_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multi-Node Communication Example with Topics

Let's create a more complex example with multiple nodes communicating with each other:

### Node 1: Sensor Simulator

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        # Fill in laser scan parameters
        msg.angle_min = -1.57  # -90 degrees in radians
        msg.angle_max = 1.57   # 90 degrees in radians
        msg.angle_increment = 0.0174  # 1 degree in radians
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.1
        msg.range_max = 10.0
        
        # Generate random range data
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [random.uniform(0.5, 5.0) for _ in range(num_readings)]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published laser scan with {num_readings} readings')

def main(args=None):
    rclpy.init(args=args)
    sensor_simulator = SensorSimulator()
    
    try:
        rclpy.spin(sensor_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_simulator.destroy_node()
        rclpy.shutdown()
```

### Node 2: Obstacle Detector

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'obstacle_status', 10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Detect if there's an obstacle within 1 meter
        min_distance = min(msg.ranges) if msg.ranges else float('inf')
        
        obstacle_msg = String()
        if min_distance < 1.0:
            obstacle_msg.data = f'OBSTACLE_DETECTED at {min_distance:.2f}m'
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m!')
        else:
            obstacle_msg.data = f'FREE_PATH ahead, nearest obstacle at {min_distance:.2f}m'
            self.get_logger().info(f'Free path, nearest obstacle at {min_distance:.2f}m')
        
        self.publisher_.publish(obstacle_msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    
    try:
        rclpy.spin(obstacle_detector)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_detector.destroy_node()
        rclpy.shutdown()
```

## Parameter Server Usage in ROS 2 Nodes

Parameters allow you to configure your nodes at runtime without recompiling. Here's how to use them:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('safety_distance', 0.5)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value
        
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Safety distance: {self.safety_distance}')
        
        # You can also set up a callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.PARAMETER_DOUBLE:
                self.max_velocity = param.value
                self.get_logger().info(f'Max velocity updated to: {self.max_velocity}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()
    
    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        pass
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()
```

## Action Servers and Clients in ROS 2

Actions are used for long-running tasks with feedback. Here's an example:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate work
            from time import sleep
            sleep(1)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy()
        rclpy.shutdown()
```

## Debugging and Logging Techniques for ROS 2 Nodes

Effective debugging and logging are crucial for developing robust ROS 2 applications:

```python
import rclpy
from rclpy.node import Node
import traceback

class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')
        
        # Set up logging at different levels
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Different logging levels
        self.get_logger().debug('This is a debug message')
        self.get_logger().info('This is an info message')
        self.get_logger().warn('This is a warning message')
        self.get_logger().error('This is an error message')
        self.get_logger().fatal('This is a fatal message')
        
        # Log with formatting
        robot_id = 1
        position = (2.5, 3.0)
        self.get_logger().info(f'Robot {robot_id} is at position {position}')
        
        # Exception logging
        try:
            result = 10 / 0  # This will cause an exception
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

def main(args=None):
    rclpy.init(args=args)
    logging_node = LoggingNode()
    
    try:
        rclpy.spin(logging_node)
    except KeyboardInterrupt:
        logging_node.get_logger().info('Node interrupted by user')
    finally:
        logging_node.destroy_node()
        rclpy.shutdown()
```

## Learning Objectives

After completing this chapter, you should be able to:
- Create custom message types for ROS 2
- Implement complex publisher-subscriber patterns with custom messages
- Set up multi-node communication systems
- Use the parameter server to configure nodes at runtime
- Implement action servers and clients for long-running tasks
- Apply effective debugging and logging techniques to ROS 2 nodes

## Chapter Summary

This chapter expanded on the basic ROS 2 concepts introduced in Chapter 1, demonstrating more advanced implementations. We explored custom message types, multi-node communication patterns, parameter servers, action servers, and effective debugging techniques. These concepts are fundamental to building complex, robust robotic applications that can operate in real-world environments.
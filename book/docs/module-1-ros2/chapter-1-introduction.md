# Chapter 1: Introduction to ROS 2 - The Robotic Nervous System

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's not an operating system in the traditional sense, but rather a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Think of ROS 2 as the "nervous system" of a robot - it provides the communication infrastructure that allows different parts of the robot (sensors, controllers, actuators) to work together seamlessly.

## Core Concepts

### Nodes

Nodes are the basic building blocks of any ROS 2 application. Each node represents a single process that performs a specific function within the robot system.

**Key characteristics of nodes:**
- Each node has a unique name
- Nodes can communicate with other nodes through topics, services, and actions
- A single machine can run multiple nodes
- Nodes can be distributed across multiple machines

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Hello from my_node!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic, while other nodes can subscribe to that same topic to receive those messages.

**Publish-Subscribe Pattern:**
- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic
- **Topic**: The named channel over which messages are exchanged

```python
# Publisher example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Services

Services provide a request-response communication pattern between nodes. Unlike topics, which allow one-to-many communication, services enable one-to-one communication where a client requests a specific action from a server.

**Service Components:**
- **Service Server**: A node that provides a service
- **Service Client**: A node that requests a service
- **Service Type**: Defines the request and response message types

```python
# Service server example
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\n a: %d b: %d' % (request.a, request.b))
        return response
```

## Why Use ROS 2?

ROS 2 offers several advantages for robotics development:

1. **Modularity**: Applications are built from independent components (nodes) that communicate through well-defined interfaces.
2. **Reusability**: Many common robot functionalities (navigation, perception, control) are available as pre-built packages.
3. **Community**: Large ecosystem of developers, documentation, and third-party packages.
4. **Cross-platform**: Runs on various operating systems and hardware platforms.
5. **Real-time support**: Designed to handle real-time constraints in robotic applications.
6. **Distributed architecture**: Allows nodes to run on different machines connected via a network.

## Getting Started

To begin working with ROS 2, you'll need to:
1. Install ROS 2 Humble Hawksbill (see Chapter 2 for detailed setup instructions)
2. Set up your development environment
3. Create your first ROS 2 package
4. Write your first nodes and establish communication between them

In the following chapters, we'll dive deeper into each of these concepts and show you how to implement them in practice.

## Learning Objectives

After completing this chapter, you should be able to:
- Explain what ROS 2 is and its role in robotics
- Identify the core components of the ROS 2 architecture (Nodes, Topics, Services)
- Understand the publish-subscribe communication pattern
- Recognize the advantages of using ROS 2 for robotics development

## Chapter Summary

ROS 2 serves as the communication nervous system of robotic systems, providing a flexible framework for creating complex robot behaviors. With its core concepts of Nodes, Topics, and Services, ROS 2 enables different components of a robot to work together seamlessly. This modular approach allows for robust and scalable robotic applications, making it an essential tool for humanoid robotics development.
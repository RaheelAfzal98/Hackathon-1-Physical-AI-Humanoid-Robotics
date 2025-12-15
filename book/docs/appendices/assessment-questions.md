# Assessment Questions: Module 1 - ROS 2 Nervous System

## Chapter 1: Introduction to ROS 2 - The Robotic Nervous System

### Multiple Choice Questions

1. What does ROS 2 stand for?
   a) Robot Operating System 2
   b) Robotic Operations Suite 2
   c) Robot Operating Software 2
   d) Robotic Operating System 2

   **Answer: a) Robot Operating System 2**

2. In ROS 2, what are the three core concepts that make up the communication framework?
   a) Nodes, Topics, Services
   b) Clients, Servers, Messages
   c) Publishers, Subscribers, Transforms
   d) Actions, Interfaces, Libraries

   **Answer: a) Nodes, Topics, Services**

3. Which of the following is NOT a characteristic of a ROS 2 Node?
   a) Each node has a unique name
   b) Nodes communicate with each other through topics and services
   c) A single machine can run multiple nodes
   d) A single machine can only run one node

   **Answer: d) A single machine can only run one node**

### Short Answer Questions

4. Explain the publish-subscribe communication pattern in ROS 2 and provide an example of when it would be used.

   **Answer: The publish-subscribe pattern allows one or more nodes to publish messages to a topic, and one or more nodes to subscribe to that same topic to receive those messages. This enables one-to-many communication where multiple subscribers can receive the same information from a single publisher. For example, a sensor node could publish laser scan data to a topic, and multiple nodes (navigation, mapping, obstacle detection) could subscribe to receive this data simultaneously.**

5. What are the advantages of using ROS 2 for robotics development?

   **Answer: ROS 2 offers several advantages: modularity (applications built from independent components), reusability (common robot functionalities available as pre-built packages), large community and documentation, cross-platform compatibility, real-time support, and distributed architecture for nodes across multiple machines.**

## Chapter 2: Implementing ROS 2 Nodes and Communication

### Multiple Choice Questions

1. What is the main difference between revolute and continuous joints in a URDF file?
   a) Revolute joints are for arms, continuous joints are for wheels
   b) Revolute joints have position limits, continuous joints don't
   c) Revolute joints are rigid, continuous joints are flexible
   d) There is no difference between them

   **Answer: b) Revolute joints have position limits, continuous joints don't**

2. Which ROS 2 message type would you use for sending commands to control a robot's velocity?
   a) sensor_msgs/LaserScan
   b) geometry_msgs/Pose
   c) geometry_msgs/Twist
   d) std_msgs/Float64

   **Answer: c) geometry_msgs/Twist**

3. In ROS 2, what is the purpose of the parameter server?
   a) To store configuration values that can be accessed and modified at runtime
   b) To manage network connections
   c) To store sensor data
   d) To manage robot's internal memory

   **Answer: a) To store configuration values that can be accessed and modified at runtime**

### Programming Questions

4. Write a ROS 2 publisher node in Python that publishes the message "Hello from Chapter 2 Exercise" to a topic called 'chapter2_test' at 2 Hz.

   **Answer:**
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class Chapter2Publisher(Node):
       def __init__(self):
           super().__init__('chapter2_publisher')
           self.publisher = self.create_publisher(String, 'chapter2_test', 10)
           timer_period = 0.5  # seconds (2 Hz)
           self.timer = self.create_timer(timer_period, self.timer_callback)

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello from Chapter 2 Exercise'
           self.publisher.publish(msg)
           self.get_logger().info(f'Publishing: {msg.data}')

   def main(args=None):
       rclpy.init(args=args)
       node = Chapter2Publisher()
       
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Chapter 3: Python Agents Bridging with ROS 2

### Multiple Choice Questions

1. What is the primary purpose of a Python agent in the context of ROS 2?
   a) To replace C++ nodes completely
   b) To provide high-level decision making and integration with AI/ML models
   c) To manage system memory
   d) To store robot logs

   **Answer: b) To provide high-level decision making and integration with AI/ML models**

2. Which of the following state machine patterns is most appropriate for a robot that needs to patrol an area while avoiding obstacles?
   a) IDLE → MOVING
   b) IDLE → PATROLLING → CHARGING
   c) IDLE → EXPLORING → AVOIDING_OBSTACLE → RETURNING_HOME
   d) MOVING_ONLY → STOPPING

   **Answer: c) IDLE → EXPLORING → AVOIDING_OBSTACLE → RETURNING_HOME**

### Short Answer Questions

3. Explain how a Python agent can integrate AI/ML models with ROS 2, and provide one practical example.

   **Answer: Python agents can integrate AI/ML models by using the models to process sensor data received from ROS 2 topics and using the results to make decisions that are published back to ROS 2 topics or services. For example, a Python agent could subscribe to camera image data, run an object detection model to identify people, and then publish velocity commands to navigate toward the detected person.**

## Chapter 4: Understanding URDF for Humanoid Robots

### Multiple Choice Questions

1. In a URDF file, what does the `<inertial>` tag define?
   a) How the link appears in visualization
   b) The physical properties of the link for physics simulation
   c) The material properties of the link
   d) The sensor properties of the link

   **Answer: b) The physical properties of the link for physics simulation**

2. Which joint type would you use for a typical elbow joint in a humanoid robot?
   a) Fixed
   b) Continuous
   c) Revolute
   d) Prismatic

   **Answer: c) Revolute**

### Programming Questions

3. Write the URDF XML code to define a simple box-shaped link with these properties:
   - Name: "test_link"
   - Mass: 2.0 kg
   - Dimensions: 0.2m x 0.1m x 0.05m (length x width x height)
   - Position: at origin (0, 0, 0)

   **Answer:**
   ```xml
   <link name="test_link">
     <inertial>
       <mass value="2.0" />
       <origin xyz="0 0 0" rpy="0 0 0" />
       <inertia ixx="0.0083" ixy="0" ixz="0" iyy="0.0092" iyz="0" izz="0.0167" />
     </inertial>
     <visual>
       <origin xyz="0 0 0" rpy="0 0 0" />
       <geometry>
         <box size="0.2 0.1 0.05" />
       </geometry>
     </visual>
     <collision>
       <origin xyz="0 0 0" rpy="0 0 0" />
       <geometry>
         <box size="0.2 0.1 0.05" />
       </geometry>
     </collision>
   </link>
   ```

## Chapter 5: Complete Humanoid Robot Implementation Example

### Short Answer Questions

1. Explain how the different components from previous chapters (Nodes, Topics, Services, URDF, Python agents) work together in the complete humanoid robot implementation.

   **Answer: The complete implementation integrates all components as follows: 
   - URDF defines the robot's physical structure and kinematic chains
   - Nodes manage different aspects of the robot (sensors, controllers, agents)
   - Topics enable communication between different parts of the system (sensor data, commands)
   - Services handle specific interactions that require request-response patterns
   - Python agents provide high-level decision making by processing sensor data and issuing commands
   - All parts work together to create a complete robotic system that can perceive, decide, and act.**

### Scenario-Based Questions

2. A student is trying to run the complete humanoid robot example but the robot doesn't move when given velocity commands. What are three potential issues to check?

   **Answer: 
   - Check if the joint controllers are running and properly configured
   - Verify that the robot state publisher is correctly publishing transforms
   - Make sure the URDF model is properly loaded and the simulation environment is set up correctly
   - Check if there are any errors in the command velocity topic subscriptions**

## General Module Questions

### Integration Questions

1. How do the concepts learned in this module (ROS 2 Nervous System) prepare a student for the advanced topics in the subsequent modules (Digital Twin, AI-Robot Brain, VLA)?

   **Answer: The ROS 2 Nervous System module provides the fundamental communication framework that underlies all other modules. Understanding Nodes, Topics, and Services is essential for:
   - Module 2: Understanding how simulation systems integrate with real robots through ROS 2
   - Module 3: Using ROS 2 as the communication backbone for AI-driven behaviors
   - Module 4: Creating the communication infrastructure for Vision-Language-Action systems
   This module establishes the "nervous system" that connects all other capabilities.**

### Practical Application

2. Describe how you would modify the example robot controller from Chapter 5 to implement a simple pick-and-place task using the robot's arms.

   **Answer: To implement a pick-and-place task, I would:
   - Add inverse kinematics solvers for both arms to calculate joint angles needed to reach specific positions
   - Create a grasp planner to determine how to approach and grasp objects
   - Add joint controllers for grippers/hands if hardware exists
   - Implement state machine logic for: APPROACH_OBJECT → GRASP → LIFT → MOVE_TO_PLACE_POSITION → RELEASE → RETRACT
   - Add sensor feedback (camera, force/torque sensors) to verify grasp success and adjust as needed
   - Create trajectory generators for smooth motion between waypoints**
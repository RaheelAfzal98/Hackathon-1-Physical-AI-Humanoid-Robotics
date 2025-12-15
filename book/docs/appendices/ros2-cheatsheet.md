# ROS 2 Cheatsheet

This cheatsheet provides quick references for common ROS 2 commands, concepts, and code patterns.

## Common Command Line Tools

### System Commands
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Ubuntu
# or
source C:\dev\ros_humble\setup.bat  # Windows

# Check ROS 2 installation
ros2 --version

# Get help
ros2 --help
```

### Package Management
```bash
# Create a new package
ros2 pkg create --build-type ament_python my_robot_package

# List all packages
ros2 pkg list

# Show package information
ros2 pkg info my_robot_package
```

### Node Commands
```bash
# List active nodes
ros2 node list

# Get info about a specific node
ros2 node info my_node

# Execute a node directly
ros2 run my_package my_node
```

### Topic Commands
```bash
# List all topics
ros2 topic list

# Show topic information
ros2 topic info /my_topic

# Echo messages from a topic
ros2 topic echo /my_topic std_msgs/msg/String

# Publish a message to a topic
ros2 topic pub /my_topic std_msgs/msg/String "data: 'Hello World'"

# Show topic type
ros2 topic type /my_topic
```

### Service Commands
```bash
# List all services
ros2 service list

# Show service information
ros2 service info /my_service

# Call a service
ros2 service call /my_service example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### Action Commands
```bash
# List all actions
ros2 action list

# Send a goal to an action
ros2 action send_goal /my_action example_interfaces/action/Fibonacci "{order: 5}"
```

### Parameter Commands
```bash
# List parameters of a node
ros2 param list my_node

# Get a parameter value
ros2 param get my_node my_parameter

# Set a parameter value
ros2 param set my_node my_parameter 42
```

## Common Python Code Patterns

### Basic Node Structure
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize the node here

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Pattern
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1
```

### Subscriber Pattern
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

### Service Server Pattern
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddService(Node):
    def __init__(self):
        super().__init__('add_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Service Client Pattern
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

### Action Server Pattern
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

### Action Client Pattern
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')
```

## Common Message Types

### Standard Messages
- `std_msgs/msg/Bool`
- `std_msgs/msg/Int32`
- `std_msgs/msg/Float64` 
- `std_msgs/msg/String`
- `std_msgs/msg/ColorRGBA`

### Geometry Messages
- `geometry_msgs/msg/Twist` - Linear and angular velocity
- `geometry_msgs/msg/Pose` - Position and orientation
- `geometry_msgs/msg/Point` - 3D point
- `geometry_msgs/msg/Quaternion` - Rotation as quaternion

### Sensor Messages
- `sensor_msgs/msg/LaserScan` - Laser scanner data
- `sensor_msgs/msg/Imu` - Inertial measurement unit data
- `sensor_msgs/msg/JointState` - Joint positions, velocities, efforts
- `sensor_msgs/msg/Image` - Image data
- `sensor_msgs/msg/CameraInfo` - Camera calibration data

### Navigation Messages
- `nav_msgs/msg/Odometry` - Odometry data
- `nav_msgs/msg/Path` - A path of poses
- `geometry_msgs/msg/PoseStamped` - A pose with timestamp and frame_id

## Launch Files

### Python Launch File Example
```python
# launch/my_launch_file.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[
                {'param1': 'value1'},
                {'param2': 42}
            ],
            remappings=[
                ('original_topic', 'new_topic')
            ]
        )
    ])
```

### Running Launch Files
```bash
# Run a launch file
ros2 launch my_package my_launch_file.py

# Run with arguments
ros2 launch my_package my_launch_file.py arg_name:=arg_value
```

## Working with Parameters

### Declaring and Using Parameters
```python
def __init__(self):
    super().__init__('parameter_node')
    
    # Declare parameter with default value
    self.declare_parameter('param_name', 'default_value')
    
    # Get parameter value
    param_value = self.get_parameter('param_name').value
```

### YAML Parameter Files
```yaml
# config/my_params.yaml
my_node:
  ros__parameters:
    param1: value1
    param2: 42
    param3: true
```

### Loading Parameters from YAML
```bash
ros2 run my_package my_node --ros-args --params-file config/my_params.yaml
```

## Quality of Service (QoS)

### Common QoS Profiles
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Reliable communication
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Best effort (for sensors)
best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

## Debugging Commands

### Monitoring System
```bash
# Monitor CPU and memory usage
htop

# Monitor network connections
netstat -tuln

# Check ROS 2 graph
ros2 doctor
```

### Profiling Tools
```bash
# Install ros2 tracing
sudo apt install ros-humble-tracing

# Trace system
ros2 trace my_trace_directory
```

## Common Issues and Solutions

### Node Not Connecting
- Check if ROS_DOMAIN_ID is the same for all nodes
- Ensure nodes are on the same network (for multi-robot systems)

### Topic Not Receiving Data
- Check topic names match exactly (`ros2 topic list`)
- Verify publisher is active (`ros2 node info`)
- Check QoS compatibility between publisher and subscriber

### Memory Leaks
- Always call `destroy_node()` during shutdown
- Be careful with callback groups and executors
- Monitor memory usage over time

### Performance Tips
- Use appropriate QoS settings for your use case
- Keep callbacks lightweight
- Use multi-threaded executors when needed
- Consider using intra-process communication for same-process nodes
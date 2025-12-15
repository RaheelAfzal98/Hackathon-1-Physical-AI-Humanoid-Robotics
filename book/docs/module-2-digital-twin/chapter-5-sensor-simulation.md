# Chapter 5: Sensor Simulation - LiDAR, Depth Cameras, IMUs

## Overview of Sensor Simulation in Robotics

Sensors are critical components of robotic systems, providing the data necessary for perception, navigation, and interaction with the environment. In Digital Twin environments, accurate sensor simulation is essential for creating realistic training data, validating perception algorithms, and ensuring that robots behave correctly in both simulation and the real world.

This chapter focuses on simulating three essential sensor types for humanoid robots:

1. **LiDAR (Light Detection and Ranging)**: Provides 360-degree range data for mapping and navigation
2. **Depth Cameras**: Supplies 3D spatial information for object recognition and scene understanding
3. **IMU (Inertial Measurement Unit)**: Delivers orientation, acceleration, and angular velocity data for balance and locomotion

## Understanding Sensor Simulation

### Why Simulate Sensors?

Sensor simulation offers several advantages:

- **Safety**: Test algorithms without physical risk
- **Cost Effectiveness**: Run thousands of trials without expensive equipment
- **Reproducibility**: Generate identical scenarios to compare algorithms
- **Edge Cases**: Create rare but critical situations for robustness testing
- **Ground Truth**: Access perfect information for training and validation

### Real vs. Simulated Sensors

Real sensors have:
- Physical limitations (limited FOV, range, resolution)
- Noise characteristics (Gaussian, salt-and-pepper, quantization)
- Environmental dependencies (lighting, weather, interference)
- Latency and frequency constraints

Simulated sensors should replicate these characteristics to ensure successful transfer from simulation to reality.

## LiDAR Simulation

### Understanding LiDAR Sensors

LiDAR sensors emit laser pulses and measure the time it takes for the reflected signal to return, calculating distances to objects with high precision. For humanoid robots, LiDAR provides critical data for:

- Obstacle detection and avoidance
- Environment mapping (SLAM)
- Localization
- Navigation path planning
- Human detection and tracking

### Configuring LiDAR in Gazebo

Here's how to configure a LiDAR sensor in Gazebo for your humanoid robot:

```xml
<!-- In your robot's URDF/XACRO file -->
<gazebo reference="lidar_mount_link">
  <sensor name="humanoid_lidar" type="ray">
    <!-- Visualize the LiDAR beams in Gazebo (optional for debugging) -->
    <visualize>false</visualize>
    
    <!-- Update rate in Hz -->
    <update_rate>10</update_rate>
    
    <!-- LiDAR properties -->
    <ray>
      <!-- Horizontal scan properties -->
      <scan>
        <horizontal>
          <!-- Number of rays per scan -->
          <samples>720</samples>
          
          <!-- Angular resolution -->
          <resolution>1</resolution>
          
          <!-- Field of view (in radians) -->
          <min_angle>-3.14159</min_angle> <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>  <!-- 180 degrees -->
        </horizontal>
      </scan>
      
      <!-- Range properties -->
      <range>
        <!-- Minimum and maximum detection range -->
        <min>0.1</min>   <!-- 10 cm minimum range -->
        <max>10.0</max>  <!-- 10 m maximum range -->
        <resolution>0.01</resolution>  <!-- 1 cm resolution -->
      </range>
    </ray>
    
    <!-- Noise model for realistic behavior -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev> <!-- 1 cm standard deviation -->
    </noise>
    
    <!-- Plugin for ROS communication -->
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs::LaserScan</output_type>
      <frame_name>lidar_mount_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Advanced LiDAR Configuration

For more realistic LiDAR simulation with specific characteristics:

```xml
<!-- Higher-performance LiDAR configuration -->
<gazebo reference="high_performance_lidar">
  <sensor name="front_3d_lidar" type="gpu_ray">
    <update_rate>20</update_rate>
    
    <ray>
      <scan>
        <horizontal>
          <samples>1081</samples> <!-- Higher resolution than previous example -->
          <resolution>1</resolution>
          <min_angle>-2.35619</min_angle> <!-- ~-135 degrees -->
          <max_angle>2.35619</max_angle>  <!-- ~135 degrees -->
        </horizontal>
        <vertical>
          <samples>16</samples> <!-- 16 beams for 3D scanning -->
          <min_angle>-0.2618</min_angle> <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>  <!-- 15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.08</min>  <!-- Minimum range for realistic sensors -->
        <max>120.0</max>  <!-- Maximum range -->
        <resolution>0.001</resolution> <!-- High resolution -->
      </range>
    </ray>
    
    <!-- More detailed noise model -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.008</stddev> <!-- Slightly better than basic LiDAR -->
    </noise>
    
    <!-- GPU-accelerated plugin -->
    <plugin name="gpu_lidar_controller" filename="libgazebo_ros_gpu_laser.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=front_3d_laser_scan</remapping>
      </ros>
      <output_type>sensor_msgs::LaserScan</output_type>
      <frame_name>front_3d_lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Processing LiDAR Data

Example ROS 2 code to process LiDAR data from our humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import numpy as np
from collections import deque


class LiDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Subscribe to LiDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.lidar_callback,
            10
        )
        
        # Publisher for navigation commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid_robot/cmd_vel', 10)
        
        # Buffer for smoothing LiDAR data
        self.scan_buffer = deque(maxlen=5)
        
        # Robot parameters
        self.robot_safety_distance = 0.5  # meters
        self.min_clear_path_width = 0.6  # meters (space for humanoid robot width)
        
        self.get_logger().info('LiDAR Processor initialized')

    def lidar_callback(self, msg):
        """Process incoming LiDAR data"""
        # Add to buffer for smoothing
        self.scan_buffer.append(msg)
        
        # Process the latest scan
        if len(self.scan_buffer) == self.scan_buffer.maxlen:
            smoothed_scan = self.smooth_scans()
            self.process_obstacle_detection(smoothed_scan)

    def smooth_scans(self):
        """Smooth multiple consecutive scans to reduce noise"""
        # Get all scans in buffer
        scans = []
        for scan_msg in self.scan_buffer:
            ranges = np.array(scan_msg.ranges)
            # Convert inf and NaN values to max range for processing
            ranges[~np.isfinite(ranges)] = scan_msg.range_max
            scans.append(ranges)
        
        # Average the ranges to smooth out noise
        averaged_ranges = np.mean(scans, axis=0)
        
        # Create a new LaserScan message with averaged ranges
        smoothed_msg = LaserScan()
        smoothed_msg.header = self.scan_buffer[-1].header
        smoothed_msg.angle_min = self.scan_buffer[-1].angle_min
        smoothed_msg.angle_max = self.scan_buffer[-1].angle_max
        smoothed_msg.angle_increment = self.scan_buffer[-1].angle_increment
        smoothed_msg.time_increment = self.scan_buffer[-1].time_increment
        smoothed_msg.scan_time = self.scan_buffer[-1].scan_time
        smoothed_msg.range_min = self.scan_buffer[-1].range_min
        smoothed_msg.range_max = self.scan_buffer[-1].range_max
        smoothed_msg.ranges = averaged_ranges.tolist()
        
        return smoothed_msg

    def process_obstacle_detection(self, scan_msg):
        """Process scan data to detect obstacles and plan navigation"""
        # Convert ranges to numpy array for processing
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        
        # Define sectors: front, left, right, rear
        mid_idx = len(ranges) // 2
        sector_size = len(ranges) // 8  # 45-degree sectors
        
        front_sector = ranges[mid_idx - sector_size//2 : mid_idx + sector_size//2]
        left_sector = ranges[mid_idx + sector_size//2 : mid_idx + sector_size//2 + sector_size]
        right_sector = ranges[mid_idx - sector_size//2 - sector_size : mid_idx - sector_size//2]
        rear_sector = ranges[:sector_size] + ranges[-sector_size:]
        
        # Find minimum distances in each sector
        front_min = np.min(front_sector[np.isfinite(front_sector)]) if np.any(np.isfinite(front_sector)) else float('inf')
        left_min = np.min(left_sector[np.isfinite(left_sector)]) if np.any(np.isfinite(left_sector)) else float('inf')
        right_min = np.min(right_sector[np.isfinite(right_sector)]) if np.any(np.isfinite(right_sector)) else float('inf')
        rear_min = np.min(rear_sector[np.isfinite(rear_sector)]) if np.any(np.isfinite(rear_sector)) else float('inf')
        
        # Determine navigation behavior based on obstacle detection
        cmd_vel = Twist()
        
        if front_min < self.robot_safety_distance:
            # Obstacle in front - stop and evaluate
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.evaluate_turn_direction(left_min, right_min)
            self.get_logger().warn(f'Obstacle detected in front: {front_min:.2f}m. Turning.')
        elif self.path_is_clear_in_front(scan_msg, 45):  # Check 45-degree cone in front
            # Path is clear, move forward
            cmd_vel.linear.x = 0.3  # Move forward at 0.3 m/s
            cmd_vel.angular.z = 0.0
        else:
            # No clear path directly ahead, look for alternatives
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.2  # Look around by turning slowly
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def evaluate_turn_direction(self, left_dist, right_dist):
        """Determine which direction to turn based on obstacle distances"""
        if left_dist > right_dist:
            return 0.3  # Turn left
        elif right_dist > left_dist:
            return -0.3  # Turn right
        else:
            return 0.1  # Slight turn if distances are equal
    
    def path_is_clear_in_front(self, scan_msg, cone_angle_deg=30):
        """Check if the path is clear in a cone in front of the robot"""
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        
        # Calculate indices for the front cone
        half_cone_rads = np.radians(cone_angle_deg / 2)
        half_cone_idx = int(half_cone_rads / angle_increment)
        mid_idx = len(ranges) // 2
        
        front_cone = ranges[mid_idx - half_cone_idx : mid_idx + half_cone_idx]
        valid_distances = front_cone[np.isfinite(front_cone)]
        
        # Check if there's a clear path (no obstacles within safety distance)
        if len(valid_distances) == 0:
            return True  # No readings in front means clear path
        
        min_distance = np.min(valid_distances)
        return min_distance > self.robot_safety_distance

def main(args=None):
    rclpy.init(args=args)
    processor = LiDARProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Shutting down LiDAR processor')
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Camera Simulation

### Understanding Depth Cameras

Depth cameras provide both color (RGB) and distance (depth) information, making them invaluable for humanoid robots for:

- 3D scene reconstruction
- Object recognition with spatial context
- Human detection and pose estimation
- Manipulation tasks requiring precise positioning
- SLAM in textured environments

### Configuring Depth Camera in Gazebo

Here's how to configure a depth camera in Gazebo:

```xml
<!-- Depth camera mounted on the robot's head -->
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera>
      <!-- Camera intrinsics -->
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>   <!-- 10 cm near clip -->
        <far>10.0</far>    <!-- 10 m far clip -->
      </clip>
    </camera>
    
    <!-- Noise model for realistic depth data -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.03</stddev> <!-- 3cm standard deviation -->
    </noise>
    
    <!-- Plugin for ROS communication -->
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/depth/image_raw:=/camera/depth/image_raw</remapping>
        <remapping>~/rgb/image_raw:=/camera/rgb/image_raw</remapping>
        <remapping>~/depth/camera_info:=/camera/depth/camera_info</remapping>
        <remapping>~/rgb/camera_info:=/camera/rgb/camera_info</remapping>
      </ros>
      
      <!-- Camera properties -->
      <camera_name>head_camera</camera_name>
      <frame_name>camera_optical_frame</frame_name>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
    </plugin>
  </sensor>
</gazebo>
```

### Processing Depth Camera Data

Example ROS 2 code to process depth camera data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Subscribe to depth and RGB images
        self.depth_sub = self.create_subscription(
            Image,
            '/humanoid_robot/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.rgb_sub = self.create_subscription(
            Image,
            '/humanoid_robot/camera/rgb/image_raw',
            self.rgb_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/humanoid_robot/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/humanoid_robot/points', 10)
        self.closest_object_pub = self.create_publisher(PointStamped, '/humanoid_robot/closest_object', 10)
        
        # Internal state
        self.latest_depth = None
        self.latest_rgb = None
        self.camera_info = None
        self.intrinsic_matrix = None
        
        # Processing timer
        self.process_timer = self.create_timer(0.1, self.process_images)  # 10 Hz
        
        self.get_logger().info('Depth Camera Processor initialized')

    def camera_info_callback(self, msg):
        """Receive camera calibration parameters"""
        self.camera_info = msg
        # Extract intrinsic matrix from camera info
        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        """Process depth image"""
        try:
            # Convert ROS Image to OpenCV format
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def rgb_callback(self, msg):
        """Process RGB image"""
        try:
            # Convert ROS Image to OpenCV format
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')

    def process_images(self):
        """Process synchronized depth and RGB images"""
        if self.latest_depth is None or self.latest_rgb is None:
            return
        
        # Process depth to find closest object
        closest_point = self.find_closest_object(self.latest_depth)
        
        if closest_point is not None:
            # Publish the closest object as a point
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "camera_optical_frame"
            point_msg.point.x = closest_point[0]
            point_msg.point.y = closest_point[1]
            point_msg.point.z = closest_point[2]
            
            self.closest_object_pub.publish(point_msg)
        
        # Convert to point cloud
        if self.intrinsic_matrix is not None:
            pointcloud_msg = self.create_pointcloud(
                self.latest_depth, 
                self.latest_rgb, 
                self.intrinsic_matrix
            )
            if pointcloud_msg:
                self.pointcloud_pub.publish(pointcloud_msg)

    def find_closest_object(self, depth_image):
        """Find the closest object in the depth image"""
        # Convert to meters if needed
        depth_meters = depth_image
        
        # Find non-zero (valid) depth values
        valid_depths = depth_meters[np.isfinite(depth_meters) & (depth_meters > 0)]
        
        if len(valid_depths) == 0:
            return None
        
        # Find the minimum depth and its position
        min_depth = np.min(valid_depths)
        
        # Find all positions with minimum depth (or close to it)
        min_positions = np.where(depth_meters == min_depth)
        
        if len(min_positions[0]) > 0:
            # Get the center of the minimum depth region
            u = int(np.mean(min_positions[1]))  # x coordinate in image
            v = int(np.mean(min_positions[0]))  # y coordinate in image
            
            # Convert pixel coordinates to 3D coordinates
            if self.intrinsic_matrix is not None:
                fx, fy = self.intrinsic_matrix[0, 0], self.intrinsic_matrix[1, 1]
                cx, cy = self.intrinsic_matrix[0, 2], self.intrinsic_matrix[1, 2]
                
                # Calculate 3D position
                x = (u - cx) * min_depth / fx
                y = (v - cy) * min_depth / fy
                z = min_depth
                
                return (x, y, z)
        
        return None

    def create_pointcloud(self, depth_image, rgb_image, intrinsic_matrix):
        """Create PointCloud2 message from depth and RGB images"""
        height, width = depth_image.shape
        
        # Create XYZRGB point cloud
        points = []
        
        fx = intrinsic_matrix[0, 0]
        fy = intrinsic_matrix[1, 1]
        cx = intrinsic_matrix[0, 2]
        cy = intrinsic_matrix[1, 2]
        
        # Sample every nth pixel to reduce point cloud size
        sample_rate = 4  # Only sample every 4th pixel
        
        for v in range(0, height, sample_rate):
            for u in range(0, width, sample_rate):
                z = depth_image[v, u]
                
                # Only include valid depth points
                if np.isfinite(z) and z > 0:
                    # Convert pixel to 3D coordinates
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    # Get RGB values (need to convert from BGR to RGB)
                    if u < rgb_image.shape[1] and v < rgb_image.shape[0]:
                        bgr = rgb_image[v, u]
                        rgb = (int(bgr[2]) << 16) | (int(bgr[1]) << 8) | int(bgr[0])
                    else:
                        rgb = 0  # Default to black if out of bounds
                    
                    points.append([x, y, z, rgb])
        
        if len(points) == 0:
            return None
        
        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_optical_frame"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        
        # Create the message
        pointcloud_msg = point_cloud2.create_cloud(header, fields, points)
        
        return pointcloud_msg


def main(args=None):
    rclpy.init(args=args)
    processor = DepthCameraProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Shutting down depth camera processor')
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU Simulation

### Understanding IMU Sensors

An IMU (Inertial Measurement Unit) typically combines:
- **Accelerometer**: Measures linear acceleration in 3 axes
- **Gyroscope**: Measures angular velocity in 3 axes
- **Magnetometer**: Measures magnetic field (provides heading reference)

For humanoid robots, IMUs are critical for:
- Balance control and stabilization
- Motion detection and classification
- Navigation (dead reckoning)
- Sensor fusion with other positioning systems

### Configuring IMU in Gazebo

Here's how to configure an IMU in Gazebo:

```xml
<!-- IMU sensor configuration -->
<gazebo reference="imu_link">
  <sensor name="inertial_measurement_unit" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz update rate for humanoid balance -->
    <visualize>false</visualize>
    
    <imu>
      <!-- Gyroscope properties -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev> <!-- 0.01°/s stddev -->
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
      
      <!-- Linear acceleration properties -->
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev> <!-- 17 mg stddev -->
            <bias_mean>0.1</bias_mean>  <!-- 100 mg bias -->
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
    
    <!-- Plugin for ROS communication -->
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Processing IMU Data

Example ROS 2 code to process IMU data for humanoid robot balance:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float32
import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class IMUBalanceController(Node):
    def __init__(self):
        super().__init__('imu_balance_controller')
        
        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )
        
        # Publishers for balance metrics
        self.roll_pitch_pub = self.create_publisher(Vector3, '/humanoid_robot/balance/roll_pitch', 10)
        self.balance_score_pub = self.create_publisher(Float32, '/humanoid_robot/balance/score', 10)
        
        # Internal state
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])  # x, y, z
        
        # Balance thresholds
        self.max_tilt = 0.3  # 17 degrees in radians
        self.max_angular_velocity = 0.5  # rad/s
        
        # Processing timer
        self.process_timer = self.create_timer(0.02, self.process_balance)  # 50 Hz
        
        self.get_logger().info('IMU Balance Controller initialized')

    def imu_callback(self, msg):
        """Update internal state with new IMU data"""
        # Update orientation (quaternion)
        self.orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        # Update angular velocity
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Update linear acceleration
        self.linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def process_balance(self):
        """Process IMU data for balance control"""
        if self.orientation is None:
            return
        
        # Convert quaternion to roll and pitch angles
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation)
        
        # Create vector for roll and pitch
        roll_pitch_msg = Vector3()
        roll_pitch_msg.x = float(roll)  # Roll
        roll_pitch_msg.y = float(pitch)  # Pitch
        roll_pitch_msg.z = float(yaw)  # Yaw
        self.roll_pitch_pub.publish(roll_pitch_msg)
        
        # Calculate balance score (0.0 = perfectly balanced, 1.0 = critically unbalanced)
        tilt_angle = math.sqrt(roll**2 + pitch**2)
        balance_score = min(tilt_angle / self.max_tilt, 1.0)
        
        # Publish balance score
        score_msg = Float32()
        score_msg.data = balance_score
        self.balance_score_pub.publish(score_msg)
        
        # Check for balance issues
        if tilt_angle > self.max_tilt or np.linalg.norm(self.angular_velocity) > self.max_angular_velocity:
            self.get_logger().warn(
                f'Balance issue detected! Tilt: {math.degrees(tilt_angle):.2f}°, '
                f'Angular vel: {np.linalg.norm(self.angular_velocity):.2f} rad/s'
            )
            self.trigger_balance_recovery(roll, pitch, self.angular_velocity)
        else:
            self.get_logger().info(f'Balance OK: Tilt: {math.degrees(tilt_angle):.2f}°')

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def trigger_balance_recovery(self, roll, pitch, angular_vel):
        """Trigger balance recovery behaviors"""
        self.get_logger().info('Initiating balance recovery sequence')
        
        # In a real humanoid robot, this would trigger:
        # 1. Adjust joint positions to shift center of mass
        # 2. Modify stepping pattern to regain balance
        # 3. Adjust foot positions if needed
        # 4. Slow down or pause other motions
        
        # For simulation, we'll just log the recovery attempt
        self.get_logger().info(
            f'Balance recovery - Roll: {math.degrees(roll):.2f}°, '
            f'Pitch: {math.degrees(pitch):.2f}°, '
            f'Ang Vel: {np.linalg.norm(angular_vel):.2f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    controller = IMUBalanceController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down IMU balance controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration and Calibration

### Sensor Calibration

For accurate simulation, sensor calibration is critical. Here's how to calibrate virtual sensors to match real hardware:

```python
# calibration_verification.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import numpy as np

class CalibrationVerification(Node):
    def __init__(self):
        super().__init__('calibration_verification')
        
        # Subscribers for all sensor types
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.verify_lidar_calibration,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/humanoid_robot/camera/depth/image_raw',
            self.verify_depth_calibration,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.verify_imu_calibration,
            10
        )
        
        self.get_logger().info('Calibration verification node initialized')

    def verify_lidar_calibration(self, scan_msg):
        """Verify LiDAR calibration against known environment"""
        # Check if range values are within expected limits
        valid_ranges = [r for r in scan_msg.ranges if np.isfinite(r) and r > 0]
        
        if valid_ranges:
            # Calculate basic statistics
            range_mean = np.mean(valid_ranges)
            range_std = np.std(valid_ranges)
            
            # In a real calibration check, we'd compare these to expected values
            # for known objects in the test environment
            self.get_logger().info(f'LiDAR stats - Mean: {range_mean:.2f}m, Std: {range_std:.2f}m')

    def verify_depth_calibration(self, depth_msg):
        """Verify depth camera calibration"""
        # Convert to numpy array
        depth_cv = CvBridge().imgmsg_to_cv2(depth_msg, "32FC1")
        
        # Check for expected depth patterns (e.g., flat surfaces at known distances)
        valid_depths = depth_cv[np.isfinite(depth_cv)]
        
        if valid_depths.size > 0:
            depth_mean = np.mean(valid_depths)
            depth_std = np.std(valid_depths)
            
            self.get_logger().info(f'Depth stats - Mean: {depth_mean:.2f}m, Std: {depth_std:.2f}m')

    def verify_imu_calibration(self, imu_msg):
        """Verify IMU calibration"""
        # In a calibrated IMU, acceleration due to gravity should be ~9.8 m/s² when robot is static
        acc_norm = np.sqrt(
            imu_msg.linear_acceleration.x**2 +
            imu_msg.linear_acceleration.y**2 +
            imu_msg.linear_acceleration.z**2
        )
        
        # When robot is upright and stationary, z acceleration should be ~9.8
        expected_z_acc = 9.8
        error = abs(imu_msg.linear_acceleration.z - expected_z_acc)
        
        if error < 0.5:  # Acceptable error range
            self.get_logger().info(f'IMU calibrated - Acc norm: {acc_norm:.2f}, Z acc error: {error:.2f}')
        else:
            self.get_logger().warn(f'IMU may need calibration - Z acc error: {error:.2f}')


def main(args=None):
    rclpy.init(args=args)
    verifier = CalibrationVerification()
    
    try:
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        verifier.get_logger().info('Shutting down calibration verification')
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality Assurance for Sensor Simulation

### Testing Sensor Accuracy

It's important to validate that simulated sensors behave like their real-world counterparts:

1. **Range Validation**: Ensure sensor ranges match specifications
2. **Noise Characterization**: Verify noise models match real sensor characteristics
3. **Timing**: Check that update rates match specifications
4. **Coordinate Frames**: Validate that TF transforms are correct

### Performance Considerations

Different sensors have different computational requirements:
- **LiDAR**: Moderate CPU usage, can be GPU-accelerated in some simulators
- **Depth Cameras**: High computational usage due to image processing
- **IMU**: Low computational overhead

These considerations affect simulation performance, especially when multiple sensors are active simultaneously.

## Chapter Summary

Sensor simulation is fundamental to creating effective Digital Twin systems for humanoid robots. Accurately modeling LiDAR, depth cameras, and IMUs allows for:

- Safe development and testing of perception algorithms
- Generation of realistic training data for AI models
- Validation of robot behaviors before physical deployment
- Cost-effective development cycles

Properly configured sensor simulation ensures that algorithms developed in simulation can successfully transfer to real hardware with minimal re-tuning. The key to success lies in accurately modeling the physical properties, noise characteristics, and update rates of real sensors.

## Learning Objectives

After completing this chapter, students should be able to:
- Configure LiDAR sensors in Gazebo with appropriate parameters for humanoid robots
- Set up depth camera simulation with realistic noise and range characteristics
- Implement IMU simulation for balance control applications
- Process sensor data from each sensor type using ROS 2 nodes
- Calibrate simulated sensors to match real hardware characteristics
- Validate sensor simulation accuracy and performance
# Chapter 4: Isaac ROS Acceleration for VSLAM and Navigation

## Introduction to Isaac ROS Acceleration

Isaac ROS is a collection of packages that accelerate perception and navigation tasks using NVIDIA's GPU hardware. For humanoid robots, this acceleration is particularly important as they often require real-time processing of multiple sensor streams to maintain balance, navigate environments, and interact safely with humans.

Isaac ROS provides hardware-accelerated implementations of common robotic algorithms, particularly:
- **Visual SLAM (VSLAM)**: For localization and mapping
- **Perception pipelines**: For object detection and scene understanding
- **Navigation**: For path planning and obstacle avoidance
- **Manipulation**: For grasp planning and motion control

The acceleration comes from leveraging GPU processing for algorithms that are traditionally CPU-intensive, such as image processing, point cloud processing, and complex AI inference tasks.

## Isaac ROS Architecture

### Hardware Acceleration Layers

Isaac ROS leverages multiple layers of NVIDIA hardware acceleration:

1. **CUDA Cores**: For parallel computation acceleration
2. **Tensor Cores**: For AI inference acceleration
3. **RT Cores**: For real-time ray tracing (in some applications)
4. **Video Processing Units**: For accelerated video encoding/decoding

### Key Isaac ROS Packages

- `isaac_ros_apriltag`: Hardware-accelerated AprilTag detection
- `isaac_ros_dnn_decoders`: DNN output decoders with TensorRT acceleration
- `isaac_ros_image_pipeline`: Optimized image processing pipeline
- `isaac_ros_pointcloud_utils`: Point cloud processing utilities
- `isaac_ros_visual_slam`: Visual SLAM with GPU acceleration
- `isaac_ros_nitros`: Nitros data type converters for zero-copy transfers
- `isaac_ros_freespace_segmentation`: Free space detection from stereo images
- `isaac_ros_realsense`: Hardware acceleration for Intel RealSense cameras
- `isaac_ros_pose_graph_localizer`: Pose graph localization
- `isaac_ros_detect_net`: Hardware-accelerated object detection

## Installing Isaac ROS

### Prerequisites

Isaac ROS requires specific NVIDIA hardware and software:

1. **Hardware**: NVIDIA GPU with compute capability 6.0 or higher (Pascal architecture or newer)
2. **Driver**: NVIDIA driver version 470 or newer
3. **CUDA**: CUDA 11.8 or newer
4. **ROS 2**: ROS 2 Humble Hawksbill (recommended)
5. **OS**: Ubuntu 22.04 LTS (recommended)

### Installation Methods

Isaac ROS can be installed in multiple ways:

#### 1. Binary Installation via apt
```bash
# Add NVIDIA ROS2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://repos.jetson-agi.nvidia.com/ros/packages/jetson-agi-keyring.gpg -o /usr/share/keyrings/jetson-agi-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/jetson-agi-keyring.gpg] https://repos.jetson-agi.nvidia.com/ros/packages/$(lsb_release -cs)/ stable main" | sudo tee /etc/apt/sources.list.d/jetson-agi-packages-$(lsb_release -cs).list > /dev/null

sudo apt update
sudo apt install ros-humble-isaac-ros-all
```

#### 2. From Source
```bash
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git -b ros2
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git -b ros2
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git -b ros2
# Add other packages as needed...

cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  isaac_ros_common \
  isaac_ros_visual_slam \
  isaac_ros_apriltag

source install/setup.bash
```

## Isaac ROS for Visual SLAM (VSLAM)

### Understanding Visual SLAM

Visual SLAM (Simultaneous Localization and Mapping) allows robots to:
- Build a map of their environment using visual sensors
- Determine their location within that map in real-time
- Navigate safely without prior knowledge of the environment

Traditional VSLAM algorithms (like ORB-SLAM, LSD-SLAM) are computationally intensive, making them challenging to run in real-time on humanoid robots that have other demanding processes.

### Isaac ROS Visual SLAM Components

Isaac ROS provides an optimized Visual SLAM implementation that leverages:
- **Hardware acceleration**: GPU-accelerated image processing and feature extraction
- **Multi-modal sensing**: Integration with IMU data for improved accuracy
- **Real-time performance**: Optimized for the demands of humanoid robots

#### Key Components:

1. **Image Preprocessing**: Hardware-accelerated image rectification and undistortion
2. **Feature Extraction**: GPU-accelerated feature detection and matching
3. **Pose Estimation**: Real-time camera pose calculation
4. **Mapping**: Map building and maintenance
5. **Loop Closure**: Recognition of previously visited locations

### VSLAM Configuration for Humanoid Robots

```yaml
# config/visual_slam_config.yaml
# Isaac ROS Visual SLAM Configuration for Humanoid Robot

/**:
  ros__parameters:
    # Processing parameters
    enable_imu: true
    enable_rectification: true
    enable_debug_mode: false
    publish_tf: true
    
    # Camera parameters (should match your stereo/RGBD camera)
    left_cam_fps: 30.0
    right_cam_fps: 30.0
    left_cam_resolution: [720, 480]
    right_cam_resolution: [720, 480]
    
    # Feature parameters
    feature_detector_type: "ORB"
    max_features: 1000
    matching_threshold: 0.8
    
    # Tracking parameters
    tracking_rate_hz: 30.0
    min_translation_meters: 0.05
    min_rotation_radians: 0.1
    
    # Mapping parameters
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    publish_map_odom_transform: true
    
    # IMU integration
    use_imu: true
    imu_queue_size: 10
    linear_acceleration_stddev: 0.017
    angular_velocity_stddev: 0.0035
```

### Example: Isaac ROS VSLAM Node Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from isaac_ros_visual_slam_interfaces.msg import SlamStatus


class IsaacVSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_vslam_node')
        
        # Publishers for VSLAM results
        self.pose_publisher = self.create_publisher(TransformStamped, '/visual_slam/pose', 10)
        self.map_publisher = self.create_publisher(MarkerArray, '/visual_slam/map', 10)
        self.status_publisher = self.create_publisher(SlamStatus, '/visual_slam/status', 10)
        
        # Subscribers for sensor data
        self.left_camera_sub = self.create_subscription(
            Image, 
            '/stereo_camera/left/image_rect_color', 
            self.left_camera_callback, 
            10
        )
        
        self.right_camera_sub = self.create_subscription(
            Image, 
            '/stereo_camera/right/image_rect_color', 
            self.right_camera_callback, 
            10
        )
        
        self.left_info_sub = self.create_subscription(
            CameraInfo, 
            '/stereo_camera/left/camera_info', 
            self.camera_info_callback, 
            10
        )
        
        self.right_info_sub = self.create_subscription(
            CameraInfo, 
            '/stereo_camera/right/camera_info', 
            self.camera_info_callback, 
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/data', 
            self.imu_callback, 
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('enable_imu', True),
                ('publish_tf', True),
                ('tracking_rate_hz', 30.0),
                ('feature_detector_type', 'ORB'),
                ('max_features', 1000)
            ]
        )
        
        # Store sensor data
        self.left_image = None
        self.right_image = None
        self.camera_info = {}
        self.imu_data = None
        self.robot_pose = np.eye(4)  # 4x4 transformation matrix
        self.map_points = []
        
        # Processing flags
        self.new_images_available = False
        self.initialized = False
        
        # Create timer for processing loop
        tracking_rate = self.get_parameter('tracking_rate_hz').value
        self.process_timer = self.create_timer(1.0/tracking_rate, self.process_vslam)
        
        self.get_logger().info('Isaac ROS VSLAM node initialized')
    
    def left_camera_callback(self, msg):
        """Process left camera image"""
        self.left_image = msg
        self.check_images_ready()
    
    def right_camera_callback(self, msg):
        """Process right camera image""" 
        self.right_image = msg
        self.check_images_ready()
    
    def camera_info_callback(self, msg):
        """Process camera info"""
        if 'left' in msg.header.frame_id:
            self.camera_info['left'] = msg
        elif 'right' in msg.header.frame_id:
            self.camera_info['right'] = msg
    
    def imu_callback(self, msg):
        """Process IMU data"""
        if self.get_parameter('enable_imu').value:
            self.imu_data = msg
    
    def check_images_ready(self):
        """Check if both stereo images are available"""
        if self.left_image and self.right_image:
            self.new_images_available = True
    
    def process_vslam(self):
        """Main VSLAM processing loop"""
        if not self.new_images_available:
            return
        
        if not self.initialized:
            if self.validate_initialization():
                self.initialize_vslam()
            else:
                return
        
        # Process stereo pair for visual SLAM using Isaac ROS acceleration
        success, pose_change, new_points = self.accelerated_vslam_process(
            self.left_image, 
            self.right_image, 
            self.imu_data if self.get_parameter('enable_imu').value else None
        )
        
        if success:
            # Update internal pose
            self.robot_pose = self.update_robot_pose(self.robot_pose, pose_change)
            
            # Update map points
            self.map_points.extend(new_points)
            
            # Publish results
            self.publish_vslam_results()
            
            # Publish TF transform
            if self.get_parameter('publish_tf').value:
                self.publish_transform()
        
        # Reset flag
        self.new_images_available = False
    
    def accelerated_vslam_process(self, left_img, right_img, imu_data):
        """
        Accelerated VSLAM processing using Isaac ROS hardware acceleration
        This would interface with Isaac ROS packages in real implementation
        """
        # This is a conceptual implementation showing the flow
        # In practice, this would interface with Isaac ROS accelerated nodes
        
        # In real Isaac ROS, you would use Isaac's hardware-accelerated nodes
        # which would be launched separately and communicate via ROS topics
        
        # For this example, we'll simulate the process
        dt = 1.0 / self.get_parameter('tracking_rate_hz').value
        
        # Simulate pose change based on robot movement
        pose_delta = self.simulate_pose_from_images(left_img, right_img)
        
        # Integrate IMU data if available
        if imu_data:
            pose_delta = self.integrate_imu_with_visual(pose_delta, imu_data, dt)
        
        # Simulate new map points detection
        new_points = self.detect_new_map_points(left_img)
        
        return True, pose_delta, new_points
    
    def update_robot_pose(self, current_pose, pose_change):
        """
        Update robot's pose in the map frame
        """
        # Apply the pose change to current pose
        updated_pose = np.dot(current_pose, pose_change)
        return updated_pose
    
    def publish_vslam_results(self):
        """Publish VSLAM results to ROS topics"""
        # Publish pose
        pose_msg = TransformStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.child_frame_id = "visual_slam"
        
        # Convert transformation matrix to position/quaternion
        position = self.matrix_to_position(self.robot_pose)
        quaternion = self.matrix_to_quaternion(self.robot_pose)
        
        pose_msg.transform.translation.x = position[0]
        pose_msg.transform.translation.y = position[1]
        pose_msg.transform.translation.z = position[2]
        pose_msg.transform.rotation.w = quaternion[0]
        pose_msg.transform.rotation.x = quaternion[1]
        pose_msg.transform.rotation.y = quaternion[2]
        pose_msg.transform.rotation.z = quaternion[3]
        
        self.pose_publisher.publish(pose_msg)
        
        # Publish map points
        marker_array = self.create_map_marker_array()
        self.map_publisher.publish(marker_array)
        
        # Publish status
        status_msg = SlamStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.tracking_confidence = 0.95  # Simulated high confidence
        status_msg.mapping_confidence = 0.88
        status_msg.status = "TRACKING"
        self.status_publisher.publish(status_msg)
    
    def publish_transform(self):
        """Publish transform to TF tree"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # Extract position and orientation from robot pose
        position = self.matrix_to_position(self.robot_pose)
        quaternion = self.matrix_to_quaternion(self.robot_pose)
        
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1] 
        t.transform.translation.z = position[2]
        t.transform.rotation.w = quaternion[0]
        t.transform.rotation.x = quaternion[1]
        t.transform.rotation.y = quaternion[2]
        t.transform.rotation.z = quaternion[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def create_map_marker_array(self):
        """Create visualization markers for map points"""
        marker_array = MarkerArray()
        
        # Create markers for each map point
        for i, point in enumerate(self.map_points[-100:]):  # Only show last 100 points
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "vslam_map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.05  # 5cm spheres
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            marker.color.a = 1.0  # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.8   # Amber for visibility
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        return marker_array
    
    def validate_initialization(self):
        """Validate that needed parameters and connections are available"""
        # Check if camera info is available
        if 'left' not in self.camera_info or 'right' not in self.camera_info:
            self.get_logger().warn('Waiting for camera info messages...')
            return False
        
        # Check if images are coming in
        if not self.left_image or not self.right_image:
            self.get_logger().warn('Waiting for stereo images...')
            return False
        
        return True
    
    def initialize_vslam(self):
        """Initialize the VSLAM system"""
        self.get_logger().info('Initializing Isaac ROS VSLAM...')
        
        # Set initial pose to identity
        self.robot_pose = np.eye(4)
        self.initialized = True
        
        self.get_logger().info('Isaac ROS VSLAM initialized successfully')
    
    def matrix_to_position(self, matrix):
        """Extract position from 4x4 transformation matrix"""
        return [matrix[0, 3], matrix[1, 3], matrix[2, 3]]
    
    def matrix_to_quaternion(self, matrix):
        """Extract quaternion from 4x4 transformation matrix"""
        # Extract rotation matrix
        R = matrix[:3, :3]
        
        # Convert to quaternion using standard algorithm
        trace = np.trace(R)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        
        # Normalize quaternion
        norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        return [qw/norm, qx/norm, qy/norm, qz/norm]
    
    def simulate_pose_from_images(self, left_img, right_img):
        """
        Simulate pose estimation from stereo images
        In real Isaac ROS, this would use hardware acceleration
        """
        # This is a simplified simulation - real implementation would use 
        # Isaac ROS's hardware-accelerated stereo processing
        dt = 1.0 / self.get_parameter('tracking_rate_hz').value
        
        # Simulate small translational and rotational movement
        translation = np.array([0.02, 0.01, 0.005]) * dt  # m/s to m per frame
        rotation = np.array([0.001, 0.002, 0.003]) * dt   # rad/s to rad per frame
        
        # Create transformation matrix
        pose_change = np.eye(4)
        pose_change[:3, 3] = translation
        
        # Add small rotation (simplified)
        pose_change[:3, :3] = self.euler_to_rotation_matrix(rotation)
        
        return pose_change
    
    def euler_to_rotation_matrix(self, euler_angles):
        """
        Convert Euler angles to rotation matrix
        """
        rx, ry, rz = euler_angles
        
        # Rotation around X axis
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        
        # Rotation around Y axis
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        
        # Rotation around Z axis
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation matrix
        R = Rz @ Ry @ Rx
        return R


def main(args=None):
    rclpy.init(args=args)
    vslam_node = IsaacVSLAMNode()
    
    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        vslam_node.get_logger().info('Shutting down Isaac ROS VSLAM node')
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Isaac ROS for Navigation

### Accelerated Navigation Stack

Isaac ROS enhances the traditional ROS 2 navigation stack (Nav2) with hardware acceleration:

```yaml
# config/nav2_isaac_params.yaml
# Isaac ROS Enhanced Navigation Configuration

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "nav2_bt_xml_v0/through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "nav2_bt_xml_v0/navigate_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_backup_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_close_to_goal_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    
    # Isaac ROS accelerated controller
    FollowPath:
      plugin: "isaac_ros.NavfnPlanner"  # Isaac ROS optimized planner
      max_iterations: 10000
      tolerance: 0.5
      use_astar: false
      allow_unknown: false

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # Higher resolution for humanoid precision
      origin_x: -3.0
      origin_y: -3.0
      footprint: "[[-0.4, -0.3], [-0.4, 0.3], [0.4, 0.3], [0.4, -0.3]]"
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # Higher for humanoid safety
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /humanoid_robot/scan
          max_obstacle_height: 2.0  # Humanoid robot consideration
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /humanoid_robot/depth_camera/points
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          min_obstacle_height: 0.0
          obstacle_range: 4.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.4  # For humanoid robot
      resolution: 0.05  # Fine resolution for humanoid navigation
      track_unknown_space: true
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 1.0  # Larger for safety
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /humanoid_robot/scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    
    # Isaac ROS accelerated planner
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"  # Base plugin
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### Isaac ROS Accelerated Navigation Node

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
from scipy.spatial.distance import cdist


class IsaacNavigationController(Node):
    def __init__(self):
        super().__init__('isaac_navigation_controller')
        
        # Navigation parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('planning_frequency', 10.0),
                ('control_frequency', 20.0),
                ('acceleration_limit', 1.0),  # m/s²
                ('deceleration_limit', 2.0), # m/s²
                ('linear_velocity_max', 0.5), # m/s for humanoid
                ('angular_velocity_max', 0.8), # rad/s
                ('collision_threshold', 0.3), # m
                ('path_tolerance', 0.2), # m
                ('yaw_tolerance', 0.1), # rad
            ]
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.global_plan_pub = self.create_publisher(Path, '/global_plan', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        self.collision_markers_pub = self.create_publisher(MarkerArray, '/collision_warnings', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        
        # Navigation state
        self.current_pose = None
        self.current_twist = None
        self.goal_pose = None
        self.scan_data = None
        self.global_plan = []
        self.local_plan = []
        self.navigation_active = False
        self.path_index = 0
        
        # Timers
        self.planning_timer = self.create_timer(
            1.0 / self.get_parameter('planning_frequency').value, 
            self.planning_callback)
        
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control_frequency').value, 
            self.control_callback)
        
        # Isaac ROS acceleration status
        self.acceleration_status = {
            'path_planning': True,  # Using Isaac-accelerated planners
            'local_map': True,      # Using Isaac-accelerated costmap
            'obstacle_detection': True,  # Using Isaac-accelerated obstacle detection
        }
        
        self.get_logger().info('Isaac Navigation Controller initialized')
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = msg
    
    def goal_callback(self, msg):
        """Process new navigation goal"""
        self.goal_pose = msg.pose
        self.navigation_active = True
        self.path_index = 0
        
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
        # Plan to the new goal using Isaac-accelerated planning
        self.plan_path_to_goal()
    
    def plan_path_to_goal(self):
        """Plan a path to the goal using accelerated algorithms"""
        if not self.current_pose or not self.goal_pose:
            return
        
        # In real implementation, this would interface with Isaac ROS accelerated planners
        # For this example, we'll simulate path planning with acceleration considerations
        
        # Calculate path from current position to goal
        start_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        goal_pos = np.array([self.goal_pose.position.x, self.goal_pose.position.y])
        
        # Simulate path planning with Isaac acceleration
        path_points = self.isaac_accelerated_path_planning(start_pos, goal_pos)
        
        self.global_plan = path_points
        self.local_plan = []  # Will be populated by local planner
        
        # Publish global plan
        self.publish_global_plan()
        
        self.get_logger().info(f'Planned path with {len(path_points)} waypoints')
    
    def isaac_accelerated_path_planning(self, start_pos, goal_pos):
        """
        Simulate Isaac ROS accelerated path planning
        In reality, this would interface with Isaac's GPU-accelerated planners
        """
        # Simulate finding a path with various optimizations
        path = [start_pos.tolist()]
        
        # Simple direct path with intermediate waypoints
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:  # Only if not already at goal
            num_waypoints = max(2, int(distance / 0.5))  # 0.5m between waypoints
            
            for i in range(1, num_waypoints):
                t = i / num_waypoints
                waypoint = start_pos + t * direction
                path.append(waypoint.tolist())
        
        path.append(goal_pos.tolist())
        return path
    
    def planning_callback(self):
        """Periodic path planning and obstacle avoidance"""
        if not self.navigation_active or not self.current_pose or not self.goal_pose:
            return
        
        # Update local plan based on obstacle detection
        if self.scan_data and self.global_plan:
            self.update_local_plan()
    
    def update_local_plan(self):
        """Update local plan based on sensor data and obstacles"""
        if not self.scan_data or not self.global_plan:
            return
        
        # Process laser scan to detect obstacles in path
        obstacles_ahead = self.detect_obstacles_in_path()
        
        if obstacles_ahead:
            # Recalculate local path around obstacles using Isaac acceleration
            self.local_plan = self.isaac_accelerated_local_planning(obstacles_ahead)
        else:
            # Follow global plan
            self.local_plan = self.get_local_segment_of_global_plan()
    
    def detect_obstacles_in_path(self):
        """Detect obstacles that would block the current path"""
        if not self.scan_data or not self.global_plan:
            return []
        
        min_distance = self.get_parameter('collision_threshold').value
        obstacles = []
        
        # Convert scan ranges to Cartesian coordinates
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        
        for i, range_val in enumerate(self.scan_data.ranges):
            if not (np.isinf(range_val) or np.isnan(range_val)) and range_val < min_distance:
                angle = angle_min + i * angle_increment
                
                # Convert to world coordinates relative to robot
                world_x = range_val * np.cos(angle)
                world_y = range_val * np.sin(angle)
                
                # Transform to map coordinates based on robot pose
                # (Simplified - assumes robot at origin for this calculation)
                obstacles.append((world_x, world_y, range_val))
        
        return obstacles
    
    def isaac_accelerated_local_planning(self, obstacles):
        """
        Simulate Isaac ROS local path planning with GPU acceleration
        In reality, this would use Isaac's hardware-accelerated obstacle avoidance
        """
        # This would interface with Isaac's accelerated local planners
        # For now, simulate by creating a local plan that avoids detected obstacles
        
        if not self.current_pose or not self.global_plan:
            return []
        
        local_plan = []
        
        # Implement simple obstacle avoidance (in real implementation would be Isaac-accelerated)
        robot_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        # Add current position
        local_plan.append(robot_pos.tolist())
        
        # Calculate next few waypoints that avoid obstacles
        current_idx = self.path_index
        max_lookahead = min(len(self.global_plan), current_idx + 10)  # Look ahead at most 10 points
        
        for i in range(current_idx, max_lookahead):
            wp = np.array(self.global_plan[i])
            
            # Check if this waypoint conflicts with obstacles
            safe = True
            for obs_x, obs_y, obs_dist in obstacles:
                # Calculate distance from potential path to obstacle
                # (very simplified collision check)
                dist_to_obs = np.linalg.norm(wp[:2] - [obs_x, obs_y])
                
                if dist_to_obs < 0.5:  # Within danger zone
                    # Try to find a detour point
                    detour = self.calculate_detour_point(wp, [obs_x, obs_y])
                    local_plan.append(detour)
                    safe = False
                    break
            
            if safe:
                local_plan.append(wp.tolist())
        
        return local_plan
    
    def calculate_detour_point(self, waypoint, obstacle_pos):
        """Calculate a detour point to avoid an obstacle"""
        # Simple algorithm to go around obstacle
        robot_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        wp = np.array(waypoint)
        obs = np.array(obstacle_pos)
        
        # Calculate perpendicular offset from line robot->waypoint
        direction = wp - robot_pos
        direction_norm = direction / (np.linalg.norm(direction) + 1e-6)
        
        # Perpendicular vector
        perp = np.array([-direction_norm[1], direction_norm[0]])
        
        # Calculate detour point
        detour = obs + 0.6 * perp  # 60cm clearance
        return detour.tolist()
    
    def control_callback(self):
        """Main navigation control loop"""
        if not self.navigation_active or not self.current_pose:
            # If not navigating, stop the robot
            if self.navigation_active:
                self.publish_zero_velocity()
                self.navigation_active = False
            return
        
        if not self.local_plan or not self.goal_pose:
            return
        
        # Calculate control commands based on local plan
        cmd_vel = self.calculate_control_command()
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Check if we've reached the goal
        if self.has_reached_goal():
            self.navigation_complete()
    
    def calculate_control_command(self):
        """Calculate velocity commands based on local plan"""
        cmd = Twist()
        
        if not self.local_plan or len(self.local_plan) == 0:
            return cmd
        
        # Get current robot position
        robot_pos = np.array([
            self.current_pose.position.x, 
            self.current_pose.position.y
        ])
        
        # Find the next waypoint in the local plan
        next_waypoint = None
        for wp in self.local_plan[1:]:  # Skip current position
            wp_array = np.array(wp[:2])
            dist = np.linalg.norm(robot_pos - wp_array)
            if dist > 0.05:  # At least 5cm from current position
                next_waypoint = wp_array
                break
        
        if next_waypoint is None:
            # If no valid waypoint found, just try to slow down
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd
        
        # Calculate direction to waypoint
        direction = next_waypoint - robot_pos
        distance = np.linalg.norm(direction)
        
        if distance > 0.01:  # Avoid division by zero
            # Normalize direction
            direction_norm = direction / distance
            
            # Calculate desired heading
            desired_yaw = np.arctan2(direction_norm[1], direction_norm[0])
            
            # Get current robot orientation
            q = self.current_pose.orientation
            current_yaw = self.quaternion_to_yaw(q)
            
            # Calculate heading error
            heading_error = self.wrap_angle(desired_yaw - current_yaw)
            
            # PID-style control for orientation and forward motion
            kp_lin = 0.8  # Proportional gain for linear velocity
            kp_ang = 2.0  # Proportional gain for angular velocity
            
            # Limit linear velocity based on distance to waypoint and heading error
            max_vel = self.get_parameter('linear_velocity_max').value
            linear_vel = min(max_vel * 0.8, kp_lin * distance)
            
            # Higher angular correction when far from desired heading
            angular_vel = kp_ang * heading_error * (1.0 - min(1.0, abs(heading_error) / 1.57))  # Reduce linear vel when turning sharply
            
            # Apply acceleration/deceleration limits
            if self.current_twist:
                dt = 0.05  # Assuming 20Hz control frequency
                max_acc_lin = self.get_parameter('acceleration_limit').value
                max_dec_lin = self.get_parameter('deceleration_limit').value
                max_acc_ang = max_acc_lin / 0.5  # Assume turning radius of 0.5m
                
                # Limit acceleration
                desired_diff_lin = linear_vel - self.current_twist.linear.x
                if desired_diff_lin > 0:
                    max_change_lin = max_acc_lin * dt
                else:
                    max_change_lin = max_dec_lin * dt
                linear_vel = self.current_twist.linear.x + np.clip(desired_diff_lin, -max_change_lin, max_change_lin)
                
                desired_diff_ang = angular_vel - self.current_twist.angular.z
                max_change_ang = max_acc_ang * dt
                angular_vel = self.current_twist.angular.z + np.clip(desired_diff_ang, -max_change_ang, max_change_ang)
            
            # Apply velocity limits
            linear_vel = max(0.0, min(max_vel, linear_vel))  # Only forward motion for humanoid
            angular_vel = max(-self.get_parameter('angular_velocity_max').value, 
                             min(self.get_parameter('angular_velocity_max').value, angular_vel))
            
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
        
        return cmd
    
    def has_reached_goal(self):
        """Check if robot has reached the navigation goal"""
        if not self.goal_pose or not self.current_pose:
            return False
        
        goal_pos = np.array([self.goal_pose.position.x, self.goal_pose.position.y])
        robot_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        distance = np.linalg.norm(goal_pos - robot_pos)
        tolerance = self.get_parameter('path_tolerance').value
        
        # Check position tolerance
        if distance > tolerance:
            return False
        
        # Check orientation tolerance if specified
        q = self.current_pose.orientation
        current_yaw = self.quaternion_to_yaw(q)
        
        goal_q = self.goal_pose.orientation
        goal_yaw = self.quaternion_to_yaw(goal_q)
        
        yaw_error = abs(self.wrap_angle(goal_yaw - current_yaw))
        yaw_tolerance = self.get_parameter('yaw_tolerance').value
        
        return yaw_error <= yaw_tolerance
    
    def navigation_complete(self):
        """Handle completion of navigation task"""
        self.get_logger().info('Navigation goal reached successfully!')
        self.publish_zero_velocity()
        self.navigation_active = False
        
        # Reset plans
        self.global_plan = []
        self.local_plan = []
    
    def publish_zero_velocity(self):
        """Publish zero velocity to stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def publish_global_plan(self):
        """Publish the global plan to visualization topics"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in self.global_plan:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0  # Assuming 2D navigation
            pose.pose.orientation.w = 1.0  # No rotation
            path_msg.poses.append(pose)
        
        self.global_plan_pub.publish(path_msg)
    
    def get_local_segment_of_global_plan(self):
        """Extract local segment from global plan"""
        if not self.global_plan or not self.current_pose:
            return []
        
        robot_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        # Find the closest point in global plan to current position
        global_array = np.array(self.global_plan)
        distances = np.linalg.norm(global_array[:, :2] - robot_pos, axis=1)
        
        closest_idx = np.argmin(distances)
        
        # Return next N points from closest point (local window)
        look_ahead = min(10, len(self.global_plan) - closest_idx)  # Look ahead 10 points max
        local_segment = self.global_plan[closest_idx:closest_idx + look_ahead]
        
        return local_segment
    
    def wrap_angle(self, angle):
        """Wrap angle to [-π, π] range"""
        return ((angle + np.pi) % (2 * np.pi)) - np.pi
    
    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    nav_controller = IsaacNavigationController()
    
    try:
        rclpy.spin(nav_controller)
    except KeyboardInterrupt:
        nav_controller.get_logger().info('Shutting down Isaac Navigation Controller')
    finally:
        nav_controller.publish_zero_velocity()
        nav_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Isaac ROS for Perception

### Hardware-Accelerated Perception Pipelines

Isaac ROS provides GPU-accelerated perception pipelines that are crucial for humanoid robots:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray, Detection3DArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cuda import cudart  # NVIDIA CUDA Runtime API wrapper


class IsaacPerceptionPipeline(Node):
    def __init__(self):
        super().__init__('isaac_perception_pipeline')
        
        # Perception parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detection_frequency', 10.0),  # Hz
                ('tracking_frequency', 30.0),  # Hz
                ('confidence_threshold', 0.5),
                ('tracking_iou_threshold', 0.3),
                ('use_tensorrt', True),
                ('input_image_width', 640),
                ('input_image_height', 480),
            ]
        )
        
        # Publishers
        self.detections_2d_pub = self.create_publisher(Detection2DArray, '/perception/detections_2d', 10)
        self.detections_3d_pub = self.create_publisher(Detection3DArray, '/perception/detections_3d', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/rgb/camera_info', self.camera_info_callback, 10)
        
        # Perception buffers
        self.latest_image = None
        self.latest_depth = None
        self.camera_intrinsics = None
        
        # Isaac ROS acceleration objects
        self.detection_model = None
        self.tracking_pipeline = None
        
        # Initialize Isaac-accelerated perception
        self.initialize_perception_models()
        
        # Timer for processing
        self.process_timer = self.create_timer(
            1.0 / self.get_parameter('detection_frequency').value, 
            self.process_perception)
        
        self.get_logger().info('Isaac Perception Pipeline initialized with GPU acceleration')
    
    def initialize_perception_models(self):
        """Initialize Isaac ROS accelerated perception models"""
        try:
            # Initialize CUDA context
            _, device_count = cudart.cudaGetDeviceCount()
            if device_count == 0:
                self.get_logger().warn('No CUDA devices found, falling back to CPU')
                return
            
            # In real implementation, this would initialize Isaac's
            # TensorRT-accelerated models for detection and tracking
            self.get_logger().info('Initialized CUDA context for Isaac perception acceleration')
            
            # For simulation purposes, store acceleration capabilities
            self.acceleration_capabilities = {
                'object_detection': True,
                'depth_processing': True,
                'feature_matching': True,
                'tracking': True
            }
            
        except ImportError:
            self.get_logger().warn('CUDA libraries not available, using CPU processing')
        except Exception as e:
            self.get_logger().warn(f'CUDA initialization failed: {e}')
    
    def image_callback(self, msg):
        """Process incoming RGB image"""
        self.latest_image = msg
    
    def depth_callback(self, msg):
        """Process incoming depth image"""
        self.latest_depth = msg
    
    def camera_info_callback(self, msg):
        """Process camera intrinsics"""
        self.camera_intrinsics = msg
    
    def process_perception(self):
        """Main perception processing loop with Isaac acceleration"""
        if not self.latest_image or not self.latest_depth or not self.camera_intrinsics:
            return
        
        try:
            # Process image using Isaac-accelerated pipeline
            detections_2d = self.accelerated_object_detection(self.latest_image)
            detections_3d = self.accelerated_3d_detection(
                self.latest_image, 
                self.latest_depth, 
                self.camera_intrinsics)
            
            # Publish results
            self.publish_detections_2d(detections_2d)
            self.publish_detections_3d(detections_3d)
            
        except Exception as e:
            self.get_logger().error(f'Perception processing error: {e}')
    
    def accelerated_object_detection(self, image_msg):
        """
        Simulate Isaac ROS accelerated object detection
        In reality, this would interface with Isaac's TensorRT-accelerated models
        """
        # This is a simulation - in real Isaac ROS, this would use:
        # - Isaac_ROS_DetectNet for object detection (with TensorRT acceleration)
        # - Isaac_ROS_AprilTag for marker detection (with GPU acceleration)
        
        # For simulation, return mock detections
        import random
        
        # Simulate 1-3 random detections
        num_detections = random.randint(1, 3)
        detections = []
        
        for i in range(num_detections):
            detection = {
                'class_name': random.choice(['person', 'chair', 'table', 'robot']),
                'confidence': random.uniform(0.6, 0.95),
                'bbox': {  # [x, y, width, height] in pixels
                    'x': random.randint(50, 550),
                    'y': random.randint(50, 350),
                    'width': random.randint(30, 100),
                    'height': random.randint(30, 150)
                }
            }
            
            # Only include detections above threshold
            if detection['confidence'] >= self.get_parameter('confidence_threshold').value:
                detections.append(detection)
        
        return detections
    
    def accelerated_3d_detection(self, image_msg, depth_msg, camera_info):
        """
        Simulate Isaac ROS accelerated 3D detection
        Converts 2D detections to 3D positions using depth data
        """
        detections_2d = self.accelerated_object_detection(image_msg)
        detections_3d = []
        
        # Process depth image (simplified simulation)
        for det in detections_2d:
            # Get average depth in bounding box region
            bbox = det['bbox']
            avg_depth = self.get_average_depth_in_bbox(
                depth_msg, 
                bbox, 
                camera_info)
            
            if avg_depth > 0:  # Valid depth
                # Convert pixel coordinates to 3D world coordinates
                center_x = bbox['x'] + bbox['width'] / 2
                center_y = bbox['y'] + bbox['height'] / 2
                
                # Use pinhole camera model to convert to 3D
                point_3d = self.pixel_to_3d_point(
                    center_x, 
                    center_y, 
                    avg_depth, 
                    camera_info)
                
                detection_3d = {
                    'class_name': det['class_name'],
                    'confidence': det['confidence'],
                    'position': point_3d,
                    'bbox_2d': bbox
                }
                
                detections_3d.append(detection_3d)
        
        return detections_3d
    
    def get_average_depth_in_bbox(self, depth_msg, bbox, camera_info):
        """Get average depth value within a bounding box"""
        # Convert ROS Image to NumPy array (simplified simulation)
        # In real implementation, would convert depth_msg.data properly
        # based on depth_msg.encoding
        
        # For simulation, return a random valid depth
        return random.uniform(1.0, 5.0)
    
    def pixel_to_3d_point(self, u, v, depth, camera_info):
        """Convert pixel coordinates + depth to 3D world coordinates"""
        # Extract camera intrinsics
        fx = camera_info.k[0]  # K[0, 0]
        fy = camera_info.k[4]  # K[1, 1]
        cx = camera_info.k[2]  # K[0, 2]
        cy = camera_info.k[5]  # K[1, 2]
        
        # Pinhole camera model
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return [x, y, z]
    
    def publish_detections_2d(self, detections_2d):
        """Publish 2D detections to ROS topic"""
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_rgb_optical_frame"
        
        for det in detections_2d:
            detection_msg = Detection2D()
            detection_msg.bbox.center.x = det['bbox']['x'] + det['bbox']['width'] / 2
            detection_msg.bbox.center.y = det['bbox']['y'] + det['bbox']['height'] / 2
            detection_msg.bbox.size_x = det['bbox']['width']
            detection_msg.bbox.size_y = det['bbox']['height']
            
            # Add classification result
            from vision_msgs.msg import ObjectHypothesisWithPose
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class_name']
            hypothesis.hypothesis.score = det['confidence']
            detection_msg.results.append(hypothesis)
            
            msg.detections.append(detection_msg)
        
        self.detections_2d_pub.publish(msg)
    
    def publish_detections_3d(self, detections_3d):
        """Publish 3D detections to ROS topic"""
        msg = Detection3DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_rgb_optical_frame"
        
        for det in detections_3d:
            detection_msg = Detection3D()
            
            # Set 3D position
            detection_msg.bbox.center.position.x = det['position'][0]
            detection_msg.bbox.center.position.y = det['position'][1]
            detection_msg.bbox.center.position.z = det['position'][2]
            
            # Set orientation (identity for detected objects)
            detection_msg.bbox.center.orientation.w = 1.0
            detection_msg.bbox.center.orientation.x = 0.0
            detection_msg.bbox.center.orientation.y = 0.0
            detection_msg.bbox.center.orientation.z = 0.0
            
            # Set size (simplified - using fixed size based on class)
            if det['class_name'] in ['person']:
                detection_msg.bbox.size.x = 0.6  # width
                detection_msg.bbox.size.y = 0.5  # depth
                detection_msg.bbox.size.z = 1.7  # height
            else:
                detection_msg.bbox.size.x = 0.3
                detection_msg.bbox.size.y = 0.3
                detection_msg.bbox.size.z = 0.3
            
            # Add classification result
            from vision_msgs.msg import ObjectHypothesisWithPose
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class_name']
            hypothesis.hypothesis.score = det['confidence']
            detection_msg.results.append(hypothesis)
            
            msg.detections.append(detection_msg)
        
        self.detections_3d_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    perception_node = IsaacPerceptionPipeline()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        perception_node.get_logger().info('Shutting down Isaac Perception Pipeline')
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Performance Optimization and Benchmarking

### Monitoring Isaac ROS Performance

Isaac ROS acceleration provides significant performance benefits that should be monitored:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import time
import psutil
import GPUtil


class IsaacPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('isaac_performance_monitor')
        
        # Publishers for performance metrics
        self.cpu_load_pub = self.create_publisher(Float32, '/diagnostics/cpu_load', 10)
        self.gpu_load_pub = self.create_publisher(Float32, '/diagnostics/gpu_load', 10)
        self.ram_usage_pub = self.create_publisher(Float32, '/diagnostics/ram_usage', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Timer for periodic monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_performance)
        
        self.get_logger().info('Isaac Performance Monitor initialized')
    
    def monitor_performance(self):
        """Monitor system performance with Isaac ROS acceleration"""
        # Get CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        
        # Get RAM usage
        ram_percent = psutil.virtual_memory().percent
        
        # Get GPU usage (if available)
        gpu_percent = 0.0
        gpus = GPUtil.getGPUs()
        if gpus:
            gpu_percent = gpus[0].load * 100 if gpus[0].load is not None else 0.0
        
        # Publish individual metrics
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_load_pub.publish(cpu_msg)
        
        gpu_msg = Float32()
        gpu_msg.data = float(gpu_percent)
        self.gpu_load_pub.publish(gpu_msg)
        
        ram_msg = Float32()
        ram_msg.data = float(ram_percent)
        self.ram_usage_pub.publish(ram_msg)
        
        # Publish diagnostic message
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        # CPU status
        cpu_diag = DiagnosticStatus()
        cpu_diag.name = "CPU Usage"
        cpu_diag.level = DiagnosticStatus.OK if cpu_percent < 80 else DiagnosticStatus.WARN
        cpu_diag.message = "OK" if cpu_percent < 80 else "High CPU usage"
        cpu_diag.hardware_id = "cpu0"
        cpu_diag.values = [{
            "key": "Load (%)",
            "value": str(cpu_percent)
        }]
        diag_msg.status.append(cpu_diag)
        
        # GPU status
        gpu_diag = DiagnosticStatus()
        gpu_diag.name = "GPU Usage"
        gpu_diag.level = DiagnosticStatus.OK if gpu_percent < 80 else DiagnosticStatus.WARN
        gpu_diag.message = "OK" if gpu_percent < 80 else "High GPU usage"
        gpu_diag.hardware_id = "gpu0"
        gpu_diag.values = [{
            "key": "Load (%)",
            "value": str(gpu_percent)
        }]
        diag_msg.status.append(gpu_diag)
        
        # RAM status
        ram_diag = DiagnosticStatus()
        ram_diag.name = "RAM Usage"
        ram_diag.level = DiagnosticStatus.OK if ram_percent < 80 else DiagnosticStatus.WARN
        ram_diag.message = "OK" if ram_percent < 80 else "High RAM usage"
        ram_diag.hardware_id = "system"
        ram_diag.values = [{
            "key": "Usage (%)",
            "value": str(ram_percent)
        }]
        diag_msg.status.append(ram_diag)
        
        self.diag_pub.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    monitor = IsaacPerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Stopping Isaac Performance Monitor')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Quality Assurance for Isaac ROS Systems

### Validation Testing

When using Isaac ROS acceleration, validation is crucial:

```python
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseStamped
import numpy as np


class IsaacRosValidationTestSuite(unittest.TestCase):
    """Comprehensive test suite for Isaac ROS accelerated systems"""
    
    def setUp(self):
        """Set up the test environment"""
        rclpy.init()
        self.test_node = TestIsaacValidationNode()
    
    def tearDown(self):
        """Clean up after tests"""
        self.test_node.destroy_node()
        rclpy.shutdown()
    
    def test_lidar_accuracy(self):
        """Test LiDAR simulation accuracy against ground truth"""
        # In a real test, this would compare simulated LiDAR readings
        # against known distances to objects in the simulation
        test_distances = [1.0, 2.0, 3.0, 5.0]  # meters
        expected_errors = []
        
        for distance in test_distances:
            # Simulate generating LiDAR data for an object at known distance
            simulated_error = self.evaluate_lidar_accuracy(distance)
            
            # Verify error is within acceptable bounds (e.g., ±3cm)
            self.assertLess(abs(simulated_error), 0.03,
                          f"LiDAR error {simulated_error} exceeds threshold for distance {distance}m")
            
            expected_errors.append(simulated_error)
        
        # Verify that error doesn't increase proportionally with distance beyond threshold
        max_error = max(abs(err) for err in expected_errors)
        self.assertLess(max_error, 0.1, "Maximum LiDAR error exceeds acceptable threshold")
    
    def evaluate_lidar_accuracy(self, true_distance):
        """Evaluate LiDAR accuracy for a given true distance (simulation)"""
        # This would interface with the actual simulation in a real test
        # For now, return a simulated error value
        import random
        # Simulate realistic LiDAR error characteristics
        systematic_error = 0.005  # 5mm systematic bias
        random_error = random.gauss(0, 0.01)  # 1cm standard deviation
        distance_dependent_error = 0.001 * true_distance  # 1mm per meter
        
        return systematic_error + random_error + distance_dependent_error
    
    def test_depth_camera_precision(self):
        """Test depth camera precision for 3D reconstruction"""
        # Test that depth camera provides accurate distance measurements
        test_points = [
            [0.5, 0.0, 1.0],   # 1m in front
            [1.0, 0.5, 1.5],   # Diagonal point
            [0.0, 0.0, 3.0],   # Far point
        ]
        
        for point in test_points:
            measured_depth = self.evaluate_depth_camera_at_point(point)
            true_depth = point[2]  # Z coordinate is the true depth
            
            error = abs(measured_depth - true_depth)
            self.assertLess(error, 0.05,  # 5cm threshold
                          f"Depth camera error {error:.3f}m exceeds threshold at point {point}")
    
    def evaluate_depth_camera_at_point(self, point):
        """Evaluate depth camera at a specific 3D point (simulation)"""
        # Simulate depth measurement with realistic noise characteristics
        import random
        true_depth = point[2]
        
        # Add depth-dependent noise
        noise_std = 0.005 + (0.01 * true_depth)  # Noise increases with distance
        noise = random.gauss(0, noise_std)
        
        return true_depth + noise
    
    def test_navigation_performance(self):
        """Test Isaac-accelerated navigation performance"""
        # Test that navigation maintains performance under load
        start_time = time.time()
        
        # Simulate navigation task for 10 seconds
        for i in range(100):  # 100 iterations at 10Hz
            # Simulate navigation planning and control
            path_found = self.simulate_navigation_planning()
            self.assertTrue(path_found, f"Navigation planning failed at iteration {i}")
            
            # Sleep briefly to simulate real-time processing
            time.sleep(0.1)
        
        end_time = time.time()
        elapsed = end_time - start_time
        
        # Verify that the task completed within expected time (+/- 20%)
        expected_time = 10.0  # 10 seconds
        tolerance = 2.0  # Allow 2 seconds tolerance
        
        self.assertLess(elapsed, expected_time + tolerance,
                      f"Navigation test took {elapsed:.2f}s, exceeding expected time")
        
        # Verify real-time factor is good enough
        real_time_factor = expected_time / elapsed
        self.assertGreater(real_time_factor, 0.8,
                         f"Real-time factor {real_time_factor:.2f} is too low - system is lagging")


class TestIsaacValidationNode(Node):
    """Test node for Isaac ROS validation"""
    def __init__(self):
        super().__init__('test_isaac_validation')
        
        # This node would handle communication with Isaac ROS systems
        # during validation tests
        pass


def run_validation_tests():
    """Run the complete validation test suite"""
    test_suite = unittest.TestSuite()
    test_suite.addTest(unittest.makeSuite(IsaacRosValidationTestSuite))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_validation_tests()
    exit(0 if success else 1)
```

## Chapter Summary

Isaac ROS acceleration provides critical hardware acceleration for the computational needs of humanoid robots. By leveraging NVIDIA's GPU architecture, Isaac ROS enables:

- Real-time visual SLAM with IMU integration
- Accelerated navigation planning and obstacle avoidance
- High-performance perception pipelines for object detection and 3D scene understanding
- Efficient processing of multiple sensor streams simultaneously

The integration of Isaac ROS with traditional ROS 2 systems enhances performance while maintaining compatibility with the broader ROS ecosystem. This acceleration is particularly valuable for humanoid robots, which must process multiple sensor streams in real-time to maintain balance and navigate complex environments safely.

Proper validation ensures that the simulated sensors provide accurate and reliable data that can be used for both training and deployment of robotic systems.

## Learning Objectives

After completing this chapter, readers should be able to:

1. Understand the architecture and components of Isaac ROS acceleration
2. Configure LiDAR sensors with realistic parameters in simulation environments
3. Set up and validate depth camera simulation for 3D perception
4. Implement Isaac ROS-accelerated navigation systems for humanoid robots
5. Configure perception pipelines using TensorRT acceleration
6. Validate sensor simulation accuracy and performance
7. Monitor and optimize Isaac ROS system performance
8. Understand the benefits and limitations of accelerated robotics systems
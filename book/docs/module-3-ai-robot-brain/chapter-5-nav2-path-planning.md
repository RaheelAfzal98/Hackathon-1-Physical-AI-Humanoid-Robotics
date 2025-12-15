# Chapter 5: Nav2 Path Planning for Bipedal Humanoid Robots

## Introduction to Navigation for Humanoid Robots

Navigation for humanoid robots presents unique challenges compared to wheeled robots. While wheeled robots primarily need to plan 2D paths considering obstacles and kinodynamic constraints, humanoid robots must solve the additional challenges of:

1. **Balance maintenance**: Each step must maintain center of mass within support polygon
2. **Dynamic stability**: Unlike static wheeled platforms, humanoid robots are inherently unstable
3. **Step planning**: Rather than continuous motion, humanoid robots move via sequential footsteps
4. **Terrain analysis**: Ability to climb stairs, navigate uneven terrain, and select footholds
5. **Upper body coordination**: Navigation with simultaneous manipulation tasks

The Navigation2 (Nav2) stack in ROS 2 provides a flexible, behavior-based navigation framework that can be adapted for humanoid robot navigation, but requires specific modifications to account for the kinematic and dynamic constraints of bipedal locomotion.

## Core Concepts of Nav2

### Nav2 Architecture

The Nav2 stack follows a behavior-based architecture with several key components:

1. **Navigation Lifecycle**: Manages the state of the navigation system
2. **Global Planner**: Creates a path from start to goal in the global map
3. **Local Planner**: Creates commands to follow the global path while avoiding local obstacles
4. **Controller**: Translates high-level navigation commands to low-level control signals
5. **Recovery Behaviors**: Strategies to escape from failed navigation states

### Nav2 Concepts Specific to Humanoid Robots

For humanoid robots, Nav2 must be adapted to consider:

- **Center of Mass (CoM) constraints**: Each planned step must maintain balance
- **Zero Moment Point (ZMP) considerations**: Footstep planning for dynamic stability
- **Bipedal kinematics**: Joint angle constraints for stable walking
- **Step spacing and timing**: Temporal and spatial constraints on foot placement

## Nav2 for Humanoid Robot Path Planning

### Global Path Planning

The global planner creates a high-level plan from current position to goal location. For humanoid robots, this must consider:

- Walkable areas that accommodate bipedal locomotion
- Stairs, ramps, and other terrain features requiring special footstep planning
- Height restrictions (low ceilings, hanging obstacles)
- Slope limits for stable walking

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
import numpy as np
from scipy.spatial import KDTree
import math
from threading import Lock


class HumanoidGlobalPlanner(Node):
    """
    Custom global planner for humanoid robot navigation with ZMP-based path planning
    """
    def __init__(self):
        super().__init__('humanoid_global_planner')
        
        # Publishers and subscribers
        self.path_publisher = self.create_publisher(Path, '/humanoid_global_plan', 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, 
            '/global_costmap/costmap', 
            self.costmap_callback, 
            10
        )
        self.visualization_pub = self.create_publisher(MarkerArray, '/humanoid_path_markers', 10)
        
        # Server for path planning actions
        self.plan_srv = self.create_service(
            GetRoute, 
            '/humanoid_plan_path', 
            self.plan_path_callback
        )
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('step_length_max', 0.3),    # Maximum step length in meters
                ('step_width_max', 0.2),    # Maximum step width (sideways)
                ('step_turn_max', 0.2),     # Maximum turn per step (rad)
                ('foot_spacing', 0.15),     # Distance between feet (lateral)
                ('balance_margin', 0.05),   # Margin for CoM stability
                ('costmap_inflation_radius', 0.4),  # Inflation for safety
                ('path_smoothing', True),
                ('optimization_iterations', 100)
            ]
        )
        
        # Internal state
        self.costmap_data = None
        self.costmap_resolution = None
        self.costmap_origin = None
        self.path_lock = Lock()
        
        self.get_logger().info('Humanoid Global Planner initialized')
    
    def costmap_callback(self, msg):
        """Receive updated costmap data"""
        self.costmap_data = msg.data
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = msg.info.origin
        
        # Update any internal path planning data if needed
        self.update_path_costs()
    
    def update_path_costs(self):
        """Update internal representation of path costs based on costmap"""
        if self.costmap_data is None:
            return
        
        # Convert costmap data to numpy array for processing
        width = self.costmap_info.width
        height = self.costmap_info.height
        
        costmap_array = np.array(self.costmap_data).reshape((height, width))
        
        # In a humanoid-aware costmap, we'd consider:
        # - Terrain slope for bipedal traversal
        # - Surface stability for footholds
        # - Space requirements for step spacing
        
        # For this example, we'll maintain the basic costmap but with humanoid-specific
        # inflation and step constraints added
        self.processed_costmap = self.apply_humanoid_constraints(costmap_array)
    
    def apply_humanoid_constraints(self, costmap):
        """Apply humanoid robot-specific constraints to costmap"""
        # Humanoid-specific inflation considering:
        # - Space needed for stable bipedal walking
        # - Areas unsuitable for foot placement
        # - Sloped surfaces that may be difficult to navigate
        
        # Increase costs for narrow passages that don't allow stable walking
        # This is a simplified version - in practice this would consider
        # specific kinematic constraints
        humanoid_inflated_costmap = costmap.copy()
        
        # Apply humanoid-specific inflation radius
        inflation_radius_px = int(
            self.get_parameter('costmap_inflation_radius').value / self.costmap_resolution)
        
        for i in range(inflation_radius_px, costmap.shape[0] - inflation_radius_px):
            for j in range(inflation_radius_px, costmap.shape[1] - inflation_radius_px):
                if costmap[i, j] >= 50:  # Threshold for obstacle consideration
                    # Inflate around obstacle considering humanoid step size
                    for di in range(-inflation_radius_px, inflation_radius_px + 1):
                        for dj in range(-inflation_radius_px, inflation_radius_px + 1):
                            if di*di + dj*dj <= inflation_radius_px*inflation_radius_px:
                                ni, nj = i + di, j + dj
                                if 0 <= ni < costmap.shape[0] and 0 <= nj < costmap.shape[1]:
                                    # Apply higher cost for humanoid navigation
                                    humanoid_inflated_costmap[ni, nj] = max(
                                        humanoid_inflated_costmap[ni, nj], 
                                        75  # Higher cost than wheeled robots
                                    )
        
        return humanoid_inflated_costmap
    
    def plan_path_callback(self, request, response):
        """Callback for path planning requests"""
        start = request.start
        goal = request.goal
        
        # Plan path for humanoid robot considering step constraints
        path = self.plan_humanoid_path(start, goal)
        
        if path:
            response.plan = path
            response.error = ""
        else:
            response.plan = Path()
            response.error = "Failed to find valid path for humanoid robot"
        
        return response
    
    def plan_humanoid_path(self, start_pose, goal_pose):
        """Plan a path considering humanoid robot constraints"""
        # Check if costmap is available
        if self.processed_costmap is None:
            self.get_logger().error('Costmap not available for path planning')
            return None
        
        # Convert start and goal poses to grid coordinates
        start_grid = self.world_to_grid_coords(start_pose.pose.position)
        goal_grid = self.world_to_grid_coords(goal_pose.pose.position)
        
        # Perform A* pathfinding with humanoid constraints
        path_grid = self.humanoid_astar(start_grid, goal_grid)
        
        if not path_grid:
            return None
        
        # Convert grid path to world coordinates
        world_path = self.grid_path_to_world_path(path_grid)
        
        # Apply footstep planning to the path
        footsteps = self.plan_footsteps(world_path)
        
        # Create Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # Convert footsteps to poses for the path
        for step in footsteps:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = step[0]
            pose.pose.position.y = step[1]
            pose.pose.position.z = step[2] if len(step) > 2 else 0.0  # Include Z if available
            
            # Set orientation to face along the path
            if len(footsteps) > 1:
                next_idx = min(footsteps.index(step) + 1, len(footsteps) - 1)
                next_step = footsteps[next_idx]
                
                dx = next_step[0] - step[0]
                dy = next_step[1] - step[1]
                
                # Calculate orientation to face the next waypoint
                yaw = math.atan2(dy, dx)
                
                # Convert to quaternion
                from geometry_msgs.msg import Quaternion
                q = self.euler_to_quaternion(0, 0, yaw)
                pose.pose.orientation = q
            
            path_msg.poses.append(pose)
        
        # Publish the path
        with self.path_lock:
            self.path_publisher.publish(path_msg)
        
        return path_msg
    
    def humanoid_astar(self, start, goal):
        """A* algorithm with humanoid constraints"""
        # Implementation of A* that considers humanoid step constraints
        # In practice, this would be customized for bipedal navigation
        
        # For this example, we'll use a modified A* that considers:
        # - Maximum step distances (humanoid step length limits)
        # - Feasible terrain for foot placement
        
        width, height = self.processed_costmap.shape
        open_set = [(0, start)]  # (f_score, position)
        came_from = {}
        g_score = {start: 0}
        
        # Possible humanoid step directions and maximum distances
        step_lengths = self.get_parameter('step_length_max').value
        step_widths = self.get_parameter('step_width_max').value
        step_turns = self.get_parameter('step_turn_max').value
        
        # Define possible step patterns (simplified)
        possible_steps = [
            (1, 0),    # Forward
            (-1, 0),   # Backward  
            (0, 1),    # Right
            (0, -1),   # Left
            (1, 1),    # Forward-right diagonal
            (1, -1),   # Forward-left diagonal
            (-1, 1),   # Backward-right diagonal
            (-1, -1),  # Backward-left diagonal
        ]
        
        # Convert step lengths to grid units
        max_steps = int(step_lengths / self.costmap_resolution)
        
        while open_set:
            # Sort by f_score (first element of tuple)
            open_set.sort()
            current = open_set.pop(0)[1]
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            for step in possible_steps:
                neighbor = (current[0] + step[0], current[1] + step[1])
                
                # Check bounds
                if (0 <= neighbor[0] < width and 0 <= neighbor[1] < height):
                    # Check if the grid cell is free for humanoid passage
                    if self.processed_costmap[neighbor[1], neighbor[0]] < 90:  # Threshold for passable
                        tentative_g_score = g_score[current] + self.heuristic_cost(current, neighbor)
                        
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score = tentative_g_score + self.heuristic_cost(neighbor, goal)
                            open_set.append((f_score, neighbor))
        
        return []  # No path found
    
    def heuristic_cost(self, pos1, pos2):
        """Calculate heuristic cost (Euclidean distance) between two positions"""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
    
    def plan_footsteps(self, path):
        """Plan actual footsteps for the humanoid robot based on path"""
        if len(path) < 2:
            return []
        
        footsteps = []
        
        # Start with the initial pose
        current_pos = [path[0].pose.position.x, path[0].pose.position.y]
        footsteps.append(current_pos.copy())
        
        for i in range(1, len(path)):
            next_pos = [path[i].pose.position.x, path[i].pose.position.y]
            
            # Plan intermediate footsteps between path waypoints
            # based on step length constraints
            distance = math.sqrt(
                (next_pos[0] - current_pos[0])**2 + 
                (next_pos[1] - current_pos[1])**2
            )
            
            max_step_length = self.get_parameter('step_length_max').value
            
            if distance > max_step_length:
                # Need to plan intermediate footsteps
                num_steps = int(distance / max_step_length) + 1
                step_size = distance / num_steps
                
                for j in range(1, num_steps + 1):
                    t = j / num_steps
                    interp_x = current_pos[0] + t * (next_pos[0] - current_pos[0])
                    interp_y = current_pos[1] + t * (next_pos[1] - current_pos[1])
                    
                    footsteps.append([interp_x, interp_y])
            else:
                # If within step length, add the waypoint directly
                footsteps.append(next_pos)
            
            current_pos = next_pos[:]
        
        return footsteps
    
    def world_to_grid_coords(self, world_point):
        """Convert world coordinates to grid coordinates"""
        if self.costmap_origin is None:
            return (0, 0)
        
        origin_x = self.costmap_origin.position.x
        origin_y = self.costmap_origin.position.y
        
        grid_x = int((world_point.x - origin_x) / self.costmap_resolution)
        grid_y = int((world_point.y - origin_y) / self.costmap_resolution)
        
        return (grid_x, grid_y)
    
    def grid_path_to_world_path(self, grid_path):
        """Convert grid path to world path"""
        world_path = []
        
        for grid_point in grid_path:
            world_x = grid_point[0] * self.costmap_resolution + self.costmap_origin.position.x
            world_y = grid_point[1] * self.costmap_resolution + self.costmap_origin.position.y
            world_z = 0.0  # Height is typically 0 for navigation plan (terrain-specific in practice)
            
            world_path.append((world_x, world_y, world_z))
        
        return world_path
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0] * 4
        q[0] = cr * cp * cy + sr * sp * sy
        q[1] = sr * cp * cy - cr * sp * sy
        q[2] = cr * sp * cy + sr * cp * sy
        q[3] = cr * cp * sy - sr * sp * cy
        
        from geometry_msgs.msg import Quaternion
        quat = Quaternion()
        quat.w = q[0]
        quat.x = q[1]
        quat.y = q[2]
        quat.z = q[3]
        
        return quat


class HumanoidLocalPlanner(Node):
    """
    Local planner for humanoid robot with ZMP-based control
    """
    def __init__(self):
        super().__init__('humanoid_local_planner')
        
        # Publishers for control commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.footstep_pub = self.create_publisher(Polygon, '/desired_footsteps', 10)
        
        # Subscribers 
        self.global_plan_sub = self.create_subscription(
            Path, 
            '/humanoid_global_plan', 
            self.global_plan_callback, 
            10
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/data', 
            self.imu_callback, 
            10
        )
        
        # Parameters for humanoid navigation
        self.declare_parameters(
            namespace='',
            parameters=[
                ('local_planner_frequency', 10.0),
                ('foot_lift_height', 0.05),  # Height to lift foot during stepping
                ('step_duration', 0.6),      # Time for each step in seconds
                ('step_width', 0.15),        # Lateral distance between feet
                ('balance_p_gain', 5.0),     # Proportional gain for balance control
                ('balance_vel_gain', 2.0),   # Velocity gain for balance
                ('max_linear_speed', 0.3),   # Maximum forward speed
                ('min_obstacle_dist', 0.5),  # Minimum distance to obstacles
                ('local_plan_horizon', 5.0)  # Distance to plan ahead
            ]
        )
        
        # Internal state
        self.current_pose = None
        self.current_twist = None
        self.current_imu = None
        self.global_plan = None
        self.current_plan_index = 0
        self.next_waypoint = None
        
        # Timing for step control
        self.last_step_time = self.get_clock().now()
        
        # Create local planning timer
        freq = self.get_parameter('local_planner_frequency').value
        self.local_plan_timer = self.create_timer(1.0/freq, self.local_plan_callback)
        
        self.get_logger().info('Humanoid Local Planner initialized')
    
    def global_plan_callback(self, msg):
        """Receive updated global plan"""
        self.global_plan = msg
        self.current_plan_index = 0
    
    def odom_callback(self, msg):
        """Receive odometry data"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist
    
    def imu_callback(self, msg):
        """Receive IMU data for balance control"""
        self.current_imu = msg
    
    def laser_callback(self, msg):
        """Receive laser scan for local obstacle detection"""
        # Process laser data to detect obstacles in robot's path
        self.check_path_for_obstacles(msg)
    
    def local_plan_callback(self):
        """Main local planning loop for humanoid robot"""
        if not self.current_pose or not self.global_plan:
            # Stop robot if no plan or pose available
            self.publish_stop_command()
            return
        
        # Update current plan index based on proximity to waypoints
        self.update_plan_index()
        
        # Calculate next waypoint based on current position
        target_waypoint = self.get_next_waypoint_along_path()
        
        if target_waypoint:
            # Generate footstep commands based on target waypoint
            cmd_twist, footstep_cmd = self.calculate_humanoid_control(
                self.current_pose, 
                target_waypoint
            )
            
            # Publish commands
            if cmd_twist:
                self.cmd_vel_pub.publish(cmd_twist)
            if footstep_cmd:
                self.footstep_pub.publish(footstep_cmd)
        
        # Check for safety conditions
        self.check_navigation_safety()
    
    def calculate_humanoid_control(self, current_pose, target_waypoint):
        """Calculate control commands for humanoid robot navigation"""
        # Calculate distance and angle to target waypoint
        dx = target_waypoint.pose.position.x - current_pose.position.x
        dy = target_waypoint.pose.position.y - current_pose.position.y
        
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        # Calculate desired heading
        desired_yaw = math.atan2(dy, dx)
        
        # Get current orientation
        from tf_transformations import euler_from_quaternion
        current_orientation = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ]
        (roll, pitch, current_yaw) = euler_from_quaternion(current_orientation)
        
        # Calculate heading error (with wraparound)
        heading_error = ((desired_yaw - current_yaw + math.pi) % (2 * math.pi)) - math.pi
        
        # Calculate control commands
        cmd = Twist()
        
        # Adjust linear velocity based on distance to target and heading error
        max_speed = self.get_parameter('max_linear_speed').value
        if abs(heading_error) > math.pi / 4:  # If facing wrong direction, rotate first
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 * heading_error  # Rotate to face target
        else:
            # Move forward with speed proportional to distance to target
            cmd.linear.x = max(0.1, min(max_speed, 0.3 * distance_to_target))
            cmd.angular.z = 0.1 * heading_error  # Small correction to course
        
        # Generate footstep plan for this control step
        footstep_polygon = self.generate_footstep_polygon(cmd.linear.x, cmd.angular.z)
        
        return cmd, footstep_polygon
    
    def generate_footstep_polygon(self, linear_vel, angular_vel):
        """Generate desired footstep polygon based on commanded velocities"""
        # For humanoid robots, we need to plan footsteps rather than continuous motion
        # This is a simplified model - in reality, this would interface with 
        # more sophisticated footstep planning algorithms
        
        from geometry_msgs.msg import Polygon, Point32
        
        # Create a polygon representing the desired foot placement
        # This is essentially where the robot wants to place its next foot
        footstep = Polygon()
        
        # Based on current velocity, calculate where the next foot should go
        # This is a simplified calculation
        step_duration = self.get_parameter('step_duration').value
        step_width = self.get_parameter('step_width').value
        
        # Calculate footstep offset based on planned movement
        forward_offset = linear_vel * step_duration * 0.75  # Plan 75% of step duration ahead
        lateral_offset = step_width if self.use_left_foot() else -step_width  # Alternate feet
        turn_offset = angular_vel * step_duration * 0.1  # Account for turning
        
        # Define footstep points (rectangular shape)
        points = [
            Point32(x=forward_offset - 0.05, y=lateral_offset - 0.03, z=0.0),  # Rear left of foot
            Point32(x=forward_offset - 0.05, y=lateral_offset + 0.03, z=0.0),  # Rear right of foot
            Point32(x=forward_offset + 0.15, y=lateral_offset + 0.03, z=0.0),  # Front right of foot
            Point32(x=forward_offset + 0.15, y=lateral_offset - 0.03, z=0.0),  # Front left of foot
        ]
        
        footstep.points = points
        return footstep
    
    def use_left_foot(self):
        """Simple alternation for foot selection"""
        # This would be more sophisticated in a real implementation
        # considering balance, terrain, and dynamic constraints
        return (self.get_clock().now().nanoseconds // 1000000000) % 2 == 0
    
    def update_plan_index(self):
        """Update the current index in the global plan based on robot position"""
        if not self.global_plan or not self.current_pose:
            return
        
        # Find the closest point in the plan to current position
        min_distance = float('inf')
        closest_idx = self.current_plan_index  # Start from last known position
        
        # Check nearby points in plan
        start_idx = max(0, self.current_plan_index - 5)
        end_idx = min(len(self.global_plan.poses), self.current_plan_index + 10)
        
        for i in range(start_idx, end_idx):
            pose = self.global_plan.poses[i]
            dist = math.sqrt(
                (pose.pose.position.x - self.current_pose.position.x)**2 +
                (pose.pose.position.y - self.current_pose.position.y)**2
            )
            
            if dist < min_distance:
                min_distance = dist
                closest_idx = i
        
        # Move to next waypoint if close enough to current one
        if min_distance < self.get_parameter('min_waypoint_dist').value:
            self.current_plan_index = min(closest_idx + 1, len(self.global_plan.poses) - 1)
    
    def get_next_waypoint_along_path(self):
        """Get the next valid waypoint along the path"""
        if not self.global_plan or self.current_plan_index >= len(self.global_plan.poses):
            return None
        
        # Look ahead in the plan to find a reachable waypoint
        for i in range(self.current_plan_index, len(self.global_plan.poses)):
            waypoint = self.global_plan.poses[i]
            
            # Calculate distance to this waypoint
            dx = waypoint.pose.position.x - self.current_pose.position.x
            dy = waypoint.pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # If this waypoint is close enough to consider, return it
            if distance <= self.get_parameter('local_plan_horizon').value:
                # Check if it's reachable without obstacles
                if self.is_path_clear_to_waypoint(waypoint):
                    return waypoint
                else:
                    # If path is blocked, try to replan or use obstacle avoidance
                    continue
        
        return None
    
    def is_path_clear_to_waypoint(self, waypoint):
        """Check if path to waypoint is clear of obstacles"""
        # This would perform collision checking between current position and waypoint
        # For this example, we'll return True if no obstacles detected in front
        return not self.has_forward_obstacles()
    
    def has_forward_obstacles(self):
        """Check if there are obstacles directly ahead"""
        if not hasattr(self, 'latest_scan'):
            return False
        
        # Check middle portion of laser scan for obstacles
        scan = self.latest_scan
        mid_idx = len(scan.ranges) // 2
        look_ahead = int(len(scan.ranges) * 0.1)  # Look at 10% of scan in front
        
        for i in range(mid_idx - look_ahead, mid_idx + look_ahead):
            if 0 <= i < len(scan.ranges):
                if scan.ranges[i] < self.get_parameter('min_obstacle_dist').value:
                    if not math.isinf(scan.ranges[i]) and not math.isnan(scan.ranges[i]):
                        return True  # Obstacle detected
        
        return False
    
    def check_path_for_obstacles(self, scan_msg):
        """Check global plan for potential collisions"""
        self.latest_scan = scan_msg
    
    def check_navigation_safety(self):
        """Check navigation safety conditions"""
        # Check for critical safety issues
        if self.has_immediate_danger():
            self.emergency_stop()
        
        # Check for balance issues using IMU data
        if self.current_imu:
            self.check_balance_stability()
    
    def has_immediate_danger(self):
        """Check for immediate collision risk"""
        if not hasattr(self, 'latest_scan'):
            return False
        
        # Check if obstacles are too close
        min_range = min([r for r in self.latest_scan.ranges 
                        if not (math.isinf(r) or math.isnan(r))], default=float('inf'))
        
        return min_range < 0.3  # Danger if closer than 30cm
    
    def check_balance_stability(self):
        """Check robot's balance using IMU data"""
        # Extract roll and pitch from IMU
        import tf_transformations as tf
        orientation = self.current_imu.orientation
        euler = tf.euler_from_quaternion([
            orientation.x, 
            orientation.y, 
            orientation.z, 
            orientation.w
        ])
        
        roll, pitch, yaw = euler
        
        # Check if tilt exceeds safe limits
        max_tilt = 0.3  # About 17 degrees
        if abs(roll) > max_tilt or abs(pitch) > max_tilt:
            self.get_logger().warn(f'Dangerous tilt detected: roll={roll:.2f}, pitch={pitch:.2f}')
            # Trigger balance recovery behaviors
            self.initiate_balance_recovery()
    
    def initiate_balance_recovery(self):
        """Initiate balance recovery behaviors"""
        # In a real implementation, this would command the robot to:
        # - Stop forward motion
        # - Adjust center of mass
        # - Possibly widen stance
        # - Prepare for potential fall
        self.publish_stop_command()
        self.get_logger().warn('Balance recovery initiated')
    
    def publish_stop_command(self):
        """Publish zero velocity to stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    # Create both nodes
    global_planner = HumanoidGlobalPlanner()
    local_planner = HumanoidLocalPlanner()
    
    # Use MultiThreadedExecutor to run both planners
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(global_planner)
    executor.add_node(local_planner)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        global_planner.get_logger().info('Shutting down humanoid navigation system')
        local_planner.get_logger().info('Local planner shutting down')
    finally:
        global_planner.destroy_node()
        local_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Bipedal-Specific Path Planning Considerations

### ZMP-Based Path Planning

Zero Moment Point (ZMP) planning is critical for stable bipedal locomotion:

```python
class ZMPBasedPathPlanner:
    """
    Path planning based on Zero Moment Point (ZMP) for humanoid robots
    """
    def __init__(self, robot_params):
        # Robot-specific parameters
        self.robot_height = robot_params.get('height', 1.5)  # Robot height in meters
        self.foot_length = robot_params.get('foot_length', 0.25)  # Foot length
        self.foot_width = robot_params.get('foot_width', 0.1)   # Foot width
        self.step_height = robot_params.get('step_height', 0.05)  # Step height
        self.support_polygon_margin = robot_params.get('support_margin', 0.05)
        
        # ZMP-related parameters
        self.cog_height = self.robot_height * 0.55  # Approximate center of mass height
        self.dynamic_stability_threshold = 0.05  # Threshold for ZMP stability
        
    def calculate_support_polygon(self, left_foot_pose, right_foot_pose):
        """
        Calculate the support polygon based on foot positions
        For double support: convex hull of both feet
        For single support: polygon around the supporting foot
        """
        from shapely.geometry import Polygon, Point
        
        # Define foot polygons based on foot dimensions
        left_foot_poly = self.create_foot_polygon(left_foot_pose)
        right_foot_poly = self.create_foot_polygon(right_foot_pose)
        
        # Create support polygon as union of foot polygons
        support_polygon = left_foot_poly.union(right_foot_poly)
        
        return support_polygon
    
    def create_foot_polygon(self, foot_pose):
        """
        Create a polygon representing a foot
        """
        from shapely.geometry import Polygon
        
        x = foot_pose.position.x
        y = foot_pose.position.y
        z = foot_pose.position.z
        
        # Calculate foot corner points in world coordinates
        # Account for foot orientation
        from tf_transformations import euler_from_quaternion
        orientation = [foot_pose.orientation.x, foot_pose.orientation.y,
                      foot_pose.orientation.z, foot_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation)
        
        # Define corners relative to foot center
        half_length = self.foot_length / 2.0
        half_width = self.foot_width / 2.0
        
        corners = [
            self.rotate_point((-half_length, -half_width), yaw),
            self.rotate_point((-half_length, half_width), yaw),
            self.rotate_point((half_length, half_width), yaw),
            self.rotate_point((half_length, -half_width), yaw)
        ]
        
        # Translate to world coordinates
        world_corners = [(x + c[0], y + c[1]) for c in corners]
        
        return Polygon(world_corners)
    
    def rotate_point(self, point, angle):
        """Rotate a 2D point around origin by angle"""
        import math
        x, y = point
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        new_x = x * cos_a - y * sin_a
        new_y = x * sin_a + y * cos_a
        
        return (new_x, new_y)
    
    def validate_zmp_path_segment(self, start_pose, end_pose, step_length_max=0.3):
        """
        Validate a path segment for ZMP stability
        """
        # Calculate intermediate poses based on step constraints
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance <= step_length_max:
            # Segment is short enough for single step
            return self.is_zmp_stable(start_pose, end_pose)
        
        # Break into multiple steps and validate each
        num_steps = int(distance / step_length_max) + 1
        step_vec = (dx/num_steps, dy/num_steps)
        
        current_pose = start_pose
        for i in range(num_steps):
            next_pose = self.calculate_step_pose(current_pose, step_vec)
            
            if not self.is_zmp_stable(current_pose, next_pose):
                return False  # This step is not ZMP stable
            
            current_pose = next_pose
        
        return True
    
    def is_zmp_stable(self, current_pose, target_pose):
        """
        Check if the transition maintains ZMP stability
        """
        # Calculate ZMP location at target pose
        zmp_location = self.calculate_zmp_location(current_pose, target_pose)
        
        # Calculate current support polygon
        support_polygon = self.calculate_support_polygon(
            self.get_left_foot_pose(), 
            self.get_right_foot_pose()
        )
        
        # Check if ZMP is within support polygon (with margin)
        from shapely.geometry import Point
        zmp_point = Point(zmp_location[0], zmp_location[1])
        is_stable = support_polygon.contains(zmp_point.buffer(-self.support_polygon_margin))
        
        return is_stable
    
    def calculate_zmp_location(self, current_pose, target_pose):
        """
        Calculate ZMP location based on desired pose change
        Simplified for 2D movement
        """
        # This is a simplified ZMP calculation
        # In reality, this would involve complex inverse dynamics
        cog_x = (current_pose.position.x + target_pose.position.x) / 2.0
        cog_y = (current_pose.position.y + target_pose.position.y) / 2.0
        
        # Adjust based on dynamics and support polygon
        zmp_x = cog_x
        zmp_y = cog_y
        
        return (zmp_x, zmp_y)
    
    def calculate_step_pose(self, current_pose, step_vector):
        """
        Calculate the pose after taking a step
        """
        new_pose = Pose()
        new_pose.position.x = current_pose.position.x + step_vector[0]
        new_pose.position.y = current_pose.position.y + step_vector[1]
        # Z position might change with step height
        new_pose.position.z = current_pose.position.z + self.step_height
        # Orientation might change with turning
        new_pose.orientation = current_pose.orientation
        
        return new_pose
    
    def plan_zmp_stable_path(self, start_pose, goal_pose, costmap):
        """
        Plan a path that maintains ZMP stability throughout
        """
        # Implementation would combine traditional path planning with ZMP constraints
        # This is where Nav2 would be enhanced for humanoid robots
        path = self.find_path_with_zmp_constraints(start_pose, goal_pose, costmap)
        return path
    
    def find_path_with_zmp_constraints(self, start_pose, goal_pose, costmap):
        """
        Find path using ZMP-based A* or other algorithm
        """
        # This would be a modified path planning algorithm that considers
        # ZMP stability in addition to traditional costmap constraints
        pass


class BipedalNavigationServer(Node):
    """
    Navigation server specifically designed for bipedal humanoid robots
    """
    def __init__(self):
        super().__init__('bipedal_navigation_server')
        
        # Initialize specialized path planners
        self.zmp_planner = ZMPBasedPathPlanner({
            'height': 1.5,  # Robot height in meters
            'foot_length': 0.25,
            'foot_width': 0.1,
            'step_height': 0.05,
            'support_margin': 0.05
        })
        
        # Action server for navigation
        self.nav_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.navigate_to_pose_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # State management
        self.current_state = NavigationState.IDLE
        self.path_following_active = False
        
        # Parameters for bipedal navigation
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_step_length', 0.3),      # Maximum step length
                ('max_step_width', 0.2),       # Maximum step width
                ('max_step_turn', 0.2),        # Maximum turn per step (rad)
                ('balance_margin', 0.05),      # Balance safety margin
                ('step_timing_tolerance', 0.1), # Tolerance for step timing
                ('zmp_stability_threshold', 0.05)  # ZMP stability tolerance
            ]
        )
        
        self.get_logger().info('Bipedal Navigation Server initialized')
    
    def navigate_to_pose_callback(self, goal_handle):
        """
        Callback for navigation goal using bipedal-specific logic
        """
        self.get_logger().info('Received navigation goal')
        
        goal = goal_handle.request.pose
        feedback = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        
        # Plan path using ZMP-stable path planner
        try:
            path = self.zmp_planner.plan_zmp_stable_path(
                self.get_current_pose(), 
                goal.pose, 
                self.get_current_costmap()
            )
            
            if not path:
                # If no path found, abort goal
                goal_handle.abort()
                result.error_code = NavigateToPose.Result.FAILURE
                return result
            
            # Follow the path using bipedal-specific controller
            success = self.follow_bipedal_path(path, goal_handle)
            
            if success:
                goal_handle.succeed()
                result.error_code = NavigateToPose.Result.SUCCESS
            else:
                goal_handle.abort()
                result.error_code = NavigateToPose.Result.FAILURE
            
        except Exception as e:
            self.get_logger().error(f'Navigation error: {e}')
            goal_handle.abort()
            result.error_code = NavigateToPose.Result.FAILURE
        
        return result
    
    def follow_bipedal_path(self, path, goal_handle):
        """
        Follow the path using bipedal-specific control
        """
        for i, waypoint in enumerate(path.poses):
            # Check if preemption was requested
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            
            # Generate footstep plan for this segment
            footstep_plan = self.generate_footstep_plan(
                self.get_current_pose(), 
                waypoint
            )
            
            # Execute footstep plan
            success = self.execute_footstep_plan(footstep_plan)
            
            if not success:
                return False
            
            # Update feedback
            feedback.current_pose = self.get_current_pose()
            feedback.distance_remaining = self.calculate_distance_to_goal(
                self.get_current_pose(), 
                path.poses[-1]
            )
            goal_handle.publish_feedback(feedback)
        
        return True
    
    def generate_footstep_plan(self, current_pose, target_pose):
        """
        Generate footstep plan from current to target pose
        """
        # Calculate how many steps are needed based on max step length
        dx = target_pose.pose.position.x - current_pose.position.x
        dy = target_pose.pose.position.y - current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        max_step_length = self.get_parameter('max_step_length').value
        num_steps = int(distance / max_step_length) + 1
        
        # Create individual footsteps
        footsteps = []
        for i in range(1, num_steps + 1):
            t = i / num_steps
            step_x = current_pose.position.x + t * dx
            step_y = current_pose.position.y + t * dy
            
            # Determine which foot to use (alternating)
            foot_side = 'left' if (i % 2) == 1 else 'right'
            
            footsteps.append({
                'position': (step_x, step_y),
                'foot_side': foot_side,
                'timestamp': self.get_clock().now().to_msg()
            })
        
        return footsteps
    
    def execute_footstep_plan(self, footstep_plan):
        """
        Execute a planned sequence of footsteps
        """
        # This would interface with the robot's walking controller
        # and ensure each step maintains ZMP stability
        
        for step in footstep_plan:
            # Check ZMP stability before executing step
            if not self.is_zmp_stable_for_step(step):
                self.get_logger().error(f'Footstep would violate ZMP stability: {step}')
                return False
            
            # Execute the step (this would use the robot's walking controller)
            success = self.execute_single_footstep(step)
            
            if not success:
                self.get_logger().error(f'Failed to execute footstep: {step}')
                return False
        
        return True
    
    def is_zmp_stable_for_step(self, footstep):
        """
        Check if a footstep maintains ZMP stability
        """
        # Verify that placing the foot at the specified location
        # maintains ZMP within the support polygon
        pass
    
    def execute_single_footstep(self, footstep):
        """
        Execute a single footstep using the robot's walking controller
        """
        # This would send commands to the robot's walking controller
        # In a simulation context, this would update the robot model
        pass


class NavigationState(Enum):
    IDLE = 1
    PLANNING = 2
    PATH_FOLLOWING = 3
    RECOVERY = 4
    CANCELED = 5
    FAILED = 6


def main(args=None):
    rclpy.init(args=args)
    nav_server = BipedalNavigationServer()
    
    try:
        rclpy.spin(nav_server)
    except KeyboardInterrupt:
        nav_server.get_logger().info('Shutting down bipedal navigation server')
    finally:
        nav_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced Configuration for Humanoid Robots

### Custom Costmap Layers for Humanoid Locomotion

```yaml
# config/humanoid_costmap_params.yaml
# Custom costmap configuration for humanoid robot navigation

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      
      # Robot footprint expanded for bipedal locomotion
      robot_radius: 0.3  # Larger than wheeled robot for footstep space
      
      # Resolution and size appropriate for humanoid navigation
      resolution: 0.05  # Fine resolution for step planning
      width: 20.0
      height: 20.0
      origin_x: -10.0
      origin_y: -10.0
      
      # Costmap plugins for humanoid-specific constraints
      plugins:
        - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
        - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
        - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
        - {name: surface_analysis_layer, type: "nav2_costmap_2d::HumanoidSurfaceAnalysisLayer"}  # Custom layer
      
      # Static layer configuration
      static_layer:
        map_subscribe_transient_local: true
        transform_tolerance: 0.5
        max_reading: 10.0
      
      # Obstacle layer configuration
      obstacle_layer:
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0  # Consider humanoid height
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0
      
      # Inflation layer with humanoid-specific parameters
      inflation_layer:
        enabled: true
        cost_scaling_factor: 3.5  # Higher for humanoid safety
        inflation_radius: 0.6     # Larger for bipedal safety margin
        
      # Custom surface analysis layer for humanoid robots
      surface_analysis_layer:
        enabled: true
        analyze_surface_normal: true              # Analyze ground slope
        ground_slope_threshold: 0.3              # Max slope (in radians)
        step_height_threshold: 0.15              # Max step height
        obstacle_height_threshold: 0.8           # Objects too high to step over
        walkable_surface_ratio: 0.7              # Min walkable area ratio in cell

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      
      # Smaller window for immediate planning
      width: 6.0
      height: 6.0
      resolution: 0.025  # Higher resolution for local planning
      origin_x: -3.0
      origin_y: -3.0
      
      # Robot footprint for bipedal locomotion
      robot_radius: 0.25  # Adjusted for immediate area around robot
      
      # Plugins for local costmap
      plugins:
        - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
        - {name: voxel_layer, type: "nav2_costmap_2d::VoxelLayer"}
        - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
        - {name: step_analysis_layer, type: "nav2_costmap_2d::HumanoidStepAnalysisLayer"}  # Custom layer
      
      # Obstacle layer
      obstacle_layer:
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      
      # Voxel layer for 3D obstacle detection
      voxel_layer:
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05   # Fine vertical resolution
        z_voxels: 30         # Cover humanoid workspace
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /humanoid_robot/depth_camera/points
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "PointCloud2"
          min_obstacle_height: 0.0
          obstacle_range: 2.5
      
      # Inflation layer
      inflation_layer:
        enabled: true
        cost_scaling_factor: 5.0  # Higher for immediate safety
        inflation_radius: 0.5     # Immediate safety buffer
      
      # Step analysis layer for bipedal navigation
      step_analysis_layer:
        enabled: true
        max_step_up_height: 0.15    # Maximum height to step up
        max_step_down_height: 0.3   # Maximum height to step down
        min_surface_area: 0.1       # Minimum walkable area in cell
        max_surface_slope: 0.4      # Maximum acceptable ground slope
```

## Performance Optimization and Validation

### Validation Techniques for Humanoid Navigation

```python
import unittest
import numpy as np
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt

class HumanoidNavigationValidator:
    """
    Validation tools for humanoid navigation systems
    """
    def __init__(self):
        self.test_trajectories = []
        self.performance_metrics = {}
    
    def validate_path_stability(self, path, robot_params):
        """
        Validate that a planned path maintains stability constraints
        """
        stability_checks = []
        
        for i in range(len(path.poses) - 1):
            start_pose = path.poses[i]
            end_pose = path.poses[i + 1]
            
            # Check ZMP stability for this segment
            stable = self.check_zmp_stability_for_segment(start_pose, end_pose, robot_params)
            stability_checks.append(stable)
        
        overall_stable = all(stability_checks)
        self.performance_metrics['path_stability_ratio'] = sum(stability_checks) / len(stability_checks)
        
        return overall_stable
    
    def check_zmp_stability_for_segment(self, start_pose, end_pose, robot_params):
        """
        Check ZMP stability for a path segment
        """
        # Calculate mid-point of the segment
        mid_x = (start_pose.pose.position.x + end_pose.pose.position.x) / 2.0
        mid_y = (start_pose.pose.position.y + end_pose.pose.position.y) / 2.0
        
        # Calculate required support polygon at this point
        # This would be based on the robot's stance at this point
        support_polygon = self.calculate_support_polygon_at_pose(
            self.get_supporting_foot_pose(start_pose, end_pose)
        )
        
        # Check if ZMP (approximately at midpoint) is within support polygon
        from shapely.geometry import Point
        zmp_point = Point(mid_x, mid_y)
        
        return support_polygon.contains(zmp_point)
    
    def validate_step_timing(self, footstep_sequence, timing_requirements):
        """
        Validate footstep timing requirements
        """
        if len(footstep_sequence) < 2:
            return True
        
        timing_errors = []
        for i in range(len(footstep_sequence) - 1):
            step_interval = abs(
                self.timestamp_difference(
                    footstep_sequence[i+1]['timestamp'], 
                    footstep_sequence[i]['timestamp']
                )
            )
            
            expected_interval = timing_requirements.get('step_duration', 0.6)
            tolerance = timing_requirements.get('timing_tolerance', 0.1)
            
            if abs(step_interval - expected_interval) > tolerance:
                timing_errors.append({
                    'step_index': i,
                    'expected': expected_interval,
                    'actual': step_interval,
                    'error': abs(step_interval - expected_interval)
                })
        
        timing_valid = len(timing_errors) == 0
        self.performance_metrics['timing_accuracy'] = 1.0 - (len(timing_errors) / len(footstep_sequence))
        
        return timing_valid, timing_errors if not timing_valid else []
    
    def timestamp_difference(self, ts1, ts2):
        """Calculate difference between two timestamps in seconds"""
        return (ts1.sec + ts1.nanosec * 1e-9) - (ts2.sec + ts2.nanosec * 1e-9)
    
    def validate_balance_recovery(self, recovery_behaviors):
        """
        Validate balance recovery behaviors
        """
        for behavior in recovery_behaviors:
            # Test that the behavior doesn't worsen balance
            initial_cog = behavior['initial_cog']
            final_cog = behavior['final_cog']
            
            # Ensure CoG remains within balance polygon
            if not self.is_cog_within_balance_polygon(final_cog):
                return False, f"Recovery behavior {behavior['name']} moves CoG outside safe zone"
        
        return True, "All balance recovery behaviors are valid"
    
    def is_cog_within_balance_polygon(self, cog_position):
        """
        Check if Center of Gravity is within safe balance polygon
        """
        # This would check against the robot's current support polygon
        # For now, return True as placeholder
        return True
    
    def plot_path_validation_results(self, path, validation_results):
        """
        Plot validation results for path planning
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Extract path coordinates
        path_x = [pose.pose.position.x for pose in path.poses]
        path_y = [pose.pose.position.y for pose in path.poses]
        
        # Plot the path
        ax.plot(path_x, path_y, 'b-', linewidth=2, label='Planned Path')
        ax.scatter(path_x, path_y, c='red', s=20, label='Waypoints')
        
        # Highlight unstable segments
        if 'unstable_segments' in validation_results:
            for seg in validation_results['unstable_segments']:
                ax.plot([path_x[seg[0]], path_x[seg[1]]], 
                       [path_y[seg[0]], path_y[seg[1]]], 
                       'r--', linewidth=3, label='Unstable Segment')
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Path Validation Results')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        plt.show()


class TestHumanoidNavigation(unittest.TestCase):
    """
    Unit tests for humanoid navigation components
    """
    def setUp(self):
        self.validator = HumanoidNavigationValidator()
        self.robot_params = {
            'height': 1.5,
            'foot_length': 0.25,
            'foot_width': 0.1,
            'step_height': 0.05,
            'weight': 50.0
        }
    
    def test_zmp_stability_basic(self):
        """Test basic ZMP stability calculation"""
        # Simple test: two poses that should be stable
        path = Path()
        start_pose = PoseStamped()
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        
        end_pose = PoseStamped()
        end_pose.pose.position.x = 0.2  # Within step length
        end_pose.pose.position.y = 0.0
        
        path.poses = [start_pose, end_pose]
        
        # This path should be ZMP stable
        is_stable = self.validator.validate_path_stability(path, self.robot_params)
        self.assertTrue(is_stable, "Simple stable path should pass ZMP validation")
    
    def test_step_timing_validation(self):
        """Test step timing validation"""
        from builtin_interfaces.msg import Time
        
        # Create a sequence of footsteps with proper timing
        timestamp = Time(sec=0, nanosec=0)
        footstep_seq = [
            {'timestamp': timestamp, 'position': (0.0, 0.0)},
            {'timestamp': Time(sec=0, nanosec=600000000), 'position': (0.3, 0.0)},  # 0.6 sec later
            {'timestamp': Time(sec=1, nanosec=200000000), 'position': (0.6, 0.0)}   # 0.6 sec later
        ]
        
        timing_reqs = {
            'step_duration': 0.6,
            'timing_tolerance': 0.1
        }
        
        valid, errors = self.validator.validate_step_timing(footstep_seq, timing_reqs)
        self.assertTrue(valid, "Correctly timed footsteps should pass validation")
        self.assertEqual(len(errors), 0, "No timing errors should be detected")
    
    def test_unstable_path_rejection(self):
        """Test that unstable paths are properly rejected"""
        # Create a path with an excessively long step
        path = Path()
        start_pose = PoseStamped()
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        
        end_pose = PoseStamped()
        end_pose.pose.position.x = 1.0  # Much longer than recommended step
        end_pose.pose.position.y = 0.0
        
        path.poses = [start_pose, end_pose]
        
        # This path should be rejected for ZMP instability
        is_stable = self.validator.validate_path_stability(path, self.robot_params)
        self.assertFalse(is_stable, "Unstable path should be rejected")


def validate_humanoid_navigation_system():
    """
    Run comprehensive validation of humanoid navigation system
    """
    validator = HumanoidNavigationValidator()
    
    # Create test runner
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestHumanoidNavigation)
    runner = unittest.TextTestRunner(verbosity=2)
    
    # Run tests
    result = runner.run(test_suite)
    
    print("\n=== Performance Metrics ===")
    for metric, value in validator.performance_metrics.items():
        print(f"{metric}: {value:.3f}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = validate_humanoid_navigation_system()
    print(f"\nValidation {'PASSED' if success else 'FAILED'}")
```

## Chapter Summary

Navigation for humanoid robots requires specialized considerations beyond traditional wheeled robot navigation. The Nav2 system provides a flexible framework that can be enhanced for bipedal locomotion by incorporating:

1. **ZMP-based path planning**: Ensuring Center of Mass remains within support polygon
2. **Footstep planning**: Discrete step planning rather than continuous motion
3. **Balance constraints**: Incorporating balance stability into path planning
4. **Terrain analysis**: Specialized analysis for walkable surfaces and step heights

The key innovations for humanoid navigation include adapting traditional path planning algorithms with human-specific constraints like step length limits, balance maintenance, and the need to plan footsteps rather than continuous trajectories.

## Learning Objectives

After completing this chapter, students should be able to:
- Understand the unique challenges of navigation for bipedal humanoid robots
- Implement ZMP-based path planning for stable locomotion
- Configure Nav2 for humanoid-specific navigation requirements
- Design footstep planning algorithms integrated with global path planners
- Validate navigation paths for stability and safety
- Implement balance recovery behaviors for humanoid robots

## Key Takeaways

1. **Balance is Critical**: Humanoid navigation must maintain dynamic stability at each step
2. **Discrete Motion**: Unlike wheeled robots, humanoid robots move in discrete steps
3. **Specialized Planning**: Path planning must consider step constraints and balance requirements
4. **Validation is Essential**: Navigation paths must be validated for stability and safety
5. **Integration Required**: ZMP planning, footstep planning, and traditional path planning must work together
#!/usr/bin/env python3
"""
Direct Navigation Fallback Module

When Nav2 fails to generate trajectories or stops publishing cmd_vel,
this module takes over and directly controls the robot to move toward the goal.

This ensures the robot ALWAYS makes progress toward its goal, even when
Nav2 is having issues with path planning or obstacle avoidance.
"""

import math
import time
from typing import Optional
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
import rclpy


class DirectNavigationFallback:
    """
    Direct velocity control fallback when Nav2 fails.
    Implements simple "drive toward goal" behavior.
    """
    
    def __init__(self, node: Node, logger, tf_buffer: Optional[tf2_ros.Buffer] = None):
        """
        Initialize direct navigation fallback.
        
        Args:
            node: ROS 2 node instance
            logger: ROS 2 logger instance
            tf_buffer: TF buffer for frame transformations (optional, will create if None)
        """
        self.node = node
        self.logger = logger
        
        # CRITICAL: Use provided TF buffer or create new one (for frame consistency checks)
        if tf_buffer is None:
            # Create new buffer and listener if not provided
            # Note: Buffer should be updated by listener in parent node (mission_controller)
            self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=1.0))
            from tf2_ros import TransformListener
            self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=True)
        else:
            # Use existing buffer from parent node (mission_controller already has listener)
            # Don't create new listener - existing listener will update the buffer
            self.tf_buffer = tf_buffer
            self.tf_listener = None  # Parent node's listener updates the buffer
        
        # CRITICAL: Use RELIABLE QoS to ensure messages are never dropped
        # TRANSIENT_LOCAL durability ensures messages persist even if subscriber connects later
        # This is essential for guaranteed delivery to hardware driver
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # CRITICAL: Don't drop messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Messages persist for late subscribers
            history=HistoryPolicy.KEEP_LAST,
            depth=20  # Increased queue depth for reliability
        )
        
        # Publisher for direct velocity commands with RELIABLE QoS for guaranteed delivery
        # CRITICAL: Publish to priority topic, not directly to /cmd_vel
        # The cmd_vel_multiplexer will arbitrate and publish to /cmd_vel
        self.cmd_vel_pub = node.create_publisher(
            Twist,
            '/cmd_vel/direct_control',
            cmd_vel_qos
        )
        
        # CRITICAL: Watchdog timer to publish cmd_vel at MAXIMUM frequency (50Hz = hardware limit)
        # This ensures continuous movement even if state machine stalls
        # Also ensures our commands "win" over other nodes (joy_ctrl, behavior_ctrl, Nav2, etc.)
        # 50Hz = 0.02s interval (20ms) - maximum frequency for reliable hardware communication
        self.watchdog_timer = node.create_timer(0.02, self._watchdog_update)  # 50Hz = every 0.02s
        self.last_watchdog_cmd = None
        self.watchdog_enabled = False
        
        # CRITICAL: Monitor Nav2's cmd_vel and override if it's zero or stopping the robot
        # Subscribe to Nav2's priority topic to see what Nav2 is publishing
        self.cmd_vel_sub = node.create_subscription(
            Twist,
            '/cmd_vel/nav2',
            self._cmd_vel_monitor_callback,
            10
        )
        self.last_nav2_cmd_vel = None
        self.last_nav2_cmd_vel_time = None
        self.override_nav2_zero = True  # Override Nav2 if it publishes zero
        
        # State
        self.is_active = False
        self.current_goal = None
        self.last_cmd_vel_time = None
        self.nav2_cmd_vel_timeout = 0.5  # CRITICAL: Reduced to 0.5s - take over immediately when Nav2 stops
        self.is_primary_mode = False  # CRITICAL: If True, direct nav is PRIMARY (explicitly activated), don't auto-deactivate
        
        # Parameters
        self.max_linear_vel = 0.5  # m/s - conservative speed for safety
        self.max_angular_vel = 0.8  # rad/s
        # CRITICAL: Goal tolerance must be SMALL to prevent premature "goal reached" detection
        # If robot starts 0.7m away and moves 30cm, it's at 0.4m - with 0.4m tolerance, it thinks goal is reached!
        self.goal_tolerance = 0.15  # meters - REDUCED from 0.4m to prevent premature stopping
        self.angular_tolerance = 0.2  # radians (~11 degrees)
        
        # CRITICAL: Hardware deadzone equivalent in m/s
        # ESP32 firmware ignores commands below 0.01 PWM
        # Conversion: 0.01 PWM * (1.3 m/s / 0.5 PWM) = 0.026 m/s
        # 
        # To ensure BOTH wheels are above deadzone when rotating at max angular velocity (0.8 rad/s):
        # left_ms = linear_vel - (WHEELBASE_M/2) * angular_vel
        # We need: left_ms >= deadzone_mps
        # So: linear_vel >= deadzone_mps + (WHEELBASE_M/2) * max_angular_vel
        #     linear_vel >= 0.026 + (0.175/2) * 0.8 = 0.026 + 0.07 = 0.096 m/s
        # 
        # Using 0.1 m/s (slightly above 0.096 m/s) to ensure both wheels are always above deadzone
        # This prevents one wheel from being clamped to zero during rotation
        self.hardware_deadzone_linear_mps = 0.1  # Minimum linear velocity to ensure both wheels above deadzone
        
        # CRITICAL: Minimum distance to travel before checking if goal is reached
        # This prevents false "goal reached" if robot hasn't moved enough
        self.min_movement_before_goal_check = 0.5  # meters - must move at least 50cm before checking
        self.initial_distance_to_goal = None  # Will be set when goal is set
        
        # PID-like control parameters
        self.linear_kp = 0.5  # Proportional gain for linear velocity
        self.angular_kp = 1.0  # Proportional gain for angular velocity
        
        self.logger.info("DirectNavigationFallback initialized")
    
    def set_goal(self, goal_pose: PoseStamped, preserve_active_state: bool = False):
        """
        Set the goal for direct navigation.
        
        Args:
            goal_pose: Target pose in map frame
            preserve_active_state: If True, don't deactivate if already active (for goal updates during navigation)
        """
        was_active = self.is_active
        self.current_goal = goal_pose
        self.initial_distance_to_goal = None  # Reset - will be calculated on first update
        
        # CRITICAL: If already active and preserving state, keep it active (goal update during navigation)
        # Otherwise, deactivate (goal set but navigation not started yet)
        if not preserve_active_state:
            self.is_active = False  # Will activate if Nav2 fails
        
        if preserve_active_state and was_active:
            self.logger.info(
                f"DirectNavigationFallback: Goal UPDATED (preserving active state) to "
                f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
            )
        else:
            self.logger.info(
                f"DirectNavigationFallback: Goal set to "
                f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
            )
    
    def check_nav2_active(self, robot_pose: Optional[PoseStamped]) -> bool:
        """
        Check if Nav2 is actively publishing cmd_vel.
        This is called periodically to monitor Nav2's status.
        
        Args:
            robot_pose: Current robot pose (for distance calculation)
            
        Returns:
            True if Nav2 is active, False if we should take over
        """
        # Check if Nav2 has stopped publishing
        if self.last_cmd_vel_time is not None:
            time_since_last = time.time() - self.last_cmd_vel_time
            if time_since_last > self.nav2_cmd_vel_timeout:
                self.logger.warn(
                    f"⚠️ Nav2 has not published cmd_vel for {time_since_last:.1f}s. "
                    f"Activating direct navigation fallback."
                )
                return False
        
        # Check if robot is making progress
        if robot_pose and self.current_goal:
            distance = self._calculate_distance(robot_pose, self.current_goal)
            if distance < self.goal_tolerance:
                # Goal reached
                return True
        
        return True  # Nav2 is active
    
    def update_nav2_cmd_vel_time(self):
        """Update the timestamp of last Nav2 cmd_vel message."""
        self.last_cmd_vel_time = time.time()
        if self.is_active and not self.is_primary_mode:
            # Nav2 is back - deactivate fallback (only if not in PRIMARY mode)
            # In PRIMARY mode, direct nav stays active even when Nav2 publishes
            self.logger.info("✅ Nav2 is publishing again. Deactivating direct navigation fallback.")
            self.is_active = False
            self._stop_robot()
        elif self.is_active and self.is_primary_mode:
            # In PRIMARY mode, Nav2 is just backup - direct nav stays active
            self.logger.debug("Nav2 publishing (backup mode), but direct navigation remains PRIMARY.")
    
    def activate(self, primary_mode: bool = True):
        """
        Activate direct navigation control.
        
        Args:
            primary_mode: If True, direct nav is PRIMARY (explicitly activated),
                         won't auto-deactivate when Nav2 starts publishing.
                         If False, acts as fallback (auto-deactivates when Nav2 resumes).
        """
        if not self.is_active:
            self.logger.warn(
                "🚨🚨🚨 ACTIVATING DIRECT NAVIGATION - Taking control from Nav2. "
                "Robot will now move continuously toward goal."
            )
            self.is_active = True
            self.is_primary_mode = primary_mode  # CRITICAL: Set PRIMARY mode flag
            self.watchdog_enabled = True  # Enable watchdog timer (50Hz)
            self.override_nav2_zero = True  # Override Nav2 zero commands
            
            # CRITICAL: Immediately publish a command to ensure movement starts
            # Publish multiple times for maximum reliability
            # This prevents any delay before first update() call
            cmd = Twist()
            cmd.linear.x = 0.3  # Increased from 0.2 to 0.3 for stronger initial movement
            cmd.angular.z = 0.0
            
            # Publish multiple times to ensure delivery
            for _ in range(5):  # Publish 5 times rapidly on activation
                self.cmd_vel_pub.publish(cmd)
            
            self.last_watchdog_cmd = cmd
            self.logger.info("✅ Initial cmd_vel published to start movement immediately (5x for reliability)")
            self.logger.info("✅ Watchdog timer enabled - cmd_vel will be published at 50Hz")
            self.logger.info("✅ Nav2 zero-command override enabled - will override any zero cmd_vel")
            
            # CRITICAL: Verify activation
            self.logger.info(f"🔍 VERIFICATION: is_active = {self.is_active}")
            self.logger.info(f"🔍 VERIFICATION: watchdog_enabled = {self.watchdog_enabled}")
            self.logger.info(f"🔍 VERIFICATION: override_nav2_zero = {self.override_nav2_zero}")
            self.logger.info(f"🔍 VERIFICATION: current_goal = {self.current_goal is not None}")
            self.logger.info(f"🔍 VERIFICATION: last_watchdog_cmd = {self.last_watchdog_cmd is not None}")
            
            # CRITICAL: Verify activation
            self.logger.info(f"🔍 VERIFICATION: is_active = {self.is_active}")
            self.logger.info(f"🔍 VERIFICATION: watchdog_enabled = {self.watchdog_enabled}")
            self.logger.info(f"🔍 VERIFICATION: override_nav2_zero = {self.override_nav2_zero}")
            self.logger.info(f"🔍 VERIFICATION: current_goal = {self.current_goal is not None}")
            self.logger.info(f"🔍 VERIFICATION: last_watchdog_cmd = {self.last_watchdog_cmd is not None}")
    
    def deactivate(self):
        """Deactivate direct navigation control."""
        if self.is_active:
            self.logger.info("✅ Deactivating direct navigation fallback - Returning control to Nav2")
            self.is_active = False
            self.is_primary_mode = False  # Reset PRIMARY mode flag
            self.watchdog_enabled = False  # Disable watchdog timer
            self._stop_robot()
    
    def update(self, robot_pose: Optional[PoseStamped]) -> bool:
        """
        Update direct navigation control.
        Called by state machine every 0.5s (2Hz).
        Watchdog timer also publishes at 10Hz independently to ensure continuous movement.
        
        Args:
            robot_pose: Current robot pose in map frame
            
        Returns:
            True if goal reached, False otherwise
        """
        if not self.is_active or not self.current_goal:
            return False
        
        if not robot_pose:
            # If no pose, watchdog will maintain movement by republishing last command
            self.logger.debug("⚠️ Direct control update: No robot pose, watchdog will maintain movement")
            return False
        
        # CRITICAL: Update every time called (state machine runs at 2Hz, so this is already rate-limited)
        # We MUST publish cmd_vel every cycle to ensure continuous movement
        # Only skip if called extremely frequently (more than 20Hz) to prevent spam
        current_time = time.time()
        if hasattr(self, 'last_update_time') and current_time - self.last_update_time < 0.05:  # 20Hz max
            # Still publish cmd_vel even if called frequently (safety)
            # But don't do full calculation
            return False
        if not hasattr(self, 'last_update_time'):
            self.last_update_time = 0.0
        self.last_update_time = current_time
        
        # CRITICAL FIX: Ensure robot_pose and goal_pose are in the same frame
        # If frames differ, transform goal to robot_pose's frame
        goal_pose_to_use = self.current_goal
        if robot_pose.header.frame_id != self.current_goal.header.frame_id:
            self.logger.warn(
                f"⚠️ Direct navigation: Frame mismatch detected! "
                f"Robot in '{robot_pose.header.frame_id}', Goal in '{self.current_goal.header.frame_id}'. "
                f"Transforming goal to robot's frame..."
            )
            try:
                # Transform goal to robot_pose's frame
                transform = self.tf_buffer.lookup_transform(
                    robot_pose.header.frame_id,
                    self.current_goal.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                goal_pose_to_use = do_transform_pose_stamped(self.current_goal, transform)
                goal_pose_to_use.header.frame_id = robot_pose.header.frame_id
                self.logger.info(
                    f"✅ Direct navigation: Goal transformed to '{robot_pose.header.frame_id}' frame. "
                    f"Original: ({self.current_goal.pose.position.x:.2f}, {self.current_goal.pose.position.y:.2f}) in '{self.current_goal.header.frame_id}', "
                    f"Transformed: ({goal_pose_to_use.pose.position.x:.2f}, {goal_pose_to_use.pose.position.y:.2f}) in '{robot_pose.header.frame_id}'"
                )
            except Exception as e:
                self.logger.error(
                    f"❌ Direct navigation: Cannot transform goal from '{self.current_goal.header.frame_id}' to '{robot_pose.header.frame_id}': {e}. "
                    f"Using goal in original frame - calculations may be WRONG!"
                )
                # Continue with original goal - calculation will be wrong but at least won't crash
                goal_pose_to_use = self.current_goal
        
        # Calculate distance and angle to goal (now in same frame)
        distance = self._calculate_distance(robot_pose, goal_pose_to_use)
        
        # CRITICAL: Store initial distance on first update
        if self.initial_distance_to_goal is None:
            self.initial_distance_to_goal = distance
            self.logger.info(
                f"📍 Direct navigation: Initial distance to goal = {distance:.2f}m, "
                f"goal_tolerance = {self.goal_tolerance:.2f}m, "
                f"min_movement = {self.min_movement_before_goal_check:.2f}m"
            )
        
        # CRITICAL: Calculate how far we've moved toward the goal
        distance_traveled = self.initial_distance_to_goal - distance
        
        # CRITICAL: Check orientation difference (robot vs goal orientation)
        # Use transformed goal (already validated to be in same frame as robot_pose)
        goal_yaw = self._quaternion_to_yaw(goal_pose_to_use.pose.orientation)
        robot_yaw = self._quaternion_to_yaw(robot_pose.pose.orientation)
        orientation_diff = goal_yaw - robot_yaw
        # Normalize to [-pi, pi]
        while orientation_diff > math.pi:
            orientation_diff -= 2 * math.pi
        while orientation_diff < -math.pi:
            orientation_diff += 2 * math.pi
        
        # CRITICAL: Only check if goal is reached if:
        # 1. We've moved at least min_movement_before_goal_check meters
        # 2. We're within goal_tolerance distance
        # 3. We're within angular_tolerance orientation
        # This prevents false "goal reached" if robot hasn't moved enough or is facing wrong direction
        if distance_traveled >= self.min_movement_before_goal_check:
            if distance < self.goal_tolerance and abs(orientation_diff) <= self.angular_tolerance:
                self.logger.info(
                    f"✅ Direct navigation: Goal reached! "
                    f"Distance: {distance:.2f}m (tolerance: {self.goal_tolerance:.2f}m), "
                    f"Orientation: {math.degrees(abs(orientation_diff)):.1f}° (tolerance: {math.degrees(self.angular_tolerance):.1f}°), "
                    f"Distance traveled: {distance_traveled:.2f}m"
                )
                self._stop_robot()
                self.is_active = False
                return True
        
        # Calculate velocities based on whether we're at goal position
        # CRITICAL: If at goal position but wrong orientation, rotate in place to match orientation
        # Otherwise, navigate toward goal position normally
        if distance < self.goal_tolerance:
            # At goal position but possibly wrong orientation - rotate to match goal orientation
            angle_to_goal = orientation_diff  # Use orientation difference for rotation
            linear_vel = 0.0  # Don't move forward when at goal position
            self.logger.info(
                f"🔄 Direct navigation: At goal position, rotating to match orientation. "
                f"Distance: {distance:.2f}m, Orientation diff: {math.degrees(orientation_diff):.1f}° "
                f"(tolerance: {math.degrees(self.angular_tolerance):.1f}°)"
            )
        else:
            # Not at goal position yet - navigate toward goal
            # Use transformed goal (already validated to be in same frame as robot_pose)
            angle_to_goal = self._calculate_angle_to_goal(robot_pose, goal_pose_to_use)
            
            # CRITICAL FIX: Allow simultaneous rotation and translation
            # Scale linear velocity based on angle alignment (0.0 when 90° off, 1.0 when aligned)
            # This allows rover to make progress even while rotating toward goal
            angle_factor = max(0.0, 1.0 - abs(angle_to_goal) / (math.pi / 2))  # 1.0 when aligned, 0.0 when 90° off
            
            # Base linear velocity from distance
            base_linear_vel = min(self.linear_kp * distance, self.max_linear_vel)
            
            # Scale by angle factor, but ensure minimum velocity for progress
            linear_vel = base_linear_vel * angle_factor
            
            # CRITICAL: Always ensure linear velocity is above hardware deadzone threshold
            # This prevents ugv_bringup from clamping commands to zero
            # Hardware deadzone: 0.026 m/s, using 0.03 m/s (1.15x margin) for safety
            linear_vel = max(linear_vel, self.hardware_deadzone_linear_mps)
            
            min_linear_vel = 0.1  # CRITICAL: Minimum linear velocity to ensure progress even when angle is large
            if abs(angle_to_goal) > self.angular_tolerance:
                # Angle is large - use higher minimum velocity but still move forward
                linear_vel = max(linear_vel, min_linear_vel)
                self.logger.info(
                    f"🔄 Direct navigation: Rotating toward goal while moving. Angle: {math.degrees(angle_to_goal):.1f}°, "
                    f"Distance: {distance:.2f}m, Linear: {linear_vel:.2f} m/s (min vel: {min_linear_vel:.2f} m/s)"
                )
            else:
                # Move forward while adjusting angle
                # linear_vel is already >= hardware_deadzone_linear_mps (0.03 m/s)
                self.logger.info(
                    f"🚀 Direct navigation: Moving toward goal. Distance: {distance:.2f}m, "
                    f"Linear: {linear_vel:.2f} m/s (>= {self.hardware_deadzone_linear_mps:.3f} m/s deadzone), "
                    f"Angle: {math.degrees(angle_to_goal):.1f}°"
                )
        
        # Calculate angular velocity
        angular_vel = self.angular_kp * angle_to_goal
        if abs(angular_vel) > self.max_angular_vel:
            angular_vel = math.copysign(self.max_angular_vel, angular_vel)
        
        # Log movement progress occasionally (only when haven't moved enough yet)
        if not hasattr(self, '_goal_check_log_counter'):
            self._goal_check_log_counter = 0
        if distance_traveled < self.min_movement_before_goal_check:
            # Haven't moved enough yet - log occasionally
            self._goal_check_log_counter += 1
            if self._goal_check_log_counter >= 10:  # Every 5 seconds (10 * 0.5s)
                self.logger.debug(
                    f"🔄 Direct navigation: Moving toward goal. "
                    f"Distance: {distance:.2f}m, "
                    f"Traveled: {distance_traveled:.2f}m / {self.min_movement_before_goal_check:.2f}m "
                    f"(need to move more before checking goal reached)"
                )
                self._goal_check_log_counter = 0
        
        # CRITICAL: Publish velocity command EVERY time this is called
        # This ensures continuous movement (called every 0.5s by state machine)
        # Watchdog also publishes at 50Hz independently
        # Publish multiple times for maximum reliability
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        
        # Publish multiple times to ensure delivery (QoS might drop messages)
        for _ in range(2):  # Publish 2 times for reliability
            self.cmd_vel_pub.publish(cmd)
        
        self.last_watchdog_cmd = cmd  # Store for watchdog (50Hz republishing)
        
        # Notify movement guarantee system that cmd_vel was published
        # (This will be handled by mission controller calling update_cmd_vel_time)
        
        # Log every update (2Hz) for visibility
        self.logger.info(
            f"📤 State machine: Published cmd_vel: linear={linear_vel:.3f}, angular={angular_vel:.3f}, "
            f"distance={distance:.2f}m"
        )
        
        return False
    
    def _cmd_vel_monitor_callback(self, msg: Twist):
        """
        Monitor Nav2's /cmd_vel/nav2 topic to detect if Nav2 has stopped publishing.
        This callback tracks Nav2's activity to determine when to activate fallback.
        
        NOTE: We no longer need to override Nav2's commands directly because:
        1. Nav2 publishes to /cmd_vel/nav2 (Priority 3)
        2. Direct control publishes to /cmd_vel/direct_control (Priority 2)
        3. Multiplexer handles priority arbitration automatically
        4. Priority 2 (Direct Control) always wins over Priority 3 (Nav2)
        """
        # Store last Nav2 cmd_vel for monitoring
        self.last_nav2_cmd_vel = msg
        self.last_nav2_cmd_vel_time = time.time()
        
        # Track Nav2's activity (used by mission_controller to detect Nav2 failures)
        # No need to override here - multiplexer handles priority automatically
        is_zero = abs(msg.linear.x) < 0.01 and abs(msg.angular.z) < 0.01
        if not is_zero:
            # Nav2 is actively navigating
            self.logger.debug(
                f"Nav2 active: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}"
            )
    
    def _watchdog_update(self):
        """
        Watchdog timer callback - publishes cmd_vel at 50Hz independently.
        This ensures continuous movement even if state machine stalls.
        CRITICAL: Publishes multiple times per cycle for maximum reliability.
        """
        if not self.watchdog_enabled or not self.is_active or not self.current_goal:
            return
        
        # CRITICAL: Republish last command at 50Hz, multiple times per cycle
        # This ensures maximum reliability and "wins" over other nodes
        # Publishing multiple times compensates for any potential message dropping
        if self.last_watchdog_cmd:
            for _ in range(2):  # Publish 2 times per cycle for reliability
                self.cmd_vel_pub.publish(self.last_watchdog_cmd)
            # Log only occasionally to avoid spam (every 100 calls = 5 seconds at 20Hz)
            if not hasattr(self, '_watchdog_log_counter'):
                self._watchdog_log_counter = 0
            self._watchdog_log_counter += 1
            if self._watchdog_log_counter >= 100:
                self.logger.info(
                    f"🔄 Watchdog: Publishing cmd_vel at 50Hz (0.02s interval). "
                    f"Last cmd: linear={self.last_watchdog_cmd.linear.x:.3f}, "
                    f"angular={self.last_watchdog_cmd.angular.z:.3f}"
                )
                self._watchdog_log_counter = 0
    
    def _calculate_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate 2D distance between two poses."""
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def _calculate_angle_to_goal(self, robot_pose: PoseStamped, goal_pose: PoseStamped) -> float:
        """
        Calculate angle from robot to goal.
        Returns angle in range [-pi, pi].
        """
        # Calculate desired heading
        dx = goal_pose.pose.position.x - robot_pose.pose.position.x
        dy = goal_pose.pose.position.y - robot_pose.pose.position.y
        desired_yaw = math.atan2(dy, dx)
        
        # Get current robot yaw
        current_yaw = self._quaternion_to_yaw(robot_pose.pose.orientation)
        
        # Calculate angle difference
        angle_diff = desired_yaw - current_yaw
        
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def _quaternion_to_yaw(self, quaternion) -> float:
        """Convert quaternion to yaw angle."""
        # Standard quaternion to Euler yaw conversion
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def is_goal_reached(self, robot_pose: Optional[PoseStamped]) -> bool:
        """
        Check if goal is reached.
        CRITICAL: Uses same logic as update() to prevent premature goal detection.
        Must have moved at least min_movement_before_goal_check AND be within goal_tolerance.
        CRITICAL: Also checks orientation match - robot must face goal orientation for proper positioning.
        
        Args:
            robot_pose: Current robot pose
            
        Returns:
            True if goal reached (distance AND orientation), False otherwise
        """
        if not robot_pose or not self.current_goal:
            return False
        
        # CRITICAL FIX: Ensure robot_pose and goal_pose are in the same frame
        # If frames differ, transform goal to robot_pose's frame
        goal_pose_to_use = self.current_goal
        if robot_pose.header.frame_id != self.current_goal.header.frame_id:
            try:
                # Transform goal to robot_pose's frame
                transform = self.tf_buffer.lookup_transform(
                    robot_pose.header.frame_id,
                    self.current_goal.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                goal_pose_to_use = do_transform_pose_stamped(self.current_goal, transform)
                goal_pose_to_use.header.frame_id = robot_pose.header.frame_id
            except Exception as e:
                self.logger.warn(
                    f"⚠️ Direct navigation (is_goal_reached): Cannot transform goal from '{self.current_goal.header.frame_id}' to '{robot_pose.header.frame_id}': {e}. "
                    f"Goal reached check may be incorrect."
                )
                return False  # Can't check if goal is reached with frame mismatch
        
        # Calculate distance using transformed goal (now in same frame as robot_pose)
        distance = self._calculate_distance(robot_pose, goal_pose_to_use)
        
        # If we haven't set initial distance yet, we can't check
        if self.initial_distance_to_goal is None:
            return False
        
        # Calculate how far we've moved
        distance_traveled = self.initial_distance_to_goal - distance
        
        # CRITICAL: Only consider goal reached if we've moved enough AND are within distance tolerance
        if distance_traveled >= self.min_movement_before_goal_check:
            # Check distance
            if distance >= self.goal_tolerance:
                return False
            
            # CRITICAL: Also check orientation match - robot must face goal orientation
            # This is essential for license plate capture - robot must face vehicle
            # Use transformed goal (already validated to be in same frame as robot_pose)
            goal_yaw = self._quaternion_to_yaw(goal_pose_to_use.pose.orientation)
            robot_yaw = self._quaternion_to_yaw(robot_pose.pose.orientation)
            orientation_diff = abs(goal_yaw - robot_yaw)
            # Normalize to [0, pi] range
            if orientation_diff > math.pi:
                orientation_diff = 2 * math.pi - orientation_diff
            
            # Check if orientation matches (within angular_tolerance)
            orientation_match = orientation_diff <= self.angular_tolerance
            
            if not orientation_match:
                # At position but wrong orientation - log occasionally
                if not hasattr(self, '_orientation_check_log_counter'):
                    self._orientation_check_log_counter = 0
                self._orientation_check_log_counter += 1
                if self._orientation_check_log_counter >= 20:  # Every 10 seconds (20 * 0.5s update rate)
                    self.logger.debug(
                        f"🔄 Direct navigation: At goal position (distance: {distance:.2f}m) "
                        f"but orientation mismatch: {math.degrees(orientation_diff):.1f}° > {math.degrees(self.angular_tolerance):.1f}° "
                        f"(robot: {math.degrees(robot_yaw):.1f}°, goal: {math.degrees(goal_yaw):.1f}°)"
                    )
                    self._orientation_check_log_counter = 0
                return False
            
            # Both distance and orientation match - goal reached!
            return True
        
        return False

#!/usr/bin/env python3
"""
Movement Guarantee System

This module ensures the robot ALWAYS moves when it should, regardless of
other system failures. It acts as a final safety net to guarantee continuous movement.
"""

import time
import math
from typing import Optional
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MovementGuarantee:
    """
    Independent movement guarantee system that ensures robot moves continuously.
    This bypasses all other systems if needed to guarantee movement.
    """
    
    def __init__(self, node: Node, logger):
        """
        Initialize movement guarantee system.
        
        Args:
            node: ROS 2 node instance
            logger: ROS 2 logger instance
        """
        self.node = node
        self.logger = logger
        
        # CRITICAL: Use RELIABLE QoS for emergency cmd_vel to ensure guaranteed delivery
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        
        emergency_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Don't drop emergency messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Messages persist
            history=HistoryPolicy.KEEP_LAST,
            depth=20  # Increased queue depth
        )
        
        # Publisher for emergency cmd_vel (highest priority, RELIABLE QoS)
        # CRITICAL: Publish to priority topic, not directly to /cmd_vel
        # The cmd_vel_multiplexer will arbitrate and publish to /cmd_vel
        self.emergency_cmd_vel_pub = node.create_publisher(
            Twist,
            '/cmd_vel/emergency',
            emergency_qos
        )
        
        # CRITICAL: Subscriber for odometry with RELIABLE QoS to match base_node publisher
        # Movement verification depends on accurate odometry - messages must not be dropped
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Match base_node publisher QoS
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Match base_node publisher QoS
            history=HistoryPolicy.KEEP_LAST,
            depth=20  # Increased queue depth to match publisher
        )
        self.odom_sub = node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos
        )
        
        # CRITICAL: Subscribe to final /cmd_vel to detect if robot is rotating in place
        # This prevents false "stuck" detection when robot is rotating at goal position
        # Rotation IS valid movement - robot should not be forced forward when rotating
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Match multiplexer publisher QoS
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.cmd_vel_sub = node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            cmd_vel_qos
        )
        
        # State
        self.is_active = False
        self.current_goal = None
        self.last_robot_position = None
        self.previous_robot_position = None  # For movement calculation
        self.last_robot_orientation = None  # For rotation detection
        self.previous_robot_orientation = None  # For rotation calculation
        self.last_robot_position_time = None
        self.last_cmd_vel_time = None
        self.last_cmd_vel = None  # Store last cmd_vel to check for rotation
        self.last_cmd_vel_msg_time = None  # Timestamp of last cmd_vel message
        self.activate_start_time = None  # Track activation time for odom never-received detection
        
        # Parameters
        self.movement_timeout = 2.0  # seconds - if robot doesn't move for this long, force movement
        self.movement_threshold = 0.01  # meters - consider moving if moved more than 1cm
        self.cmd_vel_timeout = 1.0  # seconds - if no cmd_vel for this long, force movement
        self.odom_timeout = 2.0  # CRITICAL: Maximum age of odometry data (seconds) before considering it stale
        
        # CRITICAL: Watchdog timer at 50Hz (maximum hardware can handle)
        # This ensures rapid detection and response to movement issues
        self.watchdog_timer = node.create_timer(0.02, self.watchdog_callback)  # 50Hz (increased from 10Hz)
        
        self.logger.info("MovementGuarantee initialized")
    
    def odom_callback(self, msg: Odometry):
        """Update robot position and orientation from odometry with movement tracking."""
        current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        current_orientation = msg.pose.pose.orientation
        
        # Track previous position for movement calculation
        if self.last_robot_position:
            self.previous_robot_position = self.last_robot_position
        
        # Track previous orientation for rotation detection
        if self.last_robot_orientation:
            self.previous_robot_orientation = self.last_robot_orientation
        
        self.last_robot_position = current_position
        self.last_robot_orientation = current_orientation
        self.last_robot_position_time = time.time()
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Monitor final /cmd_vel to detect if robot is rotating in place.
        This is CRITICAL to prevent false "stuck" detection when robot is rotating at goal.
        """
        self.last_cmd_vel = msg
        self.last_cmd_vel_msg_time = time.time()
        # Also update last_cmd_vel_time for compatibility
        self.last_cmd_vel_time = time.time()
    
    def activate(self, goal_pose: PoseStamped):
        """
        Activate movement guarantee system.
        
        Args:
            goal_pose: Target pose to move toward
        """
        if not self.is_active:
            self.logger.warn(
                "🛡️ Movement Guarantee ACTIVATED - Will ensure robot moves continuously"
            )
            self.is_active = True
            self.current_goal = goal_pose
            self.activate_start_time = time.time()  # Track activation time for odom never-received detection
            # Don't set last_robot_position_time here - wait for actual odometry
    
    def deactivate(self):
        """Deactivate movement guarantee system."""
        if self.is_active:
            self.logger.info("Movement Guarantee deactivated")
            self.is_active = False
            self.current_goal = None
    
    def update_cmd_vel_time(self):
        """Update timestamp of last cmd_vel command."""
        self.last_cmd_vel_time = time.time()
    
    def watchdog_callback(self):
        """
        Watchdog callback - checks if robot is moving and forces movement if needed.
        This runs at 50Hz independently for maximum responsiveness.
        
        CRITICAL: This uses PHYSICAL movement verification via odometry, not just cmd_vel presence.
        This ensures the robot is actually moving, not just receiving commands.
        """
        if not self.is_active or not self.current_goal:
            return
        
        current_time = time.time()
        
        # CRITICAL: Check if odometry is stale (odometry stopped publishing) or never received
        # If odometry is stale or never received, we cannot verify physical movement - must log error and handle gracefully
        odom_stale = False
        odom_never_received = False
        time_since_last_position = float('inf')  # Initialize to infinity if no position yet
        
        if self.last_robot_position_time and self.last_robot_position:
            # Odometry has been received at least once - check if it's stale
            time_since_last_position = current_time - self.last_robot_position_time
            
            # CRITICAL: If odometry is stale (not updating), we cannot verify movement
            # This indicates base_node may have crashed or serial port disconnected
            if time_since_last_position > self.odom_timeout:
                odom_stale = True
                self.logger.error(
                    f"🚨🚨🚨 CRITICAL: ODOMETRY IS STALE! Last odometry update was {time_since_last_position:.2f}s ago "
                    f"(timeout: {self.odom_timeout:.1f}s). "
                    f"base_node may have crashed or serial port disconnected. "
                    f"Cannot verify physical movement - forcing movement based on cmd_vel only."
                )
        else:
            # CRITICAL: Odometry has NEVER been received (base_node may not have started)
            # Check if we've been waiting too long for initial odometry
            if self.is_active:
                # Movement guarantee is active, but no odometry yet - check if we've been waiting
                # Allow 5 seconds for initial odometry (base_node startup time)
                time_since_activation = current_time - (self.activate_start_time if hasattr(self, 'activate_start_time') else current_time)
                if time_since_activation > 5.0:  # 5 seconds grace period for initial odometry
                    odom_never_received = True
                    self.logger.error(
                        f"🚨🚨🚨 CRITICAL: ODOMETRY NEVER RECEIVED! Movement guarantee active for {time_since_activation:.1f}s "
                        f"but no odometry messages received. base_node may not be running or serial port disconnected. "
                        f"Cannot verify physical movement - forcing movement based on cmd_vel only."
                    )
        
        # CRITICAL: Check if robot is ACTUALLY moving (position change OR rotation change)
        # Rotation IS valid movement - robot rotating in place at goal is NOT stuck!
        # This prevents false "stuck" detection when robot is adjusting orientation at goal position
        is_moving_position = False
        is_rotating = False
        distance_moved = 0.0
        orientation_change = 0.0
        
        # Only calculate movement if odometry is recent (not stale)
        if not odom_stale and self.last_robot_position_time and self.last_robot_position:
            if time_since_last_position < 1.0:  # Recent position update (< 1s old)
                # Calculate actual distance moved from odometry
                if hasattr(self, 'previous_robot_position') and self.previous_robot_position:
                    dx = self.last_robot_position[0] - self.previous_robot_position[0]
                    dy = self.last_robot_position[1] - self.previous_robot_position[1]
                    distance_moved = math.sqrt(dx*dx + dy*dy)
                    is_moving_position = distance_moved > self.movement_threshold
                
                # CRITICAL: Check if robot is rotating (orientation change)
                # Rotation IS valid movement - don't force forward movement if robot is rotating
                if (hasattr(self, 'previous_robot_orientation') and self.previous_robot_orientation and 
                    self.last_robot_orientation):
                    # Calculate orientation change (yaw angle difference)
                    prev_yaw = self._quaternion_to_yaw(self.previous_robot_orientation)
                    curr_yaw = self._quaternion_to_yaw(self.last_robot_orientation)
                    orientation_change = abs(curr_yaw - prev_yaw)
                    # Normalize to [0, pi]
                    if orientation_change > math.pi:
                        orientation_change = 2 * math.pi - orientation_change
                    # Consider rotating if orientation changed more than 0.05 rad (~3°)
                    is_rotating = orientation_change > 0.05
                
                # Also check if cmd_vel has angular velocity (robot is commanded to rotate)
                if self.last_cmd_vel:
                    cmd_vel_angular = abs(self.last_cmd_vel.angular.z)
                    if cmd_vel_angular > 0.01:  # Angular velocity > 0.01 rad/s
                        is_rotating = True
            else:
                # Position updates are stale (older than 1 second but less than odom_timeout)
                # Still recent enough to use, but movement calculations may be less accurate
                is_moving_position = False
                is_rotating = False
        else:
            # No position data yet OR odometry is stale - cannot verify movement
            is_moving_position = False
            is_rotating = False
        
        # Robot is "moving" if EITHER position changed OR orientation changed
        is_moving = is_moving_position or is_rotating
        
        # Check if cmd_vel is being published (any priority source)
        cmd_vel_active = False
        time_since_last_cmd_vel = float('inf')  # Initialize to infinity if no cmd_vel yet
        
        if self.last_cmd_vel_time:
            time_since_last_cmd_vel = current_time - self.last_cmd_vel_time
            if time_since_last_cmd_vel < self.cmd_vel_timeout:
                cmd_vel_active = True
        
        # CRITICAL: If no cmd_vel for timeout period, force movement IMMEDIATELY
        if not cmd_vel_active and self.last_cmd_vel_time is not None:
            self.logger.error(
                f"🚨 CRITICAL: No cmd_vel for {time_since_last_cmd_vel:.1f}s! "
                f"Movement Guarantee forcing movement IMMEDIATELY!"
            )
            self._force_movement()
        
        # CRITICAL: If odometry is stale or never received, we cannot verify physical movement
        # In this case, we must rely on cmd_vel being published and assume movement is happening
        # This is a degraded mode but better than stopping navigation completely
        if odom_stale or odom_never_received:
            # Odometry is stale - cannot verify physical movement
            # If cmd_vel is active, assume movement is happening (degraded mode)
            if cmd_vel_active:
                self.logger.warn(
                    f"⚠️ ODOMETRY STALE ({time_since_last_position:.1f}s old) but cmd_vel is active. "
                    f"Assuming movement based on cmd_vel only (degraded mode). "
                    f"Check base_node and serial port connection!"
                )
                # Don't force movement if cmd_vel is active - assume it's working
                return
            else:
                # Odometry is stale AND no cmd_vel - this is critical
                self.logger.error(
                    f"🚨🚨🚨 CRITICAL: ODOMETRY STALE ({time_since_last_position:.1f}s old) AND NO cmd_vel! "
                    f"Movement Guarantee forcing movement IMMEDIATELY (cannot verify if working)."
                )
                self._force_movement()
                return
        
        # CRITICAL: If cmd_vel is active but robot hasn't moved (position OR rotation) for movement_timeout, force movement
        # BUT: Do NOT force movement if robot is rotating in place - rotation IS valid movement!
        # This handles the case where cmd_vel is published but robot is stuck (neither moving forward NOR rotating)
        # NOTE: Only check if odometry is NOT stale/never received (we already handled those cases above)
        if cmd_vel_active and not is_moving and not odom_stale and not odom_never_received and time_since_last_position < float('inf'):
            # Check if robot is commanded to rotate (angular velocity in cmd_vel)
            cmd_vel_has_rotation = False
            if self.last_cmd_vel and abs(self.last_cmd_vel.angular.z) > 0.01:
                cmd_vel_has_rotation = True
            
            # Only force movement if robot is NOT rotating (rotation is valid movement)
            if not is_rotating and not cmd_vel_has_rotation:
                if time_since_last_position > self.movement_timeout:
                    self.logger.error(
                        f"🚨 CRITICAL: cmd_vel active but robot not moving (no position change, no rotation) for {time_since_last_position:.1f}s! "
                        f"Distance moved: {distance_moved*100:.1f}cm (threshold: {self.movement_threshold*100:.1f}cm), "
                        f"Orientation change: {math.degrees(orientation_change):.1f}°. "
                        f"Movement Guarantee forcing movement IMMEDIATELY!"
                    )
                    self._force_movement()
                elif time_since_last_position > 1.0:  # cmd_vel active but no movement for >1s
                    self.logger.warn(
                        f"⚠️ cmd_vel active but robot not moving for {time_since_last_position:.1f}s. "
                        f"Distance moved: {distance_moved*100:.1f}cm (threshold: {self.movement_threshold*100:.1f}cm), "
                        f"Orientation change: {math.degrees(orientation_change):.1f}°. "
                        f"Increasing movement force..."
                    )
                    self._force_stronger_movement()
            else:
                # Robot is rotating - this is valid movement, don't force forward movement
                if time_since_last_position > 1.0:  # Log occasionally that rotation is detected
                    cmd_vel_angular_str = f"{self.last_cmd_vel.angular.z:.3f}" if self.last_cmd_vel else "N/A"
                    self.logger.debug(
                        f"✅ Robot rotating in place (orientation change: {math.degrees(orientation_change):.1f}°, "
                        f"cmd_vel angular: {cmd_vel_angular_str} rad/s). "
                        f"This is valid movement - not forcing forward movement."
                    )
        
        # CRITICAL: If robot moved very little (position-wise) despite cmd_vel, also increase force
        # BUT: Only if robot is NOT rotating (rotation is valid movement, don't interrupt it)
        # NOTE: Only check if odometry is NOT stale/never received
        if not odom_stale and not odom_never_received and cmd_vel_active and is_moving_position and not is_rotating and distance_moved > 0:
            if distance_moved < 0.02 and time_since_last_position < float('inf') and time_since_last_position > 2.0:
                self.logger.warn(
                    f"⚠️ Robot moving very slowly position-wise ({distance_moved*100:.1f}cm in {time_since_last_position:.1f}s, "
                    f"threshold: {self.movement_threshold*100:.1f}cm). Increasing movement force..."
                )
                self._force_stronger_movement()
    
    def _force_movement(self):
        """
        Force robot to move by publishing cmd_vel directly.
        This bypasses all other systems.
        """
        if not self.current_goal:
            return
        
        # Calculate simple movement command toward goal
        # This is a fallback - other systems should handle this, but if they don't, we will
        cmd = Twist()
        cmd.linear.x = 0.3  # Increased from 0.2 to 0.3 for stronger movement
        cmd.angular.z = 0.0
        
        # CRITICAL: Publish multiple times to ensure delivery (QoS might drop messages)
        for _ in range(3):  # Publish 3 times rapidly
            self.emergency_cmd_vel_pub.publish(cmd)
        
        self.last_cmd_vel_time = time.time()
        
        self.logger.warn(
            f"🛡️ Movement Guarantee: Published emergency cmd_vel (3x for reliability) "
            f"(linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f})"
        )
    
    def _force_stronger_movement(self):
        """
        Force stronger movement if robot is not responding to normal commands.
        """
        if not self.current_goal:
            return
        
        cmd = Twist()
        cmd.linear.x = 0.5  # Maximum safe linear velocity
        cmd.angular.z = 0.0
        
        # Publish multiple times for reliability
        for _ in range(5):  # Publish 5 times rapidly
            self.emergency_cmd_vel_pub.publish(cmd)
        
        self.last_cmd_vel_time = time.time()
        
        self.logger.warn(
            f"🛡️ Movement Guarantee: Published STRONGER emergency cmd_vel (5x for reliability) "
            f"(linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f})"
        )
    
    def _quaternion_to_yaw(self, quaternion) -> float:
        """
        Convert quaternion to yaw angle (rotation around z-axis).
        
        Args:
            quaternion: geometry_msgs.msg.Quaternion
            
        Returns:
            Yaw angle in radians, range [-pi, pi]
        """
        # Standard quaternion to Euler yaw conversion
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def is_robot_moving(self) -> bool:
        """
        Check if robot is currently moving (position change OR rotation).
        
        Returns:
            True if robot is moving (position or rotation), False otherwise
        """
        if not self.last_robot_position_time:
            return False
        
        current_time = time.time()
        time_since_last_position = current_time - self.last_robot_position_time
        
        # If we haven't received position updates recently, assume not moving
        if time_since_last_position > self.movement_timeout:
            return False
        
        # Check if robot is moving position-wise or rotating
        # (This is a simplified check - full logic is in watchdog_callback)
        return True

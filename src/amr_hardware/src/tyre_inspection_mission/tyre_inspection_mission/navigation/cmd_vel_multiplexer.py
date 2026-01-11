#!/usr/bin/env python3
"""
Priority-based cmd_vel Multiplexer

This node subscribes to cmd_vel from multiple sources and publishes the
highest-priority command to /cmd_vel. Priority levels:
1. Emergency (movement_guarantee) - Highest priority
2. Direct Control (direct_navigation_fallback) - High priority
3. Nav2 Controller - Medium priority
4. Teleop/Manual - Lowest priority

Uses RELIABLE QoS with TRANSIENT_LOCAL durability to guarantee message delivery.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time


class CmdVelMultiplexer(Node):
    """
    Priority-based cmd_vel multiplexer.
    
    This node subscribes to cmd_vel from multiple sources and publishes the
    highest-priority command to /cmd_vel. Priority levels:
    1. Emergency (movement_guarantee) - Highest priority
    2. Direct Control (direct_navigation_fallback) - High priority
    3. Nav2 Controller - Medium priority
    4. Teleop/Manual - Lowest priority
    
    Uses RELIABLE QoS with TRANSIENT_LOCAL durability to guarantee message delivery.
    """
    
    def __init__(self):
        super().__init__('cmd_vel_multiplexer')
        
        # QoS Profile: RELIABLE with TRANSIENT_LOCAL durability
        # This ensures messages are not lost and persist for late subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20  # Buffer 20 messages
        )
        
        # Subscribe to cmd_vel from each priority level
        # Priority 1: Emergency (movement_guarantee)
        self.emergency_sub = self.create_subscription(
            Twist,
            '/cmd_vel/emergency',
            lambda msg: self.cmd_vel_callback(msg, priority=1),
            qos_profile
        )
        
        # Priority 2: Direct Control (direct_navigation_fallback)
        self.direct_control_sub = self.create_subscription(
            Twist,
            '/cmd_vel/direct_control',
            lambda msg: self.cmd_vel_callback(msg, priority=2),
            qos_profile
        )
        
        # Priority 3: Nav2 Controller
        self.nav2_sub = self.create_subscription(
            Twist,
            '/cmd_vel/nav2',
            lambda msg: self.cmd_vel_callback(msg, priority=3),
            qos_profile
        )
        
        # Priority 4: Teleop/Manual (lowest priority)
        self.teleop_sub = self.create_subscription(
            Twist,
            '/cmd_vel/teleop',
            lambda msg: self.cmd_vel_callback(msg, priority=4),
            qos_profile
        )
        
        # Publisher for final cmd_vel (highest priority command)
        # CRITICAL: Use RELIABLE QoS to guarantee delivery to hardware
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )
        
        # Store latest command from each priority level
        # Format: {priority: (cmd_vel_msg, timestamp)}
        self.commands = {
            1: None,  # Emergency
            2: None,  # Direct Control
            3: None,  # Nav2
            4: None   # Teleop
        }
        
        # Command timeout: if a priority hasn't published in 1 second, consider it inactive
        self.command_timeout = 1.0  # seconds
        
        # Publish timer: 50Hz (hardware limit) to ensure continuous command stream
        self.publish_timer = self.create_timer(0.02, self.publish_highest_priority)
        
        # Track last published command for logging and continuity
        self.last_published_priority = None
        self.last_published_time = time.time()
        self.last_published_cmd = None  # CRITICAL: Store last published command for continuity during gaps
        
        # Track when each priority last published (for diagnostics)
        self.last_publish_times = {1: None, 2: None, 3: None, 4: None}
        
        # Grace period: Republish last command for this duration even if source hasn't updated
        # This ensures continuity during brief gaps (e.g., direct nav publishes at 0.5s intervals)
        self.command_grace_period = 0.5  # seconds - allow last command to be republished for 0.5s
        
        self.get_logger().info("✅ CmdVelMultiplexer initialized - Priority arbitration active")
        self.get_logger().info("   Priority 1: /cmd_vel/emergency (Movement Guarantee)")
        self.get_logger().info("   Priority 2: /cmd_vel/direct_control (Direct Navigation)")
        self.get_logger().info("   Priority 3: /cmd_vel/nav2 (Nav2 Controller)")
        self.get_logger().info("   Priority 4: /cmd_vel/teleop (Manual Control)")
        self.get_logger().info("   Output: /cmd_vel (RELIABLE QoS, 50Hz)")
    
    def cmd_vel_callback(self, msg: Twist, priority: int):
        """Store latest command from a priority level."""
        self.commands[priority] = (msg, time.time())
        self.last_publish_times[priority] = time.time()
        self.get_logger().debug(
            f"Received cmd_vel from priority {priority}: "
            f"linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}"
        )
    
    def is_non_zero(self, msg: Twist) -> bool:
        """Check if command is non-zero (robot should move)."""
        return abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001
    
    def is_command_stale(self, timestamp: float) -> bool:
        """Check if command is stale (hasn't been updated recently)."""
        if timestamp is None:
            return True
        return time.time() - timestamp > self.command_timeout
    
    def publish_highest_priority(self):
        """
        Publish highest-priority command (even if zero - zero from high priority is an explicit stop).
        
        CRITICAL: Higher priority ALWAYS wins, even if it's zero. Zero from Priority 1 (Emergency)
        is an explicit emergency stop that must override all lower priorities, including non-zero
        commands from Nav2 or teleop.
        
        Priority order: Emergency > Direct Control > Nav2 > Teleop
        Only publishes if command is recent (within timeout).
        """
        current_time = time.time()
        
        # Find highest-priority command that is not stale
        # CRITICAL: Higher priority ALWAYS wins, even if zero (zero = explicit stop command)
        selected_priority = None
        selected_cmd = None
        
        for priority in [1, 2, 3, 4]:  # Priority order: highest to lowest
            if self.commands[priority] is not None:
                cmd_msg, timestamp = self.commands[priority]
                
                # Check if command is stale
                if self.is_command_stale(timestamp):
                    # Clear stale command
                    self.commands[priority] = None
                    continue
                
                # CRITICAL FIX: Higher priority ALWAYS wins, even if zero
                # Zero from high priority (e.g., emergency stop) is an explicit override command
                # that must override all lower priorities, regardless of their values.
                # Select this priority immediately (don't check lower priorities).
                selected_priority = priority
                selected_cmd = cmd_msg
                break
        
        # Publish selected command
        if selected_cmd is not None:
            self.cmd_vel_pub.publish(selected_cmd)
            
            # CRITICAL: Store last published command for continuity during gaps
            self.last_published_cmd = selected_cmd
            
            # Log if priority changed or if publishing zero from high priority (emergency stop)
            priority_changed = self.last_published_priority != selected_priority
            is_zero_from_high_priority = selected_priority <= 2 and not self.is_non_zero(selected_cmd)
            
            if priority_changed:
                zero_note = " (EMERGENCY STOP)" if is_zero_from_high_priority else ""
                self.get_logger().info(
                    f"📢 Publishing cmd_vel from priority {selected_priority}{zero_note}: "
                    f"linear={selected_cmd.linear.x:.3f}, angular={selected_cmd.angular.z:.3f}"
                )
                self.last_published_priority = selected_priority
            
            self.last_published_time = current_time
        else:
            # No active commands selected - check if we should republish last command for continuity
            time_since_last_publish = current_time - self.last_published_time
            
            if (self.last_published_cmd is not None and 
                time_since_last_publish < self.command_grace_period):
                # CRITICAL: Republish last command during grace period to maintain continuity
                # This handles brief gaps (e.g., direct nav publishes at 0.5s intervals, but check happens at 0.5s + 0.02s)
                # Only republish non-zero commands - don't republish zero (would cause robot to stop)
                if self.is_non_zero(self.last_published_cmd):
                    self.cmd_vel_pub.publish(self.last_published_cmd)
                    # Don't update last_published_time - keep original timestamp for grace period calculation
                    # This ensures we don't extend the grace period indefinitely
                    self.get_logger().debug(
                        f"🔄 Republishing last cmd_vel for continuity (grace period: {time_since_last_publish:.3f}s < {self.command_grace_period:.3f}s). "
                        f"Priority: {self.last_published_priority}"
                    )
                else:
                    # Last command was zero - publish zero (explicit stop, e.g., direct nav deactivated)
                    zero_cmd = Twist()
                    self.cmd_vel_pub.publish(zero_cmd)
                    # CRITICAL: Store zero command to prevent republishing non-zero on next cycle
                    self.last_published_cmd = zero_cmd
                    self.last_published_time = current_time  # Update time for zero command
            else:
                # No commands AND grace period expired - publish zero (safety)
                # Only log if this is unexpected (recently active priorities)
                zero_cmd = Twist()
                self.cmd_vel_pub.publish(zero_cmd)
                # CRITICAL: Store zero command to prevent republishing non-zero on next cycle
                if self.last_published_cmd is None or self.is_non_zero(self.last_published_cmd):
                    # Only update if last command was non-zero or None (don't overwrite zero with zero)
                    self.last_published_cmd = zero_cmd
                    self.last_published_time = current_time  # Update time for zero command
                
                # Log if we're publishing zero when we expect movement (but throttle logging)
                if time_since_last_publish > 2.0:
                    # Check if any priority has been active recently
                    recently_active = False
                    for priority, last_time in self.last_publish_times.items():
                        if last_time and (current_time - last_time) < 5.0:
                            recently_active = True
                            break
                    
                    if recently_active:
                        self.get_logger().warn(
                            f"⚠️ No active cmd_vel commands (grace period expired: {time_since_last_publish:.2f}s). "
                            f"Publishing zero (safety). Last active priority: {self.last_published_priority}"
                        )
                        self.last_published_time = current_time
                        self.last_published_cmd = None  # Clear stored command after grace period
    


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMultiplexer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

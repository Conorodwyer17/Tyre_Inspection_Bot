#!/usr/bin/env python3
"""
Command Effectiveness Monitor

This diagnostic node monitors cmd_vel commands and odometry to detect when
commands are sent but robot doesn't move (exposing deadzone or hardware issues).

CRITICAL: This exposes hidden failures where:
- cmd_vel commands are published but robot doesn't move
- Small commands are below hardware deadzone threshold
- Hardware/ESP32 firmware issues prevent movement
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time
import math


class CommandEffectivenessMonitor(Node):
    """
    Monitor cmd_vel commands and odometry to detect ineffective commands.
    
    Tracks:
    - Commands sent vs actual movement
    - Small commands that don't result in movement (deadzone detection)
    - Command-to-movement delay
    """
    
    def __init__(self):
        super().__init__('command_effectiveness_monitor')
        
        # QoS profiles for reliable monitoring
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )
        
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            cmd_vel_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos
        )
        
        # State tracking
        self.last_cmd_vel = None
        self.last_cmd_vel_time = None
        self.last_odom_position = None
        self.last_odom_time = None
        self.last_odom_velocity = None
        
        # Effectiveness tracking
        self.commands_sent = 0
        self.commands_with_movement = 0
        self.commands_without_movement = 0
        self.small_commands_ignored = 0
        
        # Parameters
        self.movement_threshold = 0.01  # 1 cm - minimum movement to count as "moved"
        self.velocity_threshold = 0.005  # 0.5 cm/s - minimum velocity to count as moving
        self.cmd_effectiveness_window = 0.5  # seconds - check movement within this window after command
        self.small_cmd_threshold = 0.05  # m/s - commands below this are considered "small"
        
        # Timer for periodic effectiveness reporting
        self.report_timer = self.create_timer(10.0, self.report_effectiveness)
        
        self.get_logger().info("✅ Command Effectiveness Monitor initialized")
        self.get_logger().info(f"   Movement threshold: {self.movement_threshold*100:.1f} cm")
        self.get_logger().info(f"   Velocity threshold: {self.velocity_threshold*100:.1f} cm/s")
        self.get_logger().info(f"   Small command threshold: {self.small_cmd_threshold:.3f} m/s")
    
    def cmd_vel_callback(self, msg: Twist):
        """Track cmd_vel commands for effectiveness analysis."""
        current_time = time.time()
        
        cmd_magnitude = math.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.angular.z**2)
        is_non_zero = cmd_magnitude > 0.001
        
        if is_non_zero:
            self.commands_sent += 1
            self.last_cmd_vel = msg
            self.last_cmd_vel_time = current_time
            
            # Check if command is small (may be below deadzone)
            if cmd_magnitude < self.small_cmd_threshold:
                self.small_commands_ignored += 1
                self.get_logger().warn(
                    f"⚠️ Small cmd_vel command: linear={msg.linear.x:.4f} m/s, "
                    f"angular={msg.angular.z:.4f} rad/s (magnitude={cmd_magnitude:.4f} < {self.small_cmd_threshold:.3f}). "
                    f"May be below hardware deadzone - robot may not move.",
                    throttle_duration_sec=2.0
                )
    
    def odom_callback(self, msg: Odometry):
        """Track odometry to correlate with cmd_vel commands."""
        current_time = time.time()
        current_position = msg.pose.pose.position
        current_velocity = msg.twist.twist.linear.x
        
        # Update position tracking
        if self.last_odom_position is None:
            self.last_odom_position = current_position
            self.last_odom_time = current_time
            self.last_odom_velocity = current_velocity
            return
        
        # Check if we had a recent command and see if robot moved
        if self.last_cmd_vel_time and self.last_cmd_vel:
            time_since_cmd = current_time - self.last_cmd_vel_time
            
            # Only check effectiveness within window after command
            if time_since_cmd <= self.cmd_effectiveness_window:
                # Calculate movement since command
                dx = current_position.x - self.last_odom_position.x
                dy = current_position.y - self.last_odom_position.y
                distance_moved = math.sqrt(dx**2 + dy**2)
                
                # Check if robot moved (position change or velocity)
                moved = (distance_moved > self.movement_threshold) or (abs(current_velocity) > self.velocity_threshold)
                
                if moved:
                    self.commands_with_movement += 1
                else:
                    # Command sent but no movement - potential deadzone or hardware issue
                    self.commands_without_movement += 1
                    cmd_magnitude = math.sqrt(
                        self.last_cmd_vel.linear.x**2 + 
                        self.last_cmd_vel.linear.y**2 + 
                        self.last_cmd_vel.angular.z**2
                    )
                    
                    if cmd_magnitude > 0.001:  # Only warn for non-zero commands
                        self.get_logger().warn(
                            f"⚠️ COMMAND INEFFECTIVE: cmd_vel sent (linear={self.last_cmd_vel.linear.x:.4f}, "
                            f"angular={self.last_cmd_vel.angular.z:.4f}) but robot didn't move. "
                            f"Distance moved: {distance_moved*100:.2f} cm, velocity: {current_velocity:.4f} m/s. "
                            f"Command may be below hardware deadzone or hardware issue.",
                            throttle_duration_sec=1.0
                        )
        
        # Update tracking
        self.last_odom_position = current_position
        self.last_odom_time = current_time
        self.last_odom_velocity = current_velocity
    
    def report_effectiveness(self):
        """Periodically report command effectiveness statistics."""
        if self.commands_sent == 0:
            return
        
        effectiveness_rate = (self.commands_with_movement / self.commands_sent * 100) if self.commands_sent > 0 else 0.0
        
        self.get_logger().info(
            f"📊 Command Effectiveness Report:\n"
            f"   Commands sent: {self.commands_sent}\n"
            f"   Commands with movement: {self.commands_with_movement} ({effectiveness_rate:.1f}%)\n"
            f"   Commands without movement: {self.commands_without_movement} "
            f"({100-effectiveness_rate:.1f}% - may indicate deadzone/hardware issues)\n"
            f"   Small commands (< {self.small_cmd_threshold:.3f} m/s): {self.small_commands_ignored}"
        )
        
        if effectiveness_rate < 50.0:
            self.get_logger().error(
                f"❌ CRITICAL: Low command effectiveness ({effectiveness_rate:.1f}%)! "
                f"More than half of commands don't result in movement. "
                f"Check hardware deadzone, ESP32 firmware, or motor calibration."
            )


def main(args=None):
    rclpy.init(args=args)
    node = CommandEffectivenessMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

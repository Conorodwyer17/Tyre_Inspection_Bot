#!/usr/bin/env python3
"""
Movement Diagnostic Node

Monitors cmd_vel publishing and robot movement to diagnose why the robot stops.
This node provides real-time diagnostics to identify the exact failure point.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from collections import deque
from datetime import datetime


class MovementDiagnostic(Node):
    """Diagnostic node to monitor robot movement and cmd_vel publishing."""
    
    def __init__(self):
        super().__init__('movement_diagnostic')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # State
        self.last_cmd_vel = None
        self.last_cmd_vel_time = None
        self.cmd_vel_history = deque(maxlen=100)  # Last 100 cmd_vel messages
        self.cmd_vel_count = 0  # Initialize counter
        self.start_time = time.time()  # Initialize start time for frequency calculation
        
        self.last_odom = None
        self.last_odom_time = None
        self.odom_history = deque(maxlen=100)  # Last 100 odom messages
        self.odom_count = 0  # Initialize counter
        
        self.last_position = None
        self.previous_position = None  # Initialize for movement tracking
        self.movement_detected = False
        self.stopped_detected = False
        
        # Diagnostic timer (1Hz)
        self.diagnostic_timer = self.create_timer(1.0, self.diagnostic_callback)
        
        self.get_logger().info("🔍 Movement Diagnostic Node started")
        self.get_logger().info("   Monitoring /cmd_vel and /odom topics")
        self.get_logger().info("   Publishing diagnostics at 1Hz")
    
    def cmd_vel_callback(self, msg: Twist):
        """Callback for cmd_vel messages."""
        current_time = time.time()
        
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = current_time
        self.cmd_vel_count += 1
        
        # Store in history
        self.cmd_vel_history.append({
            'time': current_time,
            'linear': msg.linear.x,
            'angular': msg.angular.z
        })
        
        # Check if command is non-zero
        is_non_zero = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        
        if is_non_zero:
            self.get_logger().debug(
                f"📤 cmd_vel received: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}"
            )
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry messages."""
        current_time = time.time()
        
        self.last_odom = msg
        self.last_odom_time = current_time
        self.odom_count += 1
        
        # Get current position
        current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Track previous position for distance calculation
        if self.last_position is not None:
            self.previous_position = self.last_position
        
        # Check for movement
        if self.last_position is not None:
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            distance = (dx**2 + dy**2)**0.5
            
            if distance > 0.01:  # 1cm threshold
                self.movement_detected = True
                self.stopped_detected = False
            elif distance < 0.001:  # 1mm threshold
                self.stopped_detected = True
        
        self.last_position = current_position
        
        # Store in history
        self.odom_history.append({
            'time': current_time,
            'x': current_position[0],
            'y': current_position[1]
        })
    
    def diagnostic_callback(self):
        """Periodic diagnostic callback with enhanced analysis."""
        current_time = time.time()
        
        # Calculate frequencies
        cmd_vel_freq = self.cmd_vel_count / (current_time - self.start_time) if (current_time - self.start_time) > 0 else 0.0
        odom_freq = self.odom_count / (current_time - self.start_time) if (current_time - self.start_time) > 0 else 0.0
        
        # Determine if cmd_vel is being received
        cmd_vel_active = (current_time - self.last_cmd_vel_time) < 1.0 if self.last_cmd_vel_time else False
        cmd_vel_non_zero = False
        if self.last_cmd_vel:
            cmd_vel_non_zero = abs(self.last_cmd_vel.linear.x) > 0.01 or abs(self.last_cmd_vel.angular.z) > 0.01
        
        # Determine if odom is being received
        odom_active = (current_time - self.last_odom_time) < 1.0 if self.last_odom_time else False
        robot_moving = False
        distance_moved = 0.0
        if self.last_position and self.previous_position:
            # Calculate actual distance moved
            dx = self.last_position[0] - self.previous_position[0]
            dy = self.last_position[1] - self.previous_position[1]
            distance_moved = math.sqrt(dx*dx + dy*dy)
            robot_moving = distance_moved > 0.01  # 1cm threshold
        
        # CRITICAL: Enhanced analysis
        time_since_last_cmd = current_time - self.last_cmd_vel_time if self.last_cmd_vel_time else float('inf')
        time_since_last_odom = current_time - self.last_odom_time if self.last_odom_time else float('inf')
        
        # Identify issues with severity levels
        critical_issues = []
        warnings = []
        info = []
        
        if not cmd_vel_active:
            critical_issues.append(f"❌ CRITICAL: No cmd_vel messages for {time_since_last_cmd:.1f}s")
        elif not cmd_vel_non_zero:
            warnings.append("⚠️ cmd_vel is zero (robot should be stopped)")
        elif cmd_vel_non_zero and not robot_moving and time_since_last_cmd < 5.0:
            critical_issues.append("🚨 CRITICAL: cmd_vel is non-zero but robot is NOT moving!")
            critical_issues.append(f"   cmd_vel: linear={self.last_cmd_vel.linear.x:.3f}, angular={self.last_cmd_vel.angular.z:.3f}")
            critical_issues.append(f"   Distance moved: {distance_moved*100:.1f}cm (threshold: 1cm)")
        
        if not odom_active:
            critical_issues.append(f"❌ CRITICAL: No odom messages for {time_since_last_odom:.1f}s")
        
        if cmd_vel_freq < 10.0 and cmd_vel_active:
            warnings.append(f"⚠️ Low cmd_vel frequency: {cmd_vel_freq:.1f}Hz (expected >20Hz)")
        
        if robot_moving:
            info.append(f"✅ Robot is moving: {distance_moved*100:.1f}cm since last check")
        
        # Print diagnostic report
        report_lines = [
            f"\n{'='*70}",
            f"MOVEMENT DIAGNOSTIC REPORT - {datetime.now().strftime('%H:%M:%S')}",
            f"{'='*70}",
            f"cmd_vel: freq={cmd_vel_freq:.1f}Hz, active={cmd_vel_active}, non_zero={cmd_vel_non_zero}",
            f"  Last: linear={self.last_cmd_vel.linear.x:.3f} m/s, angular={self.last_cmd_vel.angular.z:.3f} rad/s" if self.last_cmd_vel else "  Last: N/A",
            f"  Time since last: {time_since_last_cmd:.2f}s",
            f"odom: freq={odom_freq:.1f}Hz, active={odom_active}, moving={robot_moving}",
            f"  Last position: ({self.last_position[0]:.2f}, {self.last_position[1]:.2f})" if self.last_position else "  Last position: N/A",
            f"  Time since last: {time_since_last_odom:.2f}s",
        ]
        
        if critical_issues:
            report_lines.append(f"\n🚨 CRITICAL ISSUES ({len(critical_issues)}):")
            for issue in critical_issues:
                report_lines.append(f"  {issue}")
        
        if warnings:
            report_lines.append(f"\n⚠️ WARNINGS ({len(warnings)}):")
            for warning in warnings:
                report_lines.append(f"  {warning}")
        
        if info:
            report_lines.append(f"\n✅ INFO ({len(info)}):")
            for item in info:
                report_lines.append(f"  {item}")
        
        if not critical_issues and not warnings:
            report_lines.append(f"\n✅ ALL SYSTEMS NOMINAL")
        
        report_lines.append(f"{'='*70}\n")
        
        # Log at appropriate level
        if critical_issues:
            self.get_logger().error("\n".join(report_lines))
        elif warnings:
            self.get_logger().warn("\n".join(report_lines))
        else:
            self.get_logger().info("\n".join(report_lines))


def main(args=None):
    rclpy.init(args=args)
    node = MovementDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

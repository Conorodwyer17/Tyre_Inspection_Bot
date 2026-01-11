#!/usr/bin/env python3
"""
Hardware Verification System

This module verifies that hardware is actually processing cmd_vel commands
by monitoring odometry and comparing it to the commands we send.
"""

import time
import math
from typing import Optional, Tuple
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class HardwareVerification:
    """
    Verifies that hardware is processing commands correctly.
    Monitors odometry to ensure movement matches commands.
    """
    
    def __init__(self, node: Node, logger):
        """
        Initialize hardware verification system.
        
        Args:
            node: ROS 2 node instance
            logger: ROS 2 logger instance
        """
        self.node = node
        self.logger = logger
        
        # Subscribers
        self.cmd_vel_sub = node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.odom_sub = node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # State
        self.last_cmd_vel = None
        self.last_cmd_vel_time = None
        self.last_odom_position = None
        self.last_odom_time = None
        self.last_odom_velocity = None
        
        # Verification parameters
        self.verification_timeout = 2.0  # seconds - if no movement after command, flag issue
        self.min_expected_movement = 0.02  # meters - minimum movement expected per second
        self.command_velocity_threshold = 0.05  # m/s - minimum command to expect movement
        
        # Statistics
        self.commands_sent = 0
        self.commands_verified = 0
        self.commands_failed = 0
        
        # Verification timer (5Hz - check every 0.2s)
        self.verification_timer = node.create_timer(0.2, self.verification_callback)
        
        self.logger.info("HardwareVerification initialized")
    
    def cmd_vel_callback(self, msg: Twist):
        """Track cmd_vel commands."""
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = time.time()
        
        # Only count non-zero commands
        if abs(msg.linear.x) > self.command_velocity_threshold or abs(msg.angular.z) > 0.1:
            self.commands_sent += 1
    
    def odom_callback(self, msg: Odometry):
        """Track odometry updates."""
        current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract velocity from odometry (if available)
        if hasattr(msg, 'twist') and hasattr(msg.twist, 'twist'):
            self.last_odom_velocity = (
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z
            )
        
        self.last_odom_position = current_position
        self.last_odom_time = time.time()
    
    def verification_callback(self):
        """Verify that hardware is processing commands."""
        current_time = time.time()
        
        # Check if we have recent commands
        if not self.last_cmd_vel_time:
            return
        
        time_since_command = current_time - self.last_cmd_vel_time
        
        # Only verify if command was sent recently
        if time_since_command > self.verification_timeout:
            return
        
        # Check if command is significant enough to expect movement
        cmd_linear = self.last_cmd_vel.linear.x
        cmd_angular = self.last_cmd_vel.angular.z
        
        if abs(cmd_linear) < self.command_velocity_threshold and abs(cmd_angular) < 0.1:
            return  # Command too small to verify
        
        # Check if we have recent odometry
        if not self.last_odom_time:
            self.logger.warn(
                f"⚠️ Hardware Verification: No odometry received. Cannot verify command execution."
            )
            return
        
        time_since_odom = current_time - self.last_odom_time
        
        if time_since_odom > 1.0:
            self.logger.warn(
                f"⚠️ Hardware Verification: Stale odometry ({time_since_odom:.1f}s old). "
                f"Cannot verify command execution."
            )
            return
        
        # Verify movement matches command
        # This is a simplified check - in practice, you'd want more sophisticated verification
        if self.last_odom_velocity:
            odom_linear = self.last_odom_velocity[0]
            odom_angular = self.last_odom_velocity[1]
            
            # Check if velocities match (within tolerance)
            linear_match = abs(odom_linear - cmd_linear) < 0.2  # 0.2 m/s tolerance
            angular_match = abs(odom_angular - cmd_angular) < 0.3  # 0.3 rad/s tolerance
            
            if not linear_match and abs(cmd_linear) > self.command_velocity_threshold:
                self.logger.warn(
                    f"⚠️ Hardware Verification: Linear velocity mismatch! "
                    f"Command: {cmd_linear:.3f} m/s, Odometry: {odom_linear:.3f} m/s"
                )
                self.commands_failed += 1
            elif linear_match and angular_match:
                self.commands_verified += 1
    
    def get_verification_stats(self) -> Tuple[int, int, int]:
        """
        Get verification statistics.
        
        Returns:
            (commands_sent, commands_verified, commands_failed)
        """
        return (self.commands_sent, self.commands_verified, self.commands_failed)
    
    def is_hardware_responding(self) -> bool:
        """
        Check if hardware is responding to commands.
        
        Returns:
            True if hardware appears to be responding, False otherwise
        """
        if not self.last_odom_time:
            return False
        
        current_time = time.time()
        time_since_odom = current_time - self.last_odom_time
        
        # Hardware is responding if we get odometry updates
        return time_since_odom < 1.0

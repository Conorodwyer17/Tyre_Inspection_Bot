#!/usr/bin/env python3
"""
Navigation Manager Module

Manages Nav2 interaction, goal updates, and navigation state.
Handles dynamic goal recalculation when robot gets close.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time
import math


class NavigationManager:
    """Manages navigation with dynamic goal updates"""
    
    def __init__(self, node, goal_planner):
        """
        Initialize navigation manager
        
        Args:
            node: ROS 2 node
            goal_planner: GoalPlanner instance
        """
        self.node = node
        self.goal_planner = goal_planner
        self.tf_buffer = goal_planner.tf_buffer
        
        # Nav2 action client
        self.nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.nav_server_ready = False
        
        # Navigation state
        self.current_goal_handle = None
        self.current_goal_pose = None
        self.goal_start_time = None
        self.navigation_active = False
        self.nav_timeout = 120.0  # seconds
        
        # Goal update parameters
        self.goal_update_interval = 2.0  # seconds - check for updates every 2s
        self.last_goal_update_check = 0.0
        
        # Navigation result tracking
        self.nav_result_received = False
        self.nav_result_status = None
        
        # Timer for goal updates
        self.goal_update_timer = node.create_timer(
            self.goal_update_interval,
            self._check_and_update_goal
        )
        
    def wait_for_server(self, timeout_sec=30.0):
        """Wait for Nav2 action server to be ready"""
        if self.nav_server_ready:
            if self.nav_client.server_is_ready():
                return True
            else:
                self.nav_server_ready = False
        
        self.node.get_logger().info("Waiting for Nav2 action server...")
        if self.nav_client.wait_for_server(timeout_sec=timeout_sec):
            self.nav_server_ready = True
            self.node.get_logger().info("✅ Nav2 action server ready")
            return True
        else:
            self.node.get_logger().error("❌ Nav2 action server not available")
            return False
    
    def navigate_to_pose(self, goal_pose, goal_callback=None, result_callback=None):
        """
        Navigate to a pose with automatic goal updates
        
        Args:
            goal_pose: PoseStamped goal
            goal_callback: Optional callback when goal is accepted
            result_callback: Optional callback when navigation completes
            
        Returns:
            True if goal was sent, False otherwise
        """
        # Validate goal
        if not self.goal_planner.validate_goal(goal_pose):
            self.node.get_logger().error("Cannot navigate: goal validation failed")
            return False
        
        # Ensure Nav2 is ready
        if not self.wait_for_server(timeout_sec=10.0):
            return False
        
        # Cancel any existing goal
        if self.navigation_active:
            self.cancel_navigation()
        
        # Store goal
        self.current_goal_pose = goal_pose
        self.goal_start_time = time.time()
        self.navigation_active = True
        self.nav_result_received = False
        self.nav_result_status = None
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send goal
        self.node.get_logger().info(
            f"🚀 Navigating to ({goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f}) in frame {goal_pose.header.frame_id}"
        )
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback: self._nav_feedback_callback(feedback, result_callback)
        )
        send_goal_future.add_done_callback(
            lambda future: self._goal_response_callback(future, goal_callback, result_callback)
        )
        
        return True
    
    def update_goal(self, new_goal_pose):
        """
        Update current navigation goal (recalculate if robot got close)
        
        Args:
            new_goal_pose: New goal pose to navigate to
            
        Returns:
            True if goal was updated, False otherwise
        """
        if not self.navigation_active:
            return False
        
        # Validate new goal
        if not self.goal_planner.validate_goal(new_goal_pose):
            self.node.get_logger().warn("Cannot update goal: validation failed")
            return False
        
        # Cancel current goal
        self.cancel_navigation()
        
        # Start navigation to new goal
        return self.navigate_to_pose(new_goal_pose)
    
    def cancel_navigation(self):
        """Cancel current navigation"""
        if self.current_goal_handle is not None:
            try:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                self.node.get_logger().info("Canceling navigation goal")
            except Exception as e:
                self.node.get_logger().warn(f"Error canceling goal: {e}")
        
        self.current_goal_handle = None
        self.current_goal_pose = None
        self.navigation_active = False
        self.nav_result_received = False
    
    def is_navigating(self):
        """Check if navigation is currently active"""
        return self.navigation_active and not self.nav_result_received
    
    def is_goal_reached(self, tolerance=None):
        """Check if current goal has been reached"""
        if self.current_goal_pose is None:
            return False
        return self.goal_planner.is_goal_reached(self.current_goal_pose, tolerance=tolerance)
    
    def get_navigation_status(self):
        """Get current navigation status"""
        return {
            'active': self.navigation_active,
            'goal_pose': self.current_goal_pose,
            'result_received': self.nav_result_received,
            'result_status': self.nav_result_status,
            'time_elapsed': time.time() - self.goal_start_time if self.goal_start_time else 0.0
        }
    
    def _check_and_update_goal(self):
        """Periodically check if goal needs to be recalculated"""
        if not self.navigation_active or self.current_goal_pose is None:
            return
        
        # Check if goal should be recalculated
        if self.goal_planner.should_recalculate_goal(self.current_goal_pose):
            self.node.get_logger().info("🔄 Goal recalculation needed - robot too close to current goal")
            # Note: Actual recalculation should be done by mission controller
            # with updated vehicle/tyre positions
    
    def _goal_response_callback(self, future, goal_callback, result_callback):
        """Handle goal response from Nav2"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("❌ Navigation goal rejected by Nav2")
            self.navigation_active = False
            self.current_goal_pose = None
            if goal_callback:
                goal_callback(False)
            return
        
        self.current_goal_handle = goal_handle
        self.node.get_logger().info("✅ Navigation goal accepted by Nav2")
        
        if goal_callback:
            goal_callback(True)
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._nav_result_callback(future, result_callback)
        )
    
    def _nav_feedback_callback(self, feedback, result_callback):
        """Handle navigation feedback"""
        # Check for timeout
        if self.goal_start_time and (time.time() - self.goal_start_time) > self.nav_timeout:
            self.node.get_logger().warn("Navigation timeout - canceling goal")
            self.cancel_navigation()
            if result_callback:
                result_callback(4)  # TIMEOUT
            return
        
        # Check if goal reached
        if self.is_goal_reached():
            self.node.get_logger().info("✅ Goal reached (within tolerance)")
            # Don't cancel here - let Nav2 complete naturally
    
    def _nav_result_callback(self, future, result_callback):
        """Handle navigation result"""
        try:
            result = future.result().result
            status = future.result().status
            
            self.nav_result_received = True
            self.nav_result_status = status
            self.navigation_active = False
            
            if status == 4:  # SUCCEEDED
                self.node.get_logger().info("✅ Navigation succeeded")
            elif status == 2:  # CANCELED
                self.node.get_logger().warn("⚠️ Navigation was canceled")
            elif status == 3:  # ABORTED
                self.node.get_logger().error("❌ Navigation was aborted")
            else:
                self.node.get_logger().warn(f"⚠️ Navigation completed with status: {status}")
            
            if result_callback:
                result_callback(status)
                
        except Exception as e:
            self.node.get_logger().error(f"Error processing navigation result: {e}")
            self.nav_result_received = True
            self.nav_result_status = 5  # Treat error as aborted
            self.navigation_active = False
            if result_callback:
                result_callback(5)

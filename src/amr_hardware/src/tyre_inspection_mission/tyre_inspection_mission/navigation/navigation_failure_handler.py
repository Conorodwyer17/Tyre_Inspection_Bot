#!/usr/bin/env python3
"""
Navigation Failure Handler Module

Handles navigation failures, unreachable goals, and obstacle avoidance.
Provides strategies for dealing with navigation issues during tyre inspection.

This module provides:
- Navigation failure detection and classification
- Unreachable goal handling
- Alternative path strategies
- Obstacle avoidance recommendations
"""

import math
from typing import Optional, Dict
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose


class NavigationFailureHandler:
    """
    Handles navigation failures and provides recovery strategies.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize navigation failure handler.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.max_retry_attempts = parameters.get('navigation_failure_max_retries', 2)
        self.retry_backoff_distance = parameters.get('retry_backoff_distance', 0.3)  # meters
        self.min_alternative_goal_distance = parameters.get('min_alternative_goal_distance', 0.5)  # meters
        self.unreachable_goal_timeout = parameters.get('unreachable_goal_timeout', 5.0)  # seconds
        
        # Failure tracking
        self.failure_count = 0
        self.last_failure_goal = None
        self.failure_reasons = []
        
        self.logger.info(
            f"NavigationFailureHandler initialized: "
            f"max_retries={self.max_retry_attempts}, "
            f"backoff_distance={self.retry_backoff_distance}m"
        )
    
    def handle_navigation_failure(
        self,
        goal: PoseStamped,
        result_code,
        result_message: str = "",
        current_robot_pose: Optional[PoseStamped] = None
    ) -> Dict:
        """
        Handle navigation failure and provide recovery strategy.
        
        Args:
            goal: Goal that failed
            result_code: Nav2 result code (SUCCEEDED, ABORTED, CANCELED, UNKNOWN)
            result_message: Optional result message
            current_robot_pose: Optional current robot pose
            
        Returns:
            Dict with:
                - 'should_retry': bool - Should retry navigation?
                - 'should_skip': bool - Should skip this goal?
                - 'alternative_goal': PoseStamped or None - Alternative goal to try
                - 'retry_strategy': str - Strategy to use (backoff, alternative, skip)
                - 'failure_reason': str - Reason for failure
                - 'recommendation': str - Overall recommendation
        """
        result = {
            'should_retry': False,
            'should_skip': False,
            'alternative_goal': None,
            'retry_strategy': 'skip',
            'failure_reason': 'unknown',
            'recommendation': 'skip'
        }
        
        # Classify failure
        failure_type = self._classify_failure(result_code, result_message)
        result['failure_reason'] = failure_type
        
        self.failure_count += 1
        self.last_failure_goal = goal
        self.failure_reasons.append(failure_type)
        
        self.logger.warn(
            f"Navigation failure detected: {failure_type} "
            f"(attempt {self.failure_count}/{self.max_retry_attempts})"
        )
        
        # Check if we should retry
        if self.failure_count < self.max_retry_attempts:
            # Try alternative strategies based on failure type
            if failure_type == 'unreachable' or failure_type == 'aborted':
                # Goal might be unreachable due to obstacle
                # Try alternative goal (back off from original)
                alternative = self._calculate_alternative_goal(
                    goal,
                    current_robot_pose,
                    self.retry_backoff_distance
                )
                if alternative:
                    result['should_retry'] = True
                    result['alternative_goal'] = alternative
                    result['retry_strategy'] = 'alternative_goal'
                    result['recommendation'] = 'retry_alternative'
                    self.logger.info(
                        f"Proposing alternative goal for retry: "
                        f"backed off by {self.retry_backoff_distance}m"
                    )
                else:
                    result['should_skip'] = True
                    result['recommendation'] = 'skip'
            elif failure_type == 'timeout':
                # Navigation timed out - might be stuck
                # Try backing off and retrying
                alternative = self._calculate_alternative_goal(
                    goal,
                    current_robot_pose,
                    self.retry_backoff_distance
                )
                if alternative:
                    result['should_retry'] = True
                    result['alternative_goal'] = alternative
                    result['retry_strategy'] = 'backoff'
                    result['recommendation'] = 'retry_backoff'
                else:
                    result['should_skip'] = True
                    result['recommendation'] = 'skip'
            elif failure_type == 'unknown' or failure_type == 'canceled':
                # Unknown or canceled - don't retry
                result['should_skip'] = True
                result['recommendation'] = 'skip'
            else:
                # Default: skip
                result['should_skip'] = True
                result['recommendation'] = 'skip'
        else:
            # Max retries reached
            self.logger.error(
                f"Max navigation retry attempts ({self.max_retry_attempts}) reached. "
                "Skipping goal."
            )
            result['should_skip'] = True
            result['recommendation'] = 'skip'
        
        return result
    
    def _classify_failure(self, result_code, result_message: str = "") -> str:
        """
        Classify navigation failure type.
        
        Args:
            result_code: Nav2 result code
            result_message: Optional result message
            
        Returns:
            Failure type string
        """
        # Import result codes
        from rclpy.action import ResultCode
        
        if result_code == ResultCode.SUCCEEDED:
            return 'succeeded'
        elif result_code == ResultCode.ABORTED:
            # Check message for clues
            if 'unreachable' in result_message.lower() or 'plan' in result_message.lower():
                return 'unreachable'
            return 'aborted'
        elif result_code == ResultCode.CANCELED:
            return 'canceled'
        elif result_code == ResultCode.UNKNOWN:
            return 'unknown'
        else:
            # Check message for timeout
            if 'timeout' in result_message.lower():
                return 'timeout'
            return 'unknown'
    
    def _calculate_alternative_goal(
        self,
        original_goal: PoseStamped,
        current_robot_pose: Optional[PoseStamped],
        backoff_distance: float
    ) -> Optional[PoseStamped]:
        """
        Calculate alternative goal by backing off from original.
        
        Args:
            original_goal: Original goal that failed
            current_robot_pose: Current robot pose (to back off toward)
            backoff_distance: Distance to back off
            
        Returns:
            Alternative goal pose, or None if calculation fails
        """
        try:
            if not original_goal:
                return None
            
            # If no robot pose, can't calculate alternative
            if not current_robot_pose:
                return None
            
            original_pos = original_goal.pose.position
            robot_pos = current_robot_pose.pose.position
            
            # Calculate vector from goal to robot (back off toward robot)
            dx = robot_pos.x - original_pos.x
            dy = robot_pos.y - original_pos.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.01:
                # Robot is at goal - can't back off meaningfully
                return None
            
            # Normalize direction
            dx = dx / distance
            dy = dy / distance
            
            # Calculate new position (back off from original goal)
            alternative_goal = PoseStamped()
            alternative_goal.header = original_goal.header
            alternative_goal.pose.position.x = original_pos.x + backoff_distance * dx
            alternative_goal.pose.position.y = original_pos.y + backoff_distance * dy
            alternative_goal.pose.position.z = original_pos.z
            
            # Keep same orientation
            alternative_goal.pose.orientation = original_goal.pose.orientation
            
            return alternative_goal
            
        except Exception as e:
            self.logger.error(f"Error calculating alternative goal: {e}", exc_info=True)
            return None
    
    def reset_failure_count(self):
        """Reset failure count (call when goal succeeds)."""
        self.failure_count = 0
        self.last_failure_goal = None
    
    def get_failure_statistics(self) -> Dict:
        """
        Get failure statistics.
        
        Returns:
            Dict with failure statistics
        """
        failure_counts = {}
        for reason in self.failure_reasons:
            failure_counts[reason] = failure_counts.get(reason, 0) + 1
        
        return {
            'total_failures': len(self.failure_reasons),
            'failure_breakdown': failure_counts,
            'current_failure_count': self.failure_count,
            'max_retries': self.max_retry_attempts
        }

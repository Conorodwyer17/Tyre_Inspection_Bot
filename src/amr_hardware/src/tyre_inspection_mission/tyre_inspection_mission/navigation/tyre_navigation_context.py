#!/usr/bin/env python3
"""
Tyre Navigation Context Manager

Maintains persistent context during tyre navigation to ensure we always know
which tyre we're targeting, even if navigation fails or system restarts.

This module provides:
- Current target tyre tracking
- Navigation state persistence per tyre
- Recovery from navigation failures
- Context validation
"""

from typing import Optional, Dict
from geometry_msgs.msg import PoseStamped


class TyreNavigationContext:
    """
    Manages persistent context during tyre navigation.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre navigation context manager.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Context state
        self.current_target_tyre_id = None
        self.current_target_tyre_index = None
        self.current_navigation_goal = None
        self.navigation_start_pose = None
        self.navigation_start_time = None
        
        # Tyre navigation status tracking
        self.tyre_navigation_status = {}  # tyre_id -> {status, attempts, last_error}
        
        # Parameters
        self.max_navigation_attempts_per_tyre = parameters.get('max_navigation_attempts_per_tyre', 3)
        self.context_timeout = parameters.get('navigation_context_timeout', 300.0)  # seconds
        
        self.logger.info("TyreNavigationContext initialized")
    
    def set_target_tyre(self, tyre_id: str, tyre_index: int, navigation_goal: Optional[PoseStamped] = None):
        """
        Set current target tyre.
        
        Args:
            tyre_id: ID of target tyre
            tyre_index: Index in tyres list
            navigation_goal: Optional navigation goal pose
        """
        self.current_target_tyre_id = tyre_id
        self.current_target_tyre_index = tyre_index
        self.current_navigation_goal = navigation_goal
        
        # Initialize status tracking if not exists
        if tyre_id not in self.tyre_navigation_status:
            self.tyre_navigation_status[tyre_id] = {
                'status': 'pending',
                'attempts': 0,
                'last_error': None,
                'last_attempt_time': None
            }
        
        # Update status
        self.tyre_navigation_status[tyre_id]['status'] = 'navigating'
        self.tyre_navigation_status[tyre_id]['attempts'] += 1
        
        self.logger.info(
            f"Set target tyre: {tyre_id} (index: {tyre_index})"
        )
    
    def get_current_target(self) -> Optional[Dict]:
        """
        Get current target tyre information.
        
        Returns:
            Dict with tyre_id, tyre_index, navigation_goal, or None if no target
        """
        if self.current_target_tyre_id is None:
            return None
        
        return {
            'tyre_id': self.current_target_tyre_id,
            'tyre_index': self.current_target_tyre_index,
            'navigation_goal': self.current_navigation_goal,
            'navigation_status': self.tyre_navigation_status.get(
                self.current_target_tyre_id,
                {}
            )
        }
    
    def mark_tyre_navigation_success(self, tyre_id: str):
        """
        Mark navigation to tyre as successful.
        
        Args:
            tyre_id: ID of tyre that was successfully reached
        """
        if tyre_id in self.tyre_navigation_status:
            self.tyre_navigation_status[tyre_id]['status'] = 'arrived'
            self.tyre_navigation_status[tyre_id]['last_error'] = None
        
        # Clear current target if it matches
        if self.current_target_tyre_id == tyre_id:
            self.current_target_tyre_id = None
            self.current_target_tyre_index = None
            self.current_navigation_goal = None
        
        self.logger.info(f"Marked tyre {tyre_id} navigation as successful")
    
    def mark_tyre_navigation_failure(self, tyre_id: str, error_reason: str = "unknown"):
        """
        Mark navigation to tyre as failed.
        
        Args:
            tyre_id: ID of tyre that failed
            error_reason: Reason for failure
        """
        if tyre_id in self.tyre_navigation_status:
            self.tyre_navigation_status[tyre_id]['status'] = 'failed'
            self.tyre_navigation_status[tyre_id]['last_error'] = error_reason
            self.tyre_navigation_status[tyre_id]['last_attempt_time'] = None
        
        self.logger.warn(
            f"Marked tyre {tyre_id} navigation as failed: {error_reason}"
        )
    
    def can_retry_navigation(self, tyre_id: str) -> bool:
        """
        Check if navigation to tyre can be retried.
        
        Args:
            tyre_id: ID of tyre to check
            
        Returns:
            True if retry is allowed, False otherwise
        """
        if tyre_id not in self.tyre_navigation_status:
            return True
        
        status = self.tyre_navigation_status[tyre_id]
        attempts = status.get('attempts', 0)
        
        return attempts < self.max_navigation_attempts_per_tyre
    
    def get_tyre_navigation_status(self, tyre_id: str) -> Dict:
        """
        Get navigation status for a specific tyre.
        
        Args:
            tyre_id: ID of tyre to check
            
        Returns:
            Dict with navigation status information
        """
        return self.tyre_navigation_status.get(tyre_id, {
            'status': 'unknown',
            'attempts': 0,
            'last_error': None,
            'last_attempt_time': None
        })
    
    def clear_context(self):
        """Clear navigation context (call when switching vehicles or completing mission)."""
        self.current_target_tyre_id = None
        self.current_target_tyre_index = None
        self.current_navigation_goal = None
        self.navigation_start_pose = None
        self.navigation_start_time = None
        self.logger.info("Navigation context cleared")
    
    def reset_tyre_status(self, tyre_id: str):
        """
        Reset navigation status for a specific tyre.
        
        Args:
            tyre_id: ID of tyre to reset
        """
        if tyre_id in self.tyre_navigation_status:
            self.tyre_navigation_status[tyre_id] = {
                'status': 'pending',
                'attempts': 0,
                'last_error': None,
                'last_attempt_time': None
            }
    
    def validate_context(self) -> bool:
        """
        Validate that current context is still valid.
        
        Returns:
            True if context is valid, False otherwise
        """
        if self.current_target_tyre_id is None:
            return True  # No context is valid
        
        # Check timeout
        import time
        if self.navigation_start_time:
            elapsed = time.time() - self.navigation_start_time
            if elapsed > self.context_timeout:
                self.logger.warn(
                    f"Navigation context timeout: {elapsed:.1f}s > {self.context_timeout}s"
                )
                return False
        
        return True
    
    def get_context_summary(self) -> Dict:
        """
        Get summary of navigation context.
        
        Returns:
            Dict with context summary
        """
        return {
            'current_target_tyre_id': self.current_target_tyre_id,
            'current_target_tyre_index': self.current_target_tyre_index,
            'has_navigation_goal': self.current_navigation_goal is not None,
            'tyres_tracked': len(self.tyre_navigation_status),
            'tyre_statuses': {
                tyre_id: status['status']
                for tyre_id, status in self.tyre_navigation_status.items()
            }
        }

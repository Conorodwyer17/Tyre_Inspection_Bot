#!/usr/bin/env python3
"""
Tyre Goal Validator Module

Validates tyre navigation goals before attempting navigation.
Ensures goals are safe, reachable, and properly positioned.

This module provides:
- Goal reachability validation
- Safety distance checks
- Alternative goal calculation for unreachable goals
- Collision risk assessment
"""

import math
from typing import Optional, Dict, Tuple
from geometry_msgs.msg import PoseStamped, Point


class TyreGoalValidator:
    """
    Validates tyre navigation goals for safety and reachability.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre goal validator.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.min_safe_distance = parameters.get('min_goal_distance', 0.6)  # meters
        self.max_safe_distance = parameters.get('max_tyre_detection_distance', 10.0)  # meters
        self.approach_distance = parameters.get('approach_distance', 1.5)  # meters
        self.min_tyre_distance = parameters.get('min_tyre_detection_distance', 0.3)  # meters
        self.unreachable_goal_threshold = parameters.get('unreachable_goal_threshold', 0.3)  # meters - if goal is too close to obstacle
        self.max_goal_adjustment_distance = parameters.get('max_goal_adjustment_distance', 1.0)  # meters - max we can adjust goal
        
        # Validation state
        self.unreachable_goals_blacklist = []  # List of goals that were unreachable
        self.blacklist_radius = 0.5  # meters - blacklist radius around failed goals
        
        self.logger.info(
            f"TyreGoalValidator initialized: "
            f"min_safe_distance={self.min_safe_distance}m, "
            f"approach_distance={self.approach_distance}m"
        )
    
    def validate_tyre_goal(
        self,
        tyre_goal: PoseStamped,
        tyre_position: Point,
        robot_pose: Optional[PoseStamped] = None
    ) -> Dict:
        """
        Validate a tyre navigation goal.
        
        Args:
            tyre_goal: Proposed navigation goal pose
            tyre_position: 3D position of the tyre
            robot_pose: Optional current robot pose (for distance checks)
            
        Returns:
            Dict with:
                - 'valid': bool - Is goal valid?
                - 'reachable': bool - Is goal reachable?
                - 'safe': bool - Is goal safe?
                - 'adjusted_goal': PoseStamped or None - Alternative goal if original invalid
                - 'issues': List[str] - List of validation issues found
                - 'recommendation': str - Recommendation (proceed, adjust, skip)
        """
        result = {
            'valid': True,
            'reachable': True,
            'safe': True,
            'adjusted_goal': None,
            'issues': [],
            'recommendation': 'proceed'
        }
        
        if not tyre_goal:
            result['valid'] = False
            result['reachable'] = False
            result['safe'] = False
            result['issues'].append("Goal pose is None")
            result['recommendation'] = 'skip'
            return result
        
        # Check if goal is on blacklist (previously unreachable)
        if self._is_goal_blacklisted(tyre_goal):
            result['valid'] = False
            result['reachable'] = False
            result['issues'].append("Goal is on blacklist (previously unreachable)")
            result['recommendation'] = 'skip'
            return result
        
        # Validate goal structure
        if not hasattr(tyre_goal, 'pose') or not hasattr(tyre_goal.pose, 'position'):
            result['valid'] = False
            result['issues'].append("Invalid goal structure")
            result['recommendation'] = 'skip'
            return result
        
        goal_pos = tyre_goal.pose.position
        
        # Check distance from robot (if robot pose provided)
        if robot_pose:
            robot_pos = robot_pose.pose.position
            distance_to_goal = math.sqrt(
                (goal_pos.x - robot_pos.x)**2 +
                (goal_pos.y - robot_pos.y)**2
            )
            
            if distance_to_goal < self.min_safe_distance:
                result['safe'] = False
                result['issues'].append(
                    f"Goal too close to robot ({distance_to_goal:.2f}m < {self.min_safe_distance}m)"
                )
                result['recommendation'] = 'adjust'
            elif distance_to_goal > self.max_safe_distance:
                result['reachable'] = False
                result['issues'].append(
                    f"Goal too far from robot ({distance_to_goal:.2f}m > {self.max_safe_distance}m)"
                )
                result['recommendation'] = 'adjust'
        
        # Check distance from tyre
        if tyre_position:
            distance_to_tyre = math.sqrt(
                (goal_pos.x - tyre_position.x)**2 +
                (goal_pos.y - tyre_position.y)**2
            )
            
            if distance_to_tyre < self.min_tyre_distance:
                result['safe'] = False
                result['issues'].append(
                    f"Goal too close to tyre ({distance_to_tyre:.2f}m < {self.min_tyre_distance}m) - collision risk"
                )
                # Try to calculate adjusted goal further from tyre
                adjusted = self._calculate_adjusted_goal(tyre_goal, tyre_position, self.min_tyre_distance + 0.2)
                if adjusted:
                    result['adjusted_goal'] = adjusted
                    result['recommendation'] = 'adjust'
                else:
                    result['recommendation'] = 'skip'
            elif distance_to_tyre > self.approach_distance * 1.5:
                result['issues'].append(
                    f"Goal far from tyre ({distance_to_tyre:.2f}m > optimal {self.approach_distance}m)"
                )
                # Not critical, but could optimize
        
        # Overall validation
        if not result['safe']:
            result['valid'] = False
        if not result['reachable']:
            result['valid'] = False
        
        if result['issues']:
            self.logger.warn(
                f"Tyre goal validation found issues: {', '.join(result['issues'])}"
            )
        
        return result
    
    def mark_goal_unreachable(self, goal: PoseStamped):
        """
        Mark a goal as unreachable (for blacklisting).
        
        Args:
            goal: Goal that was unreachable
        """
        if goal and hasattr(goal, 'pose') and hasattr(goal.pose, 'position'):
            goal_pos = goal.pose.position
            self.unreachable_goals_blacklist.append({
                'x': goal_pos.x,
                'y': goal_pos.y,
                'z': goal_pos.z
            })
            self.logger.warn(
                f"Marking goal as unreachable: ({goal_pos.x:.2f}, {goal_pos.y:.2f}, {goal_pos.z:.2f})"
            )
    
    def _is_goal_blacklisted(self, goal: PoseStamped) -> bool:
        """
        Check if goal is on blacklist.
        
        Args:
            goal: Goal to check
            
        Returns:
            True if goal is blacklisted, False otherwise
        """
        if not goal or not hasattr(goal, 'pose') or not hasattr(goal.pose, 'position'):
            return False
        
        goal_pos = goal.pose.position
        
        for blacklisted in self.unreachable_goals_blacklist:
            distance = math.sqrt(
                (goal_pos.x - blacklisted['x'])**2 +
                (goal_pos.y - blacklisted['y'])**2
            )
            if distance < self.blacklist_radius:
                return True
        
        return False
    
    def _calculate_adjusted_goal(
        self,
        original_goal: PoseStamped,
        tyre_position: Point,
        min_distance: float
    ) -> Optional[PoseStamped]:
        """
        Calculate adjusted goal that is further from tyre.
        
        Args:
            original_goal: Original goal pose
            tyre_position: Tyre position
            min_distance: Minimum distance required
            
        Returns:
            Adjusted goal pose, or None if adjustment not possible
        """
        try:
            if not original_goal or not tyre_position:
                return None
            
            goal_pos = original_goal.pose.position
            
            # Calculate vector from tyre to goal
            dx = goal_pos.x - tyre_position.x
            dy = goal_pos.y - tyre_position.y
            current_distance = math.sqrt(dx**2 + dy**2)
            
            if current_distance >= min_distance:
                # Already far enough
                return original_goal
            
            # Calculate direction (normalized)
            if current_distance < 0.01:
                # Goal is at tyre position - move in a default direction (e.g., +x)
                dx = 1.0
                dy = 0.0
                current_distance = 1.0
            else:
                dx = dx / current_distance
                dy = dy / current_distance
            
            # Check if adjustment would be too far
            adjustment_distance = min_distance - current_distance
            if adjustment_distance > self.max_goal_adjustment_distance:
                self.logger.warn(
                    f"Goal adjustment too large ({adjustment_distance:.2f}m > {self.max_goal_adjustment_distance}m)"
                )
                return None
            
            # Calculate new position
            adjusted_goal = PoseStamped()
            adjusted_goal.header = original_goal.header
            adjusted_goal.pose.position.x = tyre_position.x + min_distance * dx
            adjusted_goal.pose.position.y = tyre_position.y + min_distance * dy
            adjusted_goal.pose.position.z = goal_pos.z  # Keep same height
            
            # Keep same orientation (facing tyre)
            adjusted_goal.pose.orientation = original_goal.pose.orientation
            
            self.logger.info(
                f"Adjusted goal: moved from {current_distance:.2f}m to {min_distance:.2f}m from tyre"
            )
            
            return adjusted_goal
            
        except Exception as e:
            self.logger.error(f"Error calculating adjusted goal: {e}", exc_info=True)
            return None
    
    def reset_blacklist(self):
        """Reset the unreachable goals blacklist."""
        self.unreachable_goals_blacklist.clear()
        self.logger.info("Unreachable goals blacklist reset")
    
    def get_validation_summary(self) -> Dict:
        """
        Get summary of validation statistics.
        
        Returns:
            Dict with validation statistics
        """
        return {
            'blacklisted_goals_count': len(self.unreachable_goals_blacklist),
            'blacklist_radius': self.blacklist_radius,
            'min_safe_distance': self.min_safe_distance,
            'approach_distance': self.approach_distance
        }

#!/usr/bin/env python3
"""
Path Alternatives Module

Generates alternative navigation paths when direct path fails.
Tries different approach angles and positions to reach tyres.

This module provides:
- Alternative path generation
- Approach angle variation
- Position offset strategies
- Path validation
"""

import math
from typing import Optional, Dict, List, Tuple
from geometry_msgs.msg import PoseStamped, Point


class PathAlternatives:
    """
    Generates alternative navigation paths.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize path alternatives generator.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.max_alternative_attempts = parameters.get('max_path_alternative_attempts', 3)
        self.approach_angle_step = parameters.get('path_alternative_angle_step', math.pi/4)  # 45 degrees
        self.approach_distance_offset = parameters.get('path_alternative_distance_offset', 0.5)  # meters
        self.min_alternative_distance = parameters.get('min_alternative_goal_distance', 0.3)  # meters
        
        self.logger.info(
            f"PathAlternatives initialized: "
            f"max_attempts={self.max_alternative_attempts}, "
            f"angle_step={self.approach_angle_step:.2f}rad"
        )
    
    def generate_alternative_goals(
        self,
        original_goal: PoseStamped,
        target_position: Point,
        current_robot_pose: Optional[PoseStamped] = None
    ) -> List[PoseStamped]:
        """
        Generate alternative goals when direct path fails.
        
        Args:
            original_goal: Original navigation goal
            target_position: Target tyre position
            current_robot_pose: Current robot pose
            
        Returns:
            List of alternative goal poses
        """
        alternatives = []
        
        try:
            # Calculate approach direction from robot to target
            if current_robot_pose and hasattr(current_robot_pose.pose, 'position'):
                robot_pos = current_robot_pose.pose.position
                approach_dx = target_position.x - robot_pos.x
                approach_dy = target_position.y - robot_pos.y
                base_angle = math.atan2(approach_dy, approach_dx)
            else:
                # Use original goal orientation as base
                base_angle = 2.0 * math.atan2(
                    original_goal.pose.orientation.z,
                    original_goal.pose.orientation.w
                )
            
            # Generate alternative goals at different angles
            for i in range(self.max_alternative_attempts):
                # Calculate offset angle
                angle_offset = (i - self.max_alternative_attempts // 2) * self.approach_angle_step
                approach_angle = base_angle + angle_offset
                
                # Calculate alternative position
                # Offset from target position at different angle
                offset_distance = self.approach_distance_offset * (i + 1)
                alt_x = target_position.x - offset_distance * math.cos(approach_angle)
                alt_y = target_position.y - offset_distance * math.sin(approach_angle)
                
                # Create alternative goal
                alt_goal = PoseStamped()
                alt_goal.header = original_goal.header
                alt_goal.pose.position.x = alt_x
                alt_goal.pose.position.y = alt_y
                alt_goal.pose.position.z = original_goal.pose.position.z
                
                # Orient toward target
                alt_goal.pose.orientation.z = math.sin(approach_angle / 2.0)
                alt_goal.pose.orientation.w = math.cos(approach_angle / 2.0)
                
                # Validate alternative goal (check distance from original)
                distance_from_original = math.sqrt(
                    (alt_x - original_goal.pose.position.x)**2 +
                    (alt_y - original_goal.pose.position.y)**2
                )
                
                if distance_from_original >= self.min_alternative_distance:
                    alternatives.append(alt_goal)
                    self.logger.debug(
                        f"Generated alternative goal {i+1}: "
                        f"({alt_x:.2f}, {alt_y:.2f}), angle_offset={math.degrees(angle_offset):.1f}°"
                    )
            
            return alternatives
            
        except Exception as e:
            self.logger.error(f"Error generating alternative goals: {e}", exc_info=True)
            return alternatives
    
    def get_next_alternative(
        self,
        original_goal: PoseStamped,
        target_position: Point,
        failed_alternatives: List[PoseStamped],
        current_robot_pose: Optional[PoseStamped] = None
    ) -> Optional[PoseStamped]:
        """
        Get next alternative goal after previous ones failed.
        
        Args:
            original_goal: Original navigation goal
            target_position: Target tyre position
            failed_alternatives: List of alternatives that have already failed
            current_robot_pose: Current robot pose
            
        Returns:
            Next alternative goal, or None if no more alternatives
        """
        # Generate all alternatives
        all_alternatives = self.generate_alternative_goals(
            original_goal,
            target_position,
            current_robot_pose
        )
        
        # Find alternatives that haven't been tried
        for alt in all_alternatives:
            # Check if this alternative is significantly different from failed ones
            is_new = True
            for failed in failed_alternatives:
                distance = math.sqrt(
                    (alt.pose.position.x - failed.pose.position.x)**2 +
                    (alt.pose.position.y - failed.pose.position.y)**2
                )
                if distance < self.min_alternative_distance:
                    is_new = False
                    break
            
            if is_new:
                self.logger.info(
                    f"Next alternative goal: ({alt.pose.position.x:.2f}, {alt.pose.position.y:.2f})"
                )
                return alt
        
        self.logger.warn("No more alternative goals available")
        return None
    
    def validate_alternative_goal(
        self,
        alternative_goal: PoseStamped,
        target_position: Point
    ) -> Tuple[bool, Optional[str]]:
        """
        Validate that alternative goal is reasonable.
        
        Args:
            alternative_goal: Alternative goal to validate
            target_position: Target tyre position
            
        Returns:
            Tuple of (is_valid: bool, issue: str or None)
        """
        try:
            # Check distance from target (shouldn't be too far)
            max_reasonable_distance = 5.0  # meters
            distance = math.sqrt(
                (alternative_goal.pose.position.x - target_position.x)**2 +
                (alternative_goal.pose.position.y - target_position.y)**2
            )
            
            if distance > max_reasonable_distance:
                return False, f"Alternative goal too far from target: {distance:.2f}m > {max_reasonable_distance}m"
            
            # Check minimum distance (shouldn't be too close to target)
            if distance < self.min_alternative_distance:
                return False, f"Alternative goal too close to target: {distance:.2f}m < {self.min_alternative_distance}m"
            
            return True, None
            
        except Exception as e:
            self.logger.error(f"Error validating alternative goal: {e}", exc_info=True)
            return False, f"Validation error: {str(e)}"

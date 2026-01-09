#!/usr/bin/env python3
"""
Tyre Capture Repositioner Module

Handles repositioning for tyre photo capture when initial position doesn't yield good photos.
Tries different positions and angles to get optimal photo quality.

This module provides:
- Alternative capture positions
- Position adjustment strategies
- Retry with repositioning
- Optimal position selection
"""

import math
from typing import Optional, Dict, List, Tuple
from geometry_msgs.msg import PoseStamped, Point


class TyreCaptureRepositioner:
    """
    Handles repositioning for tyre photo capture.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre capture repositioner.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.max_reposition_attempts = parameters.get('max_capture_reposition_attempts', 3)
        self.reposition_distance_step = parameters.get('capture_reposition_distance_step', 0.2)  # meters
        self.reposition_angle_step = parameters.get('capture_reposition_angle_step', math.pi/6)  # 30 degrees
        self.optimal_distance_min = parameters.get('optimal_tyre_distance_min', 0.8)  # meters
        self.optimal_distance_max = parameters.get('optimal_tyre_distance_max', 1.5)  # meters
        
        self.logger.info(
            f"TyreCaptureRepositioner initialized: "
            f"max_attempts={self.max_reposition_attempts}, "
            f"distance_step={self.reposition_distance_step}m, "
            f"angle_step={math.degrees(self.reposition_angle_step):.1f}°"
        )
    
    def should_reposition(
        self,
        photo_quality_result: Dict,
        current_position: PoseStamped,
        tyre_position: Point
    ) -> Tuple[bool, Optional[str]]:
        """
        Determine if repositioning is needed based on photo quality.
        
        Args:
            photo_quality_result: Result from photo quality checker
            current_position: Current robot position
            tyre_position: Tyre position
            
        Returns:
            Tuple of (should_reposition: bool, reason: str or None)
        """
        if not photo_quality_result:
            return False, None
        
        # Check if photo quality is acceptable
        quality_acceptable = photo_quality_result.get('quality_acceptable', True)
        
        if quality_acceptable:
            return False, None
        
        # Check quality issues
        issues = photo_quality_result.get('issues', [])
        
        # Check distance
        if current_position and tyre_position:
            distance = math.sqrt(
                (current_position.pose.position.x - tyre_position.x)**2 +
                (current_position.pose.position.y - tyre_position.y)**2
            )
            
            if distance < self.optimal_distance_min:
                return True, f"Too close: {distance:.2f}m < {self.optimal_distance_min}m"
            elif distance > self.optimal_distance_max:
                return True, f"Too far: {distance:.2f}m > {self.optimal_distance_max}m"
        
        # Check for specific quality issues that repositioning might help
        if any('blur' in issue.lower() or 'focus' in issue.lower() for issue in issues):
            return True, "Photo quality issues suggest repositioning needed"
        
        return False, None
    
    def generate_reposition_goals(
        self,
        current_position: PoseStamped,
        tyre_position: Point,
        attempt_number: int
    ) -> List[PoseStamped]:
        """
        Generate alternative positions for photo capture.
        
        Args:
            current_position: Current robot position
            tyre_position: Tyre position
            attempt_number: Current attempt number (1-based)
            
        Returns:
            List of alternative capture positions
        """
        alternatives = []
        
        try:
            if not current_position or not tyre_position:
                return alternatives
            
            # Calculate current distance and angle
            current_pos = current_position.pose.position
            dx = tyre_position.x - current_pos.x
            dy = tyre_position.y - current_pos.y
            current_distance = math.sqrt(dx**2 + dy**2)
            current_angle = math.atan2(dy, dx)
            
            # Generate alternatives
            for i in range(self.max_reposition_attempts):
                # Adjust distance
                if current_distance < self.optimal_distance_min:
                    # Too close - move further
                    target_distance = self.optimal_distance_min + i * self.reposition_distance_step
                elif current_distance > self.optimal_distance_max:
                    # Too far - move closer
                    target_distance = self.optimal_distance_max - i * self.reposition_distance_step
                else:
                    # Within range - try slight adjustments
                    target_distance = current_distance + (i - 1) * self.reposition_distance_step
                
                # Clamp to optimal range
                target_distance = max(
                    self.optimal_distance_min,
                    min(self.optimal_distance_max, target_distance)
                )
                
                # Adjust angle (try different approach angles)
                angle_offset = (i - self.max_reposition_attempts // 2) * self.reposition_angle_step
                target_angle = current_angle + angle_offset
                
                # Calculate new position
                new_x = tyre_position.x - target_distance * math.cos(target_angle)
                new_y = tyre_position.y - target_distance * math.sin(target_angle)
                
                # Create pose
                alt_pose = PoseStamped()
                alt_pose.header = current_position.header
                alt_pose.pose.position.x = new_x
                alt_pose.pose.position.y = new_y
                alt_pose.pose.position.z = current_pos.z
                
                # Orient toward tyre
                alt_pose.pose.orientation.z = math.sin(target_angle / 2.0)
                alt_pose.pose.orientation.w = math.cos(target_angle / 2.0)
                
                alternatives.append(alt_pose)
                
                self.logger.debug(
                    f"Generated reposition goal {i+1}: "
                    f"({new_x:.2f}, {new_y:.2f}), "
                    f"distance={target_distance:.2f}m, "
                    f"angle_offset={math.degrees(angle_offset):.1f}°"
                )
            
            return alternatives
            
        except Exception as e:
            self.logger.error(f"Error generating reposition goals: {e}", exc_info=True)
            return alternatives
    
    def get_next_reposition_goal(
        self,
        current_position: PoseStamped,
        tyre_position: Point,
        failed_positions: List[PoseStamped],
        attempt_number: int
    ) -> Optional[PoseStamped]:
        """
        Get next reposition goal after previous attempts failed.
        
        Args:
            current_position: Current robot position
            tyre_position: Tyre position
            failed_positions: List of positions that have already been tried
            attempt_number: Current attempt number
            
        Returns:
            Next reposition goal, or None if no more alternatives
        """
        if attempt_number > self.max_reposition_attempts:
            self.logger.warn(
                f"Max reposition attempts ({self.max_reposition_attempts}) reached"
            )
            return None
        
        # Generate all alternatives
        all_alternatives = self.generate_reposition_goals(
            current_position,
            tyre_position,
            attempt_number
        )
        
        # Find alternatives that haven't been tried
        for alt in all_alternatives:
            # Check if this alternative is significantly different from failed ones
            is_new = True
            min_distance = 0.1  # meters - minimum difference
            
            for failed in failed_positions:
                distance = math.sqrt(
                    (alt.pose.position.x - failed.pose.position.x)**2 +
                    (alt.pose.position.y - failed.pose.position.y)**2
                )
                if distance < min_distance:
                    is_new = False
                    break
            
            if is_new:
                self.logger.info(
                    f"Next reposition goal (attempt {attempt_number}): "
                    f"({alt.pose.position.x:.2f}, {alt.pose.position.y:.2f})"
                )
                return alt
        
        self.logger.warn("No more reposition goals available")
        return None

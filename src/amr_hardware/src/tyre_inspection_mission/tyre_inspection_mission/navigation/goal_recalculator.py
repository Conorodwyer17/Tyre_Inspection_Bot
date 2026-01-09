#!/usr/bin/env python3
"""
Goal Recalculator Module

Recalculates navigation goals if vehicle moves during navigation.
Handles dynamic goal updates with validation and oscillation prevention.

This module provides:
- Goal recalculation when vehicle moves
- Goal validation and safety checks
- Oscillation prevention
- Update cooldown management
"""

import math
import time
from typing import Optional, Dict
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class GoalRecalculator:
    """
    Recalculates navigation goals when vehicle position changes.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize goal recalculator.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.max_goal_updates = parameters.get('max_goal_updates', 3)
        self.goal_update_cooldown = parameters.get('goal_update_cooldown', 5.0)  # seconds
        self.goal_recalculation_distance = parameters.get('goal_recalculation_distance', 0.6)  # meters
        self.approach_distance = parameters.get('approach_distance', 1.5)  # meters
        self.min_goal_distance = parameters.get('min_goal_distance', 0.6)  # meters
        
        # Tracking state
        self.goal_update_count = 0
        self.last_goal_update_time = None
        self.last_updated_goal = None
        
        self.logger.info(
            f"GoalRecalculator initialized: "
            f"max_updates={self.max_goal_updates}, "
            f"cooldown={self.goal_update_cooldown}s, "
            f"approach_distance={self.approach_distance}m"
        )
    
    def recalculate_approach_pose(
        self,
        new_vehicle_pose: PoseStamped,
        approach_distance: Optional[float] = None
    ) -> Optional[PoseStamped]:
        """
        Recalculate approach pose from new vehicle position.
        
        Args:
            new_vehicle_pose: New vehicle detection pose
            approach_distance: Approach distance (uses default if None)
            
        Returns:
            New approach pose, or None if calculation fails
        """
        try:
            if not new_vehicle_pose:
                self.logger.error("GoalRecalculator: new_vehicle_pose is None")
                return None
            
            if not hasattr(new_vehicle_pose, 'pose') or not hasattr(new_vehicle_pose.pose, 'position'):
                self.logger.error("GoalRecalculator: Invalid vehicle pose structure")
                return None
            
            # Use provided approach distance or default
            dist = approach_distance if approach_distance is not None else self.approach_distance
            
            # Get vehicle position and orientation
            vehicle_pos = new_vehicle_pose.pose.position
            vehicle_ori = new_vehicle_pose.pose.orientation
            
            # Calculate yaw from quaternion
            yaw = 2.0 * math.atan2(vehicle_ori.z, vehicle_ori.w)
            
            # Calculate approach position (in front of vehicle)
            approach_pose = PoseStamped()
            approach_pose.header.frame_id = new_vehicle_pose.header.frame_id
            approach_pose.header.stamp = new_vehicle_pose.header.stamp
            
            # Position: vehicle position + approach_distance in forward direction
            approach_pose.pose.position.x = vehicle_pos.x + dist * math.cos(yaw)
            approach_pose.pose.position.y = vehicle_pos.y + dist * math.sin(yaw)
            approach_pose.pose.position.z = vehicle_pos.z  # Keep same height
            
            # Orientation: Face toward vehicle (180° from vehicle orientation)
            approach_yaw = yaw + math.pi
            approach_pose.pose.orientation.z = math.sin(approach_yaw / 2.0)
            approach_pose.pose.orientation.w = math.cos(approach_yaw / 2.0)
            
            self.logger.info(
                f"GoalRecalculator: Recalculated approach pose: "
                f"({approach_pose.pose.position.x:.2f}, {approach_pose.pose.position.y:.2f}, {approach_pose.pose.position.z:.2f}) "
                f"in frame '{approach_pose.header.frame_id}'"
            )
            
            return approach_pose
            
        except Exception as e:
            self.logger.error(f"GoalRecalculator: Error recalculating pose: {e}", exc_info=True)
            return None
    
    def should_recalculate_goal(
        self,
        movement_distance: float,
        threshold: Optional[float] = None
    ) -> bool:
        """
        Determine if goal should be recalculated based on vehicle movement.
        
        Args:
            movement_distance: Distance vehicle moved (meters)
            threshold: Movement threshold (uses default if None)
            
        Returns:
            True if goal should be recalculated, False otherwise
        """
        # Check if we've exceeded max updates
        if self.goal_update_count >= self.max_goal_updates:
            self.logger.warn(
                f"GoalRecalculator: Max goal updates ({self.max_goal_updates}) reached. "
                "Cannot recalculate goal."
            )
            return False
        
        # Check cooldown
        current_time = time.time()
        if self.last_goal_update_time is not None:
            time_since_update = current_time - self.last_goal_update_time
            if time_since_update < self.goal_update_cooldown:
                self.logger.debug(
                    f"GoalRecalculator: Cooldown active "
                    f"({time_since_update:.1f}s < {self.goal_update_cooldown}s)"
                )
                return False
        
        # Check movement threshold
        movement_threshold = threshold if threshold is not None else self.goal_recalculation_distance
        
        if movement_distance > movement_threshold:
            self.logger.info(
                f"GoalRecalculator: Vehicle movement ({movement_distance:.2f}m) "
                f"exceeds threshold ({movement_threshold:.2f}m). Recalculation recommended."
            )
            return True
        
        return False
    
    def validate_goal_update(
        self,
        old_goal: PoseStamped,
        new_goal: PoseStamped,
        min_distance: Optional[float] = None
    ) -> bool:
        """
        Validate that goal update is reasonable.
        
        Args:
            old_goal: Original goal pose
            new_goal: New goal pose
            min_distance: Minimum distance for update (uses default if None)
            
        Returns:
            True if update is valid, False otherwise
        """
        try:
            if not old_goal or not new_goal:
                return False
            
            # Check frame match
            if old_goal.header.frame_id != new_goal.header.frame_id:
                self.logger.warn(
                    f"GoalRecalculator: Frame mismatch: "
                    f"{old_goal.header.frame_id} vs {new_goal.header.frame_id}"
                )
                return False
            
            # Calculate distance between old and new goals
            old_pos = old_goal.pose.position
            new_pos = new_goal.pose.position
            
            dx = new_pos.x - old_pos.x
            dy = new_pos.y - old_pos.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Check minimum distance
            min_dist = min_distance if min_distance is not None else self.min_goal_distance
            
            if distance < min_dist:
                self.logger.warn(
                    f"GoalRecalculator: Goal update too small "
                    f"({distance:.2f}m < {min_dist:.2f}m). Update rejected."
                )
                return False
            
            # Check if new goal is reasonable (not too far from old goal)
            max_reasonable_distance = 5.0  # meters - vehicle shouldn't move more than this
            if distance > max_reasonable_distance:
                self.logger.warn(
                    f"GoalRecalculator: Goal update too large "
                    f"({distance:.2f}m > {max_reasonable_distance:.2f}m). Update rejected."
                )
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"GoalRecalculator: Error validating goal update: {e}", exc_info=True)
            return False
    
    def record_goal_update(self, new_goal: PoseStamped):
        """
        Record that a goal update was performed.
        
        Args:
            new_goal: New goal that was set
        """
        self.goal_update_count += 1
        self.last_goal_update_time = time.time()
        self.last_updated_goal = new_goal
        
        self.logger.info(
            f"GoalRecalculator: Goal update recorded ({self.goal_update_count}/{self.max_goal_updates})"
        )
    
    def reset(self):
        """Reset goal update tracking."""
        self.goal_update_count = 0
        self.last_goal_update_time = None
        self.last_updated_goal = None
        self.logger.debug("GoalRecalculator: Reset goal update tracking")
    
    def can_update_goal(self) -> bool:
        """
        Check if goal can be updated (not at max, not in cooldown).
        
        Returns:
            True if goal can be updated, False otherwise
        """
        if self.goal_update_count >= self.max_goal_updates:
            return False
        
        if self.last_goal_update_time is not None:
            current_time = time.time()
            time_since_update = current_time - self.last_goal_update_time
            if time_since_update < self.goal_update_cooldown:
                return False
        
        return True
    
    def get_update_status(self) -> Dict:
        """
        Get current goal update status.
        
        Returns:
            Dict with update statistics
        """
        return {
            'update_count': self.goal_update_count,
            'max_updates': self.max_goal_updates,
            'last_update_time': self.last_goal_update_time,
            'can_update': self.can_update_goal()
        }

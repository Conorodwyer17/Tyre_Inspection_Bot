#!/usr/bin/env python3
"""
Tyre Pose Refiner Module

Refines tyre pose estimate after arriving at navigation goal.
Uses visual feedback to improve pose accuracy for better photo positioning.

This module provides:
- Pose refinement based on visual detection
- Iterative pose adjustment
- Convergence detection
- Final pose validation
"""

import math
from typing import Optional, Dict
from geometry_msgs.msg import PoseStamped, Point
from gb_visual_detection_3d_msgs.msg import BoundingBox3d


class TyrePoseRefiner:
    """
    Refines tyre pose estimate using visual feedback.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre pose refiner.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.refinement_iterations_max = parameters.get('tyre_pose_refinement_max_iterations', 3)
        self.convergence_threshold = parameters.get('tyre_pose_convergence_threshold', 0.1)  # meters
        self.max_refinement_distance = parameters.get('max_refinement_distance', 0.5)  # meters
        
        # Refinement state
        self.refinement_active = False
        self.refinement_iteration = 0
        self.last_refined_pose = None
        self.converged = False
        
        self.logger.info(
            f"TyrePoseRefiner initialized: "
            f"max_iterations={self.refinement_iterations_max}, "
            f"convergence_threshold={self.convergence_threshold}m"
        )
    
    def refine_tyre_pose(
        self,
        detected_bbox: BoundingBox3d,
        header,
        original_pose: PoseStamped,
        bbox_to_pose_func,
        current_robot_pose: Optional[PoseStamped] = None
    ) -> Dict:
        """
        Refine tyre pose based on visual detection.
        
        Args:
            detected_bbox: Detected tyre bounding box
            header: ROS header from detection
            original_pose: Original estimated pose
            bbox_to_pose_func: Function to convert bbox to pose
            current_robot_pose: Optional current robot pose
            
        Returns:
            Dict with:
                - 'refined_pose': PoseStamped or None - Refined pose
                - 'converged': bool - Has pose converged?
                - 'iteration': int - Current refinement iteration
                - 'position_change': float - Position change in meters
                - 'should_adjust_navigation': bool - Should adjust navigation goal?
        """
        result = {
            'refined_pose': None,
            'converged': False,
            'iteration': self.refinement_iteration,
            'position_change': 0.0,
            'should_adjust_navigation': False
        }
        
        if not detected_bbox or not original_pose:
            return result
        
        try:
            # Convert detected bbox to pose
            detected_pose = bbox_to_pose_func(detected_bbox, header)
            if not detected_pose:
                return result
            
            # Calculate position change from original
            original_pos = original_pose.pose.position
            detected_pos = detected_pose.pose.position
            
            position_change = math.sqrt(
                (detected_pos.x - original_pos.x)**2 +
                (detected_pos.y - original_pos.y)**2
            )
            result['position_change'] = position_change
            
            # Check convergence
            if position_change < self.convergence_threshold:
                result['converged'] = True
                result['refined_pose'] = detected_pose
                self.logger.info(
                    f"Tyre pose converged: position_change={position_change:.3f}m "
                    f"(threshold: {self.convergence_threshold}m)"
                )
                return result
            
            # Check if refinement is too large (might be wrong detection)
            if position_change > self.max_refinement_distance:
                self.logger.warn(
                    f"Tyre pose refinement too large ({position_change:.2f}m > {self.max_refinement_distance}m). "
                    "Might be wrong detection. Using original pose."
                )
                result['refined_pose'] = original_pose
                return result
            
            # Refined pose is valid
            result['refined_pose'] = detected_pose
            
            # Check if navigation goal should be adjusted
            if position_change > self.convergence_threshold * 2:  # Significant change
                result['should_adjust_navigation'] = True
                self.logger.info(
                    f"Significant pose change detected ({position_change:.2f}m). "
                    "Navigation goal adjustment recommended."
                )
            
            self.refinement_iteration += 1
            
            if self.refinement_iteration >= self.refinement_iterations_max:
                self.logger.info(
                    f"Max refinement iterations ({self.refinement_iterations_max}) reached. "
                    f"Final position_change: {position_change:.3f}m"
                )
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error refining tyre pose: {e}", exc_info=True)
            result['refined_pose'] = original_pose  # Fallback to original
            return result
    
    def reset(self):
        """Reset refinement state."""
        self.refinement_active = False
        self.refinement_iteration = 0
        self.last_refined_pose = None
        self.converged = False
    
    def start_refinement(self):
        """Start refinement process."""
        self.refinement_active = True
        self.refinement_iteration = 0
        self.converged = False
        self.last_refined_pose = None
    
    def is_refinement_complete(self) -> bool:
        """Check if refinement is complete."""
        return self.converged or self.refinement_iteration >= self.refinement_iterations_max

#!/usr/bin/env python3
"""
Visual Verifier Module

Verifies vehicle visibility and position before critical operations.
Ensures vehicle is actually visible and in expected location before capture.

This module provides:
- Vehicle visibility verification
- Position matching verification
- Orientation verification
- Frame-based vehicle detection
"""

import math
import time
from typing import Optional, Dict, Tuple
from geometry_msgs.msg import PoseStamped, Point
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d


class VisualVerifier:
    """
    Verifies vehicle is visible and correctly positioned before operations.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize visual verifier.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.arrival_verification_timeout = parameters.get('arrival_verification_timeout', 10.0)  # seconds
        self.arrival_position_tolerance = parameters.get('arrival_position_tolerance', 1.0)  # meters
        self.arrival_orientation_tolerance = parameters.get('arrival_orientation_tolerance', 0.5)  # radians
        self.truck_class_name = parameters.get('truck_class_name', 'truck')
        self.detection_confidence_threshold = parameters.get('detection_confidence_threshold', 0.5)
        self.verification_frames_required = parameters.get('verification_frames_required', 2)  # frames
        
        # Vehicle classes
        self.vehicle_classes = ['truck', 'car'] if self.truck_class_name.lower() == 'truck' else [self.truck_class_name.lower()]
        
        # Verification state
        self.verification_attempts = 0
        self.verification_start_time = None
        self.verification_frames_seen = 0
        
        self.logger.info(
            f"VisualVerifier initialized: "
            f"position_tolerance={self.arrival_position_tolerance}m, "
            f"orientation_tolerance={self.arrival_orientation_tolerance}rad, "
            f"timeout={self.arrival_verification_timeout}s"
        )
    
    def verify_vehicle_at_arrival(
        self,
        bbox_msg: BoundingBoxes3d,
        expected_pose: PoseStamped,
        truck_id: str,
        bbox_to_pose_func,
        current_robot_pose: Optional[PoseStamped] = None
    ) -> Dict:
        """
        Verify vehicle is visible and at expected location on arrival.
        
        Args:
            bbox_msg: Current bounding boxes message
            expected_pose: Expected vehicle pose (from original detection)
            truck_id: ID of vehicle to verify
            bbox_to_pose_func: Function to convert bbox to pose
            current_robot_pose: Optional current robot pose for context
            
        Returns:
            Dict with:
                - 'vehicle_visible': bool - Is vehicle visible?
                - 'position_match': bool - Does position match expected?
                - 'orientation_match': bool - Is orientation correct?
                - 'verification_passed': bool - Overall verification result
                - 'detected_pose': PoseStamped or None - Detected vehicle pose
                - 'position_error': float - Position error in meters
                - 'orientation_error': float - Orientation error in radians
        """
        result = {
            'vehicle_visible': False,
            'position_match': False,
            'orientation_match': False,
            'verification_passed': False,
            'detected_pose': None,
            'position_error': float('inf'),
            'orientation_error': float('inf')
        }
        
        # Initialize verification tracking
        if self.verification_start_time is None:
            self.verification_start_time = time.time()
            self.verification_attempts = 0
            self.verification_frames_seen = 0
        
        self.verification_attempts += 1
        current_time = time.time()
        
        # Check timeout
        elapsed_time = current_time - self.verification_start_time
        if elapsed_time > self.arrival_verification_timeout:
            self.logger.warn(
                f"VisualVerifier: Verification timeout after {elapsed_time:.1f}s "
                f"(timeout: {self.arrival_verification_timeout}s)"
            )
            return result
        
        # Check if vehicle is visible
        vehicle_bbox = self._find_vehicle_in_detections(bbox_msg, truck_id)
        
        if vehicle_bbox is None:
            result['vehicle_visible'] = False
            self.logger.debug(
                f"VisualVerifier: Vehicle not visible (attempt {self.verification_attempts})"
            )
            return result
        
        # Vehicle is visible
        result['vehicle_visible'] = True
        self.verification_frames_seen += 1
        
        # Calculate detected vehicle pose
        try:
            detected_pose = bbox_to_pose_func(vehicle_bbox, bbox_msg.header)
            if detected_pose is None:
                self.logger.warn("VisualVerifier: Failed to calculate detected vehicle pose")
                return result
            
            result['detected_pose'] = detected_pose
            
            # Verify position match
            position_match, position_error = self._verify_position_match(
                detected_pose,
                expected_pose,
                self.arrival_position_tolerance
            )
            result['position_match'] = position_match
            result['position_error'] = position_error
            
            # Verify orientation match (if robot pose provided)
            if current_robot_pose:
                orientation_match, orientation_error = self._verify_orientation_correct(
                    current_robot_pose,
                    detected_pose
                )
                result['orientation_match'] = orientation_match
                result['orientation_error'] = orientation_error
            else:
                # Can't verify orientation without robot pose
                result['orientation_match'] = True  # Assume OK
                result['orientation_error'] = 0.0
            
            # Overall verification passed if vehicle visible and position matches
            # Require multiple frames for stability
            if (position_match and 
                self.verification_frames_seen >= self.verification_frames_required):
                result['verification_passed'] = True
                self.logger.info(
                    f"VisualVerifier: ✅ Vehicle verification passed "
                    f"(position_error: {position_error:.2f}m, "
                    f"frames_seen: {self.verification_frames_seen})"
                )
            else:
                self.logger.debug(
                    f"VisualVerifier: Verification in progress "
                    f"(position_error: {position_error:.2f}m, "
                    f"frames_seen: {self.verification_frames_seen}/{self.verification_frames_required})"
                )
            
        except Exception as e:
            self.logger.error(f"VisualVerifier: Error during verification: {e}", exc_info=True)
        
        return result
    
    def check_vehicle_in_frame(
        self,
        bbox_msg: BoundingBoxes3d,
        truck_id: str
    ) -> bool:
        """
        Quick check if vehicle is visible in current frame.
        
        Args:
            bbox_msg: Current bounding boxes
            truck_id: ID of vehicle to check
            
        Returns:
            True if vehicle visible, False otherwise
        """
        vehicle_bbox = self._find_vehicle_in_detections(bbox_msg, truck_id)
        return vehicle_bbox is not None
    
    def _find_vehicle_in_detections(
        self,
        bbox_msg: BoundingBoxes3d,
        truck_id: str
    ) -> Optional[BoundingBox3d]:
        """
        Find vehicle in bounding boxes.
        
        Args:
            bbox_msg: Bounding boxes message
            truck_id: ID of vehicle to find
            
        Returns:
            BoundingBox3d if found, None otherwise
        """
        if not bbox_msg or not bbox_msg.bounding_boxes:
            return None
        
        for bbox in bbox_msg.bounding_boxes:
            # Check if this is a vehicle class
            if bbox.object_name.lower() not in [vc.lower() for vc in self.vehicle_classes]:
                continue
            
            # Check confidence
            if bbox.probability < self.detection_confidence_threshold:
                continue
            
            # Return first matching vehicle
            # TODO: Could improve by matching position to expected pose
            return bbox
        
        return None
    
    def _verify_position_match(
        self,
        detected_pose: PoseStamped,
        expected_pose: PoseStamped,
        tolerance: float
    ) -> Tuple[bool, float]:
        """
        Verify detected position matches expected position within tolerance.
        
        Args:
            detected_pose: Detected vehicle pose
            expected_pose: Expected vehicle pose
            tolerance: Position tolerance in meters
            
        Returns:
            Tuple of (match: bool, error: float)
        """
        try:
            if not detected_pose or not expected_pose:
                return False, float('inf')
            
            if not hasattr(detected_pose, 'pose') or not hasattr(expected_pose, 'pose'):
                return False, float('inf')
            
            detected_pos = detected_pose.pose.position
            expected_pos = expected_pose.pose.position
            
            # Calculate 2D distance
            dx = detected_pos.x - expected_pos.x
            dy = detected_pos.y - expected_pos.y
            distance = math.sqrt(dx**2 + dy**2)
            
            match = distance <= tolerance
            
            return match, distance
            
        except Exception as e:
            self.logger.error(f"VisualVerifier: Error verifying position: {e}", exc_info=True)
            return False, float('inf')
    
    def _verify_orientation_correct(
        self,
        robot_pose: PoseStamped,
        vehicle_pose: PoseStamped
    ) -> Tuple[bool, float]:
        """
        Verify robot orientation is correct for viewing vehicle.
        
        Args:
            robot_pose: Current robot pose
            vehicle_pose: Vehicle pose
            
        Returns:
            Tuple of (match: bool, error: float in radians)
        """
        try:
            if not robot_pose or not vehicle_pose:
                return False, float('inf')
            
            # Calculate desired orientation (face toward vehicle)
            robot_pos = robot_pose.pose.position
            vehicle_pos = vehicle_pose.pose.position
            
            dx = vehicle_pos.x - robot_pos.x
            dy = vehicle_pos.y - robot_pos.y
            desired_yaw = math.atan2(dy, dx)
            
            # Get current robot orientation
            robot_ori = robot_pose.pose.orientation
            current_yaw = 2.0 * math.atan2(robot_ori.z, robot_ori.w)
            
            # Calculate orientation error
            yaw_error = abs(desired_yaw - current_yaw)
            # Normalize to [0, π]
            if yaw_error > math.pi:
                yaw_error = 2 * math.pi - yaw_error
            
            match = yaw_error <= self.arrival_orientation_tolerance
            
            return match, yaw_error
            
        except Exception as e:
            self.logger.error(f"VisualVerifier: Error verifying orientation: {e}", exc_info=True)
            return False, float('inf')
    
    def reset(self):
        """Reset verification state."""
        self.verification_attempts = 0
        self.verification_start_time = None
        self.verification_frames_seen = 0

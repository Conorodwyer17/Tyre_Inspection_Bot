#!/usr/bin/env python3
"""
Tyre Capture Verifier Module

Verifies tyre visibility and optimal camera position before photo capture.
Ensures tyre is visible, properly framed, and at optimal distance/angle.

This module provides:
- Tyre visibility verification
- Camera angle verification
- Distance optimization
- Frame positioning checks
"""

import math
import time
from typing import Optional, Dict
from geometry_msgs.msg import PoseStamped, Point
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d


class TyreCaptureVerifier:
    """
    Verifies tyre is visible and optimally positioned before photo capture.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre capture verifier.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.tyre_detection_confidence_threshold = parameters.get('tyre_detection_confidence_threshold', 0.5)
        self.optimal_distance_min = parameters.get('optimal_tyre_distance_min', 0.8)  # meters
        self.optimal_distance_max = parameters.get('optimal_tyre_distance_max', 1.5)  # meters
        self.optimal_angle_tolerance = parameters.get('optimal_angle_tolerance', 0.4)  # radians (~23 degrees)
        self.verification_timeout = parameters.get('tyre_capture_verification_timeout', 10.0)  # seconds
        self.verification_frames_required = parameters.get('tyre_verification_frames_required', 2)  # frames
        self.position_tolerance = parameters.get('tyre_position_tolerance', 0.3)  # meters
        
        # Verification state
        self.verification_start_time = None
        self.verification_frames_seen = 0
        self.last_tyre_bbox = None
        
        self.logger.info(
            f"TyreCaptureVerifier initialized: "
            f"optimal_distance=[{self.optimal_distance_min:.1f}, {self.optimal_distance_max:.1f}]m, "
            f"angle_tolerance={self.optimal_angle_tolerance:.2f}rad"
        )
    
    def verify_tyre_before_capture(
        self,
        bbox_msg: BoundingBoxes3d,
        expected_tyre_id: str,
        expected_tyre_position: Point,
        robot_pose: Optional[PoseStamped] = None,
        bbox_to_pose_func = None
    ) -> Dict:
        """
        Verify tyre is visible and optimally positioned before capture.
        
        Args:
            bbox_msg: Current bounding boxes message
            expected_tyre_id: ID of tyre to verify
            expected_tyre_position: Expected 3D position of tyre
            robot_pose: Optional current robot pose
            bbox_to_pose_func: Function to convert bbox to pose
            
        Returns:
            Dict with:
                - 'tyre_visible': bool - Is tyre visible?
                - 'position_ok': bool - Is position optimal?
                - 'angle_ok': bool - Is camera angle optimal?
                - 'distance_ok': bool - Is distance optimal?
                - 'verification_passed': bool - Overall verification result
                - 'detected_tyre_bbox': BoundingBox3d or None - Detected tyre
                - 'position_error': float - Position error in meters
                - 'distance_error': float - Distance error in meters
                - 'angle_error': float - Angle error in radians
                - 'recommendation': str - What to do (capture, adjust_position, adjust_angle, retry)
        """
        result = {
            'tyre_visible': False,
            'position_ok': False,
            'angle_ok': False,
            'distance_ok': False,
            'verification_passed': False,
            'detected_tyre_bbox': None,
            'position_error': float('inf'),
            'distance_error': float('inf'),
            'angle_error': float('inf'),
            'recommendation': 'retry'
        }
        
        # Initialize verification tracking
        if self.verification_start_time is None:
            self.verification_start_time = time.time()
            self.verification_frames_seen = 0
        
        current_time = time.time()
        elapsed_time = current_time - self.verification_start_time
        
        # Check timeout
        if elapsed_time > self.verification_timeout:
            self.logger.warn(
                f"Tyre verification timeout after {elapsed_time:.1f}s "
                f"(timeout: {self.verification_timeout}s)"
            )
            result['recommendation'] = 'capture_anyway'  # Proceed despite timeout
            return result
        
        # Find tyre in current detections
        tyre_bbox = self._find_tyre_in_detections(bbox_msg, expected_tyre_id)
        
        if tyre_bbox is None:
            result['tyre_visible'] = False
            result['recommendation'] = 'retry'
            self.logger.debug(
                f"Tyre {expected_tyre_id} not visible in current frame "
                f"(verification attempt: {elapsed_time:.1f}s)"
            )
            return result
        
        # Tyre is visible
        result['tyre_visible'] = True
        result['detected_tyre_bbox'] = tyre_bbox
        self.verification_frames_seen += 1
        
        # Calculate detected tyre position (if bbox_to_pose_func provided)
        if bbox_to_pose_func and expected_tyre_position:
            try:
                detected_pose = bbox_to_pose_func(tyre_bbox, bbox_msg.header)
                if detected_pose:
                    # Check position match
                    detected_pos = detected_pose.pose.position
                    position_error = math.sqrt(
                        (detected_pos.x - expected_tyre_position.x)**2 +
                        (detected_pos.y - expected_tyre_position.y)**2
                    )
                    result['position_error'] = position_error
                    result['position_ok'] = position_error <= self.position_tolerance
                else:
                    # Can't verify position, assume OK
                    result['position_ok'] = True
            except Exception as e:
                self.logger.warn(f"Error calculating tyre position: {e}")
                result['position_ok'] = True  # Assume OK if calculation fails
        
        # Check distance (if robot pose provided)
        if robot_pose and expected_tyre_position:
            robot_pos = robot_pose.pose.position
            distance = math.sqrt(
                (robot_pos.x - expected_tyre_position.x)**2 +
                (robot_pos.y - expected_tyre_position.y)**2
            )
            result['distance_error'] = abs(distance - (self.optimal_distance_min + self.optimal_distance_max) / 2)
            
            if self.optimal_distance_min <= distance <= self.optimal_distance_max:
                result['distance_ok'] = True
            elif distance < self.optimal_distance_min:
                result['recommendation'] = 'adjust_position_back'
            elif distance > self.optimal_distance_max:
                result['recommendation'] = 'adjust_position_forward'
        
        # Check angle (if robot pose provided)
        if robot_pose and expected_tyre_position:
            robot_pos = robot_pose.pose.position
            robot_ori = robot_pose.pose.orientation
            robot_yaw = 2.0 * math.atan2(robot_ori.z, robot_ori.w)
            
            # Calculate desired orientation (face toward tyre)
            dx = expected_tyre_position.x - robot_pos.x
            dy = expected_tyre_position.y - robot_pos.y
            desired_yaw = math.atan2(dy, dx)
            
            # Calculate angle error
            angle_error = abs(desired_yaw - robot_yaw)
            if angle_error > math.pi:
                angle_error = 2 * math.pi - angle_error
            
            result['angle_error'] = angle_error
            result['angle_ok'] = angle_error <= self.optimal_angle_tolerance
            
            if not result['angle_ok']:
                result['recommendation'] = 'adjust_angle'
        
        # Overall verification (require multiple frames for stability)
        if (result['tyre_visible'] and 
            result['position_ok'] and 
            result['distance_ok'] and 
            result['angle_ok'] and
            self.verification_frames_seen >= self.verification_frames_required):
            result['verification_passed'] = True
            result['recommendation'] = 'capture'
            self.logger.info(
                f"✅ Tyre verification passed: "
                f"position_error={result['position_error']:.2f}m, "
                f"distance_error={result['distance_error']:.2f}m, "
                f"angle_error={result['angle_error']:.2f}rad, "
                f"frames_seen={self.verification_frames_seen}"
            )
        else:
            # Still verifying
            self.logger.debug(
                f"Tyre verification in progress: "
                f"visible={result['tyre_visible']}, "
                f"position={result['position_ok']}, "
                f"distance={result['distance_ok']}, "
                f"angle={result['angle_ok']}, "
                f"frames={self.verification_frames_seen}/{self.verification_frames_required}"
            )
        
        return result
    
    def _find_tyre_in_detections(
        self,
        bbox_msg: BoundingBoxes3d,
        tyre_id: str
    ) -> Optional[BoundingBox3d]:
        """
        Find tyre in bounding boxes.
        
        Args:
            bbox_msg: Bounding boxes message
            tyre_id: ID of tyre to find
            
        Returns:
            BoundingBox3d if found, None otherwise
        """
        if not bbox_msg or not bbox_msg.bounding_boxes:
            return None
        
        for bbox in bbox_msg.bounding_boxes:
            # Check if this is a tyre
            if bbox.object_name.lower() not in ['tyre', 'tire']:
                continue
            
            # Check confidence
            if bbox.probability < self.tyre_detection_confidence_threshold:
                continue
            
            # For now, return first matching tyre
            # TODO: Could improve by matching position to expected tyre position
            return bbox
        
        return None
    
    def check_tyre_visible(
        self,
        bbox_msg: BoundingBoxes3d,
        tyre_id: str
    ) -> bool:
        """
        Quick check if tyre is visible in current frame.
        
        Args:
            bbox_msg: Current bounding boxes
            tyre_id: ID of tyre to check
            
        Returns:
            True if tyre visible, False otherwise
        """
        tyre_bbox = self._find_tyre_in_detections(bbox_msg, tyre_id)
        return tyre_bbox is not None
    
    def reset(self):
        """Reset verification state."""
        self.verification_start_time = None
        self.verification_frames_seen = 0
        self.last_tyre_bbox = None

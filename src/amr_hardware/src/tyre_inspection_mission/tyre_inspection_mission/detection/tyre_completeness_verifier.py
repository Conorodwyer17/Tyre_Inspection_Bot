#!/usr/bin/env python3
"""
Tyre Completeness Verifier Module

Verifies that all expected tyres have been detected.
Implements strategies to ensure completeness including expected count validation
and multi-angle detection.

This module provides:
- Expected tyre count validation
- Completeness verification
- Detection gap identification
- Multi-angle detection recommendations
"""

import math
from typing import Optional, Dict, List, Tuple
from geometry_msgs.msg import Point, PoseStamped


class TyreCompletenessVerifier:
    """
    Verifies tyre detection completeness.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre completeness verifier.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.min_expected_tyres = parameters.get('min_expected_tyres', 4)  # Typical vehicle has 4 tyres
        self.max_expected_tyres = parameters.get('max_expected_tyres', 6)  # Trucks might have 6
        self.vehicle_length = parameters.get('vehicle_length', 8.0)  # meters
        self.vehicle_width = parameters.get('vehicle_width', 2.5)  # meters
        self.enable_multi_angle_detection = parameters.get('enable_multi_angle_detection', True)
        
        self.logger.info(
            f"TyreCompletenessVerifier initialized: "
            f"expected_range=[{self.min_expected_tyres}, {self.max_expected_tyres}], "
            f"multi_angle={self.enable_multi_angle_detection}"
        )
    
    def verify_completeness(
        self,
        detected_tyres: List,
        vehicle_pose: Optional[PoseStamped] = None
    ) -> Dict:
        """
        Verify that all expected tyres have been detected.
        
        Args:
            detected_tyres: List of TyreData objects
            vehicle_pose: Optional vehicle pose for spatial analysis
            
        Returns:
            Dict with:
                - 'complete': bool - Are all tyres detected?
                - 'detected_count': int - Number of tyres detected
                - 'expected_count': int - Expected number of tyres
                - 'completeness_percentage': float - Percentage of expected tyres found
                - 'missing_count': int - Number of tyres missing
                - 'recommendation': str - Recommendation (proceed, continue_detection, review)
                - 'detection_gaps': List[Dict] - Areas where tyres might be missing
        """
        result = {
            'complete': False,
            'detected_count': len(detected_tyres),
            'expected_count': self.min_expected_tyres,
            'completeness_percentage': 0.0,
            'missing_count': 0,
            'recommendation': 'continue_detection',
            'detection_gaps': []
        }
        
        try:
            detected_count = len(detected_tyres)
            
            # Determine expected count based on vehicle type (simplified)
            # Could be enhanced with vehicle classification
            if vehicle_pose:
                # Use vehicle dimensions to estimate expected count
                # Larger vehicles (trucks) typically have more tyres
                if self.vehicle_length > 10.0:
                    result['expected_count'] = self.max_expected_tyres
                else:
                    result['expected_count'] = self.min_expected_tyres
            else:
                result['expected_count'] = self.min_expected_tyres
            
            # Calculate completeness
            result['missing_count'] = max(0, result['expected_count'] - detected_count)
            if result['expected_count'] > 0:
                result['completeness_percentage'] = (detected_count / result['expected_count']) * 100.0
            
            # Identify detection gaps (areas where tyres might be missing)
            if vehicle_pose and detected_tyres:
                result['detection_gaps'] = self._identify_detection_gaps(
                    detected_tyres,
                    vehicle_pose
                )
            
            # Determine recommendation
            if detected_count >= result['expected_count']:
                result['complete'] = True
                result['recommendation'] = 'proceed'
            elif detected_count >= result['expected_count'] * 0.75:  # 75% threshold
                result['complete'] = False
                result['recommendation'] = 'proceed_with_warning'
            elif detected_count >= result['expected_count'] * 0.5:  # 50% threshold
                result['complete'] = False
                if self.enable_multi_angle_detection:
                    result['recommendation'] = 'try_multi_angle_detection'
                else:
                    result['recommendation'] = 'continue_detection'
            else:
                result['complete'] = False
                result['recommendation'] = 'continue_detection'
            
            self.logger.info(
                f"Tyre completeness check: {detected_count}/{result['expected_count']} detected "
                f"({result['completeness_percentage']:.1f}%), "
                f"recommendation: {result['recommendation']}"
            )
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error verifying completeness: {e}", exc_info=True)
            result['recommendation'] = 'review'
            return result
    
    def _identify_detection_gaps(
        self,
        detected_tyres: List,
        vehicle_pose: PoseStamped
    ) -> List[Dict]:
        """
        Identify areas where tyres might be missing.
        
        Args:
            detected_tyres: List of detected tyres
            vehicle_pose: Vehicle pose
            
        Returns:
            List of gap descriptions
        """
        gaps = []
        
        try:
            if not detected_tyres or not vehicle_pose:
                return gaps
            
            vehicle_pos = vehicle_pose.pose.position
            vehicle_ori = vehicle_pose.pose.orientation
            
            # Calculate vehicle orientation
            yaw = 2.0 * math.atan2(vehicle_ori.z, vehicle_ori.w)
            
            # Expected tyre positions (simplified: front-left, front-right, rear-left, rear-right)
            expected_positions = [
                {'side': 'front-left', 'relative_pos': (self.vehicle_length/3, -self.vehicle_width/2)},
                {'side': 'front-right', 'relative_pos': (self.vehicle_length/3, self.vehicle_width/2)},
                {'side': 'rear-left', 'relative_pos': (-self.vehicle_length/3, -self.vehicle_width/2)},
                {'side': 'rear-right', 'relative_pos': (-self.vehicle_length/3, self.vehicle_width/2)},
            ]
            
            # Check which expected positions don't have detected tyres nearby
            for expected in expected_positions:
                rel_x, rel_y = expected['relative_pos']
                
                # Transform to world coordinates
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                
                world_x = vehicle_pos.x + rel_x * cos_yaw - rel_y * sin_yaw
                world_y = vehicle_pos.y + rel_x * sin_yaw + rel_y * cos_yaw
                
                # Check if there's a tyre near this expected position
                found = False
                min_distance = float('inf')
                
                for tyre in detected_tyres:
                    if not hasattr(tyre, 'position_3d') or not tyre.position_3d:
                        continue
                    
                    distance = math.sqrt(
                        (world_x - tyre.position_3d.x)**2 +
                        (world_y - tyre.position_3d.y)**2
                    )
                    
                    if distance < 1.0:  # Within 1m of expected position
                        found = True
                        break
                    elif distance < min_distance:
                        min_distance = distance
                
                if not found:
                    gaps.append({
                        'side': expected['side'],
                        'expected_position': {'x': world_x, 'y': world_y},
                        'closest_tyre_distance': min_distance if min_distance != float('inf') else None
                    })
            
            return gaps
            
        except Exception as e:
            self.logger.error(f"Error identifying detection gaps: {e}", exc_info=True)
            return gaps
    
    def should_continue_detection(
        self,
        detected_count: int,
        detection_time_elapsed: float,
        detection_timeout: float
    ) -> Tuple[bool, str]:
        """
        Determine if detection should continue.
        
        Args:
            detected_count: Number of tyres detected so far
            detection_time_elapsed: Time elapsed since detection started
            detection_timeout: Maximum detection time
            
        Returns:
            Tuple of (should_continue: bool, reason: str)
        """
        # Check if we have minimum expected tyres
        if detected_count >= self.min_expected_tyres:
            return False, f"Minimum expected tyres ({self.min_expected_tyres}) detected"
        
        # Check if timeout approaching
        time_remaining = detection_timeout - detection_time_elapsed
        if time_remaining < 5.0:  # Less than 5 seconds remaining
            if detected_count >= self.min_expected_tyres * 0.5:  # At least 50%
                return False, f"Timeout approaching, {detected_count} tyres detected (minimum acceptable)"
            else:
                return False, f"Timeout approaching, insufficient tyres detected ({detected_count})"
        
        return True, f"Continuing detection: {detected_count}/{self.min_expected_tyres} tyres found"
    
    def get_multi_angle_detection_positions(
        self,
        vehicle_pose: PoseStamped,
        current_robot_pose: Optional[PoseStamped] = None
    ) -> List[Dict]:
        """
        Get recommended positions for multi-angle detection.
        
        Args:
            vehicle_pose: Vehicle pose
            current_robot_pose: Current robot pose
            
        Returns:
            List of recommended detection positions
        """
        positions = []
        
        try:
            if not vehicle_pose:
                return positions
            
            vehicle_pos = vehicle_pose.pose.position
            vehicle_ori = vehicle_pose.pose.orientation
            
            # Calculate vehicle orientation
            yaw = 2.0 * math.atan2(vehicle_ori.z, vehicle_ori.w)
            
            # Recommended detection positions around vehicle
            # Positions at corners of vehicle for better visibility
            detection_distance = 3.0  # meters from vehicle
            
            angles = [0, math.pi/2, math.pi, 3*math.pi/2]  # Four sides
            
            for angle in angles:
                # Position relative to vehicle center
                rel_x = detection_distance * math.cos(angle)
                rel_y = detection_distance * math.sin(angle)
                
                # Transform to world coordinates
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                
                world_x = vehicle_pos.x + rel_x * cos_yaw - rel_y * sin_yaw
                world_y = vehicle_pos.y + rel_x * sin_yaw + rel_y * cos_yaw
                
                positions.append({
                    'position': {'x': world_x, 'y': world_y, 'z': vehicle_pos.z},
                    'angle': angle,
                    'description': self._angle_to_description(angle)
                })
            
            return positions
            
        except Exception as e:
            self.logger.error(f"Error getting multi-angle positions: {e}", exc_info=True)
            return positions
    
    def _angle_to_description(self, angle: float) -> str:
        """Convert angle to human-readable description."""
        if abs(angle) < 0.1 or abs(angle - 2*math.pi) < 0.1:
            return "front"
        elif abs(angle - math.pi/2) < 0.1:
            return "right"
        elif abs(angle - math.pi) < 0.1:
            return "rear"
        elif abs(angle - 3*math.pi/2) < 0.1:
            return "left"
        else:
            return f"angle_{angle:.1f}"

#!/usr/bin/env python3
"""
Tyre Position Validator Module

Validates tyre positions make sense relative to vehicle.
Checks height, distance from vehicle, and spatial relationships.

This module provides:
- Position validation relative to vehicle
- Height validation (tyres should be near ground)
- Distance validation (tyres should be near vehicle)
- Spatial relationship validation
"""

import math
from typing import Optional, Dict, Tuple, List
from geometry_msgs.msg import Point, PoseStamped


class TyrePositionValidator:
    """
    Validates tyre positions make sense relative to vehicle.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre position validator.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.max_tyre_height = parameters.get('max_tyre_height', 1.5)  # meters - tyres shouldn't be too high
        self.min_tyre_height = parameters.get('min_tyre_height', -0.5)  # meters - tyres shouldn't be below ground
        self.max_distance_from_vehicle = parameters.get('max_tyre_distance_from_vehicle', 5.0)  # meters
        self.min_distance_from_vehicle = parameters.get('min_tyre_distance_from_vehicle', 0.3)  # meters
        self.vehicle_length = parameters.get('vehicle_length', 8.0)  # meters
        self.vehicle_width = parameters.get('vehicle_width', 2.5)  # meters
        
        self.logger.info(
            f"TyrePositionValidator initialized: "
            f"height_range=[{self.min_tyre_height}, {self.max_tyre_height}]m, "
            f"distance_range=[{self.min_distance_from_vehicle}, {self.max_distance_from_vehicle}]m"
        )
    
    def validate_tyre_position(
        self,
        tyre_position: Point,
        vehicle_pose: Optional[PoseStamped] = None
    ) -> Dict:
        """
        Validate tyre position makes sense.
        
        Args:
            tyre_position: Tyre position to validate
            vehicle_pose: Optional vehicle pose for relative validation
            
        Returns:
            Dict with:
                - 'valid': bool - Is position valid?
                - 'height_ok': bool - Is height reasonable?
                - 'distance_ok': bool - Is distance from vehicle reasonable?
                - 'spatial_ok': bool - Does spatial relationship make sense?
                - 'issues': List[str] - List of validation issues
                - 'recommendation': str - Recommendation (accept, reject, review)
        """
        result = {
            'valid': True,
            'height_ok': True,
            'distance_ok': True,
            'spatial_ok': True,
            'issues': [],
            'recommendation': 'accept'
        }
        
        try:
            # Check height
            if tyre_position.z < self.min_tyre_height:
                result['height_ok'] = False
                result['issues'].append(
                    f"Tyre too low: z={tyre_position.z:.2f}m < {self.min_tyre_height}m"
                )
                result['valid'] = False
                result['recommendation'] = 'reject'
            elif tyre_position.z > self.max_tyre_height:
                result['height_ok'] = False
                result['issues'].append(
                    f"Tyre too high: z={tyre_position.z:.2f}m > {self.max_tyre_height}m"
                )
                result['valid'] = False
                result['recommendation'] = 'reject'
            
            # Check distance from vehicle (if vehicle pose provided)
            if vehicle_pose and hasattr(vehicle_pose, 'pose') and hasattr(vehicle_pose.pose, 'position'):
                vehicle_pos = vehicle_pose.pose.position
                
                # Calculate distance from vehicle center
                distance = math.sqrt(
                    (tyre_position.x - vehicle_pos.x)**2 +
                    (tyre_position.y - vehicle_pos.y)**2
                )
                
                # Check if within reasonable range
                if distance < self.min_distance_from_vehicle:
                    result['distance_ok'] = False
                    result['issues'].append(
                        f"Tyre too close to vehicle: {distance:.2f}m < {self.min_distance_from_vehicle}m"
                    )
                    result['valid'] = False
                    result['recommendation'] = 'reject'
                elif distance > self.max_distance_from_vehicle:
                    result['distance_ok'] = False
                    result['issues'].append(
                        f"Tyre too far from vehicle: {distance:.2f}m > {self.max_distance_from_vehicle}m"
                    )
                    result['valid'] = False
                    result['recommendation'] = 'review'  # Might be valid if vehicle is long
                else:
                    # Check if position is within vehicle bounds (shouldn't be inside vehicle)
                    if self._is_position_inside_vehicle_bounds(tyre_position, vehicle_pose):
                        result['spatial_ok'] = False
                        result['issues'].append(
                            "Tyre position is inside vehicle bounds (unlikely)"
                        )
                        result['valid'] = False
                        result['recommendation'] = 'reject'
            
            if result['issues']:
                self.logger.warn(
                    f"Tyre position validation issues: {', '.join(result['issues'])}"
                )
            else:
                self.logger.debug(
                    f"Tyre position validated: ({tyre_position.x:.2f}, {tyre_position.y:.2f}, {tyre_position.z:.2f})"
                )
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error validating tyre position: {e}", exc_info=True)
            result['valid'] = False
            result['recommendation'] = 'review'
            return result
    
    def _is_position_inside_vehicle_bounds(
        self,
        position: Point,
        vehicle_pose: PoseStamped
    ) -> bool:
        """
        Check if position is inside vehicle bounding box.
        
        Args:
            position: Position to check
            vehicle_pose: Vehicle pose
            
        Returns:
            True if inside vehicle bounds, False otherwise
        """
        try:
            vehicle_pos = vehicle_pose.pose.position
            vehicle_ori = vehicle_pose.pose.orientation
            
            # Calculate vehicle orientation
            yaw = 2.0 * math.atan2(vehicle_ori.z, vehicle_ori.w)
            
            # Transform position to vehicle frame
            dx = position.x - vehicle_pos.x
            dy = position.y - vehicle_pos.y
            
            # Rotate to vehicle frame
            cos_yaw = math.cos(-yaw)
            sin_yaw = math.sin(-yaw)
            
            local_x = dx * cos_yaw - dy * sin_yaw
            local_y = dx * sin_yaw + dy * cos_yaw
            
            # Check if within vehicle bounds
            half_length = self.vehicle_length / 2.0
            half_width = self.vehicle_width / 2.0
            
            return (
                -half_length <= local_x <= half_length and
                -half_width <= local_y <= half_width
            )
            
        except Exception as e:
            self.logger.error(f"Error checking vehicle bounds: {e}", exc_info=True)
            return False
    
    def validate_tyre_positions_batch(
        self,
        tyre_positions: List[Point],
        vehicle_pose: Optional[PoseStamped] = None
    ) -> Dict:
        """
        Validate multiple tyre positions.
        
        Args:
            tyre_positions: List of tyre positions
            vehicle_pose: Optional vehicle pose
            
        Returns:
            Dict with validation results for each position
        """
        results = {}
        
        for i, position in enumerate(tyre_positions):
            result = self.validate_tyre_position(position, vehicle_pose)
            results[f"tyre_{i}"] = result
        
        # Summary
        valid_count = sum(1 for r in results.values() if r['valid'])
        total_count = len(results)
        
        return {
            'individual_results': results,
            'valid_count': valid_count,
            'total_count': total_count,
            'all_valid': valid_count == total_count
        }

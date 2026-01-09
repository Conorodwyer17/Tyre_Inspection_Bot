#!/usr/bin/env python3
"""
Tyre Side Identifier Module

Identifies tyre positions relative to vehicle (front-left, front-right, rear-left, rear-right).
Helps organize tyres and ensure all positions are covered.

This module provides:
- Tyre side identification based on position
- Relative position calculation
- Side-based organization
- Missing side detection
"""

import math
from typing import Optional, Dict, List, Tuple
from geometry_msgs.msg import Point, PoseStamped


class TyreSideIdentifier:
    """
    Identifies tyre positions relative to vehicle.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre side identifier.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.vehicle_length = parameters.get('vehicle_length', 8.0)  # meters
        self.vehicle_width = parameters.get('vehicle_width', 2.5)  # meters
        self.side_tolerance = parameters.get('tyre_side_tolerance', 0.5)  # meters
        
        self.logger.info(
            f"TyreSideIdentifier initialized: "
            f"vehicle_size=({self.vehicle_length}m x {self.vehicle_width}m)"
        )
    
    def identify_tyre_side(
        self,
        tyre_position: Point,
        vehicle_pose: PoseStamped
    ) -> Dict:
        """
        Identify tyre side relative to vehicle.
        
        Args:
            tyre_position: Tyre position
            vehicle_pose: Vehicle pose
            
        Returns:
            Dict with:
                - 'side': str - Identified side (front-left, front-right, rear-left, rear-right, unknown)
                - 'confidence': float - Confidence in identification (0-1)
                - 'relative_position': Dict - Position relative to vehicle center
                - 'distance_from_center': float - Distance from vehicle center
        """
        result = {
            'side': 'unknown',
            'confidence': 0.0,
            'relative_position': {'x': 0.0, 'y': 0.0},
            'distance_from_center': 0.0
        }
        
        try:
            if not vehicle_pose or not hasattr(vehicle_pose.pose, 'position'):
                return result
            
            vehicle_pos = vehicle_pose.pose.position
            vehicle_ori = vehicle_pose.pose.orientation
            
            # Calculate vehicle orientation
            yaw = 2.0 * math.atan2(vehicle_ori.z, vehicle_ori.w)
            
            # Transform tyre position to vehicle frame
            dx = tyre_position.x - vehicle_pos.x
            dy = tyre_position.y - vehicle_pos.y
            
            # Rotate to vehicle frame
            cos_yaw = math.cos(-yaw)
            sin_yaw = math.sin(-yaw)
            
            local_x = dx * cos_yaw - dy * sin_yaw
            local_y = dx * sin_yaw + dy * cos_yaw
            
            result['relative_position'] = {'x': local_x, 'y': local_y}
            result['distance_from_center'] = math.sqrt(local_x**2 + local_y**2)
            
            # Determine side based on position
            # Front/rear: based on x coordinate (positive = front, negative = rear)
            # Left/right: based on y coordinate (positive = right, negative = left)
            
            front_threshold = self.vehicle_length / 4.0  # Quarter of vehicle length
            side_threshold = self.vehicle_width / 4.0  # Quarter of vehicle width
            
            is_front = local_x > front_threshold
            is_rear = local_x < -front_threshold
            is_left = local_y < -side_threshold
            is_right = local_y > side_threshold
            
            # Identify side
            if is_front and is_left:
                result['side'] = 'front-left'
                result['confidence'] = self._calculate_confidence(local_x, local_y, is_front, is_left)
            elif is_front and is_right:
                result['side'] = 'front-right'
                result['confidence'] = self._calculate_confidence(local_x, local_y, is_front, is_right)
            elif is_rear and is_left:
                result['side'] = 'rear-left'
                result['confidence'] = self._calculate_confidence(local_x, local_y, is_rear, is_left)
            elif is_rear and is_right:
                result['side'] = 'rear-right'
                result['confidence'] = self._calculate_confidence(local_x, local_y, is_rear, is_right)
            elif is_front:
                # Front but not clearly left/right
                result['side'] = 'front-center'
                result['confidence'] = 0.5
            elif is_rear:
                # Rear but not clearly left/right
                result['side'] = 'rear-center'
                result['confidence'] = 0.5
            else:
                # Near center - unclear
                result['side'] = 'unknown'
                result['confidence'] = 0.3
            
            self.logger.debug(
                f"Tyre side identified: {result['side']} "
                f"(confidence: {result['confidence']:.2f}, "
                f"local_pos: ({local_x:.2f}, {local_y:.2f}))"
            )
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error identifying tyre side: {e}", exc_info=True)
            return result
    
    def _calculate_confidence(
        self,
        local_x: float,
        local_y: float,
        is_primary: bool,
        is_secondary: bool
    ) -> float:
        """
        Calculate confidence in side identification.
        
        Args:
            local_x: X position in vehicle frame
            local_y: Y position in vehicle frame
            is_primary: Is primary direction clear (front/rear)?
            is_secondary: Is secondary direction clear (left/right)?
            
        Returns:
            Confidence value (0-1)
        """
        # Base confidence on how far from center
        distance_from_center = math.sqrt(local_x**2 + local_y**2)
        max_distance = math.sqrt(
            (self.vehicle_length/2)**2 + (self.vehicle_width/2)**2
        )
        
        distance_confidence = min(1.0, distance_from_center / max_distance)
        
        # Boost confidence if both directions are clear
        if is_primary and is_secondary:
            return min(1.0, distance_confidence * 1.2)
        elif is_primary or is_secondary:
            return distance_confidence * 0.8
        else:
            return distance_confidence * 0.5
    
    def identify_all_tyre_sides(
        self,
        tyres: List,
        vehicle_pose: PoseStamped
    ) -> Dict:
        """
        Identify sides for all tyres.
        
        Args:
            tyres: List of TyreData objects
            vehicle_pose: Vehicle pose
            
        Returns:
            Dict with:
                - 'tyre_sides': Dict[tyre_id, side_info]
                - 'side_coverage': Dict[side, count] - How many tyres per side
                - 'missing_sides': List[str] - Sides with no tyres
        """
        result = {
            'tyre_sides': {},
            'side_coverage': {
                'front-left': 0,
                'front-right': 0,
                'rear-left': 0,
                'rear-right': 0,
                'front-center': 0,
                'rear-center': 0,
                'unknown': 0
            },
            'missing_sides': []
        }
        
        try:
            # Identify each tyre
            for tyre in tyres:
                if not hasattr(tyre, 'position_3d') or not tyre.position_3d:
                    continue
                
                side_info = self.identify_tyre_side(tyre.position_3d, vehicle_pose)
                tyre_id = getattr(tyre, 'tyre_id', 'unknown')
                
                result['tyre_sides'][tyre_id] = side_info
                result['side_coverage'][side_info['side']] += 1
            
            # Identify missing sides (expected sides with no tyres)
            expected_sides = ['front-left', 'front-right', 'rear-left', 'rear-right']
            for side in expected_sides:
                if result['side_coverage'][side] == 0:
                    result['missing_sides'].append(side)
            
            self.logger.info(
                f"Tyre side identification complete: "
                f"{len(result['tyre_sides'])} tyres identified, "
                f"missing sides: {result['missing_sides']}"
            )
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error identifying all tyre sides: {e}", exc_info=True)
            return result

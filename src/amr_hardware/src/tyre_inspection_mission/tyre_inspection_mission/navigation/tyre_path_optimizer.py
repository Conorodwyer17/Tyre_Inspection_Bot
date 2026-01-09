#!/usr/bin/env python3
"""
Tyre Path Optimizer Module

Optimizes the order of tyre visits to minimize total travel distance.
Uses nearest-neighbor algorithm with improvements for real-world navigation.

This module provides:
- Optimal tyre visit ordering
- Travel distance minimization
- Side-based grouping (visit all tyres on one side before crossing)
- Distance-based sorting
"""

import math
from typing import List, Tuple, Optional, Dict
from geometry_msgs.msg import Point


class TyrePathOptimizer:
    """
    Optimizes the order of tyre visits to minimize travel distance.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre path optimizer.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.use_side_based_grouping = parameters.get('use_side_based_grouping', True)
        self.optimal_tyre_distance = parameters.get('approach_distance', 1.5)  # meters
        
        self.logger.info(
            f"TyrePathOptimizer initialized: "
            f"side_based_grouping={self.use_side_based_grouping}"
        )
    
    def optimize_tyre_visit_order(
        self,
        tyres: List,  # List of TyreData objects
        start_position: Optional[Point] = None
    ) -> List[int]:
        """
        Optimize the order of tyre visits.
        
        Uses nearest-neighbor algorithm with optional side-based grouping.
        Groups tyres by vehicle side (left/right) to minimize crossovers.
        
        Args:
            tyres: List of TyreData objects
            start_position: Optional starting position (robot or vehicle center)
            
        Returns:
            List of indices in optimal visit order
        """
        if not tyres:
            return []
        
        if len(tyres) == 1:
            return [0]
        
        try:
            # Extract tyre positions
            tyre_positions = []
            for i, tyre in enumerate(tyres):
                if hasattr(tyre, 'position_3d') and tyre.position_3d:
                    tyre_positions.append((i, tyre.position_3d))
            
            if not tyre_positions:
                self.logger.warn("No tyre positions available. Returning original order.")
                return list(range(len(tyres)))
            
            # Use starting position if provided, otherwise use first tyre position
            if start_position:
                start_pos = (start_position.x, start_position.y)
            else:
                first_tyre_pos = tyre_positions[0][1]
                start_pos = (first_tyre_pos.x, first_tyre_pos.y)
            
            # Strategy 1: Side-based grouping (visit all tyres on one side first)
            if self.use_side_based_grouping and len(tyre_positions) > 2:
                optimized_indices = self._optimize_with_side_grouping(
                    tyre_positions,
                    start_pos,
                    tyres
                )
            else:
                # Strategy 2: Simple nearest-neighbor
                optimized_indices = self._optimize_nearest_neighbor(
                    tyre_positions,
                    start_pos
                )
            
            self.logger.info(
                f"Optimized tyre visit order: {optimized_indices} "
                f"(original: {list(range(len(tyres)))})"
            )
            
            return optimized_indices
            
        except Exception as e:
            self.logger.error(f"Error optimizing tyre path: {e}", exc_info=True)
            # Fallback: return original order
            return list(range(len(tyres)))
    
    def _optimize_with_side_grouping(
        self,
        tyre_positions: List[Tuple[int, Point]],
        start_pos: Tuple[float, float],
        tyres: List
    ) -> List[int]:
        """
        Optimize with side-based grouping.
        Groups tyres by vehicle side and visits all on one side before crossing.
        
        Args:
            tyre_positions: List of (index, position) tuples
            start_pos: Starting position (x, y)
            tyres: Original tyres list (for reference)
            
        Returns:
            Optimized indices
        """
        # Calculate vehicle center (average of all tyre positions)
        center_x = sum(pos[1].x for pos in tyre_positions) / len(tyre_positions)
        center_y = sum(pos[1].y for pos in tyre_positions) / len(tyre_positions)
        
        # Determine vehicle orientation (principal direction)
        # Use first two tyres to estimate vehicle direction
        if len(tyre_positions) >= 2:
            dx = tyre_positions[1][1].x - tyre_positions[0][1].x
            dy = tyre_positions[1][1].y - tyre_positions[0][1].y
            vehicle_angle = math.atan2(dy, dx)
        else:
            vehicle_angle = 0.0
        
        # Separate tyres by side (relative to vehicle center)
        left_side = []
        right_side = []
        
        # Calculate perpendicular vector to vehicle orientation
        perp_x = -math.sin(vehicle_angle)
        perp_y = math.cos(vehicle_angle)
        
        for idx, pos in tyre_positions:
            # Vector from center to tyre
            dx = pos.x - center_x
            dy = pos.y - center_y
            
            # Dot product with perpendicular vector determines side
            side = dx * perp_x + dy * perp_y
            
            if side >= 0:
                right_side.append((idx, pos))
            else:
                left_side.append((idx, pos))
        
        # Choose starting side based on which is closer to start position
        left_dist = self._min_distance_to_group(left_side, start_pos) if left_side else float('inf')
        right_dist = self._min_distance_to_group(right_side, start_pos) if right_side else float('inf')
        
        # Optimize each side independently
        optimized_order = []
        current_pos = start_pos
        
        if left_dist <= right_dist:
            # Start with left side
            if left_side:
                left_indices = self._optimize_nearest_neighbor(left_side, current_pos)
                optimized_order.extend(left_indices)
                # Update current position to last visited tyre
                last_left_idx = left_indices[-1]
                last_pos = next(pos[1] for i, pos in enumerate(left_side) if pos[0] == last_left_idx)
                current_pos = (last_pos.x, last_pos.y)
            
            if right_side:
                right_indices = self._optimize_nearest_neighbor(right_side, current_pos)
                optimized_order.extend(right_indices)
        else:
            # Start with right side
            if right_side:
                right_indices = self._optimize_nearest_neighbor(right_side, current_pos)
                optimized_order.extend(right_indices)
                # Update current position
                last_right_idx = right_indices[-1]
                last_pos = next(pos[1] for i, pos in enumerate(right_side) if pos[0] == last_right_idx)
                current_pos = (last_pos.x, last_pos.y)
            
            if left_side:
                left_indices = self._optimize_nearest_neighbor(left_side, current_pos)
                optimized_order.extend(left_indices)
        
        return optimized_order
    
    def _optimize_nearest_neighbor(
        self,
        tyre_positions: List[Tuple[int, Point]],
        start_pos: Tuple[float, float]
    ) -> List[int]:
        """
        Optimize using nearest-neighbor algorithm.
        
        Args:
            tyre_positions: List of (index, position) tuples
            start_pos: Starting position (x, y)
            
        Returns:
            Optimized indices
        """
        if not tyre_positions:
            return []
        
        if len(tyre_positions) == 1:
            return [tyre_positions[0][0]]
        
        unvisited = tyre_positions.copy()
        optimized = []
        current_pos = start_pos
        
        while unvisited:
            # Find nearest unvisited tyre
            nearest_idx = 0
            nearest_dist = float('inf')
            
            for i, (idx, pos) in enumerate(unvisited):
                dist = math.sqrt(
                    (pos.x - current_pos[0])**2 +
                    (pos.y - current_pos[1])**2
                )
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_idx = i
            
            # Visit nearest tyre
            visited_idx, visited_pos = unvisited.pop(nearest_idx)
            optimized.append(visited_idx)
            current_pos = (visited_pos.x, visited_pos.y)
        
        return optimized
    
    def _min_distance_to_group(
        self,
        group: List[Tuple[int, Point]],
        position: Tuple[float, float]
    ) -> float:
        """
        Calculate minimum distance from position to any tyre in group.
        
        Args:
            group: List of (index, position) tuples
            position: Position (x, y)
            
        Returns:
            Minimum distance
        """
        min_dist = float('inf')
        for idx, pos in group:
            dist = math.sqrt(
                (pos.x - position[0])**2 +
                (pos.y - position[1])**2
            )
            min_dist = min(min_dist, dist)
        return min_dist
    
    def calculate_total_distance(
        self,
        tyres: List,
        order: List[int],
        start_position: Optional[Point] = None
    ) -> float:
        """
        Calculate total travel distance for given order.
        
        Args:
            tyres: List of TyreData objects
            order: Visit order (list of indices)
            start_position: Optional starting position
            
        Returns:
            Total distance in meters
        """
        if not tyres or not order:
            return 0.0
        
        try:
            total_distance = 0.0
            
            # Start position
            if start_position:
                current_pos = (start_position.x, start_position.y)
            else:
                first_tyre = tyres[order[0]]
                if hasattr(first_tyre, 'position_3d') and first_tyre.position_3d:
                    current_pos = (first_tyre.position_3d.x, first_tyre.position_3d.y)
                else:
                    return 0.0
            
            # Calculate distance to each tyre in order
            for idx in order:
                if idx >= len(tyres):
                    continue
                
                tyre = tyres[idx]
                if not hasattr(tyre, 'position_3d') or not tyre.position_3d:
                    continue
                
                tyre_pos = tyre.position_3d
                distance = math.sqrt(
                    (tyre_pos.x - current_pos[0])**2 +
                    (tyre_pos.y - current_pos[1])**2
                )
                total_distance += distance
                current_pos = (tyre_pos.x, tyre_pos.y)
            
            return total_distance
            
        except Exception as e:
            self.logger.error(f"Error calculating total distance: {e}", exc_info=True)
            return 0.0

#!/usr/bin/env python3
"""
Vehicle Obstacle Manager Module

Manages vehicle as a dynamic obstacle during tyre navigation.
Ensures paths avoid vehicle collisions and provides waypoint planning around vehicle.

This module provides:
- Vehicle obstacle representation
- Collision detection with vehicle
- Waypoint calculation around vehicle
- Path validation against vehicle bounds
"""

import math
from typing import Optional, List, Dict, Tuple
from geometry_msgs.msg import PoseStamped, Point


class VehicleObstacleManager:
    """
    Manages vehicle as obstacle during tyre navigation.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize vehicle obstacle manager.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.vehicle_safety_margin = parameters.get('vehicle_safety_margin', 1.0)  # meters
        self.min_corner_clearance = parameters.get('min_corner_clearance', 1.5)  # meters
        self.use_waypoint_navigation = parameters.get('use_waypoint_navigation', True)
        
        # Vehicle representation
        self.vehicle_pose = None
        self.vehicle_bounds = None  # Dict with min_x, max_x, min_y, max_y
        self.vehicle_length = 8.0  # meters - typical truck length
        self.vehicle_width = 2.5  # meters - typical truck width
        
        self.logger.info(
            f"VehicleObstacleManager initialized: "
            f"safety_margin={self.vehicle_safety_margin}m, "
            f"corner_clearance={self.min_corner_clearance}m"
        )
    
    def update_vehicle_obstacle(self, vehicle_pose: PoseStamped):
        """
        Update vehicle position for obstacle avoidance.
        
        Args:
            vehicle_pose: Current vehicle pose
        """
        if not vehicle_pose or not hasattr(vehicle_pose, 'pose'):
            return
        
        self.vehicle_pose = vehicle_pose
        
        # Calculate vehicle bounds (rectangle around vehicle)
        pos = vehicle_pose.pose.position
        ori = vehicle_pose.pose.orientation
        
        # Calculate vehicle orientation (yaw)
        yaw = 2.0 * math.atan2(ori.z, ori.w)
        
        # Vehicle dimensions
        length = self.vehicle_length
        width = self.vehicle_width
        
        # Calculate corners of vehicle bounding box
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Half dimensions
        half_length = length / 2.0
        half_width = width / 2.0
        
        # Corner offsets (relative to center)
        corners = [
            (-half_length, -half_width),  # back-left
            (half_length, -half_width),   # front-left
            (half_length, half_width),    # front-right
            (-half_length, half_width)    # back-right
        ]
        
        # Transform corners to world coordinates
        x_coords = []
        y_coords = []
        
        for dx, dy in corners:
            # Rotate by yaw
            world_dx = dx * cos_yaw - dy * sin_yaw
            world_dy = dx * sin_yaw + dy * cos_yaw
            
            # Translate to vehicle position
            x_coords.append(pos.x + world_dx)
            y_coords.append(pos.y + world_dy)
        
        # Calculate bounding box with safety margin
        self.vehicle_bounds = {
            'min_x': min(x_coords) - self.vehicle_safety_margin,
            'max_x': max(x_coords) + self.vehicle_safety_margin,
            'min_y': min(y_coords) - self.vehicle_safety_margin,
            'max_y': max(y_coords) + self.vehicle_safety_margin
        }
        
        self.logger.debug(
            f"Vehicle obstacle updated: "
            f"bounds=({self.vehicle_bounds['min_x']:.2f}, {self.vehicle_bounds['min_y']:.2f}) "
            f"to ({self.vehicle_bounds['max_x']:.2f}, {self.vehicle_bounds['max_y']:.2f})"
        )
    
    def check_path_clear(self, start_pose: Point, end_pose: Point) -> Tuple[bool, Optional[str]]:
        """
        Check if straight-line path from start to end avoids vehicle.
        
        Args:
            start_pose: Starting position
            end_pose: Ending position
            
        Returns:
            Tuple of (is_clear: bool, issue: str or None)
        """
        if not self.vehicle_bounds:
            return True, None
        
        # Check if start or end is inside vehicle bounds
        if self._point_in_bounds(start_pose):
            return False, "Start position is inside vehicle obstacle bounds"
        
        if self._point_in_bounds(end_pose):
            return False, "End position is inside vehicle obstacle bounds"
        
        # Check if path intersects vehicle bounds
        # Simple check: see if line segment intersects bounding box
        if self._line_intersects_bounds(start_pose, end_pose):
            return False, "Path intersects vehicle obstacle bounds"
        
        return True, None
    
    def calculate_waypoint_around_vehicle(
        self,
        start_pose: Point,
        target_pose: Point
    ) -> Optional[Point]:
        """
        Calculate waypoint to navigate around vehicle if direct path is blocked.
        
        Args:
            start_pose: Starting position
            target_pose: Target position
            
        Returns:
            Waypoint Point, or None if direct path is clear
        """
        if not self.vehicle_bounds:
            return None
        
        # Check if path is clear
        is_clear, issue = self.check_path_clear(start_pose, target_pose)
        if is_clear:
            return None  # No waypoint needed
        
        # Calculate waypoint around vehicle
        # Strategy: Find closest corner and go around it
        
        # Calculate vehicle center
        center_x = (self.vehicle_bounds['min_x'] + self.vehicle_bounds['max_x']) / 2.0
        center_y = (self.vehicle_bounds['min_y'] + self.vehicle_bounds['max_y']) / 2.0
        
        # Determine which side of vehicle to go around
        # Check if target is on left or right side of vehicle (relative to start)
        dx = target_pose.x - start_pose.x
        dy = target_pose.y - start_pose.y
        
        # Calculate angle from start to target
        angle = math.atan2(dy, dx)
        
        # Calculate perpendicular offset to go around vehicle
        # Use corner clearance distance
        offset_x = math.cos(angle + math.pi/2) * self.min_corner_clearance
        offset_y = math.sin(angle + math.pi/2) * self.min_corner_clearance
        
        # Calculate waypoint
        waypoint = Point()
        waypoint.x = center_x + offset_x
        waypoint.y = center_y + offset_y
        waypoint.z = start_pose.z  # Keep same height
        
        self.logger.info(
            f"Calculated waypoint around vehicle: ({waypoint.x:.2f}, {waypoint.y:.2f})"
        )
        
        return waypoint
    
    def _point_in_bounds(self, point: Point) -> bool:
        """Check if point is inside vehicle bounds."""
        if not self.vehicle_bounds:
            return False
        
        return (
            self.vehicle_bounds['min_x'] <= point.x <= self.vehicle_bounds['max_x'] and
            self.vehicle_bounds['min_y'] <= point.y <= self.vehicle_bounds['max_y']
        )
    
    def _line_intersects_bounds(self, start: Point, end: Point) -> bool:
        """
        Check if line segment intersects vehicle bounding box.
        Uses simplified axis-aligned bounding box intersection.
        """
        if not self.vehicle_bounds:
            return False
        
        # Check if any part of line segment is inside bounds
        # Sample points along the line
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            x = start.x + t * (end.x - start.x)
            y = start.y + t * (end.y - start.y)
            
            point = Point()
            point.x = x
            point.y = y
            point.z = start.z
            
            if self._point_in_bounds(point):
                return True
        
        return False
    
    def validate_navigation_goal(self, goal_pose: PoseStamped) -> Tuple[bool, Optional[str]]:
        """
        Validate that navigation goal doesn't collide with vehicle.
        
        Args:
            goal_pose: Navigation goal pose
            
        Returns:
            Tuple of (is_valid: bool, issue: str or None)
        """
        if not self.vehicle_bounds:
            return True, None
        
        goal_pos = goal_pose.pose.position
        
        if self._point_in_bounds(goal_pos):
            return False, "Navigation goal is inside vehicle obstacle bounds"
        
        # Check minimum distance to vehicle
        center_x = (self.vehicle_bounds['min_x'] + self.vehicle_bounds['max_x']) / 2.0
        center_y = (self.vehicle_bounds['min_y'] + self.vehicle_bounds['max_y']) / 2.0
        
        distance = math.sqrt(
            (goal_pos.x - center_x)**2 +
            (goal_pos.y - center_y)**2
        )
        
        min_safe_distance = max(self.vehicle_length, self.vehicle_width) / 2.0 + self.vehicle_safety_margin
        
        if distance < min_safe_distance:
            return False, f"Goal too close to vehicle: {distance:.2f}m < {min_safe_distance:.2f}m"
        
        return True, None
    
    def get_vehicle_bounds(self) -> Optional[Dict]:
        """Get current vehicle bounds."""
        return self.vehicle_bounds
    
    def clear_vehicle_obstacle(self):
        """Clear vehicle obstacle (call when vehicle inspection complete)."""
        self.vehicle_pose = None
        self.vehicle_bounds = None
        self.logger.info("Vehicle obstacle cleared")

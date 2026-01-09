#!/usr/bin/env python3
"""
Local Search Module

Performs local search patterns to re-locate vehicle if not found at expected location.
Uses spiral or grid patterns around arrival location.

This module provides:
- Spiral search pattern generation
- Grid search pattern generation
- Search execution with vehicle re-detection
- Optimal position finding
"""

import math
import time
from typing import List, Optional, Dict
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class LocalSearch:
    """
    Performs local search patterns to find vehicle.
    """
    
    def __init__(self, logger, nav_client: ActionClient, parameters: Dict):
        """
        Initialize local search.
        
        Args:
            logger: ROS 2 logger instance
            nav_client: Nav2 action client
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        self.nav_client = nav_client
        
        # Parameters
        self.local_search_radius = parameters.get('local_search_radius', 2.0)  # meters
        self.local_search_points = parameters.get('local_search_points', 8)  # number of positions
        self.local_search_timeout = parameters.get('local_search_timeout', 30.0)  # seconds
        self.local_search_spacing = parameters.get('local_search_spacing', 0.5)  # meters
        self.arrival_distance_threshold = parameters.get('arrival_distance_threshold', 0.5)  # meters
        self.navigation_timeout = parameters.get('navigation_timeout', 60.0)  # seconds
        
        # Search state
        self.search_active = False
        self.search_start_time = None
        self.current_search_index = 0
        self.search_poses = []
        self.vehicle_found = False
        self.found_pose = None
        
        self.logger.info(
            f"LocalSearch initialized: "
            f"radius={self.local_search_radius}m, "
            f"points={self.local_search_points}, "
            f"timeout={self.local_search_timeout}s"
        )
    
    def search_for_vehicle(
        self,
        center_pose: PoseStamped,
        vehicle_id: str,
        bbox_callback_func,
        check_vehicle_visible_func,
        get_robot_pose_func,
        frame_id: str = "map"
    ) -> Dict:
        """
        Execute local search pattern to find vehicle.
        
        Args:
            center_pose: Center position for search (arrival location)
            vehicle_id: ID of vehicle to search for
            bbox_callback_func: Function to check bounding boxes (for vehicle detection)
            check_vehicle_visible_func: Function to check if vehicle is visible
            get_robot_pose_func: Function to get current robot pose
            frame_id: Frame ID for search poses (default: "map")
            
        Returns:
            Dict with:
                - 'vehicle_found': bool - Was vehicle found?
                - 'found_pose': PoseStamped or None - Position where vehicle found
                - 'search_complete': bool - Is search complete?
                - 'search_positions_checked': int - Number of positions checked
        """
        result = {
            'vehicle_found': False,
            'found_pose': None,
            'search_complete': False,
            'search_positions_checked': 0
        }
        
        if self.search_start_time is None:
            # Initialize search
            self.search_start_time = time.time()
            self.search_active = True
            self.current_search_index = 0
            self.vehicle_found = False
            self.found_pose = None
            
            # Generate search pattern
            self.search_poses = self._generate_spiral_search_pattern(
                center_pose,
                self.local_search_radius,
                self.local_search_points,
                frame_id
            )
            
            self.logger.info(
                f"LocalSearch: Starting search for {vehicle_id} "
                f"around ({center_pose.pose.position.x:.2f}, {center_pose.pose.position.y:.2f}) "
                f"with {len(self.search_poses)} search positions"
            )
        
        current_time = time.time()
        elapsed_time = current_time - self.search_start_time
        
        # Check timeout
        if elapsed_time > self.local_search_timeout:
            self.logger.warn(
                f"LocalSearch: Search timeout after {elapsed_time:.1f}s "
                f"(timeout: {self.local_search_timeout}s)"
            )
            result['search_complete'] = True
            self._reset_search()
            return result
        
        # Check if we've checked all positions
        if self.current_search_index >= len(self.search_poses):
            self.logger.warn(
                f"LocalSearch: All search positions checked. Vehicle not found."
            )
            result['search_complete'] = True
            self._reset_search()
            return result
        
        # Get current search position
        current_search_pose = self.search_poses[self.current_search_index]
        
        # Check if vehicle is visible at current position
        # (This would be called from mission_controller with current bbox_msg)
        # For now, we'll return the current search state
        
        result['search_positions_checked'] = self.current_search_index
        
        # If vehicle found (set externally via check_vehicle_at_position)
        if self.vehicle_found and self.found_pose:
            result['vehicle_found'] = True
            result['found_pose'] = self.found_pose
            result['search_complete'] = True
            self.logger.info(
                f"LocalSearch: ✅ Vehicle found at search position {self.current_search_index + 1} "
                f"at ({self.found_pose.pose.position.x:.2f}, {self.found_pose.pose.position.y:.2f})"
            )
            self._reset_search()
            return result
        
        # Continue search - return current state
        # Mission controller will navigate to current_search_pose and check for vehicle
        return result
    
    def get_current_search_pose(self) -> Optional[PoseStamped]:
        """
        Get current search position to navigate to.
        
        Returns:
            Current search pose, or None if search not active
        """
        if not self.search_active or self.current_search_index >= len(self.search_poses):
            return None
        
        return self.search_poses[self.current_search_index]
    
    def advance_search_position(self):
        """Advance to next search position."""
        if self.search_active:
            self.current_search_index += 1
            if self.current_search_index < len(self.search_poses):
                self.logger.info(
                    f"LocalSearch: Advancing to search position {self.current_search_index + 1}/{len(self.search_poses)}"
                )
    
    def mark_vehicle_found(self, found_pose: PoseStamped):
        """
        Mark vehicle as found at given pose.
        
        Args:
            found_pose: Pose where vehicle was found
        """
        self.vehicle_found = True
        self.found_pose = found_pose
    
    def _generate_spiral_search_pattern(
        self,
        center_pose: PoseStamped,
        radius: float,
        num_points: int,
        frame_id: str
    ) -> List[PoseStamped]:
        """
        Generate spiral search pattern around center position.
        
        Args:
            center_pose: Center position for search
            radius: Maximum search radius
            num_points: Number of search positions
            frame_id: Frame ID for poses
            
        Returns:
            List of PoseStamped search positions
        """
        search_poses = []
        center_x = center_pose.pose.position.x
        center_y = center_pose.pose.position.y
        center_z = center_pose.pose.position.z
        
        # Generate spiral pattern
        for i in range(num_points):
            # Spiral angle
            angle = (2 * math.pi * i) / num_points
            
            # Spiral radius (increases with index)
            spiral_radius = (radius * (i + 1)) / num_points
            
            # Calculate position
            x = center_x + spiral_radius * math.cos(angle)
            y = center_y + spiral_radius * math.sin(angle)
            
            # Create pose
            search_pose = PoseStamped()
            search_pose.header.frame_id = frame_id
            search_pose.header.stamp = center_pose.header.stamp
            
            search_pose.pose.position.x = float(x)
            search_pose.pose.position.y = float(y)
            search_pose.pose.position.z = center_z
            
            # Orient toward center
            yaw = math.atan2(center_y - y, center_x - x) + math.pi
            search_pose.pose.orientation.z = math.sin(yaw / 2.0)
            search_pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            search_poses.append(search_pose)
        
        return search_poses
    
    def _generate_grid_search_pattern(
        self,
        center_pose: PoseStamped,
        grid_size: int,
        spacing: float,
        frame_id: str
    ) -> List[PoseStamped]:
        """
        Generate grid search pattern around center position.
        
        Args:
            center_pose: Center position for search
            grid_size: Grid size (e.g., 3 = 3x3 grid)
            spacing: Spacing between grid points (meters)
            frame_id: Frame ID for poses
            
        Returns:
            List of PoseStamped search positions
        """
        search_poses = []
        center_x = center_pose.pose.position.x
        center_y = center_pose.pose.position.y
        center_z = center_pose.pose.position.z
        
        # Generate grid pattern
        half_grid = (grid_size - 1) / 2.0
        
        for i in range(grid_size):
            for j in range(grid_size):
                # Skip center position (already checked)
                if i == half_grid and j == half_grid:
                    continue
                
                x = center_x + (i - half_grid) * spacing
                y = center_y + (j - half_grid) * spacing
                
                # Create pose
                search_pose = PoseStamped()
                search_pose.header.frame_id = frame_id
                search_pose.header.stamp = center_pose.header.stamp
                
                search_pose.pose.position.x = float(x)
                search_pose.pose.position.y = float(y)
                search_pose.pose.position.z = center_z
                
                # Orient toward center
                yaw = math.atan2(center_y - y, center_x - x) + math.pi
                search_pose.pose.orientation.z = math.sin(yaw / 2.0)
                search_pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                search_poses.append(search_pose)
        
        return search_poses
    
    def _reset_search(self):
        """Reset search state."""
        self.search_active = False
        self.search_start_time = None
        self.current_search_index = 0
        self.search_poses = []
        self.vehicle_found = False
        self.found_pose = None
    
    def is_search_active(self) -> bool:
        """Check if search is currently active."""
        return self.search_active
    
    def cancel_search(self):
        """Cancel active search."""
        self.logger.info("LocalSearch: Cancelling search")
        self._reset_search()

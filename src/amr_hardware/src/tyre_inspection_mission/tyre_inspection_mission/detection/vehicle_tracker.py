#!/usr/bin/env python3
"""
Vehicle Tracker Module

Handles vehicle detection, tracking, and LiDAR fusion.
Manages vehicle data structures and detection state.

This module uses shared data structures from data_structures module
to avoid duplication and ensure consistency.
"""

from typing import Optional, List, Dict, Tuple
import time
from geometry_msgs.msg import PoseStamped
from gb_visual_detection_3d_msgs.msg import BoundingBox3d
from sensor_msgs.msg import LaserScan
import tf2_ros
import math

# Import shared data structures and utilities
from tyre_inspection_mission.common.data_structures import VehicleData, TyreData
from tyre_inspection_mission.common.config import DetectionConfig
from tyre_inspection_mission.common.structured_logger import StructuredLogger, LogCategory
from tyre_inspection_mission.common.exceptions import DetectionError
from tyre_inspection_mission.common.utils import calculate_distance_2d


class VehicleTracker:
    """
    Manages vehicle detection and tracking with camera and LiDAR fusion.
    
    This class provides:
    - Vehicle detection from camera bounding boxes
    - Tyre detection from camera bounding boxes
    - LiDAR-based vehicle tracking
    - Vehicle/tyre matching and association
    - Sensor fusion for continuous tracking
    
    Attributes:
        node: ROS 2 node (for logging and parameters)
        detected_vehicles: Dictionary mapping vehicle_id to VehicleData
        vehicle_counter: Counter for generating unique vehicle IDs
        vehicle_class_names: List of vehicle class names to detect
        tyre_class_name: Class name for tyres
        lidar_vehicle_min_size: Minimum vehicle size in LiDAR (meters)
        lidar_vehicle_max_size: Maximum vehicle size in LiDAR (meters)
        lidar_tracking_distance_threshold: Max distance to match detections (meters)
        latest_lidar_scan: Most recent LiDAR scan
        slogger: Structured logger instance
    """
    
    def __init__(self, node):
        """
        Initialize vehicle tracker.
        
        Args:
            node: ROS 2 node (for logging and parameters)
            
        Raises:
            ValueError: If required parameters are invalid
        """
        self.node = node
        
        # Initialize structured logger
        enable_debug = node.get_parameter('enable_debug_logs').value if node.has_parameter('enable_debug_logs') else False
        self.slogger = StructuredLogger(node.get_logger(), enable_debug=enable_debug)
        
        self.detected_vehicles: Dict[str, VehicleData] = {}  # vehicle_id -> VehicleData
        self.vehicle_counter: int = 0
        
        # Get parameters with defaults from config
        vehicle_classes_param = node.get_parameter('vehicle_class_names').value
        if isinstance(vehicle_classes_param, str):
            self.vehicle_class_names: List[str] = [vehicle_classes_param.lower()]
        elif isinstance(vehicle_classes_param, list):
            self.vehicle_class_names: List[str] = [v.lower() for v in vehicle_classes_param]
        else:
            self.vehicle_class_names: List[str] = DetectionConfig.DEFAULT_VEHICLE_CLASSES.copy()
        
        self.tyre_class_name: str = node.get_parameter('tyre_class_name').value.lower()
        
        # LiDAR tracking parameters with defaults from config
        self.lidar_vehicle_min_size: float = node.get_parameter('lidar_vehicle_min_size').value if node.has_parameter('lidar_vehicle_min_size') else DetectionConfig.LIDAR_VEHICLE_MIN_SIZE
        self.lidar_vehicle_max_size: float = node.get_parameter('lidar_vehicle_max_size').value if node.has_parameter('lidar_vehicle_max_size') else DetectionConfig.LIDAR_VEHICLE_MAX_SIZE
        self.lidar_tracking_distance_threshold: float = node.get_parameter('lidar_tracking_distance_threshold').value if node.has_parameter('lidar_tracking_distance_threshold') else DetectionConfig.LIDAR_TRACKING_DISTANCE_THRESHOLD
        
        self.latest_lidar_scan: Optional[LaserScan] = None
        
        self.slogger.info(f"Vehicle tracker initialized: vehicle_classes={self.vehicle_class_names}, "
                         f"tyre_class={self.tyre_class_name}", category=LogCategory.SYSTEM)
        
    def process_bbox_detection(self, bbox: BoundingBox3d, header, goal_planner) -> Optional[VehicleData]:
        """
        Process a bounding box detection from camera.
        
        Determines if detection is a vehicle or tyre and processes accordingly.
        
        Args:
            bbox: BoundingBox3d message with detection information
            header: ROS header with frame_id and timestamp
            goal_planner: GoalPlanner instance for calculating poses
            
        Returns:
            VehicleData if vehicle detected, None otherwise
            (Tyre detections are handled separately by mission controller)
            
        Raises:
            DetectionError: If detection processing fails
        """
        if bbox is None:
            raise DetectionError("bbox cannot be None", error_code="DET_002")
        if header is None:
            raise DetectionError("header cannot be None", error_code="DET_002")
        if goal_planner is None:
            raise DetectionError("goal_planner cannot be None", error_code="DET_002")
        
        try:
            class_name = bbox.class_name.lower() if bbox.class_name else ""
            
            # Check if it's a vehicle
            if class_name in self.vehicle_class_names:
                return self._process_vehicle_detection(bbox, header, goal_planner)
            
            # Tyre detections are handled by mission controller, not here
            # This allows mission controller to associate tyres with vehicles
            
            return None
            
        except Exception as e:
            self.slogger.error(f"Error processing bbox detection: {e}", category=LogCategory.DETECTION)
            raise DetectionError(f"Failed to process detection: {e}", error_code="DET_002", 
                               context={'class_name': bbox.class_name if bbox else None})
    
    def _process_vehicle_detection(self, bbox: BoundingBox3d, header, goal_planner) -> Optional[VehicleData]:
        """
        Process vehicle detection and create/update VehicleData.
        
        Args:
            bbox: BoundingBox3d message
            header: ROS header with frame_id
            goal_planner: GoalPlanner instance for calculating poses
            
        Returns:
            VehicleData object (new or updated), or None if calculation fails
            
        Raises:
            DetectionError: If vehicle detection processing fails
        """
        try:
            # Calculate detection pose using goal planner
            detection_pose = goal_planner.calculate_vehicle_approach_goal(bbox, header)
            if detection_pose is None:
                self.slogger.warn(f"Failed to calculate detection pose for {bbox.class_name}", 
                                category=LogCategory.DETECTION)
                return None
            
            # Check if this matches an existing vehicle
            vehicle_id = self._match_existing_vehicle(detection_pose)
            
            if vehicle_id is None:
                # New vehicle detected
                vehicle_id = f"{bbox.class_name.lower()}_{self.vehicle_counter:03d}"
                self.vehicle_counter += 1
                
                vehicle = VehicleData(vehicle_id, bbox.class_name.lower(), detection_pose)
                self.detected_vehicles[vehicle_id] = vehicle
                
                self.slogger.detection_found(
                    object_type="vehicle",
                    object_id=vehicle_id,
                    position=(detection_pose.pose.position.x, detection_pose.pose.position.y),
                    confidence=None
                )
                
                self.slogger.info(f"New vehicle detected: {vehicle_id} ({vehicle.vehicle_type})",
                                category=LogCategory.DETECTION)
            else:
                # Update existing vehicle
                vehicle = self.detected_vehicles[vehicle_id]
                vehicle.update_detection(detection_pose)
                
                time_since = time.time() - vehicle.last_detection_time
                self.slogger.debug(f"Updated vehicle: {vehicle_id} (last seen {time_since:.1f}s ago)",
                                 category=LogCategory.DETECTION)
            
            return vehicle
            
        except Exception as e:
            self.slogger.error(f"Error processing vehicle detection: {e}", category=LogCategory.DETECTION)
            raise DetectionError(f"Failed to process vehicle detection: {e}", error_code="DET_002",
                               context={'class_name': bbox.class_name if bbox else None})
    
    def _match_existing_vehicle(self, detection_pose: PoseStamped) -> Optional[str]:
        """
        Match detection to existing vehicle based on position.
        
        Uses position-based matching with distance threshold to associate
        new detections with previously detected vehicles.
        
        Args:
            detection_pose: PoseStamped in navigation frame
            
        Returns:
            vehicle_id if match found, None if no match
        """
        if detection_pose is None:
            return None
        
        detection_pos = (detection_pose.pose.position.x, detection_pose.pose.position.y)
        
        for vehicle_id, vehicle in self.detected_vehicles.items():
            # Priority 1: Check camera detection position
            if vehicle.last_detection_pose:
                vehicle_pos = (vehicle.last_detection_pose.pose.position.x,
                             vehicle.last_detection_pose.pose.position.y)
                try:
                    distance = calculate_distance_2d(detection_pos, vehicle_pos)
                    if distance < self.lidar_tracking_distance_threshold:
                        return vehicle_id
                except ValueError:
                    continue  # Invalid position, skip
            
            # Priority 2: Check LiDAR position (fallback)
            if vehicle.lidar_position:
                vehicle_pos = (vehicle.lidar_position.pose.position.x,
                             vehicle.lidar_position.pose.position.y)
                try:
                    distance = calculate_distance_2d(detection_pos, vehicle_pos)
                    if distance < self.lidar_tracking_distance_threshold:
                        return vehicle_id
                except ValueError:
                    continue  # Invalid position, skip
        
        return None
    
    def process_lidar_scan(self, scan: LaserScan, tf_buffer: tf2_ros.Buffer, 
                          nav_frame: str) -> List[Dict[str, float]]:
        """
        Process LiDAR scan for vehicle tracking.
        
        Analyzes LiDAR scan to detect large objects (vehicles) by clustering
        consecutive range measurements. Returns detections that match vehicle size.
        
        Args:
            scan: LaserScan message from LiDAR
            tf_buffer: TF2 buffer for coordinate transformations
            nav_frame: Navigation frame name (e.g., 'map' or 'odom')
            
        Returns:
            List of dictionaries with keys: 'angle', 'distance', 'size'
            Empty list if no vehicles detected or scan is invalid
        """
        if scan is None:
            return []
        
        self.latest_lidar_scan = scan
        
        # Simple LiDAR-based vehicle detection
        # Look for large objects (vehicles) in scan
        detections: List[Dict[str, float]] = []
        
        if not scan.ranges or len(scan.ranges) == 0:
            return detections
        
        # Find clusters of valid ranges that could be vehicles
        min_range = scan.range_min
        max_range = scan.range_max
        
        i = 0
        while i < len(scan.ranges):
            # Check if range is valid
            if min_range <= scan.ranges[i] <= max_range:
                # Start of potential vehicle cluster
                cluster_start = i
                cluster_end = i
                
                # Find cluster extent (consecutive valid ranges)
                while cluster_end < len(scan.ranges) and \
                      min_range <= scan.ranges[cluster_end] <= max_range:
                    cluster_end += 1
                
                # Calculate cluster size if valid
                if cluster_start < cluster_end:
                    angle_start = scan.angle_min + cluster_start * scan.angle_increment
                    angle_end = scan.angle_min + (cluster_end - 1) * scan.angle_increment
                    range_start = scan.ranges[cluster_start]
                    range_end = scan.ranges[cluster_end - 1]
                    
                    # Estimate size (rough approximation)
                    # Size = range difference + arc length
                    size = abs(range_start - range_end) + \
                           abs(range_start * (angle_end - angle_start))
                    
                    # Check if size matches vehicle dimensions
                    if self.lidar_vehicle_min_size <= size <= self.lidar_vehicle_max_size:
                        # Calculate center of cluster
                        center_angle = (angle_start + angle_end) / 2.0
                        center_range = (range_start + range_end) / 2.0
                        
                        # Store detection (position calculation done elsewhere with proper TF)
                        detections.append({
                            'angle': float(center_angle),
                            'distance': float(center_range),
                            'size': float(size)
                        })
                
                i = cluster_end
            else:
                i += 1
        
        return detections
    
    def get_unprocessed_vehicles(self) -> List[VehicleData]:
        """
        Get list of vehicles that haven't been fully processed.
        
        A vehicle is considered unprocessed if:
        - License plate photo not taken, OR
        - Not all tyres photographed
        
        Returns:
            List of VehicleData objects that need processing
        """
        return [
            vehicle for vehicle in self.detected_vehicles.values()
            if not vehicle.is_inspection_complete()
        ]
    
    def get_vehicle(self, vehicle_id: str) -> Optional[VehicleData]:
        """
        Get vehicle by ID.
        
        Args:
            vehicle_id: Unique vehicle identifier
            
        Returns:
            VehicleData object if found, None otherwise
        """
        return self.detected_vehicles.get(vehicle_id)
    
    def get_all_vehicles(self) -> Dict[str, VehicleData]:
        """
        Get all detected vehicles.
        
        Returns:
            Dictionary mapping vehicle_id to VehicleData
        """
        return self.detected_vehicles.copy()
    
    def clear_vehicles(self) -> None:
        """
        Clear all detected vehicles and reset counter.
        
        Useful for starting a new mission or resetting state.
        """
        self.detected_vehicles.clear()
        self.vehicle_counter = 0
        self.slogger.info("All vehicles cleared", category=LogCategory.SYSTEM)
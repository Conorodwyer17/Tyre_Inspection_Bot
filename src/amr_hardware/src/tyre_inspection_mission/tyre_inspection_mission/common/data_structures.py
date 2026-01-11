#!/usr/bin/env python3
"""
Data Structures Module

Single source of truth for all mission-related data structures:
- MissionState: State machine enumeration
- VehicleData: Vehicle inspection tracking
- TyreData: Tyre inspection tracking

All classes include comprehensive validation, type hints, and documentation.
"""

from enum import Enum
from typing import Optional, List
import time
from geometry_msgs.msg import PoseStamped, Point
from gb_visual_detection_3d_msgs.msg import BoundingBox3d


class MissionState(Enum):
    """
    Mission state machine states.
    
    States represent the current phase of the tyre inspection mission:
    - IDLE: System ready, waiting for start command
    - PLANNING: Pre-planning all waypoints before navigation starts
    - SEARCHING_VEHICLES: Actively searching for vehicles in yard
    - VEHICLE_DETECTED: Vehicle detected, preparing for navigation
    - NAVIGATING_TO_LICENSE_PLATE: Navigating to vehicle's license plate
    - CAPTURING_LICENSE_PLATE: Capturing license plate photo with OCR
    - SWITCHING_TO_INSPECTION: Switching detection mode for tyre detection
    - DETECTING_TYRES: Detecting all tyres on current vehicle
    - NAVIGATING_TO_TYRE: Navigating to a specific tyre
    - CAPTURING_TYRE: Capturing tyre photo (with centering)
    - CHECKING_COMPLETION: Verifying all tyres photographed
    - MISSION_COMPLETE: All vehicles inspected
    - ERROR_RECOVERY: Error state with recovery logic
    """
    IDLE = "idle"
    PLANNING = "planning"
    SEARCHING_VEHICLES = "searching_vehicles"
    VEHICLE_DETECTED = "vehicle_detected"
    NAVIGATING_TO_LICENSE_PLATE = "navigating_to_license_plate"
    CAPTURING_LICENSE_PLATE = "capturing_license_plate"
    SWITCHING_TO_INSPECTION = "switching_to_inspection"
    DETECTING_TYRES = "detecting_tyres"
    NAVIGATING_TO_TYRE = "navigating_to_tyre"
    CAPTURING_TYRE = "capturing_tyre"
    CHECKING_COMPLETION = "checking_completion"
    MISSION_COMPLETE = "mission_complete"
    ERROR_RECOVERY = "error_recovery"


class TyreData:
    """
    Data structure for tracking individual tyre inspection progress.
    
    Attributes:
        tyre_id (str): Unique identifier for this tyre
        bbox_3d (BoundingBox3d): 3D bounding box detection
        position_3d (Point): Calculated center position in camera frame
        frame_id (Optional[str]): Frame ID of detection
        photo_path (Optional[str]): Path to captured photo
        photo_taken (bool): Whether photo has been successfully captured
        attempts (int): Number of capture attempts
        max_attempts (int): Maximum allowed attempts before giving up
    
    Methods:
        _calculate_center: Calculate center point from 3D bounding box
        is_complete: Check if tyre inspection is complete
        reset_attempts: Reset attempt counter
    """
    
    def __init__(self, tyre_id: str, bbox_3d: BoundingBox3d, frame_id: Optional[str] = None):
        """
        Initialize tyre data structure.
        
        Args:
            tyre_id: Unique identifier for this tyre
            bbox_3d: 3D bounding box detection message
            frame_id: Optional frame ID of detection (for coordinate transforms)
        """
        if not tyre_id:
            raise ValueError("tyre_id cannot be empty")
        if bbox_3d is None:
            raise ValueError("bbox_3d cannot be None")
        
        self.tyre_id: str = tyre_id
        self.bbox_3d: BoundingBox3d = bbox_3d
        self.position_3d: Point = self._calculate_center(bbox_3d)
        self.frame_id: Optional[str] = frame_id
        self.photo_path: Optional[str] = None
        self.photo_taken: bool = False
        self.attempts: int = 0
        self.max_attempts: int = 3
        self.detection_timestamp: float = time.time()
    
    def _calculate_center(self, bbox: BoundingBox3d) -> Point:
        """
        Calculate center point of 3D bounding box.
        
        Args:
            bbox: 3D bounding box message
            
        Returns:
            Point with center coordinates (x, y, z)
        """
        return Point(
            x=float((bbox.xmin + bbox.xmax) / 2.0),
            y=float((bbox.ymin + bbox.ymax) / 2.0),
            z=float((bbox.zmin + bbox.zmax) / 2.0)
        )
    
    def is_complete(self) -> bool:
        """
        Check if tyre inspection is complete.
        
        Returns:
            True if photo has been taken, False otherwise
        """
        return self.photo_taken
    
    def reset_attempts(self) -> None:
        """Reset attempt counter to zero."""
        self.attempts = 0
    
    def increment_attempts(self) -> int:
        """
        Increment attempt counter.
        
        Returns:
            New attempt count
        """
        self.attempts += 1
        return self.attempts
    
    def has_attempts_remaining(self) -> bool:
        """
        Check if more attempts are allowed.
        
        Returns:
            True if attempts < max_attempts, False otherwise
        """
        return self.attempts < self.max_attempts
    
    def __repr__(self) -> str:
        """String representation for debugging."""
        return (f"TyreData(id={self.tyre_id}, photo_taken={self.photo_taken}, "
                f"attempts={self.attempts}/{self.max_attempts})")


class VehicleData:
    """
    Data structure for tracking vehicle inspection progress.
    
    This class manages all aspects of a vehicle's inspection:
    - Detection and tracking (camera + LiDAR fusion)
    - License plate capture and OCR
    - Tyre detection and photography
    - Navigation waypoint planning
    
    Attributes:
        vehicle_id (str): Unique identifier for this vehicle
        vehicle_type (str): Type of vehicle ('truck' or 'car')
        detection_pose (PoseStamped): Initial detection pose in navigation frame
        last_detection_pose (PoseStamped): Last known position from camera
        last_detection_time (float): Timestamp of last camera detection
        last_lidar_detection_time (float): Timestamp of last LiDAR detection
        lidar_position (Optional[PoseStamped]): Position from LiDAR fusion
        lidar_distance (Optional[float]): Distance measurement from LiDAR
        lidar_angle (Optional[float]): Angle measurement from LiDAR
        license_plate_photo_path (Optional[str]): Path to license plate photo
        license_plate_photo_taken (bool): Whether license plate photo captured
        license_plate_text (Optional[str]): OCR-detected license plate text
        license_plate_confidence (Optional[float]): OCR confidence (0.0-1.0)
        license_plate_position_3d (Optional[Point]): 3D position in camera frame
        license_plate_position_nav (Optional[PoseStamped]): 3D position in nav frame
        license_plate_bbox_2d (Optional[tuple]): 2D bounding box (x_min, y_min, x_max, y_max)
        tyres (List[TyreData]): List of detected tyres
        detection_timestamp (float): Initial detection timestamp
        navigation_started (bool): Whether navigation to this vehicle has started
        navigation_start_time (Optional[float]): Timestamp when navigation started
        license_plate_waypoint (Optional[PoseStamped]): Pre-planned license plate waypoint
        tyre_waypoints (List[PoseStamped]): Pre-planned tyre waypoints
        waypoints_planned (bool): Whether waypoints have been calculated
        completion_check_count (int): Number of completion verification attempts
    
    Methods:
        add_tyre: Add a detected tyre to this vehicle
        all_tyres_photographed: Check if all tyres have been photographed
        get_current_position: Get best estimate of current vehicle position
        update_detection: Update position from new camera detection
        update_lidar_position: Update position from LiDAR detection
        is_license_plate_complete: Check if license plate photo taken
        is_inspection_complete: Check if entire vehicle inspection is complete
    """
    
    def __init__(self, vehicle_id: str, vehicle_type: str, detection_pose: PoseStamped):
        """
        Initialize vehicle data structure.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            vehicle_type: Type of vehicle ('truck' or 'car')
            detection_pose: Initial detection pose in navigation frame
            
        Raises:
            ValueError: If vehicle_id is empty, vehicle_type is invalid, or detection_pose is None
        """
        if not vehicle_id:
            raise ValueError("vehicle_id cannot be empty")
        if vehicle_type not in ['truck', 'car']:
            raise ValueError(f"vehicle_type must be 'truck' or 'car', got '{vehicle_type}'")
        if detection_pose is None:
            raise ValueError("detection_pose cannot be None")
        
        self.vehicle_id: str = vehicle_id
        self.vehicle_type: str = vehicle_type.lower()
        self.detection_pose: PoseStamped = detection_pose
        self.last_detection_pose: PoseStamped = detection_pose
        self.last_detection_time: float = time.time()
        self.last_lidar_detection_time: float = time.time()
        self.lidar_position: Optional[PoseStamped] = None
        self.lidar_distance: Optional[float] = None
        self.lidar_angle: Optional[float] = None
        
        # License plate data
        self.license_plate_photo_path: Optional[str] = None
        self.license_plate_photo_taken: bool = False
        self.license_plate_text: Optional[str] = None
        self.license_plate_confidence: Optional[float] = None
        self.license_plate_position_3d: Optional[Point] = None
        self.license_plate_position_nav: Optional[PoseStamped] = None
        self.license_plate_bbox_2d: Optional[tuple] = None
        
        # NEW: Direct license plate detection (from YOLO)
        self.license_plate_pose: Optional[PoseStamped] = None  # Actual detected position in nav frame
        self.license_plate_detected: bool = False  # Was license plate detected directly?
        
        # Tyre data
        self.tyres: List[TyreData] = []
        self.detection_timestamp: float = time.time()
        
        # Navigation tracking
        self.navigation_started: bool = False
        self.navigation_start_time: Optional[float] = None
        
        # Pre-planned waypoints (calculated before navigation starts)
        self.license_plate_waypoint: Optional[PoseStamped] = None
        self.tyre_waypoints: List[PoseStamped] = []
        self.waypoints_planned: bool = False
        
        # Completion checking
        self.completion_check_count: int = 0
    
    def add_tyre(self, tyre_id: str, bbox_3d: BoundingBox3d, frame_id: Optional[str] = None) -> TyreData:
        """
        Add a detected tyre to this vehicle.
        
        Args:
            tyre_id: Unique identifier for the tyre
            bbox_3d: 3D bounding box detection
            frame_id: Optional frame ID of detection
            
        Returns:
            TyreData object that was added
            
        Raises:
            ValueError: If tyre with same ID already exists
        """
        # Check for duplicate tyre IDs
        existing_ids = [tyre.tyre_id for tyre in self.tyres]
        if tyre_id in existing_ids:
            raise ValueError(f"Tyre with ID '{tyre_id}' already exists for vehicle '{self.vehicle_id}'")
        
        tyre = TyreData(tyre_id, bbox_3d, frame_id)
        self.tyres.append(tyre)
        return tyre
    
    def all_tyres_photographed(self) -> bool:
        """
        Check if all tyres have been photographed.
        
        Returns:
            True if all tyres have photo_taken=True, False otherwise
            Returns False if no tyres detected (inspection incomplete)
        """
        if not self.tyres:
            return False
        return all(tyre.photo_taken for tyre in self.tyres)
    
    def get_current_position(self, camera_timeout: float = 10.0, lidar_timeout: float = 10.0) -> Optional[PoseStamped]:
        """
        Get current best estimate of vehicle position using sensor fusion.
        
        Priority order:
        1. Recent camera detection (< camera_timeout seconds)
        2. Recent LiDAR detection (< lidar_timeout seconds)
        3. Last known camera position (fallback)
        4. None if no position available
        
        Args:
            camera_timeout: Maximum age of camera detection to consider "recent" (seconds)
            lidar_timeout: Maximum age of LiDAR detection to consider "recent" (seconds)
            
        Returns:
            PoseStamped in navigation frame, or None if no valid position available
        """
        current_time = time.time()
        
        # Priority 1: Recent camera detection (most accurate)
        if self.last_detection_pose and (current_time - self.last_detection_time) < camera_timeout:
            return self.last_detection_pose
        
        # Priority 2: Recent LiDAR detection (backup when camera loses view)
        if self.lidar_position and (current_time - self.last_lidar_detection_time) < lidar_timeout:
            return self.lidar_position
        
        # Priority 3: Last known camera position (fallback for short-term loss)
        if self.last_detection_pose:
            return self.last_detection_pose
        
        # No valid position available
        return None
    
    def update_detection(self, detection_pose: PoseStamped) -> None:
        """
        Update vehicle position from new camera detection.
        
        Args:
            detection_pose: New detection pose in navigation frame
            
        Raises:
            ValueError: If detection_pose is None
        """
        if detection_pose is None:
            raise ValueError("detection_pose cannot be None")
        
        self.last_detection_pose = detection_pose
        self.last_detection_time = time.time()
        # Optionally update main detection_pose if significant movement detected
        # (for now, keep original detection_pose for waypoint planning stability)
    
    def update_lidar_position(self, position: PoseStamped, distance: Optional[float] = None, 
                             angle: Optional[float] = None) -> None:
        """
        Update vehicle position from LiDAR detection.
        
        Args:
            position: LiDAR position in navigation frame
            distance: Optional distance measurement from LiDAR
            angle: Optional angle measurement from LiDAR
            
        Raises:
            ValueError: If position is None
        """
        if position is None:
            raise ValueError("position cannot be None")
        
        self.lidar_position = position
        self.last_lidar_detection_time = time.time()
        
        if distance is not None:
            self.lidar_distance = float(distance)
        if angle is not None:
            self.lidar_angle = float(angle)
    
    def is_license_plate_complete(self) -> bool:
        """
        Check if license plate photo has been captured.
        
        Returns:
            True if license plate photo taken, False otherwise
        """
        return self.license_plate_photo_taken
    
    def is_inspection_complete(self) -> bool:
        """
        Check if entire vehicle inspection is complete.
        
        Complete means:
        - License plate photo captured
        - All detected tyres photographed
        
        Returns:
            True if inspection complete, False otherwise
        """
        return self.is_license_plate_complete() and self.all_tyres_photographed()
    
    def get_tyre_count(self) -> int:
        """Get number of tyres detected for this vehicle."""
        return len(self.tyres)
    
    def get_photographed_tyre_count(self) -> int:
        """Get number of tyres that have been photographed."""
        return sum(1 for tyre in self.tyres if tyre.photo_taken)
    
    def __repr__(self) -> str:
        """String representation for debugging."""
        return (f"VehicleData(id={self.vehicle_id}, type={self.vehicle_type}, "
                f"license_plate={self.license_plate_photo_taken}, "
                f"tyres={self.get_photographed_tyre_count()}/{self.get_tyre_count()})")

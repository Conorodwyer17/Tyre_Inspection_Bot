#!/usr/bin/env python3
"""
Configuration Module

Centralized configuration for all constants, default parameters, and thresholds
used throughout the tyre inspection mission package.

All configuration values are grouped by category and fully documented.
Values can be overridden by ROS 2 parameters at runtime.
"""

from typing import List, Dict, Any
from dataclasses import dataclass


# ============================================================================
# Navigation Configuration
# ============================================================================

class NavigationConfig:
    """Navigation safety parameters and thresholds."""
    
    # Robot physical dimensions
    ROBOT_RADIUS: float = 0.1  # meters - Robot footprint radius
    INFLATION_RADIUS: float = 0.35  # meters - Costmap inflation radius
    SAFETY_MARGIN: float = 0.15  # meters - Additional safety margin
    
    # Calculated minimum safe distance
    MIN_SAFE_DISTANCE: float = ROBOT_RADIUS + INFLATION_RADIUS + SAFETY_MARGIN  # 0.6m
    
    # Goal arrival thresholds
    GOAL_ARRIVAL_DISTANCE: float = 0.4  # meters - Consider goal reached if within this
    GOAL_UPDATE_DISTANCE: float = 0.5  # meters - Recalculate goal if robot gets this close
    
    # Navigation timeouts
    NAVIGATION_TIMEOUT: float = 60.0  # seconds - Maximum time for single navigation
    GOAL_REJECTION_RETRY_LIMIT: int = 3  # Maximum retries for rejected goals
    
    # Approach distances
    DEFAULT_APPROACH_DISTANCE: float = 1.0  # meters - Default distance from target
    TOO_CLOSE_BACKUP_DISTANCE: float = 0.3  # meters - How far to back up if too close
    
    # Goal precision
    GOAL_POSITION_PRECISION: float = 0.05  # meters - Round goals to 5cm


# ============================================================================
# Detection Configuration
# ============================================================================

class DetectionConfig:
    """Vehicle and tyre detection parameters."""
    
    # Vehicle classes to detect
    DEFAULT_VEHICLE_CLASSES: List[str] = ['truck', 'car']
    
    # Tyre class name
    TYRE_CLASS_NAME: str = 'tyre'
    
    # Detection timeouts
    DETECTION_TIMEOUT: float = 60.0  # seconds - Maximum time to search for vehicles
    TYRE_DETECTION_TIMEOUT: float = 30.0  # seconds - Maximum time to detect tyres
    
    # LiDAR tracking parameters
    LIDAR_VEHICLE_MIN_SIZE: float = 1.5  # meters - Minimum vehicle size in LiDAR
    LIDAR_VEHICLE_MAX_SIZE: float = 6.0  # meters - Maximum vehicle size in LiDAR
    LIDAR_TRACKING_DISTANCE_THRESHOLD: float = 1.0  # meters - Max distance to match detections
    
    # Sensor fusion timeouts
    CAMERA_DETECTION_TIMEOUT: float = 10.0  # seconds - Camera detection considered stale after this
    LIDAR_DETECTION_TIMEOUT: float = 10.0  # seconds - LiDAR detection considered stale after this


# ============================================================================
# Camera Configuration
# ============================================================================

class CameraConfig:
    """Camera and image processing parameters."""
    
    # Image dimensions (640x480 standard)
    IMAGE_WIDTH: int = 640  # pixels
    IMAGE_HEIGHT: int = 480  # pixels
    IMAGE_CENTER_X: int = 320  # pixels (IMAGE_WIDTH / 2)
    IMAGE_CENTER_Y: int = 240  # pixels (IMAGE_HEIGHT / 2)
    
    # Camera intrinsics (from config file)
    # fx, fy: Focal lengths in pixels
    # cx, cy: Principal point (optical center)
    CAMERA_FX: float = 289.11  # pixels
    CAMERA_FY: float = 289.75  # pixels
    CAMERA_CX: float = 347.24  # pixels
    CAMERA_CY: float = 235.67  # pixels
    
    # Tyre centering parameters
    TYRE_CENTERING_TOLERANCE_PX: int = 50  # pixels - Centering tolerance
    MAX_CENTERING_ATTEMPTS: int = 3  # Maximum centering adjustment attempts
    CENTERING_TIMEOUT: float = 10.0  # seconds - Maximum time for centering
    
    # Centering adjustment factors (pixels to motion conversion)
    PIXEL_TO_YAW_FACTOR: float = 0.0016  # radians per pixel (at ~1m distance)
    PIXEL_TO_LINEAR_FACTOR: float = 0.0005  # meters per pixel (at ~1m distance)
    MAX_YAW_ADJUSTMENT: float = 0.1  # radians - Maximum yaw adjustment per step
    MAX_LINEAR_ADJUSTMENT: float = 0.1  # meters - Maximum linear adjustment per step


# ============================================================================
# Capture Configuration
# ============================================================================

class CaptureConfig:
    """Photo capture parameters."""
    
    # Photo capture settings
    PHOTO_CAPTURE_TIMEOUT: float = 10.0  # seconds - Maximum time for photo capture
    IMAGE_FORMAT: str = 'jpg'  # Image format for saved photos
    IMAGE_QUALITY: int = 95  # JPEG quality (0-100)
    
    # Retry parameters
    MAX_PHOTO_ATTEMPTS: int = 3  # Maximum attempts to capture a photo
    RETRY_DELAY: float = 1.0  # seconds - Delay between retries
    
    # Storage
    DEFAULT_STORAGE_DIR: str = '~/tyre_inspection_photos'  # Default photo storage directory


# ============================================================================
# Planning Configuration
# ============================================================================

class PlanningConfig:
    """Waypoint planning parameters."""
    
    # Planning timeouts
    PLANNING_TIMEOUT: float = 30.0  # seconds - Maximum time for planning phase
    
    # Fallback waypoint generation
    DEFAULT_VEHICLE_WIDTH: float = 2.0  # meters - Default vehicle width (truck)
    DEFAULT_CAR_WIDTH: float = 1.5  # meters - Default car width
    DEFAULT_VEHICLE_LENGTH: float = 6.0  # meters - Default vehicle length (truck)
    DEFAULT_CAR_LENGTH: float = 4.5  # meters - Default car length
    TYRE_OFFSET_FROM_EDGE: float = 0.3  # meters - Tyre distance from vehicle edge
    
    # Expected tyre positions (relative to vehicle center, meters)
    # Format: (x_offset, y_offset) for each tyre
    # Order: front-left, front-right, rear-left, rear-right
    TYRE_POSITIONS_RELATIVE: List[tuple] = [
        (-2.7, -0.7),   # Front-left  (half_length - offset, -half_width + offset)
        (-2.7, 0.7),    # Front-right (half_length - offset, half_width - offset)
        (2.7, -0.7),    # Rear-left   (half_length - offset, -half_width + offset)
        (2.7, 0.7),     # Rear-right  (half_length - offset, half_width - offset)
    ]


# ============================================================================
# Completion Checking Configuration
# ============================================================================

class CompletionConfig:
    """Vehicle inspection completion verification parameters."""
    
    # Completion check parameters
    MAX_COMPLETION_CHECKS: int = 3  # Maximum completion verification attempts
    COMPLETION_CHECK_TIMEOUT: float = 30.0  # seconds - Timeout for completion check


# ============================================================================
# System Health Configuration
# ============================================================================

class SystemHealthConfig:
    """System health monitoring parameters."""
    
    # Health check intervals
    SYSTEM_HEALTH_CHECK_INTERVAL: float = 5.0  # seconds
    DETECTION_HEALTH_CHECK_INTERVAL: float = 10.0  # seconds
    STATS_LOG_INTERVAL: float = 30.0  # seconds
    
    # Health check timeouts
    DETECTION_MESSAGE_TIMEOUT: float = 10.0  # seconds - Warning if no messages for this long
    BBOX_MESSAGE_TIMEOUT: float = 10.0  # seconds - Warning if no bbox messages for this long


# ============================================================================
# Logging Configuration
# ============================================================================

class LoggingConfig:
    """Structured logging configuration."""
    
    # Debug logging
    DEFAULT_ENABLE_DEBUG: bool = False  # Enable debug logs by default
    
    # Log throttling
    EMPTY_MESSAGE_LOG_INTERVAL: float = 30.0  # seconds - Log empty messages every N seconds


# ============================================================================
# Topic Names Configuration
# ============================================================================

class TopicConfig:
    """ROS 2 topic names."""
    
    # Input topics
    BOUNDING_BOXES_TOPIC: str = '/darknet_ros_3d/bounding_boxes'
    CAMERA_TOPIC: str = '/oak/rgb/image_rect'
    POINT_CLOUD_TOPIC: str = '/points'
    LIDAR_SCAN_TOPIC: str = '/scan'
    
    # Output topics
    SEGMENTATION_MODE_TOPIC: str = '/segmentation_mode'
    MISSION_STATUS_TOPIC: str = '/mission_controller/status'
    GOAL_MARKERS_TOPIC: str = '/mission_controller/goal_markers'
    LED_CONTROL_TOPIC: str = '/ugv/led_ctrl'
    
    # Service topics
    PHOTO_CAPTURE_SERVICE: str = '/photo_capture/capture'
    MISSION_START_SERVICE: str = '/mission_controller/start'
    MISSION_STOP_SERVICE: str = '/mission_controller/stop'
    
    # Action topics
    NAVIGATE_TO_POSE_ACTION: str = '/navigate_to_pose'


# ============================================================================
# Frame Names Configuration
# ============================================================================

class FrameConfig:
    """TF2 frame names."""
    
    # Navigation frames
    DEFAULT_NAV_FRAME: str = 'map'  # Default navigation frame (map or odom)
    ODOM_FRAME: str = 'odom'  # Odometry frame (fallback)
    
    # Robot frames
    BASE_FOOTPRINT_FRAME: str = 'base_footprint'  # Robot base frame
    
    # Camera frames
    DEFAULT_CAMERA_FRAME: str = 'oak_rgb_camera_optical_frame'  # Default camera frame


# ============================================================================
# Configuration Access Functions
# ============================================================================

def get_all_config_dict() -> Dict[str, Any]:
    """
    Get all configuration values as a dictionary.
    
    Useful for parameter validation and logging.
    
    Returns:
        Dictionary of all configuration values grouped by category
    """
    return {
        'navigation': {
            'robot_radius': NavigationConfig.ROBOT_RADIUS,
            'inflation_radius': NavigationConfig.INFLATION_RADIUS,
            'safety_margin': NavigationConfig.SAFETY_MARGIN,
            'min_safe_distance': NavigationConfig.MIN_SAFE_DISTANCE,
            'goal_arrival_distance': NavigationConfig.GOAL_ARRIVAL_DISTANCE,
            'goal_update_distance': NavigationConfig.GOAL_UPDATE_DISTANCE,
            'navigation_timeout': NavigationConfig.NAVIGATION_TIMEOUT,
            'approach_distance': NavigationConfig.DEFAULT_APPROACH_DISTANCE,
        },
        'detection': {
            'vehicle_classes': DetectionConfig.DEFAULT_VEHICLE_CLASSES,
            'tyre_class_name': DetectionConfig.TYRE_CLASS_NAME,
            'detection_timeout': DetectionConfig.DETECTION_TIMEOUT,
            'tyre_detection_timeout': DetectionConfig.TYRE_DETECTION_TIMEOUT,
        },
        'camera': {
            'image_width': CameraConfig.IMAGE_WIDTH,
            'image_height': CameraConfig.IMAGE_HEIGHT,
            'image_center_x': CameraConfig.IMAGE_CENTER_X,
            'image_center_y': CameraConfig.IMAGE_CENTER_Y,
            'centering_tolerance_px': CameraConfig.TYRE_CENTERING_TOLERANCE_PX,
        },
        'capture': {
            'photo_capture_timeout': CaptureConfig.PHOTO_CAPTURE_TIMEOUT,
            'max_photo_attempts': CaptureConfig.MAX_PHOTO_ATTEMPTS,
            'image_format': CaptureConfig.IMAGE_FORMAT,
            'image_quality': CaptureConfig.IMAGE_QUALITY,
        },
        'planning': {
            'planning_timeout': PlanningConfig.PLANNING_TIMEOUT,
            'default_vehicle_width': PlanningConfig.DEFAULT_VEHICLE_WIDTH,
            'default_vehicle_length': PlanningConfig.DEFAULT_VEHICLE_LENGTH,
        },
    }

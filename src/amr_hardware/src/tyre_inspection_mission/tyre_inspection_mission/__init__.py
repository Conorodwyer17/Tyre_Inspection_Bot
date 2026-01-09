"""
Tyre Inspection Mission Package

This package provides autonomous tyre inspection capabilities for vehicles
in a yard environment. It includes:

Core Components:
    - Mission Controller: Orchestrates the complete inspection mission
    - Photo Capture Service: Captures and stores vehicle and tyre photos
    - Vehicle Tracker: Tracks vehicles using camera and LiDAR fusion
    - Goal Planner: Calculates and validates navigation goals
    - Navigation Manager: Manages Nav2 interactions
    - License Plate Detector: OCR-based license plate detection

Common Utilities:
    - data_structures: Shared data structures (VehicleData, TyreData, MissionState)
    - utils: Utility functions (transformations, distance calculations)
    - config: Centralized configuration constants
    - exceptions: Custom exception classes
    - structured_logger: Structured logging system
    - diagnostics: System health monitoring and error recovery helpers

Module Organization:
    - Core: Mission orchestration and state management
    - Detection: Object detection and tracking
    - Navigation: Navigation goal planning and execution
    - Capture: Photo capture and storage
    - Common: Shared utilities and data structures
    - Diagnostics: System health and error recovery

For troubleshooting, see: docs/troubleshooting/TROUBLESHOOTING_GUIDE.md
For architecture, see: docs/architecture/MODULE_ORGANIZATION.md
For error recovery, see: docs/architecture/ERROR_RECOVERY_STRATEGY.md
"""

# Export shared modules for easy import (re-export from common)
from tyre_inspection_mission.common.data_structures import (
    MissionState,
    VehicleData,
    TyreData
)

from tyre_inspection_mission.common.utils import (
    yaw_to_quaternion,
    quaternion_to_yaw,
    calculate_distance_2d,
    calculate_distance_3d,
    calculate_pose_distance,
    validate_pose,
    get_robot_pose,
    transform_pose,
    is_within_tolerance,
    clamp
)

from tyre_inspection_mission.common.config import (
    NavigationConfig,
    DetectionConfig,
    CameraConfig,
    CaptureConfig,
    PlanningConfig,
    CompletionConfig,
    SystemHealthConfig,
    LoggingConfig,
    TopicConfig,
    FrameConfig,
    get_all_config_dict
)

from tyre_inspection_mission.common.exceptions import (
    TyreInspectionError,
    NavigationError,
    DetectionError,
    CaptureError,
    PlanningError,
    TransformError,
    ValidationError,
    StateMachineError
)

__all__ = [
    # Data structures
    'MissionState',
    'VehicleData',
    'TyreData',
    # Utilities
    'yaw_to_quaternion',
    'quaternion_to_yaw',
    'calculate_distance_2d',
    'calculate_distance_3d',
    'calculate_pose_distance',
    'validate_pose',
    'get_robot_pose',
    'transform_pose',
    'is_within_tolerance',
    'clamp',
    # Configuration
    'NavigationConfig',
    'DetectionConfig',
    'CameraConfig',
    'CaptureConfig',
    'PlanningConfig',
    'CompletionConfig',
    'SystemHealthConfig',
    'LoggingConfig',
    'TopicConfig',
    'FrameConfig',
    'get_all_config_dict',
    # Exceptions
    'TyreInspectionError',
    'NavigationError',
    'DetectionError',
    'CaptureError',
    'PlanningError',
    'TransformError',
    'ValidationError',
    'StateMachineError',
    # Diagnostics (optional - may not be available in all builds)
    # 'SystemDiagnostics',
    # 'ErrorRecoveryHelper',
    # 'HealthStatus',
]

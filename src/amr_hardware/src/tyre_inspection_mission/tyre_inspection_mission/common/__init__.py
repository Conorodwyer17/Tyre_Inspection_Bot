"""
Common Utilities

Shared utilities, data structures, configuration, exceptions, and logging.
"""

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

from tyre_inspection_mission.common.structured_logger import (
    StructuredLogger,
    LogCategory
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
    # Logging
    'StructuredLogger',
    'LogCategory',
]

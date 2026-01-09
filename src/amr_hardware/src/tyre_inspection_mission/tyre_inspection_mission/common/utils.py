#!/usr/bin/env python3
"""
Utilities Module

Shared utility functions for coordinate transformations, distance calculations,
and other common operations used across the tyre inspection mission package.

All functions include comprehensive type hints, error handling, and documentation.
"""

import math
from typing import Optional, Tuple
from geometry_msgs.msg import Quaternion, Point, PoseStamped
import rclpy
import tf2_ros


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Convert yaw angle (radians) to ROS 2 Quaternion.
    
    Args:
        yaw: Yaw angle in radians (rotation around Z-axis)
        
    Returns:
        Quaternion message with equivalent rotation
        
    Note:
        Assumes pitch=0 and roll=0 (2D rotation only)
    """
    if not math.isfinite(yaw):
        raise ValueError(f"yaw must be finite, got {yaw}")
    
    return Quaternion(
        x=0.0,
        y=0.0,
        z=float(math.sin(yaw / 2.0)),
        w=float(math.cos(yaw / 2.0))
    )


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    """
    Convert ROS 2 Quaternion to yaw angle (radians).
    
    Args:
        quaternion: Quaternion message
        
    Returns:
        Yaw angle in radians (rotation around Z-axis)
        
    Note:
        Assumes pitch=0 and roll=0 (2D rotation only)
    """
    if quaternion is None:
        raise ValueError("quaternion cannot be None")
    
    # Normalize if needed
    norm = math.sqrt(quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2)
    if norm > 0:
        qz = quaternion.z / norm
        qw = quaternion.w / norm
    else:
        qz = quaternion.z
        qw = quaternion.w
    
    yaw = 2.0 * math.atan2(qz, qw)
    
    # Normalize to [-pi, pi]
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    
    return float(yaw)


def calculate_distance_2d(point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
    """
    Calculate 2D Euclidean distance between two points.
    
    Args:
        point1: (x, y) coordinates of first point
        point2: (x, y) coordinates of second point
        
    Returns:
        Distance in meters
        
    Raises:
        ValueError: If points are invalid
    """
    if point1 is None or point2 is None:
        raise ValueError("Points cannot be None")
    if len(point1) != 2 or len(point2) != 2:
        raise ValueError("Points must be (x, y) tuples")
    
    x1, y1 = point1
    x2, y2 = point2
    
    if not all(math.isfinite(v) for v in [x1, y1, x2, y2]):
        raise ValueError("Point coordinates must be finite")
    
    dx = x2 - x1
    dy = y2 - y1
    distance = math.sqrt(dx * dx + dy * dy)
    
    return float(distance)


def calculate_distance_3d(point1: Tuple[float, float, float], 
                         point2: Tuple[float, float, float]) -> float:
    """
    Calculate 3D Euclidean distance between two points.
    
    Args:
        point1: (x, y, z) coordinates of first point
        point2: (x, y, z) coordinates of second point
        
    Returns:
        Distance in meters
        
    Raises:
        ValueError: If points are invalid
    """
    if point1 is None or point2 is None:
        raise ValueError("Points cannot be None")
    if len(point1) != 3 or len(point2) != 3:
        raise ValueError("Points must be (x, y, z) tuples")
    
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    
    if not all(math.isfinite(v) for v in [x1, y1, z1, x2, y2, z2]):
        raise ValueError("Point coordinates must be finite")
    
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    
    return float(distance)


def calculate_pose_distance(pose1: PoseStamped, pose2: PoseStamped) -> Optional[float]:
    """
    Calculate 2D distance between two poses (ignoring orientation and Z).
    
    Args:
        pose1: First pose
        pose2: Second pose
        
    Returns:
        Distance in meters, or None if poses are in different frames
        
    Raises:
        ValueError: If poses are invalid
    """
    if pose1 is None or pose2 is None:
        raise ValueError("Poses cannot be None")
    
    if pose1.header.frame_id != pose2.header.frame_id:
        # Cannot calculate distance between different frames
        return None
    
    pos1 = pose1.pose.position
    pos2 = pose2.pose.position
    
    return calculate_distance_2d((pos1.x, pos1.y), (pos2.x, pos2.y))


def validate_pose(pose: PoseStamped, check_finite: bool = True) -> bool:
    """
    Validate that a pose contains valid values.
    
    Args:
        pose: PoseStamped to validate
        check_finite: If True, check that all values are finite
        
    Returns:
        True if pose is valid, False otherwise
    """
    if pose is None:
        return False
    
    if not pose.header.frame_id:
        return False
    
    pos = pose.pose.position
    orient = pose.pose.orientation
    
    if check_finite:
        values = [pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w]
        if not all(math.isfinite(v) for v in values):
            return False
    
    return True


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range.
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-pi, pi]
    """
    if not math.isfinite(angle):
        raise ValueError(f"angle must be finite, got {angle}")
    
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    
    return float(angle)


def angle_difference(angle1: float, angle2: float) -> float:
    """
    Calculate shortest angular difference between two angles.
    
    Args:
        angle1: First angle in radians
        angle2: Second angle in radians
        
    Returns:
        Difference in radians, normalized to [-pi, pi]
    """
    diff = normalize_angle(angle1 - angle2)
    return diff


def get_robot_pose(tf_buffer: tf2_ros.Buffer, target_frame: str = 'map', 
                  source_frame: str = 'base_footprint', 
                  timeout_sec: float = 1.0) -> Optional[PoseStamped]:
    """
    Get current robot pose using TF2.
    
    Args:
        tf_buffer: TF2 buffer for coordinate transformations
        target_frame: Target frame (e.g., 'map' or 'odom')
        source_frame: Source frame (e.g., 'base_footprint')
        timeout_sec: Maximum time to wait for transform (seconds)
        
    Returns:
        PoseStamped in target frame, or None if transform fails
    """
    if tf_buffer is None:
        return None
    
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(seconds=0),
            timeout=rclpy.duration.Duration(seconds=timeout_sec)
        )
        
        # Convert TransformStamped to PoseStamped
        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = target_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        
        return pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
            tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
        # Transform failed - return None
        return None
    except Exception as e:
        # Unexpected error
        return None


def transform_pose(pose: PoseStamped, target_frame: str, tf_buffer: tf2_ros.Buffer,
                  timeout_sec: float = 2.0) -> Optional[PoseStamped]:
    """
    Transform a pose from its current frame to target frame.
    
    Args:
        pose: PoseStamped to transform
        target_frame: Target frame ID
        tf_buffer: TF2 buffer for coordinate transformations
        timeout_sec: Maximum time to wait for transform (seconds)
        
    Returns:
        Transformed PoseStamped in target frame, or None if transform fails
    """
    if pose is None or tf_buffer is None:
        return None
    
    if pose.header.frame_id == target_frame:
        # Already in target frame
        return pose
    
    try:
        from tf2_geometry_msgs import do_transform_pose_stamped
        
        transform = tf_buffer.lookup_transform(
            target_frame,
            pose.header.frame_id,
            rclpy.time.Time(seconds=0),
            timeout=rclpy.duration.Duration(seconds=timeout_sec)
        )
        
        transformed_pose = do_transform_pose_stamped(pose, transform)
        transformed_pose.header.frame_id = target_frame
        transformed_pose.header.stamp = transform.header.stamp
        
        return transformed_pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
            tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
        # Transform failed - return None
        return None
    except Exception as e:
        # Unexpected error
        return None


def is_within_tolerance(value: float, target: float, tolerance: float) -> bool:
    """
    Check if value is within tolerance of target.
    
    Args:
        value: Value to check
        target: Target value
        tolerance: Absolute tolerance
        
    Returns:
        True if |value - target| <= tolerance, False otherwise
    """
    if not math.isfinite(value) or not math.isfinite(target) or not math.isfinite(tolerance):
        return False
    
    return abs(value - target) <= abs(tolerance)


def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    Clamp value to [min_value, max_value] range.
    
    Args:
        value: Value to clamp
        min_value: Minimum allowed value
        max_value: Maximum allowed value
        
    Returns:
        Clamped value
        
    Raises:
        ValueError: If min_value > max_value
    """
    if not math.isfinite(value):
        raise ValueError(f"value must be finite, got {value}")
    if not math.isfinite(min_value) or not math.isfinite(max_value):
        raise ValueError("min_value and max_value must be finite")
    if min_value > max_value:
        raise ValueError(f"min_value ({min_value}) must be <= max_value ({max_value})")
    
    return float(max(min_value, min(value, max_value)))

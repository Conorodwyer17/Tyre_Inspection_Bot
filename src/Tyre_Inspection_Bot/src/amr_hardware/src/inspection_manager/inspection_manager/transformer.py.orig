import time
from typing import Optional

import rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException


def lookup_transform(tf_buffer, target_frame: str, source_frame: str, timeout_s: float = 0.2):
    """Lookup a transform with a bounded timeout. Returns TransformStamped or None."""
    try:
        return tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=float(timeout_s)),
        )
    except TransformException:
        return None


def get_current_pose(
    tf_buffer,
    world_frame: str,
    base_frame: str,
    timeout_s: float = 0.2,
    logger=None,
) -> Optional[PoseStamped]:
    """Get current robot pose in world_frame. Returns PoseStamped or None."""
    transform = lookup_transform(tf_buffer, world_frame, base_frame, timeout_s=timeout_s)
    if transform is None:
        if logger:
            logger.warn(f"TF lookup failed: {world_frame}->{base_frame}")
        return None
    pose = PoseStamped()
    pose.header.frame_id = world_frame
    pose.header.stamp = transform.header.stamp
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


def transform_pose(
    tf_buffer,
    pose: PoseStamped,
    target_frame: str,
    timeout_s: float = 0.2,
    logger=None,
) -> Optional[PoseStamped]:
    """Transform PoseStamped to target_frame. Returns transformed pose or None."""
    transform = lookup_transform(tf_buffer, target_frame, pose.header.frame_id, timeout_s=timeout_s)
    if transform is None:
        if logger:
            logger.warn(f"TF lookup failed: {target_frame}->{pose.header.frame_id}")
        return None
    try:
        return tf2_geometry_msgs.do_transform_pose_stamped(pose, transform)
    except Exception as e:
        if logger:
            logger.warn(f"TF transform failed: {e}")
        return None


def monotonic_time_s() -> float:
    """Return monotonic time in seconds (stable for timeout comparisons)."""
    return time.monotonic()

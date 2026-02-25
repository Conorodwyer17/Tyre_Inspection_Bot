import math
from geometry_msgs.msg import Quaternion


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a geometry_msgs/Quaternion from a yaw angle (Z-axis rotation)."""
    half_yaw = yaw * 0.5
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    # roll = pitch = 0, so x = y = 0
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw angle from a geometry_msgs/Quaternion."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

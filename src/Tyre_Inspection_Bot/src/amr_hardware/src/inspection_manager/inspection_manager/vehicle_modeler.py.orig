import math
from typing import List, Tuple, Optional


def estimate_tire_positions(
    vehicle_center: Tuple[float, float, float],
    robot_pos: Optional[Tuple[float, float, float]],
    wheelbase_m: float = 2.7,
    track_m: float = 1.6,
) -> List[Tuple[float, float, float]]:
    """Estimate 4 tire positions around a vehicle center.

    This is a simple geometric model: we assume the vehicle longitudinal axis
    is aligned with the vector from vehicle center to robot (the side the robot sees).
    """
    vx, vy, vz = vehicle_center
    if robot_pos is None:
        return []
    rx, ry, _ = robot_pos
    dx = vx - rx
    dy = vy - ry
    dist = math.hypot(dx, dy)
    if dist < 0.01:
        return []
    fwd_x = dx / dist
    fwd_y = dy / dist
    right_x = -fwd_y
    right_y = fwd_x
    half_wb = wheelbase_m * 0.5
    half_tr = track_m * 0.5
    # FL, FR, RL, RR order
    return [
        (vx + fwd_x * half_wb - right_x * half_tr, vy + fwd_y * half_wb - right_y * half_tr, vz),
        (vx + fwd_x * half_wb + right_x * half_tr, vy + fwd_y * half_wb + right_y * half_tr, vz),
        (vx - fwd_x * half_wb - right_x * half_tr, vy - fwd_y * half_wb - right_y * half_tr, vz),
        (vx - fwd_x * half_wb + right_x * half_tr, vy - fwd_y * half_wb + right_y * half_tr, vz),
    ]

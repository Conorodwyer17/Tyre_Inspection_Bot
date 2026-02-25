from inspection_manager.vehicle_modeler import estimate_tire_positions


def test_estimate_tire_positions_returns_four():
    vehicle_center = (5.0, 2.0, 0.0)
    robot_pos = (0.0, 0.0, 0.0)
    positions = estimate_tire_positions(vehicle_center, robot_pos, wheelbase_m=2.0, track_m=1.0)
    assert len(positions) == 4


def test_estimate_tire_positions_requires_robot_pose():
    vehicle_center = (1.0, 1.0, 0.0)
    assert estimate_tire_positions(vehicle_center, None) == []

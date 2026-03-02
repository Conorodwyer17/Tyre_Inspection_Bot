# Industrial Refinement Report

Research synthesis from nav2_waypoint_follower, QCar watchdog, velocity_smoother, unitree-go2.

## Patterns Extracted

- processAtWaypoint only on SUCCEEDED (nav2_waypoint_follower)
- cmd_vel timeout inject STOP (QCar qcar_watchdog_node)
- velocity_timeout zeros when no input (nav2_velocity_smoother)
- stop_on_failure: false for skip-unreachable (nav2_waypoint_follower)
- Mutex for image callback (PhotoAtWaypoint)

## Our Alignment

- Photo only after nav SUCCEEDED + should_trigger_photo distance check
- velocity_smoother velocity_timeout 1.0s
- stop_on_failure: false in nav_aurora
- Single-threaded; no concurrent writers
- TF fallback, pointcloud_max_age_s, hard_mission_timeout implemented

## Optional Additions

- cmd_vel watchdog 0.5s: scripts/cmd_vel_watchdog_node.py (optional)
- Collision Monitor: Nav2 node for emergency stop (slow robot; low priority)
- LifecycleNode for inspection_manager: large refactor

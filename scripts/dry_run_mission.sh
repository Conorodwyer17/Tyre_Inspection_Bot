#!/bin/bash
# Priority G: Run inspection mission with motors disabled (dry run).
# Launches stack with dry_run:=true so Nav2 goals are validated but not sent,
# or with a mock cmd_vel logger. Adjust launch targets to match your setup.

set -e
cd "${UGV_WS:-$HOME/ugv_ws}"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

echo "Dry run: launch Aurora + segment_3d + inspection_manager with dry_run:=true"
echo "Motors should be disabled or use a node that logs cmd_vel instead of driving."

# Example: if you have a single launch that brings inspection + nav:
# ros2 launch ugv_nav full_mission.launch.py dry_run:=true

# Or step by step:
# Terminal 1: ros2 launch ugv_nav aurora_testing.launch.py
# Terminal 2: ros2 launch segmentation_3d segment_3d.launch.py
# Terminal 3: ros2 run inspection_manager inspection_manager_node --ros-args -p dry_run:=true

# Placeholder: run inspection manager with dry_run (package/executable names may vary)
exec ros2 run inspection_manager inspection_manager_node --ros-args -p dry_run:=true

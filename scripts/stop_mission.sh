#!/bin/bash
# Stop all mission components cleanly
# Use after start_full_mission.sh to shut down Aurora, motor, Nav2, detection, inspection manager

echo "Stopping tire inspection mission components..."

# Kill by process pattern (more reliable than PIDs - catches child processes)
pkill -f "slamware_ros_sdk_server_node" 2>/dev/null && echo "  Stopped Aurora SDK" || true
pkill -f "motor_driver_node" 2>/dev/null && echo "  Stopped motor driver" || true
pkill -f "nav_aurora.launch" 2>/dev/null && echo "  Stopped Nav2 (nav_aurora)" || true
pkill -f "segment_3d.launch" 2>/dev/null && echo "  Stopped detection (segment_3d)" || true
pkill -f "segmentation_3d" 2>/dev/null && echo "  Stopped segmentation nodes" || true
pkill -f "inspection_manager.launch" 2>/dev/null && echo "  Stopped inspection manager" || true
pkill -f "inspection_manager_node" 2>/dev/null && echo "  Stopped inspection manager node" || true
pkill -f "photo_capture_service" 2>/dev/null && echo "  Stopped photo capture service" || true

# Nav2 spawns many child processes - kill the lifecycle and Nav2 nodes
pkill -f "nav_lifecycle_startup" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "controller_server" 2>/dev/null || true
pkill -f "planner_server" 2>/dev/null || true
pkill -f "bt_navigator" 2>/dev/null || true
pkill -f "behavior_server" 2>/dev/null || true
pkill -f "smoother_server" 2>/dev/null || true
pkill -f "velocity_smoother" 2>/dev/null || true
pkill -f "waypoint_follower" 2>/dev/null || true

sleep 2
echo "Done. Run start_full_mission.sh to restart."

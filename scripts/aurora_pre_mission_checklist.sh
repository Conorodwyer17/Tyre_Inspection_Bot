#!/bin/bash
# Pre-mission checklist: device status, Aurora native topics, depth pipeline, TF, optional Nav/detection.
# Run after Aurora bringup (and optionally full stack). Uses Aurora factory calibration (no checkerboard).
# Usage: source install/setup.bash && bash scripts/aurora_pre_mission_checklist.sh
# Exit: 0 = all checks pass, 1 = one or more failed.

set -e
UGV_WS="${UGV_WS:-/home/conor/ugv_ws}"
AURORA_IP="${AURORA_IP:-192.168.11.1}"
FAIL=0

echo "=== Pre-mission checklist (Aurora native) ==="

# 1) Device baseline
echo -n "Device (ping $AURORA_IP) ... "
if ping -c 2 -W 3 "$AURORA_IP" >/dev/null 2>&1; then
  echo "OK"
else
  echo "FAIL"
  FAIL=1
fi

# 2) Aurora SDK topics (factory-calibrated)
for topic in \
  /slamware_ros_sdk_server_node/left_image_raw \
  /slamware_ros_sdk_server_node/right_image_raw \
  /slamware_ros_sdk_server_node/robot_pose \
  /slamware_ros_sdk_server_node/depth_image_raw \
  /slamware_ros_sdk_server_node/point_cloud \
  /slamware_ros_sdk_server_node/scan \
  /slamware_ros_sdk_server_node/odom; do
  echo -n "  $topic ... "
  if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
    echo "present"
  else
    echo "MISSING"
    FAIL=1
  fi
done

# 3) Depth pipeline (depth_to_registered_pointcloud publishes these from Aurora depth)
for topic in /camera/depth/camera_info /camera/depth/points /segmentation_processor/registered_pointcloud; do
  echo -n "  $topic ... "
  if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
    echo "present"
  else
    echo "MISSING (start segment_3d launch)"
    FAIL=1
  fi
done

# 4) Perception + Nav (optional for “calibration-only” runs)
echo -n "  /darknet_ros_3d/bounding_boxes ... "
if ros2 topic list 2>/dev/null | grep -q "^/darknet_ros_3d/bounding_boxes$"; then
  echo "present"
else
  echo "absent (start segment_3d for mission)"
fi

echo -n "  Nav2 navigate_to_pose action ... "
if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
  echo "present"
else
  echo "absent (start Nav2 for mission)"
fi

# 5) TF: slamware_map -> base_link (required for tire pose -> nav goal)
echo -n "TF slamware_map -> base_link ... "
if timeout 2 ros2 run tf2_ros tf2_echo slamware_map base_link 2>/dev/null | head -1 | grep -q "At time"; then
  echo "OK"
else
  echo "FAIL (no transform)"
  FAIL=1
fi

# 6) Map (optional)
echo -n "Map server /map ... "
if ros2 topic list 2>/dev/null | grep -q "^/map$"; then
  echo "present"
else
  echo "absent (optional)"
fi

echo "=== Summary ==="
if [ $FAIL -eq 0 ]; then
  echo "PASS: Ready for mission (start inspection_manager when detection + Nav2 are up)."
  exit 0
else
  echo "FAIL: Fix failed items and re-run."
  exit 1
fi

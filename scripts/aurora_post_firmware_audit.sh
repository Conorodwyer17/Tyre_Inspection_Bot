#!/bin/bash
# Aurora Post-Firmware 2.11 Deep Audit
# Run AFTER launching Aurora bringup (slamware_ros_sdk_server_node).
# Captures all topics, publish rates, and logs to determine firmware 2.11 changes.
#
# Usage:
#   Terminal 1: ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1 raw_image_on:=true
#   Terminal 2: source install/setup.bash && bash scripts/aurora_post_firmware_audit.sh [output_dir]
#
# Output: timestamped report in logs/aurora_integration/ or specified dir

set +e  # Don't exit on command failure (audit should continue)
UGV_WS="${UGV_WS:-/home/conor/ugv_ws}"
OUT_DIR="${1:-$UGV_WS/logs/aurora_integration}"
mkdir -p "$OUT_DIR"
REPORT="$OUT_DIR/aurora_firmware_audit_$(date +%Y%m%d_%H%M%S).txt"

{
echo "=============================================="
echo "Aurora Post-Firmware Audit"
echo "Date: $(date)"
echo "Report: $REPORT"
echo "=============================================="

echo ""
echo "=== 1. ROS2 Topics (all slamware) ==="
ros2 topic list 2>/dev/null | grep -E "slamware|slamtec" | sort || echo "(none)"

echo ""
echo "=== 2. Slamware Topic Types ==="
for t in $(ros2 topic list 2>/dev/null | grep slamware); do
  ty=$(ros2 topic info -v "$t" 2>/dev/null | grep "Type:" | head -1 | sed 's/.*: //')
  echo "  $t"
  echo "    Type: $ty"
done

echo ""
echo "=== 3. Topic Publish Rates (10s sample) ==="
TOPICS=(
  /slamware_ros_sdk_server_node/left_image_raw
  /slamware_ros_sdk_server_node/right_image_raw
  /slamware_ros_sdk_server_node/depth_image_raw
  /slamware_ros_sdk_server_node/depth_image_colorized
  /slamware_ros_sdk_server_node/semantic_segmentation
  /slamware_ros_sdk_server_node/point_cloud
  /slamware_ros_sdk_server_node/stereo_keypoints
  /slamware_ros_sdk_server_node/scan
  /slamware_ros_sdk_server_node/odom
  /slamware_ros_sdk_server_node/robot_pose
  /slamware_ros_sdk_server_node/map
  /slamware_ros_sdk_server_node/imu_raw_data
  /slamware_ros_sdk_server_node/state
  /slamware_ros_sdk_server_node/system_status
  /camera/left/camera_info
  /camera/right/camera_info
)
for t in "${TOPICS[@]}"; do
  if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
    echo -n "  $t ... "
    hz=$(timeout 10 ros2 topic hz "$t" 2>/dev/null | tail -1 || echo "timeout/nodata")
    echo "$hz"
  else
    echo "  $t ... NOT PUBLISHED"
  fi
done

echo ""
echo "=== 4. One-time Message Samples ==="
echo "  /slamware_ros_sdk_server_node/state:"
timeout 3 ros2 topic echo /slamware_ros_sdk_server_node/state --once 2>/dev/null || echo "    (no message)"

echo ""
echo "  /slamware_ros_sdk_server_node/system_status (header + key fields):"
timeout 3 ros2 topic echo /slamware_ros_sdk_server_node/system_status --once 2>/dev/null | head -30 || echo "    (no message)"

echo ""
echo "=== 5. depth_image_raw (if present) ==="
if ros2 topic list 2>/dev/null | grep -q "/slamware_ros_sdk_server_node/depth_image_raw"; then
  echo "  Topic exists. Sample encoding/dimensions:"
  timeout 3 ros2 topic echo /slamware_ros_sdk_server_node/depth_image_raw --once 2>/dev/null | grep -E "height|width|encoding|frame_id" | head -5
else
  echo "  NOT PUBLISHED (Depth camera not supported by device)"
fi

echo ""
echo "=== 6. semantic_segmentation (if present) ==="
if ros2 topic list 2>/dev/null | grep -q "/slamware_ros_sdk_server_node/semantic_segmentation"; then
  echo "  Topic exists. Sample:"
  timeout 3 ros2 topic echo /slamware_ros_sdk_server_node/semantic_segmentation --once 2>/dev/null | grep -E "height|width|encoding|frame_id" | head -5
else
  echo "  NOT PUBLISHED (Semantic segmentation not supported by device)"
fi

echo ""
echo "=== 7. point_cloud frame_id and size ==="
if ros2 topic list 2>/dev/null | grep -q "/slamware_ros_sdk_server_node/point_cloud"; then
  timeout 3 ros2 topic echo /slamware_ros_sdk_server_node/point_cloud --once 2>/dev/null | grep -E "frame_id|width|height|row_step" | head -5
else
  echo "  NOT PUBLISHED"
fi

echo ""
echo "=== 8. Camera Info (if stereo_camera_info_enable) ==="
for t in /camera/left/camera_info /camera/right/camera_info; do
  if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
    echo "  $t:"
    timeout 2 ros2 topic echo "$t" --once 2>/dev/null | grep -E "frame_id|width|height|k:" | head -6
  fi
done

echo ""
echo "=== 9. TF Frames (slamware-related) ==="
ros2 run tf2_ros tf2_echo slamware_map base_link 2>/dev/null | head -3 || echo "  (tf not available)"
timeout 1 ros2 run tf2_tools view_frames 2>/dev/null && echo "  (view_frames saved frames.pdf)" || true

echo ""
echo "=== 10. Node Info (slamware_ros_sdk_server_node) ==="
ros2 node info /slamware_ros_sdk_server_node 2>/dev/null | head -50 || echo "  (node not running)"

echo ""
echo "=============================================="
echo "Audit complete. Report: $REPORT"
echo "=============================================="
} 2>&1 | tee "$REPORT"

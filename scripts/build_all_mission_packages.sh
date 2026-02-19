#!/bin/bash
# Build all mission packages in correct order
# Fixes dependency issues

set -e

echo "=========================================="
echo "Building All Mission Packages"
echo "=========================================="
echo ""

cd ~/ugv_ws

echo "Building packages in dependency order..."
echo ""

# Build order: dependencies first
echo "[1/4] Building gb_visual_detection_3d_msgs..."
colcon build --packages-select gb_visual_detection_3d_msgs 2>&1 | tail -5

echo ""
echo "[2/4] Building segmentation_msgs..."
colcon build --packages-select segmentation_msgs 2>&1 | tail -5

echo ""
echo "[3/4] Building segmentation_3d..."
colcon build --packages-select segmentation_3d 2>&1 | tail -5

echo ""
echo "[4/4] Building inspection_manager..."
colcon build --packages-select inspection_manager 2>&1 | tail -5

echo ""
echo "=========================================="
echo "Build complete!"
echo ""
echo "Verify packages:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/gb_visual_detection_3d_msgs/share/gb_visual_detection_3d_msgs/local_setup.bash"
echo "  source install/segmentation_msgs/share/segmentation_msgs/local_setup.bash"
echo "  source install/segmentation_3d/share/segmentation_3d/local_setup.bash"
echo "  source install/inspection_manager/share/inspection_manager/local_setup.bash"
echo "  ros2 pkg list | grep -E '(inspection|segmentation)'"
echo ""

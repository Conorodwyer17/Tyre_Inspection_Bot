#!/bin/bash
# Pre-Mission System Check
# Verifies all components before field test

set -e

echo "=========================================="
echo "Pre-Mission System Check"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

PASS="${GREEN}✓${NC}"
FAIL="${RED}✗${NC}"
WARN="${YELLOW}⚠${NC}"

ERRORS=0

check() {
    if [ $? -eq 0 ]; then
        echo -e "${PASS} $1"
        return 0
    else
        echo -e "${FAIL} $1"
        ERRORS=$((ERRORS + 1))
        return 1
    fi
}

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${FAIL} ROS 2 not found"
    exit 1
fi

# Source workspace packages (try individual packages first to avoid gb_visual_detection_3d_msgs issue)
# Order matters: dependencies first
if [ -f ~/ugv_ws/install/gb_visual_detection_3d_msgs/share/gb_visual_detection_3d_msgs/local_setup.bash ]; then
    source ~/ugv_ws/install/gb_visual_detection_3d_msgs/share/gb_visual_detection_3d_msgs/local_setup.bash 2>/dev/null || true
fi
if [ -f ~/ugv_ws/install/segmentation_msgs/share/segmentation_msgs/local_setup.bash ]; then
    source ~/ugv_ws/install/segmentation_msgs/share/segmentation_msgs/local_setup.bash 2>/dev/null || true
fi
if [ -f ~/ugv_ws/install/segmentation_3d/share/segmentation_3d/local_setup.bash ]; then
    source ~/ugv_ws/install/segmentation_3d/share/segmentation_3d/local_setup.bash 2>/dev/null || true
fi
if [ -f ~/ugv_ws/install/inspection_manager/share/inspection_manager/local_setup.bash ]; then
    source ~/ugv_ws/install/inspection_manager/share/inspection_manager/local_setup.bash 2>/dev/null || true
fi
# Try full workspace source as fallback (may fail due to gb_visual_detection_3d_msgs)
if [ -f ~/ugv_ws/install/setup.bash ]; then
    source ~/ugv_ws/install/setup.bash 2>/dev/null || true
fi

echo "=== 1. Hardware Checks ==="
echo ""

# Check Aurora
AURORA_IP="192.168.11.1"
if ping -c 1 -W 2 "$AURORA_IP" > /dev/null 2>&1; then
    check "Aurora reachable at $AURORA_IP"
else
    echo -e "${FAIL} Aurora not reachable - check Ethernet connection"
    ERRORS=$((ERRORS + 1))
fi

# Check UART
if [ -e /dev/ttyTHS1 ]; then
    check "UART device /dev/ttyTHS1 exists"
    if [ -r /dev/ttyTHS1 ] && [ -w /dev/ttyTHS1 ]; then
        check "UART permissions correct"
    else
        echo -e "${WARN} UART permissions may be wrong"
    fi
else
    echo -e "${FAIL} UART device /dev/ttyTHS1 not found"
    ERRORS=$((ERRORS + 1))
fi

echo ""
echo "=== 2. Software Checks ==="
echo ""

# Check packages
ros2 pkg list | grep -q "slamware_ros_sdk" && check "Aurora SDK installed" || echo -e "${FAIL} Aurora SDK not installed"
ros2 pkg list | grep -q "inspection_manager" && check "inspection_manager installed" || echo -e "${FAIL} inspection_manager not installed"
ros2 pkg list | grep -q "segmentation_3d" && check "segmentation_3d installed" || echo -e "${FAIL} segmentation_3d not installed"
ros2 pkg list | grep -q "nav2" && check "Nav2 installed" || echo -e "${WARN} Nav2 packages missing"

# Check models
if [ -f ~/ugv_ws/src/Tyre_Inspection_Bot/best.pt ]; then
    check "best.pt model file exists"
else
    echo -e "${WARN} best.pt model not found - tire detection may fail"
fi

# Check photo directory
PHOTO_DIR="$HOME/ugv_ws/tire_inspection_photos"
mkdir -p "$PHOTO_DIR"
check "Photo directory exists: $PHOTO_DIR"

# Check disk space
DISK_SPACE=$(df -h "$PHOTO_DIR" | tail -1 | awk '{print $4}')
echo -e "${PASS} Disk space available: $DISK_SPACE"

echo ""
echo "=== 3. Detection Test (Requires Aurora Running) ==="
echo ""

# Check if Aurora topics are available
if ros2 topic list 2>/dev/null | grep -q "slamware_ros_sdk_server_node"; then
    check "Aurora topics available"
    
    # Check camera topic
    if ros2 topic list | grep -q "slamware_ros_sdk_server_node/left_image_raw"; then
        check "Camera topic available"
    else
        echo -e "${WARN} Camera topic not found"
    fi
    
    # Check point cloud
    if ros2 topic list | grep -q "slamware_ros_sdk_server_node/point_cloud"; then
        check "Point cloud topic available"
    else
        echo -e "${WARN} Point cloud topic not found"
    fi
else
    echo -e "${WARN} Aurora not running - start Aurora SDK first"
    echo "  Run: ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1"
fi

echo ""
echo "=== 4. Python Dependencies ==="
echo ""

python3 -c "import cv2" 2>/dev/null && check "OpenCV available" || echo -e "${FAIL} OpenCV not installed"
python3 -c "import ultralytics" 2>/dev/null && check "Ultralytics available" || echo -e "${FAIL} Ultralytics not installed"
python3 -c "import ros2_numpy" 2>/dev/null && check "ros2_numpy available" || echo -e "${FAIL} ros2_numpy not installed"

echo ""
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}All critical checks passed!${NC}"
    echo ""
    echo "System is ready for field test."
    echo ""
    echo "Next steps:"
    echo "  1. Start mission: bash ~/ugv_ws/src/Tyre_Inspection_Bot/scripts/start_full_mission.sh"
    echo "  2. Monitor: bash ~/ugv_ws/src/Tyre_Inspection_Bot/scripts/monitor_mission.sh"
    echo ""
    echo -e "${YELLOW}Note: Start with ONE vehicle for initial test${NC}"
    exit 0
else
    echo -e "${RED}$ERRORS critical issue(s) found${NC}"
    echo ""
    echo "Please fix the issues above before field test."
    exit 1
fi

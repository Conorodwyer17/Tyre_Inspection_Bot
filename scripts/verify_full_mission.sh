#!/bin/bash
# Comprehensive Mission Verification Script
# Verifies entire tire inspection mission setup from bottom to top

set -e

echo "=========================================="
echo "Tire Inspection Mission - Full Verification"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS="${GREEN}✓${NC}"
FAIL="${RED}✗${NC}"
WARN="${YELLOW}⚠${NC}"
INFO="${BLUE}ℹ${NC}"

ERRORS=0
WARNINGS=0

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

warn() {
    echo -e "${WARN} $1"
    WARNINGS=$((WARNINGS + 1))
}

info() {
    echo -e "${INFO} $1"
}

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    ROS_DISTRO="humble"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    ROS_DISTRO="jazzy"
else
    echo -e "${FAIL} ROS 2 not found"
    exit 1
fi

# Source Aurora SDK if available
if [ -f ~/ugv_ws/install/slamware_ros_sdk/share/slamware_ros_sdk/local_setup.bash ]; then
    source ~/ugv_ws/install/slamware_ros_sdk/share/slamware_ros_sdk/local_setup.bash
fi

echo "=== 1. HARDWARE LAYER ==="
echo ""

# Check UART
if [ -e /dev/ttyTHS1 ]; then
    check "UART device /dev/ttyTHS1 exists"
    if [ -r /dev/ttyTHS1 ] && [ -w /dev/ttyTHS1 ]; then
        check "UART permissions correct"
    else
        warn "UART permissions may be wrong (run: sudo usermod -a -G dialout \$USER)"
    fi
else
    echo -e "${FAIL} UART device /dev/ttyTHS1 not found"
    ERRORS=$((ERRORS + 1))
fi

# Check Aurora network
AURORA_IP="192.168.11.1"
if ping -c 1 -W 1 "$AURORA_IP" > /dev/null 2>&1; then
    check "Aurora reachable at $AURORA_IP"
else
    echo -e "${FAIL} Aurora not reachable at $AURORA_IP"
    ERRORS=$((ERRORS + 1))
fi

echo ""
echo "=== 2. ROS 2 ENVIRONMENT ==="
echo ""

ros2 pkg list | grep -q "nav2" && check "Nav2 packages installed" || warn "Nav2 packages missing"
ros2 pkg list | grep -q "slamware_ros_sdk" && check "Aurora SDK package installed" || echo -e "${FAIL} Aurora SDK not installed"
ros2 pkg list | grep -q "segmentation_3d" && check "segmentation_3d package available" || warn "segmentation_3d package not found"

echo ""
echo "=== 3. AURORA COMMUNICATION ==="
echo ""

# Check if Aurora node is running
if ros2 node list 2>/dev/null | grep -q "slamware_ros_sdk_server_node"; then
    check "Aurora SDK node is running"
    
    # Check topics
    sleep 1
    TOPICS=$(ros2 topic list 2>/dev/null | grep slamware || echo "")
    
    if echo "$TOPICS" | grep -q "odom"; then
        check "Aurora odometry topic publishing"
    else
        warn "Aurora odometry topic not found"
    fi
    
    if echo "$TOPICS" | grep -q "scan"; then
        check "Aurora LiDAR scan topic publishing"
    else
        warn "Aurora scan topic not found"
    fi
    
    if echo "$TOPICS" | grep -q "map"; then
        check "Aurora map topic publishing"
    else
        warn "Aurora map topic not found"
    fi
    
    if echo "$TOPICS" | grep -q "point_cloud"; then
        check "Aurora point cloud topic publishing"
    else
        warn "Aurora point cloud topic not found"
    fi
    
    if echo "$TOPICS" | grep -q "left_image_raw"; then
        check "Aurora left camera image topic publishing"
    else
        warn "Aurora left image topic not found"
    fi
    
    # Check TF frames
    sleep 1
    if ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -5 | grep -q "Translation\|At time"; then
        check "TF transform map -> base_link available"
    else
        warn "TF transform map -> base_link not available (may need time to initialize)"
    fi
else
    warn "Aurora SDK node not running (start with: ros2 launch ugv_nav aurora_bringup.launch.py)"
fi

echo ""
echo "=== 4. MOTOR DRIVER (ESP32) ==="
echo ""

if ros2 node list 2>/dev/null | grep -q "motor_driver_node"; then
    check "Motor driver node is running"
    
    # Check cmd_vel subscription
    if ros2 topic info /cmd_vel 2>/dev/null | grep -q "motor_driver_node"; then
        check "Motor driver subscribed to /cmd_vel"
    else
        warn "Motor driver may not be subscribed to /cmd_vel"
    fi
else
    warn "Motor driver node not running (start with: ./scripts/run_motor_driver_standalone.sh)"
fi

# Test zero velocity command
echo ""
info "Publishing zero velocity command to stop robot..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once > /dev/null 2>&1 && check "Zero velocity command sent" || warn "Could not send zero velocity (motor driver may not be running)"

echo ""
echo "=== 5. NAVIGATION (Nav2) ==="
echo ""

# Check Nav2 configuration
if [ -f ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/param/nav_aurora.yaml ]; then
    check "Nav2 Aurora config file exists"
    
    # Verify topic names match Aurora
    if grep -q "/slamware_ros_sdk_server_node/scan" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/param/nav_aurora.yaml; then
        check "Nav2 configured for Aurora scan topic"
    else
        warn "Nav2 may not be configured for Aurora scan topic"
    fi
    
    if grep -q "/slamware_ros_sdk_server_node/odom" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/param/nav_aurora.yaml; then
        check "Nav2 configured for Aurora odom topic"
    else
        warn "Nav2 may not be configured for Aurora odom topic"
    fi
    
    if grep -q "global_frame: map" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/param/nav_aurora.yaml; then
        check "Nav2 global_frame set to 'map' (Aurora frame)"
    else
        warn "Nav2 global_frame may not be set correctly"
    fi
else
    warn "Nav2 Aurora config file not found"
fi

# Check if Nav2 nodes are running
if ros2 node list 2>/dev/null | grep -q "bt_navigator"; then
    check "Nav2 BT navigator running"
else
    info "Nav2 not running (launch with: ros2 launch ugv_nav nav_aurora.launch.py)"
fi

echo ""
echo "=== 6. OBJECT DETECTION (YOLO + 3D) ==="
echo ""

# Check detection config
if [ -f ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml ]; then
    check "Detection config file exists"
    
    # Check interested classes
    if grep -q "interested_classes:.*tire" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml; then
        check "Tire detection configured"
    else
        warn "Tire may not be in interested_classes"
    fi
    
    if grep -q "interested_classes:.*car\|truck" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml; then
        check "Vehicle detection (car/truck) configured"
    else
        warn "Vehicle detection may not be configured"
    fi
    
    if grep -q "interested_classes:.*person" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml; then
        check "Person detection configured"
    else
        warn "Person detection may not be configured"
    fi
    
    # Check point cloud topic
    if grep -q "/slamware_ros_sdk_server_node/point_cloud" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml; then
        check "Detection configured for Aurora point cloud"
    else
        warn "Detection may not be using Aurora point cloud"
    fi
    
    # Check working frame
    if grep -q "working_frame:.*map" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml; then
        check "Detection working_frame set to 'map'"
    else
        warn "Detection working_frame may not be set to 'map'"
    fi
else
    warn "Detection config file not found"
fi

# Check if detection nodes are running
if ros2 node list 2>/dev/null | grep -q "ultralytics_segmentation"; then
    check "YOLO segmentation node running"
else
    info "Detection not running (launch with: ros2 launch segmentation_3d segment_3d.launch.py)"
fi

if ros2 node list 2>/dev/null | grep -q "segmentation_processor_node"; then
    check "3D segmentation processor node running"
else
    info "3D segmentation processor not running"
fi

# Check detection topics
if ros2 topic list 2>/dev/null | grep -q "/darknet_ros_3d/bounding_boxes"; then
    check "3D bounding boxes topic exists"
    
    # Try to get a message
    if timeout 2 ros2 topic echo /darknet_ros_3d/bounding_boxes --once 2>/dev/null | head -5 | grep -q "header\|class_id"; then
        check "3D bounding boxes topic publishing data"
    else
        warn "3D bounding boxes topic exists but may not be publishing"
    fi
else
    info "3D bounding boxes topic not found (detection may not be running)"
fi

echo ""
echo "=== 7. MISSION MANAGEMENT ==="
echo ""

# Check for inspection manager
if [ -d ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager ]; then
    check "Inspection manager package found"
elif [ -d ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_simulation/inspection_manager ]; then
    check "Inspection manager package found (in amr_simulation)"
else
    warn "Inspection manager package not found - mission orchestration may be missing"
    info "  The system can detect objects but may need a mission manager to:"
    info "  - Save positions of detected vehicles/tires"
    info "  - Navigate to each tire"
    info "  - Take photos at each tire"
fi

echo ""
echo "=== 8. DATA FLOW VERIFICATION ==="
echo ""

info "Checking topic connections..."

# Aurora -> Detection
if ros2 topic list 2>/dev/null | grep -q "/slamware_ros_sdk_server_node/left_image_raw" && ros2 topic list 2>/dev/null | grep -q "/ultralytics/segmentation"; then
    check "Aurora image -> YOLO segmentation connection possible"
else
    warn "Aurora image or YOLO segmentation topics not available"
fi

# Detection -> 3D boxes
if ros2 topic list 2>/dev/null | grep -q "/ultralytics/segmentation" && ros2 topic list 2>/dev/null | grep -q "/darknet_ros_3d/bounding_boxes"; then
    check "YOLO segmentation -> 3D boxes connection possible"
else
    warn "Segmentation or 3D boxes topics not available"
fi

# Nav2 -> Motor
if ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then
    SUBSCRIBERS=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count\|Subscription count" || echo "")
    if echo "$SUBSCRIBERS" | grep -q "Subscription count: [1-9]"; then
        check "/cmd_vel has subscribers (motor driver should be subscribed)"
    else
        warn "/cmd_vel has no subscribers (motor driver may not be running)"
    fi
else
    warn "/cmd_vel topic not found"
fi

echo ""
echo "=== 9. PYTHON DEPENDENCIES ==="
echo ""

python3 -c "import ultralytics" 2>/dev/null && check "ultralytics (YOLO) installed" || warn "ultralytics not installed"
python3 -c "import torch" 2>/dev/null && check "torch installed" || warn "torch not installed"
python3 -c "import cv2" 2>/dev/null && check "opencv-python installed" || warn "opencv-python not installed"
python3 -c "import ros2_numpy" 2>/dev/null && check "ros2_numpy installed" || warn "ros2_numpy not installed"
python3 -c "import serial" 2>/dev/null && check "pyserial installed" || warn "pyserial not installed"

echo ""
echo "=== 10. SUMMARY ==="
echo ""

if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}✓ All critical components verified!${NC}"
    echo ""
    echo "System is ready for tire inspection mission."
elif [ $ERRORS -eq 0 ]; then
    echo -e "${YELLOW}⚠ System mostly ready with $WARNINGS warning(s).${NC}"
    echo "Review warnings above. System may work but some features may be limited."
else
    echo -e "${RED}✗ System has $ERRORS error(s) and $WARNINGS warning(s).${NC}"
    echo "Fix errors before running mission."
fi

echo ""
echo "=== MISSION WORKFLOW ==="
echo ""
echo "1. Start Aurora:"
echo "   source /opt/ros/humble/setup.bash"
echo "   source ~/ugv_ws/install/slamware_ros_sdk/share/slamware_ros_sdk/local_setup.bash"
echo "   ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1"
echo ""
echo "2. Start Motor Driver:"
echo "   cd ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_base_driver"
echo "   ./scripts/run_motor_driver_standalone.sh"
echo ""
echo "3. Start Navigation:"
echo "   ros2 launch ugv_nav nav_aurora.launch.py use_rviz:=false aurora_ip:=192.168.11.1"
echo ""
echo "4. Start Detection:"
echo "   ros2 launch segmentation_3d segment_3d.launch.py camera_rgb_topic:=/slamware_ros_sdk_server_node/left_image_raw"
echo ""
echo "5. Monitor Detection:"
echo "   ros2 topic echo /darknet_ros_3d/bounding_boxes"
echo ""
echo "=== NEXT STEPS ==="
echo ""
echo "To complete the mission, you may need:"
echo "- Inspection manager to save positions and orchestrate navigation"
echo "- Photo capture service at each tire location"
echo "- Mission state machine for: find car -> save location -> navigate -> inspect tires"
echo ""

exit $ERRORS

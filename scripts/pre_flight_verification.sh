#!/bin/bash
# Pre-Flight Verification Script
# Run this before real vehicle testing to verify system is ready

set -e  # Exit on error

echo "=========================================="
echo "PRE-FLIGHT VERIFICATION SCRIPT"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if command succeeded
check_result() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ $1${NC}"
        return 0
    else
        echo -e "${RED}❌ $1${NC}"
        return 1
    fi
}

# Function to check if topic exists
check_topic() {
    if ros2 topic list 2>/dev/null | grep -q "$1"; then
        echo -e "${GREEN}✅ Topic $1 exists${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠️  Topic $1 not found (may need to start system)${NC}"
        return 1
    fi
}

# Function to check if node exists
check_node() {
    if ros2 node list 2>/dev/null | grep -q "$1"; then
        echo -e "${GREEN}✅ Node $1 is running${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠️  Node $1 not found (may need to start system)${NC}"
        return 1
    fi
}

# Function to check if service exists
check_service() {
    if ros2 service list 2>/dev/null | grep -q "$1"; then
        echo -e "${GREEN}✅ Service $1 exists${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠️  Service $1 not found (may need to start system)${NC}"
        return 1
    fi
}

# Function to check file exists
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✅ File $1 exists${NC}"
        return 0
    else
        echo -e "${RED}❌ File $1 not found${NC}"
        return 1
    fi
}

# Function to check parameter value
check_param() {
    local node=$1
    local param=$2
    local expected=$3
    
    if ros2 param get "$node" "$param" 2>/dev/null | grep -q "$expected"; then
        echo -e "${GREEN}✅ Parameter $node.$param = $expected${NC}"
        return 0
    else
        actual=$(ros2 param get "$node" "$param" 2>/dev/null | tail -n 1 | awk '{print $NF}')
        echo -e "${YELLOW}⚠️  Parameter $node.$param = $actual (expected: $expected)${NC}"
        return 1
    fi
}

echo "1. BUILD VERIFICATION"
echo "--------------------"
cd ~/ugv_ws
if colcon build --packages-select tyre_inspection_mission 2>&1 | tail -n 5 | grep -q "Finished"; then
    check_result "Build completed successfully"
else
    echo -e "${RED}❌ Build failed. Please check errors above.${NC}"
    exit 1
fi
echo ""

echo "2. LAUNCH FILE VERIFICATION"
echo "---------------------------"
check_file "src/amr_hardware/src/tyre_inspection_mission/launch/autonomous_inspection.launch.py"
check_file "src/amr_hardware/src/tyre_inspection_mission/launch/nav2_navigation_with_remap.launch.py"
echo ""

echo "3. CONFIGURATION FILES"
echo "----------------------"
check_file "src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml"
# Check if license_plate is in interested_classes
if grep -q "license_plate" src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml 2>/dev/null; then
    check_result "License plate class is in config.yaml"
else
    echo -e "${RED}❌ License plate class NOT found in config.yaml${NC}"
fi
echo ""

echo "4. HARDWARE CONNECTIONS"
echo "-----------------------"
# Check serial port
if [ -e /dev/ttyTHS1 ]; then
    check_result "Serial port /dev/ttyTHS1 exists (Jetson)"
elif [ -e /dev/ttyAMA0 ]; then
    check_result "Serial port /dev/ttyAMA0 exists (Other platform)"
else
    echo -e "${RED}❌ Serial port not found (/dev/ttyTHS1 or /dev/ttyAMA0)${NC}"
fi

# Check camera (USB device)
if lsusb 2>/dev/null | grep -qiE "(depthai|oak|intel.*movidius)"; then
    check_result "OAK-D camera detected via USB"
else
    echo -e "${YELLOW}⚠️  OAK-D camera not detected via USB (may need to connect)${NC}"
fi

# Check LiDAR (if using USB serial)
if [ -e /dev/ttyACM0 ]; then
    check_result "LiDAR port /dev/ttyACM0 exists"
else
    echo -e "${YELLOW}⚠️  LiDAR port /dev/ttyACM0 not found (check launch file for correct port)${NC}"
fi
echo ""

echo "5. CODE VERIFICATION"
echo "--------------------"
# Check critical fixes are in code
if grep -q "detected_license_plate_bbox" src/amr_hardware/src/tyre_inspection_mission/tyre_inspection_mission/core/mission_controller.py 2>/dev/null; then
    check_result "Two-stage license plate detection implemented"
else
    echo -e "${RED}❌ Two-stage license plate detection NOT found in mission_controller.py${NC}"
fi

if grep -q "detection_confidence_threshold.*0.5" src/amr_hardware/src/tyre_inspection_mission/tyre_inspection_mission/core/mission_controller.py 2>/dev/null; then
    check_result "Vehicle detection threshold set to 0.5"
else
    echo -e "${YELLOW}⚠️  Vehicle detection threshold may not be 0.5 (check code)${NC}"
fi

if grep -q "ocr_min_confidence.*0.7" src/amr_hardware/src/tyre_inspection_mission/tyre_inspection_mission/core/mission_controller.py 2>/dev/null; then
    check_result "OCR confidence threshold set to 0.7"
else
    echo -e "${YELLOW}⚠️  OCR confidence threshold may not be 0.7 (check code)${NC}"
fi
echo ""

echo "6. SYSTEM CHECK (if ROS 2 is running)"
echo "-------------------------------------"
# Check if ROS 2 is running
if ros2 node list &>/dev/null; then
    echo -e "${GREEN}✅ ROS 2 is running${NC}"
    echo ""
    
    # Check critical nodes
    check_node "mission_controller"
    check_node "cmd_vel_multiplexer"
    check_node "ugv_bringup"
    echo ""
    
    # Check critical topics
    check_topic "/cmd_vel"
    check_topic "/cmd_vel/nav2"
    check_topic "/cmd_vel/direct_control"
    check_topic "/cmd_vel/emergency"
    check_topic "/odom"
    check_topic "/darknet_ros_3d/bounding_boxes"
    check_topic "/mission_controller/state"
    echo ""
    
    # Check critical services
    check_service "/capture_photo"
    check_service "/navigate_to_pose"
    echo ""
    
    # Check parameters (if mission_controller is running)
    if ros2 node list | grep -q "mission_controller"; then
        echo "7. PARAMETER VERIFICATION"
        echo "------------------------"
        check_param "/mission_controller" "detection_confidence_threshold" "0.5"
        check_param "/mission_controller" "license_plate_detection_confidence_threshold" "0.3"
        check_param "/mission_controller" "ocr_min_char_confidence" "0.7"
        check_param "/mission_controller" "ocr_min_global_confidence" "0.8"
        check_param "/mission_controller" "tyre_detection_confidence_threshold" "0.5"
        check_param "/mission_controller" "arrival_distance_threshold" "0.15"
        echo ""
    else
        echo -e "${YELLOW}⚠️  Mission controller not running. Cannot check parameters.${NC}"
        echo "   Start system first: ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true"
        echo ""
    fi
else
    echo -e "${YELLOW}⚠️  ROS 2 is not running.${NC}"
    echo "   Start system first: ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true"
    echo "   Then run this script again to check runtime verification."
    echo ""
fi

echo "8. FINAL VERIFICATION"
echo "--------------------"
echo -e "${GREEN}✅ Verification script completed${NC}"
echo ""
echo "NEXT STEPS:"
echo "1. Review any warnings above"
echo "2. If ROS 2 is not running, start the system:"
echo "   ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true"
echo "3. Run this script again to verify runtime components"
echo "4. Once all checks pass, proceed with real vehicle test"
echo ""
echo "See PRE_FLIGHT_CHECKLIST_REAL_VEHICLE.md for complete checklist"
echo ""

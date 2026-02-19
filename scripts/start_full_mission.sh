#!/bin/bash
# Complete Mission Launch Script
# Starts all components for full tire inspection mission

set -e

echo "=========================================="
echo "Starting Full Tire Inspection Mission"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
AURORA_IP="${AURORA_IP:-192.168.11.1}"
USE_RVIZ="${USE_RVIZ:-false}"

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    ROS_DISTRO="humble"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    ROS_DISTRO="jazzy"
else
    echo "ERROR: ROS 2 not found"
    exit 1
fi

# Source full workspace (ensures all packages: ugv_nav, ugv_base_driver, segmentation_3d, inspection_manager, etc.)
if [ -f ~/ugv_ws/install/setup.bash ]; then
    source ~/ugv_ws/install/setup.bash
    echo -e "${GREEN}✓${NC} Workspace sourced (~/ugv_ws/install/setup.bash)"
else
    echo -e "${YELLOW}⚠${NC} Workspace install/setup.bash not found - run: cd ~/ugv_ws && colcon build && source install/setup.bash"
fi

# Use Cyclone DDS (required for Nav2 lifecycle bringup on this setup; Fast DDS causes timing issues)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo -e "${GREEN}✓${NC} Using Cyclone DDS (RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION)"

# Create log directory
LOG_DIR="$HOME/ugv_ws/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "Logs directory: $LOG_DIR"
echo ""

# Function to check if Aurora is reachable
check_aurora() {
    if ping -c 1 -W 2 "$AURORA_IP" > /dev/null 2>&1; then
        echo -e "${GREEN}✓${NC} Aurora reachable at $AURORA_IP"
        return 0
    else
        echo -e "${YELLOW}⚠${NC} Aurora not reachable at $AURORA_IP - check connection"
        return 1
    fi
}

# Function to wait for topic (capture list to avoid BrokenPipeError when grep exits early)
wait_for_topic() {
    local topic=$1
    local timeout=${2:-10}
    echo -n "Waiting for topic $topic ... "
    for i in $(seq 1 $timeout); do
        topics=$(ros2 topic list 2>/dev/null) && echo "$topics" | grep -q "$topic" && {
            echo -e "${GREEN}✓${NC}"
            return 0
        }
        sleep 1
    done
    echo -e "${YELLOW}⚠${NC} (timeout)"
    return 1
}

# Wait for action server (navigate_to_pose is an action, not a topic - use ros2 action list)
wait_for_action() {
    local action=$1
    local timeout=${2:-180}
    echo -n "Waiting for action $action ... "
    for i in $(seq 1 $timeout); do
        actions=$(ros2 action list 2>/dev/null) && echo "$actions" | grep -q "$action" && {
            echo -e "${GREEN}✓${NC}"
            return 0
        }
        sleep 1
    done
    echo -e "${YELLOW}⚠${NC} (timeout)"
    return 1
}

echo "=== Pre-flight Checks ==="
check_aurora
echo ""

echo "=== Starting Components ==="
echo ""

# Terminal 1: Aurora SDK (must be sourced and launched first)
echo -e "${BLUE}[1/5]${NC} Starting Aurora SDK..."
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:="$AURORA_IP" > "$LOG_DIR/aurora.log" 2>&1 &
AURORA_PID=$!
echo "  Aurora PID: $AURORA_PID"
echo "  Waiting for Aurora topics (can take 10–20 s)..."
sleep 8
wait_for_topic "/slamware_ros_sdk_server_node/odom" 25
wait_for_topic "/slamware_ros_sdk_server_node/scan" 10
echo ""

# Terminal 2: Motor Driver (must have ugv_base_driver built and sourced)
echo -e "${BLUE}[2/5]${NC} Starting Motor Driver..."
if [ -f ~/ugv_ws/install/ugv_base_driver/share/ugv_base_driver/package.sh ]; then
    source ~/ugv_ws/install/ugv_base_driver/share/ugv_base_driver/package.sh 2>/dev/null || true
fi
UART_PORT="${UART_PORT:-/dev/ttyTHS1}"
ros2 run ugv_base_driver motor_driver_node --ros-args -p uart_port:="$UART_PORT" -p baud_rate:=115200 > "$LOG_DIR/motor_driver.log" 2>&1 &
MOTOR_PID=$!
echo "  Motor Driver PID: $MOTOR_PID (UART: $UART_PORT)"
sleep 4
wait_for_topic "/cmd_vel" 10
echo ""

# Terminal 3: Navigation
echo -e "${BLUE}[3/5]${NC} Starting Navigation Stack..."
ros2 launch ugv_nav nav_aurora.launch.py use_rviz:="$USE_RVIZ" aurora_ip:="$AURORA_IP" > "$LOG_DIR/nav2.log" 2>&1 &
NAV_PID=$!
echo "  Nav2 PID: $NAV_PID"
sleep 5
# Nav2 lifecycle at 120s; wait for navigate_to_pose action (use action list, not topic list)
wait_for_action "navigate_to_pose" 180
echo ""

# Terminal 4: Detection (segmentation_3d package: Ultralytics YOLO + segmentation_processor)
echo -e "${BLUE}[4/5]${NC} Starting Detection Pipeline..."
ros2 launch segmentation_3d segment_3d.launch.py camera_rgb_topic:=/slamware_ros_sdk_server_node/left_image_raw > "$LOG_DIR/detection.log" 2>&1 &
DETECTION_PID=$!
echo "  Detection PID: $DETECTION_PID"
sleep 5
# Wait for upstream (Ultralytics) then downstream (segmentation_processor) topics; YOLO model load can take 15–25s
wait_for_topic "/ultralytics/segmentation/objects_segment" 25
wait_for_topic "/darknet_ros_3d/bounding_boxes" 15
echo ""

# Terminal 5: Inspection Manager
echo -e "${BLUE}[5/5]${NC} Starting Inspection Manager..."
ros2 launch inspection_manager inspection_manager.launch.py > "$LOG_DIR/inspection_manager.log" 2>&1 &
INSPECTION_PID=$!
echo "  Inspection Manager PID: $INSPECTION_PID"
sleep 3
wait_for_topic "/inspection_state" 10
wait_for_topic "/inspection_manager/capture_photo" 5
echo ""

echo "=========================================="
echo -e "${GREEN}All components started!${NC}"
echo "=========================================="
echo ""
echo "Component PIDs:"
echo "  Aurora SDK:      $AURORA_PID"
echo "  Motor Driver:    $MOTOR_PID"
echo "  Nav2:            $NAV_PID"
echo "  Detection:       $DETECTION_PID"
echo "  Inspection Mgr: $INSPECTION_PID"
echo ""
echo "Logs directory: $LOG_DIR"
echo "  View Nav2 log: tail -150 $LOG_DIR/nav2.log"
echo ""
echo "=== MONITORING ==="
echo ""
echo "Option 1 - Simple State Monitor:"
echo "  ros2 topic echo /inspection_state"
echo ""
echo "Option 2 - Live Dashboard (Recommended):"
echo "  bash ~/ugv_ws/src/Tyre_Inspection_Bot/scripts/monitor_mission.sh"
echo "  (Select option 6 for full dashboard)"
echo ""
echo "Option 3 - Remote Monitoring (SSH from laptop/phone):"
echo "  ssh conor@<jetson-ip>"
echo "  bash ~/ugv_ws/src/Tyre_Inspection_Bot/scripts/monitor_mission.sh"
echo ""
echo "View photos:"
echo "  ls -lh ~/ugv_ws/tire_inspection_photos/"
echo ""
echo "See docs/MISSION_FLOW.md for step-by-step mission flow"
echo "See MISSION_MONITORING_GUIDE.md for detailed monitoring options"
echo ""
echo "To stop all components:"
echo "  bash ~/ugv_ws/src/Tyre_Inspection_Bot/scripts/stop_mission.sh"
echo "  # or: kill $AURORA_PID $MOTOR_PID $NAV_PID $DETECTION_PID $INSPECTION_PID"
echo ""
echo "Mission is running! Robot will:"
echo "  1. Build map using Aurora SLAM"
echo "  2. Detect vehicles (car/truck)"
echo "  3. Save vehicle positions"
echo "  4. Navigate to each vehicle"
echo "  5. Detect and inspect all tires"
echo "  6. Capture photos at each tire"
echo ""

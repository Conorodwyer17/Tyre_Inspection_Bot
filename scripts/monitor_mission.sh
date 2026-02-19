#!/bin/bash
# Mission Monitoring Script
# Provides real-time monitoring of tire inspection mission

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "Tire Inspection Mission Monitor"
echo "=========================================="
echo ""

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace if available
if [ -f ~/ugv_ws/install/setup.bash ]; then
    source ~/ugv_ws/install/setup.bash
fi

echo "Monitoring Options:"
echo ""
echo "1. Mission State (Current FSM state)"
echo "2. Detections (Vehicle/Tire bounding boxes)"
echo "3. Navigation Status (Nav2 goals and feedback)"
echo "4. System Health (All topics and nodes)"
echo "5. Photo Count (Number of photos captured)"
echo "6. Live Dashboard (All key metrics)"
echo ""
read -p "Select option (1-6) or 'all' for dashboard: " choice

case $choice in
    1)
        echo ""
        echo -e "${BLUE}=== Mission State ===${NC}"
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 topic echo /inspection_state --once
        watch -n 1 'ros2 topic echo /inspection_state --once 2>/dev/null | head -3'
        ;;
    2)
        echo ""
        echo -e "${BLUE}=== Detections ===${NC}"
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 topic echo /darknet_ros_3d/bounding_boxes
        ;;
    3)
        echo ""
        echo -e "${BLUE}=== Navigation Status ===${NC}"
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 action info /navigate_to_pose
        echo ""
        echo "Monitoring navigation feedback..."
        ros2 topic echo /navigate_to_pose/_action/feedback
        ;;
    4)
        echo ""
        echo -e "${BLUE}=== System Health ===${NC}"
        echo ""
        echo "Active Nodes:"
        ros2 node list
        echo ""
        echo "Active Topics:"
        ros2 topic list | grep -E "(inspection|detection|nav|aurora|slamware)" | head -20
        echo ""
        echo "Topic Rates (Hz):"
        ros2 topic hz /inspection_state 2>/dev/null &
        ros2 topic hz /darknet_ros_3d/bounding_boxes 2>/dev/null &
        sleep 5
        pkill -f "topic hz"
        ;;
    5)
        echo ""
        echo -e "${BLUE}=== Photo Count ===${NC}"
        PHOTO_DIR="$HOME/ugv_ws/tire_inspection_photos"
        if [ -d "$PHOTO_DIR" ]; then
            COUNT=$(ls -1 "$PHOTO_DIR"/*.jpg 2>/dev/null | wc -l)
            echo "Photos captured: $COUNT"
            echo ""
            echo "Latest photos:"
            ls -lht "$PHOTO_DIR"/*.jpg 2>/dev/null | head -5
        else
            echo "Photo directory not found: $PHOTO_DIR"
        fi
        ;;
    6|all)
        echo ""
        echo -e "${BLUE}=== Live Mission Dashboard ===${NC}"
        echo "Press Ctrl+C to stop"
        echo ""
        
        while true; do
            clear
            echo "=========================================="
            echo "Tire Inspection Mission - Live Dashboard"
            echo "=========================================="
            echo ""
            
            # Mission State
            echo -e "${GREEN}Mission State:${NC}"
            STATE=$(ros2 topic echo /inspection_state --once 2>/dev/null | grep "data:" | awk '{print $2}' | tr -d '"')
            if [ -n "$STATE" ]; then
                echo "  Current: $STATE"
            else
                echo "  Status: Not available"
            fi
            echo ""
            
            # Detections
            echo -e "${GREEN}Recent Detections:${NC}"
            ros2 topic echo /darknet_ros_3d/bounding_boxes --once 2>/dev/null | grep -E "(object_name|probability)" | head -6 || echo "  No detections"
            echo ""
            
            # Photo Count
            PHOTO_DIR="$HOME/ugv_ws/tire_inspection_photos"
            if [ -d "$PHOTO_DIR" ]; then
                COUNT=$(ls -1 "$PHOTO_DIR"/*.jpg 2>/dev/null | wc -l)
                echo -e "${GREEN}Photos Captured:${NC} $COUNT"
            else
                echo -e "${GREEN}Photos Captured:${NC} 0 (directory not found)"
            fi
            echo ""
            
            # Navigation
            echo -e "${GREEN}Navigation:${NC}"
            if ros2 action list | grep -q navigate_to_pose; then
                echo "  Nav2: Active"
            else
                echo "  Nav2: Not available"
            fi
            echo ""
            
            # Aurora
            echo -e "${GREEN}Aurora Status:${NC}"
            if ros2 topic list | grep -q slamware_ros_sdk_server_node; then
                echo "  Aurora SDK: Connected"
            else
                echo "  Aurora SDK: Not connected"
            fi
            echo ""
            
            # Timestamp
            echo "Last update: $(date '+%H:%M:%S')"
            echo ""
            echo "Press Ctrl+C to exit"
            
            sleep 2
        done
        ;;
    *)
        echo "Invalid option"
        exit 1
        ;;
esac

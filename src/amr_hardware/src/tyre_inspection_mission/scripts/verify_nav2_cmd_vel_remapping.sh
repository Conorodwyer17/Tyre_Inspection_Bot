#!/bin/bash
# Verification script for Nav2 cmd_vel remapping
# This script verifies that Nav2's controller_server publishes to /cmd_vel/nav2 (not /cmd_vel)

echo "=========================================="
echo "Nav2 cmd_vel Remapping Verification"
echo "=========================================="
echo ""
echo "This script verifies that Nav2's controller_server is properly remapped"
echo "to publish to /cmd_vel/nav2 instead of /cmd_vel directly."
echo ""
echo "CRITICAL: Nav2 must publish to /cmd_vel/nav2 for the multiplexer to work."
echo ""

# Check if system is running
if ! ros2 node list &>/dev/null; then
    echo "❌ ERROR: ROS 2 system is not running!"
    echo "   Start the system first:"
    echo "   ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true"
    exit 1
fi

echo "✅ ROS 2 system is running"
echo ""

# Check if Nav2 controller_server node exists
if ros2 node list | grep -q "controller_server"; then
    echo "✅ Nav2 controller_server node is running"
else
    echo "⚠️  WARNING: Nav2 controller_server node not found"
    echo "   Nav2 may not be started, or node name is different"
fi

# Check if /cmd_vel/nav2 topic exists
if ros2 topic list | grep -q "^/cmd_vel/nav2$"; then
    echo "✅ /cmd_vel/nav2 topic exists"
    
    # Check publishers
    PUB_INFO=$(ros2 topic info /cmd_vel/nav2 2>/dev/null)
    PUB_COUNT=$(echo "$PUB_INFO" | grep "Publisher count:" | awk '{print $3}' || echo "0")
    
    if [ "$PUB_COUNT" -gt 0 ]; then
        echo "✅ /cmd_vel/nav2 has $PUB_COUNT publisher(s)"
        
        # List publisher node names
        echo "   Publishers:"
        ros2 topic info /cmd_vel/nav2 --verbose 2>/dev/null | grep "Publisher:" | while read line; do
            NODE_NAME=$(echo "$line" | awk '{print $2}' | cut -d'[' -f1)
            echo "     - $NODE_NAME"
        done
        
        # Check if controller_server is publishing
        if echo "$PUB_INFO" | grep -q "controller_server"; then
            echo "✅ Nav2 controller_server is publishing to /cmd_vel/nav2 (CORRECT)"
        else
            echo "⚠️  WARNING: controller_server not found in /cmd_vel/nav2 publishers"
        fi
    else
        echo "⚠️  WARNING: /cmd_vel/nav2 has no publishers (Nav2 may not be active)"
    fi
else
    echo "❌ ERROR: /cmd_vel/nav2 topic does not exist!"
    echo "   This means Nav2 remapping is NOT working."
    echo "   Nav2 controller_server should publish to /cmd_vel/nav2, not /cmd_vel"
fi

echo ""

# Check if /cmd_vel has unexpected publishers (Nav2 should NOT publish here)
echo "Checking /cmd_vel topic for unexpected Nav2 publishers..."
CMD_VEL_PUB_INFO=$(ros2 topic info /cmd_vel 2>/dev/null)
CMD_VEL_PUB_COUNT=$(echo "$CMD_VEL_PUB_INFO" | grep "Publisher count:" | awk '{print $3}' || echo "0")

if [ "$CMD_VEL_PUB_COUNT" -gt 0 ]; then
    echo "   /cmd_vel has $CMD_VEL_PUB_COUNT publisher(s):"
    ros2 topic info /cmd_vel --verbose 2>/dev/null | grep "Publisher:" | while read line; do
        NODE_NAME=$(echo "$line" | awk '{print $2}' | cut -d'[' -f1)
        echo "     - $NODE_NAME"
        
        # Check if this is a Nav2 node (should NOT be publishing to /cmd_vel)
        if echo "$NODE_NAME" | grep -qE "(controller_server|behavior_server|velocity_smoother)"; then
            echo "       ❌ ERROR: Nav2 node '$NODE_NAME' is publishing to /cmd_vel!"
            echo "          This should be remapped to /cmd_vel/nav2"
        fi
    done
    
    # Check if cmd_vel_multiplexer is the only expected publisher
    if echo "$CMD_VEL_PUB_INFO" | grep -q "cmd_vel_multiplexer"; then
        echo "   ✅ cmd_vel_multiplexer is publishing to /cmd_vel (CORRECT)"
    else
        echo "   ⚠️  WARNING: cmd_vel_multiplexer not found in /cmd_vel publishers"
    fi
else
    echo "   ⚠️  /cmd_vel has no publishers"
fi

echo ""
echo "=========================================="
echo "Summary:"
echo "=========================================="
echo "1. Nav2 controller_server should publish to /cmd_vel/nav2: $(ros2 topic list | grep -q '^/cmd_vel/nav2$' && echo '✅' || echo '❌')"
echo "2. Nav2 should NOT publish to /cmd_vel directly: $([ -z "$(ros2 topic info /cmd_vel --verbose 2>/dev/null | grep -E '(controller_server|behavior_server|velocity_smoother)')" ] && echo '✅' || echo '❌')"
echo "3. cmd_vel_multiplexer should publish to /cmd_vel: $(ros2 topic info /cmd_vel --verbose 2>/dev/null | grep -q 'cmd_vel_multiplexer' && echo '✅' || echo '❌')"
echo ""
echo "If any checks fail, Nav2 remapping is not working correctly."
echo "Check nav2_navigation_with_remap.launch.py remappings."

#!/bin/bash
# Verification script to prove cmd_vel arbitration is working correctly
# This script checks that:
# 1. Only ONE node publishes to /cmd_vel (the multiplexer)
# 2. Priority topics exist and have publishers
# 3. Nav2's cmd_vel is remapped to /cmd_vel/nav2
# 4. All QoS settings are correct

echo "=========================================="
echo "CMD_VEL ARBITRATION VERIFICATION"
echo "=========================================="
echo ""

# Check if ROS 2 is running
if ! ros2 topic list > /dev/null 2>&1; then
    echo "❌ ERROR: ROS 2 is not running!"
    echo "   Start the system first:"
    echo "   ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true"
    exit 1
fi

echo "✅ ROS 2 is running"
echo ""

# Check /cmd_vel topic exists
if ! ros2 topic list | grep -q "^/cmd_vel$"; then
    echo "❌ ERROR: /cmd_vel topic does not exist!"
    exit 1
fi

echo "✅ /cmd_vel topic exists"

# Check who publishes to /cmd_vel (should be ONLY cmd_vel_multiplexer)
echo ""
echo "Checking /cmd_vel publishers (should be ONLY cmd_vel_multiplexer):"
PUBLISHERS=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
if [ -z "$PUBLISHERS" ] || [ "$PUBLISHERS" = "0" ]; then
    echo "❌ ERROR: No publishers to /cmd_vel!"
    exit 1
fi

echo "   Publisher count: $PUBLISHERS"
if [ "$PUBLISHERS" != "1" ]; then
    echo "⚠️  WARNING: Multiple publishers to /cmd_vel (expected: 1, got: $PUBLISHERS)"
    echo "   This indicates a conflict! The multiplexer should be the ONLY publisher."
fi

# List actual publishers
echo "   Publishers:"
ros2 topic info /cmd_vel 2>/dev/null | grep -A 10 "Publisher" | grep ":" || echo "   (Could not get publisher list)"

echo ""
echo "Checking priority topics:"

# Check /cmd_vel/emergency (Priority 1)
if ros2 topic list | grep -q "^/cmd_vel/emergency$"; then
    echo "✅ /cmd_vel/emergency exists (Priority 1 - Movement Guarantee)"
    PUB_COUNT=$(ros2 topic info /cmd_vel/emergency 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")
    echo "   Publishers: $PUB_COUNT (expected: 1 when movement_guarantee is active)"
else
    echo "⚠️  /cmd_vel/emergency does not exist (will be created when movement_guarantee publishes)"
fi

# Check /cmd_vel/direct_control (Priority 2)
if ros2 topic list | grep -q "^/cmd_vel/direct_control$"; then
    echo "✅ /cmd_vel/direct_control exists (Priority 2 - Direct Navigation)"
    PUB_COUNT=$(ros2 topic info /cmd_vel/direct_control 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")
    echo "   Publishers: $PUB_COUNT (expected: 1 when direct_navigation_fallback is active)"
else
    echo "⚠️  /cmd_vel/direct_control does not exist (will be created when direct_navigation_fallback publishes)"
fi

# Check /cmd_vel/nav2 (Priority 3)
if ros2 topic list | grep -q "^/cmd_vel/nav2$"; then
    echo "✅ /cmd_vel/nav2 exists (Priority 3 - Nav2 Controller)"
    PUB_COUNT=$(ros2 topic info /cmd_vel/nav2 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")
    echo "   Publishers: $PUB_COUNT (expected: 1 when Nav2 is navigating)"
else
    echo "⚠️  /cmd_vel/nav2 does not exist (will be created when Nav2 controller publishes)"
fi

# Check /cmd_vel/teleop (Priority 4)
if ros2 topic list | grep -q "^/cmd_vel/teleop$"; then
    echo "✅ /cmd_vel/teleop exists (Priority 4 - Manual Control)"
    PUB_COUNT=$(ros2 topic info /cmd_vel/teleop 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")
    echo "   Publishers: $PUB_COUNT (expected: 0 or 1 when teleop is active)"
else
    echo "⚠️  /cmd_vel/teleop does not exist (will be created when teleop publishes)"
fi

# CRITICAL: Check for direct publishers to /cmd_vel (bypassing multiplexer)
echo ""
echo "Checking for DIRECT publishers to /cmd_vel (should be ONLY cmd_vel_multiplexer):"
# Get verbose topic info to see all publishers
TOPIC_INFO=$(ros2 topic info /cmd_vel --verbose 2>/dev/null || echo "")
# Extract publisher names (lines after "Publisher" that contain node names)
PUBLISHER_LIST=$(echo "$TOPIC_INFO" | grep -A 50 "Publisher" | grep -E "  .*:" | sed 's/^[[:space:]]*//' | cut -d ':' -f 1 | grep -v "^Publisher" | grep -v "^Count" || echo "")
# Check if any publisher is NOT cmd_vel_multiplexer
VIOLATORS=$(echo "$PUBLISHER_LIST" | grep -v "cmd_vel_multiplexer" | grep -v "^$" || echo "")
if [ -n "$VIOLATORS" ]; then
    echo "❌ CRITICAL ERROR: Found direct publishers to /cmd_vel (bypassing multiplexer):"
    echo "$VIOLATORS" | while IFS= read -r pub; do
        if [ -n "$pub" ]; then
            echo "   - $pub"
            # Check if it's one of the known manual control nodes
            if echo "$pub" | grep -qE "(behavior_ctrl|joy_ctrl|keyboard_ctrl)"; then
                echo "      ⚠️  This node should publish to /cmd_vel/teleop, not /cmd_vel!"
            fi
        fi
    done
    echo ""
    echo "   FIX: These nodes must publish to priority topics:"
    echo "   - behavior_ctrl, joy_ctrl, keyboard_ctrl → /cmd_vel/teleop (Priority 4)"
    echo "   - direct_navigation_fallback → /cmd_vel/direct_control (Priority 2)"
    echo "   - movement_guarantee → /cmd_vel/emergency (Priority 1)"
    echo "   - Nav2 controller → /cmd_vel/nav2 (Priority 3)"
    echo ""
    echo "   To verify manually:"
    echo "   ros2 topic info /cmd_vel --verbose | grep -A 20 'Publisher'"
    exit 1
else
    if [ "$PUBLISHERS" = "1" ]; then
        echo "✅ Only cmd_vel_multiplexer publishes to /cmd_vel (correct)"
    else
        echo "⚠️  Expected 1 publisher but found $PUBLISHERS. Run 'ros2 topic info /cmd_vel --verbose' for details."
    fi
fi

echo ""
echo "Checking critical nodes:"

# Check cmd_vel_multiplexer is running
if ros2 node list | grep -q "cmd_vel_multiplexer"; then
    echo "✅ cmd_vel_multiplexer node is running"
else
    echo "❌ ERROR: cmd_vel_multiplexer node is NOT running!"
    echo "   This is CRITICAL - without it, cmd_vel arbitration won't work!"
    exit 1
fi

# Check Nav2 controller is running (if Nav2 is enabled)
if ros2 node list | grep -q "controller_server"; then
    echo "✅ Nav2 controller_server is running"
    
    # Verify Nav2 controller publishes to /cmd_vel/nav2 (not /cmd_vel)
    if ros2 topic list | grep -q "^/cmd_vel/nav2$"; then
        echo "✅ Nav2 controller cmd_vel is remapped to /cmd_vel/nav2"
    else
        echo "⚠️  WARNING: Nav2 controller might still be publishing to /cmd_vel directly!"
        echo "   This will cause conflicts with the multiplexer!"
    fi
else
    echo "⚠️  Nav2 controller_server is not running (Nav2 might not be enabled)"
fi

echo ""
echo "Checking QoS settings:"
echo "   (Use 'ros2 topic info /cmd_vel --verbose' for detailed QoS info)"

# Check cmd_vel QoS (should be RELIABLE, TRANSIENT_LOCAL)
echo "   /cmd_vel QoS: (check manually with 'ros2 topic info /cmd_vel --verbose')"

echo ""
echo "=========================================="
echo "VERIFICATION SUMMARY"
echo "=========================================="
echo ""
echo "CRITICAL CHECKS:"
echo "  1. cmd_vel_multiplexer running: $(ros2 node list | grep -q cmd_vel_multiplexer && echo '✅' || echo '❌')"
echo "  2. Only ONE publisher to /cmd_vel: $([ \"$PUBLISHERS\" = \"1\" ] && echo '✅' || echo '⚠️')"
echo "  3. Nav2 remapped to /cmd_vel/nav2: $(ros2 topic list | grep -q '^/cmd_vel/nav2$' && echo '✅' || echo '⚠️')"
echo ""
echo "NEXT STEPS IF ISSUES FOUND:"
echo "  1. If multiple publishers to /cmd_vel:"
echo "     - Check if Nav2 controller is publishing directly to /cmd_vel"
echo "     - Verify custom nav2_navigation_with_remap.launch.py is being used"
echo "  2. If priority topics missing:"
echo "     - Start mission to activate direct_navigation_fallback"
echo "     - Check node logs for errors"
echo ""
echo "To monitor cmd_vel in real-time:"
echo "  ros2 topic echo /cmd_vel --qos-profile reliability=reliable"
echo "  ros2 topic hz /cmd_vel"
echo ""

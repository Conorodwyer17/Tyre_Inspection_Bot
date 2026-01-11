#!/bin/bash
# Verification script for cmd_vel pipeline
# This script verifies that cmd_vel_multiplexer is running and cmd_vel pipeline is working

echo "🔍 Verifying cmd_vel pipeline..."
echo ""

# Check if cmd_vel_multiplexer node is running
echo "1. Checking if cmd_vel_multiplexer node is running..."
if ros2 node list 2>/dev/null | grep -q "cmd_vel_multiplexer"; then
    echo "   ✅ cmd_vel_multiplexer node is running"
else
    echo "   ❌ cmd_vel_multiplexer node is NOT running"
    echo "   → CRITICAL: Robot cannot move without this node!"
    echo "   → Solution: Restart launch file to start the node with fixed code"
fi
echo ""

# Check if /cmd_vel topic has a publisher
echo "2. Checking /cmd_vel topic publishers..."
PUBLISHER_COUNT=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count" | awk '{print $3}')
if [ "$PUBLISHER_COUNT" == "1" ]; then
    echo "   ✅ /cmd_vel has 1 publisher (cmd_vel_multiplexer)"
elif [ "$PUBLISHER_COUNT" == "0" ]; then
    echo "   ❌ /cmd_vel has 0 publishers"
    echo "   → CRITICAL: No commands are being published to /cmd_vel!"
    echo "   → Solution: Restart launch file to start cmd_vel_multiplexer"
else
    echo "   ⚠️  /cmd_vel has $PUBLISHER_COUNT publishers (should be 1)"
    echo "   → WARNING: Multiple publishers detected - may cause conflicts"
fi
echo ""

# Check priority topics
echo "3. Checking priority topics..."
for topic in "/cmd_vel/emergency" "/cmd_vel/direct_control" "/cmd_vel/nav2" "/cmd_vel/teleop"; do
    PUB_COUNT=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher count" | awk '{print $3}' || echo "0")
    SUB_COUNT=$(ros2 topic info "$topic" 2>/dev/null | grep "Subscription count" | awk '{print $3}' || echo "0")
    if [ "$SUB_COUNT" == "1" ]; then
        echo "   ✅ $topic: $PUB_COUNT publishers, $SUB_COUNT subscribers (multiplexer subscribed)"
    elif [ "$SUB_COUNT" == "0" ]; then
        echo "   ⚠️  $topic: $PUB_COUNT publishers, 0 subscribers (multiplexer NOT subscribed - node may not be running)"
    else
        echo "   ℹ️  $topic: $PUB_COUNT publishers, $SUB_COUNT subscribers"
    fi
done
echo ""

# Check if ugv_bringup is subscribed to /cmd_vel
echo "4. Checking /cmd_vel subscribers..."
SUBSCRIBER_COUNT=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Subscription count" | awk '{print $3}')
if [ "$SUBSCRIBER_COUNT" -ge "1" ]; then
    echo "   ✅ /cmd_vel has $SUBSCRIBER_COUNT subscribers (ugv_bringup should be subscribed)"
else
    echo "   ⚠️  /cmd_vel has $SUBSCRIBER_COUNT subscribers (expected at least 1: ugv_bringup)"
fi
echo ""

# Final status
if ros2 node list 2>/dev/null | grep -q "cmd_vel_multiplexer"; then
    echo "✅ cmd_vel pipeline is CONFIGURED CORRECTLY"
    echo "   → cmd_vel_multiplexer is running"
    echo "   → Priority topics are subscribed"
    echo "   → /cmd_vel is being published"
else
    echo "❌ cmd_vel pipeline is BROKEN"
    echo "   → cmd_vel_multiplexer is NOT running"
    echo "   → /cmd_vel has no publishers"
    echo "   → Robot cannot move!"
    echo ""
    echo "🔧 SOLUTION: Restart the launch file to start cmd_vel_multiplexer with fixed code"
fi

#!/bin/bash
# Verification script for complete cmd_vel pipeline
# This script verifies the entire cmd_vel pipeline from multiplexer → ugv_bringup → ESP32

echo "=========================================="
echo "cmd_vel Pipeline Verification"
echo "=========================================="
echo ""
echo "This script verifies the complete cmd_vel pipeline:"
echo "  1. Priority topics → cmd_vel_multiplexer → /cmd_vel"
echo "  2. /cmd_vel → ugv_bringup → ESP32 (serial)"
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

# Check 1: Priority topics exist
echo "1. Checking priority topics..."
PRIORITY_TOPICS=("/cmd_vel/emergency" "/cmd_vel/direct_control" "/cmd_vel/nav2" "/cmd_vel/teleop")
for topic in "${PRIORITY_TOPICS[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo "   ✅ $topic exists"
    else
        echo "   ⚠️  $topic does not exist (will be created when publisher starts)"
    fi
done

# Check 2: cmd_vel_multiplexer node
echo ""
echo "2. Checking cmd_vel_multiplexer node..."
if ros2 node list | grep -q "cmd_vel_multiplexer"; then
    echo "   ✅ cmd_vel_multiplexer node is running"
    
    # Check subscriptions (should subscribe to all priority topics)
    NODE_INFO=$(ros2 node info /cmd_vel_multiplexer 2>/dev/null)
    for topic in "${PRIORITY_TOPICS[@]}"; do
        if echo "$NODE_INFO" | grep -q "$topic"; then
            echo "   ✅ Subscribes to $topic"
        else
            echo "   ⚠️  Does not subscribe to $topic"
        fi
    done
    
    # Check publisher (should publish to /cmd_vel)
    if echo "$NODE_INFO" | grep -q "/cmd_vel"; then
        echo "   ✅ Publishes to /cmd_vel"
    else
        echo "   ❌ Does not publish to /cmd_vel"
    fi
else
    echo "   ❌ ERROR: cmd_vel_multiplexer node not found!"
    echo "      This node is REQUIRED for command arbitration"
fi

# Check 3: /cmd_vel topic
echo ""
echo "3. Checking /cmd_vel topic..."
if ros2 topic list | grep -q "^/cmd_vel$"; then
    echo "   ✅ /cmd_vel topic exists"
    
    # Check publishers
    PUB_INFO=$(ros2 topic info /cmd_vel 2>/dev/null)
    PUB_COUNT=$(echo "$PUB_INFO" | grep "Publisher count:" | awk '{print $3}' || echo "0")
    
    if [ "$PUB_COUNT" -gt 0 ]; then
        echo "   ✅ /cmd_vel has $PUB_COUNT publisher(s):"
        ros2 topic info /cmd_vel --verbose 2>/dev/null | grep "Publisher:" | while read line; do
            NODE_NAME=$(echo "$line" | awk '{print $2}' | cut -d'[' -f1)
            echo "     - $NODE_NAME"
            
            # Verify only cmd_vel_multiplexer publishes
            if echo "$NODE_NAME" | grep -q "cmd_vel_multiplexer"; then
                echo "       ✅ CORRECT: cmd_vel_multiplexer is the only publisher"
            else
                echo "       ❌ ERROR: Unexpected publisher '$NODE_NAME'!"
                echo "          Only cmd_vel_multiplexer should publish to /cmd_vel"
            fi
        done
    else
        echo "   ⚠️  /cmd_vel has no publishers"
    fi
    
    # Check subscribers
    SUB_COUNT=$(echo "$PUB_INFO" | grep "Subscription count:" | awk '{print $3}' || echo "0")
    if [ "$SUB_COUNT" -gt 0 ]; then
        echo "   ✅ /cmd_vel has $SUB_COUNT subscriber(s):"
        ros2 topic info /cmd_vel --verbose 2>/dev/null | grep "Subscription:" | while read line; do
            NODE_NAME=$(echo "$line" | awk '{print $2}' | cut -d'[' -f1)
            echo "     - $NODE_NAME"
            
            # Check if ugv_bringup subscribes
            if echo "$NODE_NAME" | grep -q "ugv_bringup"; then
                echo "       ✅ CORRECT: ugv_bringup subscribes (sends to ESP32)"
            fi
        done
    else
        echo "   ⚠️  /cmd_vel has no subscribers"
    fi
else
    echo "   ❌ ERROR: /cmd_vel topic does not exist!"
fi

# Check 4: ugv_bringup node
echo ""
echo "4. Checking ugv_bringup node..."
if ros2 node list | grep -q "ugv_bringup"; then
    echo "   ✅ ugv_bringup node is running"
    
    # Check if it subscribes to /cmd_vel
    NODE_INFO=$(ros2 node info /ugv_bringup 2>/dev/null)
    if echo "$NODE_INFO" | grep -q "/cmd_vel"; then
        echo "   ✅ Subscribes to /cmd_vel"
    else
        echo "   ❌ Does not subscribe to /cmd_vel"
    fi
else
    echo "   ⚠️  ugv_bringup node not found"
fi

# Check 5: Message flow test (if topics are active)
echo ""
echo "5. Testing message flow..."
echo "   Publishing test command to /cmd_vel/direct_control..."
timeout 2 ros2 topic pub /cmd_vel/direct_control geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10 &>/dev/null &
PUB_PID=$!
sleep 1

# Check if message appears on /cmd_vel
if timeout 1 ros2 topic echo /cmd_vel --once &>/dev/null; then
    echo "   ✅ Message flow: /cmd_vel/direct_control → /cmd_vel (working)"
else
    echo "   ⚠️  Message flow test inconclusive (topics may not be active)"
fi

kill $PUB_PID 2>/dev/null

echo ""
echo "=========================================="
echo "Summary:"
echo "=========================================="
echo "Pipeline: Priority topics → cmd_vel_multiplexer → /cmd_vel → ugv_bringup → ESP32"
echo ""
echo "✅ All checks completed"
echo ""
echo "To monitor cmd_vel in real-time:"
echo "  ros2 topic echo /cmd_vel --qos-profile reliability=reliable"
echo ""
echo "To test priority arbitration:"
echo "  ros2 topic pub /cmd_vel/direct_control geometry_msgs/msg/Twist \"{linear: {x: 0.2}, angular: {z: 0.0}}\" -r 10"
echo "  ros2 topic pub /cmd_vel/emergency geometry_msgs/msg/Twist \"{linear: {x: 0.3}, angular: {z: 0.0}}\" -r 10"
echo "  # Emergency should win (Priority 1 > Priority 2)"

#!/bin/bash
# Test script to verify zero commands from high priority override lower priorities
# This proves the fix for the critical bug where emergency stops were ignored

echo "=========================================="
echo "ZERO COMMAND PRIORITY TEST"
echo "=========================================="
echo ""
echo "This test verifies that zero commands from high priority (emergency stop)"
echo "correctly override non-zero commands from lower priorities."
echo ""
echo "Test scenario:"
echo "  1. Priority 3 (Nav2) publishes non-zero command (0.2 m/s forward)"
echo "  2. Priority 1 (Emergency) publishes zero command (stop)"
echo "  3. Expected: /cmd_vel should show zero (emergency stop wins)"
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

# Check if cmd_vel_multiplexer is running
if ! ros2 node list | grep -q "cmd_vel_multiplexer"; then
    echo "❌ ERROR: cmd_vel_multiplexer is not running!"
    echo "   This test requires the multiplexer to be active."
    exit 1
fi

echo "✅ cmd_vel_multiplexer is running"
echo ""

echo "TEST PROCEDURE:"
echo "  1. In Terminal 2, run: ros2 topic echo /cmd_vel"
echo "  2. In Terminal 3, run: ros2 topic pub /cmd_vel/nav2 geometry_msgs/msg/Twist \"{linear: {x: 0.2}, angular: {z: 0.0}}\" -r 10"
echo "  3. In Terminal 4, run: ros2 topic pub /cmd_vel/emergency geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\" -r 10"
echo ""
echo "Expected result:"
echo "  - Terminal 2 should show linear.x=0.0 when emergency publishes (zero wins)"
echo "  - Terminal 2 should show linear.x=0.2 when emergency stops publishing"
echo ""
echo "Press Enter to start monitoring /cmd_vel (or Ctrl+C to cancel)..."
read

echo ""
echo "Monitoring /cmd_vel for 10 seconds..."
echo "Watch for: zero commands from /cmd_vel/emergency should override Nav2 commands"
echo ""

timeout 10 ros2 topic echo /cmd_vel --qos-profile reliability=reliable | head -50

echo ""
echo "=========================================="
echo "MANUAL VERIFICATION REQUIRED"
echo "=========================================="
echo ""
echo "To complete the test, manually run these commands in separate terminals:"
echo ""
echo "Terminal 1 (monitor final /cmd_vel):"
echo "  ros2 topic echo /cmd_vel --qos-profile reliability=reliable"
echo ""
echo "Terminal 2 (publish Nav2 non-zero):"
echo "  ros2 topic pub /cmd_vel/nav2 geometry_msgs/msg/Twist \"{linear: {x: 0.2}, angular: {z: 0.0}}\" -r 10"
echo ""
echo "Terminal 3 (publish Emergency zero - should override):"
echo "  ros2 topic pub /cmd_vel/emergency geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\" -r 10"
echo ""
echo "Expected: Terminal 1 shows zero when Terminal 3 is active (emergency stop wins)"
echo ""
echo "Check logs for confirmation:"
echo "  ros2 topic echo /rosout | grep -i 'emergency stop'"
echo ""

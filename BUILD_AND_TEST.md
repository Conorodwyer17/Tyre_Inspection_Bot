# BUILD AND TEST INSTRUCTIONS

## CRITICAL FIXES APPLIED

This document contains the fixes applied to implement priority-based cmd_vel arbitration.

## BUILD INSTRUCTIONS

```bash
cd /home/jetson/ugv_ws

# Build the package
colcon build --packages-select tyre_inspection_mission

# Source the workspace
source install/setup.bash
```

## TEST INSTRUCTIONS

### Step 1: Verify cmd_vel Arbitration Setup

```bash
# Run verification script
bash src/amr_hardware/src/tyre_inspection_mission/scripts/verify_cmd_vel_setup.sh

# Expected output:
# ✅ ROS 2 is running
# ✅ /cmd_vel topic exists
# ✅ Publisher count: 1 (cmd_vel_multiplexer)
# ✅ Priority topics exist
# ✅ cmd_vel_multiplexer node is running
```

### Step 2: Start the System

```bash
# Start the system
ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true

# Expected nodes:
# - cmd_vel_multiplexer (MUST start first)
# - nav2_navigation_with_remap (custom launch with remapping)
# - mission_controller
# - direct_navigation_fallback
# - movement_guarantee
```

### Step 3: Verify cmd_vel Topics

In another terminal:

```bash
# Verify only multiplexer publishes to /cmd_vel
ros2 topic info /cmd_vel
# Expected: Publisher count: 1 (cmd_vel_multiplexer)

# Verify priority topics exist
ros2 topic list | grep cmd_vel
# Expected:
#   /cmd_vel
#   /cmd_vel/emergency (Priority 1)
#   /cmd_vel/direct_control (Priority 2)
#   /cmd_vel/nav2 (Priority 3)
#   /cmd_vel/teleop (Priority 4)

# Verify Nav2 remapped correctly
ros2 topic info /cmd_vel/nav2
# Expected: Publisher: controller_server (when Nav2 navigating)

# Monitor final output
ros2 topic echo /cmd_vel --qos-profile reliability=reliable
ros2 topic hz /cmd_vel
# Expected: 50Hz from multiplexer
```

### Step 4: Start Mission and Verify Movement

```bash
# Start mission
ros2 service call /mission_controller/start std_srvs/srv/Trigger

# Monitor mission status
ros2 topic echo /mission_controller/status

# Monitor cmd_vel output
ros2 topic echo /cmd_vel --qos-profile reliability=reliable

# Monitor priority topics
ros2 topic echo /cmd_vel/direct_control
ros2 topic echo /cmd_vel/emergency

# Monitor odometry
ros2 topic echo /odom
ros2 topic hz /odom
# Expected: 50-100Hz updates
```

### Step 5: Verify Movement Verification

```bash
# Run movement diagnostic
ros2 run tyre_inspection_mission movement_diagnostic

# Expected output:
# ✅ cmd_vel: freq=~50Hz, active=True, non_zero=True
# ✅ odom: freq=50-100Hz, active=True, moving=True
# ✅ Robot is moving: distance_moved > 0
```

### Step 6: Verify Priority Arbitration

```bash
# Test emergency priority (should win)
ros2 topic pub /cmd_vel/emergency geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10

# Monitor final output
ros2 topic echo /cmd_vel --qos-profile reliability=reliable
# Expected: linear.x=0.3 (from emergency)

# Test direct control priority (should win if emergency stops)
ros2 topic pub /cmd_vel/direct_control geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10

# Stop emergency
# (Ctrl+C or stop publishing)

# Monitor final output
ros2 topic echo /cmd_vel --qos-profile reliability=reliable
# Expected: linear.x=0.2 (from direct_control, emergency stopped)
```

## VERIFICATION CHECKLIST

- [ ] cmd_vel_multiplexer node is running
- [ ] Only ONE publisher to /cmd_vel (multiplexer)
- [ ] Priority topics exist (/cmd_vel/emergency, /cmd_vel/direct_control, /cmd_vel/nav2)
- [ ] Nav2 remapped to /cmd_vel/nav2 (check with `ros2 topic info /cmd_vel/nav2`)
- [ ] QoS matches across all publishers/subscribers (RELIABLE)
- [ ] Mission starts successfully
- [ ] Robot moves when mission starts
- [ ] Movement diagnostic shows robot is moving
- [ ] Priority arbitration works (emergency > direct > nav2 > teleop)

## TROUBLESHOOTING

### Issue: Multiple publishers to /cmd_vel

**Diagnosis:**
```bash
ros2 topic info /cmd_vel
# Check Publisher count
```

**Solution:**
- Verify cmd_vel_multiplexer is running
- Check if Nav2 is publishing to /cmd_vel directly (should be /cmd_vel/nav2)
- Verify custom nav2_navigation_with_remap.launch.py is being used

### Issue: Robot not moving

**Diagnosis:**
```bash
# Check cmd_vel output
ros2 topic echo /cmd_vel --qos-profile reliability=reliable

# Check priority topics
ros2 topic echo /cmd_vel/direct_control
ros2 topic echo /cmd_vel/emergency

# Check odometry
ros2 topic echo /odom
```

**Solution:**
- Verify direct_navigation_fallback.activate() was called
- Verify movement_guarantee.activate() was called
- Check node logs for errors
- Verify ugv_bringup is receiving cmd_vel (check QoS matches)

### Issue: Nav2 not remapped correctly

**Diagnosis:**
```bash
# Check if Nav2 controller publishes to /cmd_vel/nav2
ros2 topic info /cmd_vel/nav2

# Check if Nav2 controller still publishes to /cmd_vel directly
ros2 topic info /cmd_vel
# If Publisher count > 1, Nav2 might not be remapped
```

**Solution:**
- Verify custom nav2_navigation_with_remap.launch.py is being used
- Check launch file to ensure it includes nav2_navigation_with_remap.launch.py
- Verify Nav2 controller node has remapping in custom launch

### Issue: QoS mismatch

**Diagnosis:**
```bash
ros2 topic info /cmd_vel --verbose
# Check Reliability and Durability settings
```

**Solution:**
- Verify all publishers use RELIABLE, TRANSIENT_LOCAL, depth=20
- Verify ugv_bringup subscription uses RELIABLE, TRANSIENT_LOCAL, depth=20

## NEXT STEPS

After verifying cmd_vel arbitration works:
1. Test with real robot
2. Enhance movement verification (Day 2 of plan)
3. Continue with remaining fixes from plan

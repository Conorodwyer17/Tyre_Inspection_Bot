# Iteration 2: Direct Publishers & Odometry QoS Fixes

## Step 1 — Reality Check (What Would Fail RIGHT NOW?)

### Critical Issue #1: Direct Publishers Bypassing Multiplexer ✅ FIXED
**Problem**: Three manual control nodes published directly to `/cmd_vel`, bypassing the priority-based multiplexer:
- `behavior_ctrl.py` line 32: Published to `/cmd_vel`
- `joy_ctrl.py` line 32: Published to `/cmd_vel`
- `keyboard_ctrl.py` line 67: Published to `/cmd_vel`

**Impact**: These nodes could override the multiplexer's output, causing unpredictable robot behavior. Priority arbitration would fail.

**Fix Applied**:
- Changed all three nodes to publish to `/cmd_vel/teleop` (Priority 4)
- Added comments explaining the priority system

**Files Changed**:
- `/home/jetson/ugv_ws/src/ugv_main/ugv_tools/ugv_tools/behavior_ctrl.py`
- `/home/jetson/ugv_ws/src/ugv_main/ugv_tools/ugv_tools/joy_ctrl.py`
- `/home/jetson/ugv_ws/src/ugv_main/ugv_tools/ugv_tools/keyboard_ctrl.py`
- `/home/jetson/ugv_ws/src/amr_hardware/src/tyre_inspection_mission/scripts/verify_cmd_vel_setup.sh` (enhanced verification)

### Critical Issue #2: Odometry QoS Mismatch ✅ FIXED
**Problem**: 
- `base_node.cpp` published `/odom` with default QoS (BEST_EFFORT, depth=5)
- `movement_guarantee.py` subscribed with default QoS (BEST_EFFORT, depth=10)
- Messages could be dropped under load, causing movement verification to fail

**Impact**: If odometry messages are dropped, `movement_guarantee` may incorrectly think the robot isn't moving when it actually is, leading to false emergency interventions or missed stuck detection.

**Fix Applied**:
- Updated `base_node.cpp` to publish `/odom` with RELIABLE QoS + TRANSIENT_LOCAL durability
- Updated `movement_guarantee.py` to subscribe with matching RELIABLE QoS + TRANSIENT_LOCAL durability
- Increased depth to 20 for both to match cmd_vel QoS

**Files Changed**:
- `/home/jetson/ugv_ws/src/ugv_main/ugv_base_node/src/base_node.cpp`
- `/home/jetson/ugv_ws/src/amr_hardware/src/tyre_inspection_mission/tyre_inspection_mission/core/movement_guarantee.py`

## Step 2 — Trace the Exact Code + Topic Path

### Direct Publishers Issue:
1. **Publisher**: `behavior_ctrl.py` line 32 → `create_publisher(Twist, '/cmd_vel', 10)`
2. **Publisher**: `joy_ctrl.py` line 32 → `create_publisher(Twist, 'cmd_vel', 10)`
3. **Publisher**: `keyboard_ctrl.py` line 67 → `create_publisher(Twist, 'cmd_vel', 1)`
4. **Conflict**: These publish directly to `/cmd_vel`, bypassing `cmd_vel_multiplexer`
5. **Result**: Multiplexer publishes to `/cmd_vel`, but these nodes also publish directly, causing race conditions

### Odometry QoS Issue:
1. **Publisher**: `base_node.cpp` line 108 → `create_publisher<nav_msgs::msg::Odometry>("odom", 5)`
   - Uses default QoS: BEST_EFFORT, VOLATILE, depth=5
2. **Subscriber**: `movement_guarantee.py` line 54 → `create_subscription(Odometry, '/odom', self.odom_callback, 10)`
   - Uses default QoS: BEST_EFFORT, VOLATILE, depth=10
3. **Problem**: QoS compatibility, but BEST_EFFORT means messages can be dropped under load
4. **Impact**: Movement verification depends on accurate odometry - dropped messages cause false positives

## Step 3 — Implement the Fix

### Direct Publishers Fix:
Changed all three nodes to publish to `/cmd_vel/teleop` instead of `/cmd_vel`:
```python
# OLD:
self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

# NEW:
# CRITICAL: Publish to /cmd_vel/teleop (Priority 4) instead of /cmd_vel directly
# The cmd_vel_multiplexer will arbitrate and publish final /cmd_vel
self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel/teleop', 10)
```

### Odometry QoS Fix:
**base_node.cpp**:
```cpp
// OLD:
odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);

// NEW:
// CRITICAL: Publisher for odometry messages with RELIABLE QoS
// Movement verification depends on accurate odometry - messages must not be dropped
// RELIABLE + TRANSIENT_LOCAL ensures subscribers get all messages even if they connect late
odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::QoS(rclcpp::KeepLast(20))
        .reliable()
        .transient_local()
);
```

**movement_guarantee.py**:
```python
# OLD:
self.odom_sub = node.create_subscription(Odometry, '/odom', self.odom_callback, 10)

# NEW:
# CRITICAL: Subscriber for odometry with RELIABLE QoS to match base_node publisher
# Movement verification depends on accurate odometry - messages must not be dropped
odom_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Match base_node publisher QoS
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Match base_node publisher QoS
    history=HistoryPolicy.KEEP_LAST,
    depth=20  # Increased queue depth to match publisher
)
self.odom_sub = node.create_subscription(Odometry, '/odom', self.odom_callback, odom_qos)
```

## Step 4 — Add Proof

### CLI Commands to Verify Direct Publishers Fix:

```bash
# 1. Verify ONLY cmd_vel_multiplexer publishes to /cmd_vel
ros2 topic info /cmd_vel --verbose | grep -A 20 "Publisher"

# Expected output: Only "cmd_vel_multiplexer" should appear as a publisher

# 2. Verify manual control nodes publish to /cmd_vel/teleop
# Start a manual control node (e.g., joy_ctrl) and check:
ros2 topic info /cmd_vel/teleop --verbose | grep -A 20 "Publisher"

# Expected output: Manual control node (e.g., "joy_ctrl") should appear as publisher

# 3. Test priority arbitration manually:
# Terminal 1: Monitor final /cmd_vel
ros2 topic echo /cmd_vel --qos-profile reliability=reliable

# Terminal 2: Publish to /cmd_vel/teleop (Priority 4)
ros2 topic pub /cmd_vel/teleop geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# Terminal 3: Publish to /cmd_vel/emergency (Priority 1 - should win)
ros2 topic pub /cmd_vel/emergency geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10

# Expected: Terminal 1 should show emergency commands (0.3), not teleop commands (0.1)

# 4. Use the verification script:
bash /home/jetson/ugv_ws/src/amr_hardware/src/tyre_inspection_mission/scripts/verify_cmd_vel_setup.sh
```

### CLI Commands to Verify Odometry QoS Fix:

```bash
# 1. Verify base_node publishes /odom with RELIABLE QoS
ros2 topic info /odom --verbose

# Expected output should show:
# Reliability: RELIABLE
# Durability: TRANSIENT_LOCAL
# Depth: 20

# 2. Verify movement_guarantee subscribes with matching QoS
# Check node's subscription info (requires running system):
ros2 node info /movement_guarantee 2>/dev/null | grep -A 10 "Subscribers" | grep "/odom"

# 3. Test message delivery under load:
# Terminal 1: Monitor odom messages (should see all messages)
ros2 topic echo /odom --qos-profile reliability=reliable

# Terminal 2: Publish high-frequency fake odom to test (if needed):
# This would require a test publisher node - not included here

# 4. Verify no message loss:
ros2 topic hz /odom

# Expected: Should show consistent ~10 Hz (base_node publishes at 10 Hz)
# If messages are being dropped, the rate will fluctuate or be lower
```

### Logging to Expose Issues:

**Direct Publishers**: The enhanced `verify_cmd_vel_setup.sh` script now detects direct publishers:
```bash
bash /home/jetson/ugv_ws/src/amr_hardware/src/tyre_inspection_mission/scripts/verify_cmd_vel_setup.sh
```

**Odometry QoS**: Add logging to `movement_guarantee.py` to detect missing odom messages:
```python
# Already present: movement_guarantee logs when odom is stale
# Check logs for: "⚠️ No odometry received for X seconds"
```

## Step 5 — Decide the Next Risk

### Next Weakest Links (In Priority Order):

1. **Nav2 Params File Loading** (Status: VERIFY)
   - Custom launch file uses `LaunchConfiguration('params_file')` with `RewrittenYaml`
   - Main launch passes `nav2_params` as a string via `launch_arguments`
   - Need to verify this actually works at runtime (may need testing or code review)

2. **Direct Navigation Fallback Update Frequency** (Status: INVESTIGATE)
   - `direct_navigation_fallback` calls `update()` from mission_controller timer
   - Need to verify update frequency is high enough (50Hz target)
   - If called too infrequently, may not respond fast enough to Nav2 failures

3. **Movement Guarantee Timeout Values** (Status: REVIEW)
   - `movement_timeout = 2.0` seconds - is this appropriate for real robot?
   - May need tuning based on actual robot characteristics
   - Too short = false positives, too long = delayed intervention

4. **cmd_vel Staleness Detection** (Status: VERIFY)
   - `cmd_vel_multiplexer` uses `command_timeout = 1.0` second
   - Need to verify this is appropriate for all priority levels
   - Emergency (Priority 1) may need shorter timeout, teleop (Priority 4) may need longer

### Recommended Next Step:
**Verify Nav2 params_file loading works correctly** - This is critical for Nav2 to function. If params don't load, Nav2 won't work at all.

## Summary

**Fixed This Iteration**:
- ✅ All manual control publishers now use `/cmd_vel/teleop` (Priority 4)
- ✅ `/odom` QoS upgraded to RELIABLE + TRANSIENT_LOCAL for accurate movement verification
- ✅ Enhanced verification script to detect direct publishers

**Remaining Risks**:
- ⚠️ Nav2 params_file loading (needs verification)
- ⚠️ Direct navigation fallback update frequency (needs investigation)
- ⚠️ Movement guarantee timeout tuning (needs real-world testing)
- ⚠️ cmd_vel staleness detection (needs verification)

**Total Fixes Applied (All Iterations)**:
- ✅ cmd_vel_multiplexer (Priority-based arbitration)
- ✅ Nav2 remapping to `/cmd_vel/nav2`
- ✅ Direct navigation fallback remapping to `/cmd_vel/direct_control`
- ✅ Movement guarantee remapping to `/cmd_vel/emergency`
- ✅ All manual control nodes remapping to `/cmd_vel/teleop`
- ✅ `/cmd_vel` QoS: RELIABLE + TRANSIENT_LOCAL
- ✅ `/odom` QoS: RELIABLE + TRANSIENT_LOCAL
- ✅ ugv_bringup cmd_vel subscription QoS: RELIABLE

**Next Action**: Verify Nav2 params_file loading, then test system on real robot.

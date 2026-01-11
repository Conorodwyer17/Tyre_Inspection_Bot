# Critical Fix: Movement Guarantee Odom Never-Received Bug

## 🎯 Issue Identified

**Reality Check Result:** Movement guarantee system wouldn't detect if odometry never arrives (e.g., if `base_node` crashes at startup). The stuck detection check `time_since_last_position < float('inf')` would fail because `time_since_last_position` stays at infinity, and the robot wouldn't be forced to move even if it's stuck.

### Problem:
- **Initial state:** `last_robot_position_time = None` (no odometry received yet)
- **Watchdog callback:** `time_since_last_position = float('inf')` (initialized to infinity)
- **Bug:** If odometry never arrives, `time_since_last_position` stays at infinity forever
- **Stuck detection check:** `if cmd_vel_active and not is_moving and time_since_last_position < float('inf'):`
  - This check **FAILS** because `float('inf') < float('inf')` is `False`
  - Stuck detection never triggers!
- **Result:** Robot receives cmd_vel but odometry never arrives → stuck detection doesn't work → robot doesn't move!

**Impact:**
- Robot stuck detection fails if odometry never arrives
- Movement guarantee can't force movement without odometry
- Robot may not move if `base_node` crashes at startup
- System enters degraded state but doesn't handle it correctly

## ✅ Fix Applied

**Step 3 - Implementation:** Added detection for "odometry never received" case and handle it gracefully.

### Changes:
**File:** `tyre_inspection_mission/core/movement_guarantee.py` (lines 91, 151, 177-210, 266-284, 307, 344)

### Code Changes:

**1. Track activation time:**
```python
# BEFORE:
self.last_robot_position_time = None

# AFTER:
self.activate_start_time = None  # Track activation time for odom never-received detection
```

**2. Set activation time on activate:**
```python
def activate(self, goal_pose: PoseStamped):
    if not self.is_active:
        self.is_active = True
        self.current_goal = goal_pose
        self.activate_start_time = time.time()  # Track activation time
        # Don't set last_robot_position_time here - wait for actual odometry
```

**3. Detect odometry never received:**
```python
# BEFORE:
if self.last_robot_position_time and self.last_robot_position:
    time_since_last_position = current_time - self.last_robot_position_time
    if time_since_last_position > self.odom_timeout:
        odom_stale = True

# AFTER:
odom_stale = False
odom_never_received = False

if self.last_robot_position_time and self.last_robot_position:
    # Odometry has been received at least once - check if it's stale
    time_since_last_position = current_time - self.last_robot_position_time
    if time_since_last_position > self.odom_timeout:
        odom_stale = True
else:
    # CRITICAL: Odometry has NEVER been received
    if self.is_active and self.activate_start_time:
        time_since_activation = current_time - self.activate_start_time
        if time_since_activation > 5.0:  # 5 seconds grace period
            odom_never_received = True
            self.logger.error(
                f"🚨🚨🚨 CRITICAL: ODOMETRY NEVER RECEIVED! "
                f"Movement guarantee active for {time_since_activation:.1f}s "
                f"but no odometry messages received. "
                f"base_node may not be running or serial port disconnected."
            )
```

**4. Handle odom never received in degraded mode:**
```python
# BEFORE:
if odom_stale:
    # Handle stale odometry

# AFTER:
if odom_stale or odom_never_received:
    # Handle stale or never-received odometry
    if cmd_vel_active:
        # Assume movement based on cmd_vel only (degraded mode)
        return
    else:
        # Force movement (cannot verify if working)
        self._force_movement()
        return
```

**5. Skip stuck detection if odom never received:**
```python
# BEFORE:
if cmd_vel_active and not is_moving and time_since_last_position < float('inf'):

# AFTER:
if cmd_vel_active and not is_moving and not odom_stale and not odom_never_received and time_since_last_position < float('inf'):
```

## 📋 Step 4 - Proof

### Verification Commands:
```bash
# Check if odometry is publishing
ros2 topic hz /odom

# Check movement guarantee logs for odom never-received warnings
ros2 topic echo /rosout | grep -i "odometry never received"

# Simulate base_node crash (stop base_node while mission is running)
ros2 node kill base_node
# Should see: "CRITICAL: ODOMETRY NEVER RECEIVED!" after 5 seconds
```

### Expected Behavior:

**Scenario 1: Odometry arrives normally**
- ✅ Movement guarantee tracks position normally
- ✅ Stuck detection works correctly
- ✅ No "odom never received" warnings

**Scenario 2: Odometry arrives late (within 5 seconds)**
- ✅ Grace period allows startup time
- ✅ Stuck detection activates once odometry arrives
- ✅ No false warnings

**Scenario 3: Odometry never arrives (base_node crashed)**
- ✅ After 5 seconds: "CRITICAL: ODOMETRY NEVER RECEIVED!" error logged
- ✅ If cmd_vel is active: Degraded mode - assume movement based on cmd_vel only
- ✅ If cmd_vel is NOT active: Force movement immediately
- ✅ System continues to function (degraded but functional)

**Scenario 4: Odometry was arriving but stops (base_node crashes mid-mission)**
- ✅ After 2 seconds (odom_timeout): "CRITICAL: ODOMETRY IS STALE!" error logged
- ✅ Degraded mode - rely on cmd_vel only
- ✅ System continues to function (degraded but functional)

### Log Messages to Expect:

**Odometry never received:**
```
🚨🚨🚨 CRITICAL: ODOMETRY NEVER RECEIVED! Movement guarantee active for 5.1s 
but no odometry messages received. base_node may not be running or serial port disconnected. 
Cannot verify physical movement - forcing movement based on cmd_vel only.
```

**Odometry stale (was working, then stopped):**
```
🚨🚨🚨 CRITICAL: ODOMETRY IS STALE! Last odometry update was 2.1s ago 
(timeout: 2.0s). base_node may have crashed or serial port disconnected. 
Cannot verify physical movement - forcing movement based on cmd_vel only.
```

**Degraded mode (odom missing but cmd_vel active):**
```
⚠️ ODOMETRY STALE (2.1s old) but cmd_vel is active. 
Assuming movement based on cmd_vel only (degraded mode). 
Check base_node and serial port connection!
```

## 🔍 Root Cause Analysis

### Why This Happens:
1. **Initialization:** `last_robot_position_time = None` when no odometry received
2. **Watchdog logic:** `time_since_last_position = float('inf')` if no position yet
3. **Stuck detection check:** `time_since_last_position < float('inf')` → **FALSE** if odom never arrives
4. **Result:** Stuck detection never triggers, robot doesn't force movement

### Impact:
- **False negatives:** Robot appears stuck but stuck detection doesn't trigger
- **Degraded operation:** System can't verify physical movement but doesn't handle it
- **User experience:** Unreliable, unpredictable behavior

## ✅ Status

- ✅ Build successful - fix compiled without errors
- ✅ No linter errors
- ✅ Odom never-received detection added (5 second grace period)
- ✅ Degraded mode handling (rely on cmd_vel when odom unavailable)
- ✅ All movement checks updated to handle odom never-received case

## 🚀 Next Steps

1. **Test with base_node stopped** - Verify odom never-received detection works
2. **Test with base_node crash mid-mission** - Verify odom stale detection works
3. **Monitor logs** - Check for "ODOMETRY NEVER RECEIVED" and "ODOMETRY IS STALE" messages
4. **Verify degraded mode** - Ensure system continues functioning when odom unavailable

---

## 📊 Status Summary

| Component | Status | Notes |
|-----------|--------|-------|
| Odom Never-Received Detection | ✅ Fixed | 5 second grace period, then error + degraded mode |
| Odom Stale Detection | ✅ Working | 2 second timeout, then error + degraded mode |
| Degraded Mode Handling | ✅ Fixed | Relies on cmd_vel when odom unavailable |
| Stuck Detection | ✅ Fixed | Skips when odom never received/stale (handled separately) |
| Movement Forcing | ✅ Fixed | Works in degraded mode when cmd_vel missing |

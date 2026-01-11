# Iteration 5: Command Effectiveness Instrumentation

## Step 1 — Reality Check (What Would Fail RIGHT NOW?)

### Critical Issue: Small Commands May Be Below Hardware Deadzone ✅ EXPOSED WITH INSTRUMENTATION

**Problem**: Nav2 can publish very small cmd_vel commands (e.g., 0.001 m/s), which scale to extremely small PWM values (0.000385 PWM). ESP32 firmware may have a deadzone and ignore commands below a threshold (estimated ~0.01 PWM), causing robot to not move even when commands are sent.

**Concrete Failure Scenario**:
1. Nav2 publishes cmd_vel: `linear.x = 0.001 m/s` (just above `min_x_velocity_threshold`)
2. ugv_bringup scales: `(0.001 / 1.3) * 0.5 = 0.000385 PWM`
3. **HIDDEN FAILURE**: ESP32 firmware ignores commands < ~0.01 PWM (deadzone)
4. **RESULT**: Robot doesn't move, Nav2 thinks it's moving (no feedback loop), navigation fails

**Impact**: Robot appears to be following commands but actually isn't moving. Navigation gets stuck, mission fails.

## Step 2 — Trace the Exact Code + Topic Path

### File: `/home/jetson/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/ugv_bringup.py`

**Issue Location**: Lines 691-705 (command scaling)

**Calculation**:
- Nav2 `min_x_velocity_threshold: 0.001` m/s (slam_nav.yaml line 74)
- Scaling formula: `PWM = (m/s / MAX_ROBOT_SPEED_MPS) * MAX_PWM_COMMAND`
- For 0.001 m/s: `(0.001 / 1.3) * 0.5 = 0.000385 PWM`
- ESP32 firmware likely deadzone: ~0.01 PWM (20x larger than scaled command)

**Topic Flow**:
1. Nav2 controller → `/cmd_vel/nav2` (Priority 3) → cmd_vel_multiplexer → `/cmd_vel`
2. `/cmd_vel` → `ugv_bringup.cmd_vel_callback()` → scaling → serial command
3. Serial → ESP32 firmware → motor control
4. **Missing**: No feedback loop to detect if ESP32 actually executed command
5. **Hidden State**: We don't know if small commands are ignored by hardware

## Step 3 — Implement the Fix

### Fix Applied: Added Instrumentation to Expose Hidden State

**1. Added Diagnostics in ugv_bringup.py** (Lines 707-728):
- Detect when commands scale to effectively zero (< 0.01 PWM)
- Warn when non-zero cmd_vel scales to below hardware deadzone
- Log scaled PWM values for debugging

**2. Created Command Effectiveness Monitor Node** (`command_effectiveness_monitor.py`):
- Monitors `/cmd_vel` commands and `/odom` feedback
- Correlates commands sent with actual robot movement
- Detects when commands are sent but robot doesn't move
- Reports effectiveness statistics (commands with/without movement)

**Key Features**:
- Tracks command-to-movement correlation
- Warns when commands don't result in movement
- Identifies small commands that may be below deadzone
- Periodic effectiveness reports

## Step 4 — Add Proof

### CLI Commands to Verify Instrumentation:

```bash
# 1. Start command effectiveness monitor
ros2 run tyre_inspection_mission command_effectiveness_monitor

# 2. In another terminal, check for warnings about effectively zero commands
ros2 topic echo /rosout | grep -i "EFFECTIVELY ZERO\|COMMAND INEFFECTIVE\|Small cmd_vel"

# 3. Monitor command effectiveness reports (every 10 seconds)
ros2 topic echo /rosout | grep -i "Command Effectiveness Report"

# 4. Test with small command to verify deadzone detection
ros2 topic pub /cmd_vel/nav2 geometry_msgs/msg/Twist "{linear: {x: 0.001}, angular: {z: 0.0}}" -r 10

# Expected: Should see warning "EFFECTIVELY ZERO" from ugv_bringup
# Expected: Should see "COMMAND INEFFECTIVE" from monitor if robot doesn't move

# 5. Test with larger command to verify movement
ros2 topic pub /cmd_vel/nav2 geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# Expected: Should see robot move (check odom), no warnings

# 6. Verify scaling calculations in logs
ros2 topic echo /rosout | grep -i "Scaling:" | head -20

# Expected: See actual scaling calculations showing m/s → PWM conversion
```

### Logging Added:

**ugv_bringup warnings**:
```
⚠️ EFFECTIVELY ZERO: cmd_vel linear=0.0010 m/s, angular=0.0000 rad/s 
→ scaled to L=0.000385, R=0.000385 PWM (< 0.01). 
ESP32 may ignore this (deadzone). Robot may not move.
```

**command_effectiveness_monitor warnings**:
```
⚠️ COMMAND INEFFECTIVE: cmd_vel sent (linear=0.0010, angular=0.0000) 
but robot didn't move. Distance moved: 0.00 cm, velocity: 0.0000 m/s. 
Command may be below hardware deadzone or hardware issue.
```

**Periodic effectiveness reports**:
```
📊 Command Effectiveness Report:
   Commands sent: 100
   Commands with movement: 85 (85.0%)
   Commands without movement: 15 (15.0% - may indicate deadzone/hardware issues)
   Small commands (< 0.050 m/s): 12
```

### Verification Checklist:

- [x] ugv_bringup warns about effectively zero commands
- [x] command_effectiveness_monitor tracks cmd_vel → odom correlation
- [x] Detects when commands don't result in movement
- [x] Reports effectiveness statistics periodically
- [x] Exposes hidden deadzone/hardware issues

## Step 5 — Decide the Next Risk

### Next Weakest Links (In Priority Order):

1. **Hardware Deadzone Confirmation** (Status: NEEDS HARDWARE TEST)
   - We estimate deadzone at ~0.01 PWM, but need to verify with actual robot
   - **Risk**: If deadzone is different, our warnings may be inaccurate
   - **Action**: Test with real robot to determine actual deadzone threshold

2. **Nav2 Minimum Velocity Threshold** (Status: REVIEW NEEDED)
   - Current: `min_x_velocity_threshold: 0.001` m/s
   - Scaled: `0.001 m/s → 0.000385 PWM` (below estimated deadzone)
   - Should be: `≥ 0.026 m/s` to get `≥ 0.01 PWM` (above deadzone)
   - **Risk**: Nav2 may publish commands that hardware ignores
   - **Action**: Increase Nav2 min thresholds OR add deadzone filtering in ugv_bringup

3. **Command Deadzone Filtering** (Status: OPTIONAL IMPROVEMENT)
   - Currently: Send all commands, warn about small ones
   - Option: Filter commands below deadzone (round to zero)
   - **Risk**: If we filter, Nav2 won't know robot stopped (no feedback)
   - **Action**: Keep warnings, let Nav2 adjust (or increase Nav2 thresholds)

4. **Odometry Feedback Loop** (Status: VERIFY)
   - command_effectiveness_monitor uses odom to verify movement
   - Need to verify odom reflects actual hardware movement correctly
   - **Risk**: If odom is wrong, effectiveness detection is wrong
   - **Action**: Verify odom accuracy (already fixed QoS, but need hardware test)

### Recommended Next Step:
**Hardware deadzone confirmation** - Test with real robot to determine actual ESP32 deadzone threshold. Then adjust Nav2 min thresholds or add deadzone filtering accordingly.

## Summary

**Added This Iteration**:
- ✅ Instrumentation to detect effectively zero commands (ugv_bringup warnings)
- ✅ Command effectiveness monitor node (cmd_vel → odom correlation)
- ✅ Effectiveness statistics reporting
- ✅ Exposed hidden deadzone/hardware issues

**Critical Impact**:
- **Before**: No visibility into whether commands actually result in movement
- **After**: Clear visibility into command effectiveness, deadzone detection, hardware issues

**Files Changed**:
- `/home/jetson/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/ugv_bringup.py`
  - Added effectively zero command detection and warnings
  - Added detailed scaling calculation logging
- `/home/jetson/ugv_ws/src/amr_hardware/src/tyre_inspection_mission/tyre_inspection_mission/diagnostics/command_effectiveness_monitor.py` (new)
  - New diagnostic node for command effectiveness monitoring
- `/home/jetson/ugv_ws/src/amr_hardware/src/tyre_inspection_mission/setup.py`
  - Added command_effectiveness_monitor entry point

**Total Critical Fixes Applied (All Iterations)**:
- ✅ cmd_vel_multiplexer priority arbitration
- ✅ Zero command priority bug fix (emergency stop override)
- ✅ Nav2 remapping to `/cmd_vel/nav2`
- ✅ Direct navigation fallback remapping to `/cmd_vel/direct_control`
- ✅ Movement guarantee remapping to `/cmd_vel/emergency`
- ✅ All manual control nodes remapping to `/cmd_vel/teleop`
- ✅ `/cmd_vel` QoS: RELIABLE + TRANSIENT_LOCAL
- ✅ `/odom` QoS: RELIABLE + TRANSIENT_LOCAL
- ✅ Command queue lag fix (latest command only)
- ✅ Command effectiveness instrumentation (deadzone detection)

**Next Action**: Hardware deadzone confirmation - test with real robot to verify actual ESP32 deadzone threshold, then adjust Nav2 min thresholds or add deadzone filtering.

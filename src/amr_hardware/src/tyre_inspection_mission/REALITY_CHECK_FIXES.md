# Reality Check Fixes - Bottom-Up Navigation Verification

**Date:** 2025-01-XX  
**Objective:** Fix concrete failure modes identified through bottom-up code tracing

## Fixes Applied

### 1. Robot-Relative Goal Calculation Edge Case Fix ✅

**Problem:** At exactly 0.5m distance, the condition `if distance_robot_to_vehicle < 0.5` was inconsistent - robot at 0.499m would use vehicle orientation, but at 0.501m would use robot-relative. This created a discontinuity.

**Fix:** Changed to `<=` for consistency and added `MIN_RELIABLE_DISTANCE` constant for clarity.

**File:** `tyre_inspection_mission/core/mission_controller.py` (lines 2950-2971)

**Impact:** Consistent behavior at the 0.5m threshold. Robot-relative calculation works reliably from ANY angle (front, side, rear, diagonal) when distance > 0.5m.

**Verification:**
```bash
# Test robot at exactly 0.5m, 0.49m, and 0.51m distances
# All should behave consistently
```

### 2. Direct Navigation Watchdog Log Message Fix ✅

**Problem:** Watchdog log message incorrectly stated "20Hz" when timer is actually set to 50Hz (0.02s interval).

**Fix:** Updated log message to correctly state "50Hz (0.02s interval)".

**File:** `tyre_inspection_mission/navigation/direct_navigation_fallback.py` (line 414)

**Impact:** Accurate logging for debugging and verification.

### 3. Verification Scripts Created ✅

**Created:**
1. `scripts/verify_nav2_cmd_vel_remapping.sh` - Verifies Nav2 controller publishes to `/cmd_vel/nav2` (not `/cmd_vel`)
2. `scripts/verify_cmd_vel_pipeline.sh` - Verifies complete pipeline: priority topics → multiplexer → /cmd_vel → ugv_bringup

**Usage:**
```bash
# Verify Nav2 remapping
bash src/amr_hardware/src/tyre_inspection_mission/scripts/verify_nav2_cmd_vel_remapping.sh

# Verify cmd_vel pipeline
bash src/amr_hardware/src/tyre_inspection_mission/scripts/verify_cmd_vel_pipeline.sh
```

## Verified Working (From Code Review)

### ✅ Command Queue Fix
- **Location:** `ugv_bringup/ugv_bringup.py` (line 145)
- **Status:** Implemented - `queue.Queue(maxsize=1)` with `put_nowait()` to keep only latest command
- **Impact:** Eliminates command lag - robot always executes latest command

### ✅ Hardware Deadzone Clamping
- **Location:** `ugv_bringup/ugv_bringup.py` (lines 707-736)
- **Status:** Implemented - Commands below 0.01 PWM clamped to zero
- **Impact:** Prevents false "moving" state when commands below ESP32 deadzone

### ✅ TF Staleness Detection
- **Location:** `mission_controller.py` (lines 6026-6085)
- **Status:** Implemented - Rejects transforms older than 1.0s
- **Impact:** Prevents navigation failures from stale SLAM/localization data

### ✅ Odometry Staleness Detection
- **Location:** `movement_guarantee.py` (lines 100, 177-284)
- **Status:** Implemented - Detects odometry older than 2.0s, enters degraded mode
- **Impact:** Handles odometry loss gracefully, prevents false movement forcing

### ✅ Robot-Relative Goal Calculation
- **Location:** `mission_controller.py` (lines 2928-2971)
- **Status:** Implemented - Calculates approach direction from robot position
- **Impact:** Works from ANY angle (front, side, rear, diagonal)

### ✅ Direct Navigation Watchdog
- **Location:** `direct_navigation_fallback.py` (lines 62, 393-418)
- **Status:** Implemented - Publishes at 50Hz (0.02s interval) when active
- **Impact:** Ensures continuous movement even if state machine stalls

### ✅ Nav2 cmd_vel Remapping
- **Location:** `launch/nav2_navigation_with_remap.launch.py`
- **Status:** Implemented - Remaps controller_server, behavior_server, velocity_smoother to `/cmd_vel/nav2`
- **Impact:** Nav2 commands go through multiplexer as Priority 3

## Remaining Verification Needed (Requires Real Robot)

### ⚠️ Nav2 Remapping Verification
**Status:** Code looks correct, but needs runtime verification

**Test:**
```bash
# Start system
ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true

# Verify Nav2 publishes to /cmd_vel/nav2
ros2 topic info /cmd_vel/nav2 --verbose
# Should show controller_server as publisher

# Verify Nav2 does NOT publish to /cmd_vel directly
ros2 topic info /cmd_vel --verbose
# Should NOT show controller_server, behavior_server, or velocity_smoother
```

### ⚠️ Direct Navigation Watchdog Rate Verification
**Status:** Timer set to 50Hz, but needs verification of actual publish rate

**Test:**
```bash
# Activate direct navigation (start mission, wait for Nav2 timeout)
# Monitor publish rate
ros2 topic hz /cmd_vel/direct_control
# Should show ~50Hz when active
```

### ⚠️ Hardware Deadzone Confirmation
**Status:** Clamping implemented, but actual ESP32 deadzone threshold needs hardware test

**Test:**
```bash
# Test incremental PWM commands to find actual deadzone
# Update MIN_EFFECTIVE_PWM if different from 0.01
```

### ⚠️ TF Staleness Threshold Tuning
**Status:** 1.0s threshold implemented, but may need adjustment based on SLAM update rate

**Test:**
```bash
# Monitor SLAM update rate
ros2 topic hz /tf
# If SLAM updates slower than 1Hz, threshold may need adjustment
```

## Next Steps

1. **Run verification scripts** on real robot to confirm Nav2 remapping works
2. **Test robot-relative goal calculation** from multiple angles (front, side, rear, diagonal)
3. **Verify direct navigation watchdog** publishes at 50Hz when active
4. **Confirm hardware deadzone** threshold with incremental PWM test
5. **Tune TF staleness threshold** based on actual SLAM update rate

## Critical Path Verification

For the robot to work from ANY angle:

1. ✅ Robot-relative goal calculation implemented
2. ⚠️ Needs verification: Goal calculation works at all angles (0°, 90°, 180°, 45°)
3. ✅ TF staleness detection prevents wrong pose
4. ⚠️ Needs verification: TF updates frequently enough (< 1.0s)
5. ✅ Direct navigation fallback ensures continuous movement
6. ⚠️ Needs verification: Direct navigation activates when Nav2 fails
7. ✅ Command pipeline ensures latest commands reach hardware
8. ⚠️ Needs verification: Commands actually reach ESP32 and robot moves

**All fixes are code-complete. Real-world verification required.**

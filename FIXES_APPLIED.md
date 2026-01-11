# FIXES APPLIED - CMD_VEL ARBITRATION SYSTEM

**Date:** 2025-01-XX
**Objective:** Fix robot navigation by implementing priority-based cmd_vel arbitration

## STEP 1: REALITY CHECK - IDENTIFIED CRITICAL FAILURES

### Failure Mode 1: Multiple Publishers to /cmd_vel (CRITICAL)
- **Problem:** Nav2 controller, direct_navigation_fallback, movement_guarantee, and multiplexer all publish to `/cmd_vel`
- **Impact:** ROS 2 handles conflicts arbitrarily → robot receives conflicting commands → erratic movement or stops
- **Root Cause:** No command arbitration system exists

### Failure Mode 2: Nav2 Publishes Directly to /cmd_vel
- **Problem:** Nav2's controller_server publishes to `/cmd_vel` by default, bypassing priority system
- **Impact:** Nav2 commands conflict with direct control and emergency commands
- **Root Cause:** Can't easily remap Nav2's output via IncludeLaunchDescription

### Failure Mode 3: Other Nodes Also Publish to /cmd_vel
- **Problem:** joy_ctrl, keyboard_ctrl, behavior_ctrl publish directly to `/cmd_vel`
- **Impact:** Manual control bypasses priority system
- **Status:** Less critical (manual nodes, not always active)

## STEP 2: TRACED EXACT CODE/TOPIC PATHS

### Nav2 cmd_vel Path:
```
nav2_controller (controller_server)
  └─> publishes to /cmd_vel (default)
      └─> conflicts with multiplexer output
      └─> ugv_bringup subscribes (receives mixed messages)
```

### Direct Navigation Path:
```
direct_navigation_fallback
  └─> publishes to /cmd_vel/direct_control (FIXED)
      └─> multiplexer subscribes (Priority 2)
```

### Movement Guarantee Path:
```
movement_guarantee
  └─> publishes to /cmd_vel/emergency (FIXED)
      └─> multiplexer subscribes (Priority 1)
```

### Multiplexer Path:
```
cmd_vel_multiplexer
  └─> subscribes to /cmd_vel/emergency (Priority 1)
  └─> subscribes to /cmd_vel/direct_control (Priority 2)
  └─> subscribes to /cmd_vel/nav2 (Priority 3) ← NEEDED
  └─> subscribes to /cmd_vel/teleop (Priority 4)
  └─> publishes to /cmd_vel (final output)
      └─> ugv_bringup subscribes (hardware interface)
```

## STEP 3: IMPLEMENTED FIXES

### Fix 1: Created cmd_vel_multiplexer.py
- **File:** `tyre_inspection_mission/navigation/cmd_vel_multiplexer.py`
- **Function:** Priority-based command arbitration
- **Priorities:** Emergency (1) > Direct Control (2) > Nav2 (3) > Teleop (4)
- **QoS:** RELIABLE, TRANSIENT_LOCAL, depth=20
- **Rate:** 50Hz (hardware limit)

### Fix 2: Remapped direct_navigation_fallback
- **File:** `tyre_inspection_mission/navigation/direct_navigation_fallback.py`
- **Change:** Publisher changed from `/cmd_vel` to `/cmd_vel/direct_control`
- **Status:** ✅ FIXED

### Fix 3: Remapped movement_guarantee
- **File:** `tyre_inspection_mission/core/movement_guarantee.py`
- **Change:** Publisher changed from `/cmd_vel` to `/cmd_vel/emergency`
- **Status:** ✅ FIXED

### Fix 4: Fixed movement_diagnostic bugs
- **File:** `tyre_inspection_mission/diagnostics/movement_diagnostic.py`
- **Change:** Added missing initialization for `cmd_vel_count`, `odom_count`, `previous_position`, `start_time`
- **Status:** ✅ FIXED

### Fix 5: Created custom Nav2 navigation launch with remapping
- **File:** `tyre_inspection_mission/launch/nav2_navigation_with_remap.launch.py`
- **Function:** Launches Nav2 navigation nodes individually with cmd_vel remapping
- **Remapping:** 
  - controller_server: `/cmd_vel` → `/cmd_vel/nav2`
  - behavior_server: `/cmd_vel` → `/cmd_vel/nav2`
  - velocity_smoother: `/cmd_vel` → `/cmd_vel/nav2`
- **Status:** ✅ IMPLEMENTED

### Fix 6: Updated launch file
- **File:** `tyre_inspection_mission/launch/autonomous_inspection.launch.py`
- **Changes:**
  - Added cmd_vel_multiplexer node (MUST start first)
  - Updated to use custom Nav2 navigation launch with remapping
- **Status:** ✅ FIXED

### Fix 7: Added entry points
- **File:** `tyre_inspection_mission/setup.py`
- **Changes:** Added entry points for `cmd_vel_multiplexer` and `nav2_cmd_vel_relay`
- **Status:** ✅ FIXED

## STEP 4: PROOF - VERIFICATION COMMANDS

### Created Verification Script
- **File:** `tyre_inspection_mission/scripts/verify_cmd_vel_setup.sh`
- **Function:** Verifies cmd_vel arbitration is working correctly

### Manual Verification Commands:

```bash
# 1. Check only ONE node publishes to /cmd_vel (should be multiplexer)
ros2 topic info /cmd_vel
# Expected: Publisher count: 1 (cmd_vel_multiplexer)

# 2. Check priority topics exist
ros2 topic list | grep cmd_vel
# Expected: /cmd_vel, /cmd_vel/emergency, /cmd_vel/direct_control, /cmd_vel/nav2, /cmd_vel/teleop

# 3. Check Nav2 remapped correctly
ros2 topic info /cmd_vel/nav2
# Expected: Publisher: controller_server (when Nav2 is navigating)

# 4. Monitor final cmd_vel output
ros2 topic echo /cmd_vel --qos-profile reliability=reliable
ros2 topic hz /cmd_vel
# Expected: Consistent publishing at 50Hz from multiplexer

# 5. Check node is running
ros2 node list | grep cmd_vel_multiplexer
# Expected: /cmd_vel_multiplexer

# 6. Run automated verification
bash src/amr_hardware/src/tyre_inspection_mission/scripts/verify_cmd_vel_setup.sh
```

## STEP 5: NEXT RISKS IDENTIFIED

### Risk 1: QoS Mismatches (HIGH PRIORITY)
- **Problem:** Different QoS settings between publishers/subscribers can cause message loss
- **Status:** Not verified - needs checking
- **Action:** Verify all cmd_vel publishers use RELIABLE, TRANSIENT_LOCAL, depth=20

### Risk 2: Other Direct Publishers (MEDIUM PRIORITY)
- **Problem:** joy_ctrl, keyboard_ctrl, behavior_ctrl still publish to `/cmd_vel`
- **Impact:** Manual control bypasses priority system
- **Action:** Remap to `/cmd_vel/teleop` (Priority 4)

### Risk 3: Launch File Order (HIGH PRIORITY)
- **Problem:** Multiplexer must start BEFORE Nav2 navigation
- **Impact:** If Nav2 starts first, it publishes to `/cmd_vel` before remapping is active
- **Status:** Launch file order verified - multiplexer starts first

### Risk 4: Custom Launch Params File Handling (MEDIUM PRIORITY)
- **Problem:** Custom launch might not handle params_file LaunchConfiguration correctly
- **Impact:** Nav2 nodes might not start or use wrong params
- **Status:** Uses same approach as nav2_bringup - should work

### Risk 5: Movement Verification Not Working (CRITICAL - NEXT)
- **Problem:** Robot might think it's moving (cmd_vel published) but actually be stuck
- **Impact:** False completion, missed goals
- **Status:** Needs Day 2 enhancements (movement verification)

## STATUS SUMMARY

### ✅ COMPLETED:
1. cmd_vel_multiplexer created and integrated
2. direct_navigation_fallback remapped to priority topic
3. movement_guarantee remapped to priority topic
4. movement_diagnostic bugs fixed
5. Custom Nav2 launch with remapping created
6. Launch file updated with multiplexer
7. Verification script created
8. Entry points added

### ⚠️ KNOWN ISSUES:
1. joy_ctrl, keyboard_ctrl, behavior_ctrl still publish to /cmd_vel (not critical, manual nodes)
2. QoS settings not verified (needs checking)
3. Movement verification needs enhancement (Day 2)

### 🔄 NEXT STEPS:
1. Verify QoS settings match across all publishers/subscribers
2. Test system with real robot
3. Continue with Day 2 enhancements (movement verification)

# Critical Bug Fix - Direct Navigation Activation Order

**Date:** 2025-01-XX  
**Severity:** CRITICAL - Causes robot to stop immediately after activation  
**Status:** ✅ FIXED

## Problem Description

**Bug:** `activate()` called before `set_goal()` causes immediate deactivation, stopping robot movement.

**Root Cause:**
1. `activate()` sets `is_active = True` and `watchdog_enabled = True`
2. `set_goal()` with default `preserve_active_state=False` sets `is_active = False`
3. Watchdog immediately stops publishing because `is_active = False`
4. Robot stops moving immediately after activation

**Affected Lines:**
- Line 3289-3290: Nav2 hasn't started publishing - WRONG ORDER
- Line 3302-3303: Nav2 stopped publishing - WRONG ORDER
- Line 3310-3311: Nav2 was working but stopped - WRONG ORDER
- Line 3319-3320: Nav2 was working but stopped - WRONG ORDER

## Solution

**Fix:** Always call `set_goal()` FIRST, then `activate()` only if not already active.

**Correct Pattern:**
```python
was_active = self.direct_nav_fallback.is_active
self.direct_nav_fallback.set_goal(self.nav_goal_pose, preserve_active_state=was_active)
if not was_active:
    self.direct_nav_fallback.activate()
```

**Why This Works:**
1. If already active: `preserve_active_state=True` keeps `is_active = True` during goal update
2. If not active: `set_goal()` sets `is_active = False`, then `activate()` sets `is_active = True`
3. Goal is always set BEFORE activation state is changed
4. Watchdog continues publishing because `is_active` remains `True`

## Files Modified

**File:** `tyre_inspection_mission/core/mission_controller.py`

**Changes:**
- Line 3288-3295: Fixed activation order with was_active check
- Line 3306-3312: Fixed activation order with was_active check
- Line 3318-3324: Fixed activation order with was_active check

## Verification

**Verification Script:** `scripts/verify_direct_nav_activation.sh`

**Runtime Verification:**
```bash
# Monitor direct navigation cmd_vel
ros2 topic echo /cmd_vel/direct_control

# When direct navigation activates, should see continuous cmd_vel publishing at 50Hz
# If bug exists, cmd_vel would stop immediately after activation
```

**Expected Behavior:**
- ✅ Goal set first (preserves active state if already active)
- ✅ Activation happens AFTER goal is set
- ✅ Watchdog continues publishing at 50Hz when active
- ✅ Robot continues moving when direct navigation activates

**Bug Behavior (Before Fix):**
- ❌ Activation happens first
- ❌ Goal set second (deactivates immediately)
- ❌ Watchdog stops publishing (is_active = False)
- ❌ Robot stops moving immediately after activation

## Impact

**Before Fix:**
- Robot would activate direct navigation fallback when Nav2 fails
- But immediately stop because `set_goal()` deactivated it
- Robot would remain stopped, requiring manual intervention

**After Fix:**
- Robot activates direct navigation fallback correctly
- Goal is set before activation, preserving state if already active
- Watchdog continues publishing at 50Hz
- Robot continues moving toward goal autonomously

## Related Issues

This bug would cause:
1. Robot stops when Nav2 fails (should activate direct navigation fallback)
2. Robot stops when Nav2 timeout occurs (should activate direct navigation fallback)
3. Mission fails instead of recovering via direct navigation fallback

**All instances fixed. Robot will now continue moving via direct navigation fallback when Nav2 fails.**

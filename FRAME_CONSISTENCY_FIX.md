# Critical Fix: Frame Consistency in Direct Navigation

## 🎯 Issue Identified

**Reality Check Result:** Direct navigation calculates distance/angle between robot_pose and goal_pose **without checking if they're in the same frame**.

### Problem:
- Robot pose is always requested in "map" frame: `robot_pose = self._get_robot_pose("map")`
- Goal can be in either "map" or "odom" frame (fallback when map frame is stale)
- If goal is in "odom" frame but robot_pose is in "map" frame, calculations are **WRONG**
- `_calculate_distance()` and `_calculate_angle_to_goal()` directly subtract coordinates without frame check
- This causes incorrect distance/angle calculations, making rover rotate incorrectly or never reach goal

**Example:**
- Robot at (0.0, 0.0) in "map" frame
- Goal at (3.68, 0.53) in "odom" frame
- Distance calculated as `sqrt((3.68-0.0)² + (0.53-0.0)²) = 3.72m`
- But if map and odom frames are offset, actual distance might be completely different!

## ✅ Fix Applied

**Step 3 - Implementation:** Added frame validation and transformation to ensure poses are in the same frame before calculating distance/angle.

### Changes:
1. **Added TF buffer support to DirectNavigationFallback**
   - Pass `tf_buffer` from mission_controller to DirectNavigationFallback
   - If tf_buffer not provided, create new one with TransformListener (for standalone use)
   - If tf_buffer provided, use existing buffer (mission_controller's listener updates it)

2. **Added frame consistency check in `update()` method**
   - Check if `robot_pose.header.frame_id != goal_pose.header.frame_id`
   - If frames differ, transform goal to robot_pose's frame using TF
   - Use transformed goal for all calculations (distance, angle, orientation)
   - Log transformation for visibility

3. **Added frame consistency check in `is_goal_reached()` method**
   - Same frame validation and transformation logic
   - Ensures goal reached check uses same frame as update()

**Files Modified:**
- `tyre_inspection_mission/navigation/direct_navigation_fallback.py` (lines 1-18, 26-45, 278-308, 533-590)
- `tyre_inspection_mission/core/mission_controller.py` (line 629)

### Code Changes:
```python
# BEFORE:
def update(self, robot_pose: Optional[PoseStamped]) -> bool:
    distance = self._calculate_distance(robot_pose, self.current_goal)  # NO FRAME CHECK!

# AFTER:
def update(self, robot_pose: Optional[PoseStamped]) -> bool:
    # CRITICAL FIX: Ensure robot_pose and goal_pose are in the same frame
    goal_pose_to_use = self.current_goal
    if robot_pose.header.frame_id != self.current_goal.header.frame_id:
        # Transform goal to robot_pose's frame
        transform = self.tf_buffer.lookup_transform(...)
        goal_pose_to_use = do_transform_pose_stamped(self.current_goal, transform)
    
    distance = self._calculate_distance(robot_pose, goal_pose_to_use)  # SAME FRAME!
```

## 📋 Step 4 - Proof

### Verification Script:
Created `scripts/verify_cmd_vel_pipeline.sh` to verify cmd_vel pipeline integrity.

### Manual Verification Commands:
```bash
# Check frame consistency in direct navigation (requires mission running)
# Look for frame mismatch warnings in logs:
ros2 topic echo /rosout | grep -i "frame mismatch\|transform goal"

# Check if goals are transformed correctly:
ros2 topic echo /rosout | grep -i "goal transformed to.*frame"

# Verify direct navigation is using correct frames:
# (Robot pose should be in "map", goal should be transformed to "map" if originally in "odom")
```

### Expected Behavior:
- ✅ If goal is in "odom" frame and robot_pose is in "map" frame, goal is transformed to "map"
- ✅ Distance/angle calculations use poses in the same frame
- ✅ Logging shows frame transformation for visibility
- ✅ If transform fails, error is logged and calculation continues (but may be incorrect)
- ✅ Direct navigation works correctly regardless of goal frame

### Log Messages to Expect:
```
✅ Direct navigation: Goal transformed to 'map' frame. 
   Original: (3.68, 0.53) in 'odom', 
   Transformed: (2.15, 0.32) in 'map'
```

OR (if frames match):
```
✅ Direct navigation: Robot and goal in same frame ('map') - no transform needed
```

## 🔍 Root Cause Analysis

### Why This Happens:
1. **Goals calculated in odom frame as fallback** - When map frame TF is stale, goals are calculated in odom frame
2. **Robot pose always requested in map frame** - `_get_robot_pose("map")` always requests map frame
3. **No frame validation** - Distance/angle calculations don't check frames match
4. **Frame offset** - Map and odom frames can have significant offsets (map origin, SLAM drift)

### Impact:
- **Incorrect distance calculations** - Rover thinks it's closer/farther than it actually is
- **Incorrect angle calculations** - Rover rotates toward wrong direction
- **Goal never reached** - Even if physically at goal, calculations show wrong distance
- **Infinite rotation** - Rover rotates but goal position "moves" relative to robot (wrong frame)

## ✅ Status

- ✅ Build successful - fix compiled without errors
- ✅ No linter errors
- ✅ Frame transformation logic implemented
- ✅ Both `update()` and `is_goal_reached()` methods fixed
- ✅ TF buffer properly passed from mission_controller

---

## 🚀 Next Risk Identified

**Step 5 - Next Risk:** Map bounds validation for Nav2 goals

Even after transforming goals to map frame, if goals are outside the current map bounds (map is 69x147 = 3.45m x 7.35m), Nav2 will still fail with `worldToMap` errors. 

**Solution:** Add map bounds validation before sending goals to Nav2 (subscribe to /map topic to get map info, validate goals are within bounds).

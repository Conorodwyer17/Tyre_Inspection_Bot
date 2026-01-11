# Critical Fix: Map Bounds Validation for Nav2 Goals

## 🎯 Issue Identified

**Reality Check Result:** Goals transformed to map frame can still be **outside current map bounds**, causing Nav2 `worldToMap` errors.

### Problem:
- Map is dynamic (SLAM building it) - currently small: 69x147 cells = 3.45m x 7.35m
- Map origin: (-1.328, -6.807)
- Goal at (3.68, 0.53) converts to map coords: (100, 146)
- Map size: (69, 147) - x=100 >= 69 (OUT OF BOUNDS!)
- Nav2 fails with: `worldToMap failed: mx,my: 170,4, size_x,size_y: 162,224`
- Even after transforming to map frame, goals outside bounds cause Nav2 to fail

**Example:**
- Goal calculated at (3.68, 0.53) in odom frame
- Transformed to map frame: (3.68, 0.53) in map frame
- Map coords: x = int((3.68 - (-1.328)) / 0.05) = 100
- Map size: width=69 - x=100 >= 69 → OUT OF BOUNDS!
- Nav2 `worldToMap` fails → Navigation fails

## ✅ Fix Applied

**Step 3 - Implementation:** Added map bounds validation before sending goals to Nav2.

### Changes:
1. **Added map subscriber** - Subscribe to `/map` topic to get map metadata
   - Stores: `width`, `height`, `resolution`, `origin_x`, `origin_y`
   - Updates automatically as SLAM builds the map
   - Uses TRANSIENT_LOCAL durability to get latest map even if subscribed late

2. **Added `map_callback()` method** - Stores latest map metadata
   - Updates `self.map_info` with map dimensions and origin
   - Logs map metadata for visibility

3. **Added `_validate_goal_in_map_bounds()` method** - Validates goal is within map bounds
   - Converts world coordinates to map coordinates
   - Checks if map coords are within bounds: `0 <= map_x < width` and `0 <= map_y < height`
   - Returns `True` if valid, `False` if outside bounds

4. **Integrated validation in `navigate_to_pose()`** - Validates goals before sending to Nav2
   - After transforming goal to map frame
   - Before creating Nav2 goal message
   - If goal is outside bounds, skip Nav2 goal (direct control can handle it with odom frame)

**Files Modified:**
- `tyre_inspection_mission/core/mission_controller.py` (lines 33, 383-390, 1006-1050, 6511-6537)

### Code Changes:
```python
# BEFORE:
# No map bounds validation - Nav2 fails with worldToMap errors
goal_msg = NavigateToPose.Goal()
goal_msg.pose = goal_pose_to_send
# Send to Nav2...

# AFTER:
# CRITICAL: Validate goal is within map bounds (prevents Nav2 worldToMap errors)
if goal_pose_to_send.header.frame_id == "map" and self.map_info:
    goal_in_bounds = self._validate_goal_in_map_bounds(goal_pose_to_send)
    if not goal_in_bounds:
        self.get_logger().warn(
            f"⚠️ navigate_to_pose(): Goal is outside current map bounds. "
            f"Skipping Nav2 goal (will use direct control only with odom frame)."
        )
        return False  # Skip Nav2 goal - direct control can handle it
```

## 📋 Step 4 - Proof

### Verification Script:
Run verification commands to check map bounds validation:

```bash
# Check if map metadata is stored
ros2 topic echo /map --once | grep -E "width|height|resolution|origin"

# Check if goal validation works (requires mission running)
# Goals outside map bounds should be skipped for Nav2
ros2 topic echo /rosout | grep -i "goal.*outside.*map bounds"

# Verify map subscription is active
ros2 topic info /map | grep "Subscription count"  # Should be >= 1 (mission_controller subscribed)
```

### Manual Verification Commands:
```bash
# Test map bounds calculation
python3 -c "
width=69; height=147; res=0.05; origin_x=-1.328; origin_y=-6.807
goal_x=3.68; goal_y=0.53
mx = int((goal_x - origin_x) / res)
my = int((goal_y - origin_y) / res)
print(f'Goal ({goal_x}, {goal_y}) -> map coords: ({mx}, {my}), map size: ({width}, {height})')
print(f'Valid: {0 <= mx < width and 0 <= my < height}')
"

# Expected output: Valid: False (goal is outside map bounds)
```

### Expected Behavior:
- ✅ Map metadata is stored when `/map` topic publishes
- ✅ Goals in map frame are validated against map bounds before sending to Nav2
- ✅ Goals outside bounds are skipped for Nav2 (direct control handles them with odom frame)
- ✅ If map info not available yet (SLAM just starting), validation is skipped (fail open)
- ✅ Logging shows when goals are outside bounds

### Log Messages to Expect:
```
⚠️ navigate_to_pose(): Goal is outside current map bounds. 
   Goal: (3.68, 0.53). 
   Map size: 69x147 cells (3.45m x 7.35m). 
   Map origin: (-1.33, -6.81). 
   Skipping Nav2 goal (will use direct control only with odom frame).
```

OR (if map info not available yet):
```
⚠️ navigate_to_pose(): Map info not available yet (SLAM building). 
   Skipping map bounds validation - goal will be validated by Nav2.
```

## 🔍 Root Cause Analysis

### Why This Happens:
1. **Map is dynamic** - SLAM builds the map over time, starting small and growing
2. **Goals calculated before map is built** - Goals can be calculated far from starting position
3. **No bounds validation** - Goals sent to Nav2 without checking if they're within map bounds
4. **Nav2 requires map coords** - Nav2 converts world coords to map coords, fails if outside bounds

### Impact:
- **Nav2 worldToMap errors** - Planner fails to convert goal coordinates to map coordinates
- **Navigation fails** - Nav2 cannot plan path to goal outside map bounds
- **No fallback** - Without bounds validation, Nav2 keeps trying to use invalid goal
- **Direct control works** - Direct control can use odom frame (always available) even if map is small

## ✅ Status

- ✅ Build successful - fix compiled without errors
- ✅ No linter errors
- ✅ Map subscriber added
- ✅ Map bounds validation implemented
- ✅ Validation integrated into navigate_to_pose()

## 🚀 Next Steps

1. **Restart launch file** to activate map bounds validation
2. **Monitor map growth** - As SLAM builds map, more goals will be within bounds
3. **Verify direct control fallback** - Goals outside map bounds should use direct control (odom frame)
4. **Check Nav2 worldToMap errors** - Should decrease as validation prevents invalid goals

---

## 📊 Status Summary

| Component | Status | Notes |
|-----------|--------|-------|
| Parameter Type Fix | ✅ Fixed | `use_sim_time` is now boolean |
| Direct Navigation Fix | ✅ Fixed | Simultaneous rotation + translation |
| Nav2 Goal Transform | ✅ Fixed | Goals transformed to map frame |
| Auto-Restart | ✅ Fixed | `respawn=True` added to critical nodes |
| Frame Consistency | ✅ Fixed | Goals transformed to robot_pose frame in direct nav |
| Map Bounds Validation | ✅ Fixed | Goals validated against map bounds before Nav2 |

# Error Recovery Strategy

## Overview

The tyre inspection mission implements a comprehensive error recovery strategy to ensure the mission can adapt to issues during testing and complete its goals. This document outlines the recovery mechanisms and how to use them.

## Error Recovery Architecture

### Error Categories

Errors are categorized by type and assigned error codes:

1. **Navigation Errors (NAV_xxx)**
   - Goal too close
   - Nav2 unavailable
   - Goal rejected
   - Navigation timeout
   - TF transform failures

2. **Detection Errors (DET_xxx)**
   - No vehicles detected
   - LiDAR processing failed
   - Tyre detection failed
   - Point cloud unavailable

3. **Planning Errors (PLAN_xxx)**
   - Waypoint calculation failed
   - Invalid pose values
   - Fallback generation failed

4. **Capture Errors (CAP_xxx)**
   - Service unavailable
   - File write failed
   - Camera not initialized

5. **Transform Errors (TF_xxx)**
   - Frame not found
   - Frame tree broken
   - Transform too old

### Recovery Mechanisms

#### 1. Automatic Retry
Most operations automatically retry up to a maximum number of attempts:
- Navigation goals: 3 retries
- Photo capture: 3 attempts per object
- Goal recalculation: Configurable max attempts

#### 2. Fallback Strategies
When primary methods fail, fallback strategies are used:

**Detection Fallbacks:**
- If no tyres detected → Generate fallback waypoints (4 standard positions)
- If vehicle lost → Use LiDAR position
- If camera lost → Use last known position

**Navigation Fallbacks:**
- If goal rejected → Recalculate with increased distance
- If Nav2 unavailable → Wait and retry
- If too close → Back up and recalculate
- If map frame unavailable → Use odom frame

**Planning Fallbacks:**
- If waypoint calculation fails → Use detection pose
- If pose invalid → Use vehicle position
- If no tyres → Generate standard 4-tyre layout

#### 3. Error Recovery State
The `ERROR_RECOVERY` state provides comprehensive recovery:

**Recovery Process:**
1. Cancel active operations (navigation, etc.)
2. Check system health
3. Determine recovery strategy based on error type
4. Attempt recovery
5. Resume from appropriate state

**Recovery Strategies:**
- **Navigation errors**: Retry with adjusted parameters
- **Detection errors**: Continue with fallback waypoints
- **Planning errors**: Use stored poses or fallback generation
- **Capture errors**: Retry or skip if max attempts reached

**Recovery Limits:**
- Maximum recovery attempts: 5
- Recovery timeout: 30 seconds
- After timeout/limit: Reset to IDLE state

#### 4. State-Specific Recovery

**SEARCHING_VEHICLES:**
- If timeout → Complete mission
- If no detections → Continue searching (up to timeout)

**PLANNING:**
- If timeout → Proceed with available waypoints
- If no waypoints → Use detection poses

**NAVIGATING_TO_LICENSE_PLATE:**
- If goal rejected → Use pre-planned waypoint
- If too close → Proceed to capture
- If timeout → Proceed if close enough

**DETECTING_TYRES:**
- If timeout → Generate fallback waypoints
- If no tyres → Use fallback waypoints

**NAVIGATING_TO_TYRE:**
- If goal rejected → Use pre-planned waypoint
- If too close → Proceed to capture
- If timeout → Proceed if close enough

**CAPTURING_TYRE:**
- If capture fails → Retry up to max attempts
- If max attempts → Continue to next tyre

**CHECKING_COMPLETION:**
- If tyres missing → Retry navigation
- If max retries → Continue with photographed tyres

## Error Code Reference

### Navigation Errors

| Code | Description | Recovery Strategy |
|------|-------------|-------------------|
| NAV_001 | Goal too close to robot | Recalculate with increased distance |
| NAV_002 | Nav2 action server not available | Wait and retry, check Nav2 status |
| NAV_003 | Goal rejected by Nav2 | Use pre-planned waypoint or recalculate |
| NAV_004 | Navigation timeout | Proceed if close enough, otherwise cancel |
| NAV_005 | TF transform failed | Fallback to alternative frame |

### Detection Errors

| Code | Description | Recovery Strategy |
|------|-------------|-------------------|
| DET_001 | No vehicles detected | Continue searching, check YOLO |
| DET_002 | LiDAR processing failed | Retry, check scan topic |
| DET_003 | Tyre detection failed | Use fallback waypoint generation |
| DET_004 | Point cloud unavailable | Wait for point cloud, check camera |

### Planning Errors

| Code | Description | Recovery Strategy |
|------|-------------|-------------------|
| PLAN_001 | License plate waypoint failed | Use vehicle detection pose |
| PLAN_002 | Tyre waypoint failed | Use fallback waypoint generation |
| PLAN_003 | Fallback generation failed | Use standard 4-tyre layout |
| PLAN_004 | Invalid pose values | Validate and sanitize |
| PLAN_005 | Invalid depth value | Check depth sensor calibration |

### Capture Errors

| Code | Description | Recovery Strategy |
|------|-------------|-------------------|
| CAP_001 | Service not available | Wait for service, check node |
| CAP_002 | File write failed | Check permissions and disk space |
| CAP_003 | Camera not initialized | Initialize camera, check node |

### Transform Errors

| Code | Description | Recovery Strategy |
|------|-------------|-------------------|
| TF_001 | Frame not found | Wait for frame to become available |
| TF_002 | Frame tree broken | Check TF tree, restart broadcaster |
| TF_003 | Transform too old | Use more recent timestamp |

## Recovery Decision Tree

```
Error Occurs
    ↓
Is it retryable?
    ├─ Yes → Retry (up to max attempts)
    │   ├─ Success → Continue
    │   └─ Failure → Try fallback
    │
    └─ No → Try fallback immediately
        ├─ Fallback available → Use fallback
        │   ├─ Success → Continue
        │   └─ Failure → Enter ERROR_RECOVERY
        │
        └─ No fallback → Enter ERROR_RECOVERY
            ├─ System health OK → Attempt recovery
            │   ├─ Success → Resume from appropriate state
            │   └─ Failure → Increment recovery attempts
            │       ├─ Under limit → Retry recovery
            │       └─ Over limit → Reset to IDLE
            │
            └─ System health critical → Reset to IDLE
```

## Implementation Details

### Error Recovery State Handler

Located in: `mission_controller.py` → `state_machine_step()` → `ERROR_RECOVERY`

**Process:**
1. Track recovery attempts and timeout
2. Cancel active operations
3. Check system health
4. Determine recovery strategy
5. Resume from appropriate state

**Recovery Strategies:**
- Navigation errors → Retry navigation
- Detection errors → Use fallback waypoints
- Planning errors → Use stored poses
- Capture errors → Retry or skip

### Diagnostic Tools

Located in: `diagnostics.py`

**SystemDiagnostics:**
- Check TF frame availability
- Check topic publishing
- Check service availability
- Check Nav2 health
- Generate health reports

**ErrorRecoveryHelper:**
- Get recovery strategy for error code
- Determine if operation should retry
- Provide recovery recommendations

## Usage Examples

### Handling Navigation Errors

```python
try:
    self.navigate_to_pose(goal_pose)
except NavigationError as e:
    if e.error_code == "NAV_001":  # Goal too close
        # Recalculate with increased distance
        new_goal = self.recalculate_goal_safe_distance(
            target_pose, 
            min_distance=0.9
        )
        if new_goal:
            self.navigate_to_pose(new_goal)
    elif e.error_code == "NAV_002":  # Nav2 unavailable
        # Wait and retry
        time.sleep(2.0)
        self.navigate_to_pose(goal_pose)
```

### Handling Detection Errors

```python
try:
    tyres = self.process_tyre_detections(msg)
except DetectionError as e:
    if e.error_code == "DET_003":  # Tyre detection failed
        # Use fallback waypoints
        self._generate_fallback_tyre_waypoints(self.current_vehicle)
        if self.current_vehicle.tyre_waypoints:
            self.state = MissionState.NAVIGATING_TO_TYRE
```

### Using Diagnostics

```python
from tyre_inspection_mission.diagnostics import SystemDiagnostics

diagnostics = SystemDiagnostics(self, self.slogger)
health_report = diagnostics.get_system_health_report()

if health_report['overall'] == HealthStatus.CRITICAL:
    # Take appropriate action
    self.state = MissionState.ERROR_RECOVERY
```

## Best Practices

1. **Always use specific exceptions**: Catch specific exception types, not generic `Exception`
2. **Provide error context**: Include error codes and context in exceptions
3. **Log recovery attempts**: Use structured logger with appropriate categories
4. **Check system health**: Before attempting recovery, check if system is healthy
5. **Limit recovery attempts**: Prevent infinite recovery loops
6. **Use fallbacks**: Always have fallback strategies for critical operations
7. **Monitor recovery**: Track recovery attempts and success rates

## Testing Error Recovery

### Simulating Errors

1. **Navigation errors**: Place goal too close to robot
2. **Detection errors**: Cover camera or disable YOLO
3. **Planning errors**: Provide invalid pose data
4. **Capture errors**: Disable photo capture service
5. **Transform errors**: Disable TF broadcaster

### Verifying Recovery

1. Check logs for recovery attempts
2. Verify state transitions
3. Confirm mission continues or resets appropriately
4. Check system health reports

---

**Last Updated:** Current Session  
**Maintained By:** Development Team

# Error Context Reference

## Overview

Complete reference for all error codes with recovery actions, severity levels, and troubleshooting steps.

---

## Navigation Errors

### NAV_001 - Goal Too Close to Robot
**Severity:** MEDIUM  
**Description:** Navigation goal is too close to robot (< minimum safe distance)

**Recovery Steps:**
1. Check current robot position
2. Recalculate goal with increased distance
3. Use validate_goal_distance() to verify
4. If still too close, use fallback waypoint or skip

**Expected Outcome:** Goal at safe distance (>0.6m)  
**Timeout:** 5s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures)

---

### NAV_002 - Nav2 Action Server Not Available
**Severity:** CRITICAL  
**Description:** Nav2 action server is not available or not ready

**Recovery Steps:**
1. Check if Nav2 is launched
2. Wait for action server to become ready
3. Check Nav2 logs for errors
4. Restart Nav2 if needed

**Expected Outcome:** Nav2 action server ready  
**Timeout:** 30s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures)

---

### NAV_003 - Navigation Goal Rejected
**Severity:** MEDIUM  
**Description:** Nav2 rejected the navigation goal

**Recovery Steps:**
1. Check goal validity and distance
2. Verify goal is not in obstacle
3. Check Nav2 costmap
4. Use fallback waypoint if available

**Expected Outcome:** Goal accepted or fallback used  
**Timeout:** 10s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures)

---

### NAV_004 - Navigation Timeout
**Severity:** HIGH  
**Description:** Navigation did not complete within timeout period

**Recovery Steps:**
1. Check if robot is stuck
2. Verify goal is reachable
3. Check Nav2 planner status
4. Cancel and retry with different goal

**Expected Outcome:** Navigation completes or retries  
**Timeout:** 60s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures)

---

## Detection Errors

### DET_001 - No Vehicles Detected
**Severity:** HIGH  
**Description:** No vehicles detected during search phase

**Recovery Steps:**
1. Check YOLO node is running
2. Verify camera is publishing images
3. Check vehicle distance (should be 0.5m - 10m)
4. Verify camera lights are on
5. Check YOLO confidence threshold
6. Review detection statistics

**Expected Outcome:** Vehicles detected  
**Timeout:** 120s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-no-vehicle-detection)

---

### DET_002 - No Tyres Detected
**Severity:** MEDIUM  
**Description:** No tyres detected on vehicle

**Recovery Steps:**
1. Check inspection mode is active
2. Verify tyre detection model is loaded
3. Check vehicle is in view
4. Use fallback waypoint generation
5. Continue with standard 4-tyre layout

**Expected Outcome:** Tyres detected or fallback waypoints generated  
**Timeout:** 60s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-tyre-detection-problems)

---

### DET_003 - Point Cloud Not Available
**Severity:** HIGH  
**Description:** Point cloud data is not available

**Recovery Steps:**
1. Check point cloud topic is publishing
2. Verify camera depth stream
3. Check QoS settings match
4. Restart point cloud node if needed

**Expected Outcome:** Point cloud available  
**Timeout:** 10s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-point-cloud-not-available)

---

## Capture Errors

### CAP_001 - Photo Capture Service Unavailable
**Severity:** HIGH  
**Description:** Photo capture service is not available

**Recovery Steps:**
1. Check photo_capture node is running
2. Verify service is advertised
3. Wait for service to become available
4. Restart photo_capture node if needed

**Expected Outcome:** Photo capture service available  
**Timeout:** 10s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-photo-capture-failures)

---

### CAP_002 - Photo Capture Failed
**Severity:** MEDIUM  
**Description:** Photo capture attempt failed

**Recovery Steps:**
1. Check camera is initialized
2. Verify file permissions
3. Check disk space
4. Retry capture (max 3 attempts)

**Expected Outcome:** Photo captured successfully  
**Timeout:** 10s

**See Also:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-photo-capture-failures)

---

## Planning Errors

### PLAN_001 - Waypoint Calculation Failed
**Severity:** HIGH  
**Description:** Failed to calculate navigation waypoints

**Recovery Steps:**
1. Check vehicle detection pose is valid
2. Verify TF transforms are available
3. Check goal distance validation
4. Use fallback waypoint generation

**Expected Outcome:** Waypoints calculated or fallback used  
**Timeout:** 30s

**See Also:** [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md#planning-errors)

---

### PLAN_002 - Invalid Vehicle Pose
**Severity:** HIGH  
**Description:** Vehicle detection pose is invalid

**Recovery Steps:**
1. Check vehicle detection is recent
2. Verify TF transform succeeded
3. Use LiDAR position if available
4. Re-detect vehicle if needed

**Expected Outcome:** Valid vehicle pose  
**Timeout:** 10s

**See Also:** [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md#planning-errors)

---

### PLAN_003 - Fallback Waypoint Generation Failed
**Severity:** MEDIUM  
**Description:** Failed to generate fallback waypoints

**Recovery Steps:**
1. Check vehicle detection pose exists
2. Verify vehicle dimensions are reasonable
3. Use default 4-tyre layout
4. Skip to next vehicle if critical

**Expected Outcome:** Fallback waypoints generated  
**Timeout:** 10s

**See Also:** [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md#planning-errors)

---

## Transform Errors

### TF_001 - TF Frame Not Found
**Severity:** HIGH  
**Description:** Required TF frame is not available

**Recovery Steps:**
1. Wait for frame to become available
2. Check TF broadcaster is running
3. Verify frame names in config
4. Check TF tree connectivity

**Expected Outcome:** TF frame available  
**Timeout:** 5s

**See Also:** [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md#transform-errors)

---

### TF_002 - TF Transform Failed
**Severity:** HIGH  
**Description:** TF transform operation failed

**Recovery Steps:**
1. Check TF tree is connected
2. Verify frame timestamps
3. Wait and retry transform
4. Use alternative frame if available

**Expected Outcome:** TF transform succeeds  
**Timeout:** 5s

**See Also:** [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md#transform-errors)

---

## Validation Errors

### VAL_001 - Invalid Pose
**Severity:** MEDIUM  
**Description:** Pose validation failed

**Recovery Steps:**
1. Check pose values are finite
2. Verify frame_id is set
3. Check for NaN/Inf values
4. Recalculate pose

**Expected Outcome:** Valid pose  
**Timeout:** 5s

---

### VAL_002 - Invalid Parameter
**Severity:** LOW  
**Description:** Parameter validation failed

**Recovery Steps:**
1. Check parameter value is in valid range
2. Verify parameter type
3. Use default value if available
4. Check configuration documentation

**Expected Outcome:** Valid parameter  
**Timeout:** 1s

---

## State Machine Errors

### STM_001 - Invalid State Transition
**Severity:** CRITICAL  
**Description:** Attempted invalid state transition

**Recovery Steps:**
1. Check state transition logic
2. Verify state requirements are met
3. Review state machine flow
4. Transition to ERROR_RECOVERY

**Expected Outcome:** Valid state transition  
**Timeout:** 1s

**See Also:** [STATE_VALIDATION.md](../../architecture/STATE_VALIDATION.md)

---

### STM_002 - State Timeout
**Severity:** HIGH  
**Description:** State exceeded maximum duration

**Recovery Steps:**
1. Check state execution logic
2. Verify no blocking operations
3. Review timeout value
4. Transition to ERROR_RECOVERY if critical

**Expected Outcome:** State completes or recovers  
**Timeout:** 30s

**See Also:** [STATE_VALIDATION.md](../../architecture/STATE_VALIDATION.md)

---

## Quick Lookup

### By Severity

**CRITICAL:**
- NAV_002 - Nav2 unavailable
- STM_001 - Invalid state transition

**HIGH:**
- NAV_004 - Navigation timeout
- DET_001 - No vehicles
- DET_003 - Point cloud unavailable
- CAP_001 - Service unavailable
- PLAN_001 - Waypoint calculation failed
- PLAN_002 - Invalid vehicle pose
- TF_001 - Frame not found
- TF_002 - Transform failed
- STM_002 - State timeout

**MEDIUM:**
- NAV_001 - Goal too close
- NAV_003 - Goal rejected
- DET_002 - No tyres
- CAP_002 - Capture failed
- PLAN_003 - Fallback failed
- VAL_001 - Invalid pose

**LOW:**
- VAL_002 - Invalid parameter

---

## Error Code Format

All error codes follow the pattern: `{CATEGORY}_{NUMBER}`

- **NAV_xxx** - Navigation errors
- **DET_xxx** - Detection errors
- **CAP_xxx** - Capture errors
- **PLAN_xxx** - Planning errors
- **TF_xxx** - Transform errors
- **VAL_xxx** - Validation errors
- **STM_xxx** - State machine errors

---

## Using Error Context

### In Code

```python
from tyre_inspection_mission.error_context import ErrorContext

try:
    # ... operation that may fail ...
except NavigationError as e:
    recovery = ErrorContext.get_recovery_action(e.error_code)
    if recovery:
        logger.info(f"Recovery: {recovery.description}")
        # Execute recovery steps
```

### In Logs

Error messages automatically include:
- Error code
- Severity level
- Recovery action description
- Step-by-step recovery steps
- Expected outcome
- Timeout information

---

**Last Updated:** Current Session  
**Version:** 1.0

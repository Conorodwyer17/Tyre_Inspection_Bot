# Issue Location Map

## 🗺️ Where to Look for Specific Issues

This map helps you quickly find the relevant code and documentation for any issue.

---

## Detection Issues

### No Vehicle Detection
**Code Location:**
- `mission_controller.py` → `bbox_callback()` (line ~1090)
- `mission_controller.py` → `process_vehicle_detections()` (line ~1200)
- `vehicle_tracker.py` → `process_vehicle_detections()`

**Logs:**
- `[DET]` category logs
- Look for "Detected:" messages
- Check detection statistics logs

**Documentation:**
- [Troubleshooting Guide - No Vehicle Detection](TROUBLESHOOTING_GUIDE.md#-issue-no-vehicle-detection)
- [Error Recovery - Detection Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#detection-errors)

**Config:**
- `config.py` → `DetectionConfig`
- YOLO parameters in launch file

---

### No Tyre Detection
**Code Location:**
- `mission_controller.py` → `process_tyre_detections()` (line ~1279)
- `vehicle_tracker.py` → Tyre tracking logic
- `mission_controller.py` → `_generate_fallback_tyre_waypoints()` (line ~3120)

**Logs:**
- `[DET]` category logs for tyres
- Look for "Tyre detected" or "Fallback waypoints generated"

**Documentation:**
- [Troubleshooting Guide - Tyre Detection Problems](TROUBLESHOOTING_GUIDE.md#-issue-tyre-detection-problems)
- [Error Recovery - Detection Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#detection-errors)

**Config:**
- `config.py` → `DetectionConfig.TYRE_CLASS_NAME`
- `config.py` → `PlanningConfig.TYRE_POSITIONS_RELATIVE`

---

### LiDAR Fusion Not Working
**Code Location:**
- `mission_controller.py` → `process_lidar_vehicle_tracking()` (line ~920)
- `mission_controller.py` → `lidar_callback()` (line ~850)

**Logs:**
- `[DET]` category logs for LiDAR
- Look for "LiDAR tracking" messages

**Documentation:**
- [Troubleshooting Guide - LiDAR Fusion Not Working](TROUBLESHOOTING_GUIDE.md#-issue-lidar-fusion-not-working)
- [Error Recovery - Detection Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#detection-errors)

**Config:**
- `config.py` → `DetectionConfig.LIDAR_*` parameters

---

## Navigation Issues

### Goal Rejected
**Code Location:**
- `mission_controller.py` → `navigate_to_pose()` (line ~3365)
- `mission_controller.py` → `nav_goal_response_callback()` (line ~3500)
- `goal_planner.py` → `validate_goal_distance()`

**Logs:**
- `[NAV]` category logs
- Look for "Goal rejected" messages
- Check error code (e.g., `NAV_001`)

**Documentation:**
- [Troubleshooting Guide - Navigation Failures](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures)
- [Error Recovery - Navigation Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#navigation-errors)

**Config:**
- `config.py` → `NavigationConfig.MIN_SAFE_DISTANCE`
- `config.py` → `NavigationConfig.DEFAULT_APPROACH_DISTANCE`

---

### Navigation Timeout
**Code Location:**
- `mission_controller.py` → `check_navigation_complete()` (line ~3600)
- `mission_controller.py` → `nav_result_callback()` (line ~3550)

**Logs:**
- `[NAV]` category logs
- Look for "Navigation timeout" messages

**Documentation:**
- [Troubleshooting Guide - Navigation Failures](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures)
- [Error Recovery - Navigation Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#navigation-errors)

**Config:**
- `config.py` → `NavigationConfig.NAVIGATION_TIMEOUT`

---

### Goal Too Close
**Code Location:**
- `goal_planner.py` → `validate_goal_distance()`
- `mission_controller.py` → `recalculate_goal_safe_distance()` (line ~3700)

**Logs:**
- `[NAV]` category logs
- Look for "Too close" messages
- Error code: `NAV_001`

**Documentation:**
- [Troubleshooting Guide - Navigation Failures](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures)
- [Error Recovery - Navigation Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#navigation-errors)

**Config:**
- `config.py` → `NavigationConfig.MIN_SAFE_DISTANCE`

---

## Planning Issues

### Waypoint Calculation Failed
**Code Location:**
- `goal_planner.py` → `calculate_license_plate_waypoint()`
- `goal_planner.py` → `calculate_tyre_waypoint()`
- `mission_controller.py` → `_plan_waypoints_for_vehicle()` (line ~3250)

**Logs:**
- `[PLAN]` category logs
- Look for "Failed to plan" messages
- Error codes: `PLAN_001`, `PLAN_002`

**Documentation:**
- [Error Recovery - Planning Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#planning-errors)

**Config:**
- `config.py` → `PlanningConfig.*`

---

### Fallback Waypoint Generation Failed
**Code Location:**
- `mission_controller.py` → `_generate_fallback_tyre_waypoints()` (line ~3120)

**Logs:**
- `[PLAN]` category logs
- Look for "Failed to generate fallback" messages
- Error code: `PLAN_003`

**Documentation:**
- [Error Recovery - Planning Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#planning-errors)

**Config:**
- `config.py` → `PlanningConfig.TYRE_POSITIONS_RELATIVE`

---

## Capture Issues

### Photo Capture Service Unavailable
**Code Location:**
- `mission_controller.py` → `capture_photo()` (line ~3833)
- `photo_capture.py` → Service implementation

**Logs:**
- `[CAP]` category logs
- Look for "Service not available" messages
- Error code: `CAP_001`

**Documentation:**
- [Troubleshooting Guide - Photo Capture Failures](TROUBLESHOOTING_GUIDE.md#-issue-photo-capture-failures)
- [Error Recovery - Capture Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#capture-errors)

**Fix:**
- Check if `photo_capture_service` node is running
- Check service name: `/photo_capture/capture`

---

### File Write Failed
**Code Location:**
- `photo_capture.py` → File writing logic

**Logs:**
- `[CAP]` category logs
- Look for "File write failed" messages
- Error code: `CAP_002`

**Documentation:**
- [Troubleshooting Guide - Photo Capture Failures](TROUBLESHOOTING_GUIDE.md#-issue-photo-capture-failures)
- [Error Recovery - Capture Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#capture-errors)

**Fix:**
- Check file permissions
- Check disk space
- Check photo storage directory

---

## State Machine Issues

### Stuck in SEARCHING_VEHICLES
**Code Location:**
- `mission_controller.py` → `state_machine_step()` → `SEARCHING_VEHICLES` (line ~1650)

**Logs:**
- `[STATE]` category logs
- `[DET]` category logs
- Check detection timeout

**Documentation:**
- [Troubleshooting Guide - State Machine Stuck](TROUBLESHOOTING_GUIDE.md#-issue-state-machine-stuck)
- [Troubleshooting Guide - No Vehicle Detection](TROUBLESHOOTING_GUIDE.md#-issue-no-vehicle-detection)

**Config:**
- `config.py` → `DetectionConfig.DETECTION_TIMEOUT`

---

### Stuck in PLANNING
**Code Location:**
- `mission_controller.py` → `state_machine_step()` → `PLANNING` (line ~1700)

**Logs:**
- `[STATE]` category logs
- `[PLAN]` category logs
- Check planning timeout

**Documentation:**
- [Troubleshooting Guide - State Machine Stuck](TROUBLESHOOTING_GUIDE.md#-issue-state-machine-stuck)
- [Error Recovery - Planning Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#planning-errors)

**Config:**
- `config.py` → `PlanningConfig.PLANNING_TIMEOUT`

---

### Stuck in ERROR_RECOVERY
**Code Location:**
- `mission_controller.py` → `state_machine_step()` → `ERROR_RECOVERY` (line ~2784)

**Logs:**
- `[ERR]` category logs
- `[STATE]` category logs
- Check error recovery attempts

**Documentation:**
- [Troubleshooting Guide - State Machine Stuck](TROUBLESHOOTING_GUIDE.md#-issue-state-machine-stuck)
- [Error Recovery Strategy](../../architecture/ERROR_RECOVERY_STRATEGY.md)

**Fix:**
- Check error code in logs
- Check system health
- Check recovery timeout (30s)
- Check max recovery attempts (5)

---

## Transform Issues

### TF Frame Not Found
**Code Location:**
- `utils.py` → `transform_pose()`
- `mission_controller.py` → All TF transform calls

**Logs:**
- `[ERR]` category logs
- Look for "TF transform failed" messages
- Error code: `TF_001`

**Documentation:**
- [Error Recovery - Transform Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#transform-errors)

**Fix:**
- Check if TF broadcaster is running
- Check frame names in `config.py` → `FrameConfig`
- Wait for frames to become available

---

### TF Tree Broken
**Code Location:**
- `utils.py` → `transform_pose()`
- `mission_controller.py` → TF transform calls

**Logs:**
- `[ERR]` category logs
- Error code: `TF_002`

**Documentation:**
- [Error Recovery - Transform Errors](../../architecture/ERROR_RECOVERY_STRATEGY.md#transform-errors)

**Fix:**
- Check TF tree: `ros2 run tf2_tools view_frames`
- Restart TF broadcaster
- Check frame connections

---

## System Health Issues

### System Health Critical
**Code Location:**
- `mission_controller.py` → `check_system_health()` (line ~765)
- `diagnostics.py` → `SystemDiagnostics.get_system_health_report()`

**Logs:**
- `[SYS]` category logs
- Look for "System health is CRITICAL" messages

**Documentation:**
- [Troubleshooting Guide - System Health](TROUBLESHOOTING_GUIDE.md#system-health)
- Run: `ros2 run tyre_inspection_mission diagnostic_check`

**Fix:**
- Check diagnostic report
- Fix critical issues (topics, services, TF frames)
- Restart affected nodes

---

## Quick Diagnostic

### Run Full Diagnostic
```bash
ros2 run tyre_inspection_mission diagnostic_check
```

This checks:
- Topics availability
- Services availability
- Nav2 status
- TF frames
- Overall system health

---

## Code File Reference

| File | Purpose | Key Functions |
|------|---------|--------------|
| `mission_controller.py` | Main orchestrator | `state_machine_step()`, `bbox_callback()`, `navigate_to_pose()` |
| `vehicle_tracker.py` | Detection tracking | `process_vehicle_detections()`, `process_tyre_detections()` |
| `goal_planner.py` | Goal calculation | `calculate_license_plate_waypoint()`, `calculate_tyre_waypoint()` |
| `navigation_manager.py` | Nav2 interaction | Navigation callbacks |
| `photo_capture.py` | Photo capture | Service implementation |
| `diagnostics.py` | Health monitoring | `SystemDiagnostics`, `ErrorRecoveryHelper` |
| `config.py` | Configuration | All config classes |
| `exceptions.py` | Error handling | Custom exception classes |
| `utils.py` | Utilities | TF transforms, distance calculations |

---

**Last Updated:** Current Session  
**Version:** 1.0

# Quick Reference Card - Troubleshooting

## 🚨 Common Issues - Quick Fixes

### No Vehicle Detection
**Quick Check:**
```bash
# Check if YOLO is detecting
ros2 topic echo /yolo/detections --once

# Check if bounding boxes are published
ros2 topic echo /darknet_ros_3d/bounding_boxes --once

# Check detection stats
ros2 run tyre_inspection_mission diagnostic_check
```

**Common Causes:**
- Vehicle too close (<0.5m) or too far (>10m)
- Camera lights off
- YOLO node not running
- Confidence threshold too high

**Fix:** See [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md#-issue-no-vehicle-detection)

---

### Navigation Failures
**Quick Check:**
```bash
# Check Nav2 status
ros2 action list | grep navigate_to_pose

# Check current goal
ros2 topic echo /mission_controller/goal_markers

# Check TF frames
ros2 run tf2_ros tf2_echo base_footprint map
```

**Common Causes:**
- Goal too close (<0.6m from robot)
- Nav2 not ready
- TF frames missing
- Goal in obstacle

**Fix:** See [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures)

---

### State Machine Stuck
**Quick Check:**
```bash
# Check current state
ros2 topic echo /mission_controller/status --once

# Check for errors
ros2 run tyre_inspection_mission mission_controller 2>&1 | grep "\[ERR\]"
```

**Common States:**
- `SEARCHING_VEHICLES` → No vehicles detected
- `PLANNING` → Waypoint calculation failed
- `NAVIGATING_TO_*` → Navigation timeout
- `ERROR_RECOVERY` → Recovery in progress

**Fix:** See [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md#-issue-state-machine-stuck)

---

### Photo Capture Failures
**Quick Check:**
```bash
# Check service availability
ros2 service list | grep photo_capture

# Test service
ros2 service call /photo_capture/capture std_srvs/srv/Trigger
```

**Common Causes:**
- Service not running
- File permissions
- Disk space
- Camera not initialized

**Fix:** See [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md#-issue-photo-capture-failures)

---

## 🔧 Diagnostic Commands

### System Health Check
```bash
ros2 run tyre_inspection_mission diagnostic_check
```

### Check Topics
```bash
ros2 topic list
ros2 topic hz /darknet_ros_3d/bounding_boxes
ros2 topic hz /oak/points
ros2 topic hz /scan
```

### Check Services
```bash
ros2 service list | grep mission_controller
ros2 service list | grep photo_capture
```

### Check TF Frames
```bash
ros2 run tf2_ros tf2_echo base_footprint odom
ros2 run tf2_ros tf2_echo base_footprint map
```

### Monitor Mission
```bash
# Mission status
ros2 topic echo /mission_controller/status

# Detection messages
ros2 topic echo /darknet_ros_3d/bounding_boxes

# Navigation goals
ros2 topic echo /mission_controller/goal_markers
```

---

## 📊 Error Codes Quick Lookup

| Code | Issue | Quick Fix |
|------|-------|-----------|
| `NAV_001` | Goal too close | Increase distance, recalculate |
| `NAV_002` | Nav2 unavailable | Wait, check Nav2 status |
| `NAV_003` | Goal rejected | Use fallback waypoint |
| `DET_001` | No vehicles | Check YOLO, distance, lighting |
| `DET_003` | No tyres | Use fallback waypoints |
| `CAP_001` | Service unavailable | Check photo_capture node |
| `TF_001` | Frame not found | Wait for frame, check TF tree |

**Full Reference:** [Error Recovery Strategy](../../architecture/ERROR_RECOVERY_STRATEGY.md#error-code-reference)

---

## 🎯 State Machine Quick Reference

| State | What It Does | Common Issues |
|-------|-------------|---------------|
| `IDLE` | Waiting for start | None |
| `SEARCHING_VEHICLES` | Looking for vehicles | No detections → Check YOLO |
| `PLANNING` | Calculating waypoints | Timeout → Check poses |
| `NAVIGATING_TO_LICENSE_PLATE` | Moving to vehicle | Goal rejected → Check distance |
| `CAPTURING_LICENSE_PLATE` | Taking photo | Service unavailable |
| `DETECTING_TYRES` | Looking for tyres | No tyres → Fallback waypoints |
| `NAVIGATING_TO_TYRE` | Moving to tyre | Goal rejected → Check distance |
| `CAPTURING_TYRE` | Taking photo | Service unavailable |
| `CHECKING_COMPLETION` | Verifying completion | Missing tyres → Retry |
| `ERROR_RECOVERY` | Recovering from error | Check logs for error code |
| `MISSION_COMPLETE` | Mission finished | None |

---

## 🚀 Emergency Recovery

### Reset Mission
```bash
# Stop mission
ros2 service call /mission_controller/stop std_srvs/srv/Trigger

# Check state
ros2 topic echo /mission_controller/status --once

# Restart if needed
ros2 service call /mission_controller/start std_srvs/srv/Trigger
```

### Cancel Navigation
```bash
# Cancel active goal (if stuck)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

### Restart Nodes
```bash
# Restart mission controller
# (Kill and relaunch)

# Restart detection pipeline
# (Restart YOLO and segmentation nodes)
```

---

## 📖 Full Documentation

- **Troubleshooting Guide:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md)
- **Error Recovery:** [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md)
- **Module Organization:** [MODULE_ORGANIZATION.md](../../architecture/MODULE_ORGANIZATION.md)

---

**Last Updated:** Current Session  
**Quick Reference Version:** 1.0

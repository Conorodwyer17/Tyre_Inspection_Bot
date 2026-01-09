# Troubleshooting Guide - Tyre Inspection Mission

## Quick Reference: Where to Look for Issues

### 🎯 **Issue: No Vehicle Detection**
**Location:** `tyre_inspection_mission/detection/`
- **Check:** `mission_controller.py` → `bbox_callback()` (line ~1090)
- **Check:** `vehicle_tracker.py` → `process_vehicle_detections()`
- **Logs to check:**
  - `[DET]` category logs for detection events
  - YOLO node logs for "YOLO detected" messages
  - Segmentation processor logs for bounding box publishing
- **Common causes:**
  1. YOLO not detecting (check confidence threshold in launch file)
  2. Point cloud QoS mismatch (check `pointcloud_sub` QoS settings)
  3. Segmentation processor not publishing (check output topic)
  4. Vehicle too close (<0.5m) or too far (>10m)
  5. Lighting conditions (check camera lights are on)

### 🗺️ **Issue: Navigation Failures**
**Location:** `tyre_inspection_mission/navigation/`
- **Check:** `mission_controller.py` → `navigate_to_pose()` (line ~3365)
- **Check:** `goal_planner.py` → `calculate_goal_pose()`
- **Check:** `navigation_manager.py` → Navigation callbacks
- **Logs to check:**
  - `[NAV]` category logs for navigation events
  - Nav2 action server status
  - Goal rejection reasons
- **Common causes:**
  1. Goal too close to robot (<0.6m) - check `MIN_SAFE_DISTANCE`
  2. Goal in obstacle/costmap inflation zone
  3. Nav2 not ready (check action server connection)
  4. TF transforms failing (check frame availability)
  5. Invalid pose values (NaN/Inf)

### 📸 **Issue: Photo Capture Failures**
**Location:** `tyre_inspection_mission/capture/`
- **Check:** `mission_controller.py` → `capture_photo()` (line ~3833)
- **Check:** `photo_capture.py` → Service implementation
- **Logs to check:**
  - `[CAP]` category logs for capture events
  - Photo capture service availability
- **Common causes:**
  1. Service not available (check service is running)
  2. File permissions (check photo directory permissions)
  3. Disk space (check available storage)
  4. Camera not initialized

### 🔄 **Issue: State Machine Stuck**
**Location:** `tyre_inspection_mission/mission_controller.py`
- **Check:** `state_machine_step()` (line ~1636)
- **Check:** State-specific handlers (SEARCHING_VEHICLES, PLANNING, etc.)
- **Logs to check:**
  - `[STATE]` category logs for state transitions
  - Current state in status topic
- **Common causes:**
  1. Detection timeout (check `DETECTION_TIMEOUT`)
  2. Planning timeout (check `PLANNING_TIMEOUT`)
  3. Navigation timeout (check `NAVIGATION_TIMEOUT`)
  4. Missing fallback logic (check error recovery state)

### 🎯 **Issue: Tyre Detection Problems**
**Location:** `tyre_inspection_mission/detection/`
- **Check:** `mission_controller.py` → `process_tyre_detections()` (line ~1279)
- **Check:** `vehicle_tracker.py` → Tyre tracking logic
- **Logs to check:**
  - `[DET]` category logs for tyre detections
  - Fallback waypoint generation logs
- **Common causes:**
  1. Tyre class name mismatch (check `tyre_class_name` parameter)
  2. Tyres not in view (check camera angle)
  3. Detection timeout (check `DETECTION_TIMEOUT`)
  4. Fallback waypoints not generated (check `_generate_fallback_tyre_waypoints()`)

### 🔧 **Issue: LiDAR Fusion Not Working**
**Location:** `tyre_inspection_mission/detection/`
- **Check:** `mission_controller.py` → `process_lidar_vehicle_tracking()` (line ~920)
- **Check:** `lidar_callback()` (line ~850)
- **Logs to check:**
  - `[DET]` category logs for LiDAR tracking
  - LiDAR scan processing logs
- **Common causes:**
  1. LiDAR not publishing (check `/scan` topic)
  2. TF transforms failing (check `base_footprint` → `odom`)
  3. Cluster threshold too strict (check `LIDAR_CLUSTER_THRESHOLD`)
  4. Vehicle size mismatch (check `LIDAR_VEHICLE_MIN_SIZE` / `MAX_SIZE`)

---

## Diagnostic Commands

### Check System Health
```bash
# Check if all nodes are running
ros2 node list

# Check if topics are publishing
ros2 topic list
ros2 topic hz /darknet_ros_3d/bounding_boxes
ros2 topic hz /oak/points
ros2 topic hz /scan

# Check service availability
ros2 service list | grep photo_capture
ros2 service list | grep mission_controller

# Check TF frames
ros2 run tf2_ros tf2_echo base_footprint odom
```

### Check Detection Pipeline
```bash
# Monitor detection messages
ros2 topic echo /darknet_ros_3d/bounding_boxes --once

# Check YOLO detections
ros2 topic echo /yolo/detections --once

# Monitor mission status
ros2 topic echo /mission_controller/status
```

### Check Navigation
```bash
# Check Nav2 action server
ros2 action list | grep navigate_to_pose

# Monitor navigation goals
ros2 topic echo /mission_controller/goal_markers

# Check costmap
ros2 topic echo /local_costmap/costmap
```

---

## Error Codes Reference

### Detection Errors (DET_xxx)
- `DET_001`: No vehicles detected after timeout
- `DET_002`: LiDAR scan processing failed
- `DET_003`: Tyre detection failed
- `DET_004`: Point cloud not available

### Navigation Errors (NAV_xxx)
- `NAV_001`: Goal too close to robot
- `NAV_002`: Nav2 action server not available
- `NAV_003`: Goal rejected by Nav2
- `NAV_004`: Navigation timeout
- `NAV_005`: TF transform failed

### Planning Errors (PLAN_xxx)
- `PLAN_001`: Failed to calculate license plate waypoint
- `PLAN_002`: Failed to calculate tyre waypoint
- `PLAN_003`: Fallback waypoint generation failed
- `PLAN_004`: Invalid pose values
- `PLAN_005`: Invalid depth value

### Capture Errors (CAP_xxx)
- `CAP_001`: Photo capture service not available
- `CAP_002`: File write failed
- `CAP_003`: Camera not initialized

### Transform Errors (TF_xxx)
- `TF_001`: LookupException - frame not found
- `TF_002`: ConnectivityException - frame tree broken
- `TF_003`: ExtrapolationException - transform too old

---

## State Machine Debugging

### Current State Check
```bash
ros2 topic echo /mission_controller/status --once
```

### State Transitions
Monitor `[STATE]` category logs for state transitions:
- `IDLE` → `SEARCHING_VEHICLES`: Mission started
- `SEARCHING_VEHICLES` → `PLANNING`: Vehicle detected
- `PLANNING` → `VEHICLE_DETECTED`: Waypoints planned
- `VEHICLE_DETECTED` → `NAVIGATING_TO_LICENSE_PLATE`: Navigation started
- `NAVIGATING_TO_LICENSE_PLATE` → `CAPTURING_LICENSE_PLATE`: Arrived at vehicle
- `CAPTURING_LICENSE_PLATE` → `SWITCHING_TO_INSPECTION`: License plate captured
- `SWITCHING_TO_INSPECTION` → `DETECTING_TYRES`: Inspection mode active
- `DETECTING_TYRES` → `NAVIGATING_TO_TYRE`: Tyres detected
- `NAVIGATING_TO_TYRE` → `CAPTURING_TYRE`: Arrived at tyre
- `CAPTURING_TYRE` → `NAVIGATING_TO_TYRE`: Tyre captured, moving to next
- `NAVIGATING_TO_TYRE` → `CHECKING_COMPLETION`: All tyres processed
- `CHECKING_COMPLETION` → `MISSION_COMPLETE`: All vehicles processed

### Stuck State Recovery
If stuck in a state:
1. Check logs for error messages
2. Check if timeouts are being hit
3. Check if required services/topics are available
4. Use error recovery state: `ERROR_RECOVERY` should attempt recovery

---

## Configuration Tuning

### Detection Sensitivity
**File:** `config.py` → `DetectionConfig`
- `conf_threshold`: Lower = more detections (default: 0.15)
- `img_size`: Higher = better accuracy (default: 640)
- `frame_skip`: Lower = more frequent processing (default: 1)

### Navigation Safety
**File:** `config.py` → `NavigationConfig`
- `MIN_SAFE_DISTANCE`: Minimum distance from robot (default: 0.6m)
- `GOAL_ARRIVAL_DISTANCE`: Consider goal reached (default: 0.4m)
- `APPROACH_DISTANCE`: Distance to approach objects (default: 1.0m)

### Timeouts
**File:** `config.py`
- `DETECTION_TIMEOUT`: Vehicle search timeout (default: 60.0s)
- `NAVIGATION_TIMEOUT`: Navigation goal timeout (default: 60.0s)
- `PLANNING_TIMEOUT`: Waypoint planning timeout (default: 30.0s)

---

## Common Issues and Solutions

### Issue: "No bounding box messages received"
**Solution:**
1. Check segmentation processor is running: `ros2 node list | grep segmentation`
2. Check topic name matches: `/darknet_ros_3d/bounding_boxes`
3. Check QoS settings match between publisher and subscriber
4. Check point cloud is available: `ros2 topic hz /oak/points`

### Issue: "Goal rejected by Nav2"
**Solution:**
1. Check goal distance: Must be >0.6m from robot
2. Check costmap: Goal may be in obstacle/inflation zone
3. Check Nav2 status: `ros2 action list | grep navigate_to_pose`
4. Check TF frames: `ros2 run tf2_ros tf2_echo base_footprint map`

### Issue: "Vehicle not detected"
**Solution:**
1. Check YOLO is detecting: Look for "YOLO detected" logs
2. Check vehicle distance: Should be 0.5m - 10m from camera
3. Check lighting: Ensure camera lights are on
4. Check confidence threshold: May be too high
5. Check vehicle class: Must match `vehicle_class_names` parameter

### Issue: "Navigation stuck"
**Solution:**
1. Check navigation timeout: May need to increase
2. Check if goal is reachable: May be blocked
3. Check Nav2 status: Action server may be down
4. Check distance to goal: May be close enough to proceed
5. Use fallback: System should proceed if close enough

---

## Log Analysis

### Key Log Categories
- `[SYS]`: System initialization and health
- `[DET]`: Detection events (vehicles, tyres)
- `[NAV]`: Navigation operations
- `[PLAN]`: Waypoint planning
- `[CAP]`: Photo capture
- `[ERR]`: Errors and exceptions
- `[STATE]`: State machine transitions

### Filtering Logs
```bash
# Filter by category
ros2 run tyre_inspection_mission mission_controller 2>&1 | grep "\[DET\]"
ros2 run tyre_inspection_mission mission_controller 2>&1 | grep "\[NAV\]"
ros2 run tyre_inspection_mission mission_controller 2>&1 | grep "\[ERR\]"
```

---

## Emergency Recovery

### Manual State Reset
If the system is completely stuck:
1. Stop mission: `ros2 service call /mission_controller/stop std_srvs/srv/Trigger`
2. Check current state: `ros2 topic echo /mission_controller/status --once`
3. Restart mission: `ros2 service call /mission_controller/start std_srvs/srv/Trigger`

### Force Navigation Cancel
```bash
# Cancel active navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

### Reset Detection
If detection is broken:
1. Check YOLO node: `ros2 node list | grep ultralytics`
2. Restart segmentation: Restart `segmentation_processor` node
3. Check point cloud: `ros2 topic hz /oak/points`

---

## Performance Monitoring

### Detection Rate
Monitor detection statistics logs (every 30s):
- Messages received
- Messages with detections
- Vehicle detections
- Tyre detections

### Navigation Performance
Monitor navigation logs:
- Goal acceptance rate
- Navigation success rate
- Average navigation time
- Goal rejection reasons

### System Health
Monitor system health logs:
- TF frame availability
- Service availability
- Topic publishing rates
- Memory usage

---

## Getting Help

1. **Check logs first**: Look for `[ERR]` category logs
2. **Check state**: Verify current state in status topic
3. **Check diagnostics**: Run diagnostic commands above
4. **Check configuration**: Verify config values are appropriate
5. **Review troubleshooting guide**: This document
6. **Check architecture docs**: `docs/architecture/` folder

---

**Last Updated:** Current Session  
**Maintained By:** Development Team

# Pre-Flight Checklist: Real Vehicle Testing 🚗
## Complete System Validation Before Real Vehicle Test

**CRITICAL:** This checklist ensures the system is production-ready for real vehicle testing. Complete ALL items before proceeding.

---

## ✅ 1. BUILD & LAUNCH FILE CHECKS

### Build Verification
- [ ] Build completes without errors: `colcon build`
- [ ] All packages build successfully (no warnings about missing dependencies)
- [ ] Launch file syntax is correct (no import errors)

**FIXED:** Launch file `SetParameter` import error - now uses parameter passing directly

**Verify:**
```bash
cd ~/ugv_ws
colcon build
# Should complete with: "Summary: 33 packages finished"
```

### Launch File Test
- [ ] Launch file starts without errors
- [ ] All nodes initialize successfully
- [ ] No critical errors in first 10 seconds

**Test Command:**
```bash
ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true
# Press Ctrl+C after 10 seconds to verify all nodes started
```

---

## ✅ 2. HARDWARE CONNECTIONS

### Serial Port Configuration
- [ ] ESP32 serial port is correct: `/dev/ttyTHS1` (Jetson) or `/dev/ttyAMA0` (other)
- [ ] Serial port permissions: `sudo chmod 666 /dev/ttyTHS1` (if needed)
- [ ] Serial port exists: `ls -l /dev/ttyTHS1`
- [ ] Baud rate matches: 115200

**Check:**
```bash
ls -l /dev/ttyTHS1
# Should show: crw-rw-rw- ... /dev/ttyTHS1
```

### Camera Connection
- [ ] OAK-D camera is connected via USB
- [ ] Camera is detected: `lsusb | grep -i depthai` or `lsusb | grep -i oak`
- [ ] Camera topics publish: `/oak/rgb/image_rect`, `/oak/stereo/image_raw`
- [ ] Point cloud publishes: `/points`

**Check:**
```bash
ros2 topic list | grep oak
ros2 topic echo /oak/rgb/image_rect --once
# Should receive image message
```

### LiDAR Connection
- [ ] LiDAR is connected: `/dev/ttyACM0` (check launch file if different)
- [ ] LiDAR publishes scans: `/scan` topic
- [ ] Scan data is valid (check with `ros2 topic echo /scan --once`)

**Check:**
```bash
ros2 topic echo /scan --once
# Should receive laser scan message with ranges array
```

### IMU/Odometry
- [ ] Odometry publishes: `/odom` topic
- [ ] Odometry QoS is RELIABLE, TRANSIENT_LOCAL (verified in code)
- [ ] TF tree is valid: `ros2 run tf2_tools view_frames`

**Check:**
```bash
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
# Should generate frames.pdf showing TF tree
```

---

## ✅ 3. DETECTION SYSTEM VERIFICATION

### YOLO Model Loading
- [ ] Navigation model loads: `yolov8n-seg.pt` (for vehicle detection)
- [ ] Inspection model loads: `best.pt` (for tire detection)
- [ ] License plate class is in config: Check `config.yaml` has `"license_plate"` in `interested_classes`

**Verify:**
```bash
# Check config file
cat src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml | grep license_plate
# Should show: interested_classes: ["person","truck", "car", "tire", "tyre", "license_plate"]
```

### Detection Topics
- [ ] Bounding boxes publish: `/darknet_ros_3d/bounding_boxes`
- [ ] Segmentation publishes: `/ultralytics/segmentation/objects_segment`
- [ ] Detection confidence thresholds are set correctly:
  - Vehicle: 0.5 (50%)
  - License plate: 0.3 (30% - detection model)
  - Tire: 0.5 (50%)
  - OCR: 0.7 (70% per character), 0.8 (80% overall)

**Check:**
```bash
ros2 topic echo /darknet_ros_3d/bounding_boxes --once
# Should receive BoundingBoxes3d message with bounding_boxes array
```

### Two-Stage License Plate Detection
- [ ] License plate detection model enabled: `enable_license_plate_detection_model = true`
- [ ] Two-stage detection parameters are set correctly:
  - `license_plate_detection_confidence_threshold = 0.3`
  - `ocr_min_char_confidence = 0.7`
  - `ocr_min_global_confidence = 0.8`

**Verify Parameters:**
```bash
ros2 param get /mission_controller enable_license_plate_detection_model
ros2 param get /mission_controller license_plate_detection_confidence_threshold
ros2 param get /mission_controller ocr_min_char_confidence
ros2 param get /mission_controller ocr_min_global_confidence
```

---

## ✅ 4. NAVIGATION SYSTEM VERIFICATION

### Nav2 Configuration
- [ ] Nav2 launches successfully (with `use_mapping_nav:=true`)
- [ ] SLAM node is running: `slam_toolbox` or `cartographer`
- [ ] Nav2 nodes are active: `controller_server`, `planner_server`, `bt_navigator`
- [ ] Nav2 cmd_vel is remapped to `/cmd_vel/nav2` (not `/cmd_vel` directly)

**Check:**
```bash
ros2 node list | grep nav
# Should show: /controller_server, /planner_server, /bt_navigator, etc.

ros2 topic list | grep cmd_vel
# Should show: /cmd_vel, /cmd_vel/nav2, /cmd_vel/direct_control, /cmd_vel/emergency
```

### Command Velocity Multiplexer
- [ ] Multiplexer node is running: `/cmd_vel_multiplexer`
- [ ] Priority system is configured correctly:
  - Priority 1: Emergency (`/cmd_vel/emergency`)
  - Priority 2: Direct control (`/cmd_vel/direct_control`)
  - Priority 3: Nav2 (`/cmd_vel/nav2`)
- [ ] Multiplexer publishes to `/cmd_vel` (final output)

**Check:**
```bash
ros2 node info /cmd_vel_multiplexer
# Should show subscriptions to /cmd_vel/emergency, /cmd_vel/direct_control, /cmd_vel/nav2
# Should show publication to /cmd_vel
```

### Direct Navigation Fallback
- [ ] Direct navigation fallback is available (fallback when Nav2 fails)
- [ ] Watchdog timer is active (50Hz command publishing)
- [ ] Fallback publishes to `/cmd_vel/direct_control`

---

## ✅ 5. MOVEMENT SYSTEM VERIFICATION

### Base Controller (ugv_bringup)
- [ ] `ugv_bringup` node is running
- [ ] Serial communication with ESP32 works (check logs for connection success)
- [ ] Hardware deadzone is enforced (commands < 0.15 m/s clamped to 0)
- [ ] Command queue uses `Queue(maxsize=1)` for latest command only
- [ ] QoS is RELIABLE, TRANSIENT_LOCAL for `/cmd_vel` subscriber

**Check Logs:**
```bash
ros2 topic echo /cmd_vel --once
# Send test command and verify ESP32 receives it
# Check ugv_bringup logs for "Serial write successful" or similar
```

### Odometry Feedback
- [ ] Odometry publishes correctly: `/odom`
- [ ] Odometry reflects actual movement (check when robot moves)
- [ ] TF tree includes `odom -> base_footprint` transform
- [ ] Odometry QoS is RELIABLE, TRANSIENT_LOCAL (verified in code)

**Test Movement:**
```bash
# In one terminal:
ros2 topic echo /odom

# In another terminal, send test command:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# Verify odometry shows position change
```

### Movement Guarantee System
- [ ] Movement guarantee node monitors odometry
- [ ] Stuck detection works (no movement for 30s triggers emergency command)
- [ ] Emergency commands publish to `/cmd_vel/emergency` (Priority 1)

---

## ✅ 6. MISSION CONTROLLER VERIFICATION

### Mission State Machine
- [ ] Mission controller node is running: `/mission_controller`
- [ ] State transitions are correct:
  - IDLE → SEARCHING_TRUCKS → TRUCK_DETECTED → NAVIGATING_TO_LICENSE_PLATE → CAPTURING_LICENSE_PLATE → ...
- [ ] State machine handles all edge cases (already at goal, Nav2 failures, etc.)

**Check Current State:**
```bash
ros2 topic echo /mission_controller/state --once
# Should show current mission state
```

### Goal Calculation
- [ ] Robot-relative goal calculation works (from any angle: front, side, rear, diagonal)
- [ ] License plate approach always targets vehicle's FRONT (where plate is located)
- [ ] Goal validation works (checks min/max distance, position bounds)

### Navigation Completion Detection
- [ ] Arrival detection works correctly:
  - Distance check: `arrival_distance_threshold = 0.15m`
  - Orientation check: Verifies robot is facing correct direction
  - Minimum movement check: Handles case where robot starts at goal
- [ ] Nav2 SUCCEEDED is verified (not blindly trusted - checks physical distance)

### Error Recovery
- [ ] Nav2 goal rejection triggers recovery (costmap clear, goal recalculation, direct nav fallback)
- [ ] Vehicle movement during navigation updates goal correctly
- [ ] Stuck detection triggers recovery (reposition, alternative path)

---

## ✅ 7. PHOTO CAPTURE VERIFICATION

### Photo Capture Service
- [ ] Photo capture service is available: `/capture_photo`
- [ ] Service responds to requests
- [ ] Photos are saved to correct directory: `~/tyre_inspection_photos/`

**Test:**
```bash
ros2 service call /capture_photo std_srvs/srv/Trigger
# Should save photo and return success
ls ~/tyre_inspection_photos/
# Should show photo files
```

### License Plate OCR
- [ ] OCR library is installed: EasyOCR or Tesseract
- [ ] License plate detector initializes correctly
- [ ] Two-stage detection works (YOLO detection → OCR on detected region)
- [ ] Confidence thresholds filter correctly (70% char, 80% global)

### Folder Creation
- [ ] License plate folder creation works (uses OCR text or truck ID)
- [ ] Folder names are sanitized (invalid characters removed)
- [ ] Photos are organized correctly

---

## ✅ 8. TIRE DETECTION VERIFICATION

### Mode Switching
- [ ] Segmentation mode switches from "navigation" to "inspection" after license plate capture
- [ ] Mode switch timeout is handled (10s timeout)
- [ ] Mode switch verification works (checks actual mode change)

### Tire Detection
- [ ] Tire detection works with inspection model (`best.pt`)
- [ ] Tire confidence threshold is correct: 0.5 (50%)
- [ ] Tire stability check works (3 consecutive frames required)
- [ ] Tire position validation works (checks if tire is at vehicle corners)

### Tire Navigation
- [ ] Tire navigation goals are calculated correctly
- [ ] Robot approaches tire from suitable distance and orientation
- [ ] Tire re-detection works during navigation (updates position if tire moves)

### Tire Photo Capture
- [ ] Tire photo capture works
- [ ] Photo quality checking works (repositions if quality is poor)
- [ ] All tires are photographed (completion check works)

---

## ✅ 9. CONFIGURATION PARAMETERS

### Critical Parameters Verification
Run this script to verify all critical parameters:

```bash
#!/bin/bash
# Verify all critical parameters are set correctly

echo "=== Vehicle Detection Parameters ==="
ros2 param get /mission_controller detection_confidence_threshold
ros2 param get /mission_controller detection_stability_frames

echo "=== License Plate Detection Parameters ==="
ros2 param get /mission_controller enable_license_plate_detection_model
ros2 param get /mission_controller license_plate_detection_confidence_threshold
ros2 param get /mission_controller ocr_min_char_confidence
ros2 param get /mission_controller ocr_min_global_confidence

echo "=== Tire Detection Parameters ==="
ros2 param get /mission_controller tyre_detection_confidence_threshold
ros2 param get /mission_controller tyre_detection_stability_frames

echo "=== Navigation Parameters ==="
ros2 param get /mission_controller arrival_distance_threshold
ros2 param get /mission_controller min_goal_distance
ros2 param get /mission_controller goal_recalculation_distance

echo "=== Movement Parameters ==="
ros2 param get /ugv_bringup serial_port
ros2 param get /ugv_bringup serial_baudrate
ros2 param get /ugv_bringup hardware_deadzone_threshold
```

**Expected Values:**
- `detection_confidence_threshold`: 0.5 (50%)
- `license_plate_detection_confidence_threshold`: 0.3 (30%)
- `ocr_min_char_confidence`: 0.7 (70%)
- `ocr_min_global_confidence`: 0.8 (80%)
- `tyre_detection_confidence_threshold`: 0.5 (50%)
- `arrival_distance_threshold`: 0.15 (0.15m)
- `min_goal_distance`: 0.8 (0.8m)
- `goal_recalculation_distance`: 0.9 (0.9m)

---

## ✅ 10. RUNTIME VALIDATION

### Start Mission and Monitor
1. **Launch System:**
   ```bash
   ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true
   ```

2. **Monitor Key Topics:**
   ```bash
   # Terminal 1: Monitor mission state
   ros2 topic echo /mission_controller/state

   # Terminal 2: Monitor detections
   ros2 topic echo /darknet_ros_3d/bounding_boxes

   # Terminal 3: Monitor cmd_vel
   ros2 topic echo /cmd_vel

   # Terminal 4: Monitor odometry
   ros2 topic echo /odom
   ```

3. **Verify Logs:**
   - Mission controller logs show state transitions
   - No critical errors in any node
   - Detection logs show vehicles/tires detected with correct confidence

### Test Scenarios (Before Real Vehicle)

1. **Test Vehicle Detection:**
   - Place vehicle in view (or use test object)
   - Verify vehicle is detected with confidence >= 0.5
   - Verify state transitions to TRUCK_DETECTED

2. **Test License Plate Detection:**
   - With vehicle detected, verify license plate detection (if YOLO model supports it)
   - If license plate detected, verify two-stage detection logs:
     - "✅ LICENSE PLATE DETECTED via YOLO (Stage 1)"
     - "✅ TWO-STAGE DETECTION: Using YOLO-detected license plate bbox"
   - If not detected, verify heuristic fallback logs:
     - "⚠️ Using heuristic fallback (upper 40% of vehicle region)"

3. **Test Navigation:**
   - Send test navigation goal manually:
     ```bash
     ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
     ```
   - Verify robot moves toward goal
   - Verify arrival detection works (robot stops when close enough)

4. **Test Emergency Stop:**
   - Send emergency stop command:
     ```bash
     ros2 topic pub /cmd_vel/emergency geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
     ```
   - Verify robot stops immediately (emergency has Priority 1)

---

## ✅ 11. SAFETY CHECKS

### Emergency Stop
- [ ] Emergency stop works: `/cmd_vel/emergency` topic is available
- [ ] Emergency stop has Priority 1 (highest priority)
- [ ] Emergency stop can be triggered manually if needed

**Emergency Stop Command:**
```bash
ros2 topic pub /cmd_vel/emergency geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -1
```

### Physical Safety
- [ ] Robot has physical emergency stop button (if available)
- [ ] Robot is on stable surface (won't fall)
- [ ] Clear area around robot (no obstacles or people)
- [ ] Battery is charged (if applicable)
- [ ] All cables are secure (won't disconnect during movement)

### Test Area
- [ ] Test area is clear and safe
- [ ] Vehicle is stationary (won't move during inspection)
- [ ] Lighting conditions are adequate (for camera and detection)
- [ ] No people or obstacles in robot path

---

## ✅ 12. FINAL VERIFICATION

### System Health Check
Run this comprehensive check before real vehicle test:

```bash
#!/bin/bash
echo "=== System Health Check ==="

echo "1. Checking nodes..."
ros2 node list | grep -E "(mission_controller|cmd_vel_multiplexer|ugv_bringup|ultralytics|segmentation_processor)" && echo "✅ All critical nodes running" || echo "❌ Missing nodes"

echo "2. Checking topics..."
ros2 topic list | grep -E "(cmd_vel|odom|bounding_boxes|mission_controller/state)" && echo "✅ All critical topics available" || echo "❌ Missing topics"

echo "3. Checking services..."
ros2 service list | grep -E "(capture_photo|/navigate_to_pose)" && echo "✅ All critical services available" || echo "❌ Missing services"

echo "4. Checking TF tree..."
ros2 run tf2_tools view_frames && echo "✅ TF tree valid" || echo "❌ TF tree invalid"

echo "5. Checking serial port..."
[ -e /dev/ttyTHS1 ] && echo "✅ Serial port /dev/ttyTHS1 exists" || echo "❌ Serial port missing"

echo "6. Checking camera..."
ros2 topic hz /oak/rgb/image_rect --window 5 && echo "✅ Camera publishing" || echo "❌ Camera not publishing"

echo "=== Health Check Complete ==="
```

---

## 🚀 READY FOR REAL VEHICLE TEST

**Once ALL items above are checked ✅, the system is ready for real vehicle testing.**

### Real Vehicle Test Procedure

1. **Position Vehicle:**
   - Place vehicle in test area
   - Ensure license plate is visible (front of vehicle)
   - Ensure tires are accessible (not obstructed)

2. **Position Robot:**
   - Place robot at starting position (any angle relative to vehicle is OK)
   - Ensure robot can see vehicle
   - Ensure clear path for navigation

3. **Start Mission:**
   ```bash
   ros2 launch tyre_inspection_mission autonomous_inspection.launch.py use_mapping_nav:=true
   ```

4. **Monitor Mission:**
   - Watch mission state transitions
   - Monitor detection logs
   - Monitor navigation progress
   - Monitor photo capture

5. **Expected Behavior:**
   - Robot detects vehicle (confidence >= 0.5)
   - Robot navigates to license plate position (always approaches front)
   - Robot captures license plate photo (two-stage detection preferred)
   - Robot switches to inspection mode
   - Robot detects tires (confidence >= 0.5)
   - Robot navigates to each tire sequentially
   - Robot captures tire photos
   - Mission completes successfully

6. **If Issues Occur:**
   - Check logs for errors
   - Verify parameters are correct
   - Check hardware connections
   - Use emergency stop if needed
   - Review this checklist again

---

## 📝 POST-TEST VERIFICATION

After real vehicle test:

- [ ] All photos were captured (license plate + 4 tires minimum)
- [ ] Photos are in correct folders (license plate folder named correctly)
- [ ] OCR text was extracted (if license plate detected)
- [ ] Mission completed successfully (state = MISSION_COMPLETE)
- [ ] No critical errors in logs
- [ ] Robot returned to safe position (if applicable)

---

## 🎯 SUCCESS CRITERIA

**Mission is successful if:**
1. ✅ Vehicle is detected reliably
2. ✅ License plate is captured (with text if OCR works)
3. ✅ All tires are detected (minimum 4 tires)
4. ✅ All tires are photographed
5. ✅ Robot navigates smoothly (no jerky movements)
6. ✅ Robot stops at correct positions (not too close, not too far)
7. ✅ Mission completes without manual intervention
8. ✅ No false positives (no incorrect detections)

---

**Good luck with your real vehicle test! 🚗🤖**

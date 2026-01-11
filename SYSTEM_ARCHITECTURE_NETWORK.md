# COMPLETE SYSTEM ARCHITECTURE NETWORK
## Full Communication Flow and File Dependencies

This document provides a complete visualization of the autonomous tyre inspection robot's system architecture, showing every communication path, file dependency, and data flow from mission start to completion.

---

## TOPIC COMMUNICATION NETWORK

### Published Topics

```
┌─────────────────────────────────────────────────────────────────┐
│ CAMERA TOPICS                                                     │
├─────────────────────────────────────────────────────────────────┤
│ /oak/rgb/image_rect (sensor_msgs/Image)                          │
│   └─> Published by: oak_d_lite node (camera driver)              │
│   └─> Subscribed by:                                             │
│       ├─> segmentation_processor_node (YOLO detection)           │
│       ├─> mission_controller (OCR, system health)                │
│       └─> detection_visualizer (visualization)                   │
│                                                                   │
│ /oak/stereo/image_raw (sensor_msgs/Image)                        │
│   └─> Published by: oak_d_lite node (camera driver)              │
│   └─> Subscribed by: point_cloud_xyz_node (depth → point cloud) │
│                                                                   │
│ /oak/stereo/camera_info (sensor_msgs/CameraInfo)                 │
│   └─> Published by: oak_d_lite node (camera driver)              │
│   └─> Subscribed by: point_cloud_xyz_node (calibration)          │
│                                                                   │
│ /oak/depth/points (sensor_msgs/PointCloud2)                      │
│   └─> Published by: oak_d_lite node (camera driver)              │
│   └─> Subscribed by: mission_controller (3D position)            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ POINT CLOUD TOPICS                                               │
├─────────────────────────────────────────────────────────────────┤
│ /points (sensor_msgs/PointCloud2)                                │
│   └─> Published by: point_cloud_xyz_node (depth_image_proc)     │
│   └─> Subscribed by: segmentation_processor_node (3D detection) │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ DETECTION TOPICS                                                 │
├─────────────────────────────────────────────────────────────────┤
│ /darknet_ros_3d/bounding_boxes (gb_visual_detection_3d_msgs/BoundingBoxes3d) │
│   └─> Published by: segmentation_processor_node (C++)           │
│   └─> Subscribed by:                                             │
│       ├─> mission_controller (primary detection input)           │
│       └─> detection_visualizer (visualization)                   │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ NAVIGATION TOPICS                                                │
├─────────────────────────────────────────────────────────────────┤
│ /cmd_vel (geometry_msgs/Twist) - FINAL OUTPUT                    │
│   └─> Published by: cmd_vel_multiplexer (NEW - Priority-based)  │
│   └─> Subscribed by:                                             │
│       ├─> ugv_bringup (ESP32 motor control)                      │
│       ├─> direct_navigation_fallback (monitor for zero commands) │
│       ├─> movement_guarantee (monitor for movement)              │
│       └─> movement_diagnostic (diagnostics)                      │
│                                                                   │
│ /cmd_vel/emergency (geometry_msgs/Twist) - Priority 1            │
│   └─> Published by: movement_guarantee (Emergency override)      │
│   └─> Subscribed by: cmd_vel_multiplexer                         │
│                                                                   │
│ /cmd_vel/direct_control (geometry_msgs/Twist) - Priority 2       │
│   └─> Published by: direct_navigation_fallback (Primary control) │
│   └─> Subscribed by: cmd_vel_multiplexer                         │
│                                                                   │
│ /cmd_vel/nav2 (geometry_msgs/Twist) - Priority 3                 │
│   └─> Published by: Nav2 controller (Fine positioning)           │
│   └─> Subscribed by: cmd_vel_multiplexer                         │
│                                                                   │
│ /cmd_vel/teleop (geometry_msgs/Twist) - Priority 4               │
│   └─> Published by: teleop_twist_joy (Manual override)           │
│   └─> Subscribed by: cmd_vel_multiplexer                         │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ ODOMETRY TOPICS                                                  │
├─────────────────────────────────────────────────────────────────┤
│ /odom/odom_raw (std_msgs/Float32MultiArray)                      │
│   └─> Published by: ugv_bringup (ESP32 encoder data)             │
│   └─> Subscribed by: base_node (odometry calculation)            │
│                                                                   │
│ /odom (nav_msgs/Odometry)                                        │
│   └─> Published by: base_node (odometry processing)              │
│   └─> Subscribed by:                                             │
│       ├─> mission_controller (robot position tracking)           │
│       ├─> direct_navigation_fallback (goal progress)             │
│       ├─> movement_guarantee (movement verification)             │
│       ├─> movement_diagnostic (movement diagnostics)             │
│       └─> Nav2 (localization)                                    │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ IMU TOPICS                                                       │
├─────────────────────────────────────────────────────────────────┤
│ /imu/data_raw (sensor_msgs/Imu)                                  │
│   └─> Published by: ugv_bringup (ESP32 IMU data)                 │
│   └─> Subscribed by: complementary_filter_node (filtering)       │
│                                                                   │
│ /imu/data (sensor_msgs/Imu)                                      │
│   └─> Published by: complementary_filter_node (filtered IMU)     │
│   └─> Subscribed by: base_node (odometry fusion)                 │
│                                                                   │
│ /imu/mag (sensor_msgs/MagneticField)                             │
│   └─> Published by: ugv_bringup (ESP32 magnetometer)             │
│   └─> Subscribed by: complementary_filter_node (orientation)     │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ LIDAR TOPICS                                                     │
├─────────────────────────────────────────────────────────────────┤
│ /scan (sensor_msgs/LaserScan)                                    │
│   └─> Published by: ldlidar node (LiDAR driver)                  │
│   └─> Subscribed by:                                             │
│       ├─> Nav2 (mapping, obstacle avoidance)                     │
│       ├─> rf2o_laser_odometry (odometry)                         │
│       └─> SLAM (mapping)                                         │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ MISSION CONTROL TOPICS                                           │
├─────────────────────────────────────────────────────────────────┤
│ /mission_controller/status (std_msgs/String)                     │
│   └─> Published by: mission_controller (mission status)          │
│   └─> Subscribed by: (external monitoring)                       │
│                                                                   │
│ /segmentation_mode (std_msgs/String)                             │
│   └─> Published by: mission_controller (mode switching)          │
│   └─> Subscribed by: segmentation_processor_node (YOLO mode)     │
└─────────────────────────────────────────────────────────────────┘
```

### Subscribed Topics Summary

| Node | Subscriptions | Purpose |
|------|--------------|---------|
| `mission_controller` | `/darknet_ros_3d/bounding_boxes`, `/oak/rgb/image_rect`, `/oak/depth/points`, `/odom`, `/cmd_vel` | Mission orchestration, detection processing, position tracking |
| `direct_navigation_fallback` | `/odom`, `/cmd_vel` | Direct velocity control, goal progress, Nav2 monitoring |
| `movement_guarantee` | `/odom`, `/cmd_vel` | Movement verification, stuck detection, emergency override |
| `movement_diagnostic` | `/cmd_vel`, `/odom` | Real-time diagnostics, movement verification |
| `ugv_bringup` | `/cmd_vel`, `/ugv/led_ctrl` | ESP32 communication, motor control, LED control |
| `segmentation_processor_node` | `/oak/rgb/image_rect`, `/points`, `/segmentation_mode` | YOLO detection, 3D bounding box generation |
| `base_node` | `/odom/odom_raw`, `/imu/data` | Odometry calculation, TF publishing |
| `Nav2` | `/odom`, `/scan`, `/tf`, `/tf_static` | Navigation, mapping, obstacle avoidance |

---

## SERVICE COMMUNICATION NETWORK

### Services

```
┌─────────────────────────────────────────────────────────────────┐
│ MISSION CONTROL SERVICES                                         │
├─────────────────────────────────────────────────────────────────┤
│ /mission_controller/start (std_srvs/Trigger)                    │
│   └─> Server: mission_controller::start_mission_callback()      │
│   └─> Client: (external/user)                                   │
│   └─> Action: Start mission, transition to SEARCHING_TRUCKS     │
│                                                                   │
│ /mission_controller/stop (std_srvs/Trigger)                     │
│   └─> Server: mission_controller::stop_mission_callback()       │
│   └─> Client: (external/user)                                   │
│   └─> Action: Stop mission, transition to IDLE                  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ PHOTO CAPTURE SERVICES                                           │
├─────────────────────────────────────────────────────────────────┤
│ /photo_capture/capture (std_srvs/Trigger)                       │
│   └─> Server: photo_capture_service                             │
│   └─> Client: mission_controller                                │
│   └─> Action: Capture photo from camera, save to disk           │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ NAV2 SERVICES                                                    │
├─────────────────────────────────────────────────────────────────┤
│ /local_costmap/clear_entirely_global_costmap (nav2_msgs/ClearEntireCostmap) │
│   └─> Server: Nav2 local costmap                                │
│   └─> Client: mission_controller (error recovery)               │
│   └─> Action: Clear local costmap for recovery                  │
│                                                                   │
│ /global_costmap/clear_entirely_global_costmap (nav2_msgs/ClearEntireCostmap) │
│   └─> Server: Nav2 global costmap                               │
│   └─> Client: mission_controller (error recovery)               │
│   └─> Action: Clear global costmap for recovery                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## ACTION COMMUNICATION NETWORK

### Actions

```
┌─────────────────────────────────────────────────────────────────┐
│ NAV2 ACTIONS                                                     │
├─────────────────────────────────────────────────────────────────┤
│ /navigate_to_pose (nav2_msgs/NavigateToPose)                    │
│   └─> Server: Nav2 navigation stack                             │
│   └─> Client: mission_controller                                │
│   └─> Action: Navigate robot to goal pose                       │
│   └─> Feedback: Current pose, distance to goal                  │
│   └─> Result: Success/failure, final pose                       │
└─────────────────────────────────────────────────────────────────┘
```

---

## TF TRANSFORM NETWORK

### Transform Tree

```
┌─────────────────────────────────────────────────────────────────┐
│ TRANSFORM TREE                                                   │
├─────────────────────────────────────────────────────────────────┤
│ map (if SLAM/Nav2 enabled)                                      │
│   └─> odom (base_node, Nav2)                                    │
│       └─> base_link (base_node)                                 │
│           ├─> base_footprint (robot_description)                │
│           ├─> base_imu_link (robot_description)                 │
│           ├─> camera_link (robot_description)                   │
│           │   └─> camera_optical_frame (robot_description)      │
│           └─> lidar_link (robot_description)                    │
│                                                                   │
│ Transform Flow:                                                  │
│   1. Detection pose (camera_link) → TF → map/odom               │
│   2. Robot pose (base_link) → TF → map/odom                     │
│   3. Goal pose (map/odom) → Nav2 → base_link                    │
└─────────────────────────────────────────────────────────────────┘
```

### Transform Usage

| Transform | Publisher | Subscriber | Purpose |
|-----------|-----------|------------|---------|
| `map → odom` | Nav2 (SLAM) | mission_controller | Global navigation frame |
| `odom → base_link` | base_node | All nodes | Robot position tracking |
| `base_link → camera_link` | robot_state_publisher | mission_controller | Detection pose transformation |
| `base_link → lidar_link` | robot_state_publisher | Nav2, SLAM | LiDAR frame for mapping |

---

## FILE DEPENDENCY NETWORK

### Core Mission Files

```
mission_controller.py (CORE ORCHESTRATOR)
├── imports from:
│   ├── detection/
│   │   ├── license_plate_detector.py
│   │   ├── tyre_re_detection_handler.py
│   │   ├── tyre_identifier.py
│   │   ├── tyre_position_validator.py
│   │   ├── tyre_completeness_verifier.py
│   │   └── tyre_side_identifier.py
│   ├── navigation/
│   │   ├── direct_navigation_fallback.py ⭐ PRIMARY NAVIGATION
│   │   ├── navigation_manager.py
│   │   ├── navigation_failure_handler.py
│   │   ├── goal_recalculator.py
│   │   ├── tyre_goal_validator.py
│   │   ├── tyre_path_optimizer.py
│   │   ├── tyre_pose_refiner.py
│   │   ├── tyre_navigation_context.py
│   │   ├── vehicle_obstacle_manager.py
│   │   ├── vehicle_monitor.py
│   │   ├── visual_verifier.py
│   │   ├── local_search.py
│   │   └── path_alternatives.py
│   ├── capture/
│   │   ├── tyre_capture_repositioner.py
│   │   ├── tyre_capture_verifier.py
│   │   └── photo_quality_checker.py
│   ├── core/
│   │   ├── movement_guarantee.py ⭐ MOVEMENT ENFORCEMENT
│   │   ├── mission_resumer.py
│   │   ├── mission_timeout_handler.py
│   │   └── mission_state_manager.py
│   └── diagnostics/
│       ├── movement_diagnostic.py ⭐ DIAGNOSTICS
│       ├── mission_logger.py
│       └── mission_monitor.py
│
└── uses ROS 2 interfaces:
    ├── Topics: /darknet_ros_3d/bounding_boxes, /odom, /cmd_vel, ...
    ├── Services: /photo_capture/capture, ...
    └── Actions: /navigate_to_pose (Nav2)
```

### Navigation Files (Critical Path)

```
direct_navigation_fallback.py ⭐
├── publishes: /cmd_vel/direct_control (Priority 2)
├── subscribes: /odom, /cmd_vel
├── uses: cmd_vel_multiplexer (NEW) for final output
└── features:
    ├── RELIABLE QoS with TRANSIENT_LOCAL durability
    ├── 50Hz watchdog timer for continuous publishing
    ├── Nav2 zero-command override
    ├── Goal progress tracking
    └── Physical movement verification

movement_guarantee.py ⭐
├── publishes: /cmd_vel/emergency (Priority 1)
├── subscribes: /odom, /cmd_vel
├── uses: cmd_vel_multiplexer (NEW) for final output
└── features:
    ├── Independent movement monitoring
    ├── Stuck detection via odom distance calculation
    ├── Emergency force movement (0.5 m/s)
    ├── Slow movement detection
    └── 50Hz watchdog timer

cmd_vel_multiplexer.py (NEW - DAY 1) ⭐
├── publishes: /cmd_vel (FINAL OUTPUT - RELIABLE QoS)
├── subscribes:
│   ├── /cmd_vel/emergency (Priority 1)
│   ├── /cmd_vel/direct_control (Priority 2)
│   ├── /cmd_vel/nav2 (Priority 3)
│   └── /cmd_vel/teleop (Priority 4)
└── features:
    ├── Priority-based arbitration
    ├── RELIABLE QoS with TRANSIENT_LOCAL durability
    ├── 50Hz publishing rate
    └── Highest priority command always wins
```

### Hardware Control Files (Critical Path)

```
ugv_bringup.py (CRITICAL - LOW-LEVEL CONTROL)
├── publishes:
│   ├── /imu/data_raw
│   ├── /imu/mag
│   ├── /odom/odom_raw
│   └── /voltage
├── subscribes: /cmd_vel (FINAL INPUT)
├── features:
│   ├── Serial communication with ESP32 (/dev/ttyTHS1)
│   ├── cmd_vel_callback() converts Twist → T:1 command
│   ├── Differential drive kinematics (WHEELBASE_M = 0.175m)
│   ├── Speed scaling (MAX_ROBOT_SPEED_MPS = 1.3 m/s)
│   ├── PWM command range (-0.5 to +0.5)
│   └── Queue-based async command sending
└── flow:
    cmd_vel → cmd_vel_callback() → BaseController::send_command() →
    command_queue → process_commands thread → Serial write →
    ESP32 → Motors

BaseController (ugv_bringup.py internal class)
├── methods:
│   ├── __init__(): Open serial port, start process_commands thread
│   ├── send_command(): Queue command for async sending
│   ├── process_commands(): Thread that writes to serial (T:1 format)
│   ├── feedback_data(): Read ESP32 feedback (odometry, IMU)
│   └── write_command_direct(): Direct serial write (immediate)
└── serial communication:
    ├── Port: /dev/ttyTHS1 (Jetson) or /dev/ttyAMA0 (other)
    ├── Baud: 115200 (GPIO UART standard)
    ├── Format: JSON {"T": 1, "L": left, "R": right}\n
    └── Flush immediately after write for real-time control
```

---

## STATE MACHINE FLOW

### Mission States

```
┌─────────────────────────────────────────────────────────────────┐
│ MISSION STATE MACHINE                                            │
├─────────────────────────────────────────────────────────────────┤
│ IDLE                                                             │
│   ├─> Trigger: /mission_controller/start service                │
│   └─> Next: SEARCHING_TRUCKS                                    │
│                                                                   │
│ SEARCHING_TRUCKS                                                 │
│   ├─> Subscribe: /darknet_ros_3d/bounding_boxes                  │
│   ├─> Process: process_truck_detections()                        │
│   ├─> Check: Stability (frame_count >= stability_frames)         │
│   ├─> Success: Stable vehicle detected                           │
│   │   └─> Next: TRUCK_DETECTED                                   │
│   └─> Timeout: vehicle_search_timeout                            │
│       └─> Next: ERROR_RECOVERY                                   │
│                                                                   │
│ TRUCK_DETECTED                                                   │
│   ├─> Calculate: license_pose (approach_distance = 2.5m)         │
│   ├─> Transform: camera_link → map/odom via TF                   │
│   ├─> Validate: Goal in free space, path clear                   │
│   ├─> Decision: Distance > 2.0m?                                 │
│   │   ├─> YES: direct_navigation_fallback.activate()             │
│   │   └─> NO: nav_client.send_goal_async()                       │
│   └─> Next: NAVIGATING_TO_LICENSE_PLATE                          │
│                                                                   │
│ NAVIGATING_TO_LICENSE_PLATE                                      │
│   ├─> Monitor: nav_result_future, direct_nav goal progress       │
│   ├─> Check: Nav2 cmd_vel timeout (0.5s)                         │
│   │   └─> If timeout: direct_navigation_fallback.activate()      │
│   ├─> Monitor: movement_guarantee (stuck detection)              │
│   ├─> Check: Goal arrival (distance < 0.15m, moved >= 0.5m)     │
│   ├─> Success: Goal reached                                      │
│   │   └─> Next: CAPTURING_LICENSE_PLATE                          │
│   └─> Failure: Navigation timeout, unreachable goal              │
│       └─> Next: ERROR_RECOVERY                                   │
│                                                                   │
│ CAPTURING_LICENSE_PLATE                                          │
│   ├─> Call: /photo_capture/capture service                       │
│   ├─> Process: OCR (optional)                                    │
│   ├─> Success: Photo captured                                    │
│   │   └─> Next: SWITCHING_TO_INSPECTION                          │
│   └─> Failure: Capture timeout                                   │
│       └─> Retry (max attempts: 3)                                │
│                                                                   │
│ SWITCHING_TO_INSPECTION                                          │
│   ├─> Publish: /segmentation_mode = "inspection"                 │
│   ├─> Wait: mode_switch_wait_time (3.0s)                         │
│   ├─> Verify: Mode switch confirmed                              │
│   ├─> Success: Mode switched                                     │
│   │   └─> Next: DETECTING_TYRES                                  │
│   └─> Failure: Mode switch timeout                               │
│       └─> Retry (max attempts: 3)                                │
│                                                                   │
│ DETECTING_TYRES                                                  │
│   ├─> Subscribe: /darknet_ros_3d/bounding_boxes (tyre class)     │
│   ├─> Process: process_tyre_detections()                         │
│   ├─> Check: Stability (tyre_stability_frames = 3)               │
│   ├─> Validate: Tyre position, dimensions                        │
│   ├─> Group: Tyres by side (left/right)                          │
│   ├─> Success: All tyres detected (min: 4, max: 6)               │
│   │   └─> Next: NAVIGATING_TO_TYRE                               │
│   └─> Timeout: tyre_detection_timeout (30.0s)                    │
│       └─> Next: ERROR_RECOVERY                                   │
│                                                                   │
│ NAVIGATING_TO_TYRE                                               │
│   ├─> Select: Next tyre from list (optimized path)               │
│   ├─> Calculate: tyre_pose (optimal_tyre_distance = 0.8-1.5m)   │
│   ├─> Decision: Distance > 2.0m?                                 │
│   │   ├─> YES: direct_navigation_fallback.activate()             │
│   │   └─> NO: nav_client.send_goal_async()                       │
│   ├─> Monitor: Navigation progress, stuck detection              │
│   ├─> Success: Goal reached                                      │
│   │   └─> Next: CAPTURING_TYRE                                   │
│   └─> Failure: Navigation timeout, unreachable goal              │
│       └─> Retry (max attempts: max_navigation_attempts_per_tyre) │
│                                                                   │
│ CAPTURING_TYRE                                                   │
│   ├─> Verify: Tyre position, camera angle                        │
│   ├─> Reposition: If needed (max attempts: 3)                    │
│   ├─> Call: /photo_capture/capture service                       │
│   ├─> Verify: Photo quality (brightness, contrast, sharpness)    │
│   ├─> Success: Photo captured and verified                       │
│   │   ├─> Mark: Tyre as photographed                             │
│   │   └─> Next: CHECKING_COMPLETION                              │
│   └─> Failure: Capture timeout, quality check failed             │
│       └─> Retry (max attempts: max_tyre_capture_attempts)        │
│                                                                   │
│ CHECKING_COMPLETION                                              │
│   ├─> Check: All tyres photographed?                             │
│   ├─> Yes: Mission complete                                      │
│   │   └─> Next: MISSION_COMPLETE                                 │
│   └─> No: Missing tyres                                          │
│       ├─> Re-search: Local search around vehicle                 │
│       ├─> Success: Missing tyres found                           │
│       │   └─> Next: NAVIGATING_TO_TYRE                           │
│       └─> Failure: Tyres not found after retries                 │
│           └─> Next: ERROR_RECOVERY                               │
│                                                                   │
│ MISSION_COMPLETE                                                 │
│   └─> Final state: Mission successfully completed                │
│                                                                   │
│ ERROR_RECOVERY                                                   │
│   ├─> Analyze: Error cause (navigation, detection, capture)      │
│   ├─> Recovery: Multi-level strategies                           │
│   │   ├─> Costmap clearing                                       │
│   │   ├─> Goal recalculation                                     │
│   │   ├─> Alternative path generation                            │
│   │   └─> Mode reset                                             │
│   ├─> Success: Recovery successful                               │
│   │   └─> Return to previous state                               │
│   └─> Failure: Recovery failed after max attempts                │
│       └─> Next: IDLE (manual intervention required)              │
└─────────────────────────────────────────────────────────────────┘
```

---

## DATA FLOW DIAGRAM

### Complete Mission Flow

```
┌─────────────────────────────────────────────────────────────────┐
│ MISSION START                                                    │
│   └─> Service Call: /mission_controller/start                    │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ CAMERA → DETECTION PIPELINE                                      │
│   Camera → YOLO Segmentation → 3D Processing → Bounding Boxes    │
│   └─> /darknet_ros_3d/bounding_boxes                              │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ MISSION CONTROLLER PROCESSING                                    │
│   ├─> Receive: BoundingBoxes3d message                          │
│   ├─> Filter: Vehicle class (truck/car), confidence threshold   │
│   ├─> Validate: Dimensions, distance, 3D position                │
│   ├─> Track: Stability across frames (frame_count)               │
│   └─> Deduplicate: Merge nearby detections                       │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ STABLE DETECTION CONFIRMED                                       │
│   └─> State: TRUCK_DETECTED                                       │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ GOAL CALCULATION                                                 │
│   ├─> Extract: Vehicle 3D position (camera_link frame)          │
│   ├─> Transform: camera_link → map/odom (via TF)                │
│   ├─> Calculate: license_pose (approach_distance = 2.5m)         │
│   ├─> Validate: Goal in free space, path clear                   │
│   └─> Decision: Distance > 2.0m?                                 │
└─────────────────────────────────────────────────────────────────┘
                            │
            ┌───────────────┴───────────────┐
            │                               │
            ▼                               ▼
┌───────────────────────────┐  ┌───────────────────────────┐
│ DIRECT CONTROL (>2.0m)   │  │ NAV2 (<2.0m)               │
│   └─> activate()           │  │   └─> send_goal_async()    │
│       ├─> Set goal         │  │       └─> Nav2 controller │
│       ├─> Start watchdog   │  │           └─> /cmd_vel     │
│       └─> Publish cmd_vel  │  │                           │
│           (50Hz)           │  │                           │
└───────────────────────────┘  └───────────────────────────┘
            │                               │
            └───────────────┬───────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ COMMAND ARBITRATION                                              │
│   └─> cmd_vel_multiplexer                                         │
│       ├─> Priority 1: Emergency (movement_guarantee)             │
│       ├─> Priority 2: Direct Control                             │
│       ├─> Priority 3: Nav2                                       │
│       └─> Priority 4: Teleop                                     │
│       └─> Publish: Highest priority command                      │
│           └─> /cmd_vel (FINAL - RELIABLE QoS)                    │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ HARDWARE CONTROL                                                 │
│   └─> ugv_bringup::cmd_vel_callback()                            │
│       ├─> Extract: linear.x, angular.z                           │
│       ├─> Convert: Differential drive kinematics                  │
│       ├─> Scale: (speed / MAX_SPEED) * MAX_PWM                   │
│       └─> Send: T:1 command to ESP32 via serial                  │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ ESP32 MICROCONTROLLER                                            │
│   ├─> Receive: T:1 JSON command via GPIO UART                    │
│   ├─> Parse: Extract L and R values (-0.5 to +0.5)              │
│   ├─> Convert: PWM duty cycle                                    │
│   └─> Execute: Drive motors                                      │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHYSICAL MOVEMENT                                                │
│   └─> Robot moves toward goal                                    │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ ODOMETRY FEEDBACK                                                │
│   ESP32 → Serial: {"T": 1001, "odl": left, "odr": right}        │
│     └─> ugv_bringup::feedback_loop()                             │
│         └─> publish_odom_raw() → /odom/odom_raw                  │
│             └─> base_node → /odom                                │
│                 └─> TF: /odom → /base_link                       │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ MOVEMENT VERIFICATION                                            │
│   ├─> movement_guarantee: Calculate distance_moved               │
│   ├─> direct_navigation_fallback: Update robot_pose              │
│   ├─> mission_controller: Check goal progress                    │
│   └─> movement_diagnostic: Report status                         │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ GOAL ARRIVAL CHECK                                               │
│   ├─> Distance < arrival_distance_threshold (0.15m)?            │
│   ├─> Distance traveled >= min_movement_before_arrival_check?   │
│   └─> If YES → State: CAPTURING_LICENSE_PLATE                    │
│       └─> Continue mission...                                    │
└─────────────────────────────────────────────────────────────────┘
```

---

## CRITICAL PATHS

### Path 1: Detection → Navigation (Primary Flow)

```
Camera → YOLO → 3D Processing → /darknet_ros_3d/bounding_boxes →
mission_controller::bbox_callback() → process_truck_detections() →
Stability check → State: TRUCK_DETECTED → Calculate goal →
Direct Control/Nav2 → cmd_vel_multiplexer → /cmd_vel →
ugv_bringup → ESP32 → Motors → Robot moves
```

**Critical Points:**
- Detection stability: `frame_count >= stability_frames (3)`
- Goal calculation: Transform camera_link → map/odom
- Navigation decision: Distance > 2.0m → Direct Control, else Nav2
- Command priority: Emergency > Direct Control > Nav2 > Teleop

### Path 2: Movement Verification (Safety Net)

```
Odometry → /odom → Multiple subscribers →
├─> movement_guarantee: Calculate distance_moved
│   └─> If stuck → Force emergency cmd_vel (Priority 1)
├─> direct_navigation_fallback: Update robot_pose
│   └─> Check goal progress, adjust velocity
└─> movement_diagnostic: Report status
    └─> Identify issues, predict failures
```

**Critical Points:**
- Physical verification: Actual distance moved from odom
- Stuck detection: `distance_moved < 0.02m` over `2.0s`
- Emergency override: Force `0.5 m/s` if stuck detected
- Diagnostic reporting: Issue detection within `5 seconds`

### Path 3: Command Execution (Hardware)

```
cmd_vel (Twist) → ugv_bringup::cmd_vel_callback() →
Extract linear.x, angular.z → Differential drive kinematics →
Scale to PWM (-0.5 to +0.5) → BaseController::send_command() →
command_queue → process_commands thread → Serial write →
ESP32 receives → Parse JSON → Convert to PWM → Drive motors →
Physical movement
```

**Critical Points:**
- Kinematics: `WHEELBASE_M = 0.175m` (must match base_node.cpp)
- Scaling: `MAX_ROBOT_SPEED_MPS = 1.3 m/s`, `MAX_PWM_COMMAND = 0.5`
- Serial: `/dev/ttyTHS1` (Jetson), `115200` baud, immediate flush
- Command format: `{"T": 1, "L": left, "R": right}\n`

---

## CONCLUSION

This document provides a **complete visualization** of the autonomous tyre inspection robot's system architecture, showing:

- **All communication paths:** Topics, services, actions, TF transforms
- **Complete file dependencies:** Every module and its relationships
- **Full state machine flow:** All mission states and transitions
- **Critical paths:** Detection → Navigation, Movement Verification, Command Execution

**Key Takeaways:**

1. **Multi-layer redundancy:** Multiple independent systems ensure continuous movement
2. **Priority-based arbitration:** Highest-priority command always wins
3. **Physical verification:** Actual movement validated via odometry
4. **Complete traceability:** Every communication path documented

**This network diagram enables:**
- Debugging: Trace any issue to its source
- Optimization: Identify bottlenecks and improve paths
- Understanding: Complete system comprehension
- Maintenance: Clear documentation for future development

**The system is now fully documented and ready for implementation.**
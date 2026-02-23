# Repository Inventory — UGV Workspace & Aurora Integration

Generated: 2026-02-23. Project root: `~/ugv_ws`. Aurora SDK demo: `~/ugv_ws/aurora_remote_sdk_demo`.

---

## 1. ROS2 packages (colcon list)

| Package | Path | Type |
|---------|------|------|
| Aurora_Remote_SDK_Demo | aurora_remote_sdk_demo | cmake |
| aurora_interface | src/aurora_interface | ros.ament_cmake |
| aurora_sdk_bridge | src/aurora_sdk_bridge | ros.ament_cmake |
| gb_visual_detection_3d_msgs | src/Tyre_Inspection_Bot/.../gb_visual_detection_3d_msgs | ros.ament_cmake |
| inspection_manager | src/Tyre_Inspection_Bot/.../inspection_manager | ros.ament_python |
| segmentation_3d | src/Tyre_Inspection_Bot/.../segment_3d/segmentation_3d | ros.ament_cmake |
| segmentation_msgs | src/Tyre_Inspection_Bot/.../segment_3d/segmentation_msgs | ros.ament_cmake |
| slamware_ros_sdk | src/aurora_ros2_sdk_linux/src/slamware_ros_sdk | ros.ament_cmake |
| ugv_base_driver | src/Tyre_Inspection_Bot/.../ugv_base_driver | ros.ament_python |
| ugv_description | src/Tyre_Inspection_Bot/.../ugv_description | ros.ament_cmake |
| ugv_nav | src/Tyre_Inspection_Bot/.../ugv_nav | ros.ament_cmake |
| ugv_vision | src/Tyre_Inspection_Bot/.../ugv_vision | ros.ament_python |

---

## 2. Node files (executables / ROS nodes)

- **slamware_ros_sdk**: `slamware_ros_sdk_server_node` (C++, slamware_ros_sdk_server.h)
- **aurora_sdk_bridge**: `aurora_sdk_bridge_node` (Python, aurora_sdk_bridge_node.py) — depth pipeline, camera_info, points
- **segmentation_3d**: `segmentation_processor_node` (C++, segmentation_processor), `ultralytics_node`, `aurora_camera_info_node`, `depth_to_registered_pointcloud_node`
- **inspection_manager**: `inspection_manager_node.py`
- **ugv_nav**: `depth_gate_node`, `nav_lifecycle_startup`
- **ugv_base_driver**: `motor_driver_node`
- **ugv_vision**: color_track, apriltag_ctrl, apriltag, apriltag_track_*, hsv, gesture, gesture_ctrl
- **aurora_interface**: `aurora_health_monitor.py`

Standalone scripts (rclpy.init): aurora_stereo_calibration.py, aurora_stereo_diagnostics.py, test_depth_pipeline.py, tf_audit.py, check_tf_watchdog.py, aurora_sdk_bridge_node.py, validate_metric_scale.py, validate_rectification.py, aurora_depth_stereo_test.py, aurora_camera_info_node.py, collect_calib_debug_images.py, calibration_diagnose.py, aurora_camera_depth_analysis.py, aurora_pose_publisher.py, aurora_sensor_validate.py, aurora_calibration_preview.py, pointcloud_validate.py, collect_lr_timestamps.py, tf_latency_trace.py, perception_validation.py, drift_characterization.py, validate_stereo_depth.py, depth_to_registered_pointcloud_node.py, photo_capture_service.py.

---

## 3. Launch files

| Launch file | Package | Purpose |
|-------------|---------|--------|
| aurora_sdk_bridge.launch.py | aurora_sdk_bridge | Depth pipeline (stereo → depth, camera_info, points) |
| aurora_testing.launch.py | ugv_nav | Aurora SDK + TF + bridge (no Nav2, no motors) |
| aurora_bringup.launch.py | ugv_nav | Aurora SDK + bridge (full bringup) |
| aurora_calibration.launch.py | ugv_nav | Calibration workflow |
| nav_aurora.launch.py | ugv_nav | Nav2 + Aurora config |
| segment_3d.launch.py | segmentation_3d | YOLO + 3D bbox + depth → registered pointcloud |
| inspection_manager.launch.py | inspection_manager | Inspection mission logic |
| aurora_full.launch.py | aurora_interface | Aurora full stack |
| apriltag_track.launch.py, camera.launch.py | ugv_vision | Vision nodes |

---

## 4. Sensor / pipeline topics (canonical)

- **Aurora SDK (slamware_ros_sdk_server_node)**:  
  `/slamware_ros_sdk_server_node/left_image_raw`, `right_image_raw`, `scan`, `odom`, `map` (slamware_map), `imu_raw_data`, `robot_pose`, `depth_image_raw`, `point_cloud`, etc.
- **Aurora SDK Bridge**:  
  `/camera/depth/image`, `/camera/depth/camera_info`, `/camera/depth/points`, `/stereo/navigation_permitted`
- **Segmentation / 3D**:  
  `/segmentation_processor/*`, `/camera/depth/*`, registered pointcloud
- **Navigation**:  
  `/cmd_vel`, `/goal_pose`, costmaps (via Nav2)
- **Detection**:  
  `/ultralytics/*`, `/darknet_ros_3d/*` (if used)

---

## 5. TF frame chain (from aurora_testing.launch.py)

- `slamware_map` → `odom` (static)
- `base_link` → `camera_left` (static, mounting)
- `camera_left` → `camera_right` (baseline)
- `camera_left` → `imu_link` (static)
- `camera_left` → `camera_depth_optical_frame` (identity for depth)

---

## 6. SDK demo (aurora_remote_sdk_demo)

- **demos**: depthcam_view, calibration_exporter, frame_preview, map_render, lidar_*, pose_*, time_sync, etc.
- **depthcam_view**: build from `aurora_remote_sdk_demo/build` with `cmake .. && make depthcam_view`; run with `./depthcam_view "tcp://192.168.11.1:1445"` (or auto-discover). Key: `s` = save PLY.
- **calibration_exporter**: exports device calibration (XML/YAML) for use in ROS.

---

## 7. Calibration & config

- **Bridge**: `aurora_sdk_bridge/config/equidistant_calibration.yaml`, optional `stereo_calibration_verified.yaml`
- **Segment_3d**: `segmentation_3d/config/stereo_calibration_verified.yaml`, `config.yaml`
- **SDK demo**: `aurora_remote_sdk_demo/config/stereo_calibration.yaml`

Target for exported device calib: `segmentation_3d/config/camera_calibration/aurora_device_calib.yaml` (to be created in integration).

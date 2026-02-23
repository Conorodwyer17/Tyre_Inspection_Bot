# Aurora Integration Summary

**Branch:** cursor/aurora-integration  
**Date:** 2026-02-23

This document summarizes changes made to make the Aurora-based perception pipeline reliable and production-ready for the autonomous tire-inspection mission.

---

## 1. Repository and architecture

- **REPO_INVENTORY.md** added: packages, node files, launch files, sensor topics, TF chain, SDK demo layout, calibration paths.
- **SDK demo:** `aurora_remote_sdk_demo` — depthcam_view and calibration_exporter built from `build/`; connection string `tcp://192.168.11.1:1445`.

---

## 2. SDK demo and calibration

- **depthcam_view:** Built; run with device IP to validate depth and export PLY (`s` key). See backups/.../depth_demo_report.md.
- **calibration_exporter:** Run `./calibration_exporter -o <dir> -f yml "tcp://192.168.11.1:1445"`. Device was unreachable during integration; when it returns NOT_READY or connection fails, use existing YAML calibration.
- **aurora_device_calib.yaml:** Added under `segmentation_3d/config/camera_calibration/` (factory-style, baseline 0.06 m). Replace with exporter output when available.

---

## 3. Calibration and camera_info in the pipeline

- **Bridge** loads stereo calibration from YAML (equidistant or stereo_calibration_verified) and publishes `/camera/depth/camera_info`, `/camera/depth/image`, `/camera/depth/points`.
- **segment_3d.launch.py:** Depth consumers (depth_to_registered_pointcloud, segmentation_processor) start after a **3 s delay** (TimerAction) so the bridge can publish camera_info first. See docs/camera_info_integration.md.

---

## 4. TF and IMU

- **TF chain** (aurora_testing.launch.py): slamware_map → odom → base_link → camera_left → {camera_right, camera_depth_optical_frame, imu_link}. No launch changes.
- **Parameters:** segmentation_3d config: tf_max_age_ms 250; bridge: max_stereo_sync_delta_ms 80. See docs/tf_changes.md.
- **tf_audit.py:** Run with Aurora launch to verify transforms and baseline.

---

## 5. Stereo sync and disparity

- **collect_image_sync_stats.py:** New script: `--left-topic`, `--right-topic`, `--samples 100`; outputs mean/std/min/max of L/R timestamp deltas. If mean > 25 ms, consider increasing max_stereo_sync_delta_ms.
- **Bridge disparity:** num_disparities 64, block_size 5. Tuning notes in docs/stereo_tuning_report.md.
- **Depth accuracy:** Validate with `ros2 run segmentation_3d test_depth_pipeline --expected-distance 1.0` (and 1.5) when device and flat wall available.

---

## 6. Point cloud and costmap

- **/camera/depth/points:** Unorganized (bridge); depth_to_registered_pointcloud produces **organized** cloud for segmentation_processor. Frame: camera_depth_optical_frame.
- **Sanity:** Z span ≤ 20 m, NaN% < 20%; test_depth_pipeline with --check-points. See docs/pointcloud_validation.md.
- **Costmap:** Ensure nav_aurora (or equivalent) includes camera/depth/points; confirm in RViz with LIDAR.

---

## 7. SDK PLY vs ROS PointCloud2

- Same optical frame (Z forward, X right, Y down). When device is available, export PLY from depthcam_view and from ROS at same pose; compare with ICP or cloud stats. See docs/pc_comparison_report.md.

---

## 8. Mission logic (inspection_manager and segmentation)

- **Audit:** inspection_manager uses slamware_map, approach_offset 0.5 m, tire_offset 0.4 m; goals built in map frame and transformed to map for Nav2. segmentation_processor expects organized cloud from depth_to_registered_pointcloud; bounds checks present. No code changes. See docs/inspection_manager_fix_log.md.

---

## 9. Repo cleanup

- **cleanup_candidates.txt:** docs/test.txt, frames_*.gv (generated).
- **Backup:** backups/2026-02-23T13:23:57/cleanup_archive/.
- **Deleted:** those files only. See docs/cleanup_manifest.md.

---

## 10. Parameters reference

| Component | Parameter | Value |
|-----------|-----------|--------|
| segmentation_3d | tf_max_age_ms | 250 |
| segmentation_3d | approach_offset / tire_offset | 0.5 / 0.4 |
| aurora_sdk_bridge | max_stereo_sync_delta_ms | 80 |
| aurora_sdk_bridge | num_disparities, block_size | 64, 5 |
| depth_gate | stale_timeout_s | 0.3 |

---

## 11. How to reproduce

1. **Backup/preflight:** See backups/2026-02-23T13:23:57/preflight.txt and git-status.txt.
2. **Build:** `colcon build --packages-select segmentation_3d aurora_sdk_bridge` (and dependencies).
3. **Launch Aurora + bridge:** `ros2 launch ugv_nav aurora_testing.launch.py`.
4. **Verify:** `ros2 topic echo /camera/depth/camera_info --once`, `python3 scripts/tf_audit.py`, `python3 scripts/collect_image_sync_stats.py --samples 100`.
5. **Launch segment_3d:** `ros2 launch segmentation_3d segment_3d.launch.py`.
6. **Depth test:** `ros2 run segmentation_3d test_depth_pipeline --expected-distance 1.0 --timeout 60`.
7. **Dry run:** inspection_manager with dry_run:=true.

---

## 12. Files added or modified

- **Added:** docs/REPO_INVENTORY.md, docs/AURORA_INTEGRATION_SUMMARY.md, docs/camera_info_integration.md, docs/tf_changes.md, docs/stereo_tuning_report.md, docs/pointcloud_validation.md, docs/pc_comparison_report.md, docs/cleanup_manifest.md, docs/final_acceptance_report.md, docs/inspection_manager_fix_log.md, calib_export_report.md, segmentation_3d/config/camera_calibration/aurora_device_calib.yaml, scripts/collect_image_sync_stats.py, cleanup_candidates.txt.
- **Modified:** segment_3d.launch.py (TimerAction delay for depth pipeline).
- **Backup reports:** backups/2026-02-23T13:23:57/depth_demo_report.md, preflight.txt, cleanup_archive/.

---

## 13. Blockers and next steps

- **Device unreachable:** Calibration export and depthcam_view validation could not be run. When Aurora is on and reachable: run calibration_exporter, run depthcam_view and export PLY, run test_depth_pipeline at 1.0 m and 1.5 m, then update calib_export_report and final_acceptance_report.
- **RTPS/SHM:** If port errors appear, try `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and document in env_changes.log.

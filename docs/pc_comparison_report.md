# SDK Demo PLY vs ROS PointCloud2 Comparison (Step 8)

## Method

1. **SDK:** Run `depthcam_view`, press `s` to export PLY at a test pose; save as `demo_pointcloud.ply`.
2. **ROS:** At the same pose, save `/camera/depth/points` to PLY (e.g. with `ros2 run pcl_ros pointcloud_to_pcd` or a small Python script using sensor_msgs.PointCloud2 and write_ply).
3. **Compare:** ICP or cloud-to-cloud distance stats to confirm geometry matches within tolerance (e.g. mean error < 5 cm at 1 m range).

## Coordinate frames

- **SDK PLY:** Camera coordinate system (Z forward, X right, Y down) per depthcam_view README.
- **ROS:** `camera_depth_optical_frame` (ROS optical: X right, Y down, Z forward) — same convention.
- If mismatch appears, verify timestamp alignment and that both clouds are from the same physical frame (no extra transform).

## Status

- **Blocked on device:** Comparison requires both depthcam_view and ROS pipeline to be run against the same Aurora device at the same pose. When device is available, run both, save PLY from each, and run comparison script; document results here or in `logs/aurora_integration/<ts>/`.

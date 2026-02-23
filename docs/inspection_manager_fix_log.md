# Mission Logic Audit & Node Fixes (Step 9)

## inspection_manager_node.py

- **Frame:** Uses `world_frame` (default `slamware_map`), matching segmentation_processor's `working_frame` and bounding box frame. No change needed.
- **Vehicle positions:** Saved in map frame via `_save_vehicle_position(box)`; box center is in slamware_map. Goals are built in slamware_map and transformed to `map` for Nav2 when possible.
- **Goal offset:** `approach_offset` = 0.5 m (vehicles), `tire_offset` = 0.4 m (tires). Both within the 0.5–1.0 m range for stopping before the tire. No change.
- **Dry run:** `dry_run` parameter validates goals without sending to Nav2; use for safe testing with motors disabled.

## segmentation_processor.cpp

- **Point cloud:** Consumes `/segmentation_processor/registered_pointcloud` from depth_to_registered_pointcloud_node, which publishes **organized** (height × width) cloud. Indexing `(y * width) + x` is correct. No change.
- **Bounds checks:** Out-of-bounds checks are present (`x >= width || y >= height`); invalid indices are skipped with RCLCPP_WARN. No fix required.
- **TF:** Uses `working_frame` (slamware_map) and `tf_max_age_ms` (250) for transform age; already configured.

## depth_to_registered_pointcloud_node.py

- **Input:** Subscribes to `/camera/depth/image` and `/camera/depth/camera_info` (from bridge). Produces organized PointCloud2 for segmentation_processor. No change.

## Logging

- **inspection_manager / segment_3d:** Use `get_logger().warn()` and `get_logger().info()`; no deprecated `warn_throttle` found.
- **motor_driver_node:** Uses `get_logger().warn_once()` (rclpy supports this in Humble). No change.

## State machine

- Inspection manager uses mission states and goal callbacks; failure states (e.g. goal rejected) trigger error logging and state transitions (e.g. NEXT_VEHICLE). No infinite spin on failure identified.

## Summary

No code patches were required. Pipeline is consistent: map-frame coordinates, appropriate offsets, organized point cloud from depth_to_registered_pointcloud, and bounds checks in segmentation_processor.

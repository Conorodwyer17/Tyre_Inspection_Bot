# Point Cloud Sanity and Costmap Integration (Step 7)

## /camera/depth/points

- **Publisher:** aurora_sdk_bridge_node
- **Frame:** `camera_depth_optical_frame` (identity with camera_left for depth)
- **Layout:** Unorganized (height=1, width=N valid points). Dense organized (height x width) can be added if a consumer requires it.
- **Checks:**
  - **Z span:** Bridge clips depth to 0–10 m; reject if effective Z span > 20 m in sanity script.
  - **NaN:** Bridge uses valid mask (z in 0.1–10 m, finite); NaN count should be low in valid region. Sanity: NaN% < 20%.
  - **Mean depth error:** Within ±5% at 1.0 m and 1.5 m when validated with test_depth_pipeline.

## Sanity script

- **Command:** `ros2 run segmentation_3d test_depth_pipeline --check-points --timeout 30` (or use `scripts/pointcloud_validate.py` if present).
- **Checks:** Z span ≤ 20 m, NaN% < 20%, mean depth error within ±5% at benchmark distances.

## Costmap integration

- **Config:** nav_aurora.yaml (or equivalent) should include `camera/depth/points` in the obstacles source. Launch Nav2 and confirm in RViz that obstacles from the point cloud align with LIDAR where both see the same scene.

## RViz

- Visualize `/camera/depth/points` and costmap layers; record screenshots in `logs/aurora_integration/<ts>/` for acceptance.

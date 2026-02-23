# Stereo Sync and Disparity Tuning (Step 6)

## Sync stats collection

- **Script:** `scripts/collect_image_sync_stats.py`
- **Usage:**  
  `python3 scripts/collect_image_sync_stats.py --left-topic /slamware_ros_sdk_server_node/left_image_raw --right-topic /slamware_ros_sdk_server_node/right_image_raw --samples 100`
- **Output:** mean, std, min, max of left/right timestamp deltas (ms). Optional `-o sync_stats.json`.

If **mean delta > 25 ms**: increase `max_stereo_sync_delta_ms` in the bridge (e.g. 60–100 ms) or investigate Aurora SDK sync settings.

## Bridge disparity parameters (aurora_sdk_bridge_node)

- **min_disparity:** 0  
- **num_disparities:** 64 (16×4)  
- **block_size:** 5  
- **max_stereo_sync_delta_ms:** 80 (default); tune per sync stats.

Tuning: adjust num_disparities (e.g. 80, 96) and block_size for quality vs speed; log disparity validity and depth confidence in the bridge if needed.

## Depth accuracy validation

- **Script:** `ros2 run segmentation_3d test_depth_pipeline --expected-distance 1.0` (and 1.5).
- **Acceptance:** depth error within ±5% at 1.0 m and 1.5 m (flat wall).
- When device is available, run and record results in this report or in `logs/aurora_integration/<ts>/`.

## Parameter set (current)

| Parameter | Value |
|-----------|--------|
| max_stereo_sync_delta_ms (bridge) | 80 |
| num_disparities | 64 |
| block_size | 5 |
| tf_max_age_ms (segmentation_3d) | 250 |

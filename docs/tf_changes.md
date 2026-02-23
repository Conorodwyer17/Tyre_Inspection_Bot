# TF & IMU Stabilization (Step 5)

## Transform chain (aurora_testing.launch.py)

- **slamware_map** → **odom** (static, identity)
- **base_link** → **camera_left** (static; mounting offset)
- **camera_left** → **camera_right** (static; baseline 0.06 m)
- **camera_left** → **imu_link** (static)
- **camera_left** → **camera_depth_optical_frame** (identity for depth)

Full chain: `slamware_map` → `odom` → `base_link` → `camera_left` → {`camera_right`, `camera_depth_optical_frame`, `imu_link`}.

## Parameters (production-style)

| Component | Parameter | Value | Notes |
|-----------|-----------|--------|--------|
| segmentation_3d (config.yaml) | tf_max_age_ms | 250 | Reject frames when transform age > 250 ms |
| depth_gate_node | stale_timeout_s | 0.3 | Halt if no nav_permitted for 300 ms |
| aurora_sdk_bridge | max_stereo_sync_delta_ms | 80 (default) | Allow up to 80 ms L/R timestamp delta; can be increased to 60–100 ms if sync stats justify |

No new static_transform_publisher changes were required; existing launch files already define the chain.

## TF audit

- **Script:** `scripts/tf_audit.py` (or `ros2 run <pkg> tf_audit` if installed).
- **Checks:** camera_left→base_link→odom→slamware_map, camera_left→camera_right (baseline ~6 cm), camera_left→camera_depth_optical_frame, camera_left→imu_link, map→slamware_map.
- **Dynamic frame age:** Audit currently flags odom age > 100 ms as STALE. For production with tf_max_age_ms=250, consider running the audit with a 250 ms threshold or accepting "STALE" in the audit when the system is intentionally tuned to 250 ms.

## IMU

- Aurora SDK publishes IMU on `/slamware_ros_sdk_server_node/imu_raw_data`. Frame `imu_link` is published as a static child of `camera_left`. No change to IMU usage in this step.

## Deliverables

- **Launch:** No TF launch changes in this step; aurora_testing.launch.py and aurora_bringup.launch.py already contain the correct static transforms.
- **Config:** segmentation_3d/config/config.yaml has tf_max_age_ms: 250. Bridge max_stereo_sync_delta_ms is set via parameter (default 80).

# Camera Info Integration (Step 4)

## Rationale

- **aurora_sdk_bridge** loads stereo calibration from YAML at startup and publishes `/camera/depth/camera_info` together with `/camera/depth/image` and `/camera/depth/points`. There is no separate `stereo_camera_info_from_yaml` node; the bridge acts as the single source of depth and camera_info.
- **depth_to_registered_pointcloud_node** and **segmentation_processor_node** depend on `/camera/depth/camera_info` and `/camera/depth/image`. If they start before the bridge has published at least one camera_info, they can see empty or stale data on the first frames.

## Ordering

1. **Launch order:** When using `aurora_testing.launch.py` (or `aurora_bringup.launch.py`), the bridge is started with the SDK and TF. When `segment_3d.launch.py` is used in the same session (or in a combined launch), depth consumers are delayed by **3 seconds** so the bridge has time to publish camera_info.
2. **Implementation:** In `segment_3d.launch.py`, `depth_to_pointcloud_node` and `segmentation_processor_node` are wrapped in a `TimerAction(period=3.0, actions=[...])`. Ultralytics and the placeholder `aurora_camera_info_node` (when `use_bridge=false`) still start immediately.

## Calibration path

- Bridge calibration is set in `aurora_sdk_bridge.launch.py`: it prefers `aurora_sdk_bridge/config/stereo_calibration_verified.yaml`, then `segmentation_3d/config/stereo_calibration_verified.yaml`, then `equidistant_calibration.yaml`.
- Device-exported calibration, when available, should be saved to `segmentation_3d/config/camera_calibration/aurora_device_calib.yaml` and the launch or config can be pointed to it.

## Verification

```bash
ros2 launch ugv_nav aurora_testing.launch.py
# In another terminal:
ros2 topic echo /camera/left/camera_info --once   # N/A; we use /camera/depth/camera_info
ros2 topic echo /camera/depth/camera_info --once
ros2 launch segmentation_3d segment_3d.launch.py
# Depth and segmentation nodes start after 3 s.
```

# Calibration Export Report (Step 3)

**Date:** 2026-02-23

## Method

- **Tool:** `aurora_remote_sdk_demo/build/calibration_exporter`
- **Command:** `./calibration_exporter -o <output_dir> -f yml "tcp://192.168.11.1:1445"`
- **Output format:** YAML (OpenCV-compatible)

## Result

- **Device connection:** **Failed** — "Failed to connect to the selected device" when connecting to `192.168.11.1:1445`.
- **Possible causes:** Aurora powered off, Jetson not on same subnet as 192.168.11.1, or firewall/network restriction in the execution environment.

## When device returns NOT_READY (-7)

If `getCameraCalibration()` returns `NOT_READY`:

- Use the **stereo_calibration_from_yaml** approach: keep using the project’s existing YAML calibration (`equidistant_calibration.yaml` or `stereo_calibration_verified.yaml`).
- Flag for later: capture on-device calibration when the device reports ready (e.g. after warm-up or firmware update).

## Deliverables (file layout)

- **Project path for device calib:** `segmentation_3d/config/camera_calibration/aurora_device_calib.yaml`
- **Content:** Created from existing verified/equidistant calibration (same intrinsics and baseline 0.06 m) until the exporter succeeds. When the exporter succeeds, replace with exported left/right and stereo YAML and convert to this ROS-friendly format if needed.

## Validation

- **Intrinsics:** fx, fy, cx, cy and D consistent with 640×480 and baseline 0.06 m.
- **Baseline:** 0.06 m (Aurora spec).

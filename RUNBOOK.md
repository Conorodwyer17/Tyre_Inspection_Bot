# Runbook — Autonomous Tire Inspection Robot (Aurora 2.11)

## Detection Pipeline

- **Vehicles:** Aurora built-in COCO80 semantic segmentation (car, truck, bus, motorcycle, bicycle) via `aurora_semantic_fusion` → `/aurora_semantic/vehicle_bounding_boxes`
- **Tires:** YOLO `best.pt` via `ultralytics_node` + `segmentation_processor` → `/darknet_ros_3d/bounding_boxes`

---

## Pre-Flight Checklist

| Step | Command / Check | Expected |
|------|-----------------|----------|
| 1 | `ping -c 1 192.168.11.1` | Aurora reachable |
| 2 | `ls /dev/ttyTHS1` (or UART) | Port exists |
| 3 | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Set in all terminals |
| 4 | `source ~/ugv_ws/install/setup.bash` | Workspace sourced |
| 5 | `colcon build --packages-select ugv_nav ugv_base_driver segmentation_3d inspection_manager` | Build succeeds |
| 6 | `best.pt` in `~/ugv_ws/` or `~/ugv_ws/src/Tyre_Inspection_Bot/` | Tire model exists |

Or run the script:
```bash
bash scripts/aurora_pre_mission_checklist.sh
```

---

## Launch

### Option A: Mission Launch (recommended)

Runs pre-flight checklist, then starts full stack:

```bash
bash scripts/mission_launch.sh
bash scripts/mission_launch.sh ip_address:=192.168.11.1
```

### Option B: Startup only

```bash
bash scripts/startup.sh
bash scripts/startup.sh use_motor_driver:=false dry_run:=true  # validate without motion
```

### Option C: Manual (separate terminals)

1. Aurora: `ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1`
2. Motor: `ros2 run ugv_base_driver motor_driver_node --ros-args -p uart_port:=/dev/ttyTHS1`
3. Nav2: `ros2 launch ugv_nav nav_aurora.launch.py` (wait ~2 min)
4. Detection: `ros2 launch segmentation_3d segment_3d.launch.py`
5. Inspection: `ros2 launch inspection_manager inspection_manager.launch.py`

**Note:** Inspection manager starts ~2 min after launch (Nav2 lifecycle). PRODUCTION_CONFIG is auto-loaded from workspace.

---

## Map: Build vs Load

The Aurora device holds the map. Use RoboStudio or device UI to:
- **Build:** SLAM → Clear Map
- **Load:** Load a pre-saved map before starting the stack

---

## Verification

| Check | Command |
|-------|---------|
| Aurora odom | `ros2 topic echo /slamware_ros_sdk_server_node/odom --once` |
| Semantic vehicles | `ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once` |
| 3D boxes (tires) | `ros2 topic echo /darknet_ros_3d/bounding_boxes --once` |
| Inspection state | `ros2 topic echo /inspection_state` |
| Mission report | `ros2 topic echo /inspection_manager/mission_report` |
| TF | `ros2 run tf2_ros tf2_echo slamware_map base_link` |

---

## Troubleshooting

| Issue | Check | Fix |
|-------|-------|-----|
| No vehicles detected | semantic_labels topic | Aurora 2.11 firmware; `ros2 topic echo /slamware_ros_sdk_server_node/semantic_labels` |
| No tires detected | best.pt, segmentation_mode | Place best.pt in workspace; mode should switch to "inspection" at vehicle |
| Nav2 lifecycle fails | DDS | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| TF lookup failed | Aurora running | Start Aurora first |

---

## Stop Mission

Press `Ctrl+C` in the terminal running the launch.

---

## Post-Firmware Audit

After upgrading Aurora firmware to 2.11:

```bash
bash scripts/aurora_post_firmware_audit.sh
```

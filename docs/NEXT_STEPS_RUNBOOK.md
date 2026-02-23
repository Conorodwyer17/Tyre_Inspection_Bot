# Aurora Integration — Next Steps Runbook (Priority A–I)

Execute in order. Logs go to `~/ugv_ws/logs/aurora_integration/<ts>/`.

---

## Priority A — Get Aurora reachable

1. **Network check (run once):**
   ```bash
   chmod +x ~/ugv_ws/scripts/aurora_network_check.sh
   ~/ugv_ws/scripts/aurora_network_check.sh eth0
   ```
   Or manually:
   ```bash
   ip addr show
   ip route show
   # If Jetson not on Aurora subnet:
   sudo ip addr add 192.168.11.2/24 dev eth0   # use actual iface from ip addr
   sudo ip link set eth0 up
   ping -c 5 192.168.11.1
   ```
2. **If ping fails:** `sudo ethtool eth0`, `arp -n`, then:
   ```bash
   sudo tcpdump -i eth0 host 192.168.11.1 -c 200 -w /tmp/aurora_net.pcap
   ```
   Copy pcap to `backups/<ts>/`. If still unreachable, see BLOCKER_REPORT.md template below.
3. **Proceed only when** `ping 192.168.11.1` succeeds.

**Note:** Your launch already showed "Connected to the selected device" — so Aurora is reachable from the ROS SDK. The warning "Depth camera not supported" refers to the device’s built-in depth API; **aurora_sdk_bridge** computes depth from left/right images, which the SDK still provides.

---

## Priority B — SDK demo (confirm hardware)

1. **depthcam_view** (binary is in repo **root** build, not demo subdir):
   ```bash
   cd ~/ugv_ws/aurora_remote_sdk_demo/build
   ./depthcam_view tcp://192.168.11.1:1445
   ```
   Validate depth visually; press **`s`** to save PLY → copy to `~/ugv_ws/backups/<ts>/demo_pointcloud.ply`.
2. **Calibration exporter:**
   ```bash
   cd ~/ugv_ws/aurora_remote_sdk_demo/build
   ./calibration_exporter -o ~/ugv_ws/backups/<ts>/aurora_calib -f yml tcp://192.168.11.1:1445
   ```
   If `NOT_READY`, keep using existing YAML; log and proceed.

   **Standalone demo connection:** Even when `ping 192.168.11.1` succeeds, `depthcam_view` and `calibration_exporter` may report "Failed to connect" (observed with 1445 and 7447). The ROS SDK connects when `aurora_testing.launch.py` runs. If demos fail, use the ROS pipeline for depth/point cloud and existing YAML calibration; try exporter again with the ROS stack stopped in case the device allows only one connection.

---

## Priority C — Factory calibration in ROS

1. If exporter produced YML, convert/copy to:
   `~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d/config/camera_calibration/aurora_device_calib.yaml`
2. **Validate topics** (with `aurora_testing.launch.py` running):
   ```bash
   ros2 topic echo /camera/depth/camera_info --once
   ```
   (We use `/camera/depth/camera_info` from the bridge; no separate left/right camera_info in this pipeline.)
3. Bridge and segment_3d launch already ensure camera_info is available before depth consumers (TimerAction delay). No extra guard node needed.
4. **Commit in Tyre_Inspection_Bot subrepo:**
   ```bash
   cd ~/ugv_ws/src/Tyre_Inspection_Bot
   git add src/amr_hardware/src/segment_3d/segmentation_3d/config/camera_calibration/aurora_device_calib.yaml
   git commit -m "Add Aurora factory calibration YAML"
   git push origin HEAD || true
   ```

---

## Priority D — Stereo pipeline and baseline metrics

1. **Launch** (terminal 1):
   ```bash
   cd ~/ugv_ws && source install/setup.bash
   ros2 launch ugv_nav aurora_testing.launch.py
   ```
2. **Check** (terminal 2):
   ```bash
   ros2 node list
   ros2 topic list | grep -E "camera|depth|point|slamware|left|right"
   ```
3. **Sync stats** (script is in workspace `scripts/`, not package):
   ```bash
   cd ~/ugv_ws && source install/setup.bash
   python3 scripts/collect_image_sync_stats.py --left-topic /slamware_ros_sdk_server_node/left_image_raw --right-topic /slamware_ros_sdk_server_node/right_image_raw --samples 100 -o logs/aurora_integration/$(date +%Y%m%dT%H%M%S)/sync_stats.json
   ```
4. **Depth pipeline test:**
   ```bash
   ros2 run segmentation_3d test_depth_pipeline --timeout 60 --expected-distance 1.0
   ```
   Save stdout/stderr to `logs/aurora_integration/<ts>/`.
5. **TF audit:**
   ```bash
   python3 ~/ugv_ws/scripts/tf_audit.py
   ```
   (There is no `stereo_pipeline_graph_discovery` in this repo; use tf_audit + test_depth_pipeline.)

---

## Priority E — Fix pipeline issues

- **TF_STALE:** Run `ros2 run tf2_tools view_frames`; check static transforms (camera_left, camera_right, base_link). Increase `tf_max_age_ms` to 250 if needed (documented).
- **Compute time / DEGRADED:** Reduce StereoSGBM params or resolution; check `tegrastats` / CPU.
- **Low confidence / Z span:** Tune `numDisparities`, `blockSize`, `max_stereo_sync_delta_ms` in bridge.
- **Point cloud frame:** Ensure frame_id is `camera_depth_optical_frame` and TF camera_left → slamware_map at image time.

Re-run `test_depth_pipeline` and `tf_audit` until depth error ≤5% and TF passes.

---

## Priority F — SDK PLY vs ROS point cloud

1. Export PLY from depthcam_view at test pose (Priority B).
2. **Capture ROS cloud to PLY:**
   ```bash
   python3 ~/ugv_ws/scripts/ros_pc_to_ply.py --topic /camera/depth/points --out ~/ugv_ws/backups/<ts>/ros_pointcloud.ply
   ```
3. Compare with ICP or cloud-to-cloud script; document in `pc_comparison_report.md`.

---

## Priority G — Inspection manager and dry run

- Segmentation already uses **registered organized pointcloud** from `depth_to_registered_pointcloud_node` (see inspection_manager_fix_log.md).
- Inspection manager uses map frame and 0.5 m / 0.4 m offsets; dry run: launch with `dry_run:=true`.
- **Dry-run script** (optional): `scripts/dry_run_mission.sh` can launch mission with a mock cmd_vel logger instead of motor driver (create if needed).

---

## Priority H — Commit and push

1. **Tyre_Inspection_Bot:**
   ```bash
   cd ~/ugv_ws/src/Tyre_Inspection_Bot
   git add .
   git status   # review
   git commit -m "Cursor: integrate aurora calibration, launch ordering, stereo diagnostics"
   git push origin HEAD || true
   ```
2. **Main repo:**
   ```bash
   cd ~/ugv_ws
   git checkout cursor/aurora-integration
   git add docs scripts
   git commit -m "Cursor: aurora integration runbook, network check, ros_pc_to_ply"
   git push origin cursor/aurora-integration || true
   ```

---

## Priority I — Cleanups

- Re-check `cleanup_candidates.txt` with `rg <path>`; back up then delete only confirmed unused files. Update `cleanup_manifest.md`.

---

## If hardware is blocked — BLOCKER_REPORT.md

Create `~/ugv_ws/docs/BLOCKER_REPORT.md` and include:

```bash
TS=$(date +%Y%m%dT%H%M%S)
mkdir -p ~/ugv_ws/logs/aurora_integration/$TS
ip addr show > ~/ugv_ws/logs/aurora_integration/$TS/ip_addr.txt
ip route show >> ~/ugv_ws/logs/aurora_integration/$TS/ip_addr.txt
dmesg | tail -n 200 > ~/ugv_ws/logs/aurora_integration/$TS/dmesg_tail.txt
# If slamware runs as service:
sudo journalctl -u slamware_ros_sdk_server_node -n 500 > ~/ugv_ws/logs/aurora_integration/$TS/slamware_journal.txt || true
ls -la /dev | sed -n '1,200p' > ~/ugv_ws/logs/aurora_integration/$TS/dev_list.txt
```

In BLOCKER_REPORT.md: describe symptom, attach paths to the logs above, suggest remediation (power, firmware, cable, subnet, NetworkManager disconnect).

---

## Acceptance checklist (final)

- [ ] `ping 192.168.11.1` succeeds; depthcam_view produces valid depth PLY.
- [ ] `ros2 topic echo /camera/depth/camera_info --once` and `/camera/depth/points` publish.
- [ ] `ros2 run segmentation_3d test_depth_pipeline --expected-distance 1.0` exits 0, mean error ≤5%.
- [ ] `python3 scripts/tf_audit.py` passes (no stale above threshold).
- [ ] Inspection manager dry-run reaches goal pose (motors off or simulated).
- [ ] Commits on `cursor/aurora-integration`; logs under `logs/aurora_integration/<ts>/`.

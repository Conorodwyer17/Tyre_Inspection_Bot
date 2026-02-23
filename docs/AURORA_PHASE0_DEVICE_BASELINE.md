# Phase 0 — Aurora Device & Environment Baseline

Execute these steps to confirm firmware, scene strategy, and map initialization before calibration and mission runs.

## 1. Firmware

1. Power on Aurora (DC 12V 2A or per device label).
2. Connect the Jetson/laptop to Aurora via Ethernet or Wi‑Fi (AP `SLAMWARE-Aurora-xxxxxx`).
3. In a browser, open **http://192.168.11.1**.
4. Sign in: **admin** / **admin111**.
5. Go to **System → Firmware Update**, select the latest Aurora firmware from [SLAMTEC Support](https://www.slamtec.com/en/support), then **Start Firmware Update**.
6. Wait until the upgrade log shows **success**.

## 2. Scene strategy (RoboStudio or Aurora Remote)

- **RoboStudio**: **Debug → Scene Strategy** → choose **Indoor** (default) or **Outdoor** → **Settings** → **Restart Application**.
- **Aurora Remote**: Use the same concept if your build exposes scene/strategy options.
- Use **Indoor** for typical warehouse/garage tire inspection; **Outdoor** for open yards.

## 3. Map initialization

1. Point Aurora at a **feature‑rich** area (2–3 m), e.g. wall with texture, furniture, no large glass or moving people.
2. **Reset/Clear map**:
   - RoboStudio: **SLAM → Clear Map**
   - Aurora Remote: **Remote Commands → Reset Maps** (or **Device Operations → Reset Map**).
3. Keep the device **stationary** until initialization completes (indicator turns green; exclamation mark disappears in RoboStudio).
4. Then start mapping or leave in a stable pose for ROS2.

## 4. Confirm system health

| Check | Expected |
|-------|----------|
| Indicator | Green steady = mapping; yellow = waiting for init |
| No tracking loss | RoboStudio/Remote shows no “Tracking Lost” |
| Connectivity | `ping -c 2 192.168.11.1` succeeds |

## 5. Verify from ROS2 (after launch)

Run the Phase 0 verification script after starting the Aurora ROS2 stack:

```bash
source /home/conor/ugv_ws/install/setup.bash
bash /home/conor/ugv_ws/scripts/aurora_phase0_verify.sh
```

This checks Aurora reachability and that `slamware_ros_sdk_server_node` is publishing pose, odom, and left/right images.

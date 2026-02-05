# System architecture: autonomous tyre inspection robot

How my setup is put together: hardware, software, and how the bits talk to each other. I use a WaveShare six-wheel chassis, an ESP32 for the motors, either a Raspberry Pi 5 or a Jetson Orin running ROS 2 on Ubuntu, and a SLAMTEC Aurora for localisation and mapping. The Coral USB accelerator only comes in when I'm on a Pi; the Jetson does inference on its own GPU.

---

## 1. Hardware

### 1.1 Chassis: WaveShare UGV Rover (six-wheel 4WD)

The base is a WaveShare UGV Rover–style platform: six wheels, 4WD. Specs below are from their product and wiki pages (UGV Rover, UGV Rover Jetson Orin ROS2 kit, UGV-Rover).

**Mechanical**
- Body: 2 mm thick aluminium alloy.
- Drive: four geared motors with encoders, closed-loop speed control.
- Tyres: soft, anti-slip rubber.
- Max speed: 1.3 m/s.
- Power: 3S lithium battery UPS; I can run and charge at the same time.

**Dimensions**
- Exact dimensions and mounting are in the vendor’s dimensional diagrams and STEP models:
  - UGV Rover Jetson Orin ROS2: [Dimensional diagrams (with pan-tilt)](https://files.waveshare.com/wiki/UGV%20Rover%20Jetson%20Orin%20ROS2/UGV_Rover_Jetson_Orin_ROS2_Kit_2D.zip), [STEP model (with pan-tilt)](https://files.waveshare.com/wiki/UGV%20Rover%20Jetson%20Orin%20ROS2/UGV_Rover_PT_Jetson_Orin_ROS2_Kit_STEP.zip).
  - UGV-Rover (Pi 4B/5): [3D drawings and STEP models](https://www.waveshare.com/wiki/UGV-Rover) (PI4B_AI_Kit, PT_AI_Kit).

**Host**
- I run either a Raspberry Pi 4B/5 or an NVIDIA Jetson Orin (e.g. Orin Nano). The stack is aimed at Ubuntu 24.04 and ROS 2 Jazzy on ARM64, though I’ve documented Humble on 22.04 as well in DEPLOYMENT.md.

### 1.2 Chassis controller: ESP32

Dual-controller setup. The ESP32 handles:

- Motor PID for the four driven wheels.
- IMU and other onboard sensors.
- OLED, servo(s), LEDs.
- A JSON protocol over UART to the host.

The Pi or Jetson doesn’t talk to the motors directly; it sends velocity commands to the ESP32. That keeps the host’s I/O and real-time load down. My `ugv_base_driver` node runs on the host, talks to the ESP32 over serial (e.g. `/dev/ttyUSB0`), subscribes to `/cmd_vel` (Twist), and turns those into the JSON the ESP32 expects.

### 1.3 Onboard computer

Either:

- **Raspberry Pi 5** (e.g. 8 GB): Ubuntu 24.04 ARM64. I can plug in a Coral USB accelerator for inference; without it, inference runs on CPU.
- **Jetson Orin** (e.g. Orin Nano): Ubuntu 24.04 (or whatever JetPack supports). No Coral; I use the Jetson GPU for inference.

Same source tree and launch files for both; only Coral and a few paths/backends differ.

### 1.4 SLAM and perception: SLAMTEC Aurora

I use the Aurora (sometimes called AORA in docs) as the single source of map, odometry, LiDAR scan, and optionally camera and IMU. No SLAM or localisation runs on the Pi or Jetson.

**Specs (from SLAMTEC)**  
- Weight: 505 g. Size: 108 × 97 × 88 mm.  
- 2D map resolution: 2 / 5 / 10 cm (selectable). Max mapping area > 1 000 000 m².  
- LiDAR: up to 40 m.  
- Camera: binocular fisheye, 180° FOV, 6 cm baseline, HDR. Typically 15 Hz (10/30 Hz configurable).  
- LiDAR, vision, and IMU fused with hardware time sync. Relocalisation, map save/load, and map continuation supported.

**Connection**  
Aurora talks to the host over Ethernet or Wi-Fi. In AP mode it’s usually at 192.168.11.1. The Aurora ROS 2 SDK node on the host connects to that IP (or to whatever IP the Aurora has on the network).

---

## 2. Software

### 2.1 Workspace and packages

Everything lives in a ROS 2 workspace (e.g. `~/ugv_ws`). This repo gives you the `amr_hardware` bit; I put that under `ugv_ws/src/`. Main packages:

- **ugv_nav**: Aurora bringup and Nav2. Launch files: `aurora_bringup.launch.py`, `nav_aurora.launch.py`. Params in `nav_aurora.yaml` point at Aurora topics and frames.
- **ugv_base_driver**: Subscribes to `/cmd_vel`, talks to the ESP32 over UART/JSON. Can publish wheel odom and joint states if the ESP32 sends them.
- **segment_3d**: (1) `ultralytics_node` (Python) runs YOLO on the Aurora left image and publishes `segmentation_msgs/ObjectsSegment`. (2) The C++ `segmentation_3d` node subscribes to that plus the Aurora point cloud, builds 3D boxes, and publishes `gb_visual_detection_3d_msgs/BoundingBoxes3d` (e.g. on `/darknet_ros_3d/bounding_boxes`).
- **gb_visual_detection_3d_msgs**: Custom messages for 3D boxes (object_name, probability, xmin–zmax).
- **segmentation_msgs**: Custom messages for 2D segments (class_name, probability, mask indices).
- **ugv_vision**: Camera and vision helpers (Aurora RGB or external USB).
- **inspection_manager**: Mission state machine, goals, photo capture. Eats the 3D boxes and drives navigation. Might live in this repo under `amr_simulation` or in another repo; the README describes the launch and topic names.

The Aurora ROS 2 SDK (`slamware_ros_sdk`) is installed separately (official package or fallback). That’s what actually connects to the Aurora and publishes topics and TF.

### 2.2 Topics and data flow

**Aurora** (via `slamware_ros_sdk_server_node`):  
`/slamware_ros_sdk_server_node/scan`, `/odom`, `/map`, `/point_cloud`, left/right/depth images, `imu_raw_data`, plus TF (map, odom, base_link, laser, imu_link, camera frames).

**Nav2** uses Aurora’s map and scan for costmaps and planning, and Aurora’s odom for the controller. It publishes Twist on `/cmd_vel`.

**ugv_base_driver** subscribes to `/cmd_vel` and forwards to the ESP32 over UART.

**segment_3d** turns Aurora image + point cloud into 3D bounding boxes. **inspection_manager** uses those to run the inspection mission.

Rough flow:

```
Aurora (SLAM/odom/map/scan/images/point_cloud)
    → Nav2 → /cmd_vel → ugv_base_driver → ESP32 → motors
    → ultralytics_node → ObjectsSegment
    → segmentation_3d → BoundingBoxes3d
    → inspection_manager (mission, goals, photos)
```

### 2.3 TF and frames

TF comes mainly from the Aurora SDK (and robot_state_publisher if you use a URDF).

- **map**: Global frame; Aurora builds and keeps the map here.
- **odom**: Odometry frame; Aurora publishes pose here (and/or map → odom).
- **base_link** / **base_footprint**: Robot base.
- **laser**, **imu_link**, **camera_left**, **camera_right**: Sensor frames from the Aurora SDK.

In `nav_aurora.yaml` I set `global_frame: map`, `robot_base_frame: base_footprint` (or base_link), and `odom_topic: /slamware_ros_sdk_server_node/odom` so the stack lines up with Aurora. Local costmap uses `odom` for the rolling window; global uses `map`.

### 2.4 Localisation and odometry

I run in localisation-only mode: the map is either built and saved with Aurora or loaded at runtime. No AMCL or particle filter; Aurora gives me 6DOF pose in the map. Odometry is Aurora’s fused output on `/slamware_ros_sdk_server_node/odom`. I could add ESP32 wheel odom via robot_localization if I wanted; by default I rely on Aurora for odom and TF.

### 2.5 Mission flow

The inspection manager runs the show: navigate to waypoints or search patterns (Nav2 + Aurora map/odom), segment vehicles and tyres (pipeline above), use the 3D boxes to send goals (e.g. approach a tyre, hold standoff), trigger photos with metadata, and step through the state machine. Truck/wheel class names, standoff distance, and detection topic are in the README.

---

## 3. Summary

| Part              | Role |
|-------------------|------|
| WaveShare 6-wheel | Chassis; 4WD, encoders, 2 mm aluminium; see vendor docs for dimensions |
| ESP32             | Motors, IMU, peripherals; UART/JSON to host |
| Pi 5 / Jetson Orin| Onboard computer; Ubuntu + ROS 2 (see DEPLOYMENT.md) |
| Coral (optional)  | USB accelerator for inference on Pi only |
| SLAMTEC Aurora    | 6DOF SLAM, map, odom, scan, images, point cloud, IMU |
| ugv_nav           | Aurora bringup, Nav2, params |
| ugv_base_driver   | /cmd_vel → ESP32 UART; optional wheel odom / joint states |
| segment_3d + msgs | YOLO → ObjectsSegment → 3D boxes |
| ugv_vision        | Camera and vision utilities |
| inspection_manager| Mission state machine, goals, photos |
| TF                | map → odom → base_link (+ sensors) from Aurora SDK |

---

## 4. References

- WaveShare UGV Rover (Jetson Orin ROS2): https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2  
- WaveShare UGV-Rover (Pi): https://www.waveshare.com/wiki/UGV-Rover  
- SLAMTEC Aurora: https://www.slamtec.com/en/Aurora, https://www.slamtec.com/en/Aurora/Spec  
- Aurora ROS2 SDK: https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/  
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/  
- Nav2: https://navigation.ros.org/

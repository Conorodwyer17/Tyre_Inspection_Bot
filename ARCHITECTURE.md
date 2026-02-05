# System Architecture: Autonomous Tire Inspection Robot

This document describes the hardware and software architecture of the tire inspection autonomous robot. The system uses a WaveShare six-wheel chassis, an ESP32-based chassis controller, an onboard computer (Raspberry Pi 5 or NVIDIA Jetson Orin) running ROS 2 on Ubuntu, and a SLAMTEC Aurora 6DOF SLAM device for localization and mapping. Optional Coral USB Accelerator is used only when the host is a Raspberry Pi.

---

## 1. Hardware Overview

### 1.1 Chassis: WaveShare UGV Rover (Six-Wheel 4WD)

The mobile base is a WaveShare UGV Rover–type platform with six wheels in a 4WD configuration. The following specifications are taken from WaveShare product and wiki documentation (UGV Rover, UGV Rover Jetson Orin ROS2 kit, and UGV-Rover).

**Mechanical**
- Body: 2 mm thick aluminum alloy casing.
- Drive: Four geared motors with encoders; closed-loop speed control.
- Tires: Soft, anti-slip rubber.
- Maximum speed: 1.3 m/s.
- Power: 3S lithium battery UPS module; supports continuous operation while charging.

**Dimensions and models**
- Exact dimensions and mounting interfaces are given in the vendor dimensional diagrams and STEP models. References:
  - UGV Rover Jetson Orin ROS2: [Dimensional diagrams (with pan-tilt)](https://files.waveshare.com/wiki/UGV%20Rover%20Jetson%20Orin%20ROS2/UGV_Rover_Jetson_Orin_ROS2_Kit_2D.zip), [STEP model (with pan-tilt)](https://files.waveshare.com/wiki/UGV%20Rover%20Jetson%20Orin%20ROS2/UGV_Rover_PT_Jetson_Orin_ROS2_Kit_STEP.zip).
  - UGV-Rover (Pi 4B/5): [3D drawings and STEP models](https://www.waveshare.com/wiki/UGV-Rover) (PI4B_AI_Kit, PT_AI_Kit).

**Host compatibility**
- Supported hosts: Raspberry Pi 4B/5 or NVIDIA Jetson Orin (e.g. Jetson Orin Nano). The software stack targets Ubuntu 24.04 and ROS 2 Jazzy on ARM64 (aarch64).

### 1.2 Chassis Controller: ESP32

The chassis uses a dual-controller layout. The slave controller is an ESP32 that:

- Runs motor PID control for the four driven wheels.
- Reads IMU and other onboard sensors.
- Drives OLED display, servo(s), and LED(s).
- Exposes a high-level interface to the host over UART using a JSON protocol.

The host computer (Pi 5 or Jetson Orin) does not drive motors or encoders directly; it sends velocity or motion commands via the ESP32. This reduces host I/O and real-time requirements. The ROS 2 node `ugv_base_driver` runs on the host, talks to the ESP32 over a serial port (e.g. `/dev/ttyUSB0`), and subscribes to `geometry_msgs/msg/Twist` on `/cmd_vel`, translating those commands into the ESP32 JSON protocol.

### 1.3 Onboard Computer (Brain)

The onboard computer runs the ROS 2 stack and application logic. Two configurations are supported:

- **Raspberry Pi 5** (e.g. 8 GB RAM): Ubuntu 24.04 (64-bit ARM64). Optional Coral USB Accelerator for accelerated neural network inference; without it, inference runs on CPU.
- **NVIDIA Jetson Orin** (e.g. Jetson Orin Nano): Ubuntu 24.04 (or vendor-supported LTS) with JetPack. No Coral device; inference uses the Jetson GPU.

In both cases the platform is “ROS Ubuntu”: a single Ubuntu-based image with ROS 2 Jazzy and the project workspace (`ugv_ws`). The same source tree and launch files are used; only the presence of Coral (and possibly library paths or model backends) differs.

### 1.4 SLAM and Perception: SLAMTEC Aurora

The SLAMTEC Aurora (referred to in some docs as AORA) is an all-in-one 6DOF localization and mapping device. It is used as the sole source of map, odometry, LiDAR scan, and optionally camera and IMU data, so that no SLAM or localization algorithm runs on the Pi or Jetson.

**Physical and performance (from SLAMTEC Aurora spec and product pages)**
- Weight: 505 g.
- Size: 108 mm × 97 mm × 88 mm.
- 2D map resolution: 2 cm / 5 cm / 10 cm (selectable).
- Maximum mapping area: > 1 000 000 m².
- LiDAR: up to 40 m range.
- Camera: binocular fisheye, 180° FOV, 6 cm baseline; HDR supported.
- Camera frame rate: typically 15 Hz (configurable, e.g. 10 Hz or 30 Hz).
- Multi-sensor fusion: LiDAR, vision, and IMU with hardware time synchronization.
- Relocalization, map save/load, and map continuation are supported.

**Connection**
- The Aurora is connected to the host via Ethernet or Wi-Fi. In default AP mode the device often exposes 192.168.11.1; the host runs the Aurora ROS 2 SDK node that connects to this IP (or to the Aurora’s IP when on a shared network).

---

## 2. Software Architecture

### 2.1 ROS 2 Workspace and Packages

The functional software lives in a ROS 2 workspace (e.g. `~/ugv_ws`). The repository provides the `amr_hardware` subtree, which is intended to be placed under `ugv_ws/src/` (or equivalent). Key packages:

- **ugv_nav**: Navigation and Aurora integration. Launch files include Aurora bringup (`aurora_bringup.launch.py`), Nav2-based navigation with Aurora (`nav_aurora.launch.py`), and parameter sets (e.g. `nav_aurora.yaml`) that point to Aurora topics and frames.
- **ugv_base_driver**: ROS 2 driver for the WaveShare chassis; communicates with the ESP32 over UART/JSON and subscribes to `/cmd_vel`. Optionally publishes wheel odometry or joint states if the ESP32 provides them.
- **segment_3d**: Two-part pipeline: (1) `ultralytics_node` (Python) runs YOLO-based 2D semantic segmentation on the Aurora left image and publishes `segmentation_msgs/ObjectsSegment`. (2) `segmentation_3d` (C++) subscribes to `ObjectsSegment` and the Aurora point cloud, computes 3D bounding boxes, and publishes `gb_visual_detection_3d_msgs/BoundingBoxes3d` (e.g. on `/darknet_ros_3d/bounding_boxes`).
- **gb_visual_detection_3d_msgs**: Custom message package defining `BoundingBox3d` (object_name, probability, xmin–zmax) and `BoundingBoxes3d` (array of 3D boxes).
- **segmentation_msgs**: Custom message package defining `ObjectSegment` (class_name, probability, mask indices) and `ObjectsSegment` (header, array of segments).
- **ugv_vision**: Vision utilities and camera handling (e.g. Aurora RGB or external USB camera).
- **inspection_manager**: High-level mission control (state machine, inspection logic, photo capture). Consumes 3D bounding boxes and drives navigation and capture. May be provided in this repository under a separate subtree (e.g. `amr_simulation`) or in another repository; the launch and topic contracts are as described in the README.

The Aurora ROS 2 SDK (e.g. `slamware_ros_sdk`) is installed separately (official SLAMTEC package or fallback SDK). It runs the node that connects to the Aurora device and publishes topics and TF.

### 2.2 Topic and Data Flow

- **Aurora** (via `slamware_ros_sdk_server_node` or equivalent):
  - `/slamware_ros_sdk_server_node/scan` — `sensor_msgs/msg/LaserScan`
  - `/slamware_ros_sdk_server_node/odom` — `nav_msgs/msg/Odometry` (6DOF)
  - `/slamware_ros_sdk_server_node/map` — `nav_msgs/msg/OccupancyGrid`
  - `/slamware_ros_sdk_server_node/point_cloud` — `sensor_msgs/msg/PointCloud2`
  - `/slamware_ros_sdk_server_node/left_image_raw`, `right_image_raw`, `depth_image_raw` — images
  - `/slamware_ros_sdk_server_node/imu_raw_data` — `sensor_msgs/msg/Imu`
  - TF: map, odom, base_link, laser, imu_link, camera frames (names may be parameterized).

- **Nav2** uses Aurora’s map and scan for costmaps and planning, and Aurora’s odometry for the controller. It publishes `geometry_msgs/msg/Twist` on `/cmd_vel`.

- **ugv_base_driver** subscribes to `/cmd_vel` and sends the corresponding commands to the ESP32 over UART.

- **segment_3d** subscribes to an image topic (e.g. Aurora left image) and point cloud, and publishes 3D bounding boxes (e.g. for tires/vehicles). **inspection_manager** consumes these and runs the inspection mission.

High-level flow:

```
Aurora (SLAM/odom/map/scan/images/point_cloud)
    → Nav2 (planning + /cmd_vel) → ugv_base_driver → ESP32 → motors
    → ultralytics_node (image) → ObjectsSegment
    → segmentation_3d (ObjectsSegment + point_cloud) → BoundingBoxes3d
    → inspection_manager (mission state, goals, photo capture)
```

### 2.3 Transform (TF) and Frames

The TF tree is driven primarily by the Aurora SDK node and, if used, robot_state_publisher (URDF).

- **map**: Global fixed frame; Aurora builds and maintains the map in this frame.
- **odom**: Odometry frame; Aurora publishes the pose of the robot in this frame (and/or map → odom).
- **base_link** (or **base_footprint**): Robot base. Aurora typically publishes odom → base_link (and possibly map → base_link or map → odom → base_link depending on SDK configuration).
- **laser**, **imu_link**, **camera_left**, **camera_right**: Sensor frames relative to base_link, as published by the Aurora SDK.

Nav2 parameters in this project (e.g. in `nav_aurora.yaml`) set `global_frame: map`, `robot_base_frame: base_footprint` or `base_link`, and `odom_topic: /slamware_ros_sdk_server_node/odom`, so that the stack uses Aurora’s map, odometry, and base frame consistently. Local costmap often uses `global_frame: odom` for a rolling window; global costmap uses `map`.

### 2.4 Localization and Odometry

- **Localization**: The system operates in localization-only mode relative to a map. The map is either built and saved using Aurora’s capabilities or provided by the Aurora at runtime. No separate AMCL or particle filter is required for basic operation; Aurora provides 6DOF pose in the map.
- **Odometry**: The primary odometry source is Aurora’s fused output on `/slamware_ros_sdk_server_node/odom`. Wheel odometry from the ESP32 may be used in addition (e.g. for redundancy or smoothing) if the driver and robot_localization are configured accordingly; the current default is to rely on Aurora for odometry and TF.

### 2.5 Mission and Inspection Flow

The tire inspection mission is orchestrated by the inspection manager. Conceptually: the robot navigates (using Nav2 and Aurora map/odom) to waypoints or search patterns; the segmentation pipeline detects vehicles and tires in the camera and point cloud and publishes 3D bounding boxes; the inspection manager uses these detections to send goals (e.g. approach a tire, maintain standoff), triggers photo capture with metadata, and advances the state machine (e.g. next vehicle or next tire). Configuration (truck/wheel class names, standoff distance, detection topic) is described in the README.

---

## 3. Summary Table

| Component                  | Role                                                                 |
|----------------------------|----------------------------------------------------------------------|
| WaveShare 6-wheel          | Chassis; 4WD, encoders, 2 mm aluminum; dimensions per vendor docs    |
| ESP32                      | Motor control, IMU, peripherals; UART/JSON to host                   |
| Pi 5 / Jetson Orin         | Onboard computer; Ubuntu + ROS 2 (see DEPLOYMENT.md)                 |
| Coral (optional)           | USB accelerator for inference on Pi only                            |
| SLAMTEC Aurora             | 6DOF SLAM, map, odom, scan, images, point cloud, IMU                  |
| ugv_nav                    | Aurora bringup, Nav2, params for Aurora topics/frames                 |
| ugv_base_driver            | /cmd_vel → ESP32 UART; optional wheel odom / joint states            |
| segment_3d + msgs          | YOLO → ObjectsSegment → 3D boxes (BoundingBoxes3d)                  |
| ugv_vision                 | Camera and vision utilities                                          |
| inspection_manager         | Mission state machine, goals, photo capture                          |
| TF                         | map → odom → base_link (+ sensors) from Aurora SDK                   |

---

## 4. References

- WaveShare UGV Rover (Jetson Orin ROS2): https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2  
- WaveShare UGV-Rover (Pi): https://www.waveshare.com/wiki/UGV-Rover  
- SLAMTEC Aurora product and spec: https://www.slamtec.com/en/Aurora, https://www.slamtec.com/en/Aurora/Spec  
- Aurora ROS2 SDK (developer): https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/  
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/  
- Nav2: https://navigation.ros.org/

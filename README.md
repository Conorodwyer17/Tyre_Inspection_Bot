# Autonomous Tyre Inspection Robot

An autonomous robot for inspecting commercial vehicle tyres using ROS 2, computer vision, and SLAM-based navigation.

## Overview

This project is a fully autonomous tyre inspection system on a Waveshare UGV Rover with an onboard computer running ROS on Ubuntu. The robot:

- Detects vehicles and tyres with YOLO-based semantic segmentation
- Navigates using SLAMTEC Aurora 6DOF SLAM and Nav2
- Builds 3D bounding boxes from 2D segmentation masks and point clouds
- Runs full inspection missions with a state machine
- Captures inspection photos with metadata

## Hardware

- **Platform:** Waveshare UGV Rover with ESP32 motor controller
- **Onboard computer:** Raspberry Pi 5 (8 GB RAM) or NVIDIA Jetson Orin; Ubuntu 22.04/24.04, ROS 2
- **SLAM device:** SLAMTEC Aurora 6DOF LiDAR, vision, and IMU
- **Optional:** Coral USB AI Accelerator (Pi only; Jetson uses onboard GPU)

## Software

- **ROS 2:** Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04). See DEPLOYMENT.md.
- **Python:** 3.10+
- **Architecture:** ARM64 (aarch64) for Pi 5 or Jetson Orin

## Installation

### 1. Install ROS 2

On Ubuntu 22.04: `ros-humble-desktop`. On Ubuntu 24.04: `ros-jazzy-desktop`.

```bash
sudo apt update
sudo apt install -y ros-humble-desktop  # or ros-jazzy-desktop
sudo apt install -y python3-argcomplete python3-colcon-common-extensions
```

### 2. ROS 2 dependencies

```bash
sudo apt install -y ros-humble-nav2-* ros-humble-tf2-* ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

Use `ros-jazzy-*` if on Jazzy.

### 3. SLAMTEC Aurora ROS 2 SDK

Download from the [SLAMTEC developer site](https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/). Extract into the workspace `src/` directory, build with colcon, and add the library path to `~/.bashrc` per the SDK instructions.

### 4. Python dependencies

```bash
cd ~/ugv_ws
pip3 install -r requirements.txt
```

### 5. Build workspace

```bash
cd ~/ugv_ws
colcon build
source install/setup.bash
```

### 6. Tire detection model

Place the tire detection model `best.pt` in one of:

- `~/ugv_ws/best.pt`
- `~/ugv_ws/src/Tyre_Inspection_Bot/best.pt`

Without this file, tire detection falls back to the navigation model, which may not detect tires.

## Quick start

**Option 1: Single script (recommended)**

```bash
cd ~/ugv_ws
source install/setup.bash
bash src/Tyre_Inspection_Bot/scripts/start_full_mission.sh
```

This starts Aurora, motor driver, Nav2, detection pipeline, and inspection manager. Ensure Aurora is reachable at 192.168.11.1 and the motor UART is connected (default `/dev/ttyTHS1`).

**Option 2: Manual launch (separate terminals)**

Terminal 1 – Aurora:
```bash
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1
```

Terminal 2 – Motor driver:
```bash
ros2 run ugv_base_driver motor_driver_node --ros-args -p uart_port:=/dev/ttyTHS1 -p baud_rate:=115200
```

Terminal 3 – Navigation:
```bash
ros2 launch ugv_nav nav_aurora.launch.py
```

Terminal 4 – Detection:
```bash
ros2 launch segmentation_3d segment_3d.launch.py camera_rgb_topic:=/slamware_ros_sdk_server_node/left_image_raw
```

Terminal 5 – Inspection manager:
```bash
ros2 launch inspection_manager inspection_manager.launch.py
```

## Stopping the mission

```bash
bash ~/ugv_ws/src/Tyre_Inspection_Bot/scripts/stop_mission.sh
```

## Configuration

- **Aurora IP:** Set `AURORA_IP` (default 192.168.11.1)
- **Motor UART:** Set `UART_PORT` (default `/dev/ttyTHS1` for Jetson)
- **Segmentation:** `src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml` – `interested_classes`, `minimum_probability`
- **Inspection manager:** Launch args – `vehicle_labels`, `tire_label`, `detection_topic`, `approach_offset`, `tire_offset`

## Monitoring

```bash
ros2 topic echo /inspection_state
ros2 topic echo /darknet_ros_3d/bounding_boxes
```

Photos are saved to `~/ugv_ws/tire_inspection_photos/`.

## Project structure

```
Tyre_Inspection_Bot/
  src/amr_hardware/src/
    ugv_nav/           # Aurora bringup, Nav2
    ugv_base_driver/   # Motor driver (cmd_vel to ESP32)
    segment_3d/        # YOLO + 3D bounding boxes
    inspection_manager/# Mission state machine, photo capture
    gb_visual_detection_3d_msgs/
    segmentation_msgs/
    ugv_description/
    ugv_vision/
  scripts/
    start_full_mission.sh
    stop_mission.sh
    monitor_mission.sh
```

## Documentation

- **ARCHITECTURE.md** – Hardware and software layout, data flow
- **DEPLOYMENT.md** – ROS 2 version, Ubuntu, native vs Docker

## Licence

See individual package licences in each `package.xml`.

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
- **Onboard computer:** Raspberry Pi 5 (8 GB RAM) or NVIDIA Jetson Orin; Ubuntu 24.04, ROS 2
- **SLAM device:** SLAMTEC Aurora 6DOF LiDAR + vision + IMU
- **Optional:** Coral USB AI Accelerator (Pi only; Jetson has its own GPU)

## Software

- **ROS 2:** Jazzy Jalisco (native on Ubuntu 24.04). See DEPLOYMENT.md for Humble on 22.04.
- **Ubuntu:** 24.04 (64-bit ARM64)
- **Python:** 3.10+
- **Architecture:** ARM64 (aarch64) for Pi 5 or Jetson Orin

## Installation

### 1. System setup

Follow the first-time setup guide:

```bash
# Full instructions:
src/amr_hardware/Robot_first_time_Installation.md
```

### 2. Install ROS 2 Jazzy

```bash
# On Pi 5 or Jetson Orin with Ubuntu 24.04
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS 2 Jazzy repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y python3-argcomplete python3-colcon-common-extensions
```

### 3. ROS 2 dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-nav2-* \
  ros-jazzy-robot-localization \
  ros-jazzy-joint-state-publisher-* \
  ros-jazzy-rosbridge-* \
  ros-jazzy-rqt-* \
  ros-jazzy-tf2-* \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs
```

Don’t install: `ros-jazzy-depthai-*` (OAK-D, obsolete), `ros-jazzy-cartographer-*`, `ros-jazzy-rtabmap-*` (Aurora does SLAM), or `ros-jazzy-depth-image-proc` (Aurora gives point clouds).

### 4. SLAMTEC Aurora ROS 2 SDK

**Official SDK (recommended):**

1. Download from: https://download-en.slamtec.com/api/download/slamware-Aurora-ros2sdk/1.5?lang=en  
2. Extract into the workspace:
   ```bash
   cd ~/ugv_ws/src
   # Extract aurora_ros2_sdk_linux (or whatever the tarball contains) here
   ```
3. Build:
   ```bash
   cd ~/ugv_ws
   colcon build --packages-select slamware_ros_sdk
   source install/setup.bash
   ```
4. Add the library path to `~/.bashrc`. For ARM64 (Pi 5 or Jetson Orin):
   ```bash
   export LD_LIBRARY_PATH=~/ugv_ws/src/aurora_ros2_sdk_linux/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
   source ~/.bashrc
   ```
   For x86_64 use `linux_x86_64` in the path.

Package name is `slamware_ros_sdk`. Launch files in this repo are already set up. If the official SDK isn’t available, you can fall back to the Python/C++ SDKs (see SLAMTEC docs).

### 5. Coral USB Accelerator (optional, Pi only)

On a Pi (not a Jetson), you can add a Coral for faster inference:

```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt update && sudo apt install -y libedgetpu1-std
pip3 install pycoral
```

See [Coral USB Accelerator docs](https://coral.ai/docs/accelerator/get-started). On a Jetson you use the onboard GPU instead.

### 6. Python dependencies

```bash
cd ~/ugv_ws
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

### 7. Build workspace

```bash
cd ~/ugv_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## Quick start

Run the full system in several terminals.

**Terminal 1 – Aurora**  
Connect to Aurora Wi-Fi if it’s in AP mode (default IP 192.168.11.1):

```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1
```

Wait for `/slamware_ros_sdk_server_node/scan`, `/odom`, `/map`.

**Terminal 2 – ESP32 motor driver**

```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch ugv_base_driver esp32_driver.launch.py uart_port:=/dev/ttyUSB0
```

**Terminal 3 – Navigation**

```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch ugv_nav nav_aurora.launch.py use_rviz:=false
```

Wait for the nav stack to come up with Aurora data.

**Terminal 4 – 3D segmentation** (Aurora provides the stereo images)

```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch segmentation_3d segment_3d.launch.py \
  camera_rgb_topic:=/slamware_ros_sdk_server_node/left_image_raw
```

Wait for `/ultralytics/segmentation/objects_segment` and `/darknet_ros_3d/bounding_boxes`.

**Terminal 5 – Inspection manager**

```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch inspection_manager inspection_manager.launch.py \
  detection_topic:=/darknet_ros_3d/bounding_boxes \
  truck_label:=car \
  wheel_label:=tire
```

Monitor:

```bash
ros2 topic echo /inspection_state
ros2 topic echo /segmentation_mode
```

## Configuration

**Vehicle positions:** edit `src/amr_hardware/src/ugv_nav/maps/...` (e.g. trucks.yaml).

**Segmentation:** `src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml`  
- `interested_classes`, `minimum_probability`, `standoff_distance`

**Inspection manager:** launch args – `truck_label`, `wheel_label`, `detection_topic`, `standoff_distance`, `approach_offset`.

**ESP32 driver:** `ugv_base_driver/launch/esp32_driver.launch.py` – `uart_port`, `baud_rate`, `publish_odom`, `publish_joint_states`.

## System architecture (short)

1. **SLAMTEC Aurora** – LiDAR, 6DOF odom, IMU, map, point clouds. TF: `/map` → `/odom` → `/base_link`. No on-board SLAM.
2. **ugv_nav** – Nav2, Aurora bringup; subscribes to Aurora scan, odom, map.
3. **ugv_base_driver** – UART/JSON to the Waveshare base; subscribes to `/cmd_vel`.
4. **ugv_vision** – Aurora or USB camera, point cloud handling.
5. **segment_3d** – YOLO segmentation + point cloud → 3D bounding boxes.
6. **inspection_manager** – State machine, navigation goals, photo capture.

Data flow: Aurora → Nav2 → `/cmd_vel` → ESP32 driver → motors; Aurora camera → segmentation → 3D boxes → inspection manager.

See **ARCHITECTURE.md** and **DEPLOYMENT.md** for detail.

## Troubleshooting

**Aurora not seen:** `lsusb | grep -i slamtec`; `ros2 topic list | grep aurora`; check `/odom`, `/scan`.

**ESP32 not responding:** `ls -l /dev/ttyUSB*`; run the motor driver node with the right port; `ros2 topic echo /cmd_vel`.

**Nav issues:** Check `/slamware_ros_sdk_server_node/map`, odom and scan rate; `ros2 run tf2_ros tf2_echo map base_link` and `view_frames`.

**No 3D boxes:** Check image topic, segmentation rate, point cloud topic; tweak `minimum_probability` in config.

**Model not detecting:** Ensure `~/ugv_ws/best.pt` (or `yolov8m-seg.pt`) exists; class names in config; adjust `minimum_probability`.

## Project structure

```
ugv_ws/
├── src/
│   ├── amr_hardware/
│   │   ├── src/
│   │   │   ├── segment_3d/
│   │   │   ├── ugv_nav/
│   │   │   ├── ugv_vision/
│   │   │   └── ugv_base_driver/
│   │   ├── Robot_first_time_Installation.md
│   │   └── ros2_startup.sh
│   └── amr_simulation/
│       └── src/
│           └── inspection_manager/
├── requirements.txt
└── README.md
```

## Development

Build a single package: `colcon build --packages-select ugv_base_driver`  
Verbose: `colcon build --packages-select segmentation_3d --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON`

Tests: `colcon test --packages-select inspection_manager` then `colcon test-result --verbose`.

Coral (Pi only): see section 5 above.

## Licence

See individual package licences in each `package.xml`.

## Contributing

Fork, branch, change, open a pull request.

## Documentation

- **ARCHITECTURE.md** – Hardware and software layout, TF, data flow.
- **DEPLOYMENT.md** – ROS 2 version (Jazzy vs Humble), Ubuntu 22 vs 24, native vs Docker.

## References

- [SLAMTEC Aurora](https://www.slamtec.com/en/aurora)
- [Waveshare UGV Rover](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Nav2](https://navigation.ros.org/)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
- [Coral USB Accelerator](https://coral.ai/docs/accelerator/get-started)

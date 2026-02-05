# Autonomous Tire Inspection Robot

An autonomous robot system for inspecting commercial vehicle tires using ROS 2, computer vision, and SLAM-based navigation.

## Overview

This project implements a fully autonomous tire inspection system on a Waveshare UGV Rover with an onboard computer running ROS Ubuntu. The robot can:
- Detect vehicles and tires using YOLO-based semantic segmentation
- Navigate autonomously using SLAMTEC Aurora 6DOF SLAM and Nav2
- Generate 3D bounding boxes from 2D segmentation masks and point clouds
- Execute complete inspection missions with state machine control
- Capture inspection photographs with metadata

## Hardware Requirements

- **Platform**: Waveshare UGV Rover with ESP32 motor controller
- **Onboard computer**: Raspberry Pi 5 (8GB RAM) or NVIDIA Jetson Orin; Ubuntu 24.04, ROS 2
- **SLAM device**: SLAMTEC Aurora 6DOF LiDAR + Vision + IMU SLAM device
- **Optional**: Coral USB AI Accelerator (for Raspberry Pi; Jetson has integrated GPU)

## Software Requirements

- **ROS 2**: Jazzy Jalisco (native on Ubuntu 24.04)
- **Ubuntu**: 24.04 (64-bit ARM64)
- **Python**: 3.10+
- **Architecture**: ARM64 (aarch64) for Pi 5 or Jetson Orin

## Installation

### 1. System Setup

Follow the first-time installation guide:
```bash
# See detailed instructions in:
src/amr_hardware/Robot_first_time_Installation.md
```

### 2. Install ROS 2 Jazzy

```bash
# On Raspberry Pi 5 or Jetson Orin with Ubuntu 24.04 (ROS Ubuntu)
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS2 Jazzy repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y python3-argcomplete python3-colcon-common-extensions
```

### 3. Install ROS 2 Dependencies

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

**Note:** Do NOT install:
- `ros-jazzy-depthai-*` (OAK-D camera, obsolete)
- `ros-jazzy-cartographer-*` (SLAM, Aurora replaces)
- `ros-jazzy-rtabmap-*` (SLAM, Aurora replaces)
- `ros-jazzy-depth-image-proc` (Aurora provides point clouds natively)

### 4. Install SLAMTEC Aurora ROS2 SDK

**Official ROS2 SDK (recommended):**
1. Download from: https://download-en.slamtec.com/api/download/slamware-Aurora-ros2sdk/1.5?lang=en
2. Extract to workspace:
   ```bash
   cd ~/ugv_ws/src
   # Extract aurora_ros2_sdk_linux (or appropriate version) here
   ```
3. Build SDK:
   ```bash
   cd ~/ugv_ws
   colcon build --packages-select slamware_ros_sdk
   source install/setup.bash
   ```
4. Configure library path (add to `~/.bashrc`). For ARM64 (Raspberry Pi 5 or Jetson Orin):
   ```bash
   export LD_LIBRARY_PATH=~/ugv_ws/src/aurora_ros2_sdk_linux/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
   source ~/.bashrc
   ```
   For x86_64 use `linux_x86_64` in the path instead.

**Note:** Package name is `slamware_ros_sdk`. Launch files are pre-configured. If the official ROS2 SDK is unavailable, Python/C++ SDKs can be used as fallback (see SLAMTEC documentation).

### 5. Install Coral USB Accelerator (Optional, Raspberry Pi only)

When using Raspberry Pi (not Jetson), an optional Coral USB Accelerator can improve inference. Install manually:

```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt update && sudo apt install -y libedgetpu1-std
pip3 install pycoral
```

See [Coral USB Accelerator documentation](https://coral.ai/docs/accelerator/get-started) for details. Jetson does not require Coral; use its integrated GPU for inference.

### 6. Install Python Dependencies

```bash
cd ~/ugv_ws
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

### 7. Build Workspace

```bash
cd ~/ugv_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Launch Complete Inspection System

The system requires multiple terminals:

#### Terminal 1: Aurora SLAM Device
```bash
cd ~/ugv_ws
source install/setup.bash
# Connect to Aurora Wi-Fi if in AP mode (default IP: 192.168.11.1)
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1
```

Wait for: `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/map` topics to be available

#### Terminal 2: ESP32 Motor Driver
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch ugv_base_driver esp32_driver.launch.py uart_port:=/dev/ttyUSB0
```

#### Terminal 3: Navigation Stack
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch ugv_nav nav_aurora.launch.py use_rviz:=false
```

Wait for: Navigation stack to initialize with Aurora data

#### Terminal 4: 3D Segmentation (Aurora provides stereo cameras)
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch segmentation_3d segment_3d.launch.py \
  camera_rgb_topic:=/slamware_ros_sdk_server_node/left_image_raw
```

Wait for: `/ultralytics/segmentation/objects_segment`, `/darknet_ros_3d/bounding_boxes`

#### Terminal 5: Inspection Manager
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch inspection_manager inspection_manager.launch.py \
  detection_topic:=/darknet_ros_3d/bounding_boxes \
  truck_label:=car \
  wheel_label:=tire
```

Monitor mission progress:
```bash
ros2 topic echo /inspection_state
ros2 topic echo /segmentation_mode
```

## Configuration

### Vehicle Configuration

Edit the vehicle positions file:
```bash
# For hardware:
src/amr_hardware/src/ugv_nav/maps/...
```

Example `trucks.yaml`:
```yaml
trucks:
  - x: 2.0
    y: 0.0
    z: 0.0
    yaw: 0.0
  - x: 5.0
    y: 1.0
    z: 0.0
    yaw: 1.57
```

### Segmentation Configuration

Edit segmentation parameters:
```bash
src/amr_hardware/src/segment_3d/segmentation_3d/config/config.yaml
```

Key parameters:
- `interested_classes`: Object classes to detect (e.g., `["person","truck","car","tire"]`)
- `minimum_probability`: Minimum detection confidence (0.0-1.0)
- `standoff_distance`: Distance from vehicle for initial approach (meters)

### Inspection Manager Parameters

Launch parameters:
- `truck_label`: Class name for vehicles (default: "truck")
- `wheel_label`: Class name for tires (default: "wheel")
- `detection_topic`: Topic for 3D bounding boxes (default: `/darknet_ros_3d/bounding_boxes`)
- `standoff_distance`: Standoff distance in meters (default: 2.0)
- `approach_offset`: Approach distance in meters (default: 0.5)

### ESP32 Motor Driver Configuration

Edit launch parameters in `ugv_base_driver/launch/esp32_driver.launch.py`:
- `uart_port`: UART device path (default: `/dev/ttyUSB0`)
- `baud_rate`: Communication baud rate (default: `115200`)
- `publish_odom`: Publish odometry from ESP32 if available (default: `false`)
- `publish_joint_states`: Publish joint states from ESP32 if available (default: `false`)

## System Architecture

### Components

1. **SLAMTEC Aurora 6DOF SLAM Device**
   - Provides: LiDAR scans, 6DOF odometry, IMU data, map, point clouds
   - TF frames: `/map → /odom → /base_link`
   - No on-board SLAM computation required

2. **Navigation Stack** (`ugv_nav`)
   - Nav2 for path planning and execution
   - AMCL or EKF for localization (using Aurora map)
   - Subscribes to Aurora topics: `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/map`

3. **ESP32 Motor Driver** (`ugv_base_driver`)
   - UART/JSON communication with Waveshare UGV base
   - Subscribes to `/cmd_vel` (Twist)
   - Handles motor PID control, IMU readout, actuator state

4. **Vision System** (`ugv_vision`)
   - Aurora RGB camera (if available) or external USB camera
   - Point cloud processing (from Aurora)

5. **3D Segmentation** (`segmentation_3d`)
   - YOLO-based 2D semantic segmentation
   - Point cloud processing
   - 3D bounding box generation

6. **Inspection Manager** (`inspection_manager`)
   - State machine for mission control
   - Detection-based navigation
   - Photo capture and metadata storage

### Data Flow

```
Aurora SLAM → /slamware_ros_sdk_server_node/odom, /scan, /map, /point_cloud, /left_image_raw
   ↓
Nav2 → /cmd_vel → ESP32 Driver → Motors
   ↓
Aurora Camera → 2D Segmentation → 3D Bounding Boxes
   ↓
Inspection Manager → Mission Control
```

## Troubleshooting

### Aurora Device Not Detected

```bash
# Check USB connection
lsusb | grep -i slamtec

# Verify Aurora topics
ros2 topic list | grep aurora
ros2 topic echo /odom
ros2 topic echo /scan
```

### ESP32 Motor Driver Not Responding

```bash
# Check UART connection
ls -l /dev/ttyUSB*

# Test UART communication
ros2 run ugv_base_driver motor_driver_node --ros-args \
  -p uart_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200

# Check cmd_vel topic
ros2 topic echo /cmd_vel
```

### Navigation Issues

```bash
# Check Aurora map is available
ros2 topic echo /slamware_ros_sdk_server_node/map --once

# Verify odometry
ros2 topic hz /slamware_ros_sdk_server_node/odom
ros2 topic hz /slamware_ros_sdk_server_node/scan

# Check TF tree
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros view_frames
```

### No 3D Bounding Boxes

1. Verify Aurora camera topic exists: `ros2 topic list | grep image`
2. Check segmentation is running: `ros2 topic hz /ultralytics/segmentation/objects_segment`
3. Verify point cloud topic: `ros2 topic list | grep point`
4. Adjust `minimum_probability` threshold in config

### Model Not Detecting Objects

1. Verify model file exists: `~/ugv_ws/best.pt` (or fallback to `yolov8m-seg.pt`)
2. Check class names in config match model output
3. Adjust `minimum_probability` threshold in config

## Project Structure

```
ugv_ws/
├── src/
│   ├── amr_hardware/          # Hardware-specific packages
│   │   ├── src/
│   │   │   ├── segment_3d/     # 3D segmentation pipeline
│   │   │   ├── ugv_nav/       # Navigation and localization
│   │   │   ├── ugv_vision/   # Camera drivers (if needed)
│   │   │   └── ugv_base_driver/  # ESP32 motor driver
│   │   ├── Robot_first_time_Installation.md
│   │   └── ros2_startup.sh    # Auto-startup script
│   └── amr_simulation/        # Simulation packages
│       └── src/
│           └── inspection_manager/  # Mission control
├── docs/
│   └── PROJECT_PRESENTATION_DOCUMENT.md
├── requirements.txt
└── README.md
```

## Development

### Building Individual Packages

```bash
# Build specific package
colcon build --packages-select ugv_base_driver

# Build with verbose output
colcon build --packages-select segmentation_3d --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

### Running Tests

```bash
# Run package tests
colcon test --packages-select inspection_manager
colcon test-result --verbose
```

See section 5 for optional Coral USB Accelerator installation (Raspberry Pi only).

## License

See individual package licenses in `package.xml` files.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Documentation

- **ARCHITECTURE.md** – System architecture (hardware, software, TF, data flow).
- **DEPLOYMENT.md** – ROS 2 version choice (Jazzy vs Humble), Ubuntu 22 vs 24, and native vs Docker for onboard deployment.

## References

- [SLAMTEC Aurora Documentation](https://www.slamtec.com/en/aurora)
- [Waveshare UGV Rover Documentation](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
- [Coral USB Accelerator](https://coral.ai/docs/accelerator/get-started)
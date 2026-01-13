# Autonomous Tire Inspection Robot

An autonomous robot system for inspecting commercial vehicle tires using ROS 2, computer vision, and SLAM-based navigation.

## Overview

This project implements a fully autonomous tire inspection system on a Waveshare UGV Rover with NVIDIA Jetson Orin. The robot can:
- Detect vehicles and tires using YOLO-based semantic segmentation
- Navigate autonomously using SLAM and Nav2
- Generate 3D bounding boxes from 2D segmentation masks and point clouds
- Execute complete inspection missions with state machine control
- Capture inspection photographs with metadata

## Hardware Requirements

- **Platform**: Waveshare UGV Rover PT Jetson Orin ROS2 Kit
- **Camera**: OAK-D-Lite depth camera
- **LiDAR**: LD06 2D LiDAR
- **Computer**: NVIDIA Jetson Orin

## Software Requirements

- **ROS 2**: Humble Hawksbill
- **Ubuntu**: 22.04
- **Python**: 3.10+
- **CUDA**: For GPU acceleration (recommended)

## Installation

### 1. System Setup

Follow the first-time installation guide:
```bash
# See detailed instructions in:
src/amr_hardware/Robot_first_time_Installation.md
```

### 2. Install ROS 2 Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-depth-image-proc \
  ros-humble-cartographer-* \
  ros-humble-desktop-* \
  ros-humble-joint-state-publisher-* \
  ros-humble-nav2-* \
  ros-humble-rosbridge-* \
  ros-humble-rqt-* \
  ros-humble-rtabmap-* \
  ros-humble-usb-cam \
  ros-humble-depthai-* \
  ros-humble-gazebo-*
```

### 3. Install Python Dependencies

```bash
cd ~/ugv_ws
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

### 4. Build Workspace

```bash
cd ~/ugv_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Launch Complete Inspection System

The system requires 4 terminals:

#### Terminal 1: Navigation & SLAM
```bash
cd ~/ugv_ws
source install/setup.bash
export UGV_MODEL=ugv_rover
export LDLIDAR_MODEL=ld06
ros2 launch ugv_nav slam_nav.launch.py use_rviz:=false
```

Wait for: `/scan`, `/odom`, `/map` topics to be available

#### Terminal 2: Camera & Point Cloud
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch ugv_vision oak_d_lite.launch.py
```

Then (in same or new terminal):
```bash
ros2 run depth_image_proc point_cloud_xyz_node \
  --ros-args \
  -p use_approximate_sync:=true \
  -p queue_size:=10 \
  -r image_rect:=/oak/stereo/image_raw \
  -r camera_info:=/oak/stereo/camera_info \
  -r points:=/points
```

Wait for: `/oak/rgb/image_rect`, `/points` topics

#### Terminal 3: 3D Segmentation
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch segmentation_3d segment_3d.launch.py
```

Wait for: `/ultralytics/segmentation/objects_segment`, `/darknet_ros_3d/bounding_boxes`

#### Terminal 4: Inspection Manager
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

# For simulation:
src/amr_simulation/src/inspection_manager/config/trucks.yaml
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

## System Architecture

### Components

1. **Navigation Stack** (`ugv_nav`)
   - SLAM Toolbox for mapping
   - Nav2 for path planning and execution
   - LiDAR-based localization

2. **Vision System** (`ugv_vision`)
   - OAK-D-Lite camera driver
   - Depth image processing
   - Point cloud generation

3. **3D Segmentation** (`segmentation_3d`)
   - YOLO-based 2D semantic segmentation
   - Point cloud processing
   - 3D bounding box generation

4. **Inspection Manager** (`inspection_manager`)
   - State machine for mission control
   - Detection-based navigation
   - Photo capture and metadata storage

### Data Flow

```
Camera → 2D Segmentation → 3D Bounding Boxes
   ↓
Point Cloud ──────────────┘
   ↓
Inspection Manager → Nav2 → Robot Movement
```

## Troubleshooting

### Point Cloud Not Publishing

```bash
# Verify point cloud node is running
ros2 topic hz /points

# Check camera topics
ros2 topic hz /oak/stereo/image_raw
ros2 topic hz /oak/stereo/camera_info
```

### No 3D Bounding Boxes

```bash
# Check segmentation is running
ros2 topic hz /ultralytics/segmentation/objects_segment
ros2 topic hz /darknet_ros_3d/bounding_boxes

# Verify QoS compatibility (should both be BEST_EFFORT)
ros2 topic info /darknet_ros_3d/bounding_boxes --verbose
```

### Navigation Issues

```bash
# Check SLAM map is available
ros2 topic echo /map --once

# Verify odometry
ros2 topic hz /odom
ros2 topic hz /scan
```

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
│   │   │   ├── ugv_nav/       # Navigation and SLAM
│   │   │   └── ugv_vision/    # Camera drivers
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
colcon build --packages-select segmentation_3d

# Build with verbose output
colcon build --packages-select inspection_manager --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

### Running Tests

```bash
# Run package tests
colcon test --packages-select inspection_manager
colcon test-result --verbose
```

## License

See individual package licenses in `package.xml` files.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## References

- [Waveshare UGV Rover Documentation](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Ultralytics YOLO](https://docs.ultralytics.com/)

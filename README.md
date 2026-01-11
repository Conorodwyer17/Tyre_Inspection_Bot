# Autonomous Vehicle Tire Inspection System

**Production-Ready Implementation for Waveshare UGV (Jetson Orin)**

## Overview

This workspace implements an autonomous system for detecting vehicles (cars and trucks) and photographing each tire. The system uses:

- **ROS2 Humble** for middleware and communication
- **YOLOv8** for vehicle detection
- **Nav2** for autonomous navigation
- **SLAM Toolbox** for mapping and localization
- **Python 3.10+** for perception and mission management

## Quick Start

### 1. Environment Setup

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Install dependencies
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions
pip3 install -r requirements.txt

# Build workspace
cd /home/jetson/ugv_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. System Architecture

The system consists of several ROS2 packages:

- **ugv_msgs**: Custom message definitions
- **ugv_perception**: Vehicle detection and wheel localization
- **ugv_mission**: Mission management and state machine
- **ugv_inspection**: Image capture and storage
- **ugv_nav**: Navigation stack (existing)
- **ugv_vision**: Vision utilities (existing)
- **ugv_slam**: SLAM tools (existing)

### 3. Running the System

See **CONSTRUCTION_DOCUMENT.md** for detailed setup instructions.

Quick launch:
```bash
ros2 launch ugv_inspection vehicle_inspection_bringup.launch.py
```

## Documentation

- **CONSTRUCTION_DOCUMENT.md**: Complete construction guide with step-by-step instructions
- **docs/**: Additional documentation (to be added)

## Project Structure

```
ugv_ws/
├── src/amr_simulation/src/
│   ├── ugv_perception/      # Vehicle detection (NEW)
│   ├── ugv_mission/         # Mission management (NEW)
│   ├── ugv_inspection/      # Image capture (NEW)
│   ├── ugv_msgs/            # Custom messages (NEW)
│   ├── ugv_nav/             # Navigation (existing)
│   ├── ugv_vision/          # Vision (existing)
│   └── ugv_slam/            # SLAM (existing)
├── maps/                    # Environment maps
├── images/                  # Captured images
├── config/                  # Configuration files
└── scripts/                 # Utility scripts
```

## Requirements

- Ubuntu 22.04 (or compatible)
- ROS2 Humble Hawksbill
- Python 3.10+
- Jetson Orin (with CUDA support for YOLOv8)
- Waveshare UGV Rover with LiDAR and camera

## License

Apache 2.0

## Authors

Jetson UGV Team

## References

- [Waveshare UGV Wiki](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2)
- [ROS2 Navigation](https://navigation.ros.org/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)

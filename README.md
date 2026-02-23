# UGV Workspace – Autonomous Tyre Inspection Robot

An autonomous robot for inspecting commercial vehicle tyres using ROS 2, computer vision, and SLAM-based navigation. Runs on a Waveshare UGV Rover with SLAMTEC Aurora 6DOF.

## Overview

- **Vehicles:** Aurora COCO80 semantic (car, truck, bus)
- **Tyres:** YOLO `best.pt`
- **Navigation:** SLAMTEC Aurora 6DOF SLAM + Nav2
- **Mission:** State machine, 3D bounding boxes, inspection photo capture

## Quick start

```bash
cd ~/ugv_ws
bash scripts/mission_launch.sh
```

Or `bash scripts/startup.sh` for direct launch (no pre-flight checklist). Inspection manager starts ~55s after launch.

## Documentation

| Doc | Description |
|-----|-------------|
| [src/Tyre_Inspection_Bot/ARCHITECTURE.md](src/Tyre_Inspection_Bot/ARCHITECTURE.md) | Hardware/software layout, data flow |
| [src/Tyre_Inspection_Bot/DEPLOYMENT.md](src/Tyre_Inspection_Bot/DEPLOYMENT.md) | ROS 2 version, Ubuntu, build |
| [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) | System overview |
| [MISSION_FLOW_DIAGRAM.md](MISSION_FLOW_DIAGRAM.md) | Mission flow |
| [RUNBOOK.md](RUNBOOK.md) | Operations runbook |
| [docs/AURORA_TOPIC_NODE_REFERENCE.md](docs/AURORA_TOPIC_NODE_REFERENCE.md) | Aurora topics and nodes |
| [docs/AURORA_FIRMWARE_2.11_DEEP_DIVE.md](docs/AURORA_FIRMWARE_2.11_DEEP_DIVE.md) | Aurora firmware behaviour |
| [docs/AURORA_MISSION_SETUP.md](docs/AURORA_MISSION_SETUP.md) | Aurora mission setup |

## Installation

1. Install ROS 2 (Humble or Jazzy), Nav2, TF2
2. Install [SLAMTEC Aurora ROS 2 SDK](https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/) into `src/`
3. `pip3 install -r requirements.txt`
4. `colcon build && source install/setup.bash`
5. Place `best.pt` (tire model) in `~/ugv_ws/` or `src/Tyre_Inspection_Bot/`

See [src/Tyre_Inspection_Bot/README.md](src/Tyre_Inspection_Bot/README.md) for full install and manual launch steps.

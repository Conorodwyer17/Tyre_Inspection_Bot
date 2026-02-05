# Vision and navigation dependencies

What I install for vision and nav: ROS 2 packages, Aurora SDK, Python libs, and system bits.

## ROS 2 packages (Jazzy)

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

**What they’re for**

- **nav2_*** – Path planning and execution (controller, planner, behaviour server, etc.).
- **robot_localization** – EKF/UKF for fusing odom, IMU, GPS. Optional if Aurora already gives fused odom.
- **joint_state_publisher_*** – Joint states for the robot model in RViz.
- **rosbridge_*** – Web bridge (optional).
- **rqt_*** – GUI tools.
- **tf2_*** – Transforms between frames.

**Don’t install**

- ~~depthai-*~~ – OAK-D (obsolete; Aurora replaces it).
- ~~depth-image-proc~~ – Aurora gives point clouds.
- ~~cartographer-*~~, ~~rtabmap-*~~ – Aurora does SLAM.

## External: SLAMTEC Aurora ROS 2 SDK

Required. Install per SLAMTEC’s instructions, then:

1. Put the SDK in `~/ugv_ws/src/`.
2. `colcon build --packages-select slamware_ros_sdk`.
3. `aurora_bringup.launch.py` in ugv_nav is set up for the official package name.

Aurora publishes (namespaced): `/scan`, `/odom` (6DOF), `/imu/data`, `/map` (if provided), and `/tf`.

## Waveshare UGV ESP32 driver

In this repo: **ugv_base_driver**. It does UART/JSON to the Waveshare base, turns Twist into velocity commands, and can publish odom and joint states if the ESP32 sends them. Needs **pyserial**.

## Python dependencies

```bash
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

From requirements: ultralytics, torch, torchvision, opencv-python, numpy, Pillow, ros2-numpy, pyyaml, scipy, scikit-image, setuptools, ament-index-python. Optional for Coral: pycoral, libedgetpu1-std (system package).

## System

**Serial (ESP32):**
```bash
sudo apt install python3-pyserial
sudo usermod -a -G dialout $USER
# log out and back in
```

**Build:**
```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-argcomplete \
  build-essential \
  cmake
```

## Quick check

```bash
ros2 pkg list | grep -E "nav2|robot_localization|tf2"
python3 -c "import ultralytics, ros2_numpy, cv2, torch; print('Dependencies OK')"
ros2 pkg list | grep aurora    # after Aurora SDK install
ros2 pkg list | grep ugv_base_driver
```

## My setup

- **Platform:** Pi 5 (8 GB) or Jetson Orin; Ubuntu 24.04 (ARM64), ROS 2.
- **Distro:** Jazzy.
- **SLAM:** Aurora (no on-board SLAM on the Pi/Jetson).
- **Motors:** ESP32 over UART/JSON.
- **Vision:** Aurora RGB or an external USB camera.

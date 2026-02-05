# Vision and Navigation Dependencies

## ROS 2 Packages (Jazzy)

Install required ROS packages using `apt`:

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

**Descriptions:**

* **`nav2_*`**
  Navigation stack for path planning and execution. Includes controller, planner, behavior server, etc.

* **`robot_localization`**
  Sensor fusion (EKF/UKF) for combining odometry, IMU, GPS, etc. Optional if Aurora provides fused odometry.

* **`joint_state_publisher_*`**
  Publishes joint states for robot model visualization.

* **`rosbridge_*`**
  Web interface bridge for ROS2 (optional, for web control interface).

* **`rqt_*`**
  ROS2 GUI tools for visualization and debugging.

* **`tf2_*`**
  Transform library for coordinate frame transformations.

**Packages NOT Required (Removed):**

* ~~`ros-jazzy-depthai-*`~~ - OAK-D camera (obsolete, replaced by Aurora)
* ~~`ros-jazzy-depth-image-proc`~~ - Depth image processing (Aurora provides point clouds natively)
* ~~`ros-jazzy-cartographer-*`~~ - SLAM (Aurora provides SLAM natively)
* ~~`ros-jazzy-rtabmap-*`~~ - SLAM (Aurora provides SLAM natively)

## External ROS2 Packages

### SLAMTEC Aurora ROS2 SDK

**Required:** SLAMTEC Aurora ROS2 driver package

Installation:
1. Follow SLAMTEC Aurora ROS2 SDK installation instructions
2. Clone or install SDK into workspace: `~/ugv_ws/src/`
3. Build with: `colcon build --packages-select slamware_ros_sdk`
4. Update `aurora_bringup.launch.py` with correct package name

Expected topics from Aurora:
- `/scan` or `/aurora/scan` (LaserScan)
- `/odom` or `/aurora/odom` (Odometry, 6DOF)
- `/imu/data` or `/aurora/imu/data` (IMU)
- `/map` or `/aurora/map` (OccupancyGrid, if provided)
- `/tf` (Transform tree)

### Waveshare UGV ESP32 Driver

**Included:** `ugv_base_driver` package (custom implementation)

The ESP32 motor driver is included in this repository. It handles:
- UART/JSON communication with Waveshare UGV base
- Motor velocity command translation
- Optional odometry and joint state publishing

Dependencies:
- `pyserial` (Python serial communication library)

## Python Dependencies

Install required Python libraries with `pip3`:

```bash
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

From `requirements.txt`:
- **`ultralytics`** - YOLO-based deep learning models for object detection and vision tasks
- **`torch`** - PyTorch deep learning framework
- **`torchvision`** - Computer vision utilities for PyTorch
- **`opencv-python`** - OpenCV for image processing
- **`numpy`** - Numerical computing
- **`Pillow`** - Image I/O
- **`ros2-numpy`** - Utilities for converting ROS 2 messages to NumPy arrays
- **`pyyaml`** - YAML configuration file parsing
- **`scipy`** - Scientific computing
- **`scikit-image`** - Image processing algorithms
- **`setuptools`** - Python package build tools
- **`ament-index-python`** - ROS2 package discovery

**Optional (for Coral USB Accelerator):**
- **`pycoral`** - Python API for Coral Edge TPU
- **`libedgetpu1-std`** - Edge TPU runtime (system package)

## System Dependencies

### UART/Serial Communication

For ESP32 motor driver:
```bash
sudo apt install python3-pyserial
sudo usermod -a -G dialout $USER
# Log out and back in for group change to take effect
```

### Build Tools

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-argcomplete \
  build-essential \
  cmake
```

## Verification

You can verify the installations with:

```bash
# Check ROS2 packages
ros2 pkg list | grep -E "nav2|robot_localization|tf2"

# Check Python dependencies
python3 -c "import ultralytics, ros2_numpy, cv2, torch; print('Dependencies OK')"

# Check Aurora SDK (after installation)
ros2 pkg list | grep aurora

# Check ESP32 driver
ros2 pkg list | grep ugv_base_driver
```

## Architecture Notes

- **Platform:** Raspberry Pi 5 (8GB RAM) or NVIDIA Jetson Orin; Ubuntu 24.04 (ARM64), ROS 2
- **ROS2 Distribution:** Jazzy Jalisco
- **SLAM:** Provided by SLAMTEC Aurora (no on-Pi computation)
- **Motor Control:** ESP32 via UART/JSON protocol
- **Vision:** Aurora RGB camera (if available) or external USB camera

---
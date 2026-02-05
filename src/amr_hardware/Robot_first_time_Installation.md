# First-time setup: onboard computer (Pi 5 or Jetson Orin)

Steps I use to get the board ready and install ROS 2 and the workspace.

## 1. System preparation

### 1.1 Install Ubuntu 24.04

- **Raspberry Pi 5:** Download Ubuntu 24.04 Server (64-bit ARM64), flash to microSD with Raspberry Pi Imager, boot and do the initial setup.
- **Jetson Orin:** Use NVIDIA JetPack with Ubuntu 24.04 (or the LTS they support) and flash per their docs.

### 1.2 Connect to the board

**SSH (handiest):**
```bash
ssh <username>@<board-ip>
```

**Direct:** keyboard, mouse, monitor, or serial (USB-C on Pi; UART/serial on Jetson as per vendor docs).

### 1.3 Network

Make sure the board is on your local network:
```bash
ip addr show
ping -c 3 8.8.8.8
```

## 2. Install ROS 2 Jazzy

### 2.1 Update system

```bash
sudo apt update
sudo apt upgrade -y
```

### 2.2 Prerequisites

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
```

### 2.3 Add ROS 2 repo

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### 2.4 Install Jazzy

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y python3-argcomplete python3-colcon-common-extensions
```

### 2.5 Source ROS 2

Add to `~/.bashrc`:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. ROS 2 dependencies

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

Don’t install: `ros-jazzy-depthai-*` (OAK-D, obsolete), `ros-jazzy-cartographer-*`, `ros-jazzy-rtabmap-*` (Aurora does SLAM).

## 4. Python dependencies

```bash
sudo apt install -y python3-pip python3-venv
cd ~
mkdir -p ugv_ws/src
cd ugv_ws
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

## 5. SLAMTEC Aurora ROS 2 SDK

Follow SLAMTEC’s Aurora ROS 2 SDK instructions:

1. Download the SDK from SLAMTEC.
2. Put it in the workspace, e.g.:
   ```bash
   cd ~/ugv_ws/src
   # Extract or clone the SDK here
   ```
3. The launch file `aurora_bringup.launch.py` (in ugv_nav) expects the package name from the official SDK; adjust if you use a different one.

## 6. Build workspace

```bash
cd ~/ugv_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc`:
```bash
echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 7. ESP32 motor driver

### 7.1 Find the UART port

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
groups   # should include dialout
sudo usermod -a -G dialout $USER
# log out and back in for dialout to apply
```

### 7.2 Test ESP32

```bash
ros2 run ugv_base_driver motor_driver_node --ros-args \
  -p uart_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200
```

(Change the port if yours is different.)

## 8. Startup script

The script is at `src/amr_hardware/ros2_startup.sh`. Copy it to the workspace and make it run on boot:

```bash
cp src/amr_hardware/ros2_startup.sh ~/ugv_ws/
chmod +x ~/ugv_ws/ros2_startup.sh
crontab -e
```

Add:
```bash
@reboot sleep 15 && $HOME/ugv_ws/ros2_startup.sh >> $HOME/ugv_ws/ros2_startup.log 2>&1
```

## 9. Check everything

### ROS 2
```bash
ros2 --version
ros2 pkg list | grep nav2
```

### Aurora
- AP mode: connect to the Aurora Wi-Fi; IP is usually 192.168.11.1.
- On network: use whatever IP the Aurora gets.

```bash
ping 192.168.11.1
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1
# In another terminal:
ros2 topic list | grep slamware
ros2 topic echo /slamware_ros_sdk_server_node/odom
ros2 topic echo /slamware_ros_sdk_server_node/scan
ros2 topic echo /slamware_ros_sdk_server_node/point_cloud
ros2 topic echo /slamware_ros_sdk_server_node/left_image_raw
ros2 topic echo /slamware_ros_sdk_server_node/state
```

### ESP32
```bash
ros2 launch ugv_base_driver esp32_driver.launch.py
# Other terminal:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Nav stack
```bash
ros2 launch ugv_nav nav_aurora.launch.py use_rviz:=true
```

## 10. Optional: Coral USB Accelerator

On a Pi, for faster inference:

```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt update
sudo apt install libedgetpu1-std
pip3 install pycoral
```

## 11. Troubleshooting

**ROS 2 not found:** `ls /opt/ros/jazzy/` then `source /opt/ros/jazzy/setup.bash`.

**Build errors:** From `~/ugv_ws`, `rm -rf build/ install/ log/` then `colcon build --symlink-install`.

**Aurora SDK not found:** `ros2 pkg list | grep slamware`; check LD_LIBRARY_PATH has the aurora_remote_public lib path; `ros2 pkg prefix slamware_ros_sdk`; ping the Aurora; see https://www.slamtec.com/en/aurora.

**ESP32 permission denied:** `sudo usermod -a -G dialout $USER`, then log out and back in.

**Network:** `ip addr show`, `sudo systemctl status networking`.

## 12. Reboot and re-check

```bash
sudo reboot
```

After boot: check `cat ~/ugv_ws/ros2_startup.log`, Aurora, ESP32, and that the nav stack starts.

## 13. Web interface (if you use rosbridge)

```bash
ros2 topic list | grep rosbridge
# Browser: http://<board-ip>:9090
```

For day-to-day usage, see the main README.md.

# First-Time Setup Guide for Onboard Computer (Raspberry Pi 5 or Jetson Orin)

## 1. System Preparation

### 1.1 Install Ubuntu 24.04

- **Raspberry Pi 5:** Download Ubuntu 24.04 Server (64-bit ARM64), flash to microSD with Raspberry Pi Imager, boot and complete initial setup.
- **Jetson Orin:** Use NVIDIA JetPack with Ubuntu 24.04 (or supported LTS) and flash per NVIDIA documentation.

### 1.2 Connect to the Onboard Computer

**Via SSH (recommended):**
```bash
ssh <username>@<board-ip>
```

**Via direct connection:**
- Connect keyboard, mouse, and monitor to the board
- Or use serial console (USB-C on Pi; UART/serial on Jetson as per vendor docs)

### 1.3 Configure Network

Ensure the onboard computer is connected to your local network:
```bash
# Check network connection
ip addr show
ping -c 3 8.8.8.8
```

## 2. Install ROS 2 Jazzy

### 2.1 Update System

```bash
sudo apt update
sudo apt upgrade -y
```

### 2.2 Install Prerequisites

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
```

### 2.3 Add ROS 2 Repository

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### 2.4 Install ROS 2 Jazzy

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

## 3. Install ROS 2 Dependencies

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

**Important:** Do NOT install:
- `ros-jazzy-depthai-*` (OAK-D camera, obsolete)
- `ros-jazzy-cartographer-*` (SLAM, AORA replaces)
- `ros-jazzy-rtabmap-*` (SLAM, AORA replaces)

## 4. Install Python Dependencies

```bash
sudo apt install -y python3-pip python3-venv
cd ~
mkdir -p ugv_ws/src
cd ugv_ws
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

## 5. Install SLAMTEC AORA ROS2 SDK

**TODO:** Follow SLAMTEC AORA ROS2 SDK installation instructions:

1. Download AORA ROS2 SDK from SLAMTEC
2. Clone or install SDK into workspace:
   ```bash
   cd ~/ugv_ws/src
   # Follow AORA SDK installation instructions
   # Example: git clone <aora-sdk-url> aora_ros2_driver
   ```
3. Update `src/amr_hardware/src/ugv_nav/launch/aora_bringup.launch.py` with correct package name

## 6. Build Workspace

```bash
cd ~/ugv_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

Add workspace sourcing to `~/.bashrc`:
```bash
echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 7. Configure ESP32 Motor Driver

### 7.1 Identify UART Port

```bash
# List serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Check permissions
groups  # Should include 'dialout' group
sudo usermod -a -G dialout $USER
# Log out and back in for group change to take effect
```

### 7.2 Test ESP32 Connection

```bash
# Test UART communication (adjust port as needed)
ros2 run ugv_base_driver motor_driver_node --ros-args \
  -p uart_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200
```

## 8. Install Startup Script

### 8.1 Copy Startup Script

The `ros2_startup.sh` script is located at:
```
src/amr_hardware/ros2_startup.sh
```

Copy to workspace root:
```bash
cp src/amr_hardware/ros2_startup.sh ~/ugv_ws/
chmod +x ~/ugv_ws/ros2_startup.sh
```

### 8.2 Configure Crontab for Automatic Startup

Edit crontab:
```bash
crontab -e
```

Add the following line:
```bash
@reboot sleep 15 && $HOME/ugv_ws/ros2_startup.sh >> $HOME/ugv_ws/ros2_startup.log 2>&1
```

## 9. Verify Installation

### 9.1 Check ROS 2 Installation

```bash
ros2 --version
ros2 pkg list | grep nav2
```

### 9.2 Test Aurora Connection

**Connect to Aurora:**
- If Aurora in AP mode: Connect to Wi-Fi network "Aurora-XXXX", IP: 192.168.11.1
- If Aurora on network: Use Aurora's assigned IP address

```bash
# Test network connectivity
ping 192.168.11.1  # Or Aurora's network IP

# Launch Aurora SDK node
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1

# In another terminal, verify topics
ros2 topic list | grep slamware
ros2 topic echo /slamware_ros_sdk_server_node/odom
ros2 topic echo /slamware_ros_sdk_server_node/scan
ros2 topic echo /slamware_ros_sdk_server_node/point_cloud
ros2 topic echo /slamware_ros_sdk_server_node/left_image_raw

# Check connection state
ros2 topic echo /slamware_ros_sdk_server_node/state
```

### 9.3 Test ESP32 Connection

```bash
# After connecting ESP32
ros2 launch ugv_base_driver esp32_driver.launch.py
# In another terminal:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 9.4 Test Navigation Stack

```bash
ros2 launch ugv_nav nav_aurora.launch.py use_rviz:=true
```

## 10. Optional: Install Coral USB Accelerator

For enhanced AI inference performance:

```bash
# Add Coral repository
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -

# Install Edge TPU runtime
sudo apt update
sudo apt install libedgetpu1-std

# Install PyCoral
pip3 install pycoral
```

## 11. Troubleshooting

### ROS 2 Not Found

```bash
# Verify ROS 2 is installed
ls /opt/ros/jazzy/

# Source ROS 2
source /opt/ros/jazzy/setup.bash
```

### Build Errors

```bash
# Clean and rebuild
cd ~/ugv_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Aurora SDK Not Found

- Verify Aurora SDK is installed: `ros2 pkg list | grep slamware`
- Check LD_LIBRARY_PATH includes aurora_remote_public library path
- Verify launch file exists: `ros2 pkg prefix slamware_ros_sdk`
- Check network connection to Aurora device (Wi-Fi or Ethernet)
- Default Aurora IP in AP mode: 192.168.11.1
- Check SLAMTEC documentation: https://www.slamtec.com/en/aurora

### ESP32 Permission Denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Network Issues

```bash
# Check network configuration
ip addr show
sudo systemctl status networking
```

## 12. Restart and Verify

Reboot the onboard computer:
```bash
sudo reboot
```

After reboot, verify:
1. ROS 2 nodes start automatically (check log: `cat ~/ugv_ws/ros2_startup.log`)
2. Aurora device is detected
3. ESP32 motor driver connects
4. Navigation stack initializes

## 13. Access Web Interface (if configured)

If using rosbridge for web interface:
```bash
# Check rosbridge is running
ros2 topic list | grep rosbridge

# Access via browser
http://<board-ip>:9090
```

---

**Setup Complete!**

For detailed usage instructions, see the main README.md file.

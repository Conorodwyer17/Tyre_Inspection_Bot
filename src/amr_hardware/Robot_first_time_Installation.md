# First-Time Connection and Setup Guide for the Robot

## 1. Connect to the Jetson via USB-C
1. Connect a USB Type-C cable from the Jetson to your laptop.
2. Access the Jetson using SSH:
```bash
ssh jetson@192.168.55.1    # Default USB-C network address for Jetson
````

## 2. Configure Network and Wi-Fi

Follow the official Waveshare documentation for network configuration, Wi-Fi setup, and removing AccessPopup:
[https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2)

After completing the network setup:

* Restart the robot.
* Ensure it automatically connects to your local Wi-Fi network.
* Verify you can SSH into the Jetson using its Wi-Fi IP address:

```bash
ssh jetson@192.x.x.x   # Replace with the Jetson’s Wi-Fi IP
```

(Use USB-C again anytime to check the assigned IP.)

## 3. Remove the Default Startup Program

Follow the instructions to remove the factory startup program:
[https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_1._Preparation](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_1._Preparation)

# Local ROS 2 Installation for Development

## Install prerequisites

```bash

sudo apt-get update
sudo apt-get upgrade
sudo apt install python3-pip
sudo apt install alsa-utils
sudo apt install python3-colcon-argcomplete
sudo apt install ros-humble-cartographer-*
sudo apt install ros-humble-desktop-*
sudo apt install ros-humble-joint-state-publisher-*
sudo apt install ros-humble-nav2-*
sudo apt install ros-humble-rosbridge-*
sudo apt install ros-humble-rqt-*
sudo apt install ros-humble-rtabmap-*
sudo apt install ros-humble-usb-cam
sudo apt install ros-humble-depthai-*
sudo apt install gazebo
sudo apt install ros-humble-gazebo-*

```


Install Python dependencies:

```bash
cd ~/ugv_ws
python3 -m pip install -r requirements.txt
```

## 4. Clean and Rebuild the Workspace

Remove the old build directories:

```bash
cd ~/ugv_ws
rm -r build/ install/ log/
```

Rebuild the workspace:

```bash
colcon build --symlink-install
```

## 5. Install the Startup Script

Copy the provided `ros2_startup.sh` script into:

```
/home/jetson/ugv_ws/
```

## 6. Configure Crontab for Automatic Startup

Edit the crontab:

```bash
crontab -e
```

Remove the old entry and add:

```bash
@reboot sleep 10 && /home/jetson/ugv_ws/ros2_startup.sh >> /home/jetson/ugv_ws/ros2_startup.log 2>&1
```

## 7. Update and Source ~/.bashrc

Edit your bashrc:

```bash
nano ~/.bashrc
```

Add the following:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/ugv_ws/install/setup.bash
```

Apply changes:

```bash
source ~/.bashrc
```

## 8. Restart the Robot

Reboot the robot and verify that the ROS 2 nodes start automatically and operate correctly.

```

### 10. Access the Web Control Interface

Open any web browser on a device connected to the same network, then navigate to:


```
robot-ip:5100
```

Example:

```
[http://192.168.100.203:5100/](http://192.168.100.203:5100/)
```


After the interface loads, add the robot-model widget.  
If the robot model appears correctly, the setup is complete.


---
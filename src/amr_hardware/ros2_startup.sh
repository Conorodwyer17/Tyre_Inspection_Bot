#!/bin/bash
# wait for system to be ready
sleep 15

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace
source $HOME/ugv_ws/install/setup.bash

# log file (use generic home directory)
LOGFILE="$HOME/ugv_ws/ros2_startup.log"
echo "Starting ROS 2 launches at $(date)" >> $LOGFILE

# detect host IP automatically
HOST_IP=$(hostname -I | awk '{print $1}')
echo "Detected IP: $HOST_IP" >> $LOGFILE

# Launch navigation with Aurora (localization mode, not SLAM)
ros2 launch ugv_nav nav_aurora.launch.py use_rviz:=false >> $LOGFILE 2>&1 &



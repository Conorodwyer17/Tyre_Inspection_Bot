#!/bin/bash
# Give the system a moment to come up
sleep 15

# ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source $HOME/ugv_ws/install/setup.bash

LOGFILE="$HOME/ugv_ws/ros2_startup.log"
echo "Starting ROS 2 launches at $(date)" >> $LOGFILE

HOST_IP=$(hostname -I | awk '{print $1}')
echo "Detected IP: $HOST_IP" >> $LOGFILE

# Nav with Aurora (localisation only; Aurora does the SLAM)
ros2 launch ugv_nav nav_aurora.launch.py use_rviz:=false >> $LOGFILE 2>&1 &



#!/bin/bash
# wait for system to be ready
sleep 15


# source bashrc for environment variables
source /home/jetson/.bashrc

# log file
LOGFILE="/home/jetson/ugv_ws/ros2_startup.log"
echo "Starting ROS 2 launches at $(date)" >> $LOGFILE

# detect host IP automatically
HOST_IP=$(hostname -I | awk '{print $1}')
echo "Detected IP: $HOST_IP" >> $LOGFILE
export UGV_MODEL=ugv_rover
export LDLIDAR_MODEL=ld06
# 1️⃣ Launch SLAM navigation (without RViz)
ros2 launch ugv_nav slam_nav.launch.py use_rviz:=false >> $LOGFILE 2>&1 &



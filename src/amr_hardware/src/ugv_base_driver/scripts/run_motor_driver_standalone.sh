#!/bin/bash
# Run motor driver node with workspace and UART port for Jetson + Waveshare base.
# Uses /dev/ttyTHS1 by default (Jetson GPIO UART). Override: UART_PORT=/dev/ttyTHS0 ./run_motor_driver_standalone.sh

set -e

UART_PORT="${UART_PORT:-/dev/ttyTHS1}"
UGV_WS="${UGV_WS:-$HOME/ugv_ws}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble not found. Source your ROS 2 install."
    exit 1
fi
source /opt/ros/humble/setup.bash

if [ -f "$UGV_WS/install/setup.bash" ]; then
    source "$UGV_WS/install/setup.bash"
elif [ -f "$UGV_WS/install/ugv_base_driver/share/ugv_base_driver/package.sh" ]; then
    source "$UGV_WS/install/ugv_base_driver/share/ugv_base_driver/package.sh"
fi

export UGV_UART_PORT="$UART_PORT"
ros2 run ugv_base_driver motor_driver_node --ros-args -p uart_port:="$UART_PORT" -p baud_rate:=115200

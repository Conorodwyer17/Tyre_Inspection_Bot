#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial  
import json  
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
import subprocess
import time
import os

def is_jetson():
    result = any("ugv_jetson" in root for root, dirs, files in os.walk("/"))
    return result

# CRITICAL: ugv_bringup handles all ESP32 serial communication via /dev/ttyAMA0
# This node should NOT open a separate serial port to avoid conflicts
# Per Waveshare architecture: single process should own the serial port
# Reference: https://github.com/waveshareteam/ugv_base_ros

# NOTE: If you need ugv_driver to send commands, it should use ROS topics
# to communicate with ugv_bringup, or share the same serial port instance
# For now, we disable serial port opening here to avoid conflicts
ser = None
serial_port = None  # Not used - ugv_bringup owns serial communication

class UgvDriver(Node):
    def __init__(self, name):
        super().__init__(name)

        # NOTE: cmd_vel is handled by ugv_bringup - removing duplicate to avoid conflicts
        # ugv_bringup sends properly scaled T:1 commands, so ugv_driver shouldn't duplicate this
        # If needed, you can re-enable this, but ensure both nodes use the same serial port
        # self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # Subscribe to joint states (ugv/joint_states topic)
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)

        # Subscribe to LED control data (ugv/led_ctrl topic)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)

        # Subscribe to voltage data (voltage topic)
        self.voltage_sub = self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)

    # Callback for processing velocity commands
    # NOTE: DISABLED - ugv_bringup handles cmd_vel with proper T:1 format and scaling
    # Re-enable this only if you want ugv_driver to also send commands (not recommended)
    def cmd_vel_callback(self, msg):
        # This callback is disabled to avoid conflicts with ugv_bringup
        # ugv_bringup properly handles cmd_vel with T:1 format, scaling, and flush
        pass

    # Callback for processing joint state updates
    def joint_states_callback(self, msg):
        header = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
        }

        # Extract joint positions and convert to degrees
        name = msg.name
        position = msg.position

        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]

        x_degree = (180 * x_rad) / 3.1415926
        y_degree = (180 * y_rad) / 3.1415926

        # Send the joint data as a JSON string to the UGV
        joint_data = json.dumps({
            'T': 134, 
            'X': x_degree, 
            'Y': y_degree, 
            "SX": 600,
            "SY": 600,
        }) + "\n"
                
        # NOTE: Serial port disabled - ugv_bringup owns serial communication
        # To send joint state commands, integrate into ugv_bringup or use ROS topics
        if ser is not None and ser.is_open:
            ser.write(joint_data.encode())
            ser.flush()
        else:
            self.get_logger().debug(
                f"Joint states received but serial port not available (X={x_degree:.2f}°, Y={y_degree:.2f}°). "
                "ugv_bringup handles all serial communication."
            )

    # Callback for processing LED control commands
    def led_ctrl_callback(self, msg):
        IO4 = msg.data[0]
        IO5 = msg.data[1]
        
        # Send LED control data as a JSON string to the UGV
        led_ctrl_data = json.dumps({
            'T': 132, 
            "IO4": IO4,
            "IO5": IO5,
        }) + "\n"
                
        # NOTE: Serial port disabled - ugv_bringup owns serial communication
        # To send LED control commands, integrate into ugv_bringup or use ROS topics
        if ser is not None and ser.is_open:
            ser.write(led_ctrl_data.encode())
            ser.flush()
        else:
            self.get_logger().debug(
                f"LED control received but serial port not available (IO4={IO4}, IO5={IO5}). "
                "ugv_bringup handles all serial communication."
            )

    # Callback for processing voltage data
    def voltage_callback(self, msg):
        voltage_value = msg.data

        # If voltage drops below a threshold, play a low battery warning sound
        if 0.1 < voltage_value < 9: 
            subprocess.run(['aplay', '-D', 'plughw:3,0', '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'])
            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = UgvDriver("ugv_driver")
    
    try:
        rclpy.spin(node)  # Keep the node running and handling callbacks
    except KeyboardInterrupt:
        pass  # Graceful shutdown on user interrupt
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Serial port not opened in this node (ugv_bringup owns it)
        # if ser is not None and ser.is_open:
        #     ser.close()

if __name__ == '__main__':
    main()

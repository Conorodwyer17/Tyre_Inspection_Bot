#!/usr/bin/env python3
"""
ROS 2 node: subscribe to /cmd_vel (Twist), send Waveshare JSON over UART to ESP32.
Protocol: CMD_ROS_CTRL {"T":13,"X":linear_m/s,"Z":angular_rad_s}
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import serial
except ImportError:
    serial = None


def _send_json(ser, obj):
    """Send one JSON line to ESP32 (compact, newline)."""
    if ser is None or not ser.is_open:
        return
    line = json.dumps(obj, separators=(',', ':')) + '\n'
    ser.write(line.encode('utf-8'))


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        default_port = os.environ.get('UGV_UART_PORT', '/dev/ttyTHS1')
        self.declare_parameter('uart_port', default_port)
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)

        port = self.get_parameter('uart_port').value
        baud = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value

        self.serial = None
        if serial is None:
            self.get_logger().error('pyserial not installed: pip install pyserial')
            return
        try:
            self.serial = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
            _send_json(self.serial, {'T': 900, 'main': 2, 'module': 0})
            time.sleep(0.3)
            self.get_logger().info('Sent chassis config (UGV Rover, no module)')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32 on {port}: {e}')
            self.get_logger().warn('Motor driver will keep running but cmd_vel will be ignored')

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('Subscribed to /cmd_vel')

    def cmd_vel_callback(self, msg):
        """Twist -> Waveshare CMD_ROS_CTRL: {"T":13,"X":linear_x_m/s,"Z":angular_z_rad_s}"""
        if self.serial is None or not self.serial.is_open:
            self.get_logger().warn_once('ESP32 not connected, ignoring cmd_vel')
            return
        cmd = {
            'T': 13,
            'X': float(msg.linear.x),
            'Z': float(msg.angular.z)
        }
        try:
            _send_json(self.serial, cmd)
        except Exception as e:
            self.get_logger().error(f'UART write failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial and node.serial.is_open:
            _send_json(node.serial, {'T': 1, 'L': 0.0, 'R': 0.0})
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import serial
import json
import time

class MotorDriverNode(Node):
    """
    ROS2 node for Waveshare UGV ESP32 motor control via UART/JSON.
    
    Subscribes to /cmd_vel (Twist) and sends commands to ESP32 via UART.
    Optionally publishes odometry and joint states if ESP32 provides them.
    """
    
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # Declare parameters
        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('publish_odom', False)  # Set to True if ESP32 provides odom
        self.declare_parameter('publish_joint_states', False)  # Set to True if ESP32 provides joint states
        
        # Get parameters
        port = self.get_parameter('uart_port').value
        baud = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_joint_states = self.get_parameter('publish_joint_states').value
        
        # Initialize UART connection
        self.serial = None
        try:
            self.serial = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32 on {port}: {e}')
            self.get_logger().warn('Motor driver will continue but commands will not be sent')
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Subscribed to /cmd_vel')
        
        # Publishers (optional, based on ESP32 capabilities)
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, '/odom_esp32', 10)
            self.get_logger().info('Publishing odometry from ESP32')
        
        if self.publish_joint_states:
            self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.get_logger().info('Publishing joint states from ESP32')
        
        # Timer for reading ESP32 responses (if bidirectional communication)
        self.read_timer = self.create_timer(0.1, self.read_esp32_data)
        
        self.get_logger().info('Motor driver node initialized')
    
    def cmd_vel_callback(self, msg):
        """
        Callback for /cmd_vel topic.
        Converts Twist message to JSON and sends to ESP32 via UART.
        """
        if self.serial is None or not self.serial.is_open:
            self.get_logger().warn_once('ESP32 not connected, ignoring cmd_vel')
            return
        
        # Convert Twist to JSON format expected by ESP32
        # Adjust format based on actual Waveshare UGV protocol
        cmd = {
            'linear': {
                'x': float(msg.linear.x),
                'y': float(msg.linear.y),
                'z': float(msg.linear.z)
            },
            'angular': {
                'x': float(msg.angular.x),
                'y': float(msg.angular.y),
                'z': float(msg.angular.z)
            }
        }
        
        # Send via UART
        try:
            json_str = json.dumps(cmd) + '\n'
            self.serial.write(json_str.encode('utf-8'))
            self.get_logger().debug(f'Sent command: {json_str.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command to ESP32: {e}')
            # Try to reconnect
            try:
                if self.serial:
                    self.serial.close()
                port = self.get_parameter('uart_port').value
                baud = self.get_parameter('baud_rate').value
                timeout = self.get_parameter('timeout').value
                self.serial = serial.Serial(port, baud, timeout=timeout)
                self.get_logger().info('Reconnected to ESP32')
            except Exception as reconnect_error:
                self.get_logger().error(f'Failed to reconnect: {reconnect_error}')
                self.serial = None
    
    def read_esp32_data(self):
        """
        Read data from ESP32 (odometry, joint states, etc.)
        This is called periodically by a timer.
        """
        if self.serial is None or not self.serial.is_open:
            return
        
        try:
            # Check if data is available
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                if line:
                    try:
                        data = json.loads(line)
                        # Process ESP32 data based on protocol
                        # Example: {'odom': {...}, 'joint_states': {...}}
                        
                        if self.publish_odom and 'odom' in data:
                            odom_msg = self.parse_odometry(data['odom'])
                            if odom_msg:
                                self.odom_pub.publish(odom_msg)
                        
                        if self.publish_joint_states and 'joint_states' in data:
                            joint_msg = self.parse_joint_states(data['joint_states'])
                            if joint_msg:
                                self.joint_states_pub.publish(joint_msg)
                                
                    except json.JSONDecodeError:
                        self.get_logger().debug(f'Invalid JSON from ESP32: {line}')
        except Exception as e:
            self.get_logger().debug(f'Error reading from ESP32: {e}')
    
    def parse_odometry(self, odom_data):
        """
        Parse odometry data from ESP32 JSON format to ROS2 Odometry message.
        Adjust based on actual ESP32 protocol.
        """
        try:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # Extract pose and twist from odom_data
            # Adjust field names based on actual ESP32 protocol
            if 'pose' in odom_data:
                pose = odom_data['pose']
                odom_msg.pose.pose.position.x = float(pose.get('x', 0.0))
                odom_msg.pose.pose.position.y = float(pose.get('y', 0.0))
                odom_msg.pose.pose.position.z = float(pose.get('z', 0.0))
                # Handle orientation (quaternion or euler)
                if 'orientation' in pose:
                    orient = pose['orientation']
                    odom_msg.pose.pose.orientation.x = float(orient.get('x', 0.0))
                    odom_msg.pose.pose.orientation.y = float(orient.get('y', 0.0))
                    odom_msg.pose.pose.orientation.z = float(orient.get('z', 0.0))
                    odom_msg.pose.pose.orientation.w = float(orient.get('w', 1.0))
            
            if 'twist' in odom_data:
                twist = odom_data['twist']
                odom_msg.twist.twist.linear.x = float(twist.get('linear', {}).get('x', 0.0))
                odom_msg.twist.twist.linear.y = float(twist.get('linear', {}).get('y', 0.0))
                odom_msg.twist.twist.linear.z = float(twist.get('linear', {}).get('z', 0.0))
                odom_msg.twist.twist.angular.x = float(twist.get('angular', {}).get('x', 0.0))
                odom_msg.twist.twist.angular.y = float(twist.get('angular', {}).get('y', 0.0))
                odom_msg.twist.twist.angular.z = float(twist.get('angular', {}).get('z', 0.0))
            
            return odom_msg
        except Exception as e:
            self.get_logger().error(f'Error parsing odometry: {e}')
            return None
    
    def parse_joint_states(self, joint_data):
        """
        Parse joint state data from ESP32 JSON format to ROS2 JointState message.
        Adjust based on actual ESP32 protocol.
        """
        try:
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Extract joint names, positions, velocities, efforts
            # Adjust field names based on actual ESP32 protocol
            if 'names' in joint_data:
                joint_msg.name = [str(name) for name in joint_data['names']]
            if 'positions' in joint_data:
                joint_msg.position = [float(p) for p in joint_data['positions']]
            if 'velocities' in joint_data:
                joint_msg.velocity = [float(v) for v in joint_data['velocities']]
            if 'efforts' in joint_data:
                joint_msg.effort = [float(e) for e in joint_data['efforts']]
            
            return joint_msg
        except Exception as e:
            self.get_logger().error(f'Error parsing joint states: {e}')
            return None
    
    def destroy_node(self):
        """Clean up on node shutdown"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.get_logger().info('Closed ESP32 connection')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

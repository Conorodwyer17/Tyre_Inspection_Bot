import serial
import json
import queue
import threading
import rclpy
from rclpy.node import Node
import logging
import time
import traceback
from std_msgs.msg import Header, Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField
import math

# ============================================================================
# CRITICAL CONSTANTS - DO NOT MODIFY WITHOUT UNDERSTANDING IMPACT
# ============================================================================
# These constants are derived from Waveshare UGV Rover hardware specifications
# and MUST match the values used in base_node.cpp for odometry calculations.
# Changing these will cause incorrect motor commands and odometry drift.

# Wheelbase: Distance between left and right wheel centers (meters)
# Source: base_node.cpp line 201 uses 0.175 for differential drive calculation
# CRITICAL: This MUST match base_node.cpp or odometry will be incorrect
WHEELBASE_M = 0.175

# Maximum robot speed: Maximum linear velocity the robot can achieve (m/s)
# Source: Waveshare WAVE_ROVER documentation
# CRITICAL: This is used to scale cmd_vel commands to PWM range (-0.5 to +0.5)
# Changing this will cause incorrect speed scaling - robot will move too fast/slow
MAX_ROBOT_SPEED_MPS = 1.3

# PWM Command Range: ESP32 expects wheel speed commands in this range
# Range: -0.5 to +0.5 where:
#   -0.5 = 100% PWM reverse
#   0.0  = Stop
#  +0.5  = 100% PWM forward
# CRITICAL: This is the ESP32 firmware's expected input range. Do not change.
MAX_PWM_COMMAND = 0.5
MIN_PWM_COMMAND = -0.5

# Command Type: T:1 is wheel speed control (recommended by Waveshare)
# Alternative: T:13 is velocity control (may not be supported by all firmware)
# CRITICAL: T:1 format is required - changing this will break motor control
WHEEL_SPEED_COMMAND_TYPE = 1

# Serial Communication Constants
# CRITICAL: Baud rate must match ESP32 firmware (115200 for GPIO UART)
DEFAULT_SERIAL_BAUD = 115200
# CRITICAL: Port detection - Jetson uses /dev/ttyTHS1, others use /dev/ttyAMA0
# These are hardware-specific - changing may break communication
DEFAULT_SERIAL_PORT_JETSON = '/dev/ttyTHS1'
DEFAULT_SERIAL_PORT_OTHER = '/dev/ttyAMA0'
# ============================================================================

# Helper class for reading lines from a serial port
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()  # Buffer to store incoming data
        self.s = s  # Serial object

    # Read a line of data from the serial input
    # ESP32 sends frames delimited by \r (0x0d), not \n
    def readline(self):
        # Look for both \r and \n (for compatibility)
        i = self.buf.find(b"\r")
        if i < 0:
            i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            try:
                if self.s.in_waiting == 0:
                    # No data available, return empty bytes (timeout will be handled by caller)
                    return b''
                i = max(1, min(512, self.s.in_waiting))  # Read from serial buffer
                data = self.s.read(i)
                if len(data) == 0:
                    # Timeout occurred, return empty bytes
                    return b''
            except serial.SerialException as e:
                # Handle case where device reports readiness but returns no data
                # This can happen if ESP32 disconnects or is busy
                # Return empty bytes to let caller handle gracefully
                return b''
            # Look for both \r and \n (ESP32 uses \r, but support \n for compatibility)
            i = data.find(b"\r")
            if i < 0:
                i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

    # Clear the buffer
    def clear_buffer(self):
        self.s.reset_input_buffer()

# Base controller class for managing UART communication and processing commands
class BaseController:
    def __init__(self, uart_dev_set, baud_set, ros_logger=None):
        # Use ROS2 logger if provided, otherwise use Python logging
        if ros_logger is not None:
            self.logger = ros_logger
            self.use_ros_logger = True
        else:
            self.logger = logging.getLogger('BaseController')  # Logger setup
            self.logger.setLevel(logging.DEBUG)  # Enable DEBUG level logging to see command writes
            # Configure Python logging to output to console
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setLevel(logging.DEBUG)
                formatter = logging.Formatter('[BaseController] %(levelname)s: %(message)s')
                handler.setFormatter(formatter)
                self.logger.addHandler(handler)
            self.use_ros_logger = False
        try:
            # Match ugv_driver.py serial configuration exactly - simpler is better
            # CRITICAL: Set write_timeout to ensure writes don't hang, and use write_through=False
            # to allow flush() to work properly for immediate command transmission
            self.ser = serial.Serial(
                uart_dev_set, 
                baud_set, 
                timeout=1,  # Read timeout (seconds)
                write_timeout=1  # Write timeout (seconds) - prevents hangs
            )
            # Clear any stale data in buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            # Small delay to let serial port stabilize (matching common practice)
            time.sleep(0.1)
            self.logger.info(f"BaseController: Serial port {uart_dev_set} opened successfully at {baud_set} baud")
        except serial.SerialException as e:
            self.logger.error(f"BaseController: Failed to open serial port {uart_dev_set}: {e}")
            raise
        self.rl = ReadLine(self.ser)  # Initialize ReadLine helper
        # Initialize command queue and thread for async command processing
        # This allows non-blocking command sending while feedback loop runs
        # CRITICAL: Initialize command queue and thread BEFORE any commands are sent
        # This was a bug fix - queue must exist when send_command() is called
        # DO NOT REMOVE - send_command() will fail with AttributeError if this is missing
        self.command_queue = queue.Queue()  # Command queue for sending data
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()
        self.logger.info("BaseController: Command processing thread started")
        self.data_buffer = None  # Buffer for holding received data
        # Base data structure to hold sensor values
        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, "odl": 0, "odr": 0, "v": 0}
    
    # Function to read and return feedback data from the serial input
    def feedback_data(self):
        """
        Read feedback data from ESP32 via serial port.
        
        Handles serial read errors gracefully - if ESP32 disconnects or is busy,
        returns last known data instead of crashing. This ensures the feedback
        loop continues running even if serial communication is intermittent.
        """
        try:
            line_bytes = self.rl.readline()  # Read line from UART
            if len(line_bytes) == 0:
                # No data received (timeout), return last known data
                return self.base_data
            
            # ESP32 sends binary-framed JSON data - extract JSON from binary stream
            # Look for JSON start marker '{' (0x7b) and extract until matching '}'
            json_start = line_bytes.find(b'{')
            if json_start >= 0:
                # Found JSON start, extract JSON portion
                json_bytes = line_bytes[json_start:]
                # Find JSON end - look for closing '}' that matches the first '{'
                brace_count = 0
                json_end = -1
                for i, byte in enumerate(json_bytes):
                    if byte == ord('{'):
                        brace_count += 1
                    elif byte == ord('}'):
                        brace_count -= 1
                        if brace_count == 0:
                            json_end = i + 1
                            break
                
                if json_end > 0:
                    # Extract complete JSON
                    json_data = json_bytes[:json_end]
                    try:
                        line = json_data.decode('utf-8')
                        self.data_buffer = json.loads(line)  # Parse JSON data
                        # CRITICAL: Ensure data_buffer is a dict before assigning to base_data
                        if isinstance(self.data_buffer, dict):
                            self.base_data = self.data_buffer  # Store received data
                        else:
                            # Invalid data type - log and keep existing base_data
                            if self.use_ros_logger:
                                self.logger.warn(f"Invalid data type from JSON: {type(self.data_buffer)}, keeping existing base_data")
                            else:
                                self.logger.warn(f"Invalid data type from JSON: {type(self.data_buffer)}, keeping existing base_data")
                        return self.base_data  # Return base data (always a dict)
                    except (UnicodeDecodeError, json.JSONDecodeError) as e:
                        # JSON extraction found but decode failed - skip this message
                        if self.use_ros_logger:
                            self.logger.debug(f"Failed to decode extracted JSON: {e}")
                        else:
                            self.logger.debug(f"Failed to decode extracted JSON: {e}")
                        return self.base_data  # Return last known data (always a dict)
                else:
                    # JSON start found but no complete JSON - might need more data
                    return self.base_data
            else:
                # No JSON marker found - might be binary data or incomplete message
                # Try to decode as UTF-8 anyway (for backward compatibility)
                try:
                    line = line_bytes.decode('utf-8', errors='ignore').strip()
                    if len(line) > 0:
                        self.data_buffer = json.loads(line)  # Parse JSON data
                        # CRITICAL: Ensure data_buffer is a dict before assigning to base_data
                        if isinstance(self.data_buffer, dict):
                            self.base_data = self.data_buffer  # Store received data
                        else:
                            # Invalid data type - log and keep existing base_data
                            if self.use_ros_logger:
                                self.logger.warn(f"Invalid data type from JSON (fallback): {type(self.data_buffer)}, keeping existing base_data")
                            else:
                                self.logger.warn(f"Invalid data type from JSON (fallback): {type(self.data_buffer)}, keeping existing base_data")
                        return self.base_data  # Return base data (always a dict)
                    else:
                        return self.base_data  # Return last known data (always a dict)
                except json.JSONDecodeError as e:
                    # Not valid JSON, skip this message (log at debug level to avoid spam)
                    if self.use_ros_logger:
                        self.logger.debug(f"JSON decode error in fallback decode: {e}")
                    else:
                        self.logger.debug(f"JSON decode error in fallback decode: {e}")
                    return self.base_data  # Return last known data (always a dict)
        except json.JSONDecodeError as e:
            # Log error but don't spam - use throttling at logger level
            if self.use_ros_logger:
                self.logger.error(f"JSON decode error: {e}\n{traceback.format_exc()}")
            else:
                self.logger.error(f"JSON decode error: {e}\n{traceback.format_exc()}")
            self.rl.clear_buffer()  # Clear buffer on error
            return self.base_data  # Return last known data
        except UnicodeDecodeError as e:
            if self.use_ros_logger:
                self.logger.error(f"Unicode decode error: {e}\n{traceback.format_exc()}")
            else:
                self.logger.error(f"Unicode decode error: {e}\n{traceback.format_exc()}")
            self.rl.clear_buffer()
            return self.base_data  # Return last known data
        except serial.SerialException as e:
            # Handle serial read errors gracefully (device disconnected, multiple access, etc.)
            # This can happen if ESP32 is busy or USB serial adapter has issues
            # Log at DEBUG level to avoid spam (errors occur periodically but don't break functionality)
            if self.use_ros_logger:
                self.logger.debug(
                    f"Serial read error in feedback_data (non-critical, using last known data): {e}",
                    throttle_duration_sec=5.0
                )
            else:
                self.logger.debug(
                    f"Serial read error in feedback_data (non-critical, using last known data): {e}",
                    throttle_duration_sec=5.0
                )
            # Don't clear buffer on SerialException - might be temporary
            return self.base_data  # Return last known data
        except Exception as e:
            # Other unexpected errors - log at ERROR level
            if self.use_ros_logger:
                self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}\n{traceback.format_exc()}")
            else:
                self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}\n{traceback.format_exc()}")
            self.rl.clear_buffer()
            return self.base_data  # Return last known data

    # Receive and decode data from the serial connection
    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))  # Read and parse JSON data
        return data_read

    # Add a command to the queue to be sent via UART (legacy method)
    def send_command(self, data):
        self.command_queue.put(data)
    
    # Direct write method (like ugv_driver.py) - writes immediately without queue
    def write_command_direct(self, data):
        """Write command directly to serial port immediately, matching ugv_driver.py behavior EXACTLY"""
        try:
            # Match ugv_driver.py EXACTLY: json.dumps + "\n" then encode (same as ugv_driver line 68-69)
            cmd_str = json.dumps(data) + "\n"
            bytes_written = self.ser.write(cmd_str.encode())
            # CRITICAL: Flush immediately to ensure ESP32 receives command without delay
            self.ser.flush()
            if bytes_written > 0:
                # Log at INFO level to verify commands are being sent (matching ugv_driver debug level)
                if self.use_ros_logger:
                    self.logger.info(f"[BaseController] Sent {bytes_written} bytes to serial: {cmd_str.strip()}")
                else:
                    self.logger.info(f"[BaseController] Sent {bytes_written} bytes to serial: {cmd_str.strip()}")
            else:
                if self.use_ros_logger:
                    self.logger.warn("[BaseController] Failed to write to serial port")
                else:
                    self.logger.warn("[BaseController] Failed to write to serial port")
            return bytes_written
        except serial.SerialException as e:
            if self.use_ros_logger:
                self.logger.error(f"Serial write error in write_command_direct: {e}\n{traceback.format_exc()}")
            else:
                self.logger.error(f"Serial write error in write_command_direct: {e}\n{traceback.format_exc()}")
            return 0
        except Exception as e:
            if self.use_ros_logger:
                self.logger.error(f"Unexpected error in write_command_direct: {e}\n{traceback.format_exc()}")
            else:
                self.logger.error(f"Unexpected error in write_command_direct: {e}\n{traceback.format_exc()}")
            return 0

    # Thread function to process and send commands from the queue
    def process_commands(self):
        if self.use_ros_logger:
            self.logger.info("[BaseController] process_commands thread is running and waiting for commands")
        else:
            self.logger.info("[BaseController] process_commands thread is running and waiting for commands")
        while True:
            try:
                data = self.command_queue.get()  # Get command from the queue (blocks until command available)
                # Format JSON exactly like ugv_driver.py does - match the exact format
                cmd_json = json.dumps(data) + '\n'
                cmd_bytes = cmd_json.encode("utf-8")
                bytes_written = self.ser.write(cmd_bytes)  # Send command as JSON over UART
                # CRITICAL: Flush immediately to ensure ESP32 receives command without delay
                # Without flush(), commands may be buffered in the OS serial buffer and not sent
                # immediately, causing delayed or no motor response. This was a critical bug fix.
                # DO NOT REMOVE THIS FLUSH() CALL - it is essential for real-time control.
                self.ser.flush()
                if bytes_written > 0:
                    if self.use_ros_logger:
                        self.logger.info(f"[BaseController] Sent {bytes_written} bytes to serial: {cmd_json.strip()}")
                    else:
                        self.logger.info(f"[BaseController] Sent {bytes_written} bytes to serial: {cmd_json.strip()}")
                else:
                    if self.use_ros_logger:
                        self.logger.warn(f"[BaseController] Failed to write command to serial port: {cmd_json.strip()}")
                    else:
                        self.logger.warning(f"[BaseController] Failed to write command to serial port: {cmd_json.strip()}")
            except serial.SerialTimeoutException as e:
                # Write timeout - command couldn't be sent in time
                if self.use_ros_logger:
                    self.logger.warn(f"Serial write timeout in process_commands: {e}")
                else:
                    self.logger.warning(f"Serial write timeout in process_commands: {e}")
            except serial.SerialException as e:
                # Other serial errors (port closed, device disconnected, etc.)
                if self.use_ros_logger:
                    self.logger.error(f"Serial write error in process_commands: {e}\n{traceback.format_exc()}")
                else:
                    self.logger.error(f"Serial write error in process_commands: {e}\n{traceback.format_exc()}")
            except Exception as e:
                # Unexpected errors
                if self.use_ros_logger:
                    self.logger.error(f"Unexpected error in process_commands: {e}\n{traceback.format_exc()}")
                else:
                    self.logger.error(f"Unexpected error in process_commands: {e}\n{traceback.format_exc()}")

    # Send control data as JSON via UART
    def base_json_ctrl(self, input_json):
        self.send_command(input_json)

# ROS node class for bringing up the UGV system and publishing sensor data
class ugv_bringup(Node):
    def __init__(self):
        super().__init__('ugv_bringup')
        # Publishers for IMU data, magnetic field data, odometry, and voltage
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 100)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 100)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 100)
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", 50)
        # Initialize the base controller with the UART port and baud rate
        # CRITICAL: Port and baud must match ESP32 firmware configuration
        # - Jetson Orin: Uses /dev/ttyTHS1 (hardware UART)
        # - Other platforms: Use /dev/ttyAMA0 (GPIO UART)
        # - Baud rate: 115200 is standard for Waveshare GPIO UART
        # Reference: https://github.com/waveshareteam/ugv_base_ros and Waveshare wiki
        import os
        # Detect Jetson platform (check for ugv_jetson marker or nvidia-tegra)
        is_jetson = any("ugv_jetson" in root for root, dirs, files in os.walk("/")) or \
                   os.path.exists('/etc/nv_tegra_release')
        default_port = DEFAULT_SERIAL_PORT_JETSON if is_jetson else DEFAULT_SERIAL_PORT_OTHER
        self.declare_parameter('serial_port', default_port)
        self.declare_parameter('serial_baud', DEFAULT_SERIAL_BAUD)
        
        # Get parameter values (these can be overridden via CLI or launch files)
        serial_port = self.get_parameter('serial_port').value
        serial_baud_param = self.get_parameter('serial_baud')
        # Handle both int and string types for baud rate (launch files may pass strings)
        # CRITICAL: Validate baud rate - only standard values should be used
        if isinstance(serial_baud_param.value, str):
            try:
                serial_baud = int(serial_baud_param.value)
            except (ValueError, TypeError):
                self.get_logger().error(
                    f"Invalid serial_baud value: {serial_baud_param.value}, "
                    f"using default {DEFAULT_SERIAL_BAUD}"
                )
                serial_baud = DEFAULT_SERIAL_BAUD
        else:
            serial_baud = int(serial_baud_param.value)
        
        # Validate baud rate is reasonable
        if serial_baud not in [9600, 19200, 38400, 57600, 115200, 230400]:
            self.get_logger().warn(
                f"Unusual baud rate: {serial_baud}. Expected one of: "
                "[9600, 19200, 38400, 57600, 115200, 230400]. "
                f"Using {serial_baud} anyway."
            )
        
        # Log which values are being used (important for debugging)
        self.get_logger().info(
            f"ugv_bringup: Using serial_port={serial_port}, serial_baud={serial_baud} "
            f"(from ROS parameters: serial_port='{serial_port}', serial_baud={serial_baud_param.value})"
        )
        
        try:
            # Pass ROS2 logger to BaseController so logs appear in ROS2 output
            self.base_controller = BaseController(serial_port, serial_baud, ros_logger=self.get_logger())
            self.get_logger().info(f"ugv_bringup: BaseController initialized successfully on {serial_port} at {serial_baud} baud")
            
            # Small delay to ensure serial port is fully stabilized after opening
            time.sleep(0.3)
        except Exception as e:
            self.get_logger().error(f"ugv_bringup: Failed to initialize BaseController on {serial_port}: {e}")
            raise
        # Subscribe to velocity commands (cmd_vel topic) - REQUIRED for rover movement
        self.cmd_vel_subscription_ = self.create_subscription(
            Twist, 
            "cmd_vel", 
            self.cmd_vel_callback, 
            10
        )
        self.get_logger().info("ugv_bringup: Subscribed to /cmd_vel topic")
        
        # Subscribe to LED control commands (ugv/led_ctrl topic)
        # IO4 controls light near OAK camera, IO5 controls light near USB camera
        # Values: 0-255 (0=off, 255=max brightness)
        self.led_ctrl_subscription_ = self.create_subscription(
            Float32MultiArray,
            "/ugv/led_ctrl",
            self.led_ctrl_callback,
            10
        )
        self.get_logger().info("ugv_bringup: Subscribed to /ugv/led_ctrl topic")
        
        # No initial stop command - matching ugv_driver.py behavior exactly
        # Hardware will respond to first cmd_vel command received
        self.last_cmd_vel = None
        # Timer to periodically execute the feedback loop
        # Note: 0.01s = 100Hz is more reasonable than 0.001s = 1000Hz
        # The serial port timeout is 1s, so 100Hz should be sufficient
        self.feedback_timer = self.create_timer(0.01, self.feedback_loop)
        self.feedback_loop_count = 0
        self.last_feedback_log_time = time.time()

    # Main loop for reading sensor feedback and publishing it to ROS topics
    def feedback_loop(self):
        self.feedback_loop_count += 1
        try:
            self.base_controller.feedback_data()
            # CRITICAL: Ensure base_data is a dictionary before accessing it
            # This prevents TypeError if base_data was accidentally overwritten
            if not isinstance(self.base_controller.base_data, dict):
                self.get_logger().error(
                    f"CRITICAL: base_data is not a dict! Type: {type(self.base_controller.base_data)}, "
                    f"Value: {self.base_controller.base_data}. Resetting to default.",
                    throttle_duration_sec=5.0
                )
                # Reset to default dictionary
                self.base_controller.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, 
                                                   "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, 
                                                   "odl": 0, "odr": 0, "v": 0}
                return  # Skip this iteration
            
            if self.base_controller.base_data.get("T", 0) == 1001:  # Check if the feedback type is correct
                self.publish_imu_data_raw()  # Publish IMU raw data
                self.publish_imu_mag()  # Publish magnetic field data
                self.publish_odom_raw()  # Publish odometry data
                self.publish_voltage()  # Publish voltage data
                # Log raw feedback data periodically to debug motor commands (every 500 iterations = ~5 seconds)
                if self.feedback_loop_count % 500 == 0:
                    data = self.base_controller.base_data
                    self.get_logger().info(
                        f"Feedback data (odl={data.get('odl', 0)}, odr={data.get('odr', 0)}, "
                        f"L={data.get('L', 0)}, R={data.get('R', 0)}, v={data.get('v', 0)})"
                    )
            else:
                # Log if feedback type is wrong (throttled to avoid spam)
                self.get_logger().warn(
                    f"Unexpected feedback type: {self.base_controller.base_data.get('T', 'unknown')}",
                    throttle_duration_sec=5.0
                )
        except Exception as e:
            self.get_logger().error(f"Error in feedback_loop:\n{traceback.format_exc()}")
        
        # Log feedback loop status periodically (every 5 seconds)
        current_time = time.time()
        if current_time - self.last_feedback_log_time >= 5.0:
            self.get_logger().info(
                f"Feedback loop running: {self.feedback_loop_count} iterations, "
                f"last feedback type: {self.base_controller.base_data.get('T', 'unknown')}"
            )
            self.last_feedback_log_time = current_time

    # Publish IMU data to the ROS topic "imu/data_raw"
    def publish_imu_data_raw(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Get the current timestamp
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        # Populate the linear acceleration and angular velocity fields
        # Use .get() with default 0 to handle missing keys gracefully
        msg.linear_acceleration.x = 9.8 * float(imu_raw_data.get("ax", 0)) / 8192
        msg.linear_acceleration.y = 9.8 * float(imu_raw_data.get("ay", 0)) / 8192
        msg.linear_acceleration.z = 9.8 * float(imu_raw_data.get("az", 0)) / 8192
        
        msg.angular_velocity.x = 3.1415926 * float(imu_raw_data.get("gx", 0)) / (16.4 * 180)
        msg.angular_velocity.y = 3.1415926 * float(imu_raw_data.get("gy", 0)) / (16.4 * 180)
        msg.angular_velocity.z = 3.1415926 * float(imu_raw_data.get("gz", 0)) / (16.4 * 180)
              
        self.imu_data_raw_publisher_.publish(msg)  # Publish the IMU data
        
    # Publish magnetic field data to the ROS topic "imu/mag"
    def publish_imu_mag(self):
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Get the current timestamp
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        # Populate the magnetic field data
        # Use .get() with default 0 to handle missing keys gracefully
        msg.magnetic_field.x = float(imu_raw_data.get("mx", 0)) * 0.15
        msg.magnetic_field.y = float(imu_raw_data.get("my", 0)) * 0.15
        msg.magnetic_field.z = float(imu_raw_data.get("mz", 0)) * 0.15
              
        self.imu_mag_publisher_.publish(msg)  # Publish the magnetic field data

    # Publish odometry data to the ROS topic "odom/odom_raw"
    def publish_odom_raw(self):
        odom_raw_data = self.base_controller.base_data
        # Use .get() with default 0 to handle missing keys gracefully
        array = [odom_raw_data.get("odl", 0)/100, odom_raw_data.get("odr", 0)/100]
        msg = Float32MultiArray(data=array)
        self.odom_publisher_.publish(msg)  # Publish the odometry data
        # Log odometry publishing periodically (throttled to avoid spam)
        if not hasattr(self, 'last_odom_log_time'):
            self.last_odom_log_time = time.time()
        current_time = time.time()
        if current_time - self.last_odom_log_time >= 2.0:
            self.get_logger().info(
                f"Published odom_raw: left={array[0]:.3f} m, right={array[1]:.3f} m"
            )
            self.last_odom_log_time = current_time

    # Publish voltage data to the ROS topic "voltage"
    def publish_voltage(self):
        voltage_data = self.base_controller.base_data
        msg = Float32()
        # Use .get() with default 0 to handle missing keys gracefully
        msg.data = float(voltage_data.get("v", 0))/100
        self.voltage_publisher_.publish(msg)  # Publish the voltage data
    
    # Callback for processing velocity commands from /cmd_vel topic
    def cmd_vel_callback(self, msg):
        """
        CRITICAL FUNCTION - Velocity Command Handler
        
        This callback is triggered when a message is published to /cmd_vel topic.
        If you don't see "✅ Received cmd_vel" logs, then either:
        1. No one is publishing to /cmd_vel topic, OR
        2. The subscription is broken
        
        This function converts ROS2 cmd_vel (geometry_msgs/Twist) commands into
        ESP32 wheel speed commands (T:1 format). The scaling and kinematics MUST
        be correct or the robot will not move or will move incorrectly.
        
        Process:
        1. Extract linear.x (forward velocity) and angular.z (rotational velocity)
        2. Convert to left/right wheel speeds using differential drive kinematics
        3. Scale from m/s to PWM range (-0.5 to +0.5) using MAX_ROBOT_SPEED_MPS
        4. Clamp to safe range
        5. Send as T:1 command to ESP32
        
        DO NOT MODIFY without understanding:
        - WHEELBASE_M must match base_node.cpp (0.175m)
        - MAX_ROBOT_SPEED_MPS must match Waveshare spec (1.3 m/s)
        - MAX_PWM_COMMAND is ESP32 firmware requirement (0.5)
        - Scaling formula is: (speed_mps / MAX_ROBOT_SPEED_MPS) * MAX_PWM_COMMAND
        """
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Validate inputs (sanity checks)
        if abs(linear_velocity) > 2.0:  # Reasonable limit
            self.get_logger().warn(
                f"Unusually high linear velocity: {linear_velocity:.3f} m/s (clamping)",
                throttle_duration_sec=5.0
            )
            linear_velocity = max(-2.0, min(2.0, linear_velocity))
        
        if abs(angular_velocity) > 5.0:  # Reasonable limit
            self.get_logger().warn(
                f"Unusually high angular velocity: {angular_velocity:.3f} rad/s (clamping)",
                throttle_duration_sec=5.0
            )
            angular_velocity = max(-5.0, min(5.0, angular_velocity))

        # Convert cmd_vel (linear/angular) to wheel speeds using differential drive kinematics
        # Formula: v_left = v - (L/2) * ω, v_right = v + (L/2) * ω
        # Where: v = linear velocity, L = wheelbase, ω = angular velocity
        # CRITICAL: WHEELBASE_M must match base_node.cpp for consistent odometry
        left_wheel_speed_ms = linear_velocity - (WHEELBASE_M / 2.0) * angular_velocity
        right_wheel_speed_ms = linear_velocity + (WHEELBASE_M / 2.0) * angular_velocity
        
        # CRITICAL SCALING: Convert m/s wheel speeds to ESP32 PWM command range
        # ESP32 expects values from -0.5 to +0.5 where 0.5 = 100% PWM = MAX_ROBOT_SPEED_MPS
        # Formula: command = (wheel_speed_mps / MAX_ROBOT_SPEED_MPS) * MAX_PWM_COMMAND
        # Example: 0.2 m/s → (0.2 / 1.3) * 0.5 = 0.077 (correct scaled value)
        # WITHOUT this scaling: 0.2 m/s → 0.2 (WRONG - robot won't move correctly)
        if MAX_ROBOT_SPEED_MPS > 0:
            left_wheel_speed = (left_wheel_speed_ms / MAX_ROBOT_SPEED_MPS) * MAX_PWM_COMMAND
            right_wheel_speed = (right_wheel_speed_ms / MAX_ROBOT_SPEED_MPS) * MAX_PWM_COMMAND
        else:
            self.get_logger().error(
                f"CRITICAL: Invalid MAX_ROBOT_SPEED_MPS={MAX_ROBOT_SPEED_MPS}! "
                "Cannot scale commands. Stopping motors.",
                throttle_duration_sec=1.0
            )
            left_wheel_speed = 0.0
            right_wheel_speed = 0.0
        
        # Clamp to safe PWM range: -0.5 to +0.5 (ESP32 firmware requirement)
        left_wheel_speed = max(MIN_PWM_COMMAND, min(MAX_PWM_COMMAND, left_wheel_speed))
        right_wheel_speed = max(MIN_PWM_COMMAND, min(MAX_PWM_COMMAND, right_wheel_speed))
        
        # DEBUG: Log intermediate values to verify scaling
        self.get_logger().debug(
            f"Scaling calculation: left_ms={left_wheel_speed_ms:.4f}, right_ms={right_wheel_speed_ms:.4f}, "
            f"max_speed={MAX_ROBOT_SPEED_MPS}, scaled_left={left_wheel_speed:.4f}, scaled_right={right_wheel_speed:.4f}",
            throttle_duration_sec=2.0
        )

        # Send the wheel speed data using T:1 format (direct wheel speed control)
        # CRITICAL: WHEEL_SPEED_COMMAND_TYPE must be 1 (wheel speed control)
        # T:1 is the recommended format per Waveshare WAVE_ROVER wiki
        # Format: {"T": 1, "L": <left_speed>, "R": <right_speed>}
        # Range: -0.5 to +0.5 where 0.5 = 100% PWM duty cycle
        try:
            # Validate command values before sending
            assert MIN_PWM_COMMAND <= left_wheel_speed <= MAX_PWM_COMMAND, \
                f"Left wheel speed {left_wheel_speed} out of range [{MIN_PWM_COMMAND}, {MAX_PWM_COMMAND}]"
            assert MIN_PWM_COMMAND <= right_wheel_speed <= MAX_PWM_COMMAND, \
                f"Right wheel speed {right_wheel_speed} out of range [{MIN_PWM_COMMAND}, {MAX_PWM_COMMAND}]"
            
            cmd_data = {
                'T': WHEEL_SPEED_COMMAND_TYPE,  # Use constant, not hardcoded 1
                'L': left_wheel_speed,
                'R': right_wheel_speed
            }
            # Use queued command method for reliable async sending
            self.base_controller.send_command(cmd_data)
            self.last_cmd_vel = cmd_data
            # Log at INFO level with throttling to verify commands are being received and sent
            # CRITICAL: This log confirms cmd_vel callback is working - if you don't see this, cmd_vel isn't being received
            self.get_logger().info(
                f"✅ Received cmd_vel: linear={linear_velocity:.3f} m/s, angular={angular_velocity:.3f} rad/s -> "
                f"Converted to T:1 command: L={left_wheel_speed:.3f}, R={right_wheel_speed:.3f}",
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f"Error sending cmd_vel via BaseController: {e}\n{traceback.format_exc()}", throttle_duration_sec=1.0)
    
    def led_ctrl_callback(self, msg):
        """
        LED Control Command Handler
        
        Controls camera lights: IO4 (OAK camera) and IO5 (USB camera)
        Values: 0-255 where 0=off, 255=max brightness
        """
        try:
            if len(msg.data) < 2:
                self.get_logger().warn(f"LED control message must have at least 2 values, got {len(msg.data)}")
                return
            
            IO4 = float(msg.data[0])  # Light near OAK camera
            IO5 = float(msg.data[1])  # Light near USB camera
            
            # Clamp values to valid range (0-255)
            IO4 = max(0.0, min(255.0, IO4))
            IO5 = max(0.0, min(255.0, IO5))
            
            # Format LED control command (T:132 is LED control command type)
            led_cmd = {
                'T': 132,
                'IO4': IO4,
                'IO5': IO5
            }
            
            # Send command via base controller
            self.base_controller.send_command(led_cmd)
            
            self.get_logger().info(
                f"💡 LED control: IO4={IO4:.0f} (OAK camera), IO5={IO5:.0f} (USB camera)"
            )
        except Exception as e:
            self.get_logger().error(f"Error processing LED control command: {e}\n{traceback.format_exc()}")
                        
# Main function to initialize the ROS node and start spinning
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS
    node = ugv_bringup()  # Create the UGV bringup node
    rclpy.spin(node)  # Keep the node running
    #node.destroy_node()  # (optional) Shutdown the node
    rclpy.shutdown()  # Shutdown ROS

if __name__ == '__main__':
    main()
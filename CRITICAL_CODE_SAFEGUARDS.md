# Critical Code Safeguards - UGV Rover Motor Control

## 🚨 DO NOT MODIFY WITHOUT UNDERSTANDING

This document describes **critical code sections** that must remain unchanged or be modified only with full understanding of their impact. These fixes were hard-won through extensive debugging and are essential for proper robot operation.

---

## 1. Speed Scaling in `cmd_vel_callback()` ⚠️ CRITICAL

**Location**: `src/ugv_main/ugv_bringup/ugv_bringup/ugv_bringup.py`, lines ~438-495

### What It Does
Converts ROS2 `cmd_vel` commands (in m/s) to ESP32 PWM commands (range -0.5 to +0.5).

### Why It's Critical
**Without proper scaling, the robot will NOT move correctly or at all.** 

The ESP32 firmware expects wheel speed commands in the range **-0.5 to +0.5**, where:
- `0.5` = 100% PWM = maximum robot speed (1.3 m/s)
- `0.0` = stop
- `-0.5` = 100% reverse

If you send `linear=0.2 m/s` directly as `L=0.2, R=0.2`, the ESP32 sees this as 40% of max speed, which is correct. But if you don't scale, you might send `L=0.2` thinking it's 0.2 m/s, when it should be `L=0.077` (scaled value).

### The Scaling Formula (DO NOT CHANGE)
```python
# Step 1: Differential drive kinematics
left_wheel_speed_ms = linear_velocity - (WHEELBASE_M / 2.0) * angular_velocity
right_wheel_speed_ms = linear_velocity + (WHEELBASE_M / 2.0) * angular_velocity

# Step 2: Scale from m/s to PWM range
left_wheel_speed = (left_wheel_speed_ms / MAX_ROBOT_SPEED_MPS) * MAX_PWM_COMMAND
right_wheel_speed = (right_wheel_speed_ms / MAX_ROBOT_SPEED_MPS) * MAX_PWM_COMMAND

# Example: 0.2 m/s → (0.2 / 1.3) * 0.5 = 0.077
```

### Constants (Defined at top of file)
```python
WHEELBASE_M = 0.175              # MUST match base_node.cpp line 201
MAX_ROBOT_SPEED_MPS = 1.3        # Waveshare specification
MAX_PWM_COMMAND = 0.5            # ESP32 firmware requirement
MIN_PWM_COMMAND = -0.5           # ESP32 firmware requirement
```

### What Happens If You Break It
- **No scaling**: Robot moves at wrong speed or not at all
- **Wrong wheelbase**: Odometry drifts, navigation fails
- **Wrong max speed**: Speed commands are too fast/slow
- **Wrong PWM range**: ESP32 ignores commands or interprets incorrectly

### How to Verify It's Working
Check logs for:
```
Received cmd_vel: linear=0.200 m/s -> Converted to T:1 command: L=0.077, R=0.077
```
**If you see `L=0.200, R=0.200` instead, scaling is broken!**

---

## 2. Serial Flush After Write ⚠️ CRITICAL

**Location**: `src/ugv_main/ugv_bringup/ugv_bringup/ugv_bringup.py`, line ~246

### What It Does
Calls `self.ser.flush()` immediately after every `serial.write()` to ensure commands are transmitted without delay.

### Why It's Critical
**Without flush(), commands can be buffered by the OS and not sent immediately**, causing:
- Delayed motor response
- Commands lost in buffer
- Robot not moving despite commands being "sent"

### The Code (DO NOT REMOVE)
```python
bytes_written = self.ser.write(cmd_bytes)
self.ser.flush()  # CRITICAL: Ensures immediate transmission
```

### What Happens If You Remove It
Commands may be buffered for 10-100ms or until buffer fills, causing:
- Laggy robot response
- Commands arriving out of order
- Robot not responding to urgent stop commands

### How to Verify It's Working
Check that commands appear in ESP32 immediately after being sent (check feedback timestamps).

---

## 3. Command Queue Initialization ⚠️ CRITICAL

**Location**: `src/ugv_main/ugv_bringup/ugv_bringup/ugv_bringup.py`, lines ~96-98

### What It Does
Initializes `self.command_queue` and starts `self.command_thread` in `BaseController.__init__()`.

### Why It's Critical
**If `send_command()` is called before the queue is initialized, you get `AttributeError`** and the node crashes.

### The Code (DO NOT REMOVE)
```python
self.command_queue = queue.Queue()
self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
self.command_thread.start()
```

### What Happens If You Break It
```python
AttributeError: 'BaseController' object has no attribute 'command_queue'
```
Node crashes immediately on startup.

---

## 4. Serial Port Ownership ⚠️ CRITICAL

**Location**: 
- `ugv_bringup.py`: Opens and owns serial port
- `ugv_driver.py`: Serial port access DISABLED

### What It Does
Only `ugv_bringup` opens the serial port. `ugv_driver` does NOT open a separate port to avoid conflicts.

### Why It's Critical
**Opening the same serial port twice causes "device busy" errors** and prevents communication.

### The Code
**In `ugv_bringup.py`**: Opens serial port normally
```python
self.base_controller = BaseController(serial_port, serial_baud, ...)
```

**In `ugv_driver.py`**: Serial port disabled
```python
ser = None  # NOT opened - ugv_bringup owns serial communication
```

### What Happens If You Break It
```python
serial.serialutil.SerialException: [Errno 16] Device or resource busy
```
One or both nodes fail to communicate with ESP32.

### How to Verify It's Working
Both nodes start without serial errors. Only `ugv_bringup` logs serial port opening.

---

## 5. Command Format T:1 (Not T:13) ⚠️ CRITICAL

**Location**: `src/ugv_main/ugv_bringup/ugv_bringup/ugv_bringup.py`, line ~484

### What It Does
Uses `T:1` format (wheel speed control) instead of `T:13` (velocity control).

### Why It's Critical
- **T:1** is the recommended format per Waveshare documentation
- **T:13** may not be supported by all ESP32 firmware versions
- T:1 gives direct control over left/right wheel speeds, which matches our scaling

### The Code
```python
cmd_data = {
    'T': WHEEL_SPEED_COMMAND_TYPE,  # = 1
    'L': left_wheel_speed,
    'R': right_wheel_speed
}
```

### What Happens If You Change It
- T:13 might work, but requires different scaling/units
- ESP32 firmware might ignore T:13 commands
- Robot won't move

### Constants
```python
WHEEL_SPEED_COMMAND_TYPE = 1  # DO NOT CHANGE without verifying firmware support
```

---

## 6. Jetson Port Detection ⚠️ IMPORTANT

**Location**: `src/ugv_main/ugv_bringup/ugv_bringup/ugv_bringup.py`, line ~294
**Location**: Launch files (all `bringup_*.launch.py`), line ~42

### What It Does
Auto-detects Jetson platform and uses `/dev/ttyTHS1` (Jetson) or `/dev/ttyAMA0` (others).

### Why It's Critical
**Wrong port = no communication with ESP32 = robot doesn't work.**

### The Code
```python
is_jetson = any("ugv_jetson" in root for root, dirs, files in os.walk("/")) or \
           os.path.exists('/etc/nv_tegra_release')
default_port = DEFAULT_SERIAL_PORT_JETSON if is_jetson else DEFAULT_SERIAL_PORT_OTHER
```

### Constants
```python
DEFAULT_SERIAL_PORT_JETSON = '/dev/ttyTHS1'
DEFAULT_SERIAL_PORT_OTHER = '/dev/ttyAMA0'
```

### What Happens If You Break It
- Wrong port selected → serial open fails
- Robot cannot communicate with ESP32
- No motor control, no sensor feedback

### How to Override
```bash
ros2 launch ugv_bringup bringup_lidar.launch.py serial_port:=/dev/ttyTHS1
```

---

## 7. Baud Rate Configuration ⚠️ IMPORTANT

**Location**: Throughout `ugv_bringup.py` and launch files

### What It Does
Uses 115200 baud for serial communication (Waveshare GPIO UART standard).

### Why It's Critical
**Mismatched baud rate = garbled data = robot doesn't work.**

### Constants
```python
DEFAULT_SERIAL_BAUD = 115200  # Standard for Waveshare GPIO UART
```

### What Happens If You Change It
- ESP32 expects 115200, you send at different rate → communication fails
- No commands received, no feedback received
- Robot appears "dead"

---

## Testing Checklist

Before modifying any of the above, verify current behavior:

1. **Speed Scaling**: 
   - Send `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`
   - Check logs show `L=0.077, R=0.077` (not `L=0.200`)

2. **Serial Flush**:
   - Commands should appear in ESP32 immediately (check feedback timestamps)

3. **Serial Port**:
   - Only `ugv_bringup` logs "Serial port opened"
   - No "device busy" errors

4. **Command Format**:
   - Check logs show `{"T": 1, "L": ..., "R": ...}`
   - Robot moves correctly

5. **Port Detection**:
   - Jetson: Should use `/dev/ttyTHS1`
   - Others: Should use `/dev/ttyAMA0`
   - Logs show correct port at startup

---

## Modification Guidelines

### ✅ SAFE to Modify
- Logging levels and messages
- Error handling messages
- Comment text (as long as it's accurate)
- Code formatting/style

### ⚠️ MODIFY WITH CAUTION
- Adding new features (test thoroughly)
- Changing default parameter values (document why)
- Optimizing code (ensure behavior unchanged)

### 🚨 DO NOT MODIFY
- Speed scaling formula
- Serial flush calls
- Command queue initialization
- Serial port ownership (ugv_bringup owns it)
- Command format (T:1)
- Critical constants (wheelbase, max speed, PWM range)

---

## If You Must Modify

1. **Understand the impact**: Read this document and related code
2. **Test thoroughly**: Verify robot still moves correctly
3. **Check odometry**: Ensure odometry still accurate
4. **Update this doc**: Document what changed and why
5. **Test edge cases**: Zero velocity, maximum velocity, rotation-only, etc.

---

## Related Files

- `src/ugv_main/ugv_base_node/src/base_node.cpp`: Uses WHEELBASE_M (0.175) for odometry
- `src/ugv_main/ugv_bringup/ugv_bringup/ugv_driver.py`: Serial port disabled (by design)
- Launch files: Port detection logic

---

**Last Updated**: After successful rover movement verification
**Status**: ✅ All safeguards in place and tested

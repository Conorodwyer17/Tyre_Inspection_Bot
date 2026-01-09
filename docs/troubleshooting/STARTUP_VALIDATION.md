# Startup Validation Guide

## Overview

The system includes comprehensive startup validation to ensure all components are ready before mission start. This prevents mission failures due to missing dependencies or misconfiguration.

## Startup Validator

**Location:** `tyre_inspection_mission/startup_validator.py`

### Validation Checks

1. **Configuration Validation**
   - All configuration parameters
   - Parameter ranges and relationships
   - Frame configuration
   - Timeout values

2. **System Health Check**
   - TF frame availability
   - Topic publishers
   - Service availability
   - Nav2 health

3. **Topic Availability**
   - Required topics exist
   - Publishers are active
   - Topics match expected names

4. **Service Availability**
   - Required services exist
   - Services are advertised
   - Services match expected names

5. **Nav2 Availability**
   - Action server is ready
   - Action server responds
   - Navigation stack is operational

6. **TF Frames**
   - Required frames exist
   - Frame tree is connected
   - Transforms are available

---

## Usage

### Automatic Validation

Startup validation runs automatically when mission controller initializes (if integrated).

### Manual Validation

```python
from tyre_inspection_mission.startup_validator import validate_startup
from tyre_inspection_mission.structured_logger import StructuredLogger

# In node initialization
logger = StructuredLogger(self.get_logger())
is_valid = validate_startup(self, logger, timeout=10.0)

if not is_valid:
    self.get_logger().error("Startup validation failed. Mission cannot start.")
    return
```

---

## Validation Output

### Example: All Checks Pass

```
[INFO] [SYS] Starting comprehensive startup validation...
[INFO] [SYS] Validating configuration...
[INFO] [SYS] Configuration validation passed
[INFO] [SYS] Checking system health...
[INFO] [SYS] Checking topic availability...
[INFO] [SYS] Checking service availability...
[INFO] [SYS] Checking Nav2 availability...
[INFO] [SYS] Checking TF frames...
[INFO] [SYS] Startup Validation: ✅ VALID
[INFO] [SYS] Validation checks performed:
[INFO] [SYS]   ✅ configuration
[INFO] [SYS]   ✅ system_health
[INFO] [SYS]   ✅ topics
[INFO] [SYS]   ✅ services
[INFO] [SYS]   ✅ nav2
[INFO] [SYS]   ✅ tf_frames
```

### Example: Validation Failed

```
[INFO] [SYS] Starting comprehensive startup validation...
[ERROR] [ERR] Configuration validation failed with 1 error(s)
[ERROR] [ERR]   ❌ approach_distance (0.4m) is less than minimum safe distance (0.6m)
[ERROR] [ERR] Startup Validation: ❌ INVALID (1 errors)
[ERROR] [ERR] Mission cannot start. Fix errors and retry.
[ERROR] [ERR] Validation checks performed:
[ERROR] [ERR]   ❌ configuration
```

### Example: Validation with Warnings

```
[INFO] [SYS] Startup Validation: ✅ VALID (2 warnings)
[WARN] [SYS] Startup validation has 2 warning(s):
[WARN] [SYS]   ⚠️  Using 'map' frame. Ensure SLAM is running.
[WARN] [SYS]   ⚠️  System diagnostics not available. Skipping health check.
```

---

## Required Components

### Topics
- `/darknet_ros_3d/bounding_boxes` - 3D bounding boxes
- `/scan` - LiDAR scan
- `/oak/points` - Point cloud
- `/oak/rgb/image_rect` - Camera image

### Services
- `/photo_capture/capture` - Photo capture service
- `/mission_controller/start` - Mission start service
- `/mission_controller/stop` - Mission stop service

### Action Servers
- `/navigate_to_pose` - Nav2 navigation action

### TF Frames
- `base_footprint` - Robot base frame
- `map` or `odom` - Navigation frame

---

## Validation Timeout

Default timeout: **10 seconds**

This allows time for:
- Topics to be discovered
- Services to be advertised
- TF frames to become available
- Nav2 to initialize

**Note:** Some checks may take longer. Increase timeout if needed.

---

## Best Practices

1. **Always Validate** - Run validation before starting mission
2. **Fix Errors First** - Never start with validation errors
3. **Review Warnings** - Address warnings for optimal performance
4. **Check Logs** - Review validation output for details
5. **Increase Timeout** - If components need more time to start

---

## Common Issues

### Topics Not Available

**Error:** "Required topics not available"

**Fix:**
- Check YOLO node is running
- Check segmentation processor is running
- Check camera node is running
- Check LiDAR node is running
- Verify topic names match

### Services Not Available

**Error:** "Required services not available"

**Fix:**
- Check mission_controller node is running
- Check photo_capture node is running
- Verify service names match
- Wait for services to advertise

### Nav2 Not Available

**Error:** "Nav2 action server not ready"

**Fix:**
- Check Nav2 is launched
- Wait for Nav2 to initialize
- Check Nav2 logs for errors
- Verify action server name

### TF Frames Not Available

**Error:** "Required TF frames not available"

**Fix:**
- Check robot description is loaded
- Check SLAM is running (if using 'map' frame)
- Check odometry is running
- Wait for frames to become available

---

## Integration

### With Mission Controller

```python
# In MissionController.__init__
from tyre_inspection_mission.startup_validator import validate_startup

# After all subscriptions/services are created
if not validate_startup(self, self.slogger, timeout=10.0):
    self.get_logger().error("Startup validation failed. Mission cannot start.")
    # Optionally: set state to ERROR_RECOVERY or exit
```

### With Launch File

Validation runs automatically when mission controller starts.

---

## Validation Checklist

Before starting mission, ensure:

- [ ] Configuration validation passed
- [ ] System health check passed
- [ ] All required topics available
- [ ] All required services available
- [ ] Nav2 action server ready
- [ ] Required TF frames available
- [ ] No critical errors
- [ ] Warnings reviewed (if any)

---

## Troubleshooting

### Validation Takes Too Long

**Cause:** Components slow to start

**Fix:**
- Increase validation timeout
- Check component startup times
- Review component logs

### Validation Fails Intermittently

**Cause:** Race condition during startup

**Fix:**
- Increase validation timeout
- Add delays between component starts
- Check component dependencies

### Validation Passes But Mission Fails

**Cause:** Components fail after validation

**Fix:**
- Check component health during mission
- Review mission monitor issues
- Check mission logs

---

**Last Updated:** Current Session  
**Version:** 1.0

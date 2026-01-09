# Configuration Validation

## Overview

The system now includes comprehensive configuration validation at startup to catch configuration issues early before they cause problems during mission execution.

## Config Validator

**Location:** `tyre_inspection_mission/config_validator.py`

### Features

1. **Startup Validation**
   - Validates all configuration parameters
   - Checks parameter ranges and relationships
   - Provides clear error messages
   - Warns about suboptimal configurations

2. **Validation Categories**
   - Navigation configuration
   - Detection configuration
   - Camera configuration
   - Capture configuration
   - Frame configuration
   - System health configuration

### Validation Rules

#### Navigation Configuration

**Checks:**
- `approach_distance` >= minimum safe distance
- `navigation_timeout` within reasonable range (10s - 300s)
- Relationship between approach distance and safe distance

**Errors:**
- Approach distance too small (will cause goal rejections)
- Navigation timeout too short

**Warnings:**
- Approach distance close to minimum
- Navigation timeout very long

#### Detection Configuration

**Checks:**
- `detection_timeout` within reasonable range (5s - 300s)
- `vehicle_class_names` not empty
- Reasonable number of vehicle classes

**Errors:**
- Detection timeout too short
- Empty vehicle class names

**Warnings:**
- Detection timeout very long
- Too many vehicle classes (may slow detection)

#### Camera Configuration

**Checks:**
- Image dimensions valid
- Focal lengths calibrated
- Centering tolerance reasonable

**Errors:**
- Invalid image dimensions

**Warnings:**
- Focal lengths not calibrated
- Centering tolerance too large

#### Capture Configuration

**Checks:**
- `photo_capture_timeout` within reasonable range (1s - 60s)
- `MAX_CENTERING_ATTEMPTS` reasonable (1 - 20)

**Errors:**
- Capture timeout too short
- Max centering attempts invalid

**Warnings:**
- Capture timeout very long
- Max centering attempts very high

#### Frame Configuration

**Checks:**
- `navigation_frame` is 'map' or 'odom'
- Frame selection appropriate for system

**Errors:**
- Invalid navigation frame

**Warnings:**
- Using 'map' frame (requires SLAM)

---

## Usage

### Automatic Validation

Configuration validation runs automatically when the mission controller starts if integrated.

### Manual Validation

```python
from tyre_inspection_mission.config_validator import validate_configuration
from tyre_inspection_mission.structured_logger import StructuredLogger

# In node initialization
logger = StructuredLogger(self.get_logger())
is_valid = validate_configuration(self, logger)

if not is_valid:
    self.get_logger().error("Configuration validation failed. Fix errors before starting mission.")
    return
```

---

## Validation Output

### Example: Valid Configuration
```
[INFO] [SYS] Configuration validation passed
```

### Example: Valid with Warnings
```
[WARN] [SYS] Configuration validation passed with 2 warning(s)
[WARN] [SYS]   ⚠️  approach_distance (0.8m) is close to minimum safe distance. Consider increasing to at least 0.72m.
[WARN] [SYS]   ⚠️  Using 'map' frame. Ensure SLAM is running and map frame is available.
```

### Example: Invalid Configuration
```
[ERROR] [ERR] Configuration validation failed with 1 error(s)
[ERROR] [ERR]   ❌ approach_distance (0.4m) is less than minimum safe distance (0.6m). Navigation goals will be rejected.
```

---

## Best Practices

1. **Fix Errors Before Starting** - Never start mission with configuration errors
2. **Review Warnings** - Address warnings for optimal performance
3. **Test After Changes** - Re-validate after changing configuration
4. **Document Custom Configs** - Note any non-standard configurations

---

## Common Issues

### Goal Rejections
**Cause:** `approach_distance` too small
**Fix:** Increase `approach_distance` to at least `MIN_SAFE_DISTANCE * 1.2`

### Slow Detection
**Cause:** Too many vehicle classes or very long timeout
**Fix:** Reduce vehicle classes or timeout

### Navigation Failures
**Cause:** Using 'map' frame without SLAM
**Fix:** Use 'odom' frame or ensure SLAM is running

---

**Last Updated:** Current Session  
**Version:** 1.0

# Fixes Applied: Image Capture System

## Issues Identified

1. **Topic Mismatch**: 
   - Camera publishes to `/image_raw`
   - Image capture node was subscribed to `/camera/image_raw`
   - **Fixed**: Updated launch file to use `/image_raw`

2. **Service Not Available**:
   - Inspection manager node not running
   - Service `/inspection/capture_image` not available
   - **Solution**: Need to start inspection_manager launch file

## Changes Made

### 1. Updated Launch File

**File**: `src/amr_simulation/src/inspection_manager/launch/inspection_manager.launch.py`

**Change**: Updated `image_topic` parameter from `/camera/image_raw` to `/image_raw`

```python
parameters=[{
    'image_topic': '/image_raw',  # USB camera publishes to /image_raw
    # ...
}]
```

### 2. Rebuild Required

After changing the launch file parameter, rebuild the package:

```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select inspection_manager
source install/setup.bash
```

## Testing Steps

### 1. Start Camera (Terminal 1)
```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ugv_vision camera.launch.py
```

### 2. Start Inspection Manager (Terminal 2)
```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch inspection_manager inspection_manager.launch.py
```

### 3. Verify Service is Available
```bash
ros2 service list | grep capture_image
# Should see: /inspection/capture_image
```

### 4. Test Image Capture
```bash
ros2 service call /inspection/capture_image std_srvs/srv/Trigger

# Expected response:
# response:
# std_srvs.srv.Trigger_Response(success=True, message='Image saved successfully: ...')
```

### 5. Verify Images Saved
```bash
ls -la ~/ugv_ws/images/inspections/
```

## Camera Topic Information

**USB Camera (usb_cam package)**:
- Publishes to: `/image_raw`
- Also publishes: `/image_rect` (rectified)

**OAK-D Camera**:
- Typically publishes to: `/oak_d_lite/image_raw` or similar
- May need different topic parameter

**Topic Discovery**:
```bash
# List all image topics
ros2 topic list | grep image

# Check what camera publishes
ros2 node info /usb_cam | grep Publishers

# Echo to verify
ros2 topic echo /image_raw --once
```

## Next Steps

1. ✅ Fixed topic mismatch in launch file
2. ⏳ Rebuild inspection_manager package
3. ⏳ Start inspection_manager node
4. ⏳ Test image capture service
5. ⏳ Verify images are saved correctly

---

**Status**: ✅ Topic mismatch fixed
**Action Required**: Rebuild and restart inspection_manager node

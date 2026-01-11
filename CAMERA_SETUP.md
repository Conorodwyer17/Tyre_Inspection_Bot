# Camera Setup Guide for Image Capture System

## System Status: ✅ Image Capture System Ready

The image capture system is **fully operational** and waiting for camera input. The warning "No image available for capture" is **expected** until the camera node is started.

## Camera Configuration

Based on the workspace, the system supports multiple camera types:

### 1. USB Camera (usb_cam package)

**Launch file:** `src/amr_simulation/src/ugv_vision/launch/camera.launch.py`

**To start:**
```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start USB camera
ros2 launch ugv_vision camera.launch.py
```

**Expected topic:** `/image_raw` (may need to remap to `/camera/image_raw`)

**Check if camera is publishing:**
```bash
ros2 topic list | grep image
ros2 topic hz /image_raw
```

### 2. OAK-D Camera

**Launch file:** `src/amr_simulation/src/ugv_vision/launch/oak_d_lite.launch.py`

**To start:**
```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start OAK-D camera
ros2 launch ugv_vision oak_d_lite.launch.py
```

**Expected topics:** 
- `/oak_d_lite/image_raw` (may need remapping)
- `/camera/image_raw` (if remapped)

### 3. Camera Topic Remapping

If your camera publishes to a different topic (e.g., `/image_raw` instead of `/camera/image_raw`), you can:

**Option A: Remap in launch file**
```python
Node(
    package="inspection_manager",
    executable="image_capture_node",
    name="image_capture_node",
    parameters=[{
        "image_topic": "/image_raw",  # Change to your camera topic
        # ...
    }]
)
```

**Option B: Use topic remapping**
```bash
ros2 launch inspection_manager inspection_manager.launch.py \
    image_topic:=/image_raw
```

**Option C: Check actual camera topic**
```bash
# List all image topics
ros2 topic list | grep -i image

# Echo to verify
ros2 topic echo /image_raw --once
```

## Complete System Launch Sequence

### Terminal 1: Camera
```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start camera (choose one):
ros2 launch ugv_vision camera.launch.py        # USB camera
# OR
ros2 launch ugv_vision oak_d_lite.launch.py    # OAK-D camera
```

### Terminal 2: Navigation (if needed)
```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ugv_nav nav_bringup/nav2_bringup.launch.py
```

### Terminal 3: Segmentation
```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch segmentation_3d segment_3d.launch.py
```

### Terminal 4: Inspection Manager (includes Image Capture)
```bash
cd /home/jetson/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch inspection_manager inspection_manager.launch.py
```

## Verification Steps

### 1. Check Camera is Publishing

```bash
# List camera topics
ros2 topic list | grep -i image

# Check topic frequency
ros2 topic hz /camera/image_raw

# View one message (to verify)
ros2 topic echo /camera/image_raw --once
```

### 2. Test Image Capture Service

Once camera is publishing:

```bash
# Test service
ros2 service call /inspection/capture_image std_srvs/srv/Trigger

# Expected output:
# response:
# std_srvs.srv.Trigger_Response(success=True, message='Image saved successfully: ...')
```

### 3. Verify Images are Saved

```bash
# Check images directory
ls -la ~/ugv_ws/images/inspections/

# Should see vehicle directories with images after capture
```

## Troubleshooting

### Issue: Camera topic doesn't match

**Symptoms:** Image capture node subscribed but no images received

**Solution:**
1. Check actual camera topic:
   ```bash
   ros2 topic list | grep image
   ```
2. Update image capture node parameter:
   ```bash
   ros2 launch inspection_manager inspection_manager.launch.py \
       image_topic:=/your_camera_topic
   ```

### Issue: Camera node not starting

**Symptoms:** Camera launch fails or no topics appear

**Solution:**
1. Check camera hardware connection:
   ```bash
   ls /dev/video*
   ```
2. Check camera package installed:
   ```bash
   ros2 pkg list | grep -E "(usb_cam|v4l2|oak)"
   ```
3. Check camera node logs for errors

### Issue: Service call succeeds but no image saved

**Symptoms:** Service returns success but no file created

**Solution:**
1. Check storage directory permissions:
   ```bash
   ls -la ~/ugv_ws/images/inspections/
   chmod 755 ~/ugv_ws/images/inspections/
   ```
2. Check disk space:
   ```bash
   df -h ~/ugv_ws/images/
   ```
3. Check node logs:
   ```bash
   ros2 topic echo /rosout | grep image_capture
   ```

## Current System Status

From your test output:
```
✅ inspection_manager_node: Running (pid 44074)
✅ image_capture_node: Running (pid 44076)
✅ Service /inspection/capture_image: Available
✅ Subscribed to /camera/image_raw: Ready
✅ Subscribed to /inspection_state: Ready
⚠️  Camera not publishing: Expected (camera node not started)
```

**Next Step:** Start camera node to enable image capture

---

**System Status:** ✅ Ready - Waiting for camera input
**Implementation:** ✅ Complete
**Action Required:** Start camera node to test image capture

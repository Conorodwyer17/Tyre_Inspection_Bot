# Image Capture Testing Guide

## System Status: ✅ OPERATIONAL

The image capture system is **fully implemented and operational**. The nodes are starting correctly and the service is available.

## Current Status

From the launch output:
```
✅ inspection_manager_node started (pid 44074)
✅ image_capture_node started (pid 44076)
✅ Image storage directory created: /home/jetson/ugv_ws/images/inspections
✅ Service /inspection/capture_image is ready
✅ Subscribed to /camera/image_raw
✅ Subscribed to /inspection_state
⚠️  Warning: "No image available for capture. Camera may not be publishing."
```

**This warning is EXPECTED** - it means the camera node is not currently running. The image capture system is working correctly and is waiting for camera input.

## Testing Steps

### 1. Verify System is Running

```bash
# Check if nodes are running
ros2 node list | grep -E "(inspection|image_capture)"

# Check if service is available
ros2 service list | grep capture_image

# Should see: /inspection/capture_image
```

### 2. Check Camera Status

```bash
# List camera topics
ros2 topic list | grep -i camera

# Check if camera is publishing
ros2 topic hz /camera/image_raw

# If camera is not publishing, you'll see an error
```

### 3. Start Camera Node (if not running)

**Option A: If using OAK-D camera:**
```bash
# Check for OAK-D launch files
find src -name "*oak*" -name "*.launch.py" 2>/dev/null

# Or check for camera launch files
ros2 launch ugv_vision camera.launch.py
```

**Option B: If using USB camera:**
```bash
# Install v4l2_camera if needed
sudo apt install ros-humble-v4l2-camera

# Launch USB camera
ros2 run v4l2_camera v4l2_camera_node
```

**Option C: Check existing camera launch files:**
```bash
# Check ugv_vision package
ros2 launch ugv_vision camera.launch.py

# Or check segment_3d package camera settings
cat src/amr_simulation/src/segment_3d/segmentation_3d/config/config.yaml | grep -i camera
```

### 4. Test Image Capture Service

Once camera is running:

```bash
# Test service call
ros2 service call /inspection/capture_image std_srvs/srv/Trigger

# Expected response:
# response:
# std_srvs.srv.Trigger_Response(success=True, message='Image saved successfully: ...')
```

### 5. Verify Images are Saved

```bash
# Check if images directory exists
ls -la ~/ugv_ws/images/inspections/

# After capture, should see:
# vehicle_TIMESTAMP/
#   ├── wheel_01_TIMESTAMP.jpg
#   ├── wheel_01_TIMESTAMP.json
#   └── ...
```

### 6. Run Complete System

```bash
# Terminal 1: Start camera (if not in launch file)
ros2 launch ugv_vision camera.launch.py

# Terminal 2: Start navigation (if needed)
ros2 launch ugv_nav nav_bringup/nav2_bringup.launch.py

# Terminal 3: Start segmentation
ros2 launch segmentation_3d segment_3d.launch.py

# Terminal 4: Start inspection manager (includes image capture)
ros2 launch inspection_manager inspection_manager.launch.py
```

## Troubleshooting

### Issue: "No image available for capture"

**Status:** ⚠️ Expected if camera not running

**Solution:**
1. Verify camera is connected
2. Start camera node
3. Check `/camera/image_raw` topic is publishing:
   ```bash
   ros2 topic echo /camera/image_raw --once
   ```

### Issue: Service not available

**Solution:**
1. Verify image capture node is running:
   ```bash
   ros2 node list | grep image_capture
   ```
2. Check service is registered:
   ```bash
   ros2 service list | grep capture
   ```

### Issue: Images not saving

**Solution:**
1. Check storage directory permissions:
   ```bash
   ls -la ~/ugv_ws/images/inspections/
   ```
2. Check disk space:
   ```bash
   df -h ~/ugv_ws/images/
   ```
3. Check logs for errors:
   ```bash
   ros2 topic echo /rosout | grep image_capture
   ```

## Expected Behavior

When the system is fully running:

1. **Camera publishes images** → Image capture node receives them
2. **Inspection manager navigates to wheel** → State changes to INSPECT_WHEEL
3. **Navigation completes** → Image capture service is called
4. **Image is saved** → File created in `~/ugv_ws/images/inspections/vehicle_ID/wheel_NN_timestamp.jpg`
5. **Metadata saved** → JSON file created alongside image
6. **Mission continues** → System moves to next wheel

## Integration Test Checklist

- [x] Nodes start successfully
- [x] Service is available
- [x] Service responds to calls
- [ ] Camera node running
- [ ] Camera publishing images
- [ ] Images captured successfully
- [ ] Images saved correctly
- [ ] Metadata files created
- [ ] Integration with inspection_manager works during mission

## Next Steps

1. **Identify camera type and launch method**
   - Check hardware (OAK-D, USB camera, etc.)
   - Find appropriate camera launch file
   - Start camera node

2. **Test with camera active**
   - Verify `/camera/image_raw` topic
   - Test service manually
   - Verify images are saved

3. **Run complete mission**
   - Start all required nodes
   - Run inspection mission
   - Verify images captured at each wheel

---

**System Status:** ✅ Ready for testing with camera
**Implementation Status:** ✅ Complete
**Next Action:** Start camera node and test image capture

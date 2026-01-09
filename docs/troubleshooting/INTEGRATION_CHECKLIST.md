# Integration Checklist - Complete System Setup

## ✅ Pre-Mission Checklist

Use this checklist before every mission to ensure system is ready.

---

## 🔧 System Setup

### 1. Launch System
- [ ] Launch autonomous inspection system
- [ ] All nodes start successfully
- [ ] No critical errors in launch logs

### 2. Run Diagnostic Check
```bash
ros2 run tyre_inspection_mission diagnostic_check
```
- [ ] Exit code is 0 (healthy) or 2 (warning)
- [ ] All required topics available
- [ ] All required services available
- [ ] Nav2 action server ready
- [ ] Required TF frames available

### 3. Start Monitoring Tools
- [ ] Mission monitor running
- [ ] Mission log recorder running
- [ ] Mission status viewer running (optional)

---

## ⚙️ Configuration Validation

### Automatic Validation
- [ ] Configuration validation passed
- [ ] No configuration errors
- [ ] Warnings reviewed (if any)

### Manual Checks
- [ ] `approach_distance` >= 0.6m
- [ ] `navigation_timeout` reasonable (10s - 300s)
- [ ] `detection_timeout` reasonable (5s - 300s)
- [ ] `navigation_frame` is 'map' or 'odom'
- [ ] Vehicle class names configured

---

## 🎯 Component Verification

### Detection Pipeline
- [ ] YOLO node running
- [ ] Segmentation processor running
- [ ] Camera publishing images
- [ ] Point cloud available
- [ ] Bounding boxes topic active

### Navigation System
- [ ] Nav2 launched
- [ ] Action server ready
- [ ] SLAM running (if using 'map' frame)
- [ ] Odometry running
- [ ] Costmap updated

### Capture System
- [ ] Photo capture service available
- [ ] Camera initialized
- [ ] Storage directory writable
- [ ] Disk space available

### TF System
- [ ] Robot description loaded
- [ ] Base frame available
- [ ] Navigation frame available
- [ ] Camera frame available
- [ ] TF tree connected

---

## 📊 Health Checks

### System Health
- [ ] System health status: healthy or warning
- [ ] No critical health issues
- [ ] TF frames available
- [ ] Topics publishing
- [ ] Services available

### Mission Readiness
- [ ] Mission controller ready
- [ ] All dependencies satisfied
- [ ] No blocking errors
- [ ] Monitoring tools ready

---

## 🚀 Mission Start

### Final Checks
- [ ] All checklist items completed
- [ ] No critical errors
- [ ] Monitoring tools active
- [ ] Log recorder running

### Start Mission
```bash
ros2 service call /mission_controller/start std_srvs/srv/Trigger
```

### Monitor During Mission
- [ ] Watch mission status
- [ ] Monitor for issues
- [ ] Check health status
- [ ] Review detected issues

---

## 📝 Post-Mission

### Analysis
- [ ] Mission log recorded
- [ ] Analyze mission log
- [ ] Review errors and issues
- [ ] Check performance metrics

### Documentation
- [ ] Note any issues encountered
- [ ] Document solutions applied
- [ ] Update configuration if needed
- [ ] Archive mission log

---

## 🔍 Quick Verification Commands

### Check Topics
```bash
ros2 topic list | grep -E "bounding_boxes|scan|oak|mission"
```

### Check Services
```bash
ros2 service list | grep -E "mission_controller|photo_capture"
```

### Check Nav2
```bash
ros2 action list | grep navigate_to_pose
```

### Check TF Frames
```bash
ros2 run tf2_ros tf2_echo base_footprint map
```

### Check System Health
```bash
ros2 run tyre_inspection_mission diagnostic_check
```

---

## ⚠️ Common Issues

### Topics Not Available
**Fix:** Check YOLO, segmentation, camera, LiDAR nodes

### Services Not Available
**Fix:** Check mission_controller, photo_capture nodes

### Nav2 Not Ready
**Fix:** Wait for Nav2 initialization, check Nav2 logs

### TF Frames Missing
**Fix:** Check robot description, SLAM, odometry nodes

### Configuration Errors
**Fix:** Review configuration validation output, adjust parameters

---

## 📖 Related Documentation

- [STARTUP_VALIDATION.md](STARTUP_VALIDATION.md) - Detailed validation
- [CONFIGURATION_VALIDATION.md](CONFIGURATION_VALIDATION.md) - Config validation
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Quick fixes
- [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md) - Comprehensive guide

---

**Last Updated:** Current Session  
**Version:** 1.0

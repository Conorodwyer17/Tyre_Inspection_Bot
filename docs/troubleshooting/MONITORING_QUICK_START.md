# Monitoring Quick Start Guide

## 🚀 Quick Setup

### 1. Launch Mission System
```bash
ros2 launch tyre_inspection_mission autonomous_inspection.launch.py
```

### 2. Start Mission Monitor (Terminal 2)
```bash
ros2 run tyre_inspection_mission mission_monitor
```

### 3. View Status (Terminal 3)
```bash
ros2 run tyre_inspection_mission mission_status
```

### 4. Start Mission
```bash
ros2 service call /mission_controller/start std_srvs/srv/Trigger
```

---

## 📊 What You'll See

### Mission Status Viewer
- Current state
- Progress metrics
- Health status
- Detected issues

### Monitor Topics
- `/mission_monitor/status` - Full status
- `/mission_monitor/issues` - Issues and recommendations

---

## 🔍 Quick Checks

### Check Mission Status
```bash
ros2 topic echo /mission_controller/status --once
```

### Check Monitor Status
```bash
ros2 topic echo /mission_monitor/status --once
```

### Check Issues
```bash
ros2 topic echo /mission_monitor/issues --once
```

### Run Diagnostic Check
```bash
ros2 run tyre_inspection_mission diagnostic_check
```

---

## ⚠️ Common Issues

### No Status Updates
- Check mission controller is running
- Check topics are publishing
- Restart monitor if needed

### Issues Not Detected
- Monitor must be running
- Check issue thresholds
- Review monitor logs

---

## 📖 Full Documentation

- **Runtime Monitoring:** [RUNTIME_MONITORING.md](RUNTIME_MONITORING.md)
- **Troubleshooting:** [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md)
- **Quick Reference:** [QUICK_REFERENCE.md](QUICK_REFERENCE.md)

---

**Quick Start Version:** 1.0

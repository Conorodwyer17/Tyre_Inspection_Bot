# Runtime Monitoring and Issue Detection

## Overview

The mission system now includes comprehensive runtime monitoring that automatically detects issues, tracks progress, and provides adaptive recommendations.

## Mission Monitor

**Location:** `tyre_inspection_mission/mission_monitor.py`

### Features

1. **Progress Tracking**
   - Real-time mission progress
   - Vehicle and tyre statistics
   - Completion rate calculation
   - State transition history
   - Time spent in each state

2. **Issue Detection**
   - Stuck state detection
   - High error rate detection
   - No progress detection
   - State loop detection
   - Adaptive recommendations

3. **Health Monitoring**
   - Overall mission health status
   - Issue severity classification
   - Real-time health updates

### Usage

```bash
# Launch mission monitor (in separate terminal)
ros2 run tyre_inspection_mission mission_monitor
```

### Published Topics

- `/mission_monitor/status` - Comprehensive mission status
- `/mission_monitor/issues` - Detected issues and recommendations

---

## Mission Status Viewer

**Location:** `tyre_inspection_mission/scripts/mission_status.py`

### Features

- Real-time status display
- Progress tracking
- Issue visualization
- Health status indicator

### Usage

```bash
# View mission status in real-time
ros2 run tyre_inspection_mission mission_status
```

### Display Information

- Current state
- Elapsed time
- State duration
- Vehicles detected/completed
- Tyres detected/photographed
- Completion percentage
- Health status
- Detected issues

---

## Issue Detection

### Stuck State Detection

**Detects:** States that have been active too long

**Thresholds:**
- `SEARCHING_VEHICLES`: 120s (2 minutes)
- `PLANNING`: 60s (1 minute)
- `NAVIGATING_TO_LICENSE_PLATE`: 120s (2 minutes)
- `NAVIGATING_TO_TYRE`: 120s (2 minutes)
- `DETECTING_TYRES`: 60s (1 minute)
- `ERROR_RECOVERY`: 60s (1 minute)

**Recommendation:** Check logs for errors or system health

### High Error Rate Detection

**Detects:** Excessive errors during mission

**Threshold:** > 0.5 errors per minute

**Recommendation:** Review error logs and system health

### No Progress Detection

**Detects:** No vehicles detected after extended time

**Threshold:** > 5 minutes with no vehicles

**Recommendation:** Check detection pipeline, camera, and YOLO node

### State Loop Detection

**Detects:** Repeated state transitions (possible infinite loop)

**Threshold:** Only 2 unique states in last 5 transitions

**Recommendation:** Check state machine logic and error recovery

---

## Health Status

### Status Levels

- **healthy** - No issues detected
- **warning** - Minor issues detected
- **critical** - High severity issues detected

### Health Calculation

Health status is calculated based on:
- Number of issues
- Issue severity
- Mission progress
- Error rate

---

## Progress Tracking

### Metrics Tracked

- **Elapsed Time** - Total mission time
- **Current State** - Active state
- **State Duration** - Time in current state
- **Vehicles Detected** - Total vehicles found
- **Vehicles Completed** - Vehicles fully inspected
- **Tyres Detected** - Total tyres found
- **Tyres Photographed** - Tyres successfully captured
- **License Plates Captured** - License plate photos taken
- **Completion Rate** - Overall mission progress (0-100%)
- **Errors Encountered** - Total error count
- **Recovery Attempts** - Error recovery attempts

### Completion Rate Calculation

```
Completion = (Vehicle Completion × 60%) + (Tyre Completion × 40%)

Where:
- Vehicle Completion = vehicles_completed / vehicles_detected
- Tyre Completion = tyres_photographed / tyres_detected
```

---

## Integration

### With Mission Controller

The mission monitor automatically subscribes to:
- `/mission_controller/status` - Mission status updates

### With Diagnostic Tools

Use together for comprehensive monitoring:

```bash
# Terminal 1: Mission controller
ros2 launch tyre_inspection_mission autonomous_inspection.launch.py

# Terminal 2: Mission monitor
ros2 run tyre_inspection_mission mission_monitor

# Terminal 3: Status viewer
ros2 run tyre_inspection_mission mission_status

# Terminal 4: Diagnostic check (periodic)
ros2 run tyre_inspection_mission diagnostic_check
```

---

## Example Output

### Mission Status Viewer

```
======================================================================
MISSION STATUS VIEWER
======================================================================

📊 Current State: NAVIGATING_TO_TYRE
⏱️  Elapsed Time: 0:05:23
⏳ State Duration: 12.5s

🚗 Vehicles: 2 detected, 0 completed
🛞 Tyres: 8 detected, 3 photographed
✅ Completion: 18.8%

🟢 Health Status: HEALTHY

✅ No issues detected

🚗 Current Vehicle: vehicle_001

======================================================================
Press Ctrl+C to exit
======================================================================
```

### Issue Detection Example

```json
{
  "timestamp": 1234567890.123,
  "issues": [
    {
      "type": "stuck_state",
      "severity": "high",
      "state": "SEARCHING_VEHICLES",
      "duration": 125.3,
      "recommendation": "State SEARCHING_VEHICLES has been active for 125.3s. Check logs for errors or system health."
    }
  ]
}
```

---

## Best Practices

1. **Always Run Monitor** - Launch mission monitor alongside mission controller
2. **Watch Status Viewer** - Use status viewer for real-time visibility
3. **Check Issues** - Review detected issues and recommendations
4. **Monitor Health** - Watch health status for early problem detection
5. **Track Progress** - Use completion rate to gauge mission progress

---

## Troubleshooting

### Monitor Not Receiving Updates

**Check:**
- Mission controller is running
- `/mission_controller/status` topic is publishing
- Monitor node is running

**Fix:**
```bash
# Check topic
ros2 topic echo /mission_controller/status

# Restart monitor
ros2 run tyre_inspection_mission mission_monitor
```

### Issues Not Detected

**Check:**
- Monitor is running
- Issue thresholds are appropriate
- State transitions are being tracked

**Fix:**
- Adjust thresholds in `mission_monitor.py` if needed
- Check monitor logs for errors

---

**Last Updated:** Current Session  
**Version:** 1.0

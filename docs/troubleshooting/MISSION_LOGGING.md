# Mission Logging and Analysis

## Overview

The system includes comprehensive mission logging for post-mortem analysis, debugging, and performance optimization.

## Mission Logger

**Location:** `tyre_inspection_mission/mission_logger.py`

### Features

1. **Comprehensive Event Logging**
   - State transitions
   - Object detections
   - Navigation events
   - Photo captures
   - Errors and issues
   - System events

2. **JSONL Format**
   - One JSON object per line
   - Easy to parse and analyze
   - Human-readable
   - Efficient storage

3. **Event Types**
   - `mission_start` - Mission initialization
   - `state_transition` - State machine transitions
   - `detection` - Object detections
   - `navigation` - Navigation events
   - `capture` - Photo capture events
   - `error` - Error occurrences
   - `issue` - Detected issues

---

## Mission Log Recorder

**Location:** `tyre_inspection_mission/mission_logger.py`

### Usage

```bash
# Start log recorder
ros2 run tyre_inspection_mission mission_log_recorder

# With custom log directory
ros2 run tyre_inspection_mission mission_log_recorder --ros-args -p log_dir:=/path/to/logs
```

### Log File Location

**Default:** `~/tyre_inspection_logs/`

**Format:** `mission_log_YYYYMMDD_HHMMSS.jsonl`

---

## Log Analysis

### Mission Log Analyzer

**Location:** `tyre_inspection_mission/scripts/analyze_mission_log.py`

### Usage

```bash
ros2 run tyre_inspection_mission analyze_mission_log <log_file>
```

### Example

```bash
ros2 run tyre_inspection_mission analyze_mission_log \
  ~/tyre_inspection_logs/mission_log_20240109_120000.jsonl
```

### Analysis Output

The analyzer provides:
- Mission summary (duration, events, states)
- Events by type
- Detection statistics
- Capture success rate
- Error analysis
- Issue summary
- State transition history
- Performance statistics

---

## Log File Format

### Event Structure

```json
{
  "event_type": "state_transition",
  "timestamp": 1234567890.123,
  "elapsed": 45.67,
  "data": {
    "from": "SEARCHING_VEHICLES",
    "to": "PLANNING",
    "reason": "Vehicle detected"
  }
}
```

### Event Types

#### State Transition
```json
{
  "event_type": "state_transition",
  "data": {
    "from": "SEARCHING_VEHICLES",
    "to": "PLANNING",
    "reason": "Vehicle detected"
  }
}
```

#### Detection
```json
{
  "event_type": "detection",
  "data": {
    "object_type": "vehicle",
    "object_id": "vehicle_001",
    "position": {"x": 1.5, "y": 2.3, "z": 0.0},
    "confidence": 0.85
  }
}
```

#### Navigation
```json
{
  "event_type": "navigation",
  "data": {
    "action": "start",
    "target": "license_plate",
    "goal": {"x": 1.5, "y": 2.3, "z": 0.0},
    "result": "accepted"
  }
}
```

#### Capture
```json
{
  "event_type": "capture",
  "data": {
    "target_type": "tyre",
    "target_id": "tyre_001",
    "success": true,
    "path": "/path/to/photo.jpg"
  }
}
```

#### Error
```json
{
  "event_type": "error",
  "data": {
    "error_type": "NavigationError",
    "error_code": "NAV_001",
    "message": "Goal too close to robot",
    "context": {"distance": 0.4, "min_safe": 0.6}
  }
}
```

#### Issue
```json
{
  "event_type": "issue",
  "data": {
    "issue_type": "stuck_state",
    "severity": "high",
    "recommendation": "Check logs for errors",
    "details": {"state": "SEARCHING_VEHICLES", "duration": 125.3}
  }
}
```

---

## Analysis Use Cases

### Debugging Failed Missions

1. **Load log file**
   ```bash
   ros2 run tyre_inspection_mission analyze_mission_log <log_file>
   ```

2. **Review errors**
   - Check error list
   - Review error context
   - Identify error patterns

3. **Review state transitions**
   - Check for unexpected transitions
   - Identify stuck states
   - Find state loops

### Performance Analysis

1. **Check statistics**
   - Error rate
   - Capture success rate
   - State durations

2. **Identify bottlenecks**
   - Long state durations
   - High error rates
   - Repeated failures

### Issue Investigation

1. **Review detected issues**
   - Check issue types
   - Review recommendations
   - Analyze issue timing

2. **Correlate with events**
   - Match issues with state transitions
   - Link issues to errors
   - Find root causes

---

## Best Practices

1. **Always Record Logs** - Run log recorder during testing
2. **Analyze After Missions** - Review logs after each test
3. **Keep Log History** - Archive logs for comparison
4. **Use for Debugging** - Logs provide complete mission history

---

## Integration

### Complete Setup

```bash
# Terminal 1: Mission system
ros2 launch tyre_inspection_mission autonomous_inspection.launch.py

# Terminal 2: Mission monitor
ros2 run tyre_inspection_mission mission_monitor

# Terminal 3: Log recorder
ros2 run tyre_inspection_mission mission_log_recorder

# Terminal 4: Status viewer
ros2 run tyre_inspection_mission mission_status
```

### Post-Mission Analysis

```bash
# After mission completes
ros2 run tyre_inspection_mission analyze_mission_log \
  ~/tyre_inspection_logs/mission_log_*.jsonl
```

---

## Log File Management

### Finding Log Files

```bash
# List all log files
ls -lh ~/tyre_inspection_logs/

# Find latest log
ls -t ~/tyre_inspection_logs/ | head -1
```

### Log File Size

- Typical mission: 100KB - 1MB
- Long missions: Up to 10MB
- JSONL format: Efficient storage

### Archiving

```bash
# Archive old logs
mkdir -p ~/tyre_inspection_logs/archive
mv ~/tyre_inspection_logs/mission_log_*.jsonl ~/tyre_inspection_logs/archive/
```

---

**Last Updated:** Current Session  
**Version:** 1.0

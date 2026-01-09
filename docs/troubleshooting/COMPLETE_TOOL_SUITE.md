# Complete Tool Suite Guide

## Overview

The tyre inspection mission includes a comprehensive suite of tools for monitoring, debugging, and analyzing mission execution.

---

## 🛠️ Available Tools

### 1. Diagnostic Check
**Command:** `ros2 run tyre_inspection_mission diagnostic_check`

**Purpose:** Quick system health check before starting mission

**Checks:**
- Topics availability
- Services availability
- Nav2 status
- TF frames
- Overall system health

**Output:**
- Exit code: 0=healthy, 2=warning, 1=critical
- Detailed status report
- Recommendations for issues

**When to Use:**
- Before starting mission
- After system changes
- When troubleshooting

---

### 2. Mission Monitor
**Command:** `ros2 run tyre_inspection_mission mission_monitor`

**Purpose:** Real-time mission monitoring and issue detection

**Features:**
- Progress tracking
- Automatic issue detection
- Health status monitoring
- Adaptive recommendations

**Published Topics:**
- `/mission_monitor/status` - Comprehensive status
- `/mission_monitor/issues` - Detected issues

**When to Use:**
- During mission execution
- For real-time monitoring
- For issue detection

---

### 3. Mission Status Viewer
**Command:** `ros2 run tyre_inspection_mission mission_status`

**Purpose:** Real-time status display with visualization

**Features:**
- Live status updates
- Progress visualization
- Issue alerts
- Health indicators

**When to Use:**
- During mission execution
- For visual monitoring
- For quick status checks

---

### 4. Mission Log Recorder
**Command:** `ros2 run tyre_inspection_mission mission_log_recorder`

**Purpose:** Record all mission events for analysis

**Features:**
- Comprehensive event logging
- JSONL format
- Post-mortem analysis
- Performance tracking

**Log Location:** `~/tyre_inspection_logs/`

**When to Use:**
- During all missions
- For debugging
- For performance analysis

---

### 5. Mission Log Analyzer
**Command:** `ros2 run tyre_inspection_mission analyze_mission_log <log_file>`

**Purpose:** Analyze mission logs for debugging and optimization

**Features:**
- Event statistics
- Error analysis
- Issue summary
- Performance metrics
- State transition history

**When to Use:**
- After mission completion
- For debugging failures
- For performance analysis

---

## 🚀 Complete Setup

### Recommended Workflow

```bash
# Terminal 1: Launch system
ros2 launch tyre_inspection_mission autonomous_inspection.launch.py

# Terminal 2: Start monitor
ros2 run tyre_inspection_mission mission_monitor

# Terminal 3: Start log recorder
ros2 run tyre_inspection_mission mission_log_recorder

# Terminal 4: View status
ros2 run tyre_inspection_mission mission_status

# Terminal 5: Start mission
ros2 service call /mission_controller/start std_srvs/srv/Trigger
```

### Post-Mission Analysis

```bash
# After mission completes
ros2 run tyre_inspection_mission analyze_mission_log \
  ~/tyre_inspection_logs/mission_log_*.jsonl
```

---

## 📊 Tool Comparison

| Tool | Purpose | When to Use | Output |
|------|---------|-------------|--------|
| `diagnostic_check` | System health | Before mission | Exit code + report |
| `mission_monitor` | Runtime monitoring | During mission | Topics + logs |
| `mission_status` | Status display | During mission | Terminal display |
| `mission_log_recorder` | Event logging | During mission | Log files |
| `analyze_mission_log` | Log analysis | After mission | Analysis report |

---

## 🎯 Use Cases

### Before Mission
1. Run `diagnostic_check` - Verify system health
2. Check all tools are available
3. Review configuration

### During Mission
1. Run `mission_monitor` - Monitor progress
2. Run `mission_status` - Visual status
3. Run `mission_log_recorder` - Record events

### After Mission
1. Run `analyze_mission_log` - Analyze results
2. Review errors and issues
3. Check performance metrics

### Troubleshooting
1. Check `diagnostic_check` output
2. Review `mission_monitor` issues
3. Analyze mission logs
4. Check documentation

---

## 📖 Documentation

### Tool-Specific Guides
- **Diagnostic Check:** See [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md)
- **Mission Monitor:** See [RUNTIME_MONITORING.md](RUNTIME_MONITORING.md)
- **Mission Logging:** See [MISSION_LOGGING.md](MISSION_LOGGING.md)
- **Configuration:** See [CONFIGURATION_VALIDATION.md](CONFIGURATION_VALIDATION.md)

### Quick References
- **Quick Reference:** [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- **Issue Location:** [ISSUE_LOCATION_MAP.md](ISSUE_LOCATION_MAP.md)

---

## 🔍 Finding Issues

### By Tool

**Diagnostic Check:**
- System health issues
- Missing topics/services
- TF frame problems

**Mission Monitor:**
- Stuck states
- High error rates
- Progress issues
- State loops

**Mission Log Analyzer:**
- Error patterns
- Performance bottlenecks
- State transition issues
- Capture failures

---

## ✅ Best Practices

1. **Always Run Diagnostic Check** - Before starting mission
2. **Monitor During Mission** - Use monitor and status viewer
3. **Record All Missions** - Use log recorder
4. **Analyze After Mission** - Review logs for issues
5. **Keep Log History** - Archive logs for comparison

---

**Last Updated:** Current Session  
**Version:** 1.0

# Master Troubleshooting Index

## 🎯 Complete Guide to Finding Solutions

This is your **one-stop reference** for finding solutions to any issue. Use this index to quickly locate the right documentation.

---

## 🚨 By Problem Type

### Detection Issues

**Problem:** No vehicles/tyres detected, detection pipeline broken

**Documentation:**
- [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-no-vehicle-detection) - Comprehensive solutions
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md#no-vehicle-detection) - Quick fixes
- [ISSUE_LOCATION_MAP.md](ISSUE_LOCATION_MAP.md#detection-issues) - Where to find code
- [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#detection-errors) - Error codes DET_001-DET_003

**Tools:**
- `diagnostic_check` - Check topic availability
- `mission_monitor` - Monitor detection statistics
- Check YOLO logs

---

### Navigation Issues

**Problem:** Navigation fails, goals rejected, timeouts

**Documentation:**
- [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-navigation-failures) - Comprehensive solutions
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md#navigation-failures) - Quick fixes
- [ISSUE_LOCATION_MAP.md](ISSUE_LOCATION_MAP.md#navigation-issues) - Where to find code
- [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#navigation-errors) - Error codes NAV_001-NAV_004

**Tools:**
- `diagnostic_check` - Check Nav2 status
- `mission_monitor` - Monitor navigation events
- Check Nav2 logs

---

### Capture Issues

**Problem:** Photo capture fails, service unavailable

**Documentation:**
- [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-photo-capture-failures) - Comprehensive solutions
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md#photo-capture-failures) - Quick fixes
- [ISSUE_LOCATION_MAP.md](ISSUE_LOCATION_MAP.md#capture-issues) - Where to find code
- [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#capture-errors) - Error codes CAP_001-CAP_002

**Tools:**
- `diagnostic_check` - Check service availability
- Check photo_capture node logs

---

### State Machine Issues

**Problem:** Stuck in state, invalid transitions, loops

**Documentation:**
- [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-state-machine-stuck) - Comprehensive solutions
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md#state-machine-stuck) - Quick fixes
- [ISSUE_LOCATION_MAP.md](ISSUE_LOCATION_MAP.md#state-machine-issues) - Where to find code
- [STATE_VALIDATION.md](../../architecture/STATE_VALIDATION.md) - State validation details
- [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#state-machine-errors) - Error codes STM_001-STM_002

**Tools:**
- `mission_status` - View current state
- `mission_monitor` - Detect stuck states
- `analyze_mission_log` - Review state transitions

---

### Configuration Issues

**Problem:** Invalid configuration, parameter errors

**Documentation:**
- [CONFIGURATION_VALIDATION.md](CONFIGURATION_VALIDATION.md) - Complete validation guide
- [STARTUP_VALIDATION.md](STARTUP_VALIDATION.md) - Startup checks
- [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#validation-errors) - Error codes VAL_001-VAL_002

**Tools:**
- `diagnostic_check` - System health
- Startup validation (automatic)

---

### Transform Issues

**Problem:** TF frames missing, transforms fail

**Documentation:**
- [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md#-issue-tf-frames-missing) - Comprehensive solutions
- [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#transform-errors) - Error codes TF_001-TF_002
- [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md#transform-errors) - Recovery strategies

**Tools:**
- `diagnostic_check` - Check TF frames
- Check TF tree: `ros2 run tf2_tools view_frames`

---

### Planning Issues

**Problem:** Waypoint calculation fails, invalid poses

**Documentation:**
- [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md#planning-errors) - Recovery strategies
- [ISSUE_LOCATION_MAP.md](ISSUE_LOCATION_MAP.md#planning-issues) - Where to find code
- [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#planning-errors) - Error codes PLAN_001-PLAN_003

**Tools:**
- `mission_monitor` - Monitor planning events
- `analyze_mission_log` - Review planning failures

---

## 🔍 By Error Code

### Navigation Errors (NAV_xxx)
- **NAV_001** - Goal too close → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#nav_001---goal-too-close-to-robot)
- **NAV_002** - Nav2 unavailable → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#nav_002---nav2-action-server-not-available)
- **NAV_003** - Goal rejected → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#nav_003---navigation-goal-rejected)
- **NAV_004** - Navigation timeout → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#nav_004---navigation-timeout)

### Detection Errors (DET_xxx)
- **DET_001** - No vehicles → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#det_001---no-vehicles-detected)
- **DET_002** - No tyres → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#det_002---no-tyres-detected)
- **DET_003** - Point cloud unavailable → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#det_003---point-cloud-not-available)

### Capture Errors (CAP_xxx)
- **CAP_001** - Service unavailable → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#cap_001---photo-capture-service-unavailable)
- **CAP_002** - Capture failed → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#cap_002---photo-capture-failed)

### Planning Errors (PLAN_xxx)
- **PLAN_001** - Waypoint calculation failed → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#plan_001---waypoint-calculation-failed)
- **PLAN_002** - Invalid vehicle pose → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#plan_002---invalid-vehicle-pose)
- **PLAN_003** - Fallback failed → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#plan_003---fallback-waypoint-generation-failed)

### Transform Errors (TF_xxx)
- **TF_001** - Frame not found → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#tf_001---tf-frame-not-found)
- **TF_002** - Transform failed → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#tf_002---tf-transform-failed)

### Validation Errors (VAL_xxx)
- **VAL_001** - Invalid pose → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#val_001---invalid-pose)
- **VAL_002** - Invalid parameter → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#val_002---invalid-parameter)

### State Machine Errors (STM_xxx)
- **STM_001** - Invalid transition → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#stm_001---invalid-state-transition)
- **STM_002** - State timeout → [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md#stm_002---state-timeout)

---

## 🛠️ By Tool

### diagnostic_check
**Purpose:** System health check

**When to Use:**
- Before starting mission
- After system changes
- When troubleshooting

**Documentation:**
- [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md) - General troubleshooting
- [COMPLETE_TOOL_SUITE.md](COMPLETE_TOOL_SUITE.md#1-diagnostic-check) - Tool details

---

### mission_monitor
**Purpose:** Runtime monitoring and issue detection

**When to Use:**
- During mission execution
- For real-time monitoring
- For issue detection

**Documentation:**
- [RUNTIME_MONITORING.md](RUNTIME_MONITORING.md) - Complete monitoring guide
- [MONITORING_QUICK_START.md](MONITORING_QUICK_START.md) - Quick setup
- [COMPLETE_TOOL_SUITE.md](COMPLETE_TOOL_SUITE.md#2-mission-monitor) - Tool details

---

### mission_status
**Purpose:** Real-time status display

**When to Use:**
- During mission execution
- For visual monitoring
- For quick status checks

**Documentation:**
- [RUNTIME_MONITORING.md](RUNTIME_MONITORING.md) - Monitoring guide
- [COMPLETE_TOOL_SUITE.md](COMPLETE_TOOL_SUITE.md#3-mission-status-viewer) - Tool details

---

### mission_log_recorder
**Purpose:** Event logging

**When to Use:**
- During all missions
- For debugging
- For performance analysis

**Documentation:**
- [MISSION_LOGGING.md](MISSION_LOGGING.md) - Logging guide
- [COMPLETE_TOOL_SUITE.md](COMPLETE_TOOL_SUITE.md#4-mission-log-recorder) - Tool details

---

### analyze_mission_log
**Purpose:** Log analysis

**When to Use:**
- After mission completion
- For debugging failures
- For performance analysis

**Documentation:**
- [MISSION_LOGGING.md](MISSION_LOGGING.md) - Logging guide
- [COMPLETE_TOOL_SUITE.md](COMPLETE_TOOL_SUITE.md#5-mission-log-analyzer) - Tool details

---

## 📖 By Documentation Type

### Quick References
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Fast lookup
- [MONITORING_QUICK_START.md](MONITORING_QUICK_START.md) - Quick monitoring setup

### Comprehensive Guides
- [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md) - Complete troubleshooting
- [RUNTIME_MONITORING.md](RUNTIME_MONITORING.md) - Monitoring guide
- [MISSION_LOGGING.md](MISSION_LOGGING.md) - Logging guide

### Reference Documents
- [ISSUE_LOCATION_MAP.md](ISSUE_LOCATION_MAP.md) - Where to find code
- [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md) - Error codes
- [CONFIGURATION_VALIDATION.md](CONFIGURATION_VALIDATION.md) - Config validation
- [STARTUP_VALIDATION.md](STARTUP_VALIDATION.md) - Startup checks
- [COMPLETE_TOOL_SUITE.md](COMPLETE_TOOL_SUITE.md) - All tools

### Architecture
- [MODULE_ORGANIZATION.md](../../architecture/MODULE_ORGANIZATION.md) - System structure
- [ERROR_RECOVERY_STRATEGY.md](../../architecture/ERROR_RECOVERY_STRATEGY.md) - Error handling
- [STATE_VALIDATION.md](../../architecture/STATE_VALIDATION.md) - State validation

---

## 🎯 Quick Decision Tree

### I Have an Error Code
→ Go to [ERROR_CONTEXT_REFERENCE.md](ERROR_CONTEXT_REFERENCE.md)

### I Don't Know What's Wrong
→ Start with [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
→ Then [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md)

### I Need to Find Code
→ Go to [ISSUE_LOCATION_MAP.md](ISSUE_LOCATION_MAP.md)

### I Need to Monitor Mission
→ Go to [MONITORING_QUICK_START.md](MONITORING_QUICK_START.md)
→ Then [RUNTIME_MONITORING.md](RUNTIME_MONITORING.md)

### I Need to Analyze Logs
→ Go to [MISSION_LOGGING.md](MISSION_LOGGING.md)

### I Need to Understand System
→ Go to [MODULE_ORGANIZATION.md](../../architecture/MODULE_ORGANIZATION.md)

---

## 📊 Complete Documentation Map

```
docs/
├── troubleshooting/
│   ├── MASTER_TROUBLESHOOTING_INDEX.md  ← YOU ARE HERE
│   ├── TROUBLESHOOTING_GUIDE.md         ← Comprehensive solutions
│   ├── QUICK_REFERENCE.md               ← Fast lookup
│   ├── ISSUE_LOCATION_MAP.md            ← Where to find code
│   ├── RUNTIME_MONITORING.md            ← Monitoring guide
│   ├── MONITORING_QUICK_START.md        ← Quick setup
│   ├── CONFIGURATION_VALIDATION.md      ← Config validation
│   ├── STARTUP_VALIDATION.md            ← Startup checks
│   ├── MISSION_LOGGING.md               ← Logging guide
│   ├── ERROR_CONTEXT_REFERENCE.md       ← Error codes
│   └── COMPLETE_TOOL_SUITE.md           ← All tools
│
└── architecture/
    ├── MODULE_ORGANIZATION.md           ← System structure
    ├── ERROR_RECOVERY_STRATEGY.md       ← Error handling
    ├── STATE_VALIDATION.md              ← State validation
    └── SELF_HEALING_AND_VALIDATION.md   ← Self-healing
```

---

## ✅ Troubleshooting Workflow

1. **Identify Problem**
   - Check error code
   - Review mission status
   - Check logs

2. **Find Documentation**
   - Use this index
   - Check error code reference
   - Review issue location map

3. **Apply Solution**
   - Follow recovery steps
   - Use recommended tools
   - Check expected outcome

4. **Verify Fix**
   - Run diagnostic check
   - Monitor mission
   - Review logs

---

**Last Updated:** Current Session  
**Version:** 1.0  
**Your Complete Troubleshooting Hub**

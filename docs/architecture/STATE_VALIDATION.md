# State Validation and Self-Healing

## Overview

The mission controller now includes comprehensive state validation and self-healing mechanisms to ensure robust operation and automatic recovery from invalid states.

## State Validator

**Location:** `tyre_inspection_mission/state_validator.py`

### Features

1. **State Transition Validation**
   - Validates all state transitions before they occur
   - Blocks invalid transitions
   - Warns about suspicious transitions
   - Tracks state history

2. **State Requirement Validation**
   - Validates that required data exists for each state
   - Checks for missing vehicles, tyres, waypoints
   - Provides warnings for missing optional data

3. **State Timeout Detection**
   - Monitors time spent in each state
   - Detects timeout conditions
   - Provides recovery suggestions

4. **Recovery Suggestions**
   - Provides actionable recovery suggestions for each state
   - Context-aware recommendations
   - Based on error type and state

### Usage

```python
from tyre_inspection_mission.state_validator import StateValidator, StateValidationResult

validator = StateValidator(logger)

# Validate transition
result, reason = validator.validate_state_transition(
    MissionState.SEARCHING_VEHICLES,
    MissionState.PLANNING
)

if result == StateValidationResult.INVALID:
    # Block transition
    pass
elif result == StateValidationResult.WARNING:
    # Log warning but allow
    pass
```

## State Transition Validation

### Invalid Transitions

The validator blocks these invalid transitions:

- `IDLE` → Most active states (must go through `SEARCHING_VEHICLES`)
- `MISSION_COMPLETE` → Active states
- `SEARCHING_VEHICLES` → `NAVIGATING_TO_LICENSE_PLATE` (must plan first)
- Other logically invalid transitions

### Suspicious Transitions

The validator warns about suspicious transitions:

- Rapid state changes (possible error loop)
- Backwards transitions (might indicate error)
- Repeated state entries (possible infinite loop)

## State Requirement Validation

### States Requiring Current Vehicle

These states require `current_vehicle` to be set:

- `PLANNING`
- `NAVIGATING_TO_LICENSE_PLATE`
- `CAPTURING_LICENSE_PLATE`
- `SWITCHING_TO_INSPECTION`
- `DETECTING_TYRES`
- `NAVIGATING_TO_TYRE`
- `CAPTURING_TYRE`
- `CHECKING_COMPLETION`

### Additional Requirements

- `NAVIGATING_TO_TYRE`: Requires `current_tyre_index` and `tyre_waypoints`
- `CAPTURING_TYRE`: Requires `current_tyre_index`
- `PLANNING`: Warns if vehicle has no `detection_pose`

## State Timeout Detection

### Default Timeouts

| State | Timeout | Config |
|-------|---------|--------|
| `SEARCHING_VEHICLES` | 60s | `DetectionConfig.DETECTION_TIMEOUT` |
| `PLANNING` | 30s | `PlanningConfig.PLANNING_TIMEOUT` |
| `NAVIGATING_TO_LICENSE_PLATE` | 60s | `NavigationConfig.NAVIGATION_TIMEOUT` |
| `NAVIGATING_TO_TYRE` | 60s | `NavigationConfig.NAVIGATION_TIMEOUT` |
| `DETECTING_TYRES` | 30s | `DetectionConfig.TYRE_DETECTION_TIMEOUT` |
| `CAPTURING_LICENSE_PLATE` | 10s | `CaptureConfig.PHOTO_CAPTURE_TIMEOUT` |
| `CAPTURING_TYRE` | 10s | `CaptureConfig.PHOTO_CAPTURE_TIMEOUT` |
| `ERROR_RECOVERY` | 30s | Hardcoded |

### Timeout Handling

When a timeout is detected:

1. **Warning logged** - State timeout is logged
2. **Recovery suggestion** - Validator provides recovery suggestion
3. **Critical timeouts** - Critical states transition to `ERROR_RECOVERY`
4. **Non-critical** - Non-critical states may continue with warnings

## Recovery Suggestions

The validator provides context-aware recovery suggestions:

### SEARCHING_VEHICLES
- **No detections**: "Check YOLO node, camera, and vehicle distance"
- **Timeout**: "Continue searching or check if vehicles are in view"

### PLANNING
- **No waypoints**: "Use fallback waypoint generation"
- **Timeout**: "Proceed with available waypoints"

### NAVIGATING_TO_LICENSE_PLATE
- **Goal rejected**: "Recalculate goal with increased distance"
- **Timeout**: "Proceed if close enough, otherwise cancel and retry"

### DETECTING_TYRES
- **No tyres**: "Use fallback waypoint generation"
- **Timeout**: "Generate fallback waypoints and proceed"

### NAVIGATING_TO_TYRE
- **Goal rejected**: "Use pre-planned waypoint or recalculate"
- **Timeout**: "Proceed if close enough, otherwise skip to next tyre"

### CAPTURING_TYRE
- **Service unavailable**: "Wait for service or retry"
- **Timeout**: "Skip to next tyre if max attempts reached"

## Integration with Mission Controller

### State Transitions

All state transitions should use `_transition_to_state()`:

```python
# Instead of:
self.state = MissionState.PLANNING

# Use:
self._transition_to_state(MissionState.PLANNING, "Vehicle detected")
```

### Benefits

1. **Automatic validation** - All transitions are validated
2. **Consistent logging** - All transitions are logged
3. **Time tracking** - State entry times are recorded
4. **Error prevention** - Invalid transitions are blocked

### State Machine Step

The state machine step now includes:

1. **Pre-execution validation** - Validates state requirements
2. **Timeout checking** - Checks for state timeouts
3. **Recovery suggestions** - Provides suggestions for issues
4. **Automatic recovery** - Transitions to ERROR_RECOVERY if critical

## State Statistics

The validator tracks state statistics:

```python
stats = validator.get_state_statistics()
# Returns:
# {
#     'total_transitions': 50,
#     'state_counts': {'SEARCHING_VEHICLES': 5, 'PLANNING': 3, ...},
#     'average_state_times': {'SEARCHING_VEHICLES': 12.5, ...},
#     'recent_states': ['SEARCHING_VEHICLES', 'PLANNING', ...]
# }
```

## Error Prevention

### Invalid Transition Prevention

Invalid transitions are automatically blocked and redirected:

```python
# Attempting invalid transition:
self._transition_to_state(MissionState.NAVIGATING_TO_LICENSE_PLATE)
# From IDLE state

# Result:
# - Transition blocked
# - Error logged
# - Redirected to ERROR_RECOVERY
```

### Missing Data Detection

Missing required data is detected before state execution:

```python
# State: NAVIGATING_TO_TYRE
# Missing: current_tyre_index

# Result:
# - Validation fails
# - Error logged
# - Transition to ERROR_RECOVERY
```

## Best Practices

1. **Always use `_transition_to_state()`** - Never directly assign `self.state`
2. **Provide transition reasons** - Helps with debugging
3. **Check validation results** - Handle warnings appropriately
4. **Monitor state statistics** - Track state behavior
5. **Review recovery suggestions** - Use for troubleshooting

## Troubleshooting

### State Validation Errors

If you see "Invalid state transition blocked":

1. Check state transition logic
2. Review state machine flow
3. Check for missing state transitions
4. Verify state requirements

### State Timeout Warnings

If you see state timeout warnings:

1. Check if timeout values are appropriate
2. Review state execution logic
3. Check for blocking operations
4. Consider increasing timeout if needed

### Missing Data Warnings

If you see missing data warnings:

1. Check state requirements
2. Verify data is set before state entry
3. Review state transition logic
4. Check for data initialization issues

---

**Last Updated:** Current Session  
**Version:** 1.0

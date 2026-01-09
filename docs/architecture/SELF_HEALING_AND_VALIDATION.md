# Self-Healing and State Validation

## Overview

The tyre inspection mission now includes comprehensive state validation and self-healing mechanisms to ensure robust operation and automatic recovery from invalid states.

## State Validator Module

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

## Integration Points

### Mission Controller Integration

The state validator is integrated into the mission controller:

```python
from tyre_inspection_mission.state_validator import StateValidator

# In __init__
self.state_validator = StateValidator(self.slogger)
self.state_entry_times = {}  # Track state entry times

# In state_machine_step
# Validate state before processing
validation_result, reason = self.state_validator.validate_state_requirements(...)

# Check for timeouts
is_timeout, timeout_reason = self.state_validator.check_state_timeout(...)

# Use _transition_to_state for all transitions
self._transition_to_state(MissionState.PLANNING, "Vehicle detected")
```

### State Transition Method

All state transitions should use `_transition_to_state()`:

```python
def _transition_to_state(self, new_state: MissionState, reason: Optional[str] = None):
    """Transition with validation and logging"""
    # Validate transition
    # Record entry time
    # Log transition
    # Perform transition
```

## Validation Rules

### Invalid Transitions

These transitions are blocked:
- `IDLE` → Most active states (must go through `SEARCHING_VEHICLES`)
- `MISSION_COMPLETE` → Active states
- `SEARCHING_VEHICLES` → `NAVIGATING_TO_LICENSE_PLATE` (must plan first)

### State Requirements

**States Requiring Current Vehicle:**
- `PLANNING`
- `NAVIGATING_TO_LICENSE_PLATE`
- `CAPTURING_LICENSE_PLATE`
- `SWITCHING_TO_INSPECTION`
- `DETECTING_TYRES`
- `NAVIGATING_TO_TYRE`
- `CAPTURING_TYRE`
- `CHECKING_COMPLETION`

**Additional Requirements:**
- `NAVIGATING_TO_TYRE`: Requires `current_tyre_index` and `tyre_waypoints`
- `CAPTURING_TYRE`: Requires `current_tyre_index`
- `PLANNING`: Warns if vehicle has no `detection_pose`

## Timeout Detection

### Default Timeouts

| State | Timeout | Action on Timeout |
|-------|---------|-------------------|
| `SEARCHING_VEHICLES` | 60s | Continue or complete |
| `PLANNING` | 30s | Proceed with available waypoints |
| `NAVIGATING_TO_*` | 60s | Proceed if close enough |
| `DETECTING_TYRES` | 30s | Generate fallback waypoints |
| `CAPTURING_*` | 10s | Retry or skip |
| `ERROR_RECOVERY` | 30s | Reset to IDLE |

### Timeout Handling

1. **Warning logged** - State timeout is logged
2. **Recovery suggestion** - Validator provides recovery suggestion
3. **Critical timeouts** - Critical states transition to `ERROR_RECOVERY`
4. **Non-critical** - Non-critical states may continue with warnings

## Self-Healing Mechanisms

### Automatic Recovery

1. **Invalid Transition** → Redirected to `ERROR_RECOVERY`
2. **Missing Data** → Transition to `ERROR_RECOVERY` with context
3. **State Timeout** → Recovery suggestion + transition if critical
4. **Suspicious Pattern** → Warning logged, transition allowed

### Recovery Strategies

The validator provides context-aware recovery suggestions:

- **SEARCHING_VEHICLES**: Check YOLO, camera, vehicle distance
- **PLANNING**: Use fallback waypoint generation
- **NAVIGATING_TO_***: Recalculate goal or proceed if close
- **DETECTING_TYRES**: Use fallback waypoint generation
- **CAPTURING_***: Retry or skip to next

## State Statistics

The validator tracks state statistics:

```python
stats = validator.get_state_statistics()
# Returns:
# {
#     'total_transitions': 50,
#     'state_counts': {'SEARCHING_VEHICLES': 5, ...},
#     'average_state_times': {'SEARCHING_VEHICLES': 12.5, ...},
#     'recent_states': [...]
# }
```

## Benefits

### Error Prevention
- ✅ Blocks invalid transitions before they occur
- ✅ Detects missing required data early
- ✅ Identifies suspicious patterns

### Automatic Recovery
- ✅ Provides recovery suggestions
- ✅ Automatically redirects invalid transitions
- ✅ Handles timeouts gracefully

### Better Debugging
- ✅ Tracks state history
- ✅ Provides statistics
- ✅ Logs validation results

## Usage Example

```python
# Validate transition
result, reason = validator.validate_state_transition(
    MissionState.SEARCHING_VEHICLES,
    MissionState.PLANNING
)

if result == StateValidationResult.INVALID:
    # Block transition
    self._transition_to_state(MissionState.ERROR_RECOVERY, reason)
elif result == StateValidationResult.WARNING:
    # Log warning but allow
    self.slogger.warn(f"Transition warning: {reason}")
    self._transition_to_state(MissionState.PLANNING, "Proceeding with warning")
```

---

**Last Updated:** Current Session  
**Status:** Ready for Integration

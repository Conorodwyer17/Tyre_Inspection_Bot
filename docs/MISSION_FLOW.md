# Mission Flow Diagram

High-level mission flow for the autonomous tyre inspection robot. For the full state machine, see `mission_state_machine.py` and [MISSION_PIPELINE.md](MISSION_PIPELINE.md).

---

## Mission Flow (Simplified)

```mermaid
flowchart TD
    Start([Start]) --> IDLE
    IDLE -->|Start mission| SEARCH_VEHICLE
    SEARCH_VEHICLE -->|Vehicle detected| APPROACH_VEHICLE
    APPROACH_VEHICLE -->|Goal reached| INSPECT_TIRE

    subgraph TyreLoop["Tyre Inspection Loop"]
        direction TB
        INSPECT_TIRE --> ALL_TYRES{{"All 4 tyres done?"}}
        ALL_TYRES -->|No| INSPECT_TIRE
        ALL_TYRES -->|Yes| MORE_VEHICLES{{"More vehicles?"}}
    end

    MORE_VEHICLES -->|No| DONE([Done])
    MORE_VEHICLES -->|Yes| SEARCH_VEHICLE

    SEARCH_VEHICLE -->|Timeout / failure| ERROR([Error])
    APPROACH_VEHICLE -->|Timeout / failure| ERROR
    INSPECT_TIRE -->|Failure| ERROR
```

---

## Notes

- **Tyre order:** Nearest first, then 2nd, 3rd, 4th nearest (Nav2 paths around the vehicle).
- **Per tyre:** Navigate → face tyre → capture photo → verify → next.
- **Multi-vehicle:** After 4 tyres, the mission can search for the next vehicle or finish.

See [MISSION_PIPELINE.md](MISSION_PIPELINE.md) for phase details and [RUNBOOK.md](../RUNBOOK.md) for operations.

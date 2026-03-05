# Mission Flow Diagram

High-level mission flow for the autonomous tyre inspection robot. For the full state machine, see `mission_state_machine.py` and [MISSION_PIPELINE.md](MISSION_PIPELINE.md).

---

## Mission Flow (Simplified)

```mermaid
graph TD
    IDLE -->|Start mission| SEARCH_VEHICLE
    SEARCH_VEHICLE -->|Vehicle detected| WAIT_VEHICLE_BOX
    WAIT_VEHICLE_BOX -->|Vehicle committed| APPROACH_VEHICLE
    APPROACH_VEHICLE -->|Goal reached| INSPECT_TIRE
    INSPECT_TIRE -->|Position adjusted| FACE_TIRE
    FACE_TIRE -->|Wheel detected| VERIFY_CAPTURE
    VERIFY_CAPTURE -->|Photo OK| NEXT_TYRE
    NEXT_TYRE -->|More tyres| INSPECT_TIRE
    NEXT_TYRE -->|All tyres done| DONE
    INSPECT_TIRE -->|Timeout/error| ERROR
    ERROR -->|Recovery| IDLE
```

---

## Notes

- **Tyre order:** Nearest first, then 2nd, 3rd, 4th nearest (Nav2 paths around the vehicle).
- **Per tyre:** INSPECT_TIRE → FACE_TIRE → VERIFY_CAPTURE → NEXT_TYRE (loop until all 4 done).

See [MISSION_PIPELINE.md](MISSION_PIPELINE.md) for phase details and [RUNBOOK.md](../RUNBOOK.md) for operations.

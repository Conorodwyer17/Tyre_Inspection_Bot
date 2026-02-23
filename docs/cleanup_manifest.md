# Cleanup Manifest (Step 10)

## Backup location

- **Directory:** `~/ugv_ws/backups/2026-02-23T13:23:57/cleanup_archive/`
- **Contents:** Copies of deleted files (relative paths preserved under cleanup_archive).

## Deleted files

| File | Reason |
|------|--------|
| docs/test.txt | Empty placeholder file |
| frames_2026-02-21_15.09.39.gv | Generated TF/frame graph; can be regenerated with `ros2 run tf2_tools view_frames` |
| frames_2026-02-22_00.51.24.gv | Same |
| frames_2026-02-23_02.48.08.gv | Same |

## Candidate list

- **List used:** `~/ugv_ws/cleanup_candidates.txt` (paths relative to workspace).
- No calibration scripts or unique documentation were removed.

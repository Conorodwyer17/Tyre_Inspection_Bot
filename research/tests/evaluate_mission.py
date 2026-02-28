#!/usr/bin/env python3
"""Initial mission evaluation utility (simulated-mode capable)."""

import glob
import json
import os
import sqlite3
from datetime import datetime, timezone


def latest_capture_manifest() -> str:
    files = sorted(glob.glob("research/data/aurora_samples/aurora_capture_set_*_manifest.json"))
    return files[-1] if files else ""


def latest_mission(db_path: str):
    if not os.path.exists(db_path):
        return None, []
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        row = conn.execute("SELECT * FROM missions ORDER BY created_at DESC LIMIT 1").fetchone()
        if row is None:
            return None, []
        tires = conn.execute("SELECT * FROM tires WHERE mission_id = ? ORDER BY order_index", (row["mission_id"],)).fetchall()
        return dict(row), [dict(t) for t in tires]
    finally:
        conn.close()


def main() -> None:
    db_path = "research/state/inspection_missions.db"
    mission, tires = latest_mission(db_path)
    manifest = latest_capture_manifest()

    completed = len([t for t in tires if t.get("status") == "completed"])
    total = len(tires)
    tire_capture_rate = float(completed / total) if total else 0.0

    metrics = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "mode": "simulated",
        "aurora_capture_manifest": manifest,
        "mission_id": mission.get("mission_id") if mission else "",
        "mission_state": mission.get("state") if mission else "",
        "total_tires": total,
        "completed_tires": completed,
        "tire_capture_rate": tire_capture_rate,
        "mean_alignment_error_m": None,
        "reacquisition_success_rate": None,
        "notes": "Initial dry-run metrics from persisted mock mission and latest Aurora capture manifest.",
    }

    mission_id = metrics["mission_id"] or "no_mission"
    os.makedirs("research/logs/metrics", exist_ok=True)
    out = f"research/logs/metrics/{mission_id}.json"
    with open(out, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
    print(out)


if __name__ == "__main__":
    main()


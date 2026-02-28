#!/usr/bin/env python3
"""Mission persistence module (SQLite-backed)."""


class MissionStatePersistence:
    """Implementation added in next milestone."""

    def __init__(self, db_path: str) -> None:
        self.db_path = db_path

    def start_mission(self, object_id: str, config_json: str, allow_partial_success: bool = False) -> str:
        raise NotImplementedError

    def get_mission_state(self, mission_id: str):
        raise NotImplementedError

    def set_tire_status(self, mission_id: str, tire_id: str, status: str, **kwargs):
        raise NotImplementedError


from inspection_manager.state_persistence import MissionStatePersistence


def test_state_persistence_lifecycle(tmp_path):
    db_path = tmp_path / "inspection_missions.db"
    storage = MissionStatePersistence(str(db_path))

    mission_id = storage.start_mission("object_1", '{"max_retries":5}', allow_partial_success=False)
    storage.add_tires(mission_id, ["tire_fl", "tire_fr"])
    storage.set_tire_status(mission_id, "tire_fl", "completed", attempt_increment=1, photo_path="/tmp/a.jpg")
    storage.set_mission_state(mission_id, "POST_PROCESS")

    snapshot = storage.get_mission_state(mission_id)
    assert snapshot is not None
    assert snapshot["mission"]["mission_id"] == mission_id
    assert snapshot["mission"]["state"] == "POST_PROCESS"
    assert len(snapshot["tires"]) == 2
    completed = [t for t in snapshot["tires"] if t["status"] == "completed"]
    assert len(completed) == 1


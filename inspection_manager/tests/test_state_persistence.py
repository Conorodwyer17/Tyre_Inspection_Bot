from inspection_manager.state_persistence import MissionStatePersistence


def test_state_persistence_interface():
    storage = MissionStatePersistence("research/state/inspection_missions.db")
    assert storage.db_path.endswith(".db")


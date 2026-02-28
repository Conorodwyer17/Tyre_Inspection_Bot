from inspection_manager.tire_enumerator import TireEnumerator


def test_tire_enumerator_empty_default():
    enum = TireEnumerator()
    assert enum.enumerate_tires({}) == []


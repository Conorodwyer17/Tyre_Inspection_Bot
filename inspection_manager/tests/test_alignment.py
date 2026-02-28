from inspection_manager.alignment import AlignmentController


def test_alignment_default_false():
    align = AlignmentController()
    assert align.converge_until(5.0, 10.0) is False


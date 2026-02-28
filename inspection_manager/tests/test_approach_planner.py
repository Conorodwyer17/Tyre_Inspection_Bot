from inspection_manager.approach_planner import ApproachPlanner


def test_approach_planner_empty_default():
    planner = ApproachPlanner()
    assert planner.compute_goal({}) == {}


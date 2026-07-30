from simulation.mujoco_judge.sampler import stubbed_candidates


def test_stubbed_candidates_returns_two_well_formed_candidates():
    candidates = stubbed_candidates()
    assert len(candidates) == 2
    for c in candidates:
        assert set(c.keys()) == {"object", "bbox", "confidence", "action_type"}
        assert set(c["bbox"].keys()) == {"x", "y", "width", "height"}

from simulation.mujoco_judge.judge import MuJoCoJudge, Verdict


def test_center_bbox_candidate_is_chosen(model, data):
    judge = MuJoCoJudge(model, data)
    candidate = {
        "object": "red block",
        "bbox": {"x": 300.0, "y": 220.0, "width": 40.0, "height": 40.0},
        "confidence": 0.9,
        "action_type": "grasp",
    }
    assert judge.check(candidate) == Verdict.CHOSEN


def test_corner_bbox_candidate_is_rejected(model, data):
    judge = MuJoCoJudge(model, data)
    candidate = {
        "object": "table edge",
        "bbox": {"x": 0.0, "y": 0.0, "width": 10.0, "height": 10.0},
        "confidence": 0.4,
        "action_type": "avoid",
    }
    assert judge.check(candidate) == Verdict.REJECTED

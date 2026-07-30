from training.data import load_pairs

from simulation.mujoco_judge.pairs import make_pair, write_pair


def test_write_pair_round_trips_through_load_pairs(tmp_path):
    out_path = tmp_path / "pairs.jsonl"
    chosen = {"object": "red block", "bbox": {"x": 1.0, "y": 2.0, "width": 3.0, "height": 4.0}, "confidence": 0.9, "action_type": "grasp"}
    rejected = {"object": "table edge", "bbox": {"x": 0.0, "y": 0.0, "width": 10.0, "height": 10.0}, "confidence": 0.4, "action_type": "avoid"}
    pair = make_pair("frame_001.png", "describe the scene", chosen, rejected)

    write_pair(out_path, pair)

    loaded = load_pairs(out_path)
    assert len(loaded) == 1
    assert loaded[0].image_ref == "frame_001.png"
    assert loaded[0].chosen["object"] == "red block"


def test_write_pair_appends(tmp_path):
    out_path = tmp_path / "pairs.jsonl"
    pair = make_pair("a.png", "p", {"object": "x", "bbox": {"x": 0.0, "y": 0.0, "width": 1.0, "height": 1.0}, "confidence": 0.5, "action_type": "none"}, {"object": "y", "bbox": {"x": 0.0, "y": 0.0, "width": 1.0, "height": 1.0}, "confidence": 0.5, "action_type": "none"})

    write_pair(out_path, pair)
    write_pair(out_path, pair)

    assert len(load_pairs(out_path)) == 2

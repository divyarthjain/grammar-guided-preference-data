from training.data import load_pairs

from simulation.mujoco_judge.loop import run_continuous


def test_run_continuous_one_iteration_writes_well_formed_output(tmp_path, model, data):
    out_path = tmp_path / "pairs.jsonl"

    pairs_written = run_continuous(
        model, data, out_path, max_iterations=1, launch_viewer=False, frame_delay_seconds=0.0
    )

    # Task 6 already established that stubbed_candidates()'s two entries
    # deterministically judge as CHOSEN (center bbox) and REJECTED (corner
    # bbox) — so one iteration over both must write exactly one pair.
    assert pairs_written == 1
    loaded = load_pairs(out_path)
    assert len(loaded) == 1
    assert loaded[0].chosen["object"] == "red block"
    assert loaded[0].rejected["object"] == "table edge"

from training.data import load_pairs

from simulation.mujoco_judge.loop import _should_continue, run_continuous


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


class _FakeViewerHandle:
    """Stands in for mujoco.viewer's Handle: `is_running()` mirrors what a
    user closing the viewer window does (per mujoco's own examples), without
    needing a real GUI. Returns True for `running_iterations` calls, then
    False forever -- so the loop runs that many full iterations before the
    "window close" is observed.
    """

    def __init__(self, running_iterations: int):
        self._remaining = running_iterations
        self.close_called = False

    def is_running(self) -> bool:
        if self._remaining > 0:
            self._remaining -= 1
            return True
        return False

    def sync(self) -> None:
        pass

    def close(self) -> None:
        self.close_called = True


def test_should_continue_stops_at_max_iterations_with_no_viewer():
    assert _should_continue(iteration=2, max_iterations=3, viewer_cm=None) is True
    assert _should_continue(iteration=3, max_iterations=3, viewer_cm=None) is False


def test_should_continue_stops_when_viewer_reports_not_running():
    viewer = _FakeViewerHandle(running_iterations=1)
    # max_iterations=None (the production/infinite case) -- only the
    # viewer's is_running() can stop the loop here.
    assert _should_continue(iteration=0, max_iterations=None, viewer_cm=viewer) is True
    assert _should_continue(iteration=1, max_iterations=None, viewer_cm=viewer) is False


def test_run_continuous_stops_when_viewer_window_closes(tmp_path, model, data, monkeypatch):
    """Simulates closing the viewer window (is_running() -> False) rather
    than relying on SIGINT/Ctrl+C, which doesn't reliably interrupt this
    loop under mjpython on macOS. Also confirms the try/finally cleanup
    path (viewer_cm.close()) actually runs when the loop stops this way.
    """
    fake_viewer = _FakeViewerHandle(running_iterations=2)
    monkeypatch.setattr(
        "mujoco.viewer.launch_passive", lambda model, data: fake_viewer
    )
    out_path = tmp_path / "pairs.jsonl"

    pairs_written = run_continuous(
        model,
        data,
        out_path,
        max_iterations=None,  # would loop forever without the viewer-closed check
        launch_viewer=True,
        frame_delay_seconds=0.0,
    )

    assert pairs_written == 2
    assert fake_viewer.close_called is True

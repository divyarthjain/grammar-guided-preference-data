import time

from simulation.mujoco_judge.judge import MuJoCoJudge, Verdict
from simulation.mujoco_judge.pairs import make_pair, write_pair
from simulation.mujoco_judge.sampler import stubbed_candidates


def _should_continue(iteration: int, max_iterations: int | None, viewer_cm) -> bool:
    """Stop condition for the main loop: keep going unless `max_iterations`
    has been reached, or a live viewer reports it's no longer running (the
    user closed the window). `SIGINT`/Ctrl+C does *not* reliably interrupt
    this loop under mjpython on macOS -- the Python interpreter runs on a
    non-main OS thread while mjpython's Cocoa run loop owns the main
    thread, so a delivered SIGINT can sit pending indefinitely instead of
    raising KeyboardInterrupt. Checking `is_running()` gives the loop a
    stop path that goes through normal Python control flow (so the
    `try/finally` viewer cleanup in `run_continuous` still runs), which is
    the documented way MuJoCo's own examples handle this.
    """
    if max_iterations is not None and iteration >= max_iterations:
        return False
    if viewer_cm is not None and not viewer_cm.is_running():
        return False
    return True


def run_continuous(
    model,
    data,
    out_path,
    max_iterations: int | None = None,
    launch_viewer: bool = True,
    frame_delay_seconds: float = 1.0,
) -> int:
    """Runs the sampler -> judge -> pair-writer loop. If `launch_viewer`,
    opens a live MuJoCo viewer so each candidate attempt can be watched;
    `frame_delay_seconds` paces iterations so a human can follow along.
    Loops forever if `max_iterations` is None (the production entry
    point); tests pass a small bound and `launch_viewer=False`. When a
    viewer is active, closing its window also stops the loop (see
    `_should_continue`) -- that's the reliable way to stop the production
    entry point, since Ctrl+C does not interrupt it on macOS/mjpython.
    """
    judge = MuJoCoJudge(model, data)
    pairs_written = 0
    iteration = 0

    viewer_cm = None
    if launch_viewer:
        import mujoco.viewer

        viewer_cm = mujoco.viewer.launch_passive(model, data)

    try:
        while _should_continue(iteration, max_iterations, viewer_cm):
            candidates = stubbed_candidates()
            chosen = None
            rejected = None
            for candidate in candidates:
                verdict = judge.check(candidate)
                if viewer_cm is not None:
                    viewer_cm.sync()
                if verdict == Verdict.CHOSEN and chosen is None:
                    chosen = candidate
                elif verdict == Verdict.REJECTED and rejected is None:
                    rejected = candidate
                if frame_delay_seconds:
                    time.sleep(frame_delay_seconds)

            if chosen is not None and rejected is not None:
                pair = make_pair("frame_stub.png", "describe the scene", chosen, rejected)
                write_pair(out_path, pair)
                pairs_written += 1

            iteration += 1
    finally:
        if viewer_cm is not None:
            viewer_cm.close()

    return pairs_written

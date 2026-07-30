import time

from simulation.mujoco_judge.judge import MuJoCoJudge, Verdict
from simulation.mujoco_judge.pairs import make_pair, write_pair
from simulation.mujoco_judge.sampler import stubbed_candidates


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
    point); tests pass a small bound and `launch_viewer=False`.
    """
    judge = MuJoCoJudge(model, data)
    pairs_written = 0
    iteration = 0

    viewer_cm = None
    if launch_viewer:
        import mujoco.viewer

        viewer_cm = mujoco.viewer.launch_passive(model, data)

    try:
        while max_iterations is None or iteration < max_iterations:
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

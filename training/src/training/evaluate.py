"""Before/after evaluation on a fixed held-out test set (team doc §9).

Not yet implemented — this is where a checkpoint gets scored on grounding
accuracy, parse-failure rate, and confidence calibration.
"""

from __future__ import annotations

from pathlib import Path


def evaluate_checkpoint(checkpoint_path: Path, test_set_path: Path) -> dict:
    raise NotImplementedError(
        "held-out evaluation is future work — see team doc §9"
    )

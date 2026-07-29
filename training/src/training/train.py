"""The short DPO-style "decay stage" micro-anneal (team doc §3.4, §5.5).

Not yet implemented — this is where the accumulated preference pairs get
turned into a short fine-tuning pass on the base model.
"""

from __future__ import annotations

from pathlib import Path

from training.data import PreferencePair


def run_micro_anneal(pairs: list[PreferencePair], checkpoint_path: Path):
    raise NotImplementedError(
        "the DPO decay-stage micro-anneal is future work — see team doc §3.4/§5.5"
    )

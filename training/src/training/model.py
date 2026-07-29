"""Loads the base VLM checkpoint for the micro-anneal.

Model choice is an open question, not yet settled — see the "Model choice
(open question)" section of docs/architecture.md. Not yet implemented.
"""

from __future__ import annotations

from pathlib import Path


def load_base_model(checkpoint_path: Path):
    raise NotImplementedError(
        "loading the base GGUF checkpoint is future work — see docs/architecture.md"
    )

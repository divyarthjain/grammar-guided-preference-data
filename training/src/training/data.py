"""Loads preference pairs from data/preference_pairs/*.jsonl (team doc §5.4).

This is the sole interface between the Rust runtime and this Python
package — see docs/architecture.md.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass
class PreferencePair:
    image_ref: str
    prompt: str
    chosen: dict
    rejected: dict
    timestamp: str


def load_pairs(path: Path) -> list[PreferencePair]:
    pairs = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        pairs.append(PreferencePair(**json.loads(line)))
    return pairs

import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from training.data import PreferencePair


def make_pair(image_ref: str, prompt: str, chosen: dict, rejected: dict) -> PreferencePair:
    return PreferencePair(
        image_ref=image_ref,
        prompt=prompt,
        chosen=chosen,
        rejected=rejected,
        timestamp=datetime.now(timezone.utc).isoformat(),
    )


def write_pair(path: Path, pair: PreferencePair) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as f:
        f.write(json.dumps(asdict(pair)) + "\n")

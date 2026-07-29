from pathlib import Path

from training.data import PreferencePair, load_pairs

FIXTURE_PATH = Path(__file__).parent / "fixtures" / "sample_pairs.jsonl"


def test_load_pairs_parses_fixture():
    pairs = load_pairs(FIXTURE_PATH)

    assert len(pairs) == 2
    assert all(isinstance(pair, PreferencePair) for pair in pairs)

    first = pairs[0]
    assert first.image_ref == "frame_00001.png"
    assert first.chosen["object"] == "red block"
    assert first.rejected["action_type"] == "avoid"

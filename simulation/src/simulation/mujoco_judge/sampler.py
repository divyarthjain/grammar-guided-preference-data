"""Stub multi-sample generation (team doc §5.2), mirroring
runtime/crates/orchestrator/src/sampler.rs — no real VLM call yet.
"""


def stubbed_candidates() -> list[dict]:
    return [
        {
            "object": "red block",
            "bbox": {"x": 300.0, "y": 220.0, "width": 40.0, "height": 40.0},
            "confidence": 0.92,
            "action_type": "grasp",
        },
        {
            "object": "table edge",
            "bbox": {"x": 0.0, "y": 0.0, "width": 10.0, "height": 10.0},
            "confidence": 0.4,
            "action_type": "avoid",
        },
    ]

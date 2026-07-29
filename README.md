# grammar-guided-preference-data

Grammar-guided synthetic preference data for cheap self-improvement in
robot perception. A VLM ("MiniCPM-V 4.6" — working assumption, grounding
unverified, see [`docs/architecture.md`](docs/architecture.md)) proposes
candidate scene descriptions for each camera frame, a grammar constrains
its output to a fixed JSON schema, and the robot's own control stack
(IK feasibility + Ruckig trajectory safety) automatically labels each
candidate CHOSEN/REJECTED — building a DPO preference dataset with zero
human labeling, used for periodic cheap "micro-anneal" fine-tunes.

Start at [`docs/index.md`](docs/index.md) for the full picture.

## Layout

- `runtime/` — Rust workspace: the on-device real-time loop (targets
  Raspberry Pi, aarch64).
- `training/` — Python package: the offline DPO micro-anneal (dev
  machine, not the Pi).
- `data/preference_pairs/` — shared JSONL, the only interface between
  the two.
- `docs/` — documentation bundle, start here.

## Quick start

```
cd runtime && cargo test --workspace
cd training && uv run pytest
```

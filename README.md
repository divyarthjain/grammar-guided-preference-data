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
- `simulation/` — Python package: a MuJoCo-based training-time physical
  judge with a live viewer, feeding `data/preference_pairs/*.jsonl`
  alongside the Rust runtime's judge.
- `data/preference_pairs/` — shared JSONL, the interface between the
  runtime and training/simulation components.
- `docs/` — documentation bundle, start here.

## Quick start

```
cd runtime && cargo test --workspace
cd training && uv run pytest
cd simulation && uv run pytest
```

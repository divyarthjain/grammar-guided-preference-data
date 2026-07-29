---
type: Reference
title: Architecture
description: The Rust/Python split, why, and the current model-choice open question
tags: [architecture, meta]
status: draft
generated: { by: claude-code/claude-sonnet-5, at: 2026-07-30T00:00:00Z }
---

# The loop

Camera frame → VLM generates several candidate scene descriptions per
frame (multi-sample, varied temperature) → each candidate is forced by a
grammar into the fixed JSON schema (see [grammar-schema.md](grammar-schema.md))
→ the [physical judge](physical-judge.md) checks IK feasibility and Ruckig
trajectory safety → CHOSEN/REJECTED candidates become a DPO preference
pair → pairs accumulate into `data/preference_pairs/*.jsonl` → a periodic
short "micro-anneal" fine-tune specializes the model → the updated model
is redeployed, and the loop continues. See the team reference doc
(`Grammar_Guided_Preference_Data_Team_Doc.docx`) for the full rationale.

# The Rust/Python split

- **`runtime/`** (Rust) — the on-device real-time loop: inference
  orchestration, grammar validation, the physical judge, preference-pair
  logging. Targets the Raspberry Pi (CPU-only, aarch64). Rust was chosen
  over C++ for memory safety on hardware-driving code and easy static
  cross-compilation to the Pi, and over Python for the loop to avoid
  interpreter/GIL overhead in the real-time path.
- **`training/`** (Python, PyTorch/HF TRL) — the offline DPO micro-anneal.
  Runs on a dev machine, not the Pi. Python was kept here because Rust's
  ML training tooling isn't mature enough to bet a capstone timeline on
  reimplementing DPO from scratch.
- The only interface between them is `data/preference_pairs/*.jsonl` —
  neither side imports the other's code.

# The judge as a pluggable interface

The real IK solver and Ruckig integration don't exist yet. The `judge`
crate defines a `PhysicalJudge` trait; `MockJudge` implements it today
with fake (alternating) pass/fail behavior so the rest of the pipeline
can be built and tested now. `judge-ffi` is an empty placeholder crate
reserved for the future `cxx`-based Ruckig/IK FFI bindings. See
[physical-judge.md](physical-judge.md).

# Model choice (open question)

**MiniCPM-V 4.6 is the working assumption, not a settled decision.**
Research into whether it actually fits this pipeline found:

- It has the best-documented, currently-official llama.cpp `mtmd`
  support of any small VLM checked, and a real published vision-DPO
  lineage (RLAIF-V) that matches the planned micro-anneal.
- However, OpenBMB has never published a RefCOCO or any other
  visual-grounding/bbox benchmark for any MiniCPM-V version, despite
  vague marketing claims of grounding parity. Since `bbox` is the
  load-bearing field feeding the physical feasibility gate, this is a
  real gap, not nitpicking.

**Moondream2** (vikhyatk, Apache 2.0) is the evaluated fallback: official
llama.cpp support (hosted under `ggml-org` itself), native point/detect
grounding modes with an actual reported accuracy figure — but no
published DPO recipe, so the micro-anneal would need TRL's `DPOTrainer`
wired up manually rather than reusing a published recipe.

**PaliGemma2** has the strongest grounding evidence of all (real RefCOCO
mIoU ~73-77%, purpose-built for this) but no llama.cpp support today
(feature requests open since 2024, unmerged) — not usable as-is.

**Plan to settle this:** run a small empirical bbox-accuracy test on real
ArcheoHex camera frames comparing MiniCPM-V-4.6 and Moondream2 before
committing further. Nothing in `runtime/` currently depends on a specific
model — the scaffold is model-agnostic.

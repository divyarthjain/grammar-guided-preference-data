---
type: Reference
title: Physical judge
description: What the judge is, MockJudge today, and the swap-in point for a real judge later
tags: [judge, ik, ruckig]
status: draft
generated: { by: claude-code/claude-sonnet-5, at: 2026-07-30T00:00:00Z }
---

# What it is

The automatic physical judge (team doc §5.3) replaces a human labeler
entirely. For each candidate answer, instead of asking a person "is this
a good suggestion?", the pipeline asks the robot's own control software:

- **IK check** — given the suggested target, can the robot's arm/legs
  actually reach that position? Does the solver converge, or fail?
- **Ruckig check** — if a smooth trajectory were generated toward that
  target, does it stay within joint limits and produce a safe,
  non-jerky motion?

If both checks pass, the candidate is labeled CHOSEN. If either fails,
REJECTED. This is fully automatic and reuses control-stack code that
would otherwise already exist for the robot's three-tier architecture.

# The pluggable interface

`runtime/crates/judge` defines:

```rust
trait PhysicalJudge { fn check(&self, candidate: &Candidate) -> Verdict; }
```

`MockJudge` implements this today with fake (alternating Chosen/Rejected)
behavior — the real IK solver and Ruckig integration don't exist yet.
Any caller (the orchestrator binary, tests) only ever depends on the
trait, not on `MockJudge` specifically, so swapping in a real judge later
means: implement `PhysicalJudge` in a new crate (or in `judge-ffi` once
it has real bindings), and change the one line where the binary
constructs its judge instance. No other code changes.

`runtime/crates/judge-ffi` is an empty placeholder reserved for the
future `cxx`-based Ruckig/IK FFI bindings — it builds successfully today
but contains no logic, marking where that integration will go.

# Known limitation

The judge is a **feasibility** signal, not a full **correctness**
signal — IK solving doesn't guarantee an action is the *correct* thing
to do, just that it's *possible*. See the Risks section of the team doc
for mitigations under consideration (e.g. geometric plausibility checks
as an extra filter).

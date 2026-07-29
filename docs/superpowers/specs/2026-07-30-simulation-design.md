# Simulation design: MuJoCo training viewer + Unity checkpoint recorder

Date: 2026-07-30
Status: approved

## Purpose

Add two watchable simulations, for two different moments in the pipeline:

1. **Training-time simulation** — a continuous, always-on visual feedback
   loop while the training-data-generation loop runs, so candidate
   actions can be watched being physically attempted (not just inferred
   from a loss number going up).
2. **Deployment/checkpoint simulation** — a periodic, higher-fidelity
   recording of a trained checkpoint acting in a simulated environment,
   specifically to produce comparable video clips (checkpoint N vs.
   checkpoint N+10) for a YouTube video documenting the project.

This followed a debate about Gazebo (poor/unsupported on macOS,
especially Apple Silicon) vs. Unity+ROS (the user's initial idea) vs.
other options. Conclusion: use two different tools for the two different
jobs rather than one tool for both, since they have different priorities
(physical accuracy for the training judge vs. visual polish for the
demo), and skip ROS as a middleware layer — nothing in the existing
architecture (Rust `runtime/`, Python `training/`) assumed ROS, and
bridging Rust into a ROS graph would add an integration seam without a
concrete need it solves that direct scripting doesn't already cover.

## Decisions from brainstorming

- **Training-time physics: MuJoCo**, not Unity. The judge's job is
  checking contact-rich leg-ground feasibility for a 6-legged robot;
  MuJoCo is the standard tool for this in legged-locomotion work, runs
  natively and fast on Apple Silicon (no VM, no bridge), and has a live
  Python viewer (`mujoco.viewer.launch_passive`) — exactly the
  "watch it happen continuously" requirement.
- **Deployment demo: Unity**, not MuJoCo. Unity's rendering quality and
  Recorder package are the better fit for a polished, comparable video
  artifact meant for an audience (YouTube / a capstone committee), and
  Unity's Perception package is a bonus for visualizing predicted vs.
  ground-truth bboxes later, tying back to the still-open VLM-grounding
  question from the prior design spec.
- **No ROS.** Neither simulation goes through ROS middleware. MuJoCo is
  driven directly via its Python API. Unity is driven by reading a
  batch replay file, not a live ROS-style bridge (see below).
- **Robot model:** no ArcheoHex CAD exists yet, and the model the user
  found (`Ryan-Mirch/Aecerts_Hexapod_V1`) is raw 3D-printable CAD (STLs)
  with no URDF/MJCF/kinematic tree and no license — not usable as-is,
  legally or technically. Instead: vendor
  `HumaRobotics/phantomx_description` (confirmed BSD-Simplified
  licensed via its actual LICENSE file; contains `urdf/` + `meshes/`,
  a real, simulation-ready hexapod already used in ROS/Gazebo tutorials)
  as a stand-in, swapped for real ArcheoHex CAD later.
- **Checkpoint cadence:** one checkpoint recording per completed
  micro-anneal run (though see the scope note below — `train.py` isn't
  implemented yet, so this trigger is manual for now).
- **Video comparability:** every checkpoint recording uses the same
  fixed test scenario(s) and camera angle, reusing the fixed held-out
  test set concept from the original team doc (§9) — one source of
  truth for both evaluation metrics and video comparison, not two.
- **Unity trigger mechanism:** a batch replay file, not a live socket
  bridge. Checkpoint recording is inherently periodic/offline, so a
  persistent connection buys nothing here that a plain JSON file
  doesn't already provide, and it avoids reinventing a ROS-TCP-Connector
  -shaped bridge for no concrete benefit.
- **Where the new code lives:** a new top-level `simulation/` directory,
  sibling to `runtime/` and `training/` — not folded into `training/`.
  MuJoCo's job (simulate physical feasibility, generate preference
  pairs) is conceptually distinct from `train.py`'s job (anneal a
  checkpoint on already-generated pairs), the same reasoning that kept
  the Rust `judge` crate separate from `orchestrator`.

## Layout

```
simulation/
  models/
    phantomx/            # vendored HumaRobotics/phantomx_description (BSD), urdf/ + meshes/
  mujoco_judge/            # Python package (own pyproject.toml, uv-managed)
    pyproject.toml
    src/mujoco_judge/
      __init__.py
      judge.py              # loads the PhantomX URDF into MuJoCo; given a candidate
                              # action, drives the simulated legs toward it and reports
                              # success/failure (reachability, joint limits, self-collision,
                              # toppling) — a physically-grounded PhysicalJudge equivalent
      sampler.py              # stubbed candidate proposals, same pattern as
                                # runtime/crates/orchestrator/src/sampler.rs — no real VLM yet
      loop.py                   # the continuous loop: sampler -> judge -> write to
                                  # data/preference_pairs/*.jsonl, with the MuJoCo viewer open
    tests/
      test_judge.py
  replay/
    pyproject.toml (or folded into mujoco_judge — decide at implementation time)
    record_checkpoint.py       # runs the fixed test scenario(s) through a checkpoint +
                                 # mujoco_judge, writes a replay file
    schema.py                    # the replay-file format: scenario id, fixed camera,
                                   # per-frame {leg joint angles, camera pose, predicted bbox}
    tests/
      test_record_checkpoint.py
  unity/
    # a Unity project: PhantomX URDF imported via URDF-Importer, a C# script
    # that reads a replay file and steps the rig through it frame-by-frame,
    # Unity Recorder exports the result as an MP4. Not unit-testable the way
    # Python/Rust are — verified manually (open Unity, run it, confirm an
    # MP4 comes out).
```

`data/preference_pairs/*.jsonl` remains the sole interface between
`simulation/mujoco_judge/` and `training/` — the same file the Rust
`runtime/` also writes to, per the original design. The replay-file
format is the sole interface between `training/` (checkpoints) and
`simulation/unity/` — Unity never reads Python code, and the Python side
never reads Unity project files.

## Scope discipline

This spec adds real, working code for: MuJoCo-backed physical
feasibility checking, the continuous live viewer, the replay-file format
and writer, and Unity's playback+recording script. It does **not**
implement: real VLM sampling (still stubbed, matching the existing
`orchestrator`/training scaffolding), the real DPO anneal in
`train.py` (still `NotImplementedError` from the prior spec), or
automatic checkpoint-triggered recording (manual invocation for now,
since there's no real anneal event to hook into yet). Real ArcheoHex
CAD integration remains future work — the PhantomX model is an explicit
stand-in, not a placeholder pretending to be final.

## Testing

- `mujoco_judge`: unit tests with a scripted candidate — a reachable
  target passes, a deliberately unreachable one fails.
- `replay`: a test that running `record_checkpoint.py` against a stub
  checkpoint produces a well-formed replay file matching the schema.
- Unity: manual verification only (open the project, run it, confirm an
  MP4 is produced) — not part of any automated test suite.

## Out of scope (unchanged from the prior spec, plus this feature)

- Real VLM integration, real DPO training loop, real IK/Ruckig hardware
  judge, real ArcheoHex CAD.
- ROS as middleware, in any form.
- Automatic checkpoint-triggered recording (until `train.py` is real).

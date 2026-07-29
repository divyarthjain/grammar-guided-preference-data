# Repo scaffold design — grammar-guided-preference-data

Date: 2026-07-29
Status: approved

## Purpose

Scaffold the repository for the capstone project described in
`Grammar_Guided_Preference_Data_Team_Doc.docx`: a self-improvement loop
where a small vision-language model (MiniCPM-V 4.6) generates multiple
candidate scene descriptions per camera frame, a grammar (GBNF) forces
those into a fixed JSON schema, and the robot's own control stack (IK
feasibility + Ruckig trajectory safety) automatically judges each
candidate as CHOSEN or REJECTED — producing DPO preference pairs with
zero human labeling. Periodically, a short "micro-anneal" (decay-stage
fine-tune) specializes the model on the accumulated pairs.

This spec covers only the repo scaffold — directory layout, language
split, and the interfaces between pieces — not the implementation of
any component. See the team doc for full pipeline rationale.

## Constraints and decisions established during brainstorming

- **Deployment target:** Raspberry Pi (CPU-only, aarch64). The on-device
  loop must be fast without a GPU.
- **Language split:** Rust for the on-device real-time loop (inference
  orchestration via llama.cpp, grammar validation, physical judge,
  preference-pair logging); Python (PyTorch/HF TRL) for the offline DPO
  micro-anneal, which runs on a dev machine, not the Pi. Chosen over
  Rust-only (ML training tooling in Rust is immature — would mean
  hand-implementing DPO) and Python-only (loses the deployability and
  real-time-loop benefits of Rust on constrained hardware).
- **Control stack status:** the real IK solver and Ruckig integration do
  not exist yet ("not sure" when asked). The judge must therefore be a
  pluggable interface (Rust trait) with a mock implementation now, so a
  real implementation can be dropped in later without changing callers.
- **Single repo, two subdirectories** (`runtime/`, `training/`) rather
  than two repos — one place for the team to work, one git history.
- **Agent-readability:** the repo should be easy for an LLM agent to
  read and act on. Addressed via a single OKF v0.2 documentation bundle
  at the repo root (not per-subdirectory) that any agent reads first,
  regardless of which language it's about to touch.
- **Personal tracking:** this project also gets a project entry in the
  user's existing `~/second-brain` OKF v0.2 bundle, separate from the
  repo's own docs, so it's discoverable alongside their other projects.
- **Git/GitHub:** local git, then a public GitHub repo
  (`divyarthjain/grammar-guided-preference-data`) via `gh`, Apache 2.0
  license.

## Top-level layout

```
grammar-guided-preference-data/
  runtime/          # Rust workspace — on-device real-time loop (targets aarch64 / Raspberry Pi)
  training/          # Python package — offline DPO micro-anneal (dev machine, not the Pi)
  data/
    preference_pairs/   # shared JSONL — written by runtime/, read by training/
  docs/              # OKF v0.2 bundle — single entry point for understanding the whole repo
  README.md
  .gitignore
  LICENSE            # Apache 2.0
```

`data/preference_pairs/*.jsonl` is the *only* interface between the two
languages. Neither side imports or depends on the other.

## `runtime/` — Rust workspace

```
runtime/
  Cargo.toml                 # [workspace] members = crates/*
  crates/
    grammar/
      Cargo.toml
      src/lib.rs              # GBNF schema (object, bbox, confidence, action_type);
                                # validates/generates the grammar used to constrain model output
    judge/
      Cargo.toml
      src/lib.rs               # trait PhysicalJudge { fn check(&self, candidate: &Candidate) -> Verdict }
                                 # struct MockJudge implements PhysicalJudge with fake pass/fail,
                                 # no hardware or IK/Ruckig dependency required
    judge-ffi/
      Cargo.toml                # placeholder crate only — builds, does nothing yet
      src/lib.rs                 # doc comment marking the future cxx-based Ruckig/IK FFI bindings
    orchestrator/
      Cargo.toml
      src/main.rs                # binary: camera frame -> multi-sample generation (llama.cpp,
                                   # varied temperature) -> grammar::validate -> judge::check
                                   # (wired to MockJudge today) -> pair logging -> data/preference_pairs/*.jsonl
```

- `orchestrator` (the binary crate — named separately from the
  `runtime/` directory to avoid a crate literally called `runtime`
  nested inside `runtime/`) depends on `grammar` and `judge`. It is
  wired to `MockJudge` via its constructor today. Swapping in a real
  judge later means: implement `PhysicalJudge` in a new crate (or in
  `judge-ffi` once it has real bindings), change the one line where the
  binary constructs its judge instance. No other code changes.
- `judge-ffi` exists now only as a visible placeholder for the future
  Ruckig/IK C++ integration (via the `cxx` crate) — it is not built out
  until the control stack exists.
- llama.cpp is not vendored. `runtime` links against it via the
  `llama-cpp-2` crate (wraps llama.cpp's C API). llama.cpp itself is
  built/installed separately, and the GBNF grammar is tested standalone
  via `llama-mtmd-cli` first, per the team doc's phase-1 "done"
  criteria.

## `training/` — Python package

```
training/
  pyproject.toml
  src/training/
    __init__.py
    data.py         # loads data/preference_pairs/*.jsonl into (chosen, rejected) pairs
    model.py         # loads the base MiniCPM-V GGUF checkpoint for the anneal
    train.py          # short DPO-style training pass (the "decay stage" recipe, team doc §3.4/§5.5)
    evaluate.py         # before/after comparison on the fixed held-out test set (team doc §9)
  tests/
    test_data.py
```

### `data/preference_pairs/*.jsonl` schema

One JSON object per line:

```json
{
  "image_ref": "frame_00123.png",
  "prompt": "...",
  "chosen": { "object": "...", "bbox": [0, 0, 0, 0], "confidence": 0.0, "action_type": "grasp" },
  "rejected": { "object": "...", "bbox": [0, 0, 0, 0], "confidence": 0.0, "action_type": "grasp" },
  "timestamp": "2026-07-29T00:00:00Z"
}
```

Real pair files under `data/preference_pairs/` are gitignored (they're
generated by the robot operating, not source). Fixture files under
`training/tests/` are committed so tests are reproducible.

## `docs/` — OKF v0.2 bundle (repo-internal)

One shared bundle at the repo root, covering both subdirectories:

```
docs/
  index.md                    # root index; carries okf_version; links to all concepts below
  log.md                       # dated changelog of what got added/changed
  architecture.md               # the Rust/Python split and why (summary of this spec, kept current)
  grammar-schema.md              # the GBNF schema, explained
  physical-judge.md               # what the judge is; MockJudge today; real-judge swap-in plan
  glossary.md                       # from the team doc §11 (VLM, GBNF, IK, Ruckig, DPO, WSD, RLAIF, etc.)
```

Follows the same OKF v0.2 conventions as `~/second-brain`: YAML
frontmatter (`type`, `title`, `description`, `status`), `index.md` for
progressive disclosure, `log.md` for history. An agent or teammate
opening this repo cold reads `docs/index.md` first.

## Personal tracking (outside the repo)

A new concept file at `~/second-brain/projects/grammar-guided-preference-data.md`
(OKF v0.2, `type: Project`), linking to the local repo path and the
GitHub URL once created, plus a line appended to `~/second-brain/log.md`.
This is separate from `docs/` inside the repo — it's how the project
becomes discoverable alongside the user's other PARA-style projects.

## Testing strategy

- `grammar` crate: unit tests feeding valid and deliberately malformed
  model output through the validator; asserts malformed output is
  always rejected (ties to the team doc's "parse-failure rate should be
  ~0%" evaluation goal).
- `judge` crate: unit tests on `MockJudge`'s pass/fail behavior.
- `orchestrator` binary crate: one integration test wiring a stubbed
  sampler → `grammar` → `MockJudge` → logging, asserting a valid JSONL
  line lands in a temp `data/preference_pairs/` directory.
- `training/tests/test_data.py`: loads a small fixture JSONL, asserts
  pairs parse correctly. No live training run in tests — that needs
  real weights and compute.

## Out of scope for this scaffold

- Implementing the actual GBNF grammar content, sampling logic, real
  judge, or DPO training loop — this spec is directory structure and
  interfaces only.
- CI configuration.
- The real IK/Ruckig control-stack integration (`judge-ffi` stays an
  empty placeholder until that stack exists).

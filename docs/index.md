---
okf_version: "0.2"
---

# grammar-guided-preference-data — docs

An [OKF](https://okf.dev) v0.2 documentation bundle. Start here regardless
of which part of the repo (`runtime/`, `training/`, or `simulation/`)
you're about to touch.

* [Architecture](architecture.md) — the Rust/Python split (plus
  `simulation/`, a third MuJoCo-based component), why, and the open
  question on model choice.
* [Grammar schema](grammar-schema.md) — the JSON schema every model
  candidate must match.
* [Physical judge](physical-judge.md) — what the judge is, `MockJudge`
  today, the swap-in point for a real judge later.
* [Glossary](glossary.md) — terms used throughout (VLM, GBNF, IK, Ruckig,
  DPO, RLAIF, etc.).

See [log.md](log.md) for the dated history of this bundle, and
`../docs/superpowers/specs/2026-07-29-repo-scaffold-design.md` for the
full scaffold design spec this bundle was built from.

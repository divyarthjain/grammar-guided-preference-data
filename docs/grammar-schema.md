---
type: Reference
title: Grammar / schema
description: The fixed JSON schema every model candidate must match (team doc §5.1)
tags: [grammar, gbnf, schema]
status: draft
generated: { by: claude-code/claude-sonnet-5, at: 2026-07-30T00:00:00Z }
---

# The schema

```json
{
  "object": "<short text label>",
  "bbox": [x, y, width, height],
  "confidence": 0.00 to 1.00,
  "action_type": "approach" | "avoid" | "grasp" | "inspect" | "none"
}
```

`action_type` is a closed list matching exactly the commands the robot's
locomotion/IK layer already understands — the model literally cannot
suggest an action the robot has no way to execute.

# Two layers of validation

1. **JSON-shape validation** (implemented, `runtime/crates/grammar`) —
   given an already-generated JSON string, checks it matches this shape
   and that `confidence` is in range. This is a safety net, not a
   guarantee — a badly-behaved model could still emit malformed text.
2. **The actual GBNF grammar** (`runtime/crates/grammar/schema.gbnf`,
   **not yet written**) — the grammar file that constrains llama.cpp's
   token-level decoding so malformed output can never be generated in
   the first place. This is phase 1 of the team doc's build plan, tested
   standalone via `llama-mtmd-cli` before it's wired into the rest of
   the pipeline. Not fabricating grammar content here — an honest
   placeholder is preferable to invented rules.

See also the model-choice open question in
[architecture.md](architecture.md): the schema's `bbox` field is only as
useful as the model's actual grounding accuracy, which is currently
unverified for the working-assumption model.

# simulation

MuJoCo-based training-time physical judge for grammar-guided-preference-data
(see `../docs/architecture.md`).

## Setup

```bash
uv sync
```

### macOS one-time step: fix `mjpython`'s shared-library load

The live MuJoCo viewer (`mujoco.viewer.launch_passive`, used by
`python -m simulation.mujoco_judge`) requires macOS scripts to run under
`mjpython` rather than plain `python` (`launch_passive` raises
`RuntimeError: ... requires that the Python script be run under mjpython
on macOS` otherwise). With a `uv`-managed venv, `mjpython` itself fails
to start with:

```
Library not loaded: @executable_path/../lib/libpython3.12.dylib
```

because `uv` venvs don't copy the interpreter's shared library into
`.venv/lib/`. Fix once per fresh `uv sync`/`uv venv` (the venv is a
disposable build artifact, so this doesn't persist across a venv
recreation):

```bash
./scripts/fix_mjpython_dylib.sh
```

Then run the viewer with `mjpython`, not `python`:

```bash
uv run mjpython -m simulation.mujoco_judge
```

See `scripts/fix_mjpython_dylib.sh` for what it does and why.

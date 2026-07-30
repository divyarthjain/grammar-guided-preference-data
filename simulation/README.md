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

**To stop: close the viewer window**, not Ctrl+C. Under mjpython on
macOS the Python interpreter runs on a non-main OS thread while the
Cocoa run loop owns the main thread, so a delivered SIGINT never gets
processed and the loop won't be interrupted. Closing the window makes
the loop stop on its own (checked via the viewer handle's
`is_running()`), which also runs the normal cleanup path. `kill`/`SIGTERM`
works too as a fallback, but skips that cleanup.

See `scripts/fix_mjpython_dylib.sh` for what it does and why.

## Known limitations

`models/phantomx/scene.xml` has no actuators and no joint damping or
friction. The stability check settles the pose as a passive, unpowered
structure under gravity — it verifies the pose itself is mechanically
stable, not that an actuated robot could hold it under real closed-loop
control. Treat CHOSEN/REJECTED verdicts accordingly (see the note in
`stability.py`'s module docstring).

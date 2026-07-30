from pathlib import Path

import mujoco

from simulation.mujoco_judge.constants import SCENE_PATH
from simulation.mujoco_judge.loop import run_continuous

# On macOS, the default `launch_viewer=True` requires this to be run as
# `uv run mjpython -m simulation.mujoco_judge`, not plain `python` --
# `mujoco.viewer.launch_passive` unconditionally raises otherwise. See
# README.md ("macOS one-time step") for the one-time `mjpython` setup
# fix this also depends on.
#
# To stop: close the viewer window (Ctrl+C does not work here -- under
# mjpython on macOS, the Python interpreter runs on a non-main OS thread
# while the Cocoa run loop owns the main thread, so a delivered SIGINT
# never gets processed). Closing the window makes `run_continuous`'s
# loop stop on its own via the viewer's `is_running()` check.
if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    # __main__.py is at simulation/src/simulation/mujoco_judge/__main__.py;
    # parents[4] is the repo root (grammar-guided-preference-data/).
    out_path = Path(__file__).resolve().parents[4] / "data" / "preference_pairs" / "sim_pairs.jsonl"
    run_continuous(model, data, out_path)

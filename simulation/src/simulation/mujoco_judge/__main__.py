from pathlib import Path

import mujoco

from simulation.mujoco_judge.constants import SCENE_PATH
from simulation.mujoco_judge.loop import run_continuous

# On macOS, the default `launch_viewer=True` requires this to be run as
# `uv run mjpython -m simulation.mujoco_judge`, not plain `python` --
# `mujoco.viewer.launch_passive` unconditionally raises otherwise. See
# README.md ("macOS one-time step") for the one-time `mjpython` setup
# fix this also depends on.
if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    # __main__.py is at simulation/src/simulation/mujoco_judge/__main__.py;
    # parents[4] is the repo root (grammar-guided-preference-data/).
    out_path = Path(__file__).resolve().parents[4] / "data" / "preference_pairs" / "sim_pairs.jsonl"
    run_continuous(model, data, out_path)

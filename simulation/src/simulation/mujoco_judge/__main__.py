from pathlib import Path

import mujoco

from simulation.mujoco_judge.constants import SCENE_PATH
from simulation.mujoco_judge.loop import run_continuous

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    # __main__.py is at simulation/src/simulation/mujoco_judge/__main__.py;
    # parents[4] is the repo root (grammar-guided-preference-data/).
    out_path = Path(__file__).resolve().parents[4] / "data" / "preference_pairs" / "sim_pairs.jsonl"
    run_continuous(model, data, out_path)

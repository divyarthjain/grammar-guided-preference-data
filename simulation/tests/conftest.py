import mujoco
import pytest

from simulation.mujoco_judge.constants import SCENE_PATH


@pytest.fixture
def model():
    return mujoco.MjModel.from_xml_path(str(SCENE_PATH))


@pytest.fixture
def data(model):
    return mujoco.MjData(model)

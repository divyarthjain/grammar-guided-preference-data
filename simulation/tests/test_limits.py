import numpy as np

from simulation.mujoco_judge.limits import within_joint_limits


def test_zero_qpos_is_within_limits(model):
    assert within_joint_limits(model, np.array([0.0, 0.0, 0.0]))


def test_out_of_range_qpos_is_rejected(model):
    assert not within_joint_limits(model, np.array([0.0, 0.0, 10.0]))

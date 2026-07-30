import numpy as np

from simulation.mujoco_judge.constants import BASE_BODY
from simulation.mujoco_judge.stability import settle_and_check_stability


def test_default_pose_is_stable(model, data):
    assert settle_and_check_stability(model, data)


def test_fallen_base_is_unstable(model, data):
    base_id = model.body(BASE_BODY).id
    free_joint_qpos_adr = model.jnt_qposadr[model.body_jntadr[base_id]]
    # Free joint qpos layout is [x, y, z, qw, qx, qy, qz]. Force the base
    # to a position at ground level, tipped 90 degrees onto its side.
    data.qpos[free_joint_qpos_adr : free_joint_qpos_adr + 7] = [
        0.0, 0.0, 0.005,
        0.7071, 0.7071, 0.0, 0.0,
    ]
    assert not settle_and_check_stability(model, data)

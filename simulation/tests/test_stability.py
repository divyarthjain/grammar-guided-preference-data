import mujoco
import numpy as np

from simulation.mujoco_judge.constants import BASE_BODY
from simulation.mujoco_judge.stability import (
    _has_self_collision,
    settle_and_check_stability,
)


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


def _inject_contact(model, data, body1: int, body2: int) -> None:
    """Force a single synthetic contact between a geom of body1 and a geom
    of body2 into data, bypassing real collision detection. This model's
    convex collision meshes don't collide for every body pair we want to
    exercise (e.g. two different legs never actually touch under joint
    limits), so this directly tests _has_self_collision's kinematic-hop
    filtering in isolation.
    """
    geom1 = int(np.flatnonzero(model.geom_bodyid == body1)[0])
    geom2 = int(np.flatnonzero(model.geom_bodyid == body2)[0])
    mujoco.mj_forward(model, data)
    data.ncon = 1
    data.contact[0].geom1 = geom1
    data.contact[0].geom2 = geom2


def test_same_leg_coxa_tibia_contact_is_not_self_collision(model, data):
    # 2 hops apart (coxa -> thigh -> tibia, same leg): this is the benign
    # rest-pose mesh overlap that motivated the kinematic-hop filter.
    c1_rf = model.body("c1_rf").id
    tibia_rf = model.body("tibia_rf").id
    _inject_contact(model, data, c1_rf, tibia_rf)
    assert not _has_self_collision(model, data)


def test_chassis_to_other_leg_contact_is_self_collision(model, data):
    # 3 hops apart (chassis -> coxa -> thigh -> tibia of a leg the chassis
    # isn't directly jointed to at that point): a leg buckled into the
    # chassis must still be caught.
    base_id = model.body(BASE_BODY).id
    tibia_lm = model.body("tibia_lm").id
    _inject_contact(model, data, base_id, tibia_lm)
    assert _has_self_collision(model, data)


def test_cross_leg_contact_is_self_collision(model, data):
    # Two different legs' tibiae touching (6 hops apart) must be caught.
    tibia_rf = model.body("tibia_rf").id
    tibia_lm = model.body("tibia_lm").id
    _inject_contact(model, data, tibia_rf, tibia_lm)
    assert _has_self_collision(model, data)

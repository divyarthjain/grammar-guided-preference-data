"""Post-IK stability check: settle the pose under gravity and see if it holds.

Fidelity note: `models/phantomx/scene.xml` has no `<actuator>` elements and
no joint damping/frictionloss. That means the settle step below
(`mj_forward` + repeated `mj_step`) evaluates whether a fully passive,
unpowered structure — positioned once by kinematic IK and then held by
nothing — stays upright under gravity alone. It does not evaluate whether
an actuated robot could actually hold that pose under its own closed-loop
control. That's a real fidelity gap relative to the "physically-grounded
stand-in for the real IK+Ruckig judge" framing in `judge.py`'s module
docstring, worth keeping in mind when reading CHOSEN/REJECTED verdicts as
a proxy for real-robot feasibility.
"""

import mujoco
import numpy as np

from simulation.mujoco_judge.constants import BASE_BODY, FLOOR_GEOM


def settle_and_check_stability(
    model,
    data,
    hold_steps: int = 100,
    max_tilt_deg: float = 25.0,
    min_base_height: float = 0.02,
) -> bool:
    mujoco.mj_forward(model, data)
    for _ in range(hold_steps):
        mujoco.mj_step(model, data)

    base_id = model.body(BASE_BODY).id
    base_height = data.xpos[base_id][2]

    xmat = data.xmat[base_id].reshape(3, 3)
    body_up = xmat @ np.array([0.0, 0.0, 1.0])
    tilt_deg = np.degrees(np.arccos(np.clip(body_up[2], -1.0, 1.0)))

    if base_height < min_base_height or tilt_deg > max_tilt_deg:
        return False
    return not _has_self_collision(model, data)


# Bodies up to this many hops apart in the kinematic tree (e.g. a leg's own
# coxa and tibia, 2 hops via the thigh) have collision meshes that overlap
# slightly by design, even in a normal standing pose, and are not treated as
# a self-collision. Bodies farther apart than this (a leg buckled into the
# chassis, or into a different leg) are.
#
# Known interaction with ik.py's limitation: this hop-distance check is an
# undirected graph metric — it clears any pair of bodies within
# _MAX_BENIGN_HOPS of each other in the kinematic tree, not specifically
# "this leg's own coxa/tibia at rest". solve_leg_ik only actually drives the
# front-right leg (3 joints, but only 2 effective DOF — see ik.py), so in
# principle a large enough swing of that leg's coxa could bring it into
# genuine contact with a neighboring leg's mount, and if that neighboring
# body happens to sit within _MAX_BENIGN_HOPS of the swinging body (a
# same-leg-shaped pair by this undirected metric), this filter would wrongly
# clear it as benign self-overlap instead of flagging a real collision.
# In practice this is masked today because IK non-convergence (the `if not
# converged: return REJECTED` check in judge.py, upstream of this function)
# rejects most large-excursion targets before the collision check ever runs,
# and `stubbed_candidates()` in sampler.py only ever produces small,
# in-workspace targets. That's a property of the current stub sampler, not a
# structural guarantee. Revisit this (tighten to a same-leg-only / directed
# adjacency check, or add a regression test that exercises a large swing)
# before a real (non-stub) sampler replaces stubbed_candidates().
_MAX_BENIGN_HOPS = 2


def _has_self_collision(model, data) -> bool:
    floor_id = model.geom(FLOOR_GEOM).id
    for i in range(data.ncon):
        contact = data.contact[i]
        if contact.geom1 == floor_id or contact.geom2 == floor_id:
            continue
        body1 = model.geom_bodyid[contact.geom1]
        body2 = model.geom_bodyid[contact.geom2]
        if body1 == body2:
            continue
        if _hop_distance(model, body1, body2) <= _MAX_BENIGN_HOPS:
            continue
        return True
    return False


def _hop_distance(model, body1: int, body2: int) -> int:
    """Number of joint hops between two bodies in the kinematic tree."""
    depths = {}
    current = body1
    depth = 0
    while True:
        depths[current] = depth
        if current == 0:  # worldbody
            break
        current = model.body_parentid[current]
        depth += 1

    current = body2
    depth = 0
    while current not in depths:
        current = model.body_parentid[current]
        depth += 1
    return depths[current] + depth

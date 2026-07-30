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
        if _kinematically_related(model, body1, body2):
            # Adjacent links within the same leg chain (e.g. coxa/tibia)
            # have collision meshes that overlap slightly by design even
            # in a normal standing pose. Only contact between unrelated
            # branches (a different leg, or the body from a leg it isn't
            # attached to) counts as a genuine self-collision.
            continue
        return True
    return False


def _kinematically_related(model, body1: int, body2: int) -> bool:
    return _is_ancestor(model, body1, body2) or _is_ancestor(model, body2, body1)


def _is_ancestor(model, ancestor_id: int, body_id: int) -> bool:
    current = body_id
    while current != 0:
        if current == ancestor_id:
            return True
        current = model.body_parentid[current]
    return current == ancestor_id

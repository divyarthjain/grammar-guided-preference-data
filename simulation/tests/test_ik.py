import mujoco
import numpy as np

from simulation.mujoco_judge.constants import FOOT_BODY, LEG_JOINTS
from simulation.mujoco_judge.ik import bbox_to_target, solve_leg_ik


def test_bbox_at_image_center_maps_near_reference():
    reference = np.array([0.1, 0.0, 0.05])
    bbox = {"x": 300.0, "y": 220.0, "width": 40.0, "height": 40.0}  # near center of 640x480
    target = bbox_to_target(bbox, reference)
    assert np.linalg.norm(target - reference) < 0.05


def test_bbox_at_image_corner_maps_far_from_reference():
    reference = np.array([0.1, 0.0, 0.05])
    bbox = {"x": 0.0, "y": 0.0, "width": 10.0, "height": 10.0}  # top-left corner
    target = bbox_to_target(bbox, reference)
    assert np.linalg.norm(target - reference) > 0.15


def test_nearby_target_converges(model, data):
    mujoco.mj_kinematics(model, data)
    resting_foot_pos = data.xpos[model.body(FOOT_BODY).id].copy()
    # FOOT_BODY's own joint (j_tibia_rf) is defined at its body origin, so it
    # cannot translate that origin: only j_c1_rf/j_thigh_rf move it, spanning
    # a 2D reachable surface. A pure +x offset happens to sit nearly normal
    # to that surface for this leg's resting orientation, so we offset along
    # all three axes to land within the leg's actual reachable neighborhood.
    target = resting_foot_pos + np.array([0.01, 0.01, 0.01])

    converged, qpos_values, error = solve_leg_ik(model, data, target)

    assert converged
    assert error < 0.004
    assert qpos_values.shape == (3,)


def test_distant_target_does_not_converge(model, data):
    mujoco.mj_kinematics(model, data)
    resting_foot_pos = data.xpos[model.body(FOOT_BODY).id].copy()
    target = resting_foot_pos + np.array([5.0, 5.0, 5.0])

    converged, _, error = solve_leg_ik(model, data, target)

    assert not converged
    assert error > 0.004


def test_final_error_describes_returned_qpos_when_not_converged(model, data):
    """Regression test: on the non-convergent (max_iters exhausted) exit path,
    `final_error` must describe the *returned* `qpos_values` — not a stale
    pre-update error from the iteration before the last `qpos` change."""
    mujoco.mj_kinematics(model, data)
    resting_foot_pos = data.xpos[model.body(FOOT_BODY).id].copy()
    target = resting_foot_pos + np.array([5.0, 5.0, 5.0])

    converged, qpos_values, error = solve_leg_ik(model, data, target)
    assert not converged

    # Recompute the actual foot position implied by the returned qpos_values
    # and confirm `error` matches it exactly.
    joint_ids = [model.joint(name).id for name in LEG_JOINTS]
    qpos_adr = [model.jnt_qposadr[j] for j in joint_ids]
    for adr, value in zip(qpos_adr, qpos_values):
        data.qpos[adr] = value
    mujoco.mj_kinematics(model, data)
    actual_foot_pos = data.xpos[model.body(FOOT_BODY).id]
    recomputed_error = np.linalg.norm(target - actual_foot_pos)

    assert np.isclose(error, recomputed_error, atol=1e-9)

import mujoco
import numpy as np

from simulation.mujoco_judge.constants import FOOT_BODY, LEG_JOINTS


def bbox_to_target(
    bbox: dict,
    reference: np.ndarray,
    image_size: tuple[int, int] = (640, 480),
    workspace_radius: float = 0.25,
) -> np.ndarray:
    """Placeholder mapping from a 2D detection bbox to a 3D IK target.

    There is no real camera/depth calibration in this project yet — this
    normalizes the bbox center against an assumed image size and offsets
    `reference` by up to `workspace_radius` meters in the x/y plane.
    `workspace_radius` is deliberately larger than the leg's real reach
    so off-center detections can produce infeasible targets, which is
    what the physical judge is supposed to catch.
    """
    img_w, img_h = image_size
    center_x = bbox["x"] + bbox["width"] / 2
    center_y = bbox["y"] + bbox["height"] / 2
    norm_x = (center_x / img_w) * 2 - 1  # [-1, 1]
    norm_y = (center_y / img_h) * 2 - 1  # [-1, 1]
    offset = np.array([norm_x, norm_y, 0.0]) * workspace_radius
    return reference + offset


def solve_leg_ik(
    model,
    data,
    target: np.ndarray,
    max_iters: int = 200,
    tol: float = 0.004,
    damping: float = 0.05,
) -> tuple[bool, np.ndarray, float]:
    """Damped least-squares IK for the front-right leg's 3 joints.

    Mutates `data.qpos` toward a solution. Returns whether it converged
    within `tol` meters, the final joint values, and the final error.
    """
    joint_ids = [model.joint(name).id for name in LEG_JOINTS]
    qpos_adr = [model.jnt_qposadr[j] for j in joint_ids]
    dof_adr = [model.jnt_dofadr[j] for j in joint_ids]
    body_id = model.body(FOOT_BODY).id

    jacp = np.zeros((3, model.nv))
    error = np.inf

    for _ in range(max_iters):
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)  # populates cdof, required by mj_jacBody
        foot_pos = data.xpos[body_id].copy()
        error_vec = target - foot_pos
        error = float(np.linalg.norm(error_vec))
        if error < tol:
            break

        mujoco.mj_jacBody(model, data, jacp, None, body_id)
        j_leg = jacp[:, dof_adr]  # (3, 3): this leg's columns only
        jjt = j_leg @ j_leg.T + (damping**2) * np.eye(3)
        dq = j_leg.T @ np.linalg.solve(jjt, error_vec)
        for i, adr in enumerate(qpos_adr):
            data.qpos[adr] += dq[i]

    qpos_values = np.array([data.qpos[adr] for adr in qpos_adr])
    return error < tol, qpos_values, error

import numpy as np

from simulation.mujoco_judge.constants import LEG_JOINTS


def within_joint_limits(model, qpos_values: np.ndarray) -> bool:
    joint_ids = [model.joint(name).id for name in LEG_JOINTS]
    for joint_id, q in zip(joint_ids, qpos_values):
        lo, hi = model.jnt_range[joint_id]
        if not (lo <= q <= hi):
            return False
    return True

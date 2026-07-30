from enum import Enum

import mujoco

from simulation.mujoco_judge.constants import FOOT_BODY
from simulation.mujoco_judge.ik import bbox_to_target, solve_leg_ik
from simulation.mujoco_judge.limits import within_joint_limits
from simulation.mujoco_judge.stability import settle_and_check_stability


class Verdict(Enum):
    CHOSEN = "chosen"
    REJECTED = "rejected"


class MuJoCoJudge:
    """Physically-grounded stand-in for the real IK+Ruckig judge (team doc
    §5.3): drives the simulated front-right leg toward a candidate's
    detected target and reports whether it's reachable, within joint
    limits, and doesn't topple or self-collide the robot.

    Note: the stability leg of this check settles an unpowered model (no
    actuators, no joint damping in `scene.xml`) — see the fidelity caveat
    in `stability.py`'s module docstring before treating a CHOSEN verdict
    as evidence an actuated robot could hold the pose.
    """

    def __init__(self, model, data):
        self.model = model
        self.data = data

    def check(self, candidate: dict) -> Verdict:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_kinematics(self.model, self.data)
        reference = self.data.xpos[self.model.body(FOOT_BODY).id].copy()
        target = bbox_to_target(candidate["bbox"], reference)

        converged, qpos_values, _ = solve_leg_ik(self.model, self.data, target)
        if not converged:
            return Verdict.REJECTED
        if not within_joint_limits(self.model, qpos_values):
            return Verdict.REJECTED
        if not settle_and_check_stability(self.model, self.data):
            return Verdict.REJECTED
        return Verdict.CHOSEN

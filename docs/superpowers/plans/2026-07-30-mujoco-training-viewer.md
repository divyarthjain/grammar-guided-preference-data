# MuJoCo Training Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a continuous, watchable training-time simulation: a MuJoCo-backed physical judge that drives a stand-in hexapod's leg toward a candidate target and reports feasibility (reachable, within joint limits, stable, no self-collision), writing preference pairs to the same `data/preference_pairs/*.jsonl` file the Rust runtime uses, with a live viewer open so every candidate attempt can be watched.

**Architecture:** A new `simulation/` Python package (uv-managed, sibling to `runtime/` and `training/`) vendors a licensed stand-in hexapod model (PhantomX, BSD), wraps it in a MuJoCo scene with a ground plane, and implements a small damped-least-squares IK solver + stability/collision check against one leg. A stubbed sampler (matching the existing Rust `orchestrator`'s pattern) provides candidates until real VLM sampling exists. `simulation` depends on the local `training` package only for the shared `PreferencePair` type — nothing else changes in `runtime/` or `training/`.

**Tech Stack:** Python 3.12, `uv`, `mujoco` (3.11.0), `numpy`, `pytest`.

## Global Constraints

- Robot model: `HumaRobotics/phantomx_description`, BSD-Simplified licensed (verified from its actual LICENSE file during brainstorming) — vendor as-is, attribute the source.
- The robot's front-right leg has exactly 3 actuated revolute joints: `j_c1_rf`, `j_thigh_rf`, `j_tibia_rf` (all `limit lower="-2.6179939" upper="2.6179939"` radians), terminating at body `tibia_rf`. Other legs follow the same `_lf/_lm/_lr/_rm/_rr` naming; only the front-right leg is used by this plan.
- The URDF's mesh references use `package://phantomx_description/meshes/...` (ROS package URIs), which MuJoCo cannot resolve — they must be rewritten to relative paths as part of vendoring.
- The base link (`base_link` → `MP_BODY`, connected by a `fixed` joint) has no joint connecting it to world in the URDF, so MuJoCo's compiler adds a free joint automatically — the robot can physically fall over, which the stability check relies on.
- `data/preference_pairs/*.jsonl` is the shared interface with `training/` (already established) — this plan's writer must produce lines that `training.data.load_pairs` can already parse without modification.
- Platform: macOS, Apple Silicon (`aarch64-apple-darwin`) — no GPU/CUDA assumptions.

---

## Task 1: Vendor the PhantomX model and build a loadable MuJoCo scene

**Files:**
- Create: `simulation/pyproject.toml`
- Create: `simulation/models/phantomx/urdf/phantomx.urdf` (vendored, mesh paths rewritten)
- Create: `simulation/models/phantomx/meshes/*.STL` (vendored, ~20 files)
- Create: `simulation/models/phantomx/LICENSE` (vendored BSD-Simplified license text)
- Create: `simulation/models/phantomx/ATTRIBUTION.md`
- Create: `simulation/models/phantomx/scene.xml` (generated + hand-edited MJCF)
- Create: `simulation/tests/conftest.py`
- Test: `simulation/tests/test_scene.py`

**Interfaces:**
- Produces: `SCENE_PATH` constant (in `simulation/src/simulation/mujoco_judge/constants.py`, created in this task) pointing at `simulation/models/phantomx/scene.xml`, and a pytest fixture `model` (loaded `mujoco.MjModel`) and `data` (`mujoco.MjData`) in `simulation/tests/conftest.py`, reused by every later task's tests.

- [ ] **Step 1: Create the `simulation` uv package**

```bash
cd /Users/divyarth/Projects/grammar-guided-preference-data
uv init --package simulation
cd simulation
uv add mujoco numpy
uv add --dev pytest
uv add --editable ../training
```

The last command adds the local `training` package as a dependency so `simulation` can import `training.data.PreferencePair` later in this plan.

- [ ] **Step 2: Vendor the URDF and meshes**

```bash
cd /Users/divyarth/Projects/grammar-guided-preference-data/simulation/models
mkdir -p phantomx/urdf phantomx/meshes
curl -sL https://raw.githubusercontent.com/HumaRobotics/phantomx_description/master/urdf/phantomx.urdf -o phantomx/urdf/phantomx.urdf
curl -sL https://raw.githubusercontent.com/HumaRobotics/phantomx_description/master/LICENSE -o phantomx/LICENSE
for f in body body_coll connect connect_coll thigh_l thigh_l_coll thigh_r thigh_r_coll tibia_l tibia_l_coll tibia_r tibia_r_coll; do
  curl -sL "https://raw.githubusercontent.com/HumaRobotics/phantomx_description/master/meshes/${f}.STL" -o "phantomx/meshes/${f}.STL"
done
```

Verify every mesh referenced by the URDF actually downloaded:

```bash
grep -oE 'meshes/[A-Za-z_]+\.STL' phantomx/urdf/phantomx.urdf | sed 's#meshes/##' | sort -u > /tmp/needed_meshes.txt
ls phantomx/meshes | sort > /tmp/have_meshes.txt
diff /tmp/needed_meshes.txt /tmp/have_meshes.txt
```

If `diff` reports any mesh present in `needed_meshes.txt` but missing from `have_meshes.txt`, fetch it the same way (check the repo's `meshes/` directory listing at `https://github.com/HumaRobotics/phantomx_description/tree/master/meshes` for the exact filename — case matters).

- [ ] **Step 3: Rewrite mesh paths so MuJoCo can resolve them**

```bash
sed -i '' 's#package://phantomx_description/meshes/#../meshes/#g' phantomx/urdf/phantomx.urdf
```

- [ ] **Step 4: Write the attribution note**

```markdown
# Attribution

Vendored from https://github.com/HumaRobotics/phantomx_description
(commit: record the commit SHA you fetched from, via
`gh api repos/HumaRobotics/phantomx_description/commits/master --jq .sha`),
licensed under the Simplified BSD License (see `LICENSE` in this
directory). This is a stand-in model for the project's own ArcheoHex
robot — see `../../docs/architecture.md` in the main repo.

Changes made: `urdf/phantomx.urdf` had its `package://` mesh URIs
rewritten to relative paths (`../meshes/...`) so MuJoCo can resolve them
without a ROS package index.
```

Save this as `phantomx/ATTRIBUTION.md`, filling in the actual commit SHA from the `gh api` command.

- [ ] **Step 5: Convert the URDF to MJCF and add a ground plane**

Run this one-time conversion script (save it temporarily as `/tmp/convert.py`, run it, then delete it — it's not part of the shipped package):

```python
import mujoco

model = mujoco.MjModel.from_xml_path("phantomx/urdf/phantomx.urdf")
mujoco.mj_saveLastXML("phantomx/scene.xml", model)
```

```bash
cd /Users/divyarth/Projects/grammar-guided-preference-data/simulation/models
uv run --project ../.. python /tmp/convert.py
```

(Run this from wherever the `simulation` venv is active — e.g. `cd ../.. && uv run --project simulation python /tmp/convert.py` if that's more convenient; the important part is running it with the `simulation` package's `mujoco` installed.)

Open the generated `phantomx/scene.xml` and add a ground plane and a light inside its `<worldbody>` element (insert as the *first* children of `<worldbody>`, before the robot's own bodies):

```xml
<geom name="floor" type="plane" size="2 2 0.1" rgba="0.6 0.6 0.6 1" friction="1 0.005 0.0001"/>
<light name="top" pos="0 0 2" dir="0 0 -1" directional="true"/>
```

- [ ] **Step 6: Write the constants module**

```python
# simulation/src/simulation/mujoco_judge/constants.py
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parents[3] / "models"
SCENE_PATH = MODELS_DIR / "phantomx" / "scene.xml"

LEG_JOINTS = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
FOOT_BODY = "tibia_rf"
BASE_BODY = "MP_BODY"
FLOOR_GEOM = "floor"
```

- [ ] **Step 7: Write the shared pytest fixtures**

```python
# simulation/tests/conftest.py
import mujoco
import pytest

from simulation.mujoco_judge.constants import SCENE_PATH


@pytest.fixture
def model():
    return mujoco.MjModel.from_xml_path(str(SCENE_PATH))


@pytest.fixture
def data(model):
    return mujoco.MjData(model)
```

- [ ] **Step 8: Write the failing test**

```python
# simulation/tests/test_scene.py
from simulation.mujoco_judge.constants import LEG_JOINTS, FOOT_BODY, FLOOR_GEOM


def test_scene_loads_with_expected_joints(model):
    assert model.njnt == 19  # 18 actuated leg hinges + 1 free base joint
    for name in LEG_JOINTS:
        assert model.joint(name).id >= 0


def test_scene_has_foot_body_and_floor(model):
    assert model.body(FOOT_BODY).id >= 0
    assert model.geom(FLOOR_GEOM).id >= 0
```

- [ ] **Step 9: Run the test to verify it currently fails or passes as expected**

```bash
cd /Users/divyarth/Projects/grammar-guided-preference-data/simulation
uv run pytest tests/test_scene.py -v
```

Expected: both tests PASS if Steps 1-6 were done correctly. If `test_scene_loads_with_expected_joints` fails on the `njnt == 19` assertion, print `model.njnt` and the actual joint names (`[model.joint(i).name for i in range(model.njnt)]`) to see what MuJoCo actually produced, and adjust the assertion to match reality — the exact count matters less than confirming the 18 named leg joints exist and the base is free-floating (not welded).

- [ ] **Step 10: Commit**

```bash
git add simulation/
git commit -m "Vendor PhantomX hexapod model (BSD) and build MuJoCo scene"
```

---

## Task 2: `bbox_to_target` — placeholder bbox-to-3D-target mapping

**Files:**
- Create: `simulation/src/simulation/mujoco_judge/ik.py`
- Test: `simulation/tests/test_ik.py`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `bbox_to_target(bbox: dict, reference: "numpy.ndarray", image_size: tuple[int, int] = (640, 480), workspace_radius: float = 0.25) -> "numpy.ndarray"`. `bbox` is a dict with keys `x, y, width, height` (pixel coordinates, matching the `grammar::BBox` schema). Returns a 3D point in the same frame as `reference`.

This is an explicit placeholder — there is no real camera/depth calibration in this project yet (no VLM integration, no real camera). It maps the bbox center to an offset from a known reference point, scaled by `workspace_radius`, which is intentionally set *larger* than the leg's real reach (~0.15m for a hobby hexapod this size) so bboxes near the image edges deliberately produce targets outside the leg's workspace — this is what lets later tasks test the "infeasible" path without needing precise real-world calibration.

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_ik.py
import numpy as np

from simulation.mujoco_judge.ik import bbox_to_target


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
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_ik.py -v
```

Expected: FAIL with `ModuleNotFoundError` or `ImportError` (`ik.py` doesn't exist yet).

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/mujoco_judge/ik.py
import numpy as np


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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_ik.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/ik.py simulation/tests/test_ik.py
git commit -m "Add bbox_to_target placeholder mapping"
```

---

## Task 3: `solve_leg_ik` — damped least-squares IK for the front-right leg

**Files:**
- Modify: `simulation/src/simulation/mujoco_judge/ik.py`
- Test: `simulation/tests/test_ik.py`

**Interfaces:**
- Consumes: `LEG_JOINTS`, `FOOT_BODY` from `constants.py` (Task 1); the `model`/`data` fixtures (Task 1).
- Produces: `solve_leg_ik(model, data, target: np.ndarray, max_iters: int = 200, tol: float = 0.004, damping: float = 0.05) -> tuple[bool, "numpy.ndarray", float]`. Returns `(converged, qpos_values, final_error)` where `qpos_values` is a length-3 array for `[j_c1_rf, j_thigh_rf, j_tibia_rf]`. Mutates `data.qpos` in place (leaves the leg at the solved — or best-attempt — configuration).

- [ ] **Step 1: Write the failing test**

```python
# append to simulation/tests/test_ik.py
import mujoco

from simulation.mujoco_judge.constants import FOOT_BODY
from simulation.mujoco_judge.ik import solve_leg_ik


def test_nearby_target_converges(model, data):
    mujoco.mj_kinematics(model, data)
    resting_foot_pos = data.xpos[model.body(FOOT_BODY).id].copy()
    target = resting_foot_pos + np.array([0.02, 0.0, 0.0])

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
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_ik.py -v
```

Expected: FAIL with `ImportError: cannot import name 'solve_leg_ik'`.

- [ ] **Step 3: Write the implementation**

```python
# append to simulation/src/simulation/mujoco_judge/ik.py
import mujoco

from simulation.mujoco_judge.constants import LEG_JOINTS, FOOT_BODY


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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_ik.py -v
```

Expected: PASS. If `test_nearby_target_converges` fails to converge, increase `max_iters` or `damping` slightly and re-run — the exact numbers matter less than the qualitative behavior (near targets converge, absurdly distant ones don't).

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/ik.py simulation/tests/test_ik.py
git commit -m "Add damped least-squares IK for the front-right leg"
```

---

## Task 4: `within_joint_limits` — joint-limit check

**Files:**
- Create: `simulation/src/simulation/mujoco_judge/limits.py`
- Test: `simulation/tests/test_limits.py`

**Interfaces:**
- Consumes: `LEG_JOINTS` from `constants.py` (Task 1).
- Produces: `within_joint_limits(model, qpos_values: "numpy.ndarray") -> bool`.

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_limits.py
import numpy as np

from simulation.mujoco_judge.limits import within_joint_limits


def test_zero_qpos_is_within_limits(model):
    assert within_joint_limits(model, np.array([0.0, 0.0, 0.0]))


def test_out_of_range_qpos_is_rejected(model):
    assert not within_joint_limits(model, np.array([0.0, 0.0, 10.0]))
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_limits.py -v
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/mujoco_judge/limits.py
import numpy as np

from simulation.mujoco_judge.constants import LEG_JOINTS


def within_joint_limits(model, qpos_values: np.ndarray) -> bool:
    joint_ids = [model.joint(name).id for name in LEG_JOINTS]
    for joint_id, q in zip(joint_ids, qpos_values):
        lo, hi = model.jnt_range[joint_id]
        if not (lo <= q <= hi):
            return False
    return True
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_limits.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/limits.py simulation/tests/test_limits.py
git commit -m "Add joint-limit check"
```

---

## Task 5: `settle_and_check_stability` — toppling and self-collision check

**Files:**
- Create: `simulation/src/simulation/mujoco_judge/stability.py`
- Test: `simulation/tests/test_stability.py`

**Interfaces:**
- Consumes: `BASE_BODY`, `FLOOR_GEOM` from `constants.py` (Task 1).
- Produces: `settle_and_check_stability(model, data, hold_steps: int = 100, max_tilt_deg: float = 25.0, min_base_height: float = 0.02) -> bool`. Runs physics forward from `data`'s current state and reports whether the robot stayed upright and free of self-collision.

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_stability.py
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
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_stability.py -v
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/mujoco_judge/stability.py
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
        if model.geom_bodyid[contact.geom1] != model.geom_bodyid[contact.geom2]:
            return True
    return False
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_stability.py -v
```

Expected: PASS. If `test_fallen_base_is_unstable` doesn't fail as expected (e.g. the forced pose happens to settle back upright), increase the forced tilt or drop height, or reduce `min_base_height`/`max_tilt_deg` slightly, and re-run.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/stability.py simulation/tests/test_stability.py
git commit -m "Add toppling and self-collision stability check"
```

---

## Task 6: `MuJoCoJudge` — orchestrates IK, limits, and stability into a verdict

**Files:**
- Create: `simulation/src/simulation/mujoco_judge/judge.py`
- Test: `simulation/tests/test_judge.py`

**Interfaces:**
- Consumes: `bbox_to_target`, `solve_leg_ik` (Task 2/3), `within_joint_limits` (Task 4), `settle_and_check_stability` (Task 5), `FOOT_BODY` (Task 1).
- Produces: `class Verdict(Enum): CHOSEN = "chosen"; REJECTED = "rejected"` and `class MuJoCoJudge: def __init__(self, model, data): ...` with `def check(self, candidate: dict) -> Verdict`. `candidate` matches the grammar schema: `{"object": str, "bbox": {"x","y","width","height"}, "confidence": float, "action_type": str}`.

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_judge.py
from simulation.mujoco_judge.judge import MuJoCoJudge, Verdict


def test_center_bbox_candidate_is_chosen(model, data):
    judge = MuJoCoJudge(model, data)
    candidate = {
        "object": "red block",
        "bbox": {"x": 300.0, "y": 220.0, "width": 40.0, "height": 40.0},
        "confidence": 0.9,
        "action_type": "grasp",
    }
    assert judge.check(candidate) == Verdict.CHOSEN


def test_corner_bbox_candidate_is_rejected(model, data):
    judge = MuJoCoJudge(model, data)
    candidate = {
        "object": "table edge",
        "bbox": {"x": 0.0, "y": 0.0, "width": 10.0, "height": 10.0},
        "confidence": 0.4,
        "action_type": "avoid",
    }
    assert judge.check(candidate) == Verdict.REJECTED
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_judge.py -v
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/mujoco_judge/judge.py
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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_judge.py -v
```

Expected: PASS. If either candidate's verdict comes out opposite to expected, that's real physics feedback, not a bug in the test — inspect which check failed (add a temporary print of `converged`/`within_joint_limits`/`settle_and_check_stability` results) and adjust `workspace_radius` in `ik.py` (Task 2) or the stability thresholds (Task 5) so the two example candidates land on opposite sides, then re-run.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/judge.py simulation/tests/test_judge.py
git commit -m "Add MuJoCoJudge orchestrating IK, limits, and stability"
```

---

## Task 7: `stubbed_candidates` — stand-in sampler

**Files:**
- Create: `simulation/src/simulation/mujoco_judge/sampler.py`
- Test: `simulation/tests/test_sampler.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `stubbed_candidates() -> list[dict]`, each dict matching the grammar schema (same shape as `judge.py`'s `candidate` parameter).

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_sampler.py
from simulation.mujoco_judge.sampler import stubbed_candidates


def test_stubbed_candidates_returns_two_well_formed_candidates():
    candidates = stubbed_candidates()
    assert len(candidates) == 2
    for c in candidates:
        assert set(c.keys()) == {"object", "bbox", "confidence", "action_type"}
        assert set(c["bbox"].keys()) == {"x", "y", "width", "height"}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_sampler.py -v
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/mujoco_judge/sampler.py
"""Stub multi-sample generation (team doc §5.2), mirroring
runtime/crates/orchestrator/src/sampler.rs — no real VLM call yet.
"""


def stubbed_candidates() -> list[dict]:
    return [
        {
            "object": "red block",
            "bbox": {"x": 300.0, "y": 220.0, "width": 40.0, "height": 40.0},
            "confidence": 0.92,
            "action_type": "grasp",
        },
        {
            "object": "table edge",
            "bbox": {"x": 0.0, "y": 0.0, "width": 10.0, "height": 10.0},
            "confidence": 0.4,
            "action_type": "avoid",
        },
    ]
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_sampler.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/sampler.py simulation/tests/test_sampler.py
git commit -m "Add stubbed candidate sampler"
```

---

## Task 8: `write_pair` — preference-pair JSONL writer

**Files:**
- Create: `simulation/src/simulation/mujoco_judge/pairs.py`
- Test: `simulation/tests/test_pairs.py`

**Interfaces:**
- Consumes: `training.data.PreferencePair` (existing, from the local `training` dependency added in Task 1).
- Produces: `write_pair(path: "pathlib.Path", pair: "training.data.PreferencePair") -> None`, appending one JSON line, and `make_pair(image_ref: str, prompt: str, chosen: dict, rejected: dict) -> "training.data.PreferencePair"` (fills in `timestamp` via `datetime.now(timezone.utc).isoformat()`).

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_pairs.py
import json

from training.data import load_pairs

from simulation.mujoco_judge.pairs import make_pair, write_pair


def test_write_pair_round_trips_through_load_pairs(tmp_path):
    out_path = tmp_path / "pairs.jsonl"
    chosen = {"object": "red block", "bbox": {"x": 1.0, "y": 2.0, "width": 3.0, "height": 4.0}, "confidence": 0.9, "action_type": "grasp"}
    rejected = {"object": "table edge", "bbox": {"x": 0.0, "y": 0.0, "width": 10.0, "height": 10.0}, "confidence": 0.4, "action_type": "avoid"}
    pair = make_pair("frame_001.png", "describe the scene", chosen, rejected)

    write_pair(out_path, pair)

    loaded = load_pairs(out_path)
    assert len(loaded) == 1
    assert loaded[0].image_ref == "frame_001.png"
    assert loaded[0].chosen["object"] == "red block"


def test_write_pair_appends(tmp_path):
    out_path = tmp_path / "pairs.jsonl"
    pair = make_pair("a.png", "p", {"object": "x", "bbox": {"x": 0.0, "y": 0.0, "width": 1.0, "height": 1.0}, "confidence": 0.5, "action_type": "none"}, {"object": "y", "bbox": {"x": 0.0, "y": 0.0, "width": 1.0, "height": 1.0}, "confidence": 0.5, "action_type": "none"})

    write_pair(out_path, pair)
    write_pair(out_path, pair)

    assert len(load_pairs(out_path)) == 2
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_pairs.py -v
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/mujoco_judge/pairs.py
import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from training.data import PreferencePair


def make_pair(image_ref: str, prompt: str, chosen: dict, rejected: dict) -> PreferencePair:
    return PreferencePair(
        image_ref=image_ref,
        prompt=prompt,
        chosen=chosen,
        rejected=rejected,
        timestamp=datetime.now(timezone.utc).isoformat(),
    )


def write_pair(path: Path, pair: PreferencePair) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as f:
        f.write(json.dumps(asdict(pair)) + "\n")
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_pairs.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/pairs.py simulation/tests/test_pairs.py
git commit -m "Add preference-pair JSONL writer reusing training.data.PreferencePair"
```

---

## Task 9: `run_continuous` — the watchable training loop

**Files:**
- Create: `simulation/src/simulation/mujoco_judge/loop.py`
- Create: `simulation/src/simulation/mujoco_judge/__main__.py`
- Test: `simulation/tests/test_loop.py`

**Interfaces:**
- Consumes: `stubbed_candidates` (Task 7), `MuJoCoJudge`/`Verdict` (Task 6), `make_pair`/`write_pair` (Task 8).
- Produces: `run_continuous(model, data, out_path: "pathlib.Path", max_iterations: int | None = None, launch_viewer: bool = True, frame_delay_seconds: float = 1.0) -> int` — returns the number of pairs written. A runnable entry point via `python -m simulation.mujoco_judge`.

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_loop.py
from training.data import load_pairs

from simulation.mujoco_judge.loop import run_continuous


def test_run_continuous_one_iteration_writes_well_formed_output(tmp_path, model, data):
    out_path = tmp_path / "pairs.jsonl"

    pairs_written = run_continuous(
        model, data, out_path, max_iterations=1, launch_viewer=False, frame_delay_seconds=0.0
    )

    assert pairs_written in (0, 1)  # depends on real physics outcome of the two stub candidates
    if pairs_written == 1:
        loaded = load_pairs(out_path)
        assert len(loaded) == 1
        assert loaded[0].chosen["action_type"] in {"approach", "avoid", "grasp", "inspect", "none"}
        assert loaded[0].rejected["action_type"] in {"approach", "avoid", "grasp", "inspect", "none"}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_loop.py -v
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/mujoco_judge/loop.py
import time

from simulation.mujoco_judge.judge import MuJoCoJudge, Verdict
from simulation.mujoco_judge.pairs import make_pair, write_pair
from simulation.mujoco_judge.sampler import stubbed_candidates


def run_continuous(
    model,
    data,
    out_path,
    max_iterations: int | None = None,
    launch_viewer: bool = True,
    frame_delay_seconds: float = 1.0,
) -> int:
    """Runs the sampler -> judge -> pair-writer loop. If `launch_viewer`,
    opens a live MuJoCo viewer so each candidate attempt can be watched;
    `frame_delay_seconds` paces iterations so a human can follow along.
    Loops forever if `max_iterations` is None (the production entry
    point); tests pass a small bound and `launch_viewer=False`.
    """
    judge = MuJoCoJudge(model, data)
    pairs_written = 0
    iteration = 0

    viewer_cm = None
    if launch_viewer:
        import mujoco.viewer

        viewer_cm = mujoco.viewer.launch_passive(model, data)

    try:
        while max_iterations is None or iteration < max_iterations:
            candidates = stubbed_candidates()
            chosen = None
            rejected = None
            for candidate in candidates:
                verdict = judge.check(candidate)
                if viewer_cm is not None:
                    viewer_cm.sync()
                if verdict == Verdict.CHOSEN and chosen is None:
                    chosen = candidate
                elif verdict == Verdict.REJECTED and rejected is None:
                    rejected = candidate
                if frame_delay_seconds:
                    time.sleep(frame_delay_seconds)

            if chosen is not None and rejected is not None:
                pair = make_pair("frame_stub.png", "describe the scene", chosen, rejected)
                write_pair(out_path, pair)
                pairs_written += 1

            iteration += 1
    finally:
        if viewer_cm is not None:
            viewer_cm.close()

    return pairs_written
```

```python
# simulation/src/simulation/mujoco_judge/__main__.py
from pathlib import Path

import mujoco

from simulation.mujoco_judge.constants import SCENE_PATH
from simulation.mujoco_judge.loop import run_continuous

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    # __main__.py is at simulation/src/simulation/mujoco_judge/__main__.py;
    # parents[4] is the repo root (grammar-guided-preference-data/).
    out_path = Path(__file__).resolve().parents[4] / "data" / "preference_pairs" / "sim_pairs.jsonl"
    run_continuous(model, data, out_path)
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_loop.py -v
```

Expected: PASS.

- [ ] **Step 5: Run the full test suite**

```bash
uv run pytest -v
```

Expected: all tests across every task in this plan PASS.

- [ ] **Step 6: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/loop.py simulation/src/simulation/mujoco_judge/__main__.py simulation/tests/test_loop.py
git commit -m "Add continuous training loop with live MuJoCo viewer"
```

- [ ] **Step 7: Watch it run**

```bash
cd /Users/divyarth/Projects/grammar-guided-preference-data/simulation
uv run python -m simulation.mujoco_judge
```

A MuJoCo viewer window should open showing the PhantomX stand-in robot; the front-right leg will visibly move toward each stubbed candidate's target once per second, and `data/preference_pairs/sim_pairs.jsonl` will accumulate lines. Press Ctrl+C to stop (it loops forever by design — this is the "one continuous instance" from the brainstorming session).

---

## Plan-level verification

1. `cd simulation && uv run pytest -v` — every test across all 9 tasks passes.
2. `cd simulation && uv run python -m simulation.mujoco_judge`, watch it for a few iterations, confirm the viewer shows the leg moving and Ctrl+C stops it cleanly.
3. `cat ../data/preference_pairs/sim_pairs.jsonl | python3 -m json.tool` on the last line (or via `tail -1 ... | python3 -m json.tool`) — confirm it's well-formed and matches the schema.
4. `cd .. && python3 -c "from training.data import load_pairs; from pathlib import Path; print(load_pairs(Path('data/preference_pairs/sim_pairs.jsonl')))"` (run with the `training` venv active, e.g. `cd training && uv run python -c "..."` with an adjusted relative path) — confirms `training/` can read what `simulation/` wrote, proving the shared-JSONL interface actually works end-to-end.

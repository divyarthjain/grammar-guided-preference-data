# Unity Checkpoint Recorder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the periodic, comparable checkpoint recording: a replay-file format written by Python (reusing the MuJoCo judge from the training-viewer plan), and a Unity project that reads a replay file, animates the PhantomX stand-in's front-right leg through it, and exports an MP4 via Unity's Recorder — the same fixed scenario and camera every time, so checkpoint N and checkpoint N+10 are directly comparable for the YouTube video.

**Architecture:** Extends `simulation/mujoco_judge/judge.py` (from the MuJoCo training-viewer plan — assumes that plan is already implemented) to expose the joint angles it solves. A new `simulation/replay/` module defines a JSON replay format (no dictionaries — plain objects/arrays only, so Unity's `JsonUtility` can parse it directly) and a script that runs a fixed scenario through the judge and writes a replay file. A separate Unity project imports the same vendored PhantomX URDF and reads that file to animate and record.

**Tech Stack:** Python 3.12 / `uv` (same `simulation` package), Unity 2022.3 LTS, Unity's URDF-Importer and Recorder packages, C#.

## Global Constraints

- **Depends on the MuJoCo training-viewer plan being implemented first** (`docs/superpowers/plans/2026-07-30-mujoco-training-viewer.md`) — this plan modifies `simulation/mujoco_judge/judge.py` and reuses `simulation/models/phantomx/`, `LEG_JOINTS`, `FOOT_BODY` from it.
- The replay-file JSON must avoid dictionaries/maps entirely (objects with dynamic keys) — Unity's `JsonUtility` can only deserialize fixed fields and arrays of primitives/serializable classes, not `Dictionary<string, T>`. Joint angles are always a 3-element `float[]` in the fixed order `[j_c1_rf, j_thigh_rf, j_tibia_rf]` (matching `LEG_JOINTS` from the training-viewer plan), never a name-keyed map.
- Joint angles are radians on the Python/MuJoCo side; Unity's `ArticulationBody` joint drive targets are in **degrees** — every C# read of a replay angle must convert via `* Mathf.Rad2Deg`.
- No real trained checkpoint exists yet (`training/train.py` is still a `NotImplementedError` stub) — `record_checkpoint.py` runs against a stand-in "checkpoint" (just re-invoking the stubbed sampler + judge) and is invoked manually, not auto-triggered. This is documented, not hidden.
- **Unity is not installed on this machine as of this plan.** Task 5 covers installing it. Everything from Task 5 onward requires a human at the Unity Editor GUI — it cannot be executed unattended the way the Python tasks can.
- Testing scope: Tasks 1-4 (Python) are TDD'd with `pytest` as usual. Tasks 5-8 (Unity) have no automated test — verification is manual (open the project, press Play, confirm an MP4 comes out), matching the approved spec's testing section.

---

## Task 1: Expose the judge's last-solved joint angles

**Files:**
- Modify: `simulation/src/simulation/mujoco_judge/judge.py`
- Modify: `simulation/tests/test_judge.py`

**Interfaces:**
- Consumes: `MuJoCoJudge` (existing, from the training-viewer plan).
- Produces: `MuJoCoJudge.last_qpos: "numpy.ndarray | None"` — a new instance attribute, set to the 3-element `[j_c1_rf, j_thigh_rf, j_tibia_rf]` array every time `check()` runs, `None` before the first call. `check()`'s existing signature and return type (`Verdict`) are unchanged.

- [ ] **Step 1: Write the failing test**

```python
# append to simulation/tests/test_judge.py
import numpy as np


def test_check_records_last_qpos(model, data):
    judge = MuJoCoJudge(model, data)
    assert judge.last_qpos is None

    candidate = {
        "object": "red block",
        "bbox": {"x": 300.0, "y": 220.0, "width": 40.0, "height": 40.0},
        "confidence": 0.9,
        "action_type": "grasp",
    }
    judge.check(candidate)

    assert judge.last_qpos is not None
    assert judge.last_qpos.shape == (3,)
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /Users/divyarth/Projects/grammar-guided-preference-data/simulation
uv run pytest tests/test_judge.py -v -k last_qpos
```

Expected: FAIL with `AttributeError: 'MuJoCoJudge' object has no attribute 'last_qpos'`.

- [ ] **Step 3: Modify the implementation**

In `simulation/src/simulation/mujoco_judge/judge.py`, add `self.last_qpos = None` to `__init__`, and record it in `check`:

```python
class MuJoCoJudge:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.last_qpos = None

    def check(self, candidate: dict) -> Verdict:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_kinematics(self.model, self.data)
        reference = self.data.xpos[self.model.body(FOOT_BODY).id].copy()
        target = bbox_to_target(candidate["bbox"], reference)

        converged, qpos_values, _ = solve_leg_ik(self.model, self.data, target)
        self.last_qpos = qpos_values

        if not converged:
            return Verdict.REJECTED
        if not within_joint_limits(self.model, qpos_values):
            return Verdict.REJECTED
        if not settle_and_check_stability(self.model, self.data):
            return Verdict.REJECTED
        return Verdict.CHOSEN
```

(Only the `__init__` body and the two new lines — `self.last_qpos = None` and `self.last_qpos = qpos_values` — are new; the rest of `check` is unchanged from the training-viewer plan.)

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_judge.py -v
```

Expected: all tests in the file PASS, including the new one and the pre-existing ones from the training-viewer plan.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/mujoco_judge/judge.py simulation/tests/test_judge.py
git commit -m "Expose MuJoCoJudge.last_qpos for replay recording"
```

---

## Task 2: Replay-file schema

**Files:**
- Create: `simulation/src/simulation/replay/__init__.py`
- Create: `simulation/src/simulation/replay/schema.py`
- Test: `simulation/tests/test_replay_schema.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `@dataclass CameraSpec(position: list[float], look_at: list[float])`, `@dataclass Keyframe(object: str, action_type: str, verdict: str, start_joint_angles: list[float], end_joint_angles: list[float])`, `@dataclass ReplayFile(scenario_id: str, checkpoint_label: str, generated_at: str, camera: CameraSpec, keyframes: list[Keyframe])`, and `write_replay(path: "pathlib.Path", replay: ReplayFile) -> None`.

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_replay_schema.py
import json

from simulation.replay.schema import CameraSpec, Keyframe, ReplayFile, write_replay


def test_write_replay_produces_dict_free_json(tmp_path):
    replay = ReplayFile(
        scenario_id="fixed_scenario_1",
        checkpoint_label="stub",
        generated_at="2026-07-30T00:00:00Z",
        camera=CameraSpec(position=[0.6, -0.6, 0.4], look_at=[0.15, -0.1, 0.05]),
        keyframes=[
            Keyframe(
                object="red block",
                action_type="grasp",
                verdict="chosen",
                start_joint_angles=[0.0, 0.0, 0.0],
                end_joint_angles=[0.12, -0.35, 0.9],
            )
        ],
    )
    out_path = tmp_path / "replay.json"

    write_replay(out_path, replay)

    loaded = json.loads(out_path.read_text())
    assert loaded["scenario_id"] == "fixed_scenario_1"
    assert loaded["keyframes"][0]["end_joint_angles"] == [0.12, -0.35, 0.9]
    # No JSON object in the tree should have dynamic/name-keyed fields —
    # every dict in the structure must have a fixed, known key set.
    assert set(loaded.keys()) == {"scenario_id", "checkpoint_label", "generated_at", "camera", "keyframes"}
    assert set(loaded["camera"].keys()) == {"position", "look_at"}
    assert set(loaded["keyframes"][0].keys()) == {
        "object", "action_type", "verdict", "start_joint_angles", "end_joint_angles"
    }
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_replay_schema.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'simulation.replay'`.

- [ ] **Step 3: Write the implementation**

Create `simulation/src/simulation/replay/__init__.py` as an empty file (it only marks the directory as a package).

```python
# simulation/src/simulation/replay/schema.py
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path


@dataclass
class CameraSpec:
    position: list[float]
    look_at: list[float]


@dataclass
class Keyframe:
    object: str
    action_type: str
    verdict: str
    start_joint_angles: list[float]
    end_joint_angles: list[float]


@dataclass
class ReplayFile:
    scenario_id: str
    checkpoint_label: str
    generated_at: str
    camera: CameraSpec
    keyframes: list[Keyframe] = field(default_factory=list)


def write_replay(path: Path, replay: ReplayFile) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(replay), indent=2))
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_replay_schema.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/replay/ simulation/tests/test_replay_schema.py
git commit -m "Add dict-free replay-file schema (JsonUtility-compatible)"
```

---

## Task 3: Fixed test scenario

**Files:**
- Create: `simulation/src/simulation/replay/scenarios.py`
- Test: `simulation/tests/test_scenarios.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `FIXED_SCENARIO_ID: str = "fixed_scenario_1"`, `FIXED_CAMERA: CameraSpec`, `fixed_scenario_candidates() -> list[dict]` (same candidate shape as `stubbed_candidates()` from the training-viewer plan — this is the *same* fixed set every call, used for both evaluation and video comparability, per the approved spec).

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_scenarios.py
from simulation.replay.scenarios import FIXED_CAMERA, FIXED_SCENARIO_ID, fixed_scenario_candidates


def test_fixed_scenario_is_deterministic():
    first = fixed_scenario_candidates()
    second = fixed_scenario_candidates()
    assert first == second
    assert len(first) >= 2


def test_fixed_camera_has_position_and_look_at():
    assert len(FIXED_CAMERA.position) == 3
    assert len(FIXED_CAMERA.look_at) == 3


def test_fixed_scenario_id_is_stable():
    assert FIXED_SCENARIO_ID == "fixed_scenario_1"
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_scenarios.py -v
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/replay/scenarios.py
"""The fixed held-out test scenario (team doc §9): the same scenario and
camera are used for every checkpoint recording, so results are directly
comparable over time. No real held-out camera frames exist yet (no real
VLM/camera integration) — this is a fixed, deterministic candidate list,
the same honest-placeholder pattern as `mujoco_judge.sampler`.
"""

from simulation.replay.schema import CameraSpec

FIXED_SCENARIO_ID = "fixed_scenario_1"

FIXED_CAMERA = CameraSpec(position=[0.6, -0.6, 0.4], look_at=[0.15, -0.1, 0.05])


def fixed_scenario_candidates() -> list[dict]:
    return [
        {
            "object": "red block",
            "bbox": {"x": 300.0, "y": 220.0, "width": 40.0, "height": 40.0},
            "confidence": 0.92,
            "action_type": "grasp",
        },
        {
            "object": "blue cup",
            "bbox": {"x": 340.0, "y": 200.0, "width": 30.0, "height": 30.0},
            "confidence": 0.85,
            "action_type": "inspect",
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
uv run pytest tests/test_scenarios.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/simulation/replay/scenarios.py simulation/tests/test_scenarios.py
git commit -m "Add fixed test scenario shared by evaluation and checkpoint video recording"
```

---

## Task 4: `record_checkpoint` — produce a replay file

**Files:**
- Create: `simulation/src/simulation/replay/record_checkpoint.py`
- Test: `simulation/tests/test_record_checkpoint.py`

**Interfaces:**
- Consumes: `MuJoCoJudge` + `Verdict` (training-viewer plan, extended by Task 1 here), `fixed_scenario_candidates`/`FIXED_CAMERA`/`FIXED_SCENARIO_ID` (Task 3), `ReplayFile`/`Keyframe`/`write_replay` (Task 2).
- Produces: `record_checkpoint(model, data, checkpoint_label: str, out_path: "pathlib.Path") -> ReplayFile`. Runnable via `python -m simulation.replay.record_checkpoint --label <name> --out <path>`.

- [ ] **Step 1: Write the failing test**

```python
# simulation/tests/test_record_checkpoint.py
import json

from simulation.replay.record_checkpoint import record_checkpoint


def test_record_checkpoint_writes_one_keyframe_per_candidate(tmp_path, model, data):
    out_path = tmp_path / "replay.json"

    replay = record_checkpoint(model, data, checkpoint_label="stub", out_path=out_path)

    assert len(replay.keyframes) == 3  # matches fixed_scenario_candidates()
    for kf in replay.keyframes:
        assert kf.verdict in {"chosen", "rejected"}
        assert len(kf.start_joint_angles) == 3
        assert len(kf.end_joint_angles) == 3

    loaded = json.loads(out_path.read_text())
    assert loaded["scenario_id"] == "fixed_scenario_1"
    assert loaded["checkpoint_label"] == "stub"
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/test_record_checkpoint.py -v
```

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

```python
# simulation/src/simulation/replay/record_checkpoint.py
import argparse
from datetime import datetime, timezone
from pathlib import Path

import mujoco

from simulation.mujoco_judge.constants import SCENE_PATH
from simulation.mujoco_judge.judge import MuJoCoJudge, Verdict
from simulation.replay.schema import CameraSpec, Keyframe, ReplayFile, write_replay
from simulation.replay.scenarios import FIXED_CAMERA, FIXED_SCENARIO_ID, fixed_scenario_candidates

RESTING_JOINT_ANGLES = [0.0, 0.0, 0.0]


def record_checkpoint(model, data, checkpoint_label: str, out_path: Path) -> ReplayFile:
    """Runs the fixed scenario through the checkpoint's judge and writes a
    replay file. There's no real trained checkpoint yet (`training/train.py`
    is a stub) — this always runs the same stubbed-sampler-backed judge;
    `checkpoint_label` is recorded so future real checkpoints slot in
    without changing this function's shape.
    """
    judge = MuJoCoJudge(model, data)
    keyframes = []

    for candidate in fixed_scenario_candidates():
        verdict = judge.check(candidate)
        keyframes.append(
            Keyframe(
                object=candidate["object"],
                action_type=candidate["action_type"],
                verdict=verdict.value,
                start_joint_angles=list(RESTING_JOINT_ANGLES),
                end_joint_angles=[float(v) for v in judge.last_qpos],
            )
        )

    replay = ReplayFile(
        scenario_id=FIXED_SCENARIO_ID,
        checkpoint_label=checkpoint_label,
        generated_at=datetime.now(timezone.utc).isoformat(),
        camera=CameraSpec(position=list(FIXED_CAMERA.position), look_at=list(FIXED_CAMERA.look_at)),
        keyframes=keyframes,
    )
    write_replay(out_path, replay)
    return replay


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    record_checkpoint(model, data, checkpoint_label=args.label, out_path=args.out)
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/test_record_checkpoint.py -v
```

Expected: PASS.

- [ ] **Step 5: Run the full Python test suite**

```bash
uv run pytest -v
```

Expected: every test across both plans passes.

- [ ] **Step 6: Commit**

```bash
git add simulation/src/simulation/replay/record_checkpoint.py simulation/tests/test_record_checkpoint.py
git commit -m "Add record_checkpoint: fixed scenario -> replay file"
```

- [ ] **Step 7: Generate a real replay file to use in the Unity tasks below**

```bash
cd /Users/divyarth/Projects/grammar-guided-preference-data/simulation
mkdir -p ../simulation/replays
uv run python -m simulation.replay.record_checkpoint --label stub_001 --out replays/stub_001.json
cat replays/stub_001.json
```

Confirm it printed well-formed JSON with 3 keyframes. This file is what Task 7 below will load into Unity.

---

## Task 5: Install Unity

Not code — exact installation steps. No automated verification; confirm manually at each step.

- [ ] **Step 1: Install Unity Hub**

Download from `https://unity.com/download` (macOS, Apple Silicon build) and install it normally (drag to Applications, like any macOS app).

- [ ] **Step 2: Sign in and activate a Personal license**

Open Unity Hub, sign in (create a free Unity ID if needed), go to Preferences (or Hub menu) → Licenses → Add → **Unity Personal** (free tier — appropriate for a student/capstone project with revenue/funding under Unity's Personal-tier threshold).

- [ ] **Step 3: Install the Unity Editor**

In Unity Hub → Installs → Install Editor → choose **Unity 2022.3 LTS** (latest 2022.3.x patch available). During component selection, no extra platform build-support modules are required for this plan (no mobile/console builds).

- [ ] **Step 4: Verify**

```bash
ls /Applications | grep -i unity
```

Expected: `Unity Hub.app` and (after Editor install) a Unity Editor version folder are present. If not present in `/Applications` directly, Unity Hub installs Editors under `/Applications/Unity/Hub/Editor/2022.3.x/` — check there instead.

---

## Task 6: Create the Unity project and import the PhantomX robot

Not automatable — done through the Unity Editor GUI. No automated verification.

- [ ] **Step 1: Create the project**

In Unity Hub → Projects → New Project → template **3D (Built-In Render Pipeline)** (not URP — URDF-Importer and Recorder are both better-tested against Built-In as of Unity 2022.3). Name it `checkpoint-recorder`, location `/Users/divyarth/Projects/grammar-guided-preference-data/simulation/unity/` (create the `unity` parent folder first if Unity Hub doesn't create it for you).

- [ ] **Step 2: Install the URDF-Importer package**

Window → Package Manager → `+` → **Add package from git URL** → paste:
```
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

- [ ] **Step 3: Install the Recorder package**

Window → Package Manager → switch the dropdown from "In Project" to **Unity Registry** → search "Recorder" → Install (`com.unity.recorder`).

- [ ] **Step 4: Import the PhantomX URDF**

Copy (don't move — Plan 1's Python code depends on the original location) the model into the Unity project's `Assets/` folder:

```bash
cp -r /Users/divyarth/Projects/grammar-guided-preference-data/simulation/models/phantomx \
      /Users/divyarth/Projects/grammar-guided-preference-data/simulation/unity/checkpoint-recorder/Assets/phantomx
```

In the Unity Editor: Assets menu → **Import Robot from URDF** → select `Assets/phantomx/urdf/phantomx.urdf`. Accept the default axis conventions when prompted. This creates a GameObject hierarchy in the scene named after the URDF's link names (`MP_BODY`, `c1_rf`, `thigh_rf`, `tibia_rf`, etc.), each with an `ArticulationBody` component.

- [ ] **Step 5: Confirm the hierarchy matches what Task 7's script expects**

In the Hierarchy window, expand the imported robot and find the front-right leg chain: `MP_BODY` → `c1_rf` → `c2_rf` → `thigh_rf` → `tibia_rf`. Note the exact GameObject names — if URDF-Importer renamed any of them (e.g. stripped a prefix), write down the actual names now; Task 7's script uses `transform.Find` with these exact strings and will need adjusting to match.

- [ ] **Step 6: Add a ground plane and camera**

GameObject → 3D Object → Plane (scale it up, e.g. `(4, 1, 4)`, position at the robot's ground level). Position the Main Camera roughly at the `FIXED_CAMERA.position` from `scenarios.py` (`[0.6, -0.6, 0.4]`) looking toward `[0.15, -0.1, 0.05]` — exact framing will be refined once playback is working in Task 7.

---

## Task 7: `PlaybackRecorder.cs` — animate a replay file

**Files:**
- Create: `simulation/unity/checkpoint-recorder/Assets/Scripts/PlaybackRecorder.cs`

No automated test — this is verified by pressing Play in the Editor and watching the leg move. Written as real, complete code, not a stub.

- [ ] **Step 1: Write the script**

```csharp
// simulation/unity/checkpoint-recorder/Assets/Scripts/PlaybackRecorder.cs
using System;
using System.Collections;
using UnityEngine;

[Serializable]
public class CameraSpec
{
    public float[] position;
    public float[] look_at;
}

[Serializable]
public class Keyframe
{
    public string @object;
    public string action_type;
    public string verdict;
    public float[] start_joint_angles;
    public float[] end_joint_angles;
}

[Serializable]
public class ReplayFile
{
    public string scenario_id;
    public string checkpoint_label;
    public string generated_at;
    public CameraSpec camera;
    public Keyframe[] keyframes;
}

public class PlaybackRecorder : MonoBehaviour
{
    [Tooltip("Path to a replay JSON file produced by record_checkpoint.py")]
    public string replayFilePath;

    [Tooltip("Root GameObject of the imported PhantomX robot (drag it here in the Inspector)")]
    public Transform robotRoot;

    [Tooltip("Seconds to animate each keyframe's start -> end joint angles")]
    public float secondsPerKeyframe = 1.0f;

    [Tooltip("Seconds to hold at the end pose before moving to the next keyframe")]
    public float holdSeconds = 0.5f;

    // Front-right leg link names, in LEG_JOINTS order from
    // simulation/mujoco_judge/constants.py. If URDF-Importer produced
    // different names (see Task 6, Step 5), update these three strings.
    private static readonly string[] LegLinkNames = { "c1_rf", "thigh_rf", "tibia_rf" };

    private ArticulationBody[] _legJoints;

    private void Start()
    {
        _legJoints = new ArticulationBody[LegLinkNames.Length];
        for (int i = 0; i < LegLinkNames.Length; i++)
        {
            Transform found = FindDeepChild(robotRoot, LegLinkNames[i]);
            if (found == null)
            {
                Debug.LogError($"PlaybackRecorder: could not find link '{LegLinkNames[i]}' under {robotRoot.name}. Check the actual imported hierarchy (Task 6, Step 5) and update LegLinkNames.");
                return;
            }
            _legJoints[i] = found.GetComponent<ArticulationBody>();
        }

        string json = System.IO.File.ReadAllText(replayFilePath);
        ReplayFile replay = JsonUtility.FromJson<ReplayFile>(json);

        if (replay.camera != null && Camera.main != null)
        {
            Camera.main.transform.position = new Vector3(
                replay.camera.position[0], replay.camera.position[1], replay.camera.position[2]);
            Camera.main.transform.LookAt(new Vector3(
                replay.camera.look_at[0], replay.camera.look_at[1], replay.camera.look_at[2]));
        }

        StartCoroutine(PlayKeyframes(replay));
    }

    private IEnumerator PlayKeyframes(ReplayFile replay)
    {
        foreach (Keyframe kf in replay.keyframes)
        {
            float elapsed = 0f;
            while (elapsed < secondsPerKeyframe)
            {
                float t = elapsed / secondsPerKeyframe;
                for (int i = 0; i < _legJoints.Length; i++)
                {
                    float startDeg = kf.start_joint_angles[i] * Mathf.Rad2Deg;
                    float endDeg = kf.end_joint_angles[i] * Mathf.Rad2Deg;
                    SetJointTarget(_legJoints[i], Mathf.Lerp(startDeg, endDeg, t));
                }
                elapsed += Time.deltaTime;
                yield return null;
            }
            for (int i = 0; i < _legJoints.Length; i++)
            {
                SetJointTarget(_legJoints[i], kf.end_joint_angles[i] * Mathf.Rad2Deg);
            }
            yield return new WaitForSeconds(holdSeconds);
        }
    }

    private static void SetJointTarget(ArticulationBody joint, float targetDegrees)
    {
        ArticulationDrive drive = joint.xDrive;
        drive.target = targetDegrees;
        joint.xDrive = drive;
    }

    private static Transform FindDeepChild(Transform parent, string name)
    {
        foreach (Transform child in parent)
        {
            if (child.name == name) return child;
            Transform result = FindDeepChild(child, name);
            if (result != null) return result;
        }
        return null;
    }
}
```

- [ ] **Step 2: Attach and wire the script**

In the Unity Editor: create an empty GameObject named `PlaybackController`, add the `PlaybackRecorder` component to it. In the Inspector, drag the imported robot's root GameObject (`MP_BODY` or its parent, whichever is the top of the hierarchy from Task 6) into the `Robot Root` field, and set `Replay File Path` to the absolute path of the file generated in Task 4 Step 7 (e.g. `/Users/divyarth/Projects/grammar-guided-preference-data/simulation/replays/stub_001.json`).

- [ ] **Step 3: Press Play and verify**

Press Play in the Editor. Expected: the camera moves to the fixed position, and the front-right leg visibly rotates through each keyframe's start→end angles, holding briefly between candidates. If `FindDeepChild` logs an error, go back to Task 6 Step 5, confirm the actual link names, and update `LegLinkNames`.

- [ ] **Step 4: Commit**

```bash
cd /Users/divyarth/Projects/grammar-guided-preference-data
git add simulation/unity/
git commit -m "Add Unity project with PlaybackRecorder animating replay files"
```

(This will commit Unity project metadata files too — that's expected and normal for a Unity project; do not hand-pick individual files out of `Assets/`, `ProjectSettings/`, and `Packages/`.)

---

## Task 8: Record and export the checkpoint video

Manual verification — the actual deliverable this whole plan was for.

- [ ] **Step 1: Open the Recorder window**

Window → General → Recorder → Recorder Window.

- [ ] **Step 2: Configure a Movie recording**

Add Recorder → **Movie**. Set:
- Output Format: MP4
- Source: Game View (or Targeted Camera → Main Camera)
- Output File: `simulation/replays/stub_001.mp4` (relative to the Unity project, or an absolute path)
- Duration: Manual (stop it yourself once playback finishes — the total playback time is roughly `len(keyframes) * (secondsPerKeyframe + holdSeconds)`, e.g. 3 keyframes × 1.5s ≈ 4.5s for the `stub_001` replay from Task 4)

- [ ] **Step 3: Record**

Press Record in the Recorder window, then Play in the Editor. Let it run through all keyframes, then stop the recording.

- [ ] **Step 4: Verify the output**

```bash
ls -la /Users/divyarth/Projects/grammar-guided-preference-data/simulation/replays/stub_001.mp4
```

Expected: a non-trivially-sized `.mp4` file exists. Open it in QuickTime and confirm it shows the leg moving through the scenario — this is the artifact the whole brainstorming session was aimed at producing.

---

## Plan-level verification

1. `cd simulation && uv run pytest -v` — Tasks 1-4's Python tests all pass (and Plan 1's tests still pass, since Task 1 here only adds to `judge.py`, not changes its existing behavior).
2. `uv run python -m simulation.replay.record_checkpoint --label stub_002 --out replays/stub_002.json` — runs cleanly, produces a second replay file.
3. In Unity, swap `PlaybackController`'s `Replay File Path` to `stub_002.json`, press Play, confirm it plays through the (likely different, since MuJoCo's judge is deterministic given the same inputs but this is still real physics) keyframes without error.
4. An actual `.mp4` file exists on disk and visibly shows the robot leg moving, per Task 8.

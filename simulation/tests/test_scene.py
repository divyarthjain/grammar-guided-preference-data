from simulation.mujoco_judge.constants import LEG_JOINTS, FOOT_BODY, FLOOR_GEOM


def test_scene_loads_with_expected_joints(model):
    assert model.njnt == 19  # 18 actuated leg hinges + 1 free base joint
    for name in LEG_JOINTS:
        assert model.joint(name).id >= 0


def test_scene_has_foot_body_and_floor(model):
    assert model.body(FOOT_BODY).id >= 0
    assert model.geom(FLOOR_GEOM).id >= 0

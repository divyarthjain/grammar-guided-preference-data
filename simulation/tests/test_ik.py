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

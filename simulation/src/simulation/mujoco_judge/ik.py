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

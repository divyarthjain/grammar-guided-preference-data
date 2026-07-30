from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parents[3] / "models"
SCENE_PATH = MODELS_DIR / "phantomx" / "scene.xml"

LEG_JOINTS = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
FOOT_BODY = "tibia_rf"
BASE_BODY = "MP_BODY"
FLOOR_GEOM = "floor"

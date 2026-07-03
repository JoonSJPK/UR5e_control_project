from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CALIB = ROOT / "docs" / "task5_data" / "hand_eye.json"


def load_homography(path=CALIB):
    if not Path(path).exists():
        raise SystemExit(f"no camera calibration at {path} - run calibrate_dots.py")
    data = json.loads(Path(path).read_text())
    return np.array(data["H"], dtype=float)


def pixel_to_table(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])

from __future__ import annotations

import numpy as np


def crop_workspace(points: np.ndarray, min_xyz: list[float], max_xyz: list[float]) -> np.ndarray:
    lo = np.asarray(min_xyz, dtype=float)
    hi = np.asarray(max_xyz, dtype=float)
    mask = np.all((points >= lo) & (points <= hi), axis=1)
    return points[mask]

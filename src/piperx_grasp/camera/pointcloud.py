from __future__ import annotations

import numpy as np


def depth_to_points(depth_m: np.ndarray, intrinsics: dict[str, float]) -> np.ndarray:
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])
    v, u = np.indices(depth_m.shape)
    z = depth_m.astype(float)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=-1).reshape(-1, 3)

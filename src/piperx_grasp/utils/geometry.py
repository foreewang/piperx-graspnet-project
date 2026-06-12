from __future__ import annotations

import numpy as np


def ensure_homogeneous(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 homogeneous matrix.")
    return matrix

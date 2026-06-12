from __future__ import annotations

import math

import numpy as np


def load_matrix(value: list[list[float]]) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 matrix.")
    return matrix


def xyz_rpy_to_matrix(pose: list[float]) -> np.ndarray:
    if len(pose) != 6:
        raise ValueError("Expected pose [x, y, z, roll, pitch, yaw].")
    x, y, z, roll, pitch, yaw = pose
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    matrix = np.eye(4)
    matrix[:3, :3] = rz @ ry @ rx
    matrix[:3, 3] = [x, y, z]
    return matrix


def matrix_to_xyz_rpy(matrix: np.ndarray) -> list[float]:
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 matrix.")
    r = matrix[:3, :3]
    sy = -r[2, 0]
    pitch = math.asin(max(-1.0, min(1.0, sy)))
    cp = math.cos(pitch)
    if abs(cp) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-r[0, 1], r[1, 1])
    x, y, z = matrix[:3, 3]
    return [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    inv = np.eye(4)
    inv[:3, :3] = matrix[:3, :3].T
    inv[:3, 3] = -inv[:3, :3] @ matrix[:3, 3]
    return inv

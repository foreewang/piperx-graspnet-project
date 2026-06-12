from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PickPlan:
    pregrasp_pose: list[float]
    grasp_pose: list[float]
    lift_pose: list[float]
    gripper_width_m: float
    score: float


def make_lift_pose(pose: list[float], dz: float) -> list[float]:
    if len(pose) != 6:
        raise ValueError("Expected pose [x, y, z, roll, pitch, yaw].")
    target = list(pose)
    target[2] += dz
    return target


def make_pregrasp_pose(grasp_pose: list[float], approach_offset_m: float) -> list[float]:
    pregrasp = list(grasp_pose)
    pregrasp[2] += approach_offset_m
    return pregrasp

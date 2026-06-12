from __future__ import annotations

from piperx_grasp.calibration.transforms import load_matrix, matrix_to_xyz_rpy
from piperx_grasp.grasp.grasp_types import GraspCandidate
from piperx_grasp.robot.motion_planner import PickPlan, make_lift_pose, make_pregrasp_pose


def plan_pick(
    grasp: GraspCandidate,
    robot_cfg: dict,
    calibration_cfg: dict,
) -> PickPlan:
    T_base_camera = load_matrix(calibration_cfg["calibration"]["T_base_camera"])
    T_base_grasp = T_base_camera @ grasp.T_camera_grasp
    grasp_pose = matrix_to_xyz_rpy(T_base_grasp)
    motion_cfg = robot_cfg.get("motion", {})
    pregrasp = make_pregrasp_pose(
        grasp_pose,
        float(motion_cfg.get("pregrasp_offset_m", 0.08)),
    )
    lift = make_lift_pose(
        grasp_pose,
        float(motion_cfg.get("lift_offset_m", 0.08)),
    )
    return PickPlan(
        pregrasp_pose=pregrasp,
        grasp_pose=grasp_pose,
        lift_pose=lift,
        gripper_width_m=grasp.width_m,
        score=grasp.score,
    )

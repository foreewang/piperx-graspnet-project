import argparse

import _bootstrap  # noqa: F401

from piperx_grasp.config import load_config
from piperx_grasp.grasp.grasp_selector import select_best_grasp
from piperx_grasp.grasp.graspnet_runner import GraspNetRunner
from piperx_grasp.pipeline.plan_grasp import plan_pick
from piperx_grasp.planning.moveit_contract import pick_plan_to_motion_request
from piperx_grasp.robot.motion_planner import PickPlan, make_lift_pose, make_pregrasp_pose


def main() -> int:
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--grasp-output")
    source.add_argument(
        "--base-pose",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Manual grasp pose in robot base frame, meters and radians.",
    )
    parser.add_argument("--gripper-width", type=float, default=0.04)
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    robot_cfg = load_config("configs/robot.yaml")
    moveit_cfg = load_config("configs/ros2_moveit.yaml")

    best = None
    if args.grasp_output:
        grasp_cfg = load_config("configs/graspnet.yaml")
        calibration_cfg = load_config("configs/calibration.yaml")
        candidates = GraspNetRunner.from_config(grasp_cfg).load_candidates_npz(
            args.grasp_output
        )
        best = select_best_grasp(candidates, grasp_cfg.get("selection", {}))
        pick_plan = plan_pick(best, robot_cfg, calibration_cfg)
    else:
        motion_cfg = robot_cfg.get("motion", {})
        grasp_pose = list(args.base_pose)
        pick_plan = PickPlan(
            pregrasp_pose=make_pregrasp_pose(
                grasp_pose,
                float(motion_cfg.get("pregrasp_offset_m", 0.08)),
            ),
            grasp_pose=grasp_pose,
            lift_pose=make_lift_pose(
                grasp_pose,
                float(motion_cfg.get("lift_offset_m", 0.08)),
            ),
            gripper_width_m=float(args.gripper_width),
            score=1.0,
        )

    request = pick_plan_to_motion_request(pick_plan, moveit_cfg.get("moveit", {}))

    output_path = args.out or moveit_cfg["moveit"]["request_path"]
    request.save_json(output_path)
    if best is not None:
        print("selected grasp:", best)
    print("pick plan:", pick_plan)
    print("moveit request:", output_path)
    print("dry-run only; run this request in ROS2/MoveIt before hardware execution.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

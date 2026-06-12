import argparse

import _bootstrap  # noqa: F401

from piperx_grasp.config import load_config
from piperx_grasp.grasp.grasp_selector import select_best_grasp
from piperx_grasp.grasp.graspnet_runner import GraspNetRunner
from piperx_grasp.pipeline.execute_grasp import execute_pick
from piperx_grasp.pipeline.plan_grasp import plan_pick
from piperx_grasp.robot.piperx_client import PiperXClient


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--grasp-output", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    robot_cfg = load_config("configs/robot.yaml")
    grasp_cfg = load_config("configs/graspnet.yaml")
    calibration_cfg = load_config("configs/calibration.yaml")

    candidates = GraspNetRunner.from_config(grasp_cfg).load_candidates_npz(args.grasp_output)
    best = select_best_grasp(candidates, grasp_cfg.get("selection", {}))
    plan = plan_pick(best, robot_cfg, calibration_cfg)
    print(plan)

    if not args.execute:
        print("dry-run only; pass --execute to move.")
        return 0

    client = PiperXClient.from_config(robot_cfg)
    client.connect()
    try:
        execute_pick(client, plan)
        return 0
    finally:
        client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

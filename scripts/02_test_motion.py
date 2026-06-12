import argparse

import _bootstrap  # noqa: F401

from piperx_grasp.config import load_config
from piperx_grasp.robot.motion_planner import make_lift_pose
from piperx_grasp.robot.piperx_client import PiperXClient


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dz", type=float, default=0.005, help="Z lift in meters")
    parser.add_argument("--speed", type=int, default=5, help="speed percent")
    args = parser.parse_args()

    if args.dz <= 0 or args.dz > 0.03:
        raise ValueError("--dz must be in (0, 0.03] meters for this safety test.")

    cfg = load_config("configs/robot.yaml")
    client = PiperXClient.from_config(cfg)
    client.connect()
    try:
        current = client.wait_flange_pose()
        target = make_lift_pose(current, dz=args.dz)
        print("current:", current)
        print("target:", target)
        if not args.execute:
            print("dry-run only; pass --execute to move.")
            return 0
        client.enable()
        client.set_speed_percent(args.speed)
        client.move_l(target)
        if not client.wait_motion_done():
            raise TimeoutError("motion timeout")
        print("final:", client.get_flange_pose())
        print("status:", client.get_arm_status())
        return 0
    finally:
        client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

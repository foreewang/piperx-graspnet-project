import argparse
import time

import _bootstrap  # noqa: F401

from piperx_grasp.config import load_config
from piperx_grasp.robot.gripper import GripperClient
from piperx_grasp.robot.piperx_client import PiperXClient


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    cfg = load_config("configs/robot.yaml")
    client = PiperXClient.from_config(cfg)
    client.connect()
    try:
        gripper = GripperClient(client.robot, cfg.get("gripper", {}))
        print("gripper status:", gripper.get_status())
        if not args.execute:
            print("dry-run only; pass --execute to open and close.")
            return 0
        gripper.open()
        time.sleep(0.5)
        gripper.close()
        return 0
    finally:
        client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

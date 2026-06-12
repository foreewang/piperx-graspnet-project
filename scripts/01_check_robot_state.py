import _bootstrap  # noqa: F401

from piperx_grasp.config import load_config
from piperx_grasp.robot.piperx_client import PiperXClient


def main() -> int:
    robot_cfg = load_config("configs/robot.yaml")
    client = PiperXClient.from_config(robot_cfg)
    client.connect()
    try:
        print("connected:", client.is_connected())
        print("joint_angles:", client.get_joint_angles())
        print("flange_pose:", client.get_flange_pose())
        print("arm_status:", client.get_arm_status())
        return 0
    finally:
        client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

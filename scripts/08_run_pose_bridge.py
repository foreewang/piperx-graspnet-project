import _bootstrap  # noqa: F401

from piperx_grasp.bridge import PiperFeedbackPoseProvider, PoseBridgeClient
from piperx_grasp.config import load_config
from piperx_grasp.robot.piperx_client import PiperXClient


def main() -> int:
    robot_config = load_config("configs/robot.yaml")
    bridge_config = load_config("configs/pose_bridge.yaml")
    robot = PiperXClient.from_config(robot_config)
    robot.connect()
    try:
        print("robot connected:", robot.is_connected())
        print("read-only pose bridge; this process does not enable or move the robot")
        provider = PiperFeedbackPoseProvider.from_config(robot, bridge_config)
        bridge = PoseBridgeClient.from_config(provider, bridge_config)
        bridge.run_forever()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

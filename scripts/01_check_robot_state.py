import _bootstrap  # noqa: F401

from piperx_grasp.config import load_config
from piperx_grasp.robot.piperx_client import PiperXClient


def main() -> int:
    robot_cfg = load_config("configs/robot.yaml")
    client = PiperXClient.from_config(robot_cfg)
    client.connect()
    try:
        print("connected:", client.is_connected())
        success = True
        try:
            joint_angles = client.wait_joint_angles(timeout_s=3.0)
        except TimeoutError as error:
            joint_angles = None
            success = False
            print("joint feedback error:", error)
        print("joint_angles:", joint_angles)
        try:
            flange_pose = client.wait_flange_pose(
                timeout_s=3.0,
                min_z=float("-inf"),
            )
        except TimeoutError as error:
            flange_pose = None
            success = False
            print("flange feedback error:", error)
        print("flange_pose:", flange_pose)
        try:
            arm_status = client.wait_arm_status(timeout_s=3.0)
        except TimeoutError as error:
            arm_status = None
            success = False
            print("arm status error:", error)
        print("arm_status:", arm_status)
        print("feedback_frames:", client.get_feedback_frame_status())
        return 0 if success else 1
    finally:
        client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

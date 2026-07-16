import argparse
import time
from threading import Event

import _bootstrap  # noqa: F401

from piperx_grasp.bridge import PoseBridgeClient


class StaticPoseProvider:
    def __init__(self, stop_event: Event) -> None:
        self.stop_event = stop_event

    def capture(self, _request):
        observed_at = time.time_ns()
        self.stop_event.set()
        return {
            "observed_at_unix_ns": observed_at,
            "pose": {
                "parent_frame": "base_link",
                "child_frame": "gripper_base",
                "translation_m": [0.25, 0.03, 0.31],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "robot_state": {
                "source": "actual_feedback",
                "motion_state": "stopped",
                "stable_for_ms": 1000,
                "max_abs_joint_velocity_rad_s": 0.0,
                "joint_names": [
                    "joint1",
                    "joint2",
                    "joint3",
                    "joint4",
                    "joint5",
                    "joint6",
                ],
                "joint_positions_rad": [0.0] * 6,
            },
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    stop = Event()
    bridge = PoseBridgeClient(
        provider=StaticPoseProvider(stop),
        host=args.host,
        port=args.port,
        client_id="windows-piperx-mock-client",
        robot_id="piperx-mock",
    )
    bridge.run_forever(stop)
    print("mock pose sent successfully")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


def _resolve_constant(group: Any, name: str) -> Any:
    return getattr(group, str(name).upper())


@dataclass
class PiperXClient:
    robot_model: str = "PIPER_X"
    firmware: str = "DEFAULT"
    interface: str = "agx_cando"
    channel: str = "0"
    bitrate: int = 1_000_000
    timeout: float = 1.0
    receive_own_messages: bool = False
    local_loopback: bool = False

    def __post_init__(self) -> None:
        self.robot = None

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "PiperXClient":
        robot_cfg = config.get("robot", config)
        return cls(
            robot_model=robot_cfg.get("model", "PIPER_X"),
            firmware=robot_cfg.get("firmware", "DEFAULT"),
            interface=robot_cfg.get("interface", "agx_cando"),
            channel=str(robot_cfg.get("channel", "0")),
            bitrate=int(robot_cfg.get("bitrate", 1_000_000)),
            timeout=float(robot_cfg.get("timeout", 1.0)),
            receive_own_messages=bool(robot_cfg.get("receive_own_messages", False)),
            local_loopback=bool(robot_cfg.get("local_loopback", False)),
        )

    def connect(self) -> None:
        from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

        cfg = create_agx_arm_config(
            robot=_resolve_constant(ArmModel, self.robot_model),
            firmeware_version=_resolve_constant(PiperFW, self.firmware),
            interface=self.interface,
            channel=self.channel,
            bitrate=self.bitrate,
            timeout=self.timeout,
            receive_own_messages=self.receive_own_messages,
            local_loopback=self.local_loopback,
        )
        self.robot = AgxArmFactory.create_arm(cfg)
        self.robot.connect()

    def disconnect(self) -> None:
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

    def enable(self, timeout_s: float = 5.0, poll_s: float = 0.05) -> None:
        import time

        robot = self._require_robot()
        start_t = time.monotonic()
        while not robot.enable():
            if time.monotonic() - start_t > timeout_s:
                raise TimeoutError("enable timeout")
            time.sleep(poll_s)

    def set_speed_percent(self, value: int) -> Any:
        return self._require_robot().set_speed_percent(value)

    def wait_motion_done(self, timeout_s: float = 8.0, poll_s: float = 0.1) -> bool:
        import time

        start_t = time.monotonic()
        while True:
            status = self.get_arm_status()
            if status is not None and getattr(status, "motion_status", None) == 0:
                return True
            if time.monotonic() - start_t > timeout_s:
                return False
            time.sleep(poll_s)

    def wait_flange_pose(
        self,
        timeout_s: float = 2.0,
        poll_s: float = 0.05,
        min_z: float = 0.03,
    ) -> list[float]:
        import time

        start_t = time.monotonic()
        last_pose = None
        while time.monotonic() - start_t <= timeout_s:
            pose = self.get_flange_pose()
            if pose is not None:
                last_pose = pose
                if len(pose) == 6 and pose[2] >= min_z:
                    return pose
            time.sleep(poll_s)
        raise TimeoutError(f"no valid flange pose received; last_pose={last_pose}")

    def is_connected(self) -> bool:
        return bool(self.robot is not None and self.robot.is_connected())

    def _require_robot(self) -> Any:
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")
        return self.robot

    def get_joint_angles(self) -> Any:
        result = self._require_robot().get_joint_angles()
        return None if result is None else result.msg

    def get_flange_pose(self) -> Any:
        result = self._require_robot().get_flange_pose()
        return None if result is None else result.msg

    def get_arm_status(self) -> Any:
        result = self._require_robot().get_arm_status()
        return None if result is None else result.msg

    def move_j(self, joints: list[float]) -> Any:
        return self._require_robot().move_j(joints)

    def move_p(self, pose: list[float]) -> Any:
        return self._require_robot().move_p(pose)

    def move_l(self, pose: list[float]) -> Any:
        return self._require_robot().move_l(pose)

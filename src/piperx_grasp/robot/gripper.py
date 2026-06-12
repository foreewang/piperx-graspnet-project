from __future__ import annotations

from typing import Any


class GripperClient:
    def __init__(self, robot: Any, config: dict[str, Any]) -> None:
        self.robot = robot
        self.config = config
        self._driver = None

    def _get_driver(self) -> Any:
        if self._driver is None:
            effectors = getattr(self.robot, "OPTIONS").EFFECTOR
            self._driver = self.robot.init_effector(effectors.AGX_GRIPPER)
        return self._driver

    def get_status(self) -> Any:
        driver = self._get_driver()
        if hasattr(driver, "get_gripper_status"):
            return driver.get_gripper_status()
        return None

    def open(self) -> Any:
        return self.move(float(self.config.get("open_width_m", 0.07)))

    def close(self) -> Any:
        return self.move(float(self.config.get("close_width_m", 0.0)))

    def move(self, width_m: float) -> Any:
        driver = self._get_driver()
        if hasattr(driver, "move_gripper_m"):
            return driver.move_gripper_m(width_m)
        if hasattr(driver, "move_gripper_deg"):
            return driver.move_gripper_deg(width_m)
        raise AttributeError("Unsupported gripper driver API.")

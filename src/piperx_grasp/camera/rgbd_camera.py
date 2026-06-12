from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class RgbdCapture:
    rgb: Any
    depth: Any
    intrinsics: dict[str, float]


class RgbdCamera:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "RgbdCamera":
        return cls(config.get("camera", config))

    def capture(self) -> RgbdCapture:
        raise NotImplementedError(
            "Add your RGB-D camera adapter here, then return RgbdCapture."
        )

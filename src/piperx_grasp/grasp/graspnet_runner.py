from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from .grasp_types import GraspCandidate


class GraspNetRunner:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "GraspNetRunner":
        return cls(config.get("graspnet", config))

    def infer(self, points: np.ndarray) -> list[GraspCandidate]:
        raise NotImplementedError("Wire GraspNet inference here.")

    def load_candidates_npz(self, path: str | Path) -> list[GraspCandidate]:
        data = np.load(path)
        poses = data["poses"]
        scores = data["scores"]
        widths = data["widths"] if "widths" in data else np.zeros(len(scores))
        return [
            GraspCandidate(
                score=float(scores[i]),
                width_m=float(widths[i]),
                T_camera_grasp=np.asarray(poses[i], dtype=float),
                metadata={"index": i},
            )
            for i in range(len(scores))
        ]

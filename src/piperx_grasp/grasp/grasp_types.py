from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class GraspCandidate:
    score: float
    width_m: float
    T_camera_grasp: np.ndarray
    metadata: dict = field(default_factory=dict)

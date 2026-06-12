import numpy as np

from piperx_grasp.grasp.grasp_selector import select_best_grasp
from piperx_grasp.grasp.grasp_types import GraspCandidate


def test_select_best_grasp_by_score():
    candidates = [
        GraspCandidate(0.2, 0.03, np.eye(4)),
        GraspCandidate(0.8, 0.04, np.eye(4)),
    ]
    best = select_best_grasp(candidates, {"min_score": 0.1, "max_width_m": 0.08})
    assert best.score == 0.8

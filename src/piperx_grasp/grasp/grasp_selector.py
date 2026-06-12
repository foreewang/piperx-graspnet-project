from __future__ import annotations

from .grasp_types import GraspCandidate


def select_best_grasp(
    candidates: list[GraspCandidate],
    config: dict | None = None,
) -> GraspCandidate:
    config = config or {}
    min_score = float(config.get("min_score", 0.0))
    max_width = float(config.get("max_width_m", 1.0))
    valid = [
        g for g in candidates
        if g.score >= min_score and g.width_m <= max_width
    ]
    if not valid:
        raise ValueError("No valid grasp candidates.")
    return max(valid, key=lambda grasp: grasp.score)

from __future__ import annotations


def assert_pose_step_safe(current: list[float], target: list[float], max_step_m: float) -> None:
    if current is None:
        raise ValueError("Current pose is required.")
    if len(current) != 6 or len(target) != 6:
        raise ValueError("Expected 6D poses.")
    delta = max(abs(target[i] - current[i]) for i in range(3))
    if delta > max_step_m:
        raise ValueError(f"Linear step {delta:.4f}m exceeds limit {max_step_m:.4f}m.")

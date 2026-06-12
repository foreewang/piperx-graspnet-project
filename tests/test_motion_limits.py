import pytest

from piperx_grasp.robot.safety import assert_pose_step_safe


def test_pose_step_limit_rejects_large_move():
    with pytest.raises(ValueError):
        assert_pose_step_safe(
            [0, 0, 0, 0, 0, 0],
            [0.1, 0, 0, 0, 0, 0],
            max_step_m=0.02,
        )

from dataclasses import dataclass

import numpy as np
import pytest

from piperx_grasp.bridge.pose_bridge_client import (
    PiperFeedbackPoseProvider,
    PoseUnavailableError,
)


@dataclass
class FakeStatus:
    motion_status: int = 0


class FakeRobotClient:
    def __init__(self):
        self.pose_feedback_timestamp = 0.0

    def is_connected(self):
        return True

    def get_arm_status(self):
        return FakeStatus()

    def get_joint_velocities(self):
        return [0.0] * 6

    def get_flange_pose(self):
        return [0.2, 0.0, 0.3, 0.0, 0.0, 0.0]

    def get_flange_pose_feedback(self):
        self.pose_feedback_timestamp += 1.0
        return self.get_flange_pose(), self.pose_feedback_timestamp

    def get_joint_angles(self):
        return [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_provider_converts_flange_pose_to_gripper_base():
    flange_to_gripper = np.array([
        [0.0, -1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    provider = PiperFeedbackPoseProvider(
        client=FakeRobotClient(),
        flange_to_gripper_base=flange_to_gripper,
    )
    request = {
        "required_parent_frame": "base_link",
        "required_child_frame": "gripper_base",
        "minimum_stable_for_ms": 0,
    }

    feedback = provider.capture(request)

    assert feedback["pose"]["translation_m"] == [0.2, 0.0, 0.3]
    assert np.allclose(
        feedback["pose"]["rotation_xyzw"],
        [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)],
    )
    assert feedback["robot_state"]["source"] == "actual_feedback"


def test_provider_rejects_stale_flange_feedback():
    robot = FakeRobotClient()
    robot.get_flange_pose_feedback = lambda: (
        robot.get_flange_pose(),
        1.0,
    )
    provider = PiperFeedbackPoseProvider(
        client=robot,
        flange_to_gripper_base=np.eye(4),
        stability_wait_timeout_s=0.02,
        poll_interval_s=0.001,
    )
    request = {
        "required_parent_frame": "base_link",
        "required_child_frame": "gripper_base",
        "minimum_stable_for_ms": 0,
    }

    with pytest.raises(PoseUnavailableError, match="no fresh flange feedback"):
        provider.capture(request)


def test_provider_rejects_missing_joint_feedback():
    robot = FakeRobotClient()
    robot.get_joint_angles = lambda: None
    provider = PiperFeedbackPoseProvider(
        client=robot,
        flange_to_gripper_base=np.eye(4),
        stability_wait_timeout_s=0.02,
        poll_interval_s=0.001,
    )
    request = {
        "required_parent_frame": "base_link",
        "required_child_frame": "gripper_base",
        "minimum_stable_for_ms": 0,
    }

    with pytest.raises(PoseUnavailableError, match="joint_angles"):
        provider.capture(request)

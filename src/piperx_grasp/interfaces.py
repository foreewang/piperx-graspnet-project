from __future__ import annotations

from typing import Any, Protocol

import numpy as np

from piperx_grasp.camera.rgbd_camera import RgbdCapture
from piperx_grasp.grasp.grasp_types import GraspCandidate
from piperx_grasp.planning.moveit_contract import MotionPlanRequest, MotionPlanResult
from piperx_grasp.robot.motion_planner import PickPlan


class RgbdCameraAdapter(Protocol):
    def capture(self) -> RgbdCapture:
        """Capture one RGB-D frame."""


class GraspGenerator(Protocol):
    def infer(self, points: np.ndarray) -> list[GraspCandidate]:
        """Generate grasp candidates from a point cloud."""


class CalibrationProvider(Protocol):
    def T_base_camera(self) -> np.ndarray:
        """Return the camera-to-robot-base transform."""

    def T_flange_tcp(self) -> np.ndarray:
        """Return the flange-to-TCP transform."""


class PickPlanner(Protocol):
    def plan(self, grasp: GraspCandidate) -> PickPlan:
        """Build a robot-level pick plan from a selected grasp."""


class MotionPlanner(Protocol):
    def plan(self, request: MotionPlanRequest) -> MotionPlanResult:
        """Plan a motion trajectory for a robot or simulator."""


class PickExecutor(Protocol):
    def execute(self, plan: PickPlan) -> Any:
        """Execute a pick plan on hardware or simulation."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from piperx_grasp.robot.motion_planner import PickPlan


@dataclass(frozen=True)
class Pose6D:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    @classmethod
    def from_list(cls, value: list[float]) -> "Pose6D":
        if len(value) != 6:
            raise ValueError("Expected pose [x, y, z, roll, pitch, yaw].")
        return cls(*(float(item) for item in value))

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]


@dataclass(frozen=True)
class MotionWaypoint:
    name: str
    pose: Pose6D


@dataclass(frozen=True)
class MotionPlanRequest:
    planning_group: str
    base_frame: str
    end_effector_link: str
    waypoints: list[MotionWaypoint]
    planner_id: str = ""
    allowed_planning_time_s: float = 5.0
    max_velocity_scaling_factor: float = 0.1
    max_acceleration_scaling_factor: float = 0.1
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MotionPlanRequest":
        waypoints = [
            MotionWaypoint(
                name=str(item["name"]),
                pose=Pose6D.from_list(
                    [
                        item["pose"]["x"],
                        item["pose"]["y"],
                        item["pose"]["z"],
                        item["pose"]["roll"],
                        item["pose"]["pitch"],
                        item["pose"]["yaw"],
                    ]
                ),
            )
            for item in data["waypoints"]
        ]
        return cls(
            planning_group=str(data["planning_group"]),
            base_frame=str(data["base_frame"]),
            end_effector_link=str(data["end_effector_link"]),
            waypoints=waypoints,
            planner_id=str(data.get("planner_id", "")),
            allowed_planning_time_s=float(data.get("allowed_planning_time_s", 5.0)),
            max_velocity_scaling_factor=float(
                data.get("max_velocity_scaling_factor", 0.1)
            ),
            max_acceleration_scaling_factor=float(
                data.get("max_acceleration_scaling_factor", 0.1)
            ),
            metadata=dict(data.get("metadata", {})),
        )

    def save_json(self, path: str | Path) -> None:
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(self.to_dict(), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    @classmethod
    def load_json(cls, path: str | Path) -> "MotionPlanRequest":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


@dataclass(frozen=True)
class MotionPlanResult:
    success: bool
    message: str
    trajectory: dict[str, Any] | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def save_json(self, path: str | Path) -> None:
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(asdict(self), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )


def pick_plan_to_motion_request(
    plan: PickPlan,
    moveit_cfg: dict[str, Any],
) -> MotionPlanRequest:
    waypoints = [
        MotionWaypoint("pregrasp", Pose6D.from_list(plan.pregrasp_pose)),
        MotionWaypoint("grasp", Pose6D.from_list(plan.grasp_pose)),
        MotionWaypoint("lift", Pose6D.from_list(plan.lift_pose)),
    ]
    place_pose = moveit_cfg.get("place_pose")
    if place_pose is not None:
        waypoints.append(MotionWaypoint("place", Pose6D.from_list(place_pose)))
    return MotionPlanRequest(
        planning_group=str(moveit_cfg.get("planning_group", "piper_arm")),
        base_frame=str(moveit_cfg.get("base_frame", "piperx_base")),
        end_effector_link=str(moveit_cfg.get("end_effector_link", "tool0")),
        waypoints=waypoints,
        planner_id=str(moveit_cfg.get("planner_id", "")),
        allowed_planning_time_s=float(moveit_cfg.get("allowed_planning_time_s", 5.0)),
        max_velocity_scaling_factor=float(moveit_cfg.get("velocity_scaling", 0.1)),
        max_acceleration_scaling_factor=float(
            moveit_cfg.get("acceleration_scaling", 0.1)
        ),
        metadata={
            "score": plan.score,
            "gripper_width_m": plan.gripper_width_m,
        },
    )

from __future__ import annotations

from piperx_grasp.robot.gripper import GripperClient
from piperx_grasp.robot.motion_planner import PickPlan
from piperx_grasp.robot.piperx_client import PiperXClient


def execute_pick(client: PiperXClient, plan: PickPlan) -> None:
    if client.robot is None:
        raise RuntimeError("Robot is not connected.")
    gripper = GripperClient(client.robot, {})
    gripper.open()
    client.move_p(plan.pregrasp_pose)
    client.move_l(plan.grasp_pose)
    gripper.move(plan.gripper_width_m)
    client.move_l(plan.lift_pose)

from piperx_grasp.planning.moveit_contract import (
    MotionPlanRequest,
    pick_plan_to_motion_request,
)
from piperx_grasp.robot.motion_planner import PickPlan


def test_pick_plan_to_motion_request_round_trip():
    plan = PickPlan(
        pregrasp_pose=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
        grasp_pose=[0.1, 0.2, 0.2, 0.0, 0.0, 0.0],
        lift_pose=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
        gripper_width_m=0.04,
        score=0.8,
    )
    request = pick_plan_to_motion_request(
        plan,
        {
            "planning_group": "piper_arm",
            "base_frame": "piperx_base",
            "end_effector_link": "tool0",
        },
    )

    assert [waypoint.name for waypoint in request.waypoints] == [
        "pregrasp",
        "grasp",
        "lift",
    ]
    assert request.metadata["score"] == 0.8

    loaded = MotionPlanRequest.from_dict(request.to_dict())

    assert loaded.planning_group == "piper_arm"
    assert loaded.waypoints[1].pose.to_list() == [0.1, 0.2, 0.2, 0.0, 0.0, 0.0]

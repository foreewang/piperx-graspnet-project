# ROS2 / MoveIt Workspace Stub

This workspace is reserved for ROS2 simulation and MoveIt path planning work.
The core Python project does not depend on ROS2, so Windows hardware bring-up
and unit tests can continue to run without ROS2 installed.

Suggested flow:

1. Generate a MoveIt request from the core project:

   ```powershell
   python scripts/07_prepare_moveit_plan.py --base-pose 0.30 0.00 0.20 3.14 0.00 0.00
   ```

2. Copy or mount `data/moveit_requests/latest_plan_request.json` into the ROS2
   machine or WSL environment.

3. In ROS2, implement `piperx_moveit_bridge/plan_from_json.py` to:

   - load the JSON request;
   - convert `[x, y, z, roll, pitch, yaw]` waypoints into ROS poses;
   - call MoveIt for the configured planning group;
   - write a JSON result to `data/moveit_results/latest_plan_result.json`.

4. After planning is reliable in RViz or simulation, connect the result back to
   the real PiperX execution path.

Expected later ROS2 assets:

- PiperX URDF/Xacro;
- SRDF and MoveIt config package;
- fake hardware or ros2_control simulation config;
- planning scene objects for table, camera, gripper, and workspace boundaries.

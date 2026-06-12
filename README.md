# PiperX GraspNet Project

Development scaffold for using GraspNet grasp poses to control a PiperX arm.

Current verified hardware path:

- Windows
- Conda environment: `piperx`
- CAN backend: `agx_cando`
- CAN channel: `0`
- Bitrate: `1000000`

## Install

From this directory:

```powershell
conda activate piperx
python -m pip install -e .
```

## Suggested Bring-Up Order

1. Check CAN backend:
   ```powershell
   python scripts/00_check_can.py
   ```

2. Read robot state:
   ```powershell
   python scripts/01_check_robot_state.py
   ```

3. Review a dry-run motion command:
   ```powershell
   python scripts/02_test_motion.py
   ```

4. Execute the small motion only after confirming workspace clearance:
   ```powershell
   python scripts/02_test_motion.py --execute
   ```

5. Connect camera and collect RGB-D data.

6. Calibrate `T_base_camera` in `configs/calibration.yaml`.

7. Run GraspNet once and inspect the selected grasp.

8. Execute a dry-run pick plan:
   ```powershell
   python scripts/06_pick_once.py --grasp-output data/grasp_outputs/latest.npz
   ```

9. Execute only after the pose chain is validated:
   ```powershell
   python scripts/06_pick_once.py --grasp-output data/grasp_outputs/latest.npz --execute
   ```

## Coordinate Conventions

- `T_base_camera`: 4x4 transform from camera frame into robot base frame.
- `T_camera_grasp`: 4x4 grasp pose output by GraspNet in camera frame.
- `T_base_grasp = T_base_camera @ T_camera_grasp`.
- Robot pose commands are `[x, y, z, roll, pitch, yaw]` in meters and radians.

## Safety

The scaffold intentionally separates planning from execution. Scripts that can move
hardware require `--execute`. Keep small motion limits until hand-eye calibration,
TCP offset, workspace boundaries, and gripper behavior are verified.

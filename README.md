# PiperX GraspNet 抓取项目

本项目用于把 GraspNet 输出的抓取位姿转换为 PiperX 机械臂可执行的抓取计划，并逐步接入 ROS2 / MoveIt 仿真规划和真实机械臂控制。

当前项目分为两条链路：

```text
Windows + conda piperx
  -> AGX CANDO
  -> pyAgxArm
  -> 真实 PiperX 机械臂通信、状态读取、使能和运动控制

Ubuntu / WSL2 + ROS2 / MoveIt
  -> PiperX 模型显示
  -> 路径规划
  -> RViz / 仿真验证
```

当前重点不是直接完成完整视觉抓取，而是先打通：

```text
抓取位姿 / GraspNet 输出
  -> 抓取计划
  -> MoveIt 规划请求 JSON
  -> ROS2 / MoveIt 仿真验证
  -> Windows 真机低速执行
```

## 当前已验证环境

真实机械臂控制链路已经在 Windows 下验证：

- 系统：Windows
- Conda 环境：`piperx`
- CAN 后端：`agx_cando`
- CAN 通道：`0`
- CAN 波特率：`1000000`
- 控制库：`python-can`、`python-can-agx-cando`、`pyAgxArm`

已完成验证：

- CAN 设备识别
- CAN 帧读取
- PiperX 机械臂连接
- 机械臂状态读取
- 关节角读取
- 末端位姿读取
- 六轴使能
- 小幅运动测试

## 安装

在 Windows 的项目目录下执行：

```powershell
cd C:\piperx\piperx_graspnet_project
conda activate piperx
python -m pip install -e .
```

运行测试：

```powershell
pytest -q
```

当前测试应通过：

```text
4 passed
```

## 目录说明

```text
configs/
  robot.yaml             # PiperX 通信、运动和夹爪配置
  camera.yaml            # RGB-D 相机配置占位
  calibration.yaml       # 手眼标定和 TCP 标定矩阵，占位为单位矩阵
  graspnet.yaml          # GraspNet 输出和候选筛选配置
  ros2_moveit.yaml       # ROS2 / MoveIt 规划请求配置

scripts/
  00_check_can.py        # 检查 CAN 后端和 CAN 帧
  01_check_robot_state.py# 读取机械臂状态
  02_test_motion.py      # 小幅末端运动测试，默认 dry-run
  03_test_gripper.py     # 夹爪测试，默认 dry-run
  04_capture_rgbd.py     # RGB-D 相机采集入口，当前未实现
  05_run_graspnet_once.py# 读取 GraspNet .npz 并选最佳候选
  06_pick_once.py        # 根据 GraspNet 输出生成并执行抓取计划
  07_prepare_moveit_plan.py # 生成 MoveIt 规划请求 JSON

src/piperx_grasp/
  robot/                 # PiperX 和夹爪控制封装
  grasp/                 # GraspNet 输出解析和候选筛选
  calibration/           # 坐标变换工具
  planning/              # MoveIt 请求/结果 JSON 契约
  pipeline/              # 感知、规划、执行流程
  camera/                # 相机和点云接口

ros2_ws/
  src/piperx_moveit_bridge/ # ROS2 / MoveIt 桥接包骨架
```

## Windows 真机控制流程

建议按下面顺序检查硬件。

1. 检查 CAN 后端：

```powershell
python scripts/00_check_can.py
```

2. 读取机械臂状态：

```powershell
python scripts/01_check_robot_state.py
```

3. 查看小幅运动 dry-run：

```powershell
python scripts/02_test_motion.py
```

4. 确认机械臂周围安全后，执行小幅运动：

```powershell
python scripts/02_test_motion.py --execute --dz 0.005 --speed 5
```

5. 查看夹爪状态：

```powershell
python scripts/03_test_gripper.py
```

6. 确认夹爪安全后，执行夹爪开合：

```powershell
python scripts/03_test_gripper.py --execute
```

所有会移动真实硬件的脚本都需要显式传入：

```text
--execute
```

没有 `--execute` 时只做 dry-run 或状态检查。

## 没有相机时如何测试

当前没有 RGB-D 相机时，不需要先做 GraspNet 输入点云。建议先测试 GraspNet 输出之后的链路：

```text
手动抓取位姿
  -> 抓取计划
  -> MoveIt 请求 JSON
  -> ROS2 / MoveIt 仿真规划
```

生成一个手动抓取位姿的 MoveIt 请求：

```powershell
python scripts/07_prepare_moveit_plan.py --base-pose 0.30 0.00 0.20 3.14 0.00 0.00
```

参数含义：

```text
--base-pose X Y Z ROLL PITCH YAW
```

单位：

```text
X/Y/Z：米
ROLL/PITCH/YAW：弧度
```

生成结果：

```text
data/moveit_requests/latest_plan_request.json
```

该文件会包含：

```text
pregrasp -> grasp -> lift
```

如果要指定夹爪宽度：

```powershell
python scripts/07_prepare_moveit_plan.py --base-pose 0.30 0.00 0.20 3.14 0.00 0.00 --gripper-width 0.04
```

如果要指定输出文件：

```powershell
python scripts/07_prepare_moveit_plan.py --base-pose 0.30 0.00 0.20 3.14 0.00 0.00 --out data/moveit_requests/test_001.json
```

## 使用 GraspNet 输出测试

后续有 GraspNet 输出后，可以使用 `.npz` 文件作为输入。

`.npz` 需要包含：

```text
poses   # [N, 4, 4]，GraspNet 输出的抓取位姿矩阵
scores  # [N]，每个候选抓取评分
widths  # [N]，推荐夹爪宽度，单位米
```

先检查最佳候选：

```powershell
python scripts/05_run_graspnet_once.py --input data/grasp_outputs/latest.npz
```

生成 MoveIt 规划请求：

```powershell
python scripts/07_prepare_moveit_plan.py --grasp-output data/grasp_outputs/latest.npz
```

生成抓取计划 dry-run：

```powershell
python scripts/06_pick_once.py --grasp-output data/grasp_outputs/latest.npz
```

真实执行必须等相机标定、TCP、工作空间和仿真验证都确认后再运行：

```powershell
python scripts/06_pick_once.py --grasp-output data/grasp_outputs/latest.npz --execute
```

## ROS2 / MoveIt 仿真规划

ROS2 / MoveIt 建议在 Ubuntu 或 WSL2 中运行，不建议在 Windows PowerShell 里编译。

Windows 项目负责生成：

```text
data/moveit_requests/latest_plan_request.json
```

Ubuntu ROS2 / MoveIt 侧负责读取该 JSON，转换为 MoveIt 目标位姿，执行路径规划，并输出：

```text
data/moveit_results/latest_plan_result.json
```

当前 ROS2 桥接骨架位于：

```text
ros2_ws/src/piperx_moveit_bridge
```

其中：

```text
plan_from_json.py
```

目前只完成了 JSON 读取和结果输出骨架，尚未真正调用 MoveIt。后续需要在 Ubuntu / ROS2 环境中补充：

```text
读取 waypoints
  -> 转成 geometry_msgs/Pose
  -> 调用 MoveIt 规划
  -> 在 RViz 中显示路径
  -> 输出规划结果 JSON
```

PiperX 的 URDF、mesh 和 MoveIt 配置不建议从零编写，优先使用 AgileX 官方仓库：

- `Agx_arm_sim`
- `Agx_arm_urdf`
- `Agx_arm_ros`

本项目中的 `piperx_moveit_bridge` 只作为桥接层，不重复维护官方机械臂模型。

## 坐标约定

当前项目使用如下约定：

```text
T_base_camera：相机坐标系到机械臂基坐标系的 4x4 变换矩阵
T_camera_grasp：GraspNet 输出的相机坐标系下抓取位姿
T_base_grasp = T_base_camera @ T_camera_grasp
```

机械臂控制位姿格式为：

```text
[x, y, z, roll, pitch, yaw]
```

单位：

```text
x/y/z：米
roll/pitch/yaw：弧度
```

注意：

```text
configs/calibration.yaml
```

中的 `T_base_camera` 和 `T_flange_tcp` 当前仍是单位矩阵占位，真实抓取前必须完成手眼标定和 TCP 标定。

## 仿真和真机之间的桥梁

仿真和真机之间通过 JSON 数据契约连接：

```text
data/moveit_requests/latest_plan_request.json
data/moveit_results/latest_plan_result.json
```

核心代码：

```text
src/piperx_grasp/planning/moveit_contract.py
```

桥接关系：

```text
Windows 项目
  -> 生成 MoveIt 请求 JSON

Ubuntu ROS2 / MoveIt
  -> 读取 JSON
  -> 做路径规划和仿真验证

Windows 项目
  -> 读取验证后的结果
  -> 通过 pyAgxArm 低速控制真实 PiperX
```

当前已经完成：

```text
Windows -> 生成 MoveIt 请求 JSON
```

待完成：

```text
ROS2 -> 读取 JSON 并真正调用 MoveIt
ROS2 -> 输出规划结果 JSON
Windows -> 读取规划结果并安全执行真机动作
```

## 安全说明

真实机械臂执行前必须满足：

```text
1. CAN 通信正常
2. 机械臂状态正常
3. 六轴已使能
4. 目标位姿在安全工作空间内
5. 手眼标定已完成
6. TCP 偏移已确认
7. GraspNet 位姿坐标系方向已确认
8. MoveIt / RViz 仿真路径已验证
9. 执行速度设置为低速
10. 人工确认周围环境安全
```

不要直接把未经验证的 GraspNet 输出发送给真实机械臂执行。

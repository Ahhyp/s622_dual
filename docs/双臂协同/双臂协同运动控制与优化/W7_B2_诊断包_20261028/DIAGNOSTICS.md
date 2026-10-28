# W7 B-2 诊断证据（2026-10-28 17:12–17:15，实测）

本文件只记录**实测**输出，不含推断。环境：WSL2，`ROS_DOMAIN_ID=91`，headless Gazebo Fortress 6.16，
`ros2 launch gz_launch c43_s4_full_flow.launch.py headless:=true`，日志 `/tmp/w7fix/run3/launch.log`。

## 1. 控制器与硬件接口（运行中抓取）

```
$ ros2 control list_controllers
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster   active
left_hand_controller     joint_trajectory_controller/JointTrajectoryController  active
right_hand_controller    joint_trajectory_controller/JointTrajectoryController  active
dual_arm_qp_controller   dual_arm_qp_controller/DualArmQpController     active

$ ros2 control list_hardware_interfaces | head
command interfaces
    left_finger1_joint/position [available] [claimed]
    ...
    left_j1/position [available] [claimed]
    left_j2/position [available] [claimed]
    left_j3/position [available] [claimed]
    left_j4/position [available] [claimed]
    left_j5/position [available] [claimed]
    left_j6/position [available] [claimed]
```

⇒ QP 控制器 **active**，12 个臂关节 `position` 命令接口 **claimed**，没有第二个控制器抢同一接口。

## 2. 一次被接受的 start_transport（4 相位 × 0.5 s，dx=0/0/0.06/0，dz=0.02/0.05/0/-0.05）

```
start_accepted True | queued (applied on the next control cycle)
status_final COMPLETE  t_traj 2.0
qcmd_maxabs_delta 2.0569012339102706      # status.q_cmd 在 2 s 内变化 2.057 rad
js_maxabs_delta 0.0                       # 同时 /joint_states 12 个关节位移全为 0.0
```

`/joint_states` 逐关节（1e-6 精度）：

```
js_delta {"left_j1": 0.0, "left_j2": 0.0, "left_j3": 0.0, "left_j4": 0.0, "left_j5": 0.0,
          "left_j6": 0.0, "right_j1": 0.0, ..., "right_j6": 0.0}
```

⇒ **B-2 复现**：控制器在算、在发 `q_cmd`（2.057 rad 的指令变化），仿真关节一动不动。

## 3. 机械臂确实"被某个东西按住"

`/joint_states` 在整段运行中恒为生成位姿（`left_j1=0.500036, left_j2=-1.04974, left_j3=1.050621,
left_j4=-1.050568, left_j5=-0.800057, left_j6=-0.000193`，右臂同），没有重力下垂（初始位姿肘部弯曲，
若无位置保持会明显下沉）⇒ 存在一个**恒定的位置指令**在把关节压在生成位姿上，但它不是 QP 控制器的
`q_cmd`（后者已变 2.057 rad）。这是 B-2 的关键：不是"没有命令通道"，而是"命令通道里的值不来自 QP 控制器"。

## 4. 已排除的假设

| 假设 | 证据 | 结论 |
|---|---|---|
| QP 控制器未激活 | `list_controllers` = active | 排除 |
| 接口被别的控制器抢走 | `list_hardware_interfaces` 只有 QP 声称 | 排除 |
| 仿真时钟停了 | `t_traj` 到 2.0 s、controller 周期统计 10.000 ms / overruns 0 | 排除 |
| 控制器没算轨迹 | `status.q_cmd` 变化 2.057 rad、`state=COMPLETE` | 排除 |
| B-1 姿态门禁挡住（早前阻断） | 本次 `start_accepted True` | 已由 home_q 覆盖修复 |

## 5. 未排除、需要下一步定因的方向

1. `set_value()` 是否真的写进 gz_ros2_control 的 position 接口（需要在 `writeCommand()` 之后读回接口值，
   或在 `update()` 里打印 1 帧的 `command_interfaces_[0].get_value()`）。
2. gz_ros2_control 的 `<param name="position_proportional_gain">20.0</param>` 路径是否把"位置命令"当
   "速度目标"实现、而关节 SDF 的 damping/effort 使其无法跟随（需 `ign model -m s622_dual_arm` 看
   关节实际应用的速度/力）。
3. 是否存在两个 `controller_manager`（旧 launch 残留）分别持有接口（本次 `ros2 node list` 只有一个
   `/controller_manager`）。

## 6. 复现命令

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 HOME=/tmp/w7fix/run3
ros2 launch gz_launch c43_s4_full_flow.launch.py headless:=true
python3 /tmp/w7fix/drive.py           # 见本包 files/drive_probe.py
```

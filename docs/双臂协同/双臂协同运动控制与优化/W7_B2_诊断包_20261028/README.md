# W7 B-2 诊断包（评审要求：完整文件，不是 diff）

## 本包内容
```
files/
  dual_arm_qp_controller.cpp              # P0 嫌疑：writeCommand() / update() 的完整实现
  dual_arm_qp_controller.hpp              # 控制器接口、成员、TimingScope
  s622_gazebo_ros2_control.xacro          # gz_ros2_control 硬件插件与 position_proportional_gain
  dual_arm_qp_gazebo_controllers.yaml     # 控制器 joints 顺序与参数
  executor_node.cpp                       # W7 接线（driveWeld / onPoses / onWeldState）
  state_machine.cpp                       # W7 tickWeld / phaseDone 闸门
  drive_probe.py                          # 本次取证脚本（start_transport + joint_states/q_cmd 对照）
DIAGNOSTICS.md                            # 实测输出（原样粘贴，非推断）
```

## 结论（详 DIAGNOSTICS.md）
- 控制器 `active`、12 个 `position` 接口 `claimed`、仿真在跑、`start_transport` 被接受、
  `status.q_cmd` 在 2 s 内变化 **2.057 rad**，而 `/joint_states` 12 个关节位移 **全为 0.0**。
- 关节保持生成位姿、无重力下垂 ⇒ 有一个恒定位置指令压着关节，但它不来自 QP 控制器。
- 所以 B-2 不是"通道没接上"，而是"通道里的值不来自 QP 控制器"；
  下一步按 DIAGNOSTICS.md §5 的三条方向定因（读回接口值 / 查 gain 语义 / 查重复 controller_manager）。

## 同轮的其它修复（已编译、单测全绿）
| 编号 | 状态 | 验证 |
|---|---|---|
| P0-1 一次性动作被状态机吞掉 | 已修：`tick()` 只决策、`consumeSend()` 才发布（armed/sent 双标志） | `test_attach` 18/18、`test_full_flow` 9/9 |
| P0-2 pose 桥未 remap | 已修：`parameter_bridge` 加 `remappings=[(/world/<w>/pose/info, /c43/weld_pose)]` | launch 日志 `Creating GZ->ROS Bridge: [/world/c43_s4_world/pose/info ...]`；待真实 attach 复验 |
| P1-1 启动 detach 单发 | 已修：`startup_retry_s=0.25`，按 guard 截止时间重发（不再受 3 次上限） | 实跑日志：`detach requested` → 7 ms 后 `startup weld released and confirmed` |
| P1-2 缺角度判据 | 已修：0.5°（§14.3 C3），`quatAngle()` 比较相对姿态；新增 3 个旋转用例 | `test_attach` 含 `SnapExactlyAtTheRotationCriterionIsAccepted` |
| P1-3 增益映射 | 已核实：`arm_position_gain` → `<param name="position_proportional_gain">`（xacro:15/27） | 见 `files/s622_gazebo_ros2_control.xacro` |
| B-1 生成位姿≠home_q | 已修：controller spawner 传 `--param-file c43_s4_qp_home_override.yaml`（仅 Gazebo profile） | `start_accepted True`（此前 `start pose deviation 2.794844 rad at joint 6`） |

## 复现
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 HOME=/tmp/w7fix/run3
ros2 launch gz_launch c43_s4_full_flow.launch.py headless:=true
python3 files/drive_probe.py
```

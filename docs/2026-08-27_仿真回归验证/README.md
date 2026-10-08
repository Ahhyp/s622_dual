# 仿真回归验证计划（2026-08-27）

> 目的：真机部署引入了 2 处**共享包/共享配置**改动，需确认对仿真（单臂 + 双臂）无副作用。
> 背景：stamp 清空改动后，mock/仿真链路**尚未验证过**（上次 mock 验证在 stamp 修复之前）。

## 1. 共享改动清单（真机修复引入，仿真也在用）

| # | 文件 | 改动 | 影响面 |
|---|------|------|--------|
| 1 | `src/trajectory_retime_server/src/retime_server.cpp:470` | TOTG 输出 `out.header.stamp = rclcpp::Time(0,0)`（清空规划时刻） | 所有走 `/retime_trajectory` 的链路（单臂/双臂仿真 + 真机） |
| 2 | `src/s622_moveit_config/config/moveit_controllers.yaml` | 加 `trajectory_execution` 段（allowed_execution_duration_scaling: 1.25 / allowed_goal_duration_margin: 3.0） | 仿真 move_group（MoveItConfigsBuilder 默认加载同文件） |

> 其余真机改动（real_controllers.yaml / real_joint_limits.yaml / s622_real_*.xacro / s622_real_arm.launch.py / demo 工具 / BT real 版）均为**真机独立文件**，仿真不引用，不在回归范围。

## 2. 理论分析（为什么预期无副作用）

- **stamp 清空**：JTC（joint_trajectory_controller）源码对 `header.stamp == 0` 特殊处理 —— `time_offset = 0`，**从 JTC 收到轨迹的时刻起算**。仿真/真机行为一致；且 `time_from_start` 本就是相对时间（从 0 起），清空 stamp 只是消除"规划时刻"这个错误起点。仿真中 move_group→retime→JTC 同机低延迟，规划时刻≈收到时刻，差异更小。
- **trajectory_execution 段**：只放宽"执行超时"判定（1.25× 时长 + 3.0s 余量），仿真执行快于规划时长，不会触发 TIMED_OUT；不改变轨迹速度/加速度。

## 3. 验证矩阵

| 场景 | 启动方式 | 判据 | 执行者 |
|------|----------|------|--------|
| A. mock 快速回归 | `ros2 launch s622_arm_actions s622_mock_motion_demo.launch.py execute_motion:=true move_distance:=0.02` + service call start | demo SUCCESS、JTC 正常执行短轨迹、**无 ABORTED**、无 `non-zero start time` 报错 | agent（无 GUI） |
| B. 单臂仿真 | `ros2 launch gz_launch s622_gazebo.launch.py` + 短轨迹运动 | retime 服务正常、move_group 执行不再 ABORTED、Gazebo 无 NaN 报错、模型正常运动 | 用户（需 Gazebo GUI） |
| C. 双臂仿真 | `ros2 launch gz_launch s622_dual_arm.launch.py` + BT `pick_place_dual.xml` | BT 全流程 SUCCESS（8-25 回归修复后"重启仿真全流程"待办项） | 用户 |

## 4. 注意事项

- retime_server 二进制已确认含 stamp 改动（build/trajectory_retime_server/retime_server 2026-08-27 15:46 编译）
- 场景 A 用 `real_controllers.yaml`（position 接口 + update_rate 500），与真机同路径，验证 stamp=0 → JTC 立即执行链路
- 场景 B/C 用户启动仿真，agent 辅助看日志诊断

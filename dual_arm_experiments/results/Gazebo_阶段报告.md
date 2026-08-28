# ServoJ 动态跟踪与同步性能测试 — Gazebo 阶段报告（2026-08-27）

> 计划：`docs/2026-08-25_ServoJ动态跟踪与同步性能测试/README.md`
> 状态：✅ Phase A（Gazebo 链路验证）全部完成
> 工具：`dual_arm_experiments/`（scripts / analysis / config / results）

---

## 1. 工具链路（已验证）

```
sine_tracking_test.py（正弦轨迹生成 + 统一 T0 发送 + /joint_states 记录 + CSV）
        ↓ FollowJointTrajectory
JointTrajectoryController（全关节覆盖，allow_partial_joints_goal=false）
        ↓
gz_ros2_control → Gazebo joint
        ↓
analyze_single_arm.py（RMSE/MAE/max、幅值比、相位滞后/等效延迟、xcorr、jitter）
```

**单元验证**：
- 人工 7ms 延迟注入 → cross-correlation 恢复误差 ≤0.014ms（0.5/1/2/5Hz × 100/200/500Hz）
- 合成双臂 7ms → dual-files 分析恢复 +6.99ms

**关键设计（GPT review 后修订）**：
1. **统一未来 T0**：所有 controller 的 `trajectory.header.stamp = now + 0.5s`，JTC 到 T0 同时起跑——消除 action 传输/接收时差对双臂相对延迟的污染（实测：无统一 stamp 时测出 6.6ms 假延迟，统一后 0ms）
2. **cmd 时间基准 = t_start（header.stamp）**，非发送时刻
3. **result SUCCESS 检查**：`error_code == FollowJointTrajectory.Result.SUCCESSFUL` 才有效，否则 CSV 标 `_INVALID`
4. **trajectory_sample_rate_hz**：轨迹 waypoint 密度（=CM update_rate），非"发送频率"（整条轨迹一次交给 JTC 插值）
5. 超时 cancel goal（Humble rclpy `_cancel_goal_async`）；自动建目录；整数 ns Duration

**注意**：本阶段测的是"trajectory controller 正弦跟踪链"（Gazebo），**不是真机 ServoJ 调用链**。真机 ServoJ 耗时/周期必须在 `fairino_hardware_interface::write()` 内记录（复用同一轨迹发生器 + 分析脚本）。

---

## 2. 单臂正弦跟踪（Gazebo，j1，A=0.05rad）

单臂仿真（`s622_gazebo.launch.py`，CM update_rate=200Hz，velocity 命令接口）。

| 频率 | RMSE | MAE | max | 幅值比 G | 等效延迟 | 数据 |
|---|---|---|---|---|---|---|
| 0.5 Hz | 0.53 mrad | 0.48 | 0.75 | 0.999 | 4.76 ms | ✅ 完整 |
| 1 Hz | 42.3 mrad | 38.1 | 59.8 | 0.997 | 204 ms（不稳定，复测 104 ms） | ⚠️ 离群 |
| 2 Hz | 12.9 mrad | 11.6 | 18.3 | 0.997 | 29.3 ms | ✅ |
| 5 Hz | 4.63 mrad | 4.17 | 6.78 | 0.997 | 4.17 ms | ✅ |

- 幅值衰减几乎为 0（G≈0.997，Gazebo 物理理想）
- **1Hz 延迟异常（100-200ms，不稳定）**：xcorr 与 sin 拟合一致（数据内部自洽），但 0.5/2/5Hz 均正常（4-29ms）→ 疑似 Gazebo 控制环/负载在 1Hz 的偶发特性，**真机阶段必须复测**（Gazebo 动态不作数）
- 5Hz 峰值速度 1.57 rad/s、峰值加速度 49.3 rad/s²（真机需按速度/加速度限制评估）

---

## 3. 双臂同步（Gazebo，left_j1 vs right_j1 同轨迹，A=0.05rad，0.5Hz）

双臂仿真（`s622_dual_arm.launch.py`，CM update_rate=100Hz）。

| 发送方式 | 相对延迟（右滞后左） | 分窗 3σ |
|---|---|---|
| 两个独立实例（不同步） | ~150 ms | — |
| 同一循环发送（无统一 T0） | +6.58 ms | 6.61 ms |
| **统一 T0（header.stamp）** | **0.00 ms** | **0.00 ms** |

**结论**：统一 T0 后 Gazebo 双臂相对延迟 = 0（同一 CM、同构控制器，物理理想）。真实双臂同步性能须真机测量。

---

## 4. Gazebo 退出判据（计划第 23 节）对照

- [x] 正弦指令生成频率正确（waypoint 密度=CM update_rate）
- [x] CSV 无丢数据（actual 完整覆盖 duration）
- [x] command/actual 正确对齐（统一 T0 时间基）
- [x] RMSE / 幅值比 / 相位 / 等效延迟计算正确（0.5Hz 完美数据佐证）
- [x] cross-correlation 恢复人工注入 7ms（误差 ≤0.014ms）
- [x] jitter 计算正确（分窗 mean/σ/3σ）
- [x] 双臂 joint mapping 正确（left_j1/right_j1 各自 controller）
- [x] 测试超时自动退出 + cancel goal
- [ ] ServoJ 命令频率实际测量（需真机/插件内记录）

---

## 5. 结果文件

```
dual_arm_experiments/results/gazebo/
├── single_j1_0.5hz.csv / _plot.png
├── single_j1_1hz.csv / single_j1_1hz_v3.csv（复测）
├── single_j1_2hz.csv
├── single_j1_5hz.csv
├── dual_j1_0.5hz__left_arm_controller.csv / __right_arm_controller.csv（同循环版）
└── dual_j1_0.5hz_T0__left_arm_controller.csv / __right_arm_controller.csv（统一 T0 版）
```

## 6. 下一步

- [ ] 真机单臂 R1（±1°/0.5Hz，5-10s）→ R2（±2~3°/0.5Hz）→ R3（1/2Hz，5Hz 商榷）
- [ ] 真机 ServoJ 耗时/周期记录（`fairino_hardware_interface::write()` 内加统计）
- [ ] 真机双臂同步（明天）

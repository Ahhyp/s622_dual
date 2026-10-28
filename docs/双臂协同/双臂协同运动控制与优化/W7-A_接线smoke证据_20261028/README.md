# W7-A 接线 smoke 证据（2026-10-28）

**证据等级：** 一次性 headless 运行日志摘录。不是 W7 全流程证据，不是 S4-T13 PASS，
不是实时性结论（WSL2，非原生 RT）。

## 跑了什么

```bash
HOME=/tmp/w7a/home ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1 \
  ros2 launch dual_arm_qp_controller s622_dual_arm_qp.launch.py gui:=false
```

约 43 s 后主动停止（`timeout 170` 未到期）。原始日志 233 行留在 `/tmp/w7a/launch.log`
（未入库）；本目录 `smoke_log_filtered.txt` 是按 `Loading controller|Loaded |
Configured and activated|home_q must|protection is NOT|activated: relative|period ms:`
过滤后的 24 行摘录，sha256 `1c2affc6cbd57ab19cb538a6cf22b2e32bb2083f6d6e6cdb906450f9867b9f71`。

## 观察到的（原文见 smoke_log_filtered.txt）

| 项 | 实测 |
|---|---|
| world | `install/share/gz_launch/worlds/dual_arm_world.sdf` |
| 控制器 | joint_state_broadcaster / left_arm / right_arm / left_hand / right_hand / dual_arm_qp_controller 全部 `Configured and activated` |
| QP 控制器 | `activated: relative target captured, q0=[0.500 -1.050 1.050 -1.050 -0.800 -0.000 \| …]` |
| 控制周期 | `arg=8.000 measured(mean=8.000 P95=8.000 P99=8.000 max=8.000)` |
| overrun | `0/2500`（最后一个采样点） |
| solver | `P99=0.206 ms` |

## 发现（如实记录，未修复）

- **W7-A-F1（P1）**：`[ERROR] [dual_arm_qp_controller]: home_q must have 12 values (got 140); using the S2 q_grasp default`。
  Gazebo 档的 `home_q` 从未按配置生效，只是回落到冻结 `q_grasp12`（起点门限仍然有效，
  但“配置生效”这件事是假的）。参数来源未定位，**超出 W7 接线范围，未修**。
- **W7-A-F2（P2）**：该路径不桥接 `attach/detach/weld_state`，也不使用 W6 的
  `c43_s4_dual_arm_gazebo.urdf.xacro`。所以这份日志只证明“控制器链路能跑”，
  **不证明** W7 全流程。
- **W7-A-F3（P2）**：该路径没有 executor 心跳，`protection` 报
  `missing(e7_arm_gap,e8_executor_heartbeat)`，与 W2 既有记录一致；W7 全流程必须先起
  executor，否则 E8 会按设计冻结运动。

## 结论

`W7_A_PARTIAL`：接线的一半（Gazebo ↔ controller_manager ↔ QP 控制器）成立且 8 ms 周期
0 overrun；另一半（W6 焊接插件 + triple-topic 桥 + executor 在同一 launch）尚未建立，
W7-B/C 未执行。

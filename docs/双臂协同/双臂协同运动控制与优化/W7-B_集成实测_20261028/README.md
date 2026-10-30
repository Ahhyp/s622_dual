# W7-B 集成实测（2026-10-28 20:35–20:50，真 Gazebo）

一次真实尝试：让**执行器**驱动一次 GRASP→LIFT 流程，而不是像 B-2 复测那样直接调控制器的
`start_transport`。结论：**执行器与控制器在真实 DDS 上已经能协同，但流程在第一个周期就被
执行器自己的保护规则挡住**，GRASP 仍未发生。

## 1. 跑起来的顺序

```
ros2 launch gz_launch c43_s4_full_flow.launch.py headless:=true
# 控制器的 5 相位轨迹：approach 3 s / grasp 2 s / lift 3 s / transport 3 s / lower 3 s
# 起点为 Gazebo 的 initial_positions（home_q 已由该 Gazebo profile 覆盖为同一姿态）
ros2 service call /dual_arm_qp_controller/start_transport ...
ros2 service call /c43_s4_transport_executor/start std_srvs/srv/Trigger
```

实测（`launch_excerpt.txt`、`resume_trail.json`）：

| 环节 | 结果 |
|---|---|
| 执行器启动前解除启动焊接 | `weld: startup weld released and confirmed`（重试节奏修复后一次成功） |
| 执行器学习控制器相位表 | `phase table learned from the controller (5 phases, total 14.000 s)` |
| 执行器 `~/start` | `True | accepted` |
| 第一个活动周期 | `state APPROACH -> HOLD (code 'joint_limit_saturation')` |
| 控制器响应 HOLD | `hold requested: seq=1` → `controller HOLD confirmed: frozen_by_executor_request (seq=1, attempts=1)` |

⇒ **C7 的 HOLD 往返链在真实 Gazebo + 真实 DDS 上成立**：执行器请求、控制器冻结、执行器收到确认。

## 2. 挡路的规则

`src/dual_arm_transport_executor/src/state_machine.cpp:747`

```cpp
if (st_in->active_limit_count > 0) {
  enterProtection(Protection::Hold, "joint_limit_saturation", ...);
}
```

而控制器 `src/dual_arm_qp_controller/src/dual_arm_qp_controller.cpp:1058-1063` 的计数器实现是：

```cpp
const double vlim = v_max_[i] * dt_;
if (vlim > 0.0 && std::abs(last_dq_[i]) >= vlim - 1e-12) ++active_limits;
```

即它数的是**速度已经顶到每周期上限**的关节数（`v_max=2.5 rad/s`，`dt=8 ms` ⇒ 0.02 rad/周期），
不是"位置撞限位"。实测这在整个 approach 段是常态：

```
lim=12 → 10 → 11 → 9 → 8 → 7 → 6 …（14 s 轨迹，见 resume_trail.json / lim_series.json）
```

只要有一个关节顶到速度上限，执行器就立刻 HOLD，于是**任何**会用到速度上限的搬运轨迹都跑不下去。

## 3. 手工复位后的观察（说明审计是对的）

控制器单独 `resume_transport` 后（执行器侧 resume 被正确拒绝：`controller_failure_active:hold_by_executor`），
控制器把 14 s 轨迹跑完并报 `COMPLETE`。此时执行器看到"锁存 HOLD 期间 `q_cmd`/`t_traj` 仍在变"，
升级为 `FAULT (hold_invariant_violated)` —— 这是**正确**的：是我以操作员身份绕过执行器恢复的控制器。

同一次运行控制器报 `e_obj = 2.907e-01 m`。这与"没有抓取"一致：样品仍在支撑上，机械臂已经走开，
所以物体跟踪误差就是臂走过的距离。**没有抓取时 `e_obj` 没有物理意义**，不能拿它评价跟踪性能。

## 4. 结论与待决

* 已证实：启动焊接解除、相位表学习、执行器启动、HOLD 请求→冻结→确认，全部在真实链路成立。
* 被阻断：执行器的 `joint_limit_saturation` 规则与控制器 `active_limit_count` 的语义不匹配
  （速度饱和 vs 位置限位），且没有"连续 N 周期"或"位置裕度"的余量。
* 未发生：GRASP 的 attach、LIFT 的搬运、RELEASE 的 detach。

修法有两条，都动安全语义，需要裁决：

1. 执行器改用位置裕度（`min_limit_margin_rad < 阈值`）判定，或要求连续 N 周期才升级；
2. 控制器把计数器拆成"速度饱和"与"位置限位"两个字段，执行器只对后者反应（改 msg，属公共契约）。

## 5. 改完规则后的复跑（2026-10-28 21:00–21:06）

按裁决把 `joint_limit_saturation` 改成「位置裕度 + 连续 N 周期」后重跑（`limit_saturation_cycles: 25`
≈ 200 ms，`limit_margin_rad: 0.05`）：

* 规则本身按预期工作：前 25 个持续饱和样本被容忍，**第 26 个**才 HOLD（旧版第 1 个就 HOLD）。
* 但底层现象没变：`lim=10…12`，`margin=1.63…1.78 rad`（离位置限位很远）⇒ 保护按设计触发，
  **真正的原因是 QP 从第一拍起就把 10–12 个关节压到速度上限**，与位置限位无关。
* 显式传入 `start_world_t_object = [I | (0,0,0.275)]` 后结果相同 ⇒ 不是「物体锚定在世界原点」造成的追逐。
* 本段还验证了执行器侧数据通路：`~/start` 返回 accepted、启动焊接解除、HOLD 请求→冻结→确认均在真实链路成立。

⇒ 结论修正：执行器规则是**次要缺陷**（已按裁决修正）；**主要缺陷在控制器侧**——`dq` 长期顶到
`v_max*dt`。下一步应查 QP：posture/relaxed 项权重、`v_max` 与 `dt` 的取值、以及为什么 2 cm 的物体
位移需要关节以 2.5 rad/s 运行。证据：`driver_limit_rule_20261028.log`、`launch_limit_rule_20261028.txt`。

# C4.3 S4 — T1 定位 / T2 GRASP→LIFT 执行记录（2026-10-29）

> 结论先行：**T1 的根因不是 QP 权重，而是 Gazebo 初始位姿与冻结 grasp 相对位姿不一致**；
> 修复后同一条轨迹 `active_limit_count` 全程为 0。**T2 在修复 7 处集成缺陷后跑出一次
> 干净的 `APPROACH→GRASP→attached→LIFT→TRANSPORT→LOWER→RELEASE→detached→COMPLETE`**，
> 样品 z 从 0.2750 m 抬到 0.2964 m 后回落。这是 **Gazebo 焊接替代夹爪、SOFTWARE_ONLY 的
> 集成证据**，不是真机 PASS；S4-T16 的代码已接，但物理抬高实验仍未完成。

## 0. 环境自检（交接包 §5）结果

| 检查 | 结果 |
|---|---|
| `git log -1` | `3ac62a2`（本地原先在 `public` @ `f7934dd`，已切到 `master`，环境改动用 stash 保留） |
| `git status --porcelain` | **非 0（30 项）**：5 个迁移前已有的 Docker/GPU 环境改动 + 本次代码/配置/证据改动；**未 commit** |
| `ros-humble-osqp-vendor` | 容器重建后缺失，已 `apt` 补装；否则 `dual_arm_qp_core` 只会建 ADMM 后端 |
| 构建 | 9 包通过 |
| 用例 | `selfcheck_final.log` 为 T1/T2 阶段的 172；S4-T16 + FAULT 修复后最新 **188 tests / 0 failures** |
| `test_fail_readstate` 注入符号 | 0 |
| 冻结 S3 fixture / D2-B baseline sha256 | 与档案一致 |
| `LEDGER_OK` | 通过 |
| **D2-B 闸门** | **`INPUT_ERROR`**：`libdual_arm_qp_core.so` 重构建哈希 `d1be692…` ≠ 基线 `ff95750…`；契约要求 same-build，**没有改 provenance、没有伪造 PASS** |

> D2-B 是跨机器重构建的哈希不匹配（同一份源码、同 g++ 11.4，但构建环境不同：容器路径、
> 依赖布局、OSQP 头；具体成因未逐项归因），
> 数值回归未验证；本轮**没有**把 `LEDGER_OK` 或任何 Gate 说成 PASS。详见
> `selfcheck_final.log`。

## 1. T1：QP 为什么让 10–12 个关节持续满速

### 1.1 关键实验（真实 Gazebo，不是 mock）

用 `probe_t1.py` 在 `start_transport` 后抓控制器状态（`e_rel_pos_m / e_rel_rot_rad / dq /
active_limit_count`）：

| 阶段 | q0 | start 后 `e_rel` | `active_limit_count` | `max|dq|` | `e_obj_pos` |
|---|---|---|---|---|---|---|
| 修复前 | `[0.500 −1.050 1.051 …]` | **2.196 m / 2.927 rad** | 11 → 12 | 恒 0.00800 = `v_max·dt` | 0 → 0.55 m |
| 修复后 | `q_grasp12` | 0.0003 m / 0.0011 rad | **0** | ≤5.2e-4 | ~1e-4 |

### 1.2 根因链

1. launch 本来用 `ros_gz_sim create -J q_grasp12` 让机器人出生在抓取位姿；
2. 但 `gz_ros2_control` 会用 URDF 里 `<state_interface><param name="initial_value">` 覆盖关节初值，
   `-J` 无效 —— 实测 q0 是 `s622_moveit_config/config/initial_positions.yaml` 的
   `[0.5, −1.05, 1.05, −1.05, −0.8, 0]`；
3. W7-B1 为了让 `start_transport` 的起点门通过，把控制器 `home_q` 覆盖成了这个出生位姿；
4. 但 `TransportTask` 的相对任务目标 `left_T_right_` 是**冻结的 grasp 链**
   （`object_T_left_tcp / object_T_right_tcp`，即 `q_grasp12` 处的 FK）。离线 FK 复核：
   - 在 `q_grasp12`：`e_rel = 0.000000 m / 0.000000 rad`
   - 在 `[0.5, −1.05, …]`：`e_rel = 2.342792 m / π rad`
5. 于是 QP 从第一拍就要用 `alpha·e = 0.16 × 2.34 m ≈ 0.375 m`（旋转 0.50 rad）的增量去追一个
   2.3 m 外的相对位姿；每关节每周期上限只有 `v_max·dt = 1.0 × 0.008 = 0.008 rad`，只能全部顶满。
   `w_center=1e-4` 远小于 `w_slack=1e3`，所以 QP 宁可把 `dq` 顶到边界、牺牲物体任务。
   `e_obj` 一路涨到 0.55 m 正是“臂已经走开、样品还在支撑上”的必然结果。

**判定：QP 本身没有病；它被喂了一个与当前位姿相差 2.3 m 的冻结目标。**

### 1.3 修复（不改任何冻结判据/阈值）

- 新增 `src/gz_launch/config/c43_s4_gazebo_initial_left.yaml` /
  `…_right.yaml`，分别写左右臂的 `q_grasp12`；
- `c43_s4_dual_arm_gazebo.urdf.xacro` 让左右 `ros2_control` 各自读对应文件，
  使 **出生位姿 = 控制器 home_q = 冻结 grasp 链**；
- `c43_s4_qp_home_override.yaml` 的 `home_q` 改回 `q_grasp12`（不再是 `[0.5,…]`）。

验收：16 s 轨迹里 `active_limit_count` **全程 0**，执行器不再因 `joint_limit_saturation` HOLD。

## 2. T2：修复后暴露的 6 个集成缺陷

T1 修好后，执行器仍不能完成一次 GRASP→LIFT。逐个定位并修复（都不是冻结判据/阈值）：

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 0 | 执行器收不到 wrist/sample 位姿（必然 `attach_snap_unmeasured`） | Fortress 6.18 的 `pose/info → TFMessage` 给的是不带 `model::` 的 `left_wrist3_link` / `c43_sample`；执行器按 6.16 的 `model::link` 精确匹配 | launch 改成 6.18 的帧名（换回 6.16 要改回带前缀名） |
| 1 | `stale_state` HOLD | 控制器在容器里实际只跑 ~50 Hz wall，10 Hz 状态在墙钟上 0.26–0.35 s，超过执行器 0.3 s | 执行器 `use_sim_time=true` + `/clock` remap 到 `/world/c43_s4_world/clock`；状态周期按仿真时间 13×8 ms=0.104 s |
| 2 | `phase table learned` 永远发生在 FAULT 之后 | 相位表只能从 live status **边跑边学**，5 个相位都出现才 attach；`approach_timeout_s=5` 先到 | 执行器启动时从参数预载 6 相位表；live status 仍用 `observeExternal()` 审计 name/index，不一致照旧 HOLD |
| 3 | `attach_refused_weld_state_unknown (stale)` | `/c43/weld_state` 是**事件驱动**（只在变化时发），客户端却把 0.25 s 内无新样本当成未知 | `requestAttach` 只要求“已知 detached”（startup 已确认释放、之后无新 transition）；确认 transition 仍要求新鲜；活性由 0.5 s transition timeout 保证 |
| 4 | attach 请求从不发布 | 状态机 `tickWeld()` 和节点 `driveWeld()` 都调 `tick()`；第二个 tick 把 `action_` 改写成 `Wait`，`consumeSend()` 因此拒绝 | 新增 `armed_action_` 专用槽，`consumeSend()` 取它而不是被覆盖的 `action_` |
| 5 | `RELEASE→RETREAT→release_failed` | 相位驱动模式下“轨迹到终点”的完成规则被整体跳过；5 相位没有 `retreat`，RETREAT 等一个不会来的相位 | 相位驱动且 `t_traj >= duration` 时允许 RETREAT→COMPLETE（与原有 fallback 同一真实信号） |

另外两处是**测试轨迹自身**与修正后起点不匹配，不是产品语义：

- `approach` 原为 `dz=+0.02`：起点已是 q_grasp12，先在抓取前抬 2 cm 会把夹爪抬离样品；改为 0；
- 原 5 相位没有 `release`：执行器只按相位名推进，`lower` 后永远进不了 RELEASE、不会 detach；
  增加 2 s **静止** `release` 相位（W7 的相位驱动本来就认 `grasp`/`release` 这两个静止保持名）。

> 相位表预载是“同一份 spec 在 launch 与 driver 各写一次”，运行时仍以控制器上报的
> `phase_index/phase_name` 为准并审计；这是测试 harness 的配置，不是把安全审计绕过去。

## 3. T2 干净跑结果

`probe_t2.py`，6 相位：approach 3 s / grasp 2 s / lift 3 s / transport 3 s / lower 3 s /
release 2 s（总 16 s）。json：`t2_run8_clean.json`；launch log：`t2_launch_clean.log`。

| wall t | 状态 | 相位 | `t_traj` | weld | 样品 z (m) | `lim` |
|---|---|---|---|---|---|---|
| 0.31 | RUNNING | approach | 0.10 | — | 0.2750 | 0 |
| 8.15 | RUNNING | grasp | 3.02 | — | 0.2750 | 0 |
| 8.18 | RUNNING | grasp | 3.02 | **attached** | 0.2750 | 0 |
| 10.76 | RUNNING | lift | 5.10 | attached | 0.2768 | 0 |
| 14.40 | RUNNING | transport | 8.01 | attached | **0.2964** | 0 |
| 18.17 | RUNNING | lower | 11.02 | attached | 0.2965 | 0 |
| 21.94 | RUNNING | release | 14.04 | attached | 0.2763 | 0 |
| 21.96 | RUNNING | release | 14.04 | **detached** | 0.2763 | 0 |
| 26.77 | **COMPLETE** | release | 16.00 | detached | 0.2750 | 0 |

三条判据：

1. `weld_state` 出现 `attached` —— 是（8.18 s）；
2. 样品随臂抬起 —— 是（0.2750 → 0.2964 m，+2.1 cm；抬升发生在 lift 段）；
3. `RELEASE` 阶段出现 `detached`，之后样品落回支撑 —— 是（21.96 s detach，z 回 0.2750 m）。

全程无 HOLD / FAULT，`active_limit_count` 恒 0。

执行器自己的 artifact（`executor_artifacts_run8/`）给出同一条链，且
`protection_events=0`、`result=COMPLETE`：

```text
advance APPROACH→GRASP   controller phase 'grasp'
weld    GRASP  attach_requested
weld    GRASP  attached   weld confirmed with snap within the frozen criterion
advance GRASP→LIFT
...
weld    RELEASE detach_requested
weld    RELEASE detached
advance RELEASE→RETREAT
advance RETREAT→COMPLETE  controller reported the trajectory reached its end
```

## 4. 诚实边界（不要把这些当 PASS）

- **这是 Gazebo `DetachableJoint` 焊接，不是真实夹爪抓持**。没有手指反馈、没有抓持误差检查
  （S4-T16 代码已接、物理抬高实验仍未做，仍为 PARTIAL）；样品是被焊在 `left_wrist3_link` 上带起来的。
- 执行器启动日志硬编码打印 `dry_run=true (no actuator writes)`，而 artifact `manifest.json`
  的 `dry_run` 默认是 `false`（两者不一致，是本轮未改的既有显示问题）。语义上：执行器自己
  不写执行器命令，实际位置命令由 `dual_arm_qp_controller` 经 ros2_control 写入。不要把
  `dry_run` 读成“没有驱动”。
- 本机容器 RTF ≈ 0.5，16 s 仿真轨迹花了 ~27 s wall；执行器计时已切到仿真时间。
- Fortress **6.18** 的 `pose/info → TFMessage` bridge 给的是**不带 `model::` 前缀**的
  `left_wrist3_link` / `c43_sample`；W6/W7 在 6.16 上记录的是带前缀名。launch 里已按 6.18 配置，
  换回 6.16 要改回 `s622_dual_arm::left_wrist3_link` / `c43_sample::base_link`。
- D2-B 仍是 `INPUT_ERROR`（跨机重构建哈希），**没有**改 `PROVENANCE.md`、没有改 C3 判据；
  `LEDGER_OK` 只代表结构/覆盖，不是 S4.1 验收 PASS。
- 工作树有迁移前已有的 Docker/GPU 改动（`CLAUDE.md`、`s622_gazebo.launch.py`、
  `test_single_arm.launch.py`、`libfairino.so.2`），本轮没有回滚、没有 commit、没有 push。
  注：`AI_CONTEXT.md` 原也在该列表，已于 2026-10-30 删除。

## 5. 证据索引

| 文件 | 内容 |
|---|---|
| `probe_t1.py` / `probe_run1.json` | 修复前：`e_rel` 2.196 m / 2.927 rad，lim 11–12 |
| `probe_run2_after_spawnfix.json` | 修复后：`e_rel` 3e-4 / 1e-3，lim 0 |
| `probe_t2.py` | 6 相位执行器驱动 + weld + 样品 z 记录 |
| `t2_run8_clean.json` / `t2_run8_stdout.txt` | **最终 T2 干净跑**（三条判据） |
| `t2_run1..7*.json` | 失败路径（yaw 参数、precheck、stale、相位表、attach、retreat） |
| `t1_launch_before_fix.log` / `t1_launch_after_spawnfix.log` | 前后 `q0` 与 `lim` 对比 |
| `t2_launch_clean.log` | 最终执行的执行器/控制器/Gazebo 日志 |
| `executor_artifacts_run8/` | 执行器 artifact：`events.csv`（含 snap 判据通过）、`summary.json`（COMPLETE / 0 protection）、`manifest.json`（git_head=3ac62a2） |
| `selfcheck_final.log` | 最终 §5 自检（172 用例、LEDGER_OK、D2-B INPUT_ERROR） |
| `executor_ctest_after_fixes.log` | 修复后执行器 ctest 摘要 |
| `executor_ctest_after_T4.log` / `_full.log` | W4 台账更新后的 172 用例回归 |
| `ledger_check_after_T16_T18_update.log` | `S4-T16/T18` 更新后的 `LEDGER_OK` 输出 |
| `s4t16_normal_pass.log` / `s4t16_run7_normal.json` | S4-T16 正常跑：grasp check PASS + attach + lift |
| `s4t16_5mm_reject.log` / `s4t16_run8_inject5mm.json` | S4-T16 反例：5 mm 注入 → `grasp_check_failed`，无 attach |
| `executor_artifacts_s4t16_fault/` | 反例的执行器事件（`grasp_check_failed`，无 `attach_requested`） |
| `pose_bias_relay.py` | 5 mm 反例用的 ROS 边界位姿中继 |
| `fault_propagation_5mm.log` / `fault_prop_5mm3.json` | FAULT 传播：`grasp_check_failed` → 控制器 `ABORT`，t_traj=4.35 冻结，z=0.2748，无 attach |
| `executor_artifacts_fault_propagation/` | FAULT 传播的执行器事件（无 attach） |
| `fault_propagation_ctest_188.log` | FAULT 传播 + 仍附着 start gate 后的 188 用例回归（历史） |
| `release_full_sequence.log` / `release_run4.json` / `release_finger_positions.txt` | 完整 RELEASE：detach→100 ms→张手→retreat→COMPLETE（19.5 s），手指 ±0.0305 |
| `attached_fault_detach_fail.log` / `attached_fault_run1.json` / `attached_start_refused.txt` | 带物 FAULT：detach 注入失败，不拆焊；reset 后 start 拒绝 |
| `approach_saturation_diagnosis.txt` | APPROACH 误触发：单关节 lim=1，非 T1 多关节失控；min_joints=6 |
| `release_ctest_190.log` | APPROACH + RELEASE + 带物 FAULT 后的 190 用例回归 |
| `release_open_fail.log` / `open_fail_run4.json` / `executor_artifacts_release_open_fail/` | Gazebo 张手失败：detach 后 100 ms 报 `release_failed`，无张手/retreat/COMPLETE |
| `final_ctest_190_after_open_inject.log` | 运行时失败注入参数加入后的 190 用例回归 |
| `code_changes.diff` | 本轮产品/配置改动（不含迁移前的环境改动） |

## 6. 复现

```bash
# 容器内
cd /workspace && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch gz_launch c43_s4_full_flow.launch.py          # headless，约 22 s 起好
python3 "docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029/probe_t2.py" /tmp/t2.json
# 停止
pkill -INT -f "[r]os2 launch gz_launch"; sleep 6; pkill -TERM -f "[i]gn gazebo"
```

## 7. T4 已完成 / 仍未做

**T4（2026-10-29）已完成：**

- `C4.3_S4_W7_实施与证据_20261028.md` 补了 §10“真跑结果”，并在文首加了 10-29 状态更新；
- W4 台账按实测更新：`S4-T18` 从 `NOT_RUN` 改为 `PARTIAL`（6.18 实跑，但缺最小双体世界与
  1 s 相对位姿稳定上限）；`S4-T16` 改为 `IMPLEMENTED` / evidence `PARTIAL`（代码接入、
  正常跑 PASS、5 mm 注入拒绝；物理抬高与 3 mm/1° 标定未做）；
- 台账检查器仍 `LEDGER_OK`，执行器最新 **188 用例**全绿。

**仍未做（W7 仍 `INTEGRATION_HOLD`）：**

- S4-T16：代码已接（JTC + finger_position + GRASP 入口位姿快照 + 3 mm/1° 检查），
  正常跑 PASS、5 mm ROS 注入拒绝；**物理**抬高 5 mm 的实验与 3 mm/1° 标定仍未做；
- 完整释放顺序：`detach → 100 ms → 张手 → 后退`，目前只有 detach 闸门；
- W7-C 的重复运行与成功率统计（现在只有一次功能路径）；
- T3（原生 RT / C6）仍需非 WSL、带 PREEMPT_RT 的机器；
- 相位表预载若长期保留，建议改成控制器提供 `~/phase_table` 服务，避免 spec 两处维护；
- 6.16/6.18 的 pose frame 前缀差异建议在执行器里做兼容匹配，而不是只改 launch。


## 8. S4-T16（2026-10-29 追加）

**实现**：`require_grasp_check=true` 时，执行器进入 GRASP 后先快照**闭合前**的
`left_wrist3_link`/`c43_sample` 位姿，再向 `/left_hand_controller/follow_joint_trajectory`
发 `left_finger1/2 = ±0.0251`；`/joint_states` 确认手指到位后，用冻结
`object_T_left_tcp` + 0.2168 m 的 `wrist3→grasp` 偏置比较 `wrist3_T_object`，
超过 3 mm / 1°（规划建议值，仍 `[待标定]`）就 `FAULT(grasp_check_failed)`，
并且状态机在检查通过前**不发 attach**。

| 场景 | 结果 |
|---|---|
| 正常跑 | `grasp check PASS: pos=0.001469 m, rot=0.007637 rad` → attach → lift z 0.2750→0.2944 m → detach → COMPLETE；`lim=0` |
| +5 mm 反例（ROS 边界注入） | `grasp check FAIL: pos=0.007251 m` → `GRASP→FAULT(grasp_check_failed)`；artifact 无 `attach_requested` |

证据：`s4t16_normal_pass.log`、`s4t16_5mm_reject.log`、`s4t16_run7_normal.json`、
`s4t16_run8_inject5mm.json`、`executor_artifacts_s4t16_fault/events.csv`、
`pose_bias_relay.py`、`s4t16_ctest_187.log`（187 tests / 0 failures）。

**边界**：5 mm 是 ROS 边界注入（只改执行器看到的 `c43_sample` z），不是物理抬高；运行中塞
5 mm 垫块会把样品顶进夹爪、把臂推离 `q_grasp12`，start gate 直接拒绝。真实夹爪力/接触未测，
3 mm/1° 仍是建议值。2026-10-30 已修 FAULT 传播：执行器 FAULT 经现有 `~/abort_transport`
停控制器（`fault_propagation_5mm.log`），5 mm 反例控制器 `ABORT`、`t_traj=4.35` 冻结、
样品 z=0.2748 未被带落；已附着故障只做代码审查，`reset()` 后仍附着时 `start()` 已拒绝。


## 9. FAULT 传播（2026-10-30）

`grasp_check_failed` 原来只让执行器本地 FAULT，QP 控制器继续跑完轨迹（5 mm 反例控制器
`COMPLETE`，样品被带落到 z=0.025）。现在：

- `afterTransition()` 进入 FAULT 时经现有 `AbortRequester` 发 `~/abort_transport`（只启动一次）；
- `driveWeld()` 在 FAULT 直接 return，不再 attach/detach；
- 日志区分 `weld is attached / not attached`，两者都不拆焊；
- `start()` 在 weld 仍附着/未知时拒绝，`why=weld_state_not_detached`。

证据：`fault_propagation_5mm.log`（`grasp check FAIL 5.021 mm` → FAULT → 3 次 abort →
`controller stop confirmed`）、`fault_prop_5mm3.json`（控制器 `ABORT`，`t_traj=4.35` 冻结，
z=0.2748，无 attach）、`executor_artifacts_fault_propagation/events.csv`。
回归 **190 tests / 0 failures**。已附着故障的 Gazebo 注入见 §12；真实夹爪力/接触未测。


## 10. APPROACH 速度饱和误触发（2026-10-30）

失败运行 `fault_prop_5mm2.json` 的 `max active_limit_count=1`，窗口内 `q_state` 几乎冻结；
不是 T1 的 10–12 关节失控，也不是位置限位。执行器按 125 Hz tick 累加，控制器 Status 约
10 Hz，同一个 `lim=1` 样本被重复计数约 12 次，25 tick≈0.2 s 单关节边界就 HOLD。
修复：速度项要求 `active_limit_count >= limit_min_saturated_joints`（C4.3=6），位置裕度仍
任意关节；`SingleJointCapDoesNotHoldButMultiJointDoes` 覆盖 1 关节不拦、10 关节仍拦。

## 11. 完整 RELEASE（2026-10-30）

`release_full_sequence.log` / `release_run4.json`：detach 确认 → 100 ms → 张手 JTC 成功 →
手指 ±0.0305 → retreat 相位 → `COMPLETE`，t=19.50，样品 z=0.2750 未被推动。release 相位
从 2.0 s 延长到 3.5 s，保证张手确认在 retreat 之前；总时长 19.5 s。
张手失败由 `W7_ReleaseOpenFailureIsAFaultAndDoesNotRetreat` 和 Gazebo 运行时参数
`release_inject_open_failure=true` 双重覆盖：detach 确认后 100 ms 报 `release_failed`，
没有张手/retreat/COMPLETE；样品已 detach，停在支撑上（z=0.2750），控制器 ABORT。

## 12. 带物 FAULT（2026-10-30）

`C43_INJECT_DETACH_FAILURE=1` 让 release 的 detach 请求失败（startup 释放不受影响）：
`FAULT(release_failed)`、日志 `weld is attached... refusing to detach`、无张手/退避；
控制器 ABORT，样品保持 attached、z=0.2750；reset 后 start 返回 `weld_state_not_detached`。
证据：`attached_fault_detach_fail.log`、`attached_fault_run1.json`、`attached_start_refused.txt`。

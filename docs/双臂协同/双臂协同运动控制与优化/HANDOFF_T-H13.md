# 交接单：C4.3 S4 / W3-R1a — T-H13 受限实施

> 用途：在新会话中作为唯一入口。新会话不继承历史上下文，**本文件即全部交接内容**。
> 上一轮已确认：任务 A（Gate 范围 CR）已完成待核验；任务 B（T-H13 C1–C5）= `NOT EXECUTED`；任务 C（Gate 矩阵）仅整理、不申请 PASS。

## 0. 一句话任务

按**已冻结**的 T-H13 设计扩展现有 RT 相位计时，跑 A/B/C 三轮各 638 周期，出 C1–C5 证据，写实施证据文件；**不申请 W3-R1a PASS，不启动 W3-R2**。

## 1. 仓库与当前状态（以磁盘为准，动手前先自行核对）

- 工作目录：`/home/yep/my_S622`（ROS 2 Humble，`colcon --merge-install --symlink-install`）
- 分支/提交：`master`，`HEAD = bb36fe2`
- 工作区：约 **68 项**改动，**全部保留，不得丢弃、不得批量暂存**
- 生产库必须 **OFF**：`strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate` 应为 `0`

| 门禁 | 状态 |
|---|---|
| W2 | `BLOCKED` |
| B-4 | `P0 OPEN` |
| W3-R1a Gate | `HOLD` |
| W3-R2 | `NOT STARTED` |
| T-H13 C1–C5 | `NOT EXECUTED` |
| T-H13 C6 | `UNVERIFIED`（依赖原生 Linux） |
| S4-T2 / S4-T17 | `BLOCKED_EXTERNAL_ENV` |
| T-H16 / T-H15 / S4-T21 | `BLOCKED_BY_W3-R2` |
| T-H17 | `PARTIAL / OPEN`（分层：a-1 PASS / a-2 [PROOF] / b,c 策略层 PASS） |

## 2. 环境与构建

```bash
cd /home/yep/my_S622
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
colcon build --merge-install --symlink-install --packages-select dual_arm_qp_controller \
  --cmake-args "-DPython3_EXECUTABLE=$(which python3)"
source install/setup.bash
export LD_LIBRARY_PATH=$PWD/install/lib:$LD_LIBRARY_PATH
ctest --test-dir build/dual_arm_qp_controller --output-on-failure
```

测试二进制在 `build/dual_arm_qp_controller/test_*`。台架测试会 fork `ros2 launch`，必须 `source install/setup.bash`；每个台架用例使用独立 `ROS_DOMAIN_ID`（见 `test/test_hold_transport.cpp` 的夹具），否则会互相污染。

## 3. 必读（按序）

1. `docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W3-R1a_T-H13_RT相位计时点设计.md`（**已冻结**，215 行，判据以此为准）
2. `src/dual_arm_qp_controller/include/dual_arm_qp_controller/timing_recorder.hpp`
3. `src/dual_arm_qp_controller/src/timing_recorder.cpp`
4. `src/dual_arm_qp_controller/src/dual_arm_qp_controller.cpp` 的 `update()`
5. 参考：`docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_CR-W3-R1A-001_Gate范围变更.md`（归属，不改标准）

## 4. 实施要求（按冻结设计，不得自行改判据）

- 扩展 `TimingRecorder`：新增 `branch`、`exit_code` 与六个相位列 `p1_read_us, p2_dispatch_us, p3_safety_us, p4_task_solve_us, p5_write_us, p6_publish_us`
- 相位边界（`update()` 内插入 `mark()`）：
  - **P1** `readState()` 返回后
  - **P2** `applyPendingStart()` + hold drain
  - **P3** 安全层（臂距胶囊 / D7 health / E8 心跳）
  - **P4** `transport_->step()` 或 IDLE 分支的 `qp_->solve()`
  - **P5** `clampToLimits` + `writeCommand`
  - **P6** `publishStatus()`
- `branch`：`0=idle`、`1=transport`、`2=early_exit`；**提前返回的样本必须保留**（否则系统性剔掉最慢周期）
- 运行时开关 `timing_phases`（`0` = 只 P0/P7，`0x3F` = 全相位），**不新增构建选项**
- **`timing_phases=0` 时原有行为逐位不变**（改前改后对同一输入比较 CSV 的 P0/P7 列）
- RT 路径零分配、零 syscall（RSS/fd 只在 `configure/reset/rebaseResources/summarize` 采样）

## 5. 验证（C1–C5；C6 不设阈值）

A 轮 = 记录器关闭；B 轮 = `timing_enable=true, timing_phases=0`；C 轮 = `timing_phases=0x3F`。**每轮 638 周期**。

| 项 | 判据 |
|---|---|
| C1 | 三轮均完成 638 周期且 `dropped == 0` |
| C2 | `p99(C) − p99(B) ≤ 100 µs` |
| C3 | 每个样本 `sum(P1..P6) ≤ cycle_us`（相位不得自我重叠） |
| C4 | `P6 == 0` 的样本须 `branch != 1` 或 `publish_trylock_miss_` 递增 |
| C5 | 各相位 p50/p99/max 全部落盘且非零（提前返回样本除外） |
| C6 | **`UNVERIFIED`**，依赖原生 Linux，**禁止就地降标** |

## 6. 交付

1. 代码改动（仅 `timing_recorder.{hpp,cpp}` + 控制器 `update()` 插点 + CMake 必要项）
2. 原始日志（A/B/C 三轮 CSV + 统计输出），路径写入证据文件
3. 新建 `docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W3-R1a_T-H13_实施证据.md`：改动清单、原始数据、C1–C5 逐条判定、未验证项、残留风险
4. 据实更新 Gate 矩阵；**C1–C5 未真跑不得写 PASS**
5. 报告工作区前后对比（`git status --short` 前后项数）

## 7. 禁止事项

- 启动 W3-R2
- 改 `HoldRequester` 产品语义
- 新增相位驱动或故障注入 hook
- 修改 T-H13 / T-H17 冻结证据的历史内容
- 扩大为 T-H15 / T-H16 动态验证
- 改动无关用户文件，特别是：
  - `src/dual_arm_qp_core/include/dual_arm_qp_core/dual_arm_qp.hpp`（用户手写的 `/// CBF` 注释）
  - `ADMM_详细数学原理与S622源码对照.md`
  - 未跟踪的教学文档与 `debug_worker_view.png`
- `git commit` / `git push` / `git add -A` / `reset --hard` / `clean -fd`
- 降低任何已批准阈值

## 8. 完成后的动作

停止执行，提交实施证据与工作区前后对比，**等待独立复核**；不得自行宣布 W3-R1a PASS。

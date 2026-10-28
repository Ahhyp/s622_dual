# 交接单：C4.3 S4 / W2 + W3 审计（新窗口专用）

> 用途：在**新会话**中作为唯一入口，做 W2/W3 的**审计**（不是实施）。
> 生成时间：2026-10-27（Asia/Shanghai）。基线代码状态见 §2；本文件不改动任何既有文件。

## 0. 一句话任务

对 W2（12/15 项门禁）与 W3（R1 已完成、R2–R6 未开始）的**证据与判定**做独立审计：逐项核对判据原文、证据文件、实测数字、可复算性，输出**分级结论**（数据直接证明 / 代码结构证明 / 空真成立 / 证据缺口）。**不实施、不改代码、不改冻结设计、不启动 W3-R2。**

## 1. 边界（先读，违反即越权）

**只读审计允许**：`git status`/`git diff`/`git log`、读文件、`strings`、`md5sum`/`sha256sum`、`ctest`（只跑既有测试）、跑 `T-H13_审计快照/05_scripts/*` 重算脚本、命令行复算 CSV。

**禁止**：
1. 修改任何源码 / 测试 / 冻结设计 / 已签发的证据文件（含 `C4.3_S4_W3-R1a_T-H13_实施证据.md`、`C4.3_S4_CR-W3-R1A-001_Gate范围变更.md`、`T-H17` 冻结证据）；
2. `commit` / `push` / `git add -A` / `reset --hard` / `clean -fd`；
3. 用 `-DC43_ENABLE_TEST_HOOKS=ON` 重编后把产物留在 `install/`（生产库必须保持 OFF：`strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate` 应为 `0`）；
4. 启动 W3-R2、推进 W3 后续轮次、或把「审计结论」写成「Gate 放行」；
5. 替实施方补齐缺失数据（改前/改后 P0/P7 那段**已不可恢复**，不得伪造、不得用新数据冒充历史数据）；
6. 就地降低任何判据（尤其 T-H13 C6、W2-G4「≤1 ULP」、S4-T17 的 12 ms）。

## 2. 磁盘状态（动手前先自己核对，不要相信本文件的数字）

```bash
cd /home/yep/my_S622
git rev-parse --short HEAD          # 期望 bb36fe2
git rev-parse --abbrev-ref HEAD     # 期望 master
git status --porcelain | wc -l      # 期望 74（= 12 M + 62 ??；其中 1 项为本文件）
strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate   # 期望 0
md5sum install/lib/libdual_arm_qp_controller.so   # 期望 125d820d2c318e3f2c3ab5b4b6edb915
md5sum install/lib/libdual_arm_qp_core.so         # 本次实测 329a3bb35c286400c8438aa4c7e00cbe；W2 文件只写「md5 未变」未给基线值 ⇒ 该 PASS（G-14）缺一个可核对的基线摘要，属审计发现
```

- 工作区改动**全部保留**，不得丢弃、不得批量暂存、不得清理。
- 不可触碰的用户文件（审计时只读）：`src/dual_arm_qp_core/include/dual_arm_qp_core/dual_arm_qp.hpp`（手写 `/// CBF` 注释）、`docs/双臂协同/双臂协同运动控制与优化/教学文档/dual_arm_qp/ADMM_详细数学原理与S622源码对照.md`、以及 `debug_worker_view.png` 等未跟踪教学材料。

## 3. 审计对象 A：W2（两套矩阵并存 —— 先解决口径冲突）

存在**两份编号不同**的 W2 矩阵，必须先把它们对齐，否则无法给出结论：

| 矩阵 | 文件 | mtime | 项数 | 结论 |
|---|---|---|---|---|
| 15 项版 | `C4.3_S4_W2-R6_W2门禁矩阵与待审批条款.md`（§1） | 2026-10-24 15:15 | G-1…G-15 | 10 PASS / 1 FAIL(G-3) / 2 BLOCKED(G-7,G-11) / 1 待审批(G-9) / 1 不整改(G-10) |
| 12 项版 | `C4.3_S4_W2-W3双轨门禁矩阵.md`（轨 A） | 2026-10-25 15:26 | W2-G1…G-12 | 8 PASS + 1 CLOSED(G7) / 1 FAIL(G4) / 2 BLOCKED(G5,G6) |

12 项版自称「唯一的口径来源」（其开篇第 3 行：`本表是唯一的口径来源`），且 mtime 更新；但它**比重**了编号且**丢了三项**，其中一项是阻断项：

| 12 项版 | 15 项版 | 备注 |
|---|---|---|
| W2-G1 台架 638 周期闭环 | G-6 | PASS / PASS |
| W2-G2 行装配逐周期一致 | G-2 | PASS / PASS |
| W2-G3 前馈符号单调性 | G-4 | PASS / PASS |
| W2-G4 原判据「≤1 ULP」 | G-3 | FAIL / FAIL（**同一事实**） |
| W2-G5 S4-T17 实时性 | G-7 | BLOCKED_EXTERNAL_ENV / BLOCKED_EXTERNAL_ENV |
| W2-G6 胶水层自有分配 | G-9 | BLOCKED / 待 C1 审批（**同义，措辞不同**） |
| W2-G7 分配归属闭合 | G-8 | CLOSED / PASS（CLOSED） |
| W2-G8 E4 4/5 次边界 | G-12 | PASS / PASS |
| W2-G9 钩子隔离 | G-13 | PASS / PASS |
| W2-G10 构建/回归一致性 | G-15 | PASS / PASS |
| W2-G11 抓取变换单一来源 | G-14 | PASS / PASS |
| W2-G12 命令安全语义（E4/E8 归因） | **G-11 保护链完整性** | **冲突：12 项版 PASS vs 15 项版 BLOCKED**（`protection_complete = false`，E2 Degraded / E7 Missing / E8 未注入） |
| —（12 项版无此项） | G-1 物体任务接入 | 15 项版 PASS；12 项版**未列** |
| —（12 项版无此项） | G-5 `π−0.05` 拒绝边界 | 15 项版 PASS；12 项版**未列** |
| —（12 项版无此项） | G-10 第三方分配 OSQP 42.23/solve | 「不整改」；12 项版**未列** |

**审计重点 W-1**：12 项版 W2-G12（PASS）与 15 项版 G-11（BLOCKED）是否指同一件事？若指同一保护链，12 项版相当于把一个 BLOCKED 合并进了 PASS 行，属**结论上调**，必须由你裁定（不得由实施方自行解释）。
**审计重点 W-2**：`C4.3_S4_W2_验收记录.md`（2026-10-23，464 行）是更早的原始验收记录，其 §3 有 638 周期闭环数据（终点 `e_obj = 2.97e-08 m`）；需核对它是否被 R5/R6 覆盖或改写。

**审计重点 W-4（新发现，供你裁定口径）**：G-14「S2/S3 冻结基线未变」的证据只写「`libdual_arm_qp_core.so` md5 未变」，**未记录基线 md5 值**；本次实测为 `329a3bb35c286400c8438aa4c7e00cbe`，因此该 PASS 目前**无法被第三方对照**（只能证明「现在这个值」，不能证明「等于冻结时的值」）——除非从 git 历史或构建产物时间线反推。同类问题：G-7/S4-T17 的 12 ms 阈值有记录，但**原始 `interval` 分布**是否落盘需核。

### W2 逐项「该核什么」

| 项 | 需独立核验的点 | 关键数字（供对照） |
|---|---|---|
| 638 周期闭环 | 台架是否真跑 638 周期、`committed` 全 true、终点误差量级 | `e_obj = 2.97e-08 m` |
| 行装配一致 | S4-T3 是否 `== 0`（不是 ≤1 ULP）、B′ 7656 值 md5 是否相同 | `max|Δrow.J| = max|Δrow.e| = max|Δdq| = 0` |
| 前馈符号/单调性 | 变号与 2× 关系是否在测试中断言 | `test_object_row_wiring` |
| ≤1 ULP（FAIL） | 是否**仍标 FAIL**、是否有人改写口径；近零关节的 ULP 数值是否可复现 | `4.69e20 ULP @ 5.9e-167` |
| S4-T17 12 ms | 本机是否满足 RT 前提（`ulimit -r`、`chrt`、`isolcpus`）；阈值是否被改 | 12 ms 未改；`max interval` 实测 62.7 ms |
| 胶水层分配 | 1.03/solve 归属是否全为 `QpSolution::x`；C1 B′ 原型 0.025/solve 是否可复算；**不得**声称「控制路径零分配」 | 1.03 vs 0.025 /solve；OSQP 42.23/solve 不整改 |
| 分配归属闭合 | 32623 是否 = 27435 + 173 + 2；95 差额是否逐项闭合；UNKNOWN 是否为 0 | 32623 / 27435 / 173 / 2；4501、95 |
| E4 4/5 边界 | 注入 4 次不 HOLD、5 次 HOLD 是否由测试断言（不是口头） | `test_transport_latches` |
| 钩子隔离 | OFF 库 `strings` 是否 0、ON 是否 1；OFF 下是否无法启用 | 0 vs 1 |
| 构建/回归 | OFF/ON 是否各 8/8、35 用例、0 失败 | 72.05 s / 69.36 s |
| 抓取变换单一来源 | `frozen_baseline.hpp` 是否唯一来源、6 例是否通过、`libdual_arm_qp_core.so` md5 是否未变 | `test_frozen_baseline_compat` |
| 保护链（冲突项） | `protection_complete` 当前值；E2/E7/E8 三层各自状态；「未注入」是否等于「未验证」 | `protection_complete = false` |

## 4. 审计对象 B：W3

- `W3-G1 = PASS`（`C4.3_S4_W3-R1_状态机与执行器骨架验收.md`，T01–T14）。**审计重点 W-3**：CR §2 的 R1 行写「13 PASS、`T05` 已由 C7-A 补齐」——需确认 T05 的补齐是否算 PASS、14 项里是否存在被合并/重分类的项。
- `W3-G2…G6 = 待开始`（R2–R6），本轮无证据可审；核对「待开始」是否被某处写成 PASS。
- `W3-G7 = NOT_AUTHORIZED`（真实执行器 + 物理附着 + Gazebo）。
- 阻断项 B-1…B-8 与待批准 C1…C7 见 `C4.3_S4_W2-W3双轨门禁矩阵.md` §阻断清单 / §待批准清单。
- 注意术语陷阱：`C4.3_S4_W3-R1a_实施与验收报告.md` 里的「T-H13 零分配 UNVERIFIED」与 `T-H13` 的「RT 相位计时点」是**同名不同物**，不得互相替代。

## 5. 审计对象 C：T-H13（W3-R1a 受限实施，待独立验收）

| 项 | 状态（实施方报告） | 需你独立核验 |
|---|---|---|
| C1 | PASS（五轮，`dropped=0`） | A 轮 `dropped` 结构性恒 0（记录器关闭 ⇒ `events` 无 CSV），能否算「三轮各 638 周期」 |
| C2 | PASS（配对 1 −76.4 µs、配对 2 +11.8 µs） | 冻结 §5.2 是**有符号**判据；负号不得读作改善；配对方式（窗口 = 最长 `branch==1` 连续块前 638 行）是否可接受 |
| C3 | PASS（3051 行 0 违例） | 逐样本重算；最坏比值 0.9993/0.9992 |
| C4 | PASS（**空真**：`P6==0` 样本数 0） | 空真是否被冻结判据允许；不得升级为「发布竞争归因完成」 |
| C5 | PASS（六相位 `rec=638/638`） | `recorded` 是下界（未执行/亚粒度都记 0）、无插补、溢出为整样本 |
| C6 | `UNVERIFIED` | 依赖原生 Linux；**不得就地降标** |

三个**不可关闭**项（复核方已定性，审计时只能登记，不能消解）：
1. 改前/改后 P0/P7 原始数据缺失（旧 5 列 CSV 已被覆盖）——不伪造、不重建；
2. `ctest 9/9` 未达成（OFF 构建 9 个二进制 / 7 PASS + 2 FAIL，成因已解释但**不自动豁免**）；
3. C4 空真 PASS ≠ 实质覆盖。

另有三条实施方自报的埋点审计发现（登记项，不要求修改设计/代码）：F1 `readState` 失败与 T-H17 注入路径**零相位标记**；F2 E1 早退的耗时**不归属任何相位**（设计空隙）；F3 改用显式 `markTimingPhase` 而非设计建议的 RAII `PhaseScope`。

## 6. 文件索引（按此顺序读，别全读）

| 顺序 | 文件 | 用途 |
|---|---|---|
| 1 | `C4.3_S4_W2-W3任务清单与T-H13审计快照_20261027.md`（568 行） | **一页取全**：W2/W3 矩阵 + 阻断/待批准 + W3-R1a Gate 矩阵 + T-H13 统计与判定 + 出口审计 |
| 2 | `C4.3_S4_W2-R6_W2门禁矩阵与待审批条款.md`（65 行） | 15 项版 W2 矩阵 + C1/C3 条款 + 转入 W3 前置条件 |
| 3 | `C4.3_S4_W2-W3双轨门禁矩阵.md`（75 行） | 12 项版（自称唯一口径）+ 阻断 + 待批准 + 口径声明 |
| 4 | `C4.3_S4_CR-W3-R1A-001_Gate范围变更.md`（59 行） | W3-R1a Gate 逐项矩阵与 T-H13 回填 |
| 5 | `C4.3_S4_W3-R1a_T-H13_实施证据.md`（518 行） | T-H13 全量证据（§0 摘要、§5 统计、§6 判定、§12–§14） |
| 6 | `C4.3_S4_W3-R1a_T-H13_RT相位计时点设计.md`（215 行，冻结） | 判据原文 §5.2、§6.2；相位表 §2 |
| 7 | `T-H13_审计快照/`（25 文件，只读） | 冻结原文摘录、源码 diff、5 份 CSV、脚本、sha256 清单 |
| 8 | `C4.3_S4_W2_验收记录.md`（464 行） | W2 原始验收记录（比两版矩阵都早） |
| 9 | `C4.3_S4_W2-R5_分配归属账本.md`、`…R6_C2归属账本收口.md`、`…R3_测试钩子编译期隔离报告.md`、`…R4_ON-OFF回归矩阵.md` | W2 分项证据 |

## 7. 只读复核命令集

```bash
# 1) 快照完整性（应 24/24 OK、0 FAILED）
cd "docs/双臂协同/双臂协同运动控制与优化/T-H13_审计快照"
sha256sum -c <(grep '  \./' 00_MANIFEST.md)

# 2) T-H13 C3/C4 逐样本重算（无需 ROS）
python3 - <<'PY'
import csv
for lab in ["b","b2","c","c2","c_ee"]:
    rows=list(csv.DictReader(open(f"04_csv/{lab}_timing.csv")))
    bad=[r for r in rows if sum(float(r[k]) for k in
        ["p1_read_us","p2_dispatch_us","p3_safety_us","p4_task_solve_us","p5_write_us","p6_publish_us"])
        > float(r["cycle_us"])]
    p6z=[r for r in rows if float(r["p6_publish_us"])==0.0]
    print(lab,"rows",len(rows),"C3违规",len(bad),"P6==0",len(p6z),
          "branch",sorted({r["branch"] for r in p6z}))
PY

# 3) 生产库 OFF 与冻结基线
strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate   # 0
md5sum install/lib/libdual_arm_qp_core.so | cut -c1-32                            # 与 W2 记录比对

# 4) 回归（只跑既有测试；OFF 构建）
colcon test --merge-install --packages-select dual_arm_qp_controller
grep -c 'PASSED\|FAILED' build/dual_arm_qp_controller/Testing/Temporary/LastTest_*.log
# 已知：9 个二进制 / 7 PASS + 2 FAIL（S4T17 环境轴、T-H17 两例需 hooks=ON）
```

## 8. 结论模板（必须按此分级，禁止混同）

对每一项给出四选一：

1. **数据直接证明**：有原始数据 + 可复算路径（附文件与行号）；
2. **代码结构证明**：由代码分支/顺序推得（附 `文件:行号`）；
3. **空真成立**：判据字面成立但样本集合为空（必须写明空集原因）；
4. **证据缺口**：无数据/数据被覆盖/依赖外部环境。

并给出三句话：`W2 = ?`、`W3 = ?`、`T-H13 = ?`，以及「裁定为 PASS 的项 → 依据哪一类证明」。**审计结论不得写成 Gate 放行**；如需变更 Gate，走 CR 流程。

## 9. 开场提示词（可直接粘贴到新窗口）

```
项目 /home/yep/my_S622（S622 双机械臂，ROS 2 Humble，colcon --merge-install --symlink-install）。

本轮只做 C4.3 S4 的 W2 + W3 审计，不做任何实施。

先读（按序）：
1. docs/双臂协同/双臂协同运动控制与优化/HANDOFF_W2-W3审计.md
2. docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W2-W3任务清单与T-H13审计快照_20261027.md
3. docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W2-R6_W2门禁矩阵与待审批条款.md
4. docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W2-W3双轨门禁矩阵.md

基线（以磁盘为准，请自行核对）：HEAD = bb36fe2，分支 master，工作区 73 项全部保留；
生产库必须 OFF：strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate = 0。

硬约束：
- 只读：不改任何源码/测试/冻结设计/已签发证据；不 commit / push / git add -A / reset --hard / clean -fd；
- 不得读改写改前/改后 P0/P7 那段已缺失数据、不得伪造；不得就地降标任何判据（T-H13 C6、W2 ≤1 ULP、S4-T17 12 ms）；
- 不得启动 W3-R2，不得把审计结论写成 Gate 放行。

审计重点（至少覆盖）：
- W2 两套矩阵（12 项版 vs 15 项版）的映射与冲突，尤其 12 项版 W2-G12（PASS）vs 15 项版 G-11 保护链（BLOCKED，protection_complete=false）；
- W3 R1 的 T01–T14 是否真的 13 PASS + T05 补齐；R2–R6「待开始」是否被任何文件写成 PASS；
- T-H13 C1–C6：C2 有符号判据、C3 逐样本重算、C4 空真是否被冻结判据允许、C5 recorded 下界口径；
- 三个不可关闭项（P0/P7 缺口、ctest 9/9 未达成、C4 空真）只能登记。

输出：每一项给「数据直接证明 / 代码结构证明 / 空真成立 / 证据缺口」四选一，并给出 W2、W3、T-H13 三句结论。
完成后停止，交审计报告，不做任何实施。
```

## 10. 文件清单与时效

- 本文件为**新增**文件（未修改任何既有文件）。若与 §2 的实值不符，以磁盘为准并记录差异。
- 两版 W2 矩阵的 mtime：15 项版 = 2026-10-24 15:15，12 项版 = 2026-10-25 15:26；`C4.3_S4_W2_验收记录.md` = 2026-10-23 19:15。**版本先后与「谁覆盖谁」须由你在审计中确认**，不要默认新文件就是权威。

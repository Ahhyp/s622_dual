# G-14 baseline record (W2 gate line)

> 状态：**APPROVED — 官方基线**（审批项 **D1-B**，2026-10-27 批准）。
> 依据：`docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_审批记录.md` D1-B 行。
>
> 语义（D1-B 批准后的 G-14）：**「哈希 == 本文件 §1 记录的当前官方基线」**。
> 旧基线值（§2）不得被覆盖，保留为历史对照。回退仍必须可用，且**不得依赖
> `git reset` / `git restore HEAD`**（工作树里还有用户的未提交改动）。
>
> 记录本文的起因：审查发现 G-14 门禁行长期只写"md5 未变"而**从未记录基线值**，
> 使该行无法被第三方复核。补上测量值是修复可审计性缺口，不是改判据。

## 1. 当前实测值（D1-A 集成后，2026-10-27）

| 构件 | SHA-256 |
|---|---|
| `install/lib/libdual_arm_qp_core.so` | `ff95750afe76b49b7738c35eb1264f94791f81004513510e2c8ca939e90a4636` |
| `install/lib/libdual_arm_qp_controller.so` | `3e822417cf40c9acb61ed2436d77352192d407d69859441833c3a93f340b07d4` |

## 2. 历史值（D1-A 之前，保留作对照）

| 构件 | SHA-256 / md5 | 说明 |
|---|---|---|
| `libdual_arm_qp_core.so` | `329a3bb35c286400c8438aa4c7e00cbe` | 旧 md5（D1-A 前的构建）；SHA-256 版本未在旧记录中留存 |
| `libdual_arm_qp_controller.so` | md5 `125d820d2c318e3f2c3ab5b4b6edb915` | W2-R6 时期记录值 |

> 两个时代使用不同摘要算法，**不可直接比较**；本表只用于标识"改前/改后"。

## 3. 复算方法（可重复）

```bash
cd /home/yep/my_S622
sha256sum install/lib/libdual_arm_qp_core.so install/lib/libdual_arm_qp_controller.so
# 源码侧：确认 solveInto 已存在于公共头
grep -n "solveInto" src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp
```

构建配方：`colcon build --merge-install --symlink-install --packages-select dual_arm_qp_core dual_arm_qp_controller
--cmake-args "-DPython3_EXECUTABLE=$(which python3)" -DC43_ENABLE_TEST_HOOKS=OFF`（conda `yolov8` 环境）。

## 4. 为什么哈希会变（且必须变）

D1-A 在 `QpSolution` 上新增 `solveInto()` 虚函数（`qp_types.hpp:46`），使
`solve()` 内部可在调用方提供的目标上写入解，从而消除每周期 `x.resize(24)`
的 192 B 分配（生产实测 638 次 → 1 次/638 周期）。

- `sizeof(QpSolution)` **保持 48**，字段偏移不变（`x` 0、`converged` 16、`iterations` 20）。
- vtable 末尾**新增**一个槽位（未删除、未重排既有槽）。
- 因此 G-14 的"`.so` 哈希不变"在语义上必然被打破：**哈希变了正是该改动的预期结果**，
  而不是回归。G-14 若继续按"哈希不变"执行，会与 D1-A 互斥——这正是需要 D1-B 裁决的原因。

## 5. 与门禁的关系

| 问题 | 答案 |
|---|---|
| 本文件是否改变了 G-14？ | 否。G-14 行在批准前仍按旧口径记 **BLOCKED/待裁决**。 |
| 能否据此宣称 G-14 PASS？ | 不能。缺少 D1-B 签字。 |
| 第三方能否据此复核？ | 能：哈希、配方、复算命令三件齐全。 |
| 回退方式 | `src/dual_arm_qp_core/scripts/c1_rollback_solveInto.sh`（从快照恢复四文件并重建）。 |

## 6. 本轮补记：集成后源码哈希与重建确定性（2026-10-27）

集成脚本当初没有持久化"集成后哈希"，这是审计缺口；本轮补记，避免再次出现
"只记产物哈希、不记源码与配方"的情况。

| 文件 | sha256 |
|---|---|
| `include/dual_arm_qp_core/qp_types.hpp` | `6353ab0daf414447bf04bf74356678a368d20b96e3a6022704f12ce477cc95f8` |
| `include/dual_arm_qp_core/osqp_solver.hpp` | `77095e4e074427ea5140590fe88f64a9e9f51f62ae9b0b92397fcd4aa4ef9b75` |
| `src/osqp_solver.cpp` | `dcdf02214ab9b1dadb52cef8edeecd7d8f3e1ad7140b7d15aa2c7a8bd9b1c376` |
| `src/dual_arm_qp.cpp` | `c9f182ab006e5cd816d06127156036d9a9e118b8495a5c108948b4d7cdbc0fb9` |

**重建确定性实测**：`PKGS=dual_arm_qp_core JOBS=1 bash scripts/build_mem_safe.sh`
重建后 `libdual_arm_qp_core.so` sha256 仍为
`ff95750afe76b49b7738c35eb1264f94791f81004513510e2c8ca939e90a4636` ⇒
同一配方、同一源码树的重建是逐位确定的（因此本基线可被第三方复算并对照）。

**仍未解决**：旧基线（md5 `329a3bb3…`）的字节从未保留，配方也未记录，因此"旧值"只能作为
历史记录存在，不能用于字节级对照。D1-B 要求把 G-14 改为"哈希 + 配方 + 日期"三元组，
正是为了避免再次出现这种不可复算状态。

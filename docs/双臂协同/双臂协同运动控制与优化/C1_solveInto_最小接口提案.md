# C1 裁决材料：`solveInto` 最小接口提案（**未实施，待批准**）

> 状态：**提案 / 未写入冻结版本**。G-14 旧基线（`libdual_arm_qp_core.so` md5 未变）**保留有效**；
> 本文任何实现都还没有落到 `src/`。批准前 `qp_types.hpp` 不得改动。
> 依据：`src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp:29-45`、`src/osqp_solver.cpp:175-215`。

## 1. 问题与既有事实（来自 R5/R6 账本与本轮复算）

| 事实 | 数值 | 来源 |
|---|---|---|
| 每 solve 自有分配总数 | 655 次 / 638 solve = **1.03/solve** | `/tmp/w2r5` 账本（UNKNOWN=0） |
| 其中 `QpSolution::x` 的 `resize(24)` | **638 次**（其余 17 次为 warmup/初始化） | `osqp_solver.cpp:210`，站点 `libdual_arm_qp_core.so+0x2540f` |
| OSQP 内部（不可控） | 26 940 次 = 42.23/solve | 同上，属第三方 |
| 一次分配的字节数 | 192 B / solve | R5 账本 |

`QpSolution` 按值返回、`x` 是空的 `Eigen::VectorXd`，**任何**给 `x` 赋值/拷贝/移动都必须分配：

| 候选实现（本轮实测，638 solve） | 自有分配 | 结论 |
|---|---|---|
| 现状：`QpSolution` 按值返回 | 655（1.0016/solve） | 基线 |
| 预分配缓冲 + 赋值 | 656 | 无改善（赋值到空向量仍分配） |
| 复用同一 `QpSolution` 对象并拷贝 | 656 | 无改善（拷贝必分配） |
| 返回 `std::move` | 655 | 只是把分配推迟 |
| **`solveInto(problem, dst)` + 调用方持有 sink** | **17（0.0016/solve）** | 唯一有效方案 |
| 进程级 malloc 池（`LD_PRELOAD`） | 请求数不变 | 只减少 glibc 调用，不改分配次数 |

## 2. 最小接口改动（精确 diff）

```diff
--- a/src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp
+++ b/src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp
@@ class QpSolver {
     virtual ~QpSolver() = default;
     virtual QpSolution solve(const QpProblem& problem) = 0;
     virtual std::string name() const = 0;
+    /// Same result as solve(), written into a caller-owned sink.  `dst.x` is
+    /// resized only if its size differs, so a caller that keeps `dst` alive
+    /// across cycles performs zero allocations in steady state.
+    /// Default implementation keeps every out-of-tree backend working.
+    virtual void solveInto(const QpProblem& problem, QpSolution& dst) {
+        dst = solve(problem);
+    }
 };
```

```diff
--- a/src/dual_arm_qp_core/src/osqp_solver.cpp
+++ b/src/dual_arm_qp_core/src/osqp_solver.cpp
@@ QpSolution OsqpSolver::solve(const QpProblem& problem) {
     QpSolution solution;
+    solveInto(problem, solution);
+    return solution;
+}
+
+void OsqpSolver::solveInto(const QpProblem& problem, QpSolution& solution) {
     if (!problem.valid()) return solution;
```

```diff
--- a/src/dual_arm_qp_core/src/dual_arm_qp.cpp
+++ b/src/dual_arm_qp_core/src/dual_arm_qp.cpp
-    const QpSolution solution = solver_->solve(last_problem_);
+    solver_->solveInto(last_problem_, solution_scratch_);   // member, constructed once
+    const QpSolution & solution = solution_scratch_;
```

**未改**：`QpVec`（`Eigen::Matrix<double,Dynamic,1,0,24,1>`）**不在本提案内**——它是容量写死 24 的隐式契约，
收益与 `solveInto` 相同而侵入更大（改结构体布局）。`x.resize(n)` 在 `dst.x.size()!=n` 时仍会分配，这正确：
数据形状变化必须分配，steady state（n 恒 24）不会。

## 3. ABI 影响

| 项目 | 结论 |
|---|---|
| `sizeof(QpSolution)` | **不变**（48 B，alignof 8，`x` 偏移 0，实测） |
| vtable | 在 `QpSolver` **末尾**新增一个虚函数 ⇒ `QpSolver` 的 vtable 增长 1 槽；`QpSolution`/`QpProblem` 布局不变 |
| 源码兼容 | 是：`solve()` 保留，`solveInto` 有默认实现（掉队的外部后端不会编译失败） |
| 二进制兼容 | **否**（vtable 槽位表变化）：所有 `libdual_arm_qp_core` 使用方需重编；仓内为 `dual_arm_qp_controller`、`dual_arm_transport_mock`、qp_core 自身 10 个测试 + `test_legacy_golden_c43s3`（golden 由旧头生成，必须重跑） |
| 符号扩散 | `.so` 导出符号 133 个；本改动新增 1 个虚函数符号，不删除任何符号 |

## 4. 数值回归证据（原型，`/tmp/w2r5`）

三份独立构建（prod / ctrl / bprime）+ 同一 `l1_driver` 跑 638 周期：

| 构建 | 7656 个 `dq` 的 md5 |
|---|---|
| 生产（现状） | `c21118176f03a70df34c8be5a84b0c20` |
| 对照（仅重编） | `c21118176f03a70df34c8be5a84b0c20` |
| `solveInto` 原型 | `c21118176f03a70df34c8be5a84b0c20` |

逐位相同。**但踩过的坑必须记住**：首轮对照漏了 `-DDUAL_ARM_QP_HAVE_OSQP=1`，得到 `1.5836e-05` 的差 ——
差异来自编译配方而非代码。验证时必须固定 `CMakeCache.txt` 中的 `DUAL_ARM_QP_HAVE_OSQP`。

## 5. 与 G-14 的冲突处理

G-14 要求「`libdual_arm_qp_core.so` md5 未变」，且**其基线 md5 从未记录**（当前实测
`329a3bb35c286400c8438aa4c7e00cbe`）。本提案一旦实施，该 `.so` 必然变化 ⇒ **G-14 必须重新基线化**，
这是审批项而不是实施细节。可选的替代：仅把 `solveInto` 加入头文件、`.so` 保持不变（不可行——虚函数表在 `.so` 内）。

## 6. 审批请求（一句话）

> 批准在 `QpSolver` 末尾新增带默认实现的 `virtual void solveInto(const QpProblem&, QpSolution&)`，
> 在 `DualArmQp` 中持有 `QpSolution` sink，并**把 G-14 的 md5 基线重新定义为实施后的新值**；
> 若基线变更未获批准，则 C1 保持 **BLOCKED**（1.03/solve 继续作为 W2 阻断事实）。

## 7. 回滚

```bash
git restore --source=HEAD -- src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp \
    src/dual_arm_qp_core/src/osqp_solver.cpp src/dual_arm_qp_core/src/dual_arm_qp.cpp
rm -rf build install && colcon build      # ABI 变过必须全量
```

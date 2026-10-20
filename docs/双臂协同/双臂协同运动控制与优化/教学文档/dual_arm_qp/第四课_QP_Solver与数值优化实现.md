# 第四课：QP Solver 与数值优化实现

**S622 双臂 QP 源码与数学精读课程**

> 数学与工程原理讲义，不是 C++ 变量字典或函数参数索引。  
> 源码依据：2026-10-19 查阅 `Ahhyp/s622_dual` 的 `public` 分支中 `qp_types.hpp`、`admm_solver.cpp`、`osqp_solver.cpp`。本课没有修改源码或运行 ROS/Gazebo。  
> 在线源码：[QP 数据接口](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp) · [ADMM](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/admm_solver.cpp) · [OSQP](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/osqp_solver.cpp)

## 课程衔接

第一课讲 `dual_arm_qp_core` 的结构与 `solve()` 流程；第二课从相对位姿误差推导 QP；第三课解释 FK、SE(3)、Jacobian 和 Adjoint。**第四课回答：我们已经得到 P、q、A、l、u 后，计算机怎样真正求出下一步的关节增量？**

本课学完应当能解释：

1. QP 为什么具有全局最优的数学基础；
2. ADMM 中 x、z、y 的含义和迭代；
3. OSQP 与 ADMM 的关系；
4. CSC 稀疏矩阵如何存储；
5. warm start、残差、收敛、超时和不可行有什么区别；
6. 为什么 QP 求解成功仍不意味着真机安全。

---

# 1. 把机械臂问题变成数学试卷

标准凸二次规划（QP）是：

$$
\boxed{\min_x \frac12 x^\top P x+q^\top x}
$$

满足

$$
\boxed{l\le Ax\le u}.
$$

| 量 | 定义 | 在你的双臂系统中 |
|---|---|---|
| $x$ | 待求的向量 | 12 轴关节增量 + 两组六维松弛变量，总共 24 维 |
| $P$ | 二次代价矩阵 | 对误差、运动幅度及 slack 的惩罚 |
| $q$ | 线性代价项 | 误差反馈和偏好产生的线性系数，**不是关节角 q** |
| $A$ | 约束矩阵 | 关节增量界和可选的额外线性不等式 |
| $l,u$ | 上下界 | 限制每个约束的取值 |
| $x_0$ | 可选初始值 | 上一周期解可用于 warm start |

**Solver 不知道什么是 S622。** 它只认识矩阵。`DualArmQp::solve()` 负责制造试卷；`QpSolver` 负责解试卷；控制器决定是否执行答案。

## 1.1 为什么凸性重要？

目标的 Hessian 是 $P$。若 $P=P^\top\succeq0$，目标函数是凸的。线性约束定义的合法区域同样是凸的。

因此满足 KKT 最优性条件的合法解是**全局最优解**。如果 $P$ 正定，且可行集非空，目标通常有唯一最优点。

这**不**保证任意配置都可行，也不保证求解器在指定时间内收敛。

## 1.2 直观例子

$$
\min_x \frac12(x-2)^2,\qquad x\le1.
$$

无约束最优点是 2，但违反上界；合法的最优点是 1。QP 的工作就是在合法集合中尽量减小目标。

---

# 2. 最优性条件：KKT 是什么？

一个点除了满足约束，还要证明无法在允许的方向上继续降低代价。

写成辅助变量形式：

$$
z=Ax,\qquad l\le z\le u.
$$

KKT 的主要条件包括：

- **原始可行性**：$Ax=z$，同时 $l\le z\le u$；
- **驻点**：$Px+q+A^\top y=0$，符号取决于乘子的定义；
- **边界条件**：约束内部的乘子为零，活跃边界上的乘子方向需正确。

这些条件在数值计算中通常不会精确等于零，所以需要**容差**。

---

# 3. ADMM：反复协调目标和约束

ADMM = Alternating Direction Method of Multipliers（交替方向乘子法）。它通过辅助变量把难处理的约束与二次代价分开。

令 $z=Ax$。你的 `admm_solver.cpp` 使用的迭代形式为：

$$
x^{k+1}=
(P+\sigma I+\rho A^\top A)^{-1}
[\sigma x^k+\rho A^\top(z^k-y^k)-q]
$$

$$
z^{k+1}=\operatorname{clip}(Ax^{k+1}+y^k,l,u)
$$

$$
y^{k+1}=y^k+Ax^{k+1}-z^{k+1}.
$$

这里的 $y$ 是缩放后的对偶变量，不是另一个机械臂的运动变量。

### 第一步：更新 x

在兼顾 QP 代价与当前约束信息的情况下求一个候选答案。

### 第二步：更新 z

把 $Ax+y$ 投影到上下界区间。假如一个量被要求处于 $[-1,1]$，而当前算出来是 1.4，就裁剪为 1。

### 第三步：更新 y

累计 $Ax-z$ 的不一致程度，促使之后的迭代更好地协调候选答案和硬约束。

你可以把 ADMM 想成两个角色反复商量：一个负责“代价最小”，一个负责“约束合法”。

## 3.1 rho 和 sigma 是什么？

- $\rho$：控制 $Ax$ 和 $z$ 一致性惩罚的强度。
- $\sigma$：提供额外的近端/数值正则项。

过大的 $\rho$ **不一定**更快：一种残差可能改善，另一种却会恶化，矩阵条件也可能变差。

## 3.2 为什么不直接算矩阵逆？

代码构造：

$$
K=P+\sigma I+\rho A^\top A
$$

随后使用 Eigen `LLT` 分解和 `llt.solve(rhs)` 求线性系统，而不是显式构造 `K.inverse()`。对对称正定矩阵，这通常更稳定，也更高效。

值得注意的源码细节：ADMM 的 `LLT` 失败分支再次调用相同的 `factorize()`，但没有改变正则参数。注释声称增加正则，实际代码并未做到。**这是工程审查发现，不是 ADMM 数学原理有问题。**

---

# 4. 残差：怎么判断解得够好了？

你项目的参考 ADMM 每轮计算：

$$
r_{\rm primal}=Ax-z
$$

$$
r_{\rm dual}=\rho A^\top(z^k-z^{k-1})
$$

其中残差大小取无穷范数（各分量绝对值的最大值）。

**Primal residual** 大，意味着候选解和约束投影仍然不一致。**Dual residual** 大，意味着对偶/驻点相关的迭代尚未稳定。

代码使用绝对容差与相对容差共同判定：

$$
\epsilon_{\rm pri}=
\epsilon_{\rm abs}+\epsilon_{\rm rel}
\max(\|Ax\|_\infty,\|z\|_\infty).
$$

只有 primal 与 dual 均达到要求才设置 `converged=true`。

**重要：**不同实现可能使用略有不同的残差定义、缩放和终止策略，不能仅比较两个库输出的一串残差数字，就断言谁一定更精确。

---

# 5. OSQP 与 ADMM 是什么关系？

OSQP 是 **Operator Splitting Quadratic Program**，属于基于算子分裂/ADMM 的凸 QP 求解器。

因此：

- ADMM 是一类优化算法；
- OSQP 是面向凸二次规划的专门实现；
- 它们不是两种完全无关的数学路线。

你的项目通过 `QpSolver` 抽象接口隔离它们：

```cpp
class QpSolver {
public:
    virtual ~QpSolver() = default;
    virtual QpSolution solve(const QpProblem& problem) = 0;
    virtual std::string name() const = 0;
};
```

核心 QP 算法只提供矩阵，不需要直接依赖 OSQP 的内部 C 接口。

当前 `makeDefaultSolver()` 的条件是：**编译时**定义 `DUAL_ARM_QP_HAVE_OSQP` 就选 OSQP，否则选内置参考 ADMM。实际某次编译产物选了哪个，需要核对构建配置和运行时 solver 名称。

---

# 6. 为什么 OSQP 要把 Eigen 矩阵转成 CSC？

CSC = Compressed Sparse Column（压缩稀疏列）。

考虑：

$$
M=
\begin{bmatrix}
4&0&3\\
0&5&0\\
0&6&7
\end{bmatrix}.
$$

普通矩阵存储九个数，但非零数只有五个。CSC 按列保存：

```text
values    = [4, 5, 6, 3, 7]
row_index = [0, 1, 2, 0, 2]
col_ptr   = [0, 1, 3, 5]
```

`col_ptr` 指每列数据在前两个数组中的起始位置。它最后一个元素就是非零元素数量。

### P 为什么只传上三角？

因为 $P$ 通常对称，储存完整上下三角会重复。你代码的 OSQP 0.6.x 后端使用：

- `denseUpperToCsc(P)`：保留 P 上三角；
- `denseFullToCsc(A)`：保存 A 的完整非零项。

这不会改变原 QP 的数学含义，只是数据表示更紧凑。

---

# 7. warm start：为什么可能加速？当前到底做了多少？

连续控制中，相邻周期的 QP 往往相似，可以用上次解作为这次初值：

$$
x_0^{k+1}\gets x_{\rm solution}^{k}.
$$

但是要严格区分：

1. **初值 warm start**：给优化器上次的 $x$。
2. **状态 warm start**：还保留对偶变量和算法状态。
3. **持久 workspace**：重用已建立的求解器、稀疏结构，甚至某些矩阵分解结果。

你的 `osqp_solver.cpp` 每次调用 `solve()` 都：

```text
Eigen P/A
→ 转成 CSC
→ 分配数据
→ osqp_setup()
→ 可选 osqp_warm_start_x()
→ osqp_solve()
→ osqp_cleanup()
→ 释放数据
```

**所以当前有 x 初值 warm start，但没有跨周期持久复用 workspace。**

这件事影响实时性能，但不能仅凭源码判断它一定超时；必须用真实运行的周期耗时统计验证。

---

# 8. 求解成功与求解失败应该怎样理解？

### 8.1 常见失败分类

| 类型 | 例子 | 怎么检查 |
|---|---|---|
| 输入格式非法 | P 非方阵、A 列数错误、l 大于 u | `QpProblem::valid()` |
| 无可行解 | 同一个变量既要求 ≥2 又要求 ≤1 | 约束冲突、不可行状态 |
| 数值不收敛 | 权重差异过大、病态、迭代达到上限 | residual、iterations |
| 算得太慢 | 每轮都 setup、转换、分配内存 | 端到端耗时，P95/P99 |

### 8.2 `valid()` 不是数学可行性证明

当前 `QpProblem::valid()` 检查 P 方阵且非空、q/A/l/u 维度一致、每条约束下界不大于上界。

但是它**没有证明全部约束可以同时满足**，也没有全面检查 P 是否对称半正定、元素是否存在 NaN。

### 8.3 `converged=true` 不等于物理安全

当前 `osqp_solver.cpp` 将 `OSQP_SOLVED` 和 `OSQP_SOLVED_INACCURATE` 都接受为 `converged=true`。

这告诉你后端认为答案满足其接受条件，但还需要评估：

- 最终约束违反量；
- 求解器原始状态、primal/dual residual；
- 控制器是否健康；
- 指令与真实关节反馈是否同步；
- 物体接触和抓持是否稳定。

求解器只负责数学优化，不能替代设备级安全保护。

---

# 9. 一个可以自己验证的小 QP

为了不被 24 维矩阵吓住，我们只优化一个关节增量 $d$ 和一个松弛量 $s$：

$$
F(d,s)=\frac12(d-s-0.02)^2+2s^2,\qquad d\le0.01.
$$

令 $x=[d,s]^T$，展开得到：

$$
P=\begin{bmatrix}1&-1\\-1&5\end{bmatrix},\qquad
q=\begin{bmatrix}-0.02\\0.02\end{bmatrix}.
$$

无约束最优点是 $d=0.02,s=0$。但这违反了 $d\le0.01$。

约束生效后：

$$
\boxed{d^*=0.01,\qquad s^*=-0.002}.
$$

检验：代回 $r=d-s-0.02$，有

$$
r=0.01+0.002-0.02=-0.008.
$$

这说明：**硬约束满足了，但软任务允许有残差**。这正是项目中 QP + slack 的意义。

本算例使用 Python 验算了梯度与 P 的正定性；不是 S622 的实验测量。

---

# 10. 对照实际源码阅读的路线

| 阅读顺序 | 源码 | 重点问题 |
|---|---|---|
| 1 | `qp_types.hpp` | 为什么 `QpProblem` 不需要知道机器人信息？ |
| 2 | `admm_solver.cpp:35–113` | x、z、y 三次更新是怎样构成一轮迭代的？ |
| 3 | `osqp_solver.cpp:15–70` | 稀疏 CSC 怎样由 Eigen 数据构造？ |
| 4 | `osqp_solver.cpp:76–143` | `setup → solve → cleanup` 和 warm start 在哪里？ |
| 5 | `dual_arm_qp.cpp` | `last_problem_` 怎样送入 solver，并将解切成 dq/slack？ |
| 6 | `dual_arm_qp_controller` | 为什么不能不检查求解状态就发送 dq？ |

建议第一次只看流程，第二次才研究指针、稀疏数组和 Eigen 线性代数具体实现。那些函数参数可以另建伴读手册，本课不和它们混写。

---

# 11. 学完后的练习题

1. 为什么 $B^TB$ 总是半正定？什么时候不是正定？
2. KKT 中“可行”与“最优”有什么不同？
3. ADMM 的辅助变量 z 是什么？为什么要 `clamp` 它？
4. $\rho$ 越大是不是必然越好？
5. OSQP 为什么需要 CSC？
6. 为什么上一周期 x 的 warm start 不等于跨周期复用 factorization？
7. `OSQP_SOLVED_INACCURATE` 可以被接受，但为什么不能直接等于机器人轨迹精度达标？
8. 一个 QP 只有少数几次迭代，却仍然超时，你会检查哪些非迭代开销？
9. `QpProblem::valid()` 通过后，问题是否一定有解？
10. 单臂 ServoJ stall 时，即使 QP 收敛为什么也必须协调处理两臂？

### 参考答案要点

1. $v^TB^TBv=\|Bv\|^2\ge0$，若 B 有非零零空间则可能只有半正定。
2. 可行只保证合法，最优要求合法解中代价最低。
3. z 是 Ax 的辅助副本，把投影到约束集变成容易的区间裁剪。
4. 不一定，会改变 primal 与 dual 的收敛平衡。
5. 利用稀疏结构节省存储和数值计算。
6. 初值和因子分解/内部 workspace 是不同资源。
7. 收敛标志只描述后端接受的数学精度。
8. CSC 转换、内存分配、setup、cleanup 和控制器调度。
9. 不一定，几条约束可能互相矛盾。
10. 数学求解不等于双臂执行同步，更不保证闭链安全。

---

# 12. 本课验收标准与下一课

能够独立做到以下四件事，就算真正掌握了第四课：

- 解释 `P,q,A,l,u` 各是什么；
- 用一维或二维例子解释 ADMM 更新和约束投影；
- 在实际 `osqp_solver.cpp` 中指出 CSC、warm start、setup、solve 与 cleanup；
- 区分数学收敛、约束可行、执行同步和真实物理安全。

**第五课预告：**`dual_arm_qp_controller` 的 ROS 2 控制器生命周期与执行链路，包括 `on_init()`、`on_configure()`、`on_activate()`、`update()`、position command interface、真实控制周期与 D7 双臂 coordinated hold。

> 本文引用的源码事实均来自上述 GitHub 文件。没有声称完成本机 ROS 2 构建或 Gazebo 测试。

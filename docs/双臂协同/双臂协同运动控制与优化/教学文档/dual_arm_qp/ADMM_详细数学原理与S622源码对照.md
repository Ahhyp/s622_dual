# ADMM 详细数学原理：从增广拉格朗日到 S622 双臂 QP 求解器

> **第四课专题补充 · 数学理论讲义**
>
> 本文专门讲 **ADMM（Alternating Direction Method of Multipliers，交替方向乘子法）** 的数学原理，按「为什么需要 → 约束分裂 → 增广拉格朗日 → 三步推导 → KKT/残差 → 数值例子 → 项目实现」展开。
>
> **源码依据：**仓库 `Ahhyp/s622_dual` 的 `public` 分支：
> - [`admm_solver.cpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/admm_solver.cpp)
> - [`qp_types.hpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp)
>
> **阅读约定：**第 1～9 节是通用 ADMM 理论和教学推导；第 10～12 节专门核对项目源码。两者不相同时会单独说明，不把教学公式冒充为原文件中的实现。下面所有测试数值是独立教学算例，并非 S622 真机或 Gazebo 的测量值。

---

## 0. 先认识三个层次

| 层次 | 研究的问题 | 关键词 |
|---|---|---|
| 机器人任务建模 | 让左右手相对误差小、关节不超限 | Jacobian、QP |
| 数学优化 | 怎样在约束下找到最优增量 | 凸性、KKT、增广拉格朗日 |
| 数值求解 | 每一次迭代实际怎样计算 | ADMM、LLT、残差、warm start |

本讲重点是后两层。

**核心目标：**读懂项目里这三步为什么合理：

$$
\boxed{
\begin{aligned}
x^{k+1}&=(P+\sigma I+\rho A^\top A)^{-1}
[\sigma x^k-q+\rho A^\top(z^k-y^k)]\\
z^{k+1}&=\Pi_{[l,u]}(Ax^{k+1}+y^k)\\
y^{k+1}&=y^k+Ax^{k+1}-z^{k+1}
\end{aligned}}
$$

其中上式的 $y$ 是**缩放的对偶变量**，不是原始拉格朗日乘子。

---

# 1. 先从无约束最小化理解为什么需要算法

考虑：

$$
\min_x f(x)=\frac12(x-2)^2.
$$

一阶导数：

$$
f'(x)=x-2.
$$

令导数为零，得到 $x^*=2$。

如果再要求：

$$
0\le x\le1,
$$

答案就不能是 2；约束后的最优解是 $x^*=1$。

这揭示两类要求：

- **目标：**让代价越低越好。
- **约束：**无论目标如何，都只能选合法的 $x$。

机器人控制也是一样：QP 想尽量纠正双臂误差，但是关节角度、速度等硬约束不能任意违反。

在多变量、非单位矩阵的线性约束下，不能把“无约束解”简单逐元素夹到界内就指望得到正确最优解。ADMM 是处理这种结构的一种方法。

---

# 2. 原始 QP 写法

项目的 `QpProblem` 采用：

$$
\boxed{
\begin{aligned}
\min_{x\in\mathbb R^n}\quad
& f(x)=\frac12 x^\top P x+q^\top x\\
\text{s.t.}\quad & l\le Ax\le u
\end{aligned}}
\tag{1}
$$

- $x\in\mathbb R^n$：决策变量。双臂 QP 里包括关节增量和任务松弛量。
- $P\in\mathbb R^{n\times n}$：通常对称半正定。
- $q\in\mathbb R^n$：线性代价项。**注意它不是 12 关节角 `q_state`。**
- $A\in\mathbb R^{m\times n}$：约束矩阵。
- $l,u\in\mathbb R^m$：约束上下界，允许用无穷值表示单侧无界。

为什么这一类问题适合 ADMM？因为**代价是容易求导的二次函数，而约束是容易投影的区间**。

---

# 3. 第一步：变量分裂（Variable Splitting）

ADMM 的关键不在“给公式加很多符号”，而是引入一个额外变量，把两类困难分开。

定义：

$$
z=Ax.
$$

原问题等价改写为：

$$
\begin{aligned}
\min_{x,z}\quad& f(x)+I_C(z)\\
\text{s.t.}\quad& Ax-z=0,
\end{aligned}
\tag{2}
$$

其中 $C=[l,u]$，$I_C$ 是集合的**指示函数**：

$$
I_C(z)=
\begin{cases}
0,&z\in C,\\
+\infty,&z\notin C.
\end{cases}
$$

**为什么这样写？**

- $x$ 专注于二次目标 $f(x)$；
- $z$ 专注于能不能放在约束区间；
- 等式 $Ax-z=0$ 保证二者最终描述同一个合法解。

这里的 $z$ 是辅助的约束变量，**不是第二课里表示所有 QP 决策变量的那个 `z`**。数学资料使用字母有差异，千万按当前章节定义理解。

---

# 4. 第二步：从普通拉格朗日到增广拉格朗日

## 4.1 普通拉格朗日

对于等式 $Ax-z=0$，引入乘子 $\lambda\in\mathbb R^m$：

$$
L(x,z,\lambda)=f(x)+I_C(z)+\lambda^\top(Ax-z).
$$

单靠这个表达式，数值迭代有时不够稳定。

## 4.2 增广拉格朗日

在等式残差上加二次惩罚：

> ? 为什么加上这个惩罚项， 有啥用？ 真的就能够稳定数值迭代吗？而且为什么是二次？只是为了好求导么？

$$
\boxed{
L_\rho(x,z,\lambda)=
f(x)+I_C(z)+\lambda^\top(Ax-z)
+\frac{\rho}{2}\|Ax-z\|_2^2
}
\tag{3}
$$

其中 $\rho>0$ 是惩罚参数。

理解三个部分：

1. $f(x)$：希望优化的任务目标。
2. $I_C(z)$：确保 z 合法。
3. $\lambda^\top(Ax-z)+\frac\rho2\|Ax-z\|^2$：推动 x、z 一致。

## 4.3 为什么可以“配平方”？

令**缩放对偶变量**：

$$
y=\lambda/\rho \quad \Leftrightarrow\quad \lambda=\rho y.
$$

利用恒等式：

$$
\lambda^\top r+\frac{\rho}{2}\|r\|^2
=\frac{\rho}{2}\|r+y\|^2-\frac{\rho}{2}\|y\|^2,
\quad r=Ax-z.
$$

所以：

$$
L_\rho=
f(x)+I_C(z)
+\frac{\rho}{2}\|Ax-z+y\|^2
-\frac{\rho}{2}\|y\|^2.
\tag{4}
$$

最后一项在 x、z 的子步骤里可视为常数。

**关键：**代码里的 `y` 不是原始乘子 $\lambda$，而是 $\lambda/\rho$。

---

# 5. 第三步：标准 ADMM 的三次更新如何推导？

先讲不带 $\sigma$ 的标准形式，之后再加上源码中的近端项。

## 5.1 x-update：固定 z、y，求最优 x

$$
x^{k+1}
=\arg\min_x
f(x)+\frac{\rho}{2}\|Ax-z^k+y^k\|^2.
\tag{5}
$$

对 x 求梯度：

$$
\nabla f(x)=Px+q,
$$

$$
\nabla_x \frac{\rho}{2}\|Ax-z^k+y^k\|^2
=\rho A^\top(Ax-z^k+y^k).
$$

令梯度为零：

$$
Px+q+\rho A^\top(Ax-z^k+y^k)=0.
$$

移项：

$$
(P+\rho A^\top A)x
=-q+\rho A^\top(z^k-y^k).
$$

因此：

$$
\boxed{x^{k+1}=
(P+\rho A^\top A)^{-1}
[-q+\rho A^\top(z^k-y^k)]}.
\tag{6}
$$

这里的“逆”是数学记号；实际数值代码通常**通过矩阵分解求解线性方程，而不是显式计算逆矩阵**。

## 5.2 z-update：固定新 x 与旧 y，只需投影

$$
z^{k+1}
=\arg\min_{z\in C}
\frac{\rho}{2}\|Ax^{k+1}-z+y^k\|^2.
$$

> ? $I_C(z) + \lambda ^T (Ax - z)$ 去哪了？


忽略正的比例常数，其实是在求：

$$
\arg\min_{z\in C}\|z-(Ax^{k+1}+y^k)\|^2.
$$

这正是把向量投影到集合 C： TODO: 这句话是啥意思？？？

$$
\boxed{
z^{k+1}=\Pi_C(Ax^{k+1}+y^k)
=\operatorname{clip}(Ax^{k+1}+y^k,l,u)}.
\tag{7}
$$

对每个分量：

$$
z_i=\min(u_i,\max(l_i,(Ax+y)_i)).
$$

例如约束为 $0\le z\le1$，临时值为 1.3，那么 z 更新得到 1。

## 5.3 y-update：修正约束不一致

原始乘子的更新是：

$$
\lambda^{k+1}
=\lambda^k+\rho(Ax^{k+1}-z^{k+1}).
$$

因为 $y=\lambda/\rho$，当 $\rho$ 固定：

$$
\boxed{y^{k+1}=y^k+Ax^{k+1}-z^{k+1}}.
\tag{8}
$$

**人话：**如果 $Ax$ 超过了 z，下一轮对偶变量就加上这个误差，让迭代倾向于修正偏差。

三步合起来就是 ADMM。

---

# 6. 源码为什么多了 sigma？

你的 S622 参考 ADMM 的 x-update 不是 (6)，而是：

$$
\boxed{
x^{k+1}=
(P+\sigma I+\rho A^\top A)^{-1}
[\sigma x^k-q+\rho A^\top(z^k-y^k)]
}.
\tag{9}
$$

这意味着每轮 x 子问题额外加入：

$$
\frac{\sigma}{2}\|x-x^k\|^2.
\tag{10}
$$

它叫**近端项（proximal term）**。

我们再推导一次：

$$
\min_x
\frac12 x^\top Px+q^\top x
+\frac{\rho}{2}\|Ax-z^k+y^k\|^2
+\frac{\sigma}{2}\|x-x^k\|^2.
$$

梯度为：

$$
Px+q+\rho A^\top(Ax-z^k+y^k)+\sigma(x-x^k)=0.
$$

整理：

$$
(P+\sigma I+\rho A^\top A)x
=\sigma x^k-q+\rho A^\top(z^k-y^k).
$$

这就**严格推导出了代码的 x 更新式**。

它有两个好处：

- 在 $P$ 只有半正定的情况下改善线性系统的数值性质；
- 让新一轮的 x 不至于完全忽略上次解。

但 $\sigma$ 不是越大越好；过大可能导致迭代变化很慢。

---

# 7. 为什么是 LLT，而不是直接求逆？

代码需要求解：

$$
Kx=b,\qquad
K=P+\sigma I+\rho A^\top A.
\tag{11}
$$

若 $P\succeq0$、$\sigma>0$、$\rho>0$，则任意非零向量 $v$：

$$
v^\top Kv=v^\top Pv+\sigma\|v\|^2+\rho\|Av\|^2>0.
$$

因此 K 是**对称正定矩阵**，适合 Cholesky 分解：

$$
K=LL^\top.
$$

随后求解：

$$
Lw=b,\qquad L^\top x=w.
$$

Eigen 的 `LLT` 就是这种思路。它避免构造显式的 $K^{-1}$，在适用条件下更稳定。

**区别：**

- 数学纸面：写 $K^{-1}b$ 方便推导。
- 数值实现：调用 `llt.solve(b)`。

---

# 8. KKT 和“收敛”到底意味着什么？

ADMM 不是只看 x 的变化是否很小。

## 8.1 原始可行性（primal feasibility）

理论要求：

$$
Ax=z,\qquad z\in C.
$$

所以定义 primal residual：

$$
\boxed{r_{\rm pri}^{k+1}=Ax^{k+1}-z^{k+1}}.
$$

如果它接近零，说明约束映射与合法区间变量已基本一致。

## 8.2 对偶残差（dual residual）

常见缩放 ADMM 定义：

$$
\boxed{r_{\rm dual}^{k+1}
=\rho A^\top(z^{k+1}-z^k)}.
$$

符号差异不会影响范数。它衡量由于 z 改变所产生的驻点偏差（带 $\sigma$ 近端项时，精确驻点关系中还包含 $\sigma(x^{k+1}-x^k)$，所以只看该表达式是工程化的简化检查）。

源代码使用的是：

```cpp
primal_res = (Ax - z).lpNorm<Eigen::Infinity>();
dual_res = rho * (At * (z - z_prev)).lpNorm<Eigen::Infinity>();
```

无穷范数：

$$
\|v\|_\infty=\max_i|v_i|.
$$

即取各分量中绝对值最大的那个。

## 8.3 为什么需要 eps_abs 和 eps_rel？

数值优化很少要求残差严格为零。

项目定义：

$$
\epsilon_{\rm pri}=
\epsilon_{\rm abs}+
\epsilon_{\rm rel}\max(\|Ax\|_\infty,\|z\|_\infty).
$$

$$
\epsilon_{\rm dual}=
\epsilon_{\rm abs}+
\epsilon_{\rm rel}\|A^\top y\|_\infty.
$$

在 `primal_res` 与 `dual_res` 分别低于这两个门槛时，代码置 `converged=true`。

**精确性提醒：**代码里的 `y` 是缩放乘子，严格的未缩放拉格朗日乘子为 $\lambda=\rho y$。因此如果想与其他库使用未缩放乘子的停止判据对比，不能不经调整直接对比数值。

primal_res < eps_pri 这个判据不是随手写的，它是 Boyd ADMM 论文（2011, §3.3.1）标准停机准则 的 OSQP 风格实现。公式来源就是那篇论文，代码只是把 2-范数换成了 ∞-范数、把约束 Ax - z = 0 的常数项 c = 0 省掉了。

## 8.4 它不保证什么？

`converged=true` 不等于：

- 机器人不会碰撞；
- 末端一定跟上关节命令；
- 抓住的物体不会滑落；
- ServoJ 双臂通讯没有 stall。

它仅表示本次**优化器使用的数值停止准则**满足要求。

---

# 9. 参数 rho 与 warm start

## 9.1 rho 的直觉

$\rho$ 大：更强烈地要求 $Ax$ 接近 z；  
$\rho$ 小：相对更加关注 x 子问题的二次目标。

但它并非越大越好，往往需要平衡两种残差的收敛。

本项目在每隔若干次迭代时，对比：

$$
r_{\rm pri}>10r_{\rm dual}
\quad\Longrightarrow\quad \rho\leftarrow2\rho,
$$

$$
r_{\rm dual}>10r_{\rm pri}
\quad\Longrightarrow\quad \rho\leftarrow\rho/2.
$$

并将它限制在配置的上下界内。

**值得留意的代码细节：**它调整了 $\rho$ 但没有重新缩放现有 `y`。由于 `y=\lambda/\rho`，如果希望保持同一个未缩放乘子 $\lambda$，改变 $\rho$ 后需要相应处理 `y`。这是源码和严谨理论之间需要进一步评测的实现选择；这里不声称它必然导致错误，只要求理解其含义。

## 9.2 warm start

相邻控制周期 QP 通常相近，复用旧解作为下次 x 初值：

$$
x^0_{\rm new}=x^*_{\rm previous}.
$$

但**仅 warm start x** 不等于同时复用：

- 约束辅助变量 z；
- 对偶变量 y；
- Cholesky 分解；
- OSQP workspace。

本项目的 `AdmmSolver::solve()` 每次创建新的本地 x、z、y，并只从 `problem.x0` 初始化 x。

---

# 10. 对照你的 `admm_solver.cpp` 逐阶段理解

下面的表是**本次读取源码的直接对应关系**：

| 源码行段 | 内容 | 数学上的含义 |
|---|---|---|
| 23–32 | `QpProblem::valid()` | 仅检查矩阵尺寸及每行边界次序等 |
| 35–46 | `solve()` 初始化 x、z、y | 准备优化变量及 warm start |
| 48–57 | 构造 K / LLT | $K=P+\sigma I+\rho A^\top A$ |
| 59–62 | LLT 失败分支 | 注释称增加正则，实际重复相同分解 |
| 69–74 | x 更新 | 解带近端项的二次最小化 |
| 75–77 | z、y 更新 | 投影及缩放乘子调整 |
| 79–85 | primal、dual 残差 | 收敛状态诊断 |
| 87–90 | 停止判据 | 两类残差均满足门槛 |
| 91–103 | adaptive rho | 按残差不平衡调整 $\rho$ 并重新分解 |
| 105–113 | 输出结果 | 变量 x、收敛状态、迭代次数、耗时 |

注意三个具体实现边界：

**第一，**`valid()` 主要是形式校验，不证明 QP 整体可行，也不完整检查 P 是否半正定或含 NaN。

**第二，**当 LLT 失败时，当前代码只是对相同 K 重复分解，不存在实际新增的正则化步骤。这是值得修正和测试的工程问题。

**第三，**这里的实现是相对简洁的 **ADMM reference backend**，不能把它的所有工程细节等同于成熟 OSQP 库内部的完整算法。

---

# 11. 一个逐轮计算的例子：你会真正看懂 x、z、y

考虑：

$$
\min_x\frac12(x-2)^2,\qquad 0\le x\le1.
$$

展开为：

$$
P=1,\quad q=-2,\quad A=1,\quad l=0,\quad u=1.
$$

我们选择教学参数：

$$
\rho=1,\quad\sigma=0.1,\quad
x^0=z^0=y^0=0.
$$

迭代：

$$
x^{k+1}=
\frac{0.1x^k+2+(z^k-y^k)}{2.1},
$$

$$
z^{k+1}=\operatorname{clip}(x^{k+1}+y^k,0,1),
$$

$$
y^{k+1}=y^k+x^{k+1}-z^{k+1}.
$$

下面数值经 Python 逐步计算验证：

| 迭代 | x | z | y | primal residual |
|---:|---:|---:|---:|---:|
| 1 | 0.952381 | 0.952381 | 0 | 0 |
| 2 | 1.451247 | 1.000000 | 0.451247 | 0.451247 |
| 3 | 1.282799 | 1.000000 | 0.734046 | 0.282799 |
| 5 | 1.066597 | 1.000000 | 0.940754 | 0.066597 |
| 10 | 1.001534 | 1.000000 | 0.998639 | 0.001534 |
| 20 | 1.000001 | 1.000000 | 0.999999 | 0.000000809 |

读这个表，注意：

- 第 2 轮的 **x=1.451247 超出约束**，这是允许的**中间迭代状态**；
- 第 2 轮的 **z=1**，始终满足 `[0,1]`；
- 随着迭代，x 与 z 越来越接近；
- 极限解为 $x^*=1,z^*=1,y^*=1$。

原始乘子 $\lambda=\rho y$，所以最优时 $\lambda^*=1$。

直接检查 KKT：

$$
Px^*+q+A^\top\lambda^*
=1-2+1=0.
$$

这说明收敛点既合法、又满足驻点条件。

**非常重要：**`z` 始终被裁剪到界内，**不代表每一步的 x 都合法**。在实际控制器里，应仅使用满足验收条件的最终解，并由上层执行安全监控处理失败或超时。

---

# 12. 为什么 ADMM 不能保证每轮都立刻是合法控制命令？

ADMM 允许中间 x 暂时违背约束，再通过 z 与 y 逐渐把它拉回来。

因此：

- **未收敛中间结果不能直接当作满足硬约束的安全指令。**
- “迭代次数达到上限”不代表“已经无解”，也可能只是没有收敛足够快。
- “收敛”也不等于机器人执行器物理层面的安全。

这与 S622 双臂尤其相关：即使优化器返回理想的 12 轴关节增量，真实两臂仍可能出现位置跟踪滞后或 ServoJ stall，必须另有 coordinated hold 机制。

---

# 13. ADMM 与 OSQP：为什么仓库里两者都有？

ADMM 是一类数学方法，OSQP 是专门面向凸 QP 的高质量算子分裂求解器。

仓库通过 `QpSolver` 抽象后端，可以在构建时选择：

- 启用 OSQP：`OsqpSolver`；
- 没有 OSQP：内置 `AdmmSolver`。

不要把 `admm_solver.cpp` 理解成“OSQP 的源码”。它们是不同实现，只是数学路线相关。

**学习建议：**先亲自推导本讲的 x/z/y 三步，再学习 OSQP 的稀疏 KKT 系统、预分解与更新机制。

---

# 14. 数学边界和常见误区

### 误区 1：y 是机械臂的速度？

不是。`y` 是优化中的缩放对偶变量，用于纠正 $Ax=z$ 的不一致。

### 误区 2：z 等于最终关节位置？

不是。在本讲 ADMM 符号里，$z$ 是 $Ax$ 的辅助约束副本。具体双臂 QP 的决策向量可以另用 `x` 或其他符号，不要混淆。

### 误区 3：rho 越大越好？

不是。它影响收敛平衡和线性系统条件；要由残差、总耗时和可靠性评测决定。

### 误区 4：P 正定就不可能求解失败？

不对。硬约束可能互相矛盾；数值实现也可能超时或遇到非法数据。

### 误区 5：Primal residual 小就一定最优？

不对。还要考虑对偶条件、驻点、可行性和求解器接受标准。

### 误区 6：ADMM 中 x 越界就是算法 bug？

不一定。中间 x 是未强制投影的候选值，z 才是区间上的投影点。最终 x 与 z 应趋于一致。

### 误区 7：加了 sigma 就等于完全解决数值问题？

不对。它只是增加近端正则，不能修复所有病态输入、不可行性或代码实现错误。

---

# 15. 推荐自己动手推导的三道练习

## 练习 A：推导 x 更新

从：

$$
\frac12x^\top Px+q^\top x+
\frac{\rho}{2}\|Ax-z+y\|^2+
\frac{\sigma}{2}\|x-x^k\|^2
$$

对 x 求导，证明：

$$
(P+\sigma I+\rho A^\top A)x
=\sigma x^k-q+\rho A^\top(z-y).
$$

## 练习 B：证明 z 更新是投影

求：

$$
\arg\min_{l\le z\le u}\|z-(Ax+y)\|^2.
$$

先写标量情况，证明每个分量恰好是 `clamp(value,lower,upper)`。

## 练习 C：自行计算前三轮

设 $P=1,q=-2,A=1,l=0,u=1,\rho=1,\sigma=0.1$。

从 $x^0=z^0=y^0=0$ 出发，手算前三轮，检查是否得到本讲第 11 节的结果。

---

# 16. 本专题学习验收标准

学完后，应该能做到：

1. 不看代码，自己写出 QP 原问题、变量分裂、增广拉格朗日函数。
2. 独立求导，推出 `x-update` 的线性方程。
3. 解释 `z-update` 为什么就是投影 `clamp`。
4. 从拉格朗日乘子的梯度更新推出缩放的 `y-update`。
5. 区分 $r_{\rm pri}$、$r_{\rm dual}$、$\rho$、$\sigma$、warm start。
6. 找到 `admm_solver.cpp` 中每个数学式的代码位置。
7. 理解为什么一个最终解“算法收敛”仍不能替代机械臂的安全监控。

**一句话总结：ADMM 通过把“优化目标”和“满足约束”拆成两个较容易的子问题，并利用对偶变量不断协调二者，逐渐逼近满足 KKT 条件的最优解。**

---

## 参考资料与来源范围

**本项目源码（已核对）：**

- https://raw.githubusercontent.com/Ahhyp/s622_dual/public/src/dual_arm_qp_core/src/admm_solver.cpp
- https://raw.githubusercontent.com/Ahhyp/s622_dual/public/src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp

**外部数学知识（本讲理论部分）：**

- Boyd 等，*Distributed Optimization and Statistical Learning via the Alternating Direction Method of Multipliers*（2011）：ADMM 的经典教材式推导。
- Stellato 等，*OSQP: An Operator Splitting Solver for Quadratic Programs*：OSQP 与 QP 算子分裂求解器的数学背景。

> 本讲不要求立即修改任何生产求解器代码。建议学完后再为当前 ADMM 后端制定独立数值测试，重点检查收敛、可行性、adaptive rho 和分解失败路径。

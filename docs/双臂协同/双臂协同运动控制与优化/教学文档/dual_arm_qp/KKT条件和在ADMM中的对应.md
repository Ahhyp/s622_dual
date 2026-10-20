# KKT 条件

KKT（Karush–Kuhn–Tucker）条件是非线性/凸优化中**局部最优解必须满足的一阶必要条件**（在凸问题且满足约束品性时也是**充分条件**）。核心思想是：在最优点处，目标函数的梯度必须是所有起作用约束梯度的非负线性组合。

---

## 一般形式

考虑

$$
\min_x f(x)\quad \text{s.t.}\quad g_i(x)\le 0,\; h_j(x)=0
$$

构造拉格朗日函数

$$
L(x,\lambda,\mu)=f(x)+\sum_i \lambda_i g_i(x)+\sum_j \mu_j h_j(x)
$$

KKT 条件由四部分组成：

| 条件 | 表达式 | 含义 |
|---|---|---|
| ① 原始可行性 | $g_i(x)\le 0,\; h_j(x)=0$ | 解必须可行 |
| ② 对偶可行性 | $\lambda_i \ge 0$ | 不等式乘子非负 |
| ③ 平稳性 | $\nabla f(x)+\sum_i\lambda_i\nabla g_i(x)+\sum_j\mu_j\nabla h_j(x)=0$ | 梯度被"约束力"平衡 |
| ④ 互补松弛 | $\lambda_i\, g_i(x)=0$ | 约束不起作用($g_i<0$)时乘子为 0；乘子>0 时约束必须取等号 |

其中 ②③④ 是核心。互补松弛意味着：**只有"卡住"解的约束才产生力**。

---

## 对应你这个 QP

$$
\min_x \tfrac12 x^\top P x + q^\top x \quad\text{s.t.}\quad l \le Ax \le u
$$

拆成双边不等式 $l - Ax \le 0$、$Ax - u \le 0$，乘子记为 $\lambda^+,\lambda^-\ge 0$：

$$
L = \tfrac12 x^\top Px + q^\top x + (\lambda^+)^\top(l-Ax) + (\lambda^-)^\top(Ax-u)
$$

令 $\lambda = \lambda^+ - \lambda^-$（即"净"乘子），KKT 变为：

**平稳性**
$$
Px + q - A^\top\lambda = 0 \;\Longleftrightarrow\; Px + q = A^\top\lambda
$$

**原始可行性**：$l \le Ax \le u$

**互补松弛**（用投影形式最直观，记 $z=Ax$）：

$$
\begin{cases}
z_i \in (l_i,u_i) &\Rightarrow \lambda_i = 0\\
z_i = l_i &\Rightarrow \lambda_i \ge 0 \quad(\text{下界在推})\\
z_i = u_i &\Rightarrow \lambda_i \le 0 \quad(\text{上界在推})
\end{cases}
$$

这正是"$\lambda$ 属于区间 $[l,u]$ 在 $z$ 处的法锥"这一条件。

> 对凸 QP（$P\succeq 0$）且可行域非空，KKT 是**充要条件**——满足 KKT 的点就是全局最优解。

---

## 和这段 ADMM 代码的对应

ADMM 迭代的每一个式子，都是在**逐步逼近 KKT 的各个条件**：

**① x-更新**：解 $(P+\sigma I+\rho A^\top A)x = \sigma x_{\text{old}} - q + \rho A^\top(z-y)$。
收敛时 $x_{\text{old}}\to x$，方程退化为
$$
Px + q + \rho A^\top(Ax - z + y)=0
$$
结合 $Ax=z$，得 $Px+q+\rho A^\top y = 0$。这就是**平稳性**，其中对偶乘子 $\lambda=\rho y$（$y$ 是 scaled dual）。

**② z-更新**：$z=\mathrm{clamp}(Ax+y,\,l,\,u)$。收敛时 $Ax=z$，故 $z=\mathrm{clamp}(z+y)$，等价于 $-y$ 属于 $[l,u]$ 在 $z$ 处的法锥——这就是**互补松弛 + 对偶可行性**（$y_i$ 的符号自动满足 $\lambda_i$ 的符号要求）。

**③ y-更新**：$y \mathrel{+}= Ax-z$。这是对偶上升，用来消掉原始残差。

**④ 收敛判据正是 KKT 残差**：

| 代码变量 | KKT 含义 |
|---|---|
| `primal_res = ‖Ax − z‖∞` | 原始可行性 $l\le Ax\le u$ 的违反量 |
| `dual_res = ρ‖Aᵀ(z − z_prev)‖∞` | 平稳性 $Px+q+A^\top\lambda=0$ 的违反量 |

所以代码里

```cpp
if (primal_res <= eps_pri && dual_res <= eps_dual) converged = true;
```

本质上是在检查：**原始可行性和平稳性两个 KKT 条件是否都已满足到容差以内**。一旦两者同时满足，互补松弛/对偶可行由 z-更新自动保证，于是 KKT 全部成立，解就是最优的。

---

## 补充：为什么 ADMM 能保证收敛到 KKT 点

ADMM 交替最小化增广拉格朗日函数，理论上（凸情形）原始残差和对偶残差都趋于 0，因此极限点满足 KKT。代码里自适应 ρ 的作用只是平衡这两个残差的下降速度（原始残差大就增大 ρ，对偶残差大就减小 ρ），让两者**同时**进入容差，而不是一个早早收敛、另一个迟迟不收敛——这也是标准 OSQP 的启发式。
# 第二课：`dual_arm_qp.cpp` 函数参数与数学原理

> 建议仓库路径：`src/dual_arm_qp_core/src/第二课_dual_arm_qp_函数参数与数学原理.md`  
> 基准：[`dual_arm_qp.cpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/dual_arm_qp.cpp)、[`dual_arm_qp.hpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/dual_arm_qp.hpp) 与 [`qp_types.hpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp)，GitHub `public` 分支，2026-10-16。
>
> **阅读规则：**以源码中实际使用的公式为准；特别注意 `dual_arm_qp.hpp` 顶部的概括性代价注释与 `.cpp` 的 `bias_rel` 符号表达并不完全一致。`relativeError()` 的严格符号、平移旋转分量次序、SE(3) 微分约定，最终必须追到 `dual_arm_kinematics` 实现才能完全确认；本课不凭空断言。

## 学习目标与建议顺序

这份讲义专门补全上一课漏掉的**函数传参**，并从基础数学逐步推导源码。按以下顺序：

1. 学会读 C++ 函数签名和所有关键形参。
2. 认识旋转矩阵、刚体变换 `Iso3`、坐标系的方向。
3. 理解 FK、Jacobian、相对误差和双臂约束。
4. 从最小二乘推导二次规划 `P/q/A/l/u` 与 slack。
5. 解释 SVD、零空间、阻尼、有限差分、平滑退化。
6. 用具体数值手算一次迷你 QP，并反向联系源码。
7. 根据源码检验已实现功能和未实现/未接线功能。

---

# 第一部分：逐函数参数字典（覆盖本 `.cpp` 的核心函数）

## 1. `smoothstep01(double x)`（约 L14）

| 项目 | 含义 |
|---|---|
| `x` | 输入的归一化数值，不要求调用方保证 0～1 |
| `t`（局部） | `clamp(x,0,1)` |
| 返回 `double` | `t²(3−2t)`，处于 0～1，且端点斜率为零 |

用途：把“条件正常程度”平滑变成倍率，而不是突然切换控制模式。

## 2. `sigmaMinRel(const DualArmKinematics& kin, const Q12& q)`（约 L19）

| 形参 | C++ 类型 | 意思 |
|---|---|---|
| `kin` | `const DualArmKinematics&` | 当前机器人运动学模型的只读引用 |
| `q` | `const Q12&` | 12 个当前关节位置，单位 rad |
| 返回 | `double` | 相对任务 6×12 Jacobian 的最小奇异值 |

流程：`kin.relativeJacobian(q, BodyR)` → SVD → 最小奇异值。不是夹爪接触力指标。

## 3. `clearanceProxy(const DualArmKinematics& kin, const Q12& q)`（约 L26）

形参与上面相同。返回一个**距离代理数值（米）**：枚举左右两臂各 4 个关键连杆坐标原点之间的距离，取最小值。只是次级舒适性指标，不是几何碰撞距离或安全保证。

## 4. `ascentDirection(const Q12& q, double h, Fn f)`（约 L43）

| 形参 | 含义 |
|---|---|
| `q` | 12 轴展开点：在什么姿势附近估算方向 |
| `h` | 单关节正负扰动步长，单位 rad |
| `f` | 可调用对象：输入一个 `Q12`，返回希望尽量增大的标量 |
| `Fn` | C++ **模板类型参数**，让 lambda 等可调用对象都能传进来 |
| 返回 `Vec12` | 12 个关节方向的数值梯度 |

公式：

$$
g_i \approx \frac{f(q+h e_i)-f(q-h e_i)}{2h}.
$$

调用例子（逻辑结构）：

```cpp
ascentDirection(q, 1e-3,
    [&](const Q12& qq) { return sigmaMinRel(kin_, qq); });
```

`qq` 是一次扰动后的候选关节角，不是正在执行的真实关节角。`[&]` 表示引用捕获外部的 `kin_` 等环境。

## 5. `marginDirection(const DualArmQpParams& p, const Q12& q)`（约 L54）

- `p`：包含 `q_min/q_max` 的控制参数。
- `q`：当前关节角。
- 返回：`Vec12`，每个元素表示更靠近关节活动范围中点的方向。

$$
m_i=(q_{\min,i}+q_{\max,i})/2,\quad
g_i=-2(q_i-m_i)/(q_{\max,i}-q_{\min,i})^2.
$$

不是求解得到的 `dq`，只是推荐方向。

## 6. `defaultS622Limits(Vec12* q_min, Vec12* q_max, Vec12* v_max)`（约 L65）

这三个都是**输出指针**：函数通过 `*q_min` 等修改调用方传进来的向量。

- `q_min`：输出 12 轴下限，rad。
- `q_max`：输出 12 轴上限，rad。
- `v_max`：输出 12 轴速度上限，rad/s。
- `a*6+i`：两臂各 6 轴的索引映射。
- 返回 `void`：不单独返回值，结果写入指针所指对象。

## 7. `DualArmQp::DualArmQp(kinematics, params, solver)`（约 L79）

| 参数 | 类型 | 生命周期/职责 |
|---|---|---|
| `kinematics` | `const DualArmKinematics&` | 已存在的运动学对象，构造后由 `kin_` 保存引用，必须比 QP 对象活得久 |
| `params` | `const DualArmQpParams&` | 参数配置，会复制到 `params_` |
| `solver` | `QpSolverPtr` | 可选求解器，默认 `nullptr` 时由 `makeDefaultSolver()` 创建 |

构造函数的 `: kin_(kinematics), params_(params), ...` 叫**成员初始化列表**，不需要出现赋值符号就能初始化类成员。

## 8. 三个 `solve()` 重载（约 L83～L92）：最重要！

```cpp
DualArmQpResult solve(
    const Q12& q,
    const Iso3& left_T_right_target);

DualArmQpResult solve(
    const Q12& q,
    const Iso3& left_T_right_target,
    const Iso3& world_T_object_target);

DualArmQpResult solve(
    const Q12& q,
    const Iso3& left_T_right_target,
    const Iso3& world_T_object_target,
    const Vec6& object_body_twist_ff);
```

| 形参 | 尺寸/类型 | 坐标系/单位 | 来自哪里 | 解决什么问题 |
|---|---|---|---|---|
| `q` | `Q12`，12×1 | 左臂 6 轴 + 右臂 6 轴，rad | 通常是控制器读到的关节状态 | “机器人现在什么姿态？” |
| `left_T_right_target` | `Iso3`，4×4 刚体变换 | 右抓取坐标系相对左抓取坐标系的**期望变换** | 任务设置或启动协同模式时记录 | “两只手应该保持怎样的相对位置与方向？” |
| `world_T_object_target` | `Iso3`，4×4 刚体变换 | 期望物体/协调坐标系在世界中的位姿 | 物体轨迹规划器或任务目标 | “箱子整体应该走到哪里？” |
| `object_body_twist_ff` | `Vec6`，6×1 | 物体系中的期望瞬时 twist，头文件标 `[v;w]`（m/s 和 rad/s） | 轨迹的速度前馈 | “目标本身正在往哪个方向、以多快速度运动？” |
| 返回 `DualArmQpResult` | struct | `dq`、slack、状态、计时等 | QP 运算结果 | 告诉执行层“下一步怎么动” |

### 函数参数 `left_T_right_target` 到底是什么？

把名字断开：`left` / `T` / `right` / `target`：

- `left_T_right`：描述**右抓取坐标系在左抓取坐标系中的位姿**；
- `_target`：这是期望值而非实时测量值。
- 它不是一个标量距离！除了右手相对左手在哪里，还包括**右手朝哪边**。

数学写法：

$$
{}^{L}T_R^{*} = ({}^{W}T_L^*)^{-1}{}^{W}T_R^*.
$$

实际当前相对关系：

$$
{}^LT_R(q)=({}^WT_L(q))^{-1}{}^WT_R(q).
$$

控制器要尽量使当前变换接近期望的 `left_T_right_target`。

**区别：**

- `left_T_right_target`：期望，通常在协同模式启动时固定；
- 当前 `left_T_right(q)`：随着关节状态变化而改变。
- 两者差的不是单个位置值，而是 SE(3) 中的位姿误差。

### 三个重载的默认行为

`solve(q, left_T_right_target)` 在当前实现中转给：

```cpp
solve(q, left_T_right_target, kin_.worldTobject(q))
```

三个参数版再用 `Vec6::Zero()` 补上零前馈。最后都进入四参数版。所以仅有相对任务输入的调用方不需要显式提供物体目标。

**关键现状：**即使 `w_obj=0` 关闭了物体**代价**，完整版 `solve()` 仍调用了旧物体误差和 Jacobian 相关计算。C4.3 里提到的 π 旋转中点风险与此有关。

---

# 第二部分：先学坐标变换，再看位姿误差

## 9. 为什么 `Iso3` 不是普通的三维坐标？

机械臂末端不光有位置 \((x,y,z)\)，还需要说明朝向。旋转矩阵 \(R\) 是 3×3，位置 \(p\) 是 3×1，将它们合并：

$$
T=\begin{bmatrix}R&p\\0&1\end{bmatrix}\in SE(3).
$$

`Eigen::Isometry3d`（本项目别名 `Iso3`）常用来存这类变换。

例如一个纯平移变换：

$$
{}^WT_L=\begin{bmatrix}
1&0&0&0.30\\
0&1&0&0.10\\
0&0&1&0.50\\
0&0&0&1
\end{bmatrix}.
$$

表示左手在世界坐标系中位置为 (0.30,0.10,0.50)，旋转为单位旋转。单位假定为米。

> 这里的数值只是教学示例，不是项目实测。

## 10. 为什么 `left_T_right` 要先求逆？

我们知道世界中的两只手：

$$
{}^WT_L,\quad {}^WT_R.
$$

若想让左手自己当参照观察右手，就必须“撤销”左手相对世界的变换：

$$
{}^LT_R=({}^WT_L)^{-1}{}^WT_R.
$$

假设两手都没有旋转，左手世界 x 坐标 0.2，右手 x 坐标 0.7，则右手相对于左手在 x=0.5 处。两手整体一起移 0.1 后，世界坐标改变，但**相对位姿不变**。

这就是刚性抓持中最重要的约束。

## 11. 物体位姿为什么不能总通过左右手“姿态平均”来定义？

C2 的旧 `worldTobject(q)` 使用协调/中点定义；C4.3 的双端镜像抓持可能使两手相对旋转达到 π（180°），测地中点可能不唯一。因此 C4.3 建议**用任务指定的物体坐标系**和固定抓取关系，而不是直接复用有歧义的旧物体 Jacobian。

若物体系为 O，左右抓取为 L、R，定义固定抓取变换：

$$
{}^OT_L=G_L,\qquad {}^OT_R=G_R.
$$

那么：

$$
{}^WT_L={}^WT_O G_L,\qquad
{}^WT_R={}^WT_O G_R.
$$

你也可以从任意一只手独立反推物体：

$$
{}^WT_{O,L}={}^WT_LG_L^{-1},
\quad
{}^WT_{O,R}={}^WT_RG_R^{-1}.
$$

理想刚性抓持时二者一致。**任务指定的 O 并不要求两臂中任何一臂不动**。

---

# 第三部分：为什么需要 Jacobian？

## 12. `J` 是“关节小动作 → 末端小动作”的线性近似

单臂位置/姿态是关节角的函数：

$$
T=f(q).
$$

在当前位形附近：

$$
\delta x\approx J(q)\delta q.
$$

速度形式：

$$
v\approx J(q)\dot q.
$$

`J_rel` 为 6×12，描述双臂关节运动对**相对位姿**的局部影响。直观一维类比：

$$
d=x_R-x_L\implies \dot d=\dot x_R-\dot x_L.
$$

但真实三维旋转中要统一 world/body 表达和参考点，不能直接把任意 `[-J_L,J_R]` 拼起来。

## 13. 为什么要有 `e_rel`？

控制器既知道期望相对位姿，也能由 FK 算出当前相对位姿。二者之差被编码成**六维局部误差** `e_rel`，而不是直接做 4×4 矩阵逐元素相减。

直观理解 `e_rel`：
- 一部分表示平移偏差；
- 一部分表示旋转偏差；
- `e_rel.norm()` 是六维数值向量的欧氏范数，**不能把它直接当成米或角度**。

**待在运动学包核实：**实际 `relativeError()` 对 SE(3) 采用的误差方向（当前逆×目标还是目标逆×当前）、旋转 log 的分支和 6 维分量次序。现在我们只按 `.cpp` 的表达式解释它如何使用这个返回值。

## 14. `BodyR` 是什么？

源码调用：

```cpp
kin_.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);
```

它表明希望使用特定 body-relative 表达约定，而不是任意 frame 下的雅可比。`BodyR` 的具体数学定义应追踪 `dual_arm_kinematics.hpp/.cpp` 才能做完整导数推导。

**读法：**看到 `J_rel` 和 `e_rel` 要问——它们是否采用相容的参考坐标系、排列与符号？否则即使矩阵尺寸正确，控制方向也可能不对。

---

# 第四部分：从误差到优化问题（QP）

## 15. 为什么任务残差里出现 `alpha_rel = kp_rel * dt`？

先从一维误差纠正开始。设期望位置与当前位置的差为 \(e\)，希望每步修正其中一部分：

$$
\Delta x_{\mathrm{desired}}\approx (kp\cdot dt)e.
$$

如果 `kp=10 /s`，`dt=0.008 s`，则单步反馈系数 \(\alpha=0.08\)。此处采用的误差正方向与 `.cpp` 的实际定义要相互匹配。

源码写：

```cpp
const double alpha_rel = params_.kp_rel * dt;
const Vec6 bias_rel = -alpha_rel * e_rel;
```

并且在任务中使用：

$$
r_{\mathrm{rel}}=J_{\mathrm{rel}}dq-s_{\mathrm{rel}}
-\alpha_{\mathrm{rel}}e_{\mathrm{rel}}.
$$

在 slack=0 时，希望选 `dq` 使 `J_rel*dq ≈ alpha_rel*e_rel`。

**必须注意的源码一致性问题：**头文件顶部文字注释示意 `J_rel*dq + alpha_rel*e_rel - s_rel`，但 `.cpp` 的实际 `bias_rel=-alpha_rel*e_rel`。不要凭头文件注释推导方向；先以实现为准，之后结合 `relativeError()` 定义与有限差分测试确认符号正确。

## 16. `slack` 为什么存在？

如果想要的相对运动与关节限位、速度限制或另一任务冲突，就可能没有精确解。加入 6 维 slack：

$$
J_{\mathrm{rel}}dq-s_{\mathrm{rel}}-\alpha e_{\mathrm{rel}}\approx0.
$$

这相当于允许任务残差不完全为零，但 QP 同时通过权重惩罚 `s_rel`。

**不是**夹爪真的在物体上打滑；它只是数学上的松弛变量。

## 17. 为什么不直接写 `J^{-1}`？

`J_rel` 是 6×12 的非方阵，且有速度、关节限位、避碰等不等式约束。单纯求逆不能同时考虑这些条件。所以选优化问题：在所有合法 `dq` 中，找到相对任务表现最好、其他目标也合理的一组。

标准形式（见 `qp_types.hpp`）：

$$
\min_z\frac12z^\top Pz+q_{\mathrm{cost}}^\top z,
\qquad l\le Az\le u.
$$

这里的 `q_cost` 和函数输入 `q` 完全是两个东西。

## 18. 从平方误差推导 `P`、`q_cost`

从一项代价开始：

$$
F(z)=\frac{w}{2}\|Bz+b\|^2.
$$

展开：

$$
\begin{aligned}
F(z)
&=\frac{w}{2}(Bz+b)^\top(Bz+b)\\
&=\frac12z^\top(wB^\top B)z+(wB^\top b)^\top z
+\frac w2b^\top b.
\end{aligned}
$$

末项和 `z` 无关，不影响最优点。于是：

$$
P\mathrel{+}=wB^\top B,\quad
q_{\mathrm{cost}}\mathrel{+}=wB^\top b.
$$

代码就是：

```cpp
P_ = params_.w_rel * (A_task_rel_.transpose() * A_task_rel_);
q_cost_ = params_.w_rel * (A_task_rel_.transpose() * bias_rel);
```

**矩阵尺寸手查：**

- `A_task_rel_`：6×24。
- `A_task_rel_.transpose()`：24×6。
- 相乘得到 `P_`：24×24。
- 转置乘 `bias_rel`（6×1）得到 `q_cost_`：24×1。

这一段本质是**把“误差尽量小”翻译成求解器能吃的格式**。

## 19. 关节限位为什么是 `l<=Az<=u`？

若单关节角度现为 `q_i`，关节允许范围为 `[q_min_i,q_max_i]`，则：

$$
q_{\min,i}\le q_i+dq_i\le q_{\max,i}
$$

移项得：

$$
q_{\min,i}-q_i\le dq_i\le q_{\max,i}-q_i.
$$

另外速度界：

$$
-v_{\max,i}dt\le dq_i\le v_{\max,i}dt.
$$

二者取交集：

$$
lo_i=\max(-v_{\max,i}dt,q_{\min,i}-q_i),
$$

$$
hi_i=\min(v_{\max,i}dt,q_{\max,i}-q_i).
$$

这正对应 `.cpp` 第 212～222 行，前 12 行 `A_scratch_` 是 `[I_12,0]`。`C_ineq/d_ineq` 会成为额外不等式的行。

---

# 第五部分：零空间、SVD、奇异和退化

## 20. `sigmaMinRel()` 为什么要对 Jacobian 做 SVD？

矩阵 SVD：

$$
J=U\Sigma V^\top.
$$

\(\Sigma\) 中的奇异值表示线性映射在不同方向上的放大能力。最小奇异值接近零，表示至少一个任务方向局部能力弱或病态。

项目中的 `sigma_min_rel` 取自相对 Jacobian。它和“180° 旋转中点不唯一”**不是同一类问题**。

## 21. 什么是零空间？

满足：

$$
J_{\mathrm{rel}}dq_N=0
$$

的关节小增量，理论上一阶近似不改变双手相对任务。因为输入是 12 维、相对约束通常 6 维，就有机会利用剩余方向做关节居中、可操作度或间距优化。

实际代码先构造改善方向 `dir`，再通过：

$$
N=I-J^\top(JJ^\top+\lambda^2I)^{-1}J
$$

获得 `dir_null=N*dir`。这里 \(\lambda^2=10^{-8}\)，阻尼形式是**近似投影**，不应宣称任何情况下严格保持主任务为零。

如果活跃的相对/物体任务都存在，拼接的 `J_task` 可变成 12×12；其秩决定还有多少冗余，不应只看行列数量断言完全没有零空间。

## 22. 有限差分为什么在代码里大量出现？

`ascentDirection()` 不是算位姿，而是在求诸如“关节这样动，最小奇异值会增大还是减小”的方向：

$$
\frac{\partial f}{\partial q_i}
\approx\frac{f(q+h e_i)-f(q-h e_i)}{2h}.
$$

这叫中心差分。代码把得到的改善方向作为**次级偏好**，交给零空间投影处理。

## 23. `f_sigma` 和 `vel_scale` 在做什么？

程序用 `smoothstep01` 将最小奇异值和误差大小映射为 0～1 的健康因子。

健康因子较差时：

- `w_reg_eff` 增大：不喜欢大关节增量；
- `w_slack_eff` 降低：更愿意放松任务；
- `vel_scale` 降低：允许的单步增量变小。

这种退化策略**不等于解决 C4.3 的 π 中点数学歧义**。

另外，当前 `solve()` 仍在物体任务关闭时计算旧 `e_obj/J_obj`；启用奇异退化时 `f_err_obj` 也可能参与 `f`。这是读实现才看得出来的接口边界。

---

# 第六部分：把公式缩小到可以手算的例子

我们用一个教学问题，只优化一个 `dq` 和一个 `s`：

$$
z=[dq,s]^\top,\quad
r=dq-s-0.02.
$$

最小化：

$$
F=\frac12(dq-s-0.02)^2+\frac12(4s^2).
$$

那么：

$$
B=[1,-1],\quad b=-0.02,
$$

$$
P=B^\top B+\begin{bmatrix}0&0\\0&4\end{bmatrix}
=\begin{bmatrix}1&-1\\-1&5\end{bmatrix},
\quad q_{\rm cost}=B^\top b=\begin{bmatrix}-0.02\\0.02\end{bmatrix}.
$$

不加额外约束的最优解是 `dq=0.02`、`s=0`，此时两项代价均为零。

如果额外强制 `dq <= 0.01`，一部分任务就无法完全满足；优化器会在 `dq`、`s` 和残差间权衡。**slack 并不意味着任何目标永远能零误差满足。**

验证公式可以代回：

$$
r=0.02-0-0.02=0,\quad F=0.
$$

真实程序只是在 12 关节、两个六维任务以及更多硬约束下使用同样的矩阵拼装方式。

---

# 第七部分：在项目里逐步跟读真实调用

## 24. 四参数版 `solve()` 的源码顺序

| `.cpp` 大致行号 | 关键变量 | 当前职责 |
|---|---|---|
| 83～92 | 三个重载、四种输入参数 | 补默认值、转到完整入口 |
| 93～106 | `result,e_rel,J_rel,e_obj` | 从当前状态算反馈任务 |
| 108～139 | `sigma_min_rel,f_sigma,f_err_*,w_reg_eff` | 连续退化 |
| 140～162 | `J_obj,A_task_*,bias_*,P_,q_cost_` | 把软任务装成二次目标 |
| 163～206 | `dir,J_task,N,dir_null,dq_center_scratch_` | 零空间次级偏好 |
| 207～210 | `w_center,w_reg_eff,w_slack_eff` | 正则和 slack 惩罚 |
| 211～241 | `vlim,lo,hi,A_scratch_,l_scratch_,u_scratch_` | 硬约束 |
| 242～251 | `last_problem_,solution` | 调 solver |
| 252～275 | `result.dq,slack,warm_start_` | 读解、保存、返回 |

## 25. `QpProblem` 和 `DualArmQpResult` 的关系

`QpProblem` 是送给求解器的**试卷**：

- `P`：二次权重矩阵；
- `q`：线性项；
- `A`：约束矩阵；
- `l/u`：下界/上界；
- `x0`：可选上次答案。

`QpSolution` 是底层求解器的原始答案，`DualArmQpResult` 再把它拆成具有机器人意义的字段：`dq`、`slack`、`slack_object`、`converged`、误差范数和诊断量。

这些数据结构见 `qp_types.hpp`、`dual_arm_qp.hpp`。

## 26. 有几个细节必须保留审查意识

1. **误差符号约定：**`dual_arm_qp.hpp` 顶部注释和 `.cpp` 残差符号表述不同；要看 `relativeError()` 源码和测试才能判定完整正确性。
2. **任务坐标系：**`BodyR` 的精确含义要到 `dual_arm_kinematics` 看，不能只用一维 `v_R-v_L` 代替三维严谨公式。
3. **新旧物体坐标系：**`objectJacobianBody()` 旧实现的 π 风险与 C4.3 新任务给定的物体坐标系不是一回事。
4. **头文件参数不等于 ROS 参数：**物体跟踪功能是否在 `dual_arm_qp_controller` 中实际接线，需要看控制器调用路径。
5. **`dq` 不等于真机已经运动：**真正执行还要经过位置指令、执行反馈和健康监控。
6. **退化与求解成功不是安全证明：**零空间的 clearance proxy 不是 FCL 真实碰撞检测，`converged` 也不代表接触抓持稳定。

---

# 第八部分：自测题与参考答案

**Q1：`left_T_right_target` 为什么是 `Iso3`，不是 `double distance`？**  
答：因为双端抓持不仅要求两手间距固定，还要求相对旋转保持一致。`Iso3` 同时包含旋转与平移。

**Q2：`left_T_right_target` 和 `world_T_object_target` 有什么区别？**  
答：前者规定两只手的**内部相对关系**，后者规定物体/协调坐标系在**世界中要去哪里**。两项任务可以并存，但当前控制器入口未必都启用。

**Q3：`object_body_twist_ff` 为什么用 `Vec6`？**  
答：物体刚体运动的瞬时速度包含 3 维线速度和 3 维角速度；源码中这个输入按物体系表达，头文件约定 `[v;w]`。

**Q4：为何 `dq` 是关节增量而不是关节速度？**  
答：`alpha=kp*dt` 和 `vlim=v_max*dt` 已把每秒量转换为单步增量，执行层会将 `dq` 累加到位置命令。

**Q5：`slack` 到底是不是允许夹爪松开？**  
答：不是。它是数学上的任务松弛变量，不是夹爪开合量或真实相对滑动。

**Q6：为什么 6×24 的 `A_task_rel` 能生成 24×24 的 `P`？**  
答：因为 \(A_{\mathrm{rel}}^\top A_{\mathrm{rel}}\) 的尺寸是 `(24×6)*(6×24)=24×24`。

**Q7：`sigma_min_rel` 与 π 旋转中点歧义是不是同一问题？**  
答：不是。前者衡量相对运动映射的局部病态性，后者来自旧物体姿态定义在特殊旋转关系下的不唯一性。

**Q8：要彻底判断 `e_rel` 的正负号，下一步读哪里？**  
答：打开 `dual_arm_kinematics` 的 `relativeError()`、`relativeJacobian()` 实现，并结合有限差分对拍测试。

---

## 建议练习：只跟踪这四个输入

在你打开 `.cpp` 时，先把下面四行写在纸上：

```text
q                         = 机器人当前状态
left_T_right_target       = 两只手希望保持怎样的相对位姿
world_T_object_target     = 物体整体希望到达的绝对位姿
object_body_twist_ff      = 物体目标轨迹的瞬时速度
```

然后在 `solve()` 中查每个参数第一次被使用的行：

- `q`：`relativeError / relativeJacobian / objectError`；
- `left_T_right_target`：`relativeError`；
- `world_T_object_target`：`objectError`；
- `object_body_twist_ff`：`bias_obj`。

看懂“参数进入哪条计算链”，再看繁琐的 Eigen 操作，会容易得多。

---

## 下一步真正需要阅读的包

本课解决了**QP 如何使用位姿与 Jacobian**。但如果要做到“连 `relativeError()` 的 SE(3) 误差符号都可以自己证明”，下一节就应该进入 `dual_arm_kinematics`，追踪：

1. `worldTgrasp()`、`relativeTransform()`、`worldTobject()`；
2. `relativeError()` 和 SE(3) `Log`；
3. `relativeJacobian()` 的 `BodyR` 定义；
4. 物体中点和 π 歧义的边界。

这样你就能够把运动学基础、QP 数学和 C++ 实现串成真正完整的一条链。

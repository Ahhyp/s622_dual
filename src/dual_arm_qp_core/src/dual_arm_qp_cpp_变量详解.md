# `dual_arm_qp.cpp` 变量详解与源码阅读地图

> 建议放在 `src/dual_arm_qp_core/src/dual_arm_qp_cpp_变量详解.md`，与 `dual_arm_qp.cpp` 同级。
>
> 对照版本：`Ahhyp/s622_dual` GitHub `public` 分支，2026-10-16 核对。源文件：[dual_arm_qp.cpp](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/dual_arm_qp.cpp)（279 行）；[dual_arm_qp.hpp](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/dual_arm_qp.hpp)（152 行）。
>
> 本文用于**边看代码边查**，不是替代源码、测试和实验的设计规范。行号随提交可能变化。

## 先看总图

```text
q（12轴当前关节角） + left_T_right_target（两爪目标相对位姿）
   + 可选 object_target / object_body_twist_ff
     ├─ kin_.relativeError(q, target) → e_rel
     ├─ kin_.relativeJacobian(q, BodyR) → J_rel
     ├─ kin_.objectError(...) → e_obj
     └─ kin_.objectJacobianBody(q) → J_obj
                         ↓
            任务矩阵 A_task_rel_ / A_task_obj_
                         ↓
            优化代价 P_ / q_cost_
            硬约束 A_scratch_ / l_scratch_ / u_scratch_
                         ↓
                 solver_->solve(...)
                         ↓
     result.dq（12个关节每周期增量）、slack 和状态
```

**最容易误读的一点：**`dq` 是当前周期的**关节角增量（rad）**，不是速度（rad/s）。QP 共有 24 个优化变量，不是 24 个关节：

$$
z=[dq(12);\ s_{\mathrm{rel}}(6);\ s_{\mathrm{obj}}(6)].
$$

## 1. 名字相似但含义完全不同

| 变量 | 作用 | 维度 |
|---|---|---|
| `q` | 当前关节角，不是 QP 线性项 | 12 |
| `q_cost_` | QP 目标函数的线性项，不是关节角 | 24 |
| `dq` / `result.dq` | 求解器决定的关节增量 | 12 |
| `dq_center` / `dq_center_scratch_` | 希望优先靠近的**增量参考值**，不是最终解 | 12 |
| `e_rel` | 左右两爪的相对位姿误差 | 6 |
| `e_obj` | 旧物体/协调坐标系的位姿误差 | 6 |
| `J_rel` | 关节增量对相对位姿任务的局部线性映射 | 6×12 |
| `J_obj` | 旧物体任务 Jacobian | 6×12 |
| `A_task_rel_` | 相对任务的**软代价残差矩阵** | 6×24 |
| `A_task_obj_` | 物体任务的软代价残差矩阵 | 6×24 |
| `A_scratch_` | QP 的**硬约束矩阵** | (12+k)×24 |
| `P_` | QP 目标的二次项矩阵 | 24×24 |
| `lower_`, `upper_` | 12 个关节本轮 `dq` 的上下界 | 各 12 |
| `l_scratch_`, `u_scratch_` | **全部**约束的上下界 | 各 12+k |

变量尾部的 `_` 只是 C++ 类成员的命名习惯；`scratch` 意味着反复使用的临时工作缓存。

**单位提醒：**六维位姿误差同时包含平移和旋转，`.norm()` 不能直接解读成米或度。物体前馈 `object_body_twist_ff` 的 6 维次序在头文件明确为 `[v; w]`（线速度在前、角速度在后）。

## 2. 类和参数从哪里来？

`dual_arm_qp.hpp` 中定义：

- `DualArmQpParams`：控制增益、权重、限位、额外约束；
- `DualArmQp`：构造/求解 QP；
- `DualArmQpResult`：一次计算的返回值。

### `DualArmQpParams` 变量字典（hpp:39–85）

| 名称 | 默认值或单位 | 干什么 |
|---|---|---|
| `dt` | `0.008 s` | 控制时间步 |
| `kp_rel` | `10 1/s` | 相对位姿误差反馈 |
| `w_rel` | `1.0` | 相对任务权重 |
| `w_slack` | `1e3` | 惩罚相对任务 slack |
| `w_center` | `1e-2` | 关节增量参考值的权重 |
| `w_reg` | `1e-6` | 抑制过大的关节增量 |
| `kp_obj` | `10 1/s` | 物体位姿反馈 |
| `w_obj` | `0.0` | 物体任务权重，0 表示代价关闭 |
| `w_slack_obj` | `1e-6` | 物体 slack 正则 |
| `w_null_margin` | `0.0` | 远离关节极限的次任务权重 |
| `w_null_manip` | `0.0` | 提高相对 Jacobian 最小奇异值 |
| `w_null_clear` | `0.0` | 提高简化两臂点间距，**不是碰撞安全** |
| `null_step` | `0.005 rad/step` | 次级偏好方向的尺度 |
| `enable_singularity` | `false` | 是否开启连续退化 |
| `sigma_critical`, `sigma_safe` | `0.06`,`0.20` | 奇异值退化阈值 |
| `damp_max` | `50.0` | 增大正则的倍率 |
| `slack_relax` | `0.9` | 退化时降低 slack 惩罚 |
| `vel_scale_min` | `0.5` | 退化后最低的速度缩放比例 |
| `e_safe`, `e_critical` | `1.2`,`2.4` | 任务误差健康度阈值 |
| `q_min`, `q_max` | 12维，rad | 位置限位 |
| `v_max` | 12维，rad/s | 速度限位 |
| `dq_center` | 12维，rad/step | 不开零空间目标时的二级偏好 |
| `C_ineq` | k×24 | 用户额外线性不等式 `C*z<=d` |
| `d_ineq` | k维 | 额外线性不等式的右端上界 |

### `DualArmQpResult` 输出字典（hpp:86–99）

| 名称 | 含义 |
|---|---|
| `dq` | 本轮 12 轴增量 |
| `slack`、`slack_object` | 相对任务、物体任务松弛向量 |
| `converged` | 求解器报告是否收敛，不等于物理安全 |
| `iterations` | 求解迭代次数 |
| `solve_time_ms` | 求解耗时（毫秒），不代表整个控制周期耗时 |
| `relative_error_norm`, `object_error_norm` | 两种六维误差向量范数 |
| `sigma_min_rel` | 相对 Jacobian 最小奇异值 |
| `singularity_factor`, `error_factor` | 连续退化相关因子 |
| `solver_name` | 求解器名称 |

## 3. `dual_arm_qp.cpp` 从上往下读

### L14–17 `smoothstep01(x)`

先把 `x` 夹在 `[0,1]` 里，再返回 `t*t*(3-2*t)`。返回值从 0 到 1 平滑变化。它用于**平滑调整控制强度**，不是表示抓取力。

### L19–23 `sigmaMinRel(kin,q)`

`J` 是 6×12 的 `relativeJacobian(q, BodyR)`；`svd` 对矩阵做奇异值分解；`.singularValues().minCoeff()` 获取最小奇异值。它反映相对任务的局部运动能力，**不是**直接判断“夹爪相差 π”时是否有歧义。

### L26–40 `clearanceProxy(kin,q)`

- `ql`, `qr`：左右各 6 轴的关节配置。
- `idx[4]={2,3,4,5}`：要比较的关键帧序号。
- `pl`, `pr`：两臂关键帧原点的世界坐标（米）。
- `d_min`：所有关键点组合里最小的欧氏距离（米）。
- `i`, `j`：双层循环的索引。

这只是关键点之间的距离**代理指标**，不能替代 FCL 碰撞检测。

### L43–52 `ascentDirection(q,h,f)`

中心差分算一个标量指标 `f(q)` 的上升方向：

$$
g_i \approx \frac{f(q+h e_i)-f(q-h e_i)}{2h}.
$$

- `Fn`：C++ 模板，接受任意合适的可调用函数类型；
- `h`：对每个关节做数值扰动的步长（调用处为 `1e-3 rad`）；
- `qp`, `qm`：第 i 轴分别增加/减少 `h` 的关节配置；
- `g[i]`：指标对第 i 个关节的近似导数。

### L54–62 `marginDirection(p,q)`

`range` 是当前关节的允许角度跨度；`mid` 是上下限的中点；`g[i] = -2*(q[i]-mid)/(range*range)` 给出“向关节中心回去”的偏好方向。

### L65–81 限位和构造函数

- `lo[6]`、`hi[6]`：单臂默认角度下/上限；
- `vv[6]`：单臂默认速度上限；
- `a*6+i`：从单臂序号变成 12 维全局序号；
- `kin_`：保存外部运动学对象的 const 引用；
- `params_`：参数副本；
- `solver_`：传入的求解器；如为空使用 `makeDefaultSolver()`。

### L83–92 `solve()` 三种入口

1. `solve(q, left_T_right_target)`：用当前 `kin_.worldTobject(q)` 作为物体目标。
2. `solve(q, relative_target, object_target)`：物体前馈速度默认为零。
3. `solve(q, relative_target, object_target, object_body_twist_ff)`：真正计算完整 QP。

`Iso3` 是三维刚体位姿（SE(3) 变换）的 Eigen 表达；不是一个只有坐标位置的三维向量。

### L93–106 任务误差、常量和索引

| 变量 | 含义 |
|---|---|
| `result` | 一次 `solve()` 的返回报告 |
| `kNumVars=24` | 总决策变量数 |
| `kSRel0=12` | 相对 slack 的起始索引 |
| `kSObj0=18` | 物体 slack 的起始索引 |
| `I12`, `I6` | 12×12 和 6×6 单位矩阵 |
| `e_rel` | 相对位姿误差，6维 |
| `J_rel` | 相对任务 Jacobian，6×12 |
| `e_obj` | 旧物体任务位姿误差，6维 |

### L108–139 `f_sigma`、`f_err_*`、`f`

这些是**健康/退化控制因子**，而非机器人位置：

| 变量 | 作用 |
|---|---|
| `sigma_min_rel` | `J_rel` 最小奇异值 |
| `f_sigma` | 由奇异值映射出来的健康因子 |
| `f_err_rel` | 由相对位姿误差映射出的健康因子 |
| `f_err_obj` | 由物体误差映射出的健康因子 |
| `denom`, `e_denom` | 计算退化因子的分母，防止除零 |
| `f` | `min(f_sigma,f_err_rel,f_err_obj)` |
| `w_reg_eff` | 退化后真正使用的正则权重 |
| `w_slack_eff` | 退化后真正使用的相对 slack 惩罚 |
| `vel_scale` | 缩小本轮最大关节增量的比例 |
| `alpha_rel` | `kp_rel * dt` |
| `alpha_obj` | `kp_obj * dt` |
| `object_on` | `w_obj > 0` |

公式：

$$
f_\sigma=\operatorname{smoothstep01}
\left(\frac{\sigma_{\min}-\sigma_{\rm critical}}
{\max(10^{-9},\sigma_{\rm safe}-\sigma_{\rm critical})}\right).
$$

当 `f` 降低时，`w_reg_eff` 变大，`w_slack_eff` 变小，`vel_scale` 变小，控制器更谨慎。

**重要实现边界：** `object_on=false` 时，当前代码仍会计算 `e_obj`，如果 `enable_singularity=true`，`f_err_obj` 仍进入 `f`。所以“物体代价没开”和“物体误差对所有逻辑都无影响”不是一回事。

### L140–162 QP 任务矩阵与代价

- `A_task_rel_`：6×24，包含 `[J_rel, -I6, 0]`；
- `J_obj`：6×12，来自 `objectJacobianBody(q)`；
- `A_task_obj_`：6×24，物体任务启用时是 `[J_obj, 0, -I6]`；
- `bias_rel`：`-alpha_rel * e_rel`；
- `bias_obj`：`-(alpha_obj*e_obj + dt*object_body_twist_ff)`；
- `P_`：QP 的 24×24 二次项；
- `q_cost_`：QP 的 24维线性项。

设 `z=[dq;s_rel;s_obj]`：

$$
r_{\rm rel}=A_{\rm task,rel}z+bias_{\rm rel}
=J_{\rm rel}dq-s_{\rm rel}-\alpha_{\rm rel}e_{\rm rel}.
$$

对残差代价 \(\frac{w}{2}\|Bz+b\|^2\)，展开可得：

$$
P\mathrel{+}=w B^\top B,\qquad q_{\rm cost}\mathrel{+}=wB^\top b.
$$

所以：

```cpp
P_ = w_rel * (A_task_rel_.transpose() * A_task_rel_);
q_cost_ = w_rel * (A_task_rel_.transpose() * bias_rel);
```

**不要混淆** `A_task_rel_` 和 `A_scratch_`：前者是**软任务代价**的系数，后者是**硬约束**矩阵。

### L163–206 零空间次级目标

| 变量 | 含义 |
|---|---|
| `nullspace_on` | 三种次级权重是否至少有一个启用 |
| `dir` | 各种改善方向加权合成的 12 维向量 |
| `add` | C++ lambda：把输入方向归一化、乘权重再加到 `dir` |
| `d`, `w` | lambda 里的方向和权重 |
| `n` | 向量范数；在不同作用域重复使用该名字 |
| `qq` | 有限差分探针中的临时关节配置 |
| `m_task` | 拼接 Jacobian 的任务行数，6 或 12 |
| `J_task` | 活跃主任务 Jacobian，`m_task×12` |
| `lambda2` | 阻尼平方项，`1e-8` |
| `JJt` | `J_task*J_task.transpose()+lambda2*I` |
| `N` | 12×12 阻尼近似零空间投影矩阵 |
| `dir_null` | 投影后的次级改善方向 |
| `dq_center_scratch_` | 本轮偏好增量，不是最终 QP 结果 |

$$
N=I-J_{\rm task}^\top
(J_{\rm task}J_{\rm task}^\top+\lambda^2I)^{-1}
J_{\rm task}.
$$

由于阻尼，`N` 不一定是严格正交投影。当物体任务和相对任务均启用时，`J_task` 变成 12×12，但是否真的没有零空间仍取决于它的秩。

注意第 200 行实际缩放表达式是：

```cpp
dq_center_scratch_ = (params_.null_step / std::max(1.0, n)) * dir_null;
```

因此它对 `n<1` 的方向也会乘 `null_step`，不是“只有超长才裁剪”的通常写法。

### L207–210 正则项与 slack

- 给 `P_` 的关节块加 `(w_center+w_reg_eff)*I12`；
- 给两个 slack 块加各自惩罚；
- 给 `q_cost_` 前 12 项减去 `w_center*dq_center_scratch_`。

对应偏好：

$$
\frac{w_{\rm center}}2\|dq-dq_{\rm center}\|^2+
\frac{w_{\rm reg,eff}}2\|dq\|^2+
\frac{w_{\rm slack,eff}}2\|s_{\rm rel}\|^2+
\frac{w_{\rm slack,obj}}2\|s_{\rm obj}\|^2.
$$

### L211–241 关节边界和一般不等式

| 变量 | 含义 |
|---|---|
| `i` | 关节序号 |
| `vlim` | `v_max[i]*vel_scale*dt`，本轮关节增量限幅（rad） |
| `lo` | `max(-vlim, q_min[i]-q[i])` |
| `hi` | `min(vlim, q_max[i]-q[i])` |
| `v` | 已经越界时用于向内恢复的增量 |
| `lower_`, `upper_` | 12 轴关节增量边界 |
| `k` | 额外不等式条数 |
| `m` | QP 总硬约束条数：`12+k` |

约束写为：

$$
l_{\rm scratch}\le A_{\rm scratch}z\le u_{\rm scratch}.
$$

前 12 行的 `A_scratch_` 是 `[I12, 0]`；如有额外不等式，下面追加 `C_ineq`，对应下界 `-∞`、上界 `d_ineq`。

**注意两个风险点：**已越界时的 `lo>hi` 分支会强制一个向内增量，这不是原始约束系统可行性的数学证明；代码按列数决定是否接入 `C_ineq`，但这里没有完整验证 `d_ineq` 的长度，应确保调用方提供匹配尺寸。

### L242–275 解题、取结果和 warm start

`last_problem_` 把 `P_`、`q_cost_`、`A_scratch_`、`l_scratch_`、`u_scratch_` 和可选初值 `x0` 交给求解器。

```cpp
const QpSolution solution = solver_->solve(last_problem_);
```

| 变量 | 意义 |
|---|---|
| `solution` | 求解器返回的完整报告 |
| `solution.x` | QP 解向量 |
| `result.dq` | `solution.x.segment<12>(0)` |
| `result.slack` | `solution.x.segment<6>(12)` |
| `result.slack_object` | `solution.x.segment<6>(18)` |
| `result.converged` | 求解器报告的收敛标志 |
| `warm_start_` | 把上次解保留下来作为下次初值 |

当 `k==0` 时，代码还把 `result.dq` 逐项 clamp 进关节 box 上下界；有一般不等式时跳过 clamp，避免额外修改破坏耦合约束。**求解失败时不能仅靠 clamp 保证安全**，执行层还必须处理失败结果。

---

## 4. 一维缩小版：理解 `P` 与 `q_cost`

真实代码是 24 维，这里只构造一个**教学示例**：优化一个关节增量 `dq` 和一个 slack `s`。

假设任务误差残差：

$$
r=dq-s-0.02.
$$

目标：

$$
\frac12r^2+\frac42 s^2.
$$

记 `z=[dq,s]`，`B=[1,-1]`，`b=-0.02`，得到：

$$
P=B^\top B+\begin{bmatrix}0&0\\0&4\end{bmatrix}
=\begin{bmatrix}1&-1\\-1&5\end{bmatrix},
$$

$$
q_{\rm cost}=B^\top b=\begin{bmatrix}-0.02\\0.02\end{bmatrix}.
$$

最优解是 `dq=0.02`、`s=0`，此时 `r=0`。**这不是你的项目测得的数据，只是为了看懂矩阵怎么由残差展开得到。**

## 5. Eigen 和 C++ 语法速查

| 写法 | 含义 |
|---|---|
| `.setZero()` | 清零向量或矩阵 |
| `.block<6,12>(0,0)` | 从 0 行 0 列开始的 6×12 矩阵块 |
| `.head<12>()` | 取向量的前 12 个元素 |
| `.segment<6>(18)` | 从索引 18 开始取 6 个元素 |
| `.topRows<6>()` / `.bottomRows(k)` | 前 6 行 / 最后 k 行 |
| `.transpose()` | 矩阵转置 |
| `.norm()` | 向量欧氏范数 |
| `.inverse()` | 求逆，病态问题需要注意数值风险 |
| `const X&` | 常量引用，避免复制、不通过它修改原对象 |
| `constexpr int` | 编译期整数常量 |
| `auto add = [&](...) {...};` | 定义 lambda；`[&]` 表示按引用捕获外部变量 |

## 6. 一眼辨认代码中三种不同的“奇异”

1. `sigma_min_rel` 小：**相对任务的运动学能力/条件问题**，由 C2.6 的退化因子处理。
2. 旧 `objectJacobianBody` 在左右姿态相差 π：**旋转测地中点的定义/求导问题**，不是调 `w_reg` 就能修好。
3. QP 的矩阵数值病态：**优化问题数值求解问题**，可能与权重、正则和变量尺度有关，也不等价于前两者。

## 7. 建议你这样读这一个 `.cpp`

- **第 1 遍：**读 L83–106，搞清 QP 需要什么输入。
- **第 2 遍：**读 L140–162，亲手写出 `A_task_rel_` 的 6×24 分块。
- **第 3 遍：**读 L207–251，对照 `P/q_cost/A/l/u` 矩阵和求解器入口。
- **第 4 遍：**最后研究 L108–139（退化）与 L163–206（零空间），避免一开始被次级逻辑淹没。
- **第 5 遍：**对照 `dual_arm_qp_controller` 的 `update()`，核对 `result.dq` 最终如何累加到位置命令。

**记忆口诀：**`e/J` 说任务在哪里、怎么变；`A_task/bias` 说误差残差；`P/q_cost` 说优化偏好；`A/l/u` 说硬边界；`solver` 负责算；`result.dq` 是一步建议。

## 8. 自测题

1. `q_cost_` 为什么是 24 维，而 `q` 只有 12 维？
2. 为什么 `A_task_rel_` 和 `A_scratch_` 都以 A 命名，却不是一回事？
3. `f_sigma` 下降时，哪三个实际控制变量会变化？
4. `nullspace_on=false` 时，`dq_center_scratch_` 是从哪来的？
5. `object_on=false` 时 `J_obj` 是否还会被算？`f_err_obj` 是否还可能影响 `f`？
6. 若 `k=0`，总硬约束有多少行？若增加 k 条 CBF 不等式呢？
7. 为什么 `converged=true` 不能当成“两个夹爪已经可靠抓牢物体”的证据？

**适合放在源码目录里的原则：**这里不复制整份 C++ 文件，而是解释它已经存在的变量和公式；当你以后改动 `.cpp`，这份文档也应该一起复核。

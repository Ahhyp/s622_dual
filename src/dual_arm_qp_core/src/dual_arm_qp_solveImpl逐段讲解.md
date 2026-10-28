# `solveImpl` 逐段讲解
注意：本文的符号和docs/ 中的文档有出入， 不过不难理解， 问题不大。


先给一张**全局地图**，再逐段拆。这个函数做的事只有一件：**把"双臂任务误差"变成一个标准 QP，交给 `solver_` 解，再把解拆回 `dq / slack / slack_object`。**

---

## 全局地图：这个函数在解什么

决策变量固定 24 维：

```
z = [ dq(12) ; s_rel(6) ; s_obj(6) ]
     ↑关节增量   ↑相对任务松弛  ↑物体任务松弛
```

QP 的形式（`QpProblem`）：

$$
\min_z \ \tfrac12 z^\top P z + q^\top z \quad\text{s.t.}\quad l \le A z \le u
$$

**核心思想**：任务误差 $e$ 与 $dq$ 的关系是 $e_{\text{dot}} \approx -J\,dq$（具体符号取决于误差定义）。我们希望 $e_{\text{dot}} = -\alpha e$，即

$$
J\,dq = \alpha e
$$

但 $J$ 是 $6\times12$ 的，这个方程不一定有精确解（奇异、冲突、不可达）。所以**引入松弛 $s$**：

$$
J\,dq - s = \alpha e \quad\Longleftrightarrow\quad [J,\ -I]\begin{bmatrix}dq\\s\end{bmatrix} = \alpha e
$$

记 $A_{\text{task}}=[J,-I]$，$b=-\alpha e$（注意 $b$ 和 $\alpha e$ 差一个负号），目标变成

$$
\min \ \|A_{\text{task}} z + b\|^2
$$

展开就是标准 QP：$P = A_{\text{task}}^\top A_{\text{task}}$，$q = A_{\text{task}}^\top b$。**再给 $s$ 加二次惩罚**，防止它无限大。这就是整个函数的主干。

下面逐段。

---

## 段 0：函数入口与初始化

```cpp
DualArmQpResult result;
result.solver_name = solver_->name();
const bool use_row = (object_task_row != nullptr);
result.object_task_external = use_row;
```

- `result` 是返回值，后面逐步填。
- `use_row`：**外部是否传入了物体任务行**。这是 C4.3 S3 的设计——物体任务可以由外部（比如轨迹规划器）直接给 `J, e, ff`，而不是内部用运动学算。`use_row=true` 时，后面所有"物体任务"的量都从 `object_task_row` 读，**绝不**回退到内部计算（代码里用 `?:` 保证只求值一个分支）。
- `result.object_task_external` 只是标记来源，供上层诊断。

---

## 段 1：常量定义

```cpp
constexpr int kNumVars = 24;
constexpr int kSRel0 = 12;
constexpr int kSObj0 = 18;
const Eigen::Matrix<double, 12, 12> I12 = ...;
const Eigen::Matrix<double, 6, 6> I6 = ...;
```

- `kNumVars=24`：**决策变量维数固定**。这是文件头注释强调的："so the QP structure never changes at runtime (OSQP workspace stays warm)"。物体任务关掉时不是减少变量，而是把它的权重设 0——这样 OSQP 的 workspace 不用重建。
- `kSRel0=12`、`kSObj0=18`：`s_rel` 在 $z$ 中的起始下标是 12，`s_obj` 起始下标是 18。即

  $$
  z = [\underbrace{dq}_{0..11};\ \underbrace{s_{rel}}_{12..17};\ \underbrace{s_{obj}}_{18..23}]
  $$

- `I12, I6`：后面拼块矩阵用的单位阵。

---

## 段 2：算当前误差与雅可比

```cpp
const Vec6 e_rel = kin_.relativeError(q, left_T_right_target);
result.relative_error_norm = e_rel.norm();
const dual_arm::Mat6x12 J_rel = kin_.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);

const Vec6 e_obj = use_row ? object_task_row->e : kin_.objectError(q, world_T_object_target);
result.object_error_norm = e_obj.norm();
```

- `e_rel`：当前"左臂相对右臂"位姿与目标的 6 维误差（位置 3 + 旋转 3，旋转用某种 6 维表示，如 so3 log）。`relativeError` 内部会算 $T_{L}^{-1}T_R$ 与目标 $T_{LR}^{\text{target}}$ 的差。
- `J_rel`：**右臂体坐标系下的相对雅可比**，$6\times12$，把 12 个关节速度映射到相对位姿速度。`RelativeFrame::BodyR` 表示在右臂体坐标系表达。
- `e_obj`：物体任务误差。**关键**：`use_row ? ... : ...` 保证外部行存在时**不求值内部运动学**（`kin_.objectError` 不会被调用），这既是性能也是正确性要求（外部可能故意覆盖内部模型）。
- 两个 `norm` 存进 `result`，供上层诊断/日志。

---

## 段 3：连续奇异值退化因子

```cpp
const double sigma_min_rel = sigmaMinRel(kin_, q);
result.sigma_min_rel = sigma_min_rel;
double f_sigma = 1.0;
double f_err_rel = 1.0;
double f_err_obj = 1.0;
if (params_.enable_singularity) {
    const double denom = std::max(1e-9, params_.sigma_safe - params_.sigma_critical);
    f_sigma = smoothstep01((sigma_min_rel - params_.sigma_critical) / denom);
    const double e_denom = std::max(1e-9, params_.e_critical - params_.e_safe);
    f_err_rel = smoothstep01((params_.e_critical - e_rel.norm()) / e_denom);
    f_err_obj = smoothstep01((params_.e_critical - e_obj.norm()) / e_denom);
}
const double f = std::min({f_sigma, f_err_rel, f_err_obj});
```

这段是 **C2.6 的"连续退化"**，目的是：当机械臂接近奇异或误差很大时，**平滑地降低"任务权威"**（damping 增大、slack 放松、速度限幅），而不是硬开关。

- `sigma_min_rel`：相对雅可比 $J_{rel}$ 的**最小奇异值**。它衡量"最差方向的可操作度"——接近 0 表示某个方向几乎动不了。
- `f_sigma = smoothstep01((σ_min − σ_critical)/(σ_safe − σ_critical))`：
  - 当 $\sigma_{\min} \ge \sigma_{\text{safe}}$，`f_sigma = 1`（完全正常）；
  - 当 $\sigma_{\min} \le \sigma_{\text{critical}}$，`f_sigma = 0`（完全退化）；
  - 中间平滑过渡。`smoothstep01` 是 $t^2(3-2t)$，两端导数为 0，避免抖动。
- `f_err_rel, f_err_obj`：误差**太大**时也退化（$e \ge e_{\text{critical}}$ 时 $f=0$）。注意这里 `smoothstep01((e_critical - |e|)/e_denom)`——误差越大，括号越小，$f$ 越小。
- `f = min(f_sigma, f_err_rel, f_err_obj)`：**任一条件触发就退化**（取最保守）。

`result.singularity_factor` 和 `result.error_factor` 分开存，便于诊断。

> **注意**：这段的注释明确说"**不**降反馈增益 `kp`"。因为实测"误差大时停推"反而冻结误差。真正降的是**权威**（damping↑、slack 放松、速度限幅），见下一段。

---

## 段 4：把退化因子翻译成有效权重

```cpp
const double w_reg_eff = params_.w_reg * (1.0 + params_.damp_max * (1.0 - f));
const double w_slack_eff = params_.w_slack * std::max(0.05, 1.0 - params_.slack_relax * (1.0 - f));
const double vel_scale = params_.vel_scale_min + (1.0 - params_.vel_scale_min) * f;
```

三个旋钮都随 `f` **连续**变化：

- `w_reg_eff`：$f=1$ 时 = `w_reg`（正常阻尼）；$f=0$ 时 = `w_reg·(1+damp_max)`（阻尼放大到 `damp_max` 倍）。**退化时加大正则**，让解更"保守/平滑"。
- `w_slack_eff`：$f=1$ 时 = `w_slack`；$f=0$ 时 = `w_slack·max(0.05, 1−slack_relax)`。**退化时放松 slack 惩罚**，允许任务误差被 slack 吸收，而不是硬顶。
- `vel_scale`：$f=1$ 时 = 1（全速）；$f=0$ 时 = `vel_scale_min`（限速）。**退化时降速**。

这三个量是后面真正进入 $P$、约束的"有效值"。

---

## 段 5：步长与开关

```cpp
const double dt = params_.dt;
const double alpha_rel = params_.kp_rel * dt;
const double alpha_obj = params_.kp_obj * dt;
const bool object_on = params_.w_obj > 0.0;
```

- $\alpha = k_p \cdot dt$：**一阶误差衰减系数**。离散化 $e_{\text{dot}} = -k_p e$ 得 $e_{k+1} \approx (1-\alpha)e_k$，所以 $\alpha$ 是"每步消掉误差的比例"。
- `object_on`：物体任务是否启用，由 `w_obj > 0` 决定。**注意即使 `use_row=true`，如果 `w_obj=0`，物体任务也不进 QP**（后面 `if (object_on)` 才拼块）。

---

## 段 6：拼任务行块 $A_{\text{task}}$

```cpp
// relative: [J_rel, -I6, 0]
A_task_rel_.setZero();
A_task_rel_.block<6, 12>(0, 0) = J_rel;
A_task_rel_.block<6, 6>(0, kSRel0) = -I6;

// object: [J_obj, 0, -I6]
const dual_arm::Mat6x12 J_obj = use_row ? object_task_row->J : kin_.objectJacobianBody(q);
A_task_obj_.setZero();
if (object_on) {
    A_task_obj_.block<6, 12>(0, 0) = J_obj;
    A_task_obj_.block<6, 6>(0, kSObj0) = -I6;
}
```

这是全函数**最关键的结构**。$6\times24$ 的行块：

$$
A_{\text{task}}^{\text{rel}} =
\begin{bmatrix}
J_{rel} & -I_6 & 0
\end{bmatrix}
\quad\Longrightarrow\quad
A_{\text{task}}^{\text{rel}} z = J_{rel}\,dq - s_{rel}
$$

$$
A_{\text{task}}^{\text{obj}} =
\begin{bmatrix}
J_{obj} & 0 & -I_6
\end{bmatrix}
\quad\Longrightarrow\quad
A_{\text{task}}^{\text{obj}} z = J_{obj}\,dq - s_{obj}
$$

于是 $A_{\text{task}} z$ 就是"任务预测残差（含松弛）"。后面要让这个残差等于 $-\alpha e$（注意符号，见段 7）。

- `use_row ? object_task_row->J : kin_.objectJacobianBody(q)`：同段 2，外部行优先，且**只求值一个分支**。
- `if (object_on)`：物体任务关掉时，`A_task_obj_` 保持全零（`setZero` 已做）。全零行块后面乘 $w_{obj}=0$ 不贡献，但保留了数组形状。

---

## 段 7：算 bias（希望残差等于什么）

```cpp
// residual = A z - alpha*e  (so that edot ~= -alpha*e drives e -> 0)
const Vec6 bias_rel = -alpha_rel * e_rel;
const Vec6 ff = use_row ? object_task_row->body_twist_ff : object_body_twist_ff;
const Vec6 bias_obj = -(alpha_obj * e_obj + dt * ff);
```

注释写得很清楚：**希望 $A z = \alpha e$**，即 $e_{\text{dot}} \approx -\alpha e$（误差指数衰减）。

但代码里定义的是 `bias = -alpha*e`（带负号）。为什么？因为最小二乘要写成 $\|Az + b\|^2$ 的形式（$b$ 是"加到 $Az$ 上的偏移"）：

$$
\|Az - \alpha e\|^2 = \|Az + \underbrace{(-\alpha e)}_{b}\|^2
$$

所以 $b = -\alpha e$，这就是 `bias_rel`。**记住：$b$ 是负的 $\alpha e$。**

物体任务多了一项 `dt * ff`：`ff` 是**前馈体twist**（C2.4c），用来抵消"物体在动导致的跟随滞后"。所以物体任务希望：

$$
A_{\text{task}}^{\text{obj}} z = \alpha_{obj} e_{obj} + dt \cdot v_{ff}
$$

`bias_obj = -(alpha_obj*e_obj + dt*ff)` 同理。

---

## 段 8：构造主任务的 $P$ 和 $q$

```cpp
P_ = params_.w_rel * (A_task_rel_.transpose() * A_task_rel_);
q_cost_ = params_.w_rel * (A_task_rel_.transpose() * bias_rel);
if (object_on) {
    P_ += params_.w_obj * (A_task_obj_.transpose() * A_task_obj_);
    q_cost_ += params_.w_obj * (A_task_obj_.transpose() * bias_obj);
}
```

**推导**：我们要 $\min \ \|A_{\text{task}} z + b\|^2$。展开：

$$
\|Az+b\|^2 = z^\top A^\top A z + 2b^\top A z + b^\top b
$$

对照 QP 标准形 $\tfrac12 z^\top P z + q^\top z$（忽略常数 $b^\top b$）：

$$
P = 2 A^\top A,\qquad q = 2 A^\top b
$$

代码里省略了系数 2（不影响 argmin），所以：

$$
P = w \cdot A^\top A,\qquad q = w \cdot A^\top b
$$

两项任务按权重 `w_rel`、`w_obj` **加权求和**。物体任务关掉时（`object_on=false`），只加相对任务——**这就是"用 $w_{obj}=0$ 关任务"的实现**。

---

## 段 9：零空间（你之前看不懂的部分）

```cpp
const bool nullspace_on = params_.w_null_margin > 0.0 || params_.w_null_manip > 0.0 ||
                          params_.w_null_clear > 0.0;
if (nullspace_on) {
    Vec12 dir = Vec12::Zero();
    auto add = [&](const Vec12& d, double w) {
        const double n = d.norm();
        if (n > 1e-12) dir += w * (d / n);   // 归一化后加权
    };
    if (params_.w_null_margin > 0.0) add(marginDirection(params_, q), params_.w_null_margin);
    if (params_.w_null_manip > 0.0) {
        add(ascentDirection(q, 1e-3, [&](const Q12& qq) { return sigmaMinRel(kin_, qq); }),
            params_.w_null_manip);
    }
    if (params_.w_null_clear > 0.0) {
        add(ascentDirection(q, 1e-3, [&](const Q12& qq) { return clearanceProxy(kin_, qq); }),
            params_.w_null_clear);
    }
    const int m_task = object_on ? 12 : 6;
    Eigen::MatrixXd J_task(m_task, 12);
    J_task.topRows<6>() = J_rel;
    if (object_on) J_task.bottomRows<6>() = J_obj;
    const double lambda2 = 1e-8;
    const Eigen::MatrixXd JJt =
        J_task * J_task.transpose() + lambda2 * Eigen::MatrixXd::Identity(m_task, m_task);
    const Eigen::MatrixXd N =
        Eigen::Matrix<double, 12, 12>::Identity() - J_task.transpose() * JJt.inverse() * J_task;
    Vec12 dir_null = N * dir;
    const double n = dir_null.norm();
    if (n > 1e-12) {
        dq_center_scratch_ = (params_.null_step / std::max(1.0, n)) * dir_null;
    } else {
        dq_center_scratch_.setZero();
    }
} else {
    dq_center_scratch_ = params_.dq_center;
}
```

**这个块在干什么**：主任务（相对 + 物体）只"用掉"了任务空间的若干维。剩下的自由度（零空间）**不改变主任务误差**，可以拿来干次要的事——关节回中、远离奇异、增大两臂间隙。这段就是构造一个"期望的零空间方向" `dq_center_scratch_`，后面以**软惩罚**形式进 QP。

**三步拆解**：

**第 1 步：构造偏好方向 `dir`。** 三个来源：

| 权重 | 方向 | 含义 |
|---|---|---|
| `w_null_margin` | `marginDirection` | 关节往行程中点回：$g_i = -2(q_i - \text{mid}_i)/\text{range}_i^2$，是 $\sum((q_i-\text{mid}_i)/\text{range}_i)^2$ 的下降方向 |
| `w_null_manip` | `ascentDirection(sigmaMinRel)` | 提升 $J_{rel}$ 最小奇异值（中心差分近似梯度） |
| `w_null_clear` | `ascentDirection(clearanceProxy)` | 增大两臂关键连杆间距（中心差分） |

`add` 里**先归一化再加权**，所以权重只决定"相对重要性"，不会被量纲吞掉。

**第 2 步：投影到零空间。**

$$
N = I - J_{\text{task}}^\top (J_{\text{task}} J_{\text{task}}^\top + \lambda^2 I)^{-1} J_{\text{task}}
$$

这是**阻尼最小二乘的零空间投影矩阵**。几何直觉：$J_{\text{task}}^\top(JJ^\top+\lambda^2I)^{-1}J$ 是"投影到 $J$ 的行空间"的算子，$I$ 减去它就是"投影到 $J$ 的零空间"。所以 $N\,dir$ 是 `dir` 中**不改变主任务**的那部分。$\lambda^2=10^{-8}$ 是为了 $JJ^\top$ 奇异时也能求逆（伪逆近似）。

**第 3 步：缩放。** `dq_center_scratch_ = (null_step / max(1, n)) * dir_null`：若 `n>1` 则缩放后长度 = `null_step`，否则 = `null_step·n`。总之**不超过 `null_step`**，防止次要目标喧宾夺主。

**重要**：`m_task = object_on ? 12 : 6`。两个任务都开时 $J_{\text{task}}$ 是 $12\times12$，$N\approx 0$（没有冗余），零空间逻辑几乎失效——这是数学上合理的：12 个自由度全被主任务占满。

**它怎么进 QP**：看段 10。

---

## 段 10：正则、中心、松弛代价

```cpp
P_.block<12, 12>(0, 0) += (params_.w_center + w_reg_eff) * I12;
P_.block<6, 6>(kSRel0, kSRel0) += w_slack_eff * I6;
P_.block<6, 6>(kSObj0, kSObj0) += params_.w_slack_obj * I6;
q_cost_.head<12>() -= params_.w_center * dq_center_scratch_;
```

四行分别加四个**软代价**：

**① `dq` 的中心/正则项**：

$$
\tfrac12(w_{\text{center}}+w_{\text{reg,eff}})\|dq\|^2 - w_{\text{center}}\,dq_{\text{center}}^\top dq
$$

展开前两项正好是 $\tfrac12 w_{\text{center}}\|dq - dq_{\text{center}}\|^2 + \tfrac12 w_{\text{reg,eff}}\|dq\|^2$（差一个常数）。所以：

- `w_center` 把 $dq$ 往 `dq_center_scratch_`（零空间方向）拉；
- `w_reg_eff` 是纯正则（防解过大/病态）。

**这就是零空间偏好的"进入方式"**：不是硬约束，而是**软惩罚**。主任务权重远大于 `w_center`，所以主任务主导，中心项只在主任务不在乎的方向（零空间）上起作用。

**② `s_rel` 的松弛代价**：$\tfrac12 w_{\text{slack,eff}}\|s_{rel}\|^2$。防止松弛无限大（否则任务残差可以恒为 0，没意义）。`w_slack_eff` 已在段 4 按退化因子放松。

**③ `s_obj` 的松弛代价**：同理，用固定的 `w_slack_obj`（不随退化变）。

---

## 段 11：`dq` 的 box 约束

```cpp
for (int i = 0; i < 12; ++i) {
    const double vlim = params_.v_max[i] * vel_scale * dt;
    double lo = std::max(-vlim, params_.q_min[i] - q[i]);
    double hi = std::min(vlim, params_.q_max[i] - q[i]);
    if (lo > hi) {
        const double v = (q[i] > params_.q_max[i]) ? -vlim : vlim;
        lo = hi = v;
    }
    lower_[i] = lo;
    upper_[i] = hi;
}
```

每个关节的 `dq` 要同时满足**两个约束**：

- **速度限制**：$|dq_i| \le v_{\max,i}\cdot \text{vel\_scale}\cdot dt$（`vel_scale` 在退化时 <1，降速）；
- **位置限制**：$q_i + dq_i \in [q_{\min,i}, q_{\max,i}]$，即 $dq_i \in [q_{\min,i}-q_i,\ q_{\max,i}-q_i]$。

取交集就是 `lo = max(-vlim, q_min-q)`、`hi = min(vlim, q_max-q)`。

**`lo > hi` 的兜底**：如果当前 $q_i$ 已经越界（比如被外力推出去了），两个约束矛盾。此时强制 $dq_i = \pm vlim$，方向朝"回界内"：$q_i > q_{\max}$ 就取 $-vlim$（往小走），否则 $+vlim$。这是**安全优先**的兜底。

---

## 段 12：拼完整约束矩阵 $A$

```cpp
int k = 0;
if (params_.C_ineq.rows() > 0 && params_.C_ineq.cols() == kNumVars) {
    k = static_cast<int>(params_.C_ineq.rows());
}
const int m = 12 + k;
if (A_scratch_.rows() != m || A_scratch_.cols() != kNumVars) {
    A_scratch_.resize(m, kNumVars);
    l_scratch_.resize(m);
    u_scratch_.resize(m);
}
A_scratch_.topRows(12).setZero();
A_scratch_.topRows(12).leftCols<12>() = I12;
l_scratch_.head(12) = lower_;
u_scratch_.head(12) = upper_;
if (k > 0) {
    A_scratch_.bottomRows(k) = params_.C_ineq;
    l_scratch_.tail(k).setConstant(-std::numeric_limits<double>::infinity());
    u_scratch_.tail(k) = params_.d_ineq;
}
```

完整约束矩阵是**上下拼接**：

$$
A = \begin{bmatrix}
I_{12} & 0 & 0 \\
C_{\text{ineq}}
\end{bmatrix}
\in \mathbb{R}^{(12+k)\times 24}
$$

- **顶部 12 行**：`[I12, 0, 0]`，作用在 $z$ 上就是 $dq$，配合 `l=lower, u=upper` 实现 box 约束。
- **底部 $k$ 行**：可选的一般不等式 $C_{\text{ineq}} z \le d_{\text{ineq}}$，写成 $l=-\infty, u=d_{\text{ineq}}$。**注意 `C_ineq` 是 24 列的**，所以它可以直接约束 $[dq; s_{rel}; s_{obj}]$ 的任意线性组合，不只是 $dq$。

**为什么 `resize` 有判断**：`A_scratch_` 等是成员，跨调用复用，只在尺寸变化时重建——避免每周期分配（实时性）。

---

## 段 13：组装 `QpProblem`

```cpp
last_problem_.P = P_;
last_problem_.q = q_cost_;
last_problem_.A = A_scratch_;
last_problem_.l = l_scratch_;
last_problem_.u = u_scratch_;
if (warm_start_enabled_ && warm_start_.size() == kNumVars) {
    last_problem_.x0 = warm_start_;
}
```

把前面所有量装进 `QpProblem` 结构。`last_problem_` 是成员，保留下来供调试/诊断。

`x0` 是 **warm start**：如果启用且上次解过（`warm_start_.size()==24`），把上次解作为初值。ADMM/OSQP 都受益于好的初值（相邻控制周期 QP 相近）。

---

## 段 14：求解并拆解

```cpp
const QpSolution solution = solver_->solve(last_problem_);
result.converged = solution.converged;
result.iterations = solution.iterations;
result.solve_time_ms = solution.solve_time_ms;

if (solution.x.size() == kNumVars) {
    result.dq = solution.x.segment<12>(0);
    result.slack = solution.x.segment<6>(kSRel0);
    result.slack_object = solution.x.segment<6>(kSObj0);
} else if (solution.x.size() == 12) {
    result.dq = solution.x;
}
```

- `solver_->solve(...)`：多态调用，可能是 `OsqpSolver` 或 `AdmmSolver`（见 `makeDefaultSolver`）。
- 解出来的 `solution.x` 是 24 维，按段 1 的约定拆回 `dq / slack / slack_object`。
- 兼容分支 `size()==12`：如果某个 solver 只返回 12 维（只有 `dq`），也能处理。这是防御性代码。

---

## 段 15：安全 clamp（仅 box 情形）

```cpp
if (k == 0) {
    for (int i = 0; i < 12; ++i) {
        result.dq[i] = std::min(upper_[i], std::max(lower_[i], result.dq[i]));
    }
}
```

**仅当没有一般不等式（`k==0`）时**，把 `dq` 硬夹回 box。为什么？

- 有一般不等式时，强行 clamp `dq` **可能破坏 $C_{\text{ineq}} z \le d_{\text{ineq}}$**（因为 `s_rel/s_obj` 没动，`dq` 动了，线性组合就变了）。所以有一般不等式时**不 clamp**，信任 solver（因为它应该已经满足约束）。
- `k==0` 时约束只有 box，clamp 不会破坏任何东西，是纯安全网（防 solver 数值误差导致微小越界）。

---

## 段 16：更新 warm start

```cpp
if (warm_start_enabled_) {
    if (warm_start_.size() != kNumVars) warm_start_.setZero(kNumVars);
    warm_start_.head<12>() = result.dq;
    warm_start_.segment<6>(kSRel0) = result.slack;
    warm_start_.segment<6>(kSObj0) = result.slack_object;
}
return result;
```

把本次解存下来，下次作为 `x0`。**注意存的是 clamp 后的 `dq`**（段 15 之后），所以 warm start 与最终返回一致。

---

## 一张图总结数据流

```
q, target
   │
   ├─→ e_rel, J_rel ────────────┐
   ├─→ e_obj, J_obj ────────────┤
   ├─→ σ_min → f → w_reg_eff,   │
   │            w_slack_eff,    │
   │            vel_scale       │
   │                            ▼
   │              A_task=[J,-I,0]  (rel)
   │              A_task=[J,0,-I]  (obj)
   │                            │
   │              bias=-αe (-dt·ff)
   │                            ▼
   │              P = Σ w·AᵀA,  q = Σ w·Aᵀbias
   │                            │
   ├─→ 零空间: dir → N·dir → dq_center_scratch
   │                            │
   │              P += (w_center+w_reg)I,  q -= w_center·dq_center
   │              P += w_slack·I (s_rel), P += w_slack_obj·I (s_obj)
   │                            │
   ├─→ box: lower/upper ────────┤
   └─→ C_ineq, d_ineq ──────────┤
                                ▼
                     A = [I12,0,0; C_ineq],  l,u
                                │
                                ▼
                     solver_->solve(P,q,A,l,u,x0)
                                │
                                ▼
                     x = [dq; s_rel; s_obj]
                                │
                                ▼
              clamp(仅 k==0) → result.dq → warm_start
```

---

## 最后：三个最容易误解的点

**① 任务不是硬约束，是软代价。** 代码从没写 $J\,dq - s = \alpha e$ 作为约束，而是把它写成 $\|A_{\text{task}} z + b\|^2$ 的代价。所以"任务"和"约束"是两回事：任务用 $P,q$ 软性追求，约束（box + $C_{\text{ineq}}$）才是硬的。松弛 $s$ 的存在就是为了让任务"可违反但有代价"。

**② $w_{obj}=0$ 关任务，不改变 QP 结构。** 24 维固定、$A$ 形状固定，只是物体任务块权重为 0。这是为了 OSQP workspace 常驻（文件头注释的核心动机）。

**③ 零空间是"软偏好"，不是"严格零空间"。** $N$ 是阻尼伪逆版本，且最终 `dq` 由 QP 权衡主任务和中心项，不保证 $J\,dq$ 严格不变。真正严格的冗余分配需要把 $dq=Nv$ 作为等式约束重参数化——这里没做，是工程取舍。

如果你还想深入某一段（比如 $N$ 的几何推导、或 $C_{\text{ineq}}$ 具体怎么用、或 `smoothstep` 为什么两端导数为 0），告诉我哪段，我再展开。
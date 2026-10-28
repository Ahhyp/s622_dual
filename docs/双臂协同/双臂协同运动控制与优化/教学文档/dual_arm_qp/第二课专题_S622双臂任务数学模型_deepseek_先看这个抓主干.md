# 双臂 QP 求解器的完整数学模型：从 SE(3) 到 24 维 QP

> 本文以你提供的 `dual_arm_qp.cpp` 为准，覆盖该文件里**实际存在**的全部机制：两个主任务、连续奇异退化、零空间次级优化（三方向）、正则/中心/松弛、box 约束、通用不等式接口 `C_ineq`、安全 clamp、warm start、输入校验与显式失败。CBF、D7 健康监控、设备急停**不在本文件**，会明确标注它们的位置。

注意：其中只有两个主任务是第二课的内容， 其余部分都是后面的课程。这个文档可以帮助抓住主干。

---

## 符号约定（与 GPT 文档对齐，避免与关节角 $q$ 混淆）

和第二课其他的专题文档的符号还是有点出入。 

| 符号 | 含义 | 与 GPT 文档的对应 |
|---|---|---|
| $q=[q_L;q_R]\in\mathbb R^{12}$ | 关节角 | 第四/六/七课一致 |
| $\Delta q\in\mathbb R^{12}$ | 关节增量 | 第四/六/七课一致 |
| $z=[\Delta q;s_{rel};s_{obj}]\in\mathbb R^{24}$ | QP 决策变量 | 第七课一致 |
| $P$ | QP 二次项 | 第四课一致 |
| $c$ | **QP 线性代价项** | 第四课通用式用 $q$（注明"不是关节角"）；本讲义改用 $c$，代码变量名是 `q_cost_`（历史命名） |
| $A_{\text{task}}^{\text{rel}},A_{\text{task}}^{\text{obj}}$ | 任务行（进代价 $P,c$） | 概念对应第七课行块，**这里是代价不是硬约束** |
| $A$ | QP 约束矩阵（$l\le Az\le u$） | 第四课一致；**与 $A_{\text{task}}$ 不同** |
| $e_{rel},e_{obj}$ | SE(3) 误差（6 维） | 概念一致 |
| $J_{rel},J_{obj}$ | 雅可比（6×12） | 第六课 $J_{rel}$ 一致 |
| $s_{rel},s_{obj}$ | 松弛（6 维） | 第七课一致 |
| $b_{rel},b_{obj}$ | 最小二乘偏移（代码 `bias`） | **与第六课 $J\Delta q=b$ 的 $b$ 含义不同**；后者对应本讲义的 $\alpha e$ |
| $w_{rel},w_{obj},w_{slack},w_{slack,obj},w_{center},w_{reg}$ | 权重 | 概念一致 |
| $\alpha_{rel}=k_{p,rel}dt$，$\alpha_{obj}=k_{p,obj}dt$ | 误差衰减系数 | 代码 `alpha_rel/alpha_obj` |
| $v_{ff}$ | 前馈 twist（代码 `body_twist_ff`） | 第七课一致 |
| $f\in[0,1]$ | 连续退化总因子 | 第七课一致 |
| $N_\lambda$ | 阻尼零空间投影矩阵 | 第六课一致 |
| $C_{\text{ineq}},d_{\text{ineq}}$ | 通用不等式接口 | 第七课一致 |

---

## 零、总览：这个 QP 里到底有什么

代码文件 `dual_arm_qp.cpp` 的 `solveImpl()` 构建了如下 QP：

$$
\boxed{
\begin{aligned}
\min_z\ & \tfrac12 z^\top P z + c^\top z\\
\text{s.t.}\ & l\le A z\le u
\end{aligned}}
$$

其中：

$$
z=\begin{bmatrix}\Delta q_{12}\\ s_{rel,6}\\ s_{obj,6}\end{bmatrix}\in\mathbb R^{24}
$$

**QP 的组成部分（按代码顺序）**：

| 部分 | 数学作用 | 代码位置 |
|---|---|---|
| ① 两个主任务代价 | $P,c$ 主项 | 段 6–8 |
| ② 连续奇异退化 | 调 $w_{reg,eff},w_{slack,eff},v_{scale}$ | 段 3–4 |
| ③ 零空间次级优化 | 生成 $dq_{\text{pref}}$，进 $P,c$ | 段 9–10 |
| ④ 正则/中心/松弛 | $P$ 对角 + $c$ 线性项 | 段 10 |
| ⑤ box 约束 | $A$ 顶部 12 行 | 段 11–12 |
| ⑥ 通用不等式 $C_{\text{ineq}}$ | $A$ 底部 $k$ 行 | 段 12 |
| ⑦ 求解与拆解 | 24 维解 → dq/slack | 段 13–14 |
| ⑧ 安全 clamp（仅 $k=0$） | 后处理 | 段 15 |
| ⑨ warm start | 下次初值 | 段 16 |
| ⑩ 输入校验/显式失败 | `rejectResult` | 段 0、外部行入口 |

**不在本文件、但常被混淆的机制**：

| 机制 | 实际位置 | 说明 |
|---|---|---|
| CBF 约束的**生成** | 外部调用方 | 本文件只提供 `C_ineq/d_ineq` 接口，**不生成 CBF** |
| D7 健康监控 / HOLD | `dual_arm_qp_controller.cpp` + `stream_health_monitor.hpp` | 不在 QP Core |
| 碰撞硬停 | `dual_arm_qp_controller.cpp` | 独立于 QP 的安全层 |
| 设备急停 | 硬件安全系统 | 不应由 QP 承担 |

**关键设计**：决策变量**永远 24 维**，物体任务关掉时不是减少变量，而是把 $w_{obj}$ 设 0。这是为了让 OSQP workspace 常驻（文件头注释）。

---

## 一、系统状态与正运动学

12 维关节角，左右各 6：

$$
q=\begin{bmatrix}q_L\\q_R\end{bmatrix}\in\mathbb R^{12}
$$

正运动学给出两臂末端在世界系下的位姿：

$$
{}^WT_L(q_L),\quad {}^WT_R(q_R)\in SE(3)
$$

代码：`kin_.leftTright(q)`、`kin_.worldTframe(Arm::Left, ql, idx)`。**这是所有任务的地基**。

---

## 二、主任务一：相对位姿（左臂相对右臂）

### 2.1 位姿定义

$$
X(q)={}^WT_L^{-1}\,{}^WT_R\in SE(3)
$$

`kin_.leftTright(q)` 算的就是这个 $X$。控制器在 `on_activate` 时捕获 $X_0=X(q_0)$ 作为目标 `target_`——**保持激活瞬间的两手相对位姿**。

### 2.2 误差定义

$$
e_{rel}=\log\!\big(X^{*-1}X\big)^\vee\in\mathbb R^6
$$

平移 3 维 + 旋转 3 维（so(3) 对数/轴角）。$e_{rel}=0\iff X=X^*$。取 $X^{*-1}X$ 意味着误差在右臂体坐标系表达，对应 `RelativeFrame::BodyR`。

### 2.3 相对雅可比推导

对 $X=T_L^{-1}T_R$ 求时间导数，体速度约定 $\dot T=T[\omega]^\wedge$：

$$
v_{rel}=\big(X^{-1}\dot X\big)^\vee=v_R^R-\mathrm{Ad}_{X^{-1}}v_L^L
$$

代入 $v_L^L=J_L^L(q_L)\dot q_L$、$v_R^R=J_R^R(q_R)\dot q_R$：

$$
\boxed{
v_{rel}=\underbrace{\big[\,-\mathrm{Ad}_{X^{-1}}J_L^L\;\big|\;J_R^R\,\big]}_{J_{rel}\in\mathbb R^{6\times12}}
\begin{bmatrix}\dot q_L\\\dot q_R\end{bmatrix}
}
$$

`kin_.relativeJacobian(q, BodyR)` 返回的就是 $J_{rel}$。两个结构特征：左臂项**带负号**（$T_L$ 在"分母"上），且要乘 $\mathrm{Ad}_{X^{-1}}$ 转到 BodyR。

### 2.4 误差导数与右雅可比逆

严格地 $\dot e_{rel}=J_r^{-1}(e_{rel})v_{rel}$，$J_r$ 是 SE(3) 右雅可比。代码注释明确：**当前用近似 $\dot e_{rel}\approx v_{rel}$（$J_r^{-1}\approx I$），只在 $e$ 小时精确**；精确右雅可比逆列为 future work。这是工程简化，配合连续退化因子 $f$ 在大误差时降权威。

### 2.5 线性化成 QP 代价

希望 $\dot e_{rel}=-\alpha_{rel}e_{rel}$（$\alpha_{rel}=k_{p,rel}dt$），代入近似：

$$
J_{rel}\Delta q=\alpha_{rel}e_{rel}
$$

$J_{rel}$ 是 $6\times12$，可能行不满秩、可能和物体任务冲突。引入松弛 $s_{rel}\in\mathbb R^6$：

$$
J_{rel}\Delta q-s_{rel}=\alpha_{rel}e_{rel}
$$

矩阵形式：

$$
\underbrace{\begin{bmatrix}J_{rel}&-I_6&0\end{bmatrix}}_{A_{\text{task}}^{\text{rel}}}
\begin{bmatrix}\Delta q\\s_{rel}\\s_{obj}\end{bmatrix}
=\alpha_{rel}e_{rel}
$$

代码：`A_task_rel_ = [J_rel, -I6, 0]`，`bias_rel = -alpha_rel * e_rel`。**注意** $b_{rel}$ 是**最小二乘偏移**，与第六课 $J\Delta q=b$ 的 $b$ 含义不同——后者对应这里的 $\alpha e$（等式右端）。

---

## 三、主任务二：物体/协调位姿

### 3.1 物体位姿定义

双臂夹持同一物体时，物体世界位姿由末端位姿 + 标定抓取变换决定：

$$
{}^WT_O(q)={}^WT_L(q_L)\cdot{}^LT_O
\quad\text{或}\quad
{}^WT_O(q)=\tfrac12\big({}^WT_L{}^LT_O+{}^WT_R{}^RT_O\big)
$$

`kin_.worldTobject(q)` 算的是 ${}^WT_O$。具体取哪一臂/如何平均取决于 `grasp_geometry.hpp`。

### 3.2 误差与雅可比

$$
e_{obj}=\log\!\big({}^WT_O^{*-1}\,{}^WT_O\big)^\vee\in\mathbb R^6
$$

若物体位姿由左臂确定（${}^WT_O={}^WT_L{}^LT_O$）：

$$
v_O^O=\mathrm{Ad}_{{}^LT_O^{-1}}v_L^L
=\mathrm{Ad}_{{}^LT_O^{-1}}J_L^L(q_L)\dot q_L
\;\Longrightarrow\;
J_{obj}=\big[\,\mathrm{Ad}_{{}^LT_O^{-1}}J_L^L\;\big|\;0\,\big]
$$

两臂共同（平均）时 $J_{obj}$ 同时含左右两臂项。`kin_.objectJacobianBody(q)` 返回的就是 $J_{obj}$（Body 表示物体体坐标系）。

### 3.3 线性化成 QP 代价

希望 $\dot e_{obj}=-\alpha_{obj}e_{obj}$，叠加前馈 $dt\cdot v_{ff}$：

$$
J_{obj}\Delta q-s_{obj}=\alpha_{obj}e_{obj}+dt\,v_{ff}
$$

矩阵形式：

$$
\underbrace{\begin{bmatrix}J_{obj}&0&-I_6\end{bmatrix}}_{A_{\text{task}}^{\text{obj}}}
\begin{bmatrix}\Delta q\\s_{rel}\\s_{obj}\end{bmatrix}
=\alpha_{obj}e_{obj}+dt\,v_{ff}
$$

代码：`A_task_obj_ = [J_obj, 0, -I6]`，`bias_obj = -(alpha_obj*e_obj + dt*ff)`。

> **外部行版本（C4.3 S3 / R24）**：`ObjectTaskRow` 允许外部直接给 `J, e, body_twist_ff`，绕过内部运动学。代码用 `?:` 保证只求值一个分支：

```cpp
const Vec6 e_obj = use_row ? object_task_row->e : kin_.objectError(q, world_T_object_target);
const dual_arm::Mat6x12 J_obj = use_row ? object_task_row->J : kin_.objectJacobianBody(q);
const Vec6 ff = use_row ? object_task_row->body_twist_ff : object_body_twist_ff;
```

**绝不回退**：外部行非法时 `rejectResult` 显式失败（段 0），不会静默用内部模型。

---

## 四、主任务合成 QP

两个任务行叠起来（物体任务关掉时只有 6 行）：

$$
A_{\text{task}}=\begin{bmatrix}A_{\text{task}}^{\text{rel}}\\A_{\text{task}}^{\text{obj}}\end{bmatrix}
=\begin{bmatrix}J_{rel}&-I_6&0\\J_{obj}&0&-I_6\end{bmatrix},\qquad
b=\begin{bmatrix}b_{rel}\\b_{obj}\end{bmatrix}
=\begin{bmatrix}-\alpha_{rel}e_{rel}\\-(\alpha_{obj}e_{obj}+dt\,v_{ff})\end{bmatrix}
$$

目标 $\min\|A_{\text{task}}z+b\|^2$ 加权求和：

$$
\boxed{
\min_z\ \tfrac12 z^\top Pz+c^\top z,\quad
P=w_{rel}A_{rel}^\top A_{rel}+w_{obj}A_{obj}^\top A_{obj},\quad
c=w_{rel}A_{rel}^\top b_{rel}+w_{obj}A_{obj}^\top b_{obj}
}
$$

（省略系数 2，不影响 argmin。）代码段 8 那四行就是它。$w_{obj}=0$ 时只加相对项，对应 `object_on=false`。

---

## 五、连续奇异退化（C2.6）

### 5.1 三个健康因子

代码段 3 计算：

$$
\begin{aligned}
f_\sigma&=S\!\left(\frac{\sigma_{\min}-\sigma_{\text{critical}}}{\max(10^{-9},\sigma_{\text{safe}}-\sigma_{\text{critical}})}\right)\\
f_{e,rel}&=S\!\left(\frac{e_{\text{critical}}-\|e_{rel}\|}{\max(10^{-9},e_{\text{critical}}-e_{\text{safe}})}\right)\\
f_{e,obj}&=S\!\left(\frac{e_{\text{critical}}-\|e_{obj}\|}{\max(10^{-9},e_{\text{critical}}-e_{\text{safe}})}\right)
\end{aligned}
$$

其中 $S(t)=t^2(3-2t)$（代码 `smoothstep01`），两端导数为 0，保证 $C^1$ 平滑。$\sigma_{\min}$ 是 $J_{rel}$ 的最小奇异值（`sigmaMinRel` 用 `JacobiSVD`）。

### 5.2 合成与有效参数

$$
f=\min(f_\sigma,f_{e,rel},f_{e,obj})
$$

**取 min 的语义**：任一条件触发就退化（最保守）。

$$
\boxed{
\begin{aligned}
w_{reg,eff}&=w_{reg}\big(1+\text{damp}_{\max}(1-f)\big)\\
w_{slack,eff}&=w_{slack}\cdot\max\!\big(0.05,\,1-\text{slack}_{\text{relax}}(1-f)\big)\\
v_{scale}&=v_{\min}+(1-v_{\min})f
\end{aligned}}
$$

| 参数 | $f=1$（健康） | $f=0$（退化） | 作用 |
|---|---|---|---|
| $w_{reg,eff}$ | $w_{reg}$ | $w_{reg}(1+\text{damp}_{\max})$ | 阻尼放大，解更保守 |
| $w_{slack,eff}$ | $w_{slack}$ | $w_{slack}\cdot\max(0.05,1-\text{slack}_{\text{relax}})$ | 放松 slack 惩罚，允许任务误差 |
| $v_{scale}$ | 1 | $v_{\min}$ | 速度上限降低 |

### 5.3 关键取舍：**不降 $k_p$**

代码注释明确：不降反馈增益，因为"大误差时停推反而冻结误差"。降的是**权威**（阻尼、slack、速度），不是**纠偏意愿**。

### 5.4 两个必须知道的边界

**① $w_{obj}=0$ 时 $f_{e,obj}$ 仍参与 $\min$**：代码无条件算 `f_err_obj`，即使物体任务关掉。所以"关物体任务"≠"排除物体误差对退化因子的影响"。

**② 退化不修复 π 附近物体中点问题**：$\sigma_{\min}$ 病态（运动映射）与"相对旋转 π 导致旧物体测地中点非唯一"（姿态定义）是两类问题，后者需要改定义/微分，不是调 SVD 阈值。

---

## 六、零空间次级优化（C2.5）

### 6.1 三个偏好方向

代码段 9 先构造偏好方向：

$$
d=w_m\frac{g_m}{\|g_m\|}+w_\sigma\frac{g_\sigma}{\|g_\sigma\|}+w_c\frac{g_c}{\|g_c\|}
$$

（代码 `add` 先归一化再加权，跳过 $\|g\|\le10^{-12}$。）

| 权重 | 方向 | 数学 | 代码 |
|---|---|---|---|
| $w_m$ | 关节回中 | $g_{m,i}=-2(q_i-\text{mid}_i)/\text{range}_i^2$（$\sum((q_i-\text{mid}_i)/\text{range}_i)^2$ 的下降方向） | `marginDirection` |
| $w_\sigma$ | 远离奇异 | 中心差分近似 $\nabla\sigma_{\min}(J_{rel})$（上升方向） | `ascentDirection(sigmaMinRel)` |
| $w_c$ | 增大间隙 | 中心差分近似 $\nabla(\min_{i,j}\|p_{L,i}-p_{R,j}\|)$（上升方向） | `ascentDirection(clearanceProxy)` |

`clearanceProxy` 只比较左右各 4 个关键帧原点的 16 组距离，**是舒适度代理，不是安全判据**（源码注释明确）。

### 6.2 阻尼零空间投影

$$
N_\lambda=I-J_{\text{task}}^\top(J_{\text{task}}J_{\text{task}}^\top+\lambda^2I)^{-1}J_{\text{task}},\quad\lambda^2=10^{-8}
$$

其中

$$
J_{\text{task}}=\begin{cases}J_{rel}\in\mathbb R^{6\times12}, & \text{仅相对任务}\\ \begin{bmatrix}J_{rel}\\J_{obj}\end{bmatrix}\in\mathbb R^{12\times12}, & \text{两任务都开}\end{cases}
$$

**关键性质**：严格伪逆版 $JN=0$；阻尼版 $JN_\lambda\ne0$ 一般成立（有泄漏）。两个任务都开时 $J_{\text{task}}$ 是 $12\times12$，$N_\lambda\approx0$（无冗余），零空间逻辑基本失效——数学上合理。

### 6.3 缩放与进入 QP

$$
d_{\text{null}}=N_\lambda d,\qquad
dq_{\text{pref}}=\frac{\text{null\_step}}{\max(1,\|d_{\text{null}}\|)}\,d_{\text{null}}
$$

**注意缩放方式**：即使 $\|d_{\text{null}}\|<1$ 也乘 `null_step`，不是"超限才缩短"的常规裁剪（第六课文档特别指出）。

进入 QP（段 10）：

$$
P\mathrel{+}= (w_{center}+w_{reg,eff})\begin{bmatrix}I_{12}&0&0\\0&0&0\\0&0&0\end{bmatrix},\qquad
c\mathrel{-}= w_{center}\begin{bmatrix}dq_{\text{pref}}\\0\\0\end{bmatrix}
$$

展开等价于软惩罚 $\frac{w_{center}}{2}\|\Delta q-dq_{\text{pref}}\|^2+\frac{w_{reg,eff}}{2}\|\Delta q\|^2$。

**$w_{center}$ 远小于主任务权重**，所以主任务主导，中心项只在主任务不在乎的方向起作用。

**`nullspace_on=false` 时**：`dq_center_scratch_ = params_.dq_center`（直接配置值，不做投影）。

### 6.4 它不是严格 HQP

文档第六课明确：这是"近似零空间参考 + 单层加权 QP"，**不是**严格字典序 HQP。阻尼泄漏 + 有限权重 + 硬约束共同作用，主任务**可能**被拉偏。

---

## 七、正则、中心与松弛（段 10）

除了零空间项，还有：

$$
\begin{aligned}
P&\mathrel{+}= w_{slack,eff}\begin{bmatrix}0&0&0\\0&I_6&0\\0&0&0\end{bmatrix}
+w_{slack,obj}\begin{bmatrix}0&0&0\\0&0&0\\0&0&I_6\end{bmatrix}
\end{aligned}
$$

- $s_{rel}$ 的松弛惩罚用 $w_{slack,eff}$（随退化放松）；
- $s_{obj}$ 的松弛惩罚用**固定的** $w_{slack,obj}$（不随退化变）。

**为什么需要松弛**：主任务可能矛盾（相对位姿 + 物体位姿 + 关节限位），无松弛则 QP 无解。松弛让任务"尽量满足但有代价"，$w_{slack}$ 越大越不允许违反。代价是解出的 $s\ne0$ 意味着任务没完全达成，上层可监控。

---

## 八、硬约束

### 8.1 box 约束（段 11）

每个关节的 $\Delta q_i$ 同时满足速度限幅和位置限位：

$$
\boxed{
\Delta q_i\in\Big[\max\big(-v_{\max,i}v_{scale}dt,\ q_{\min,i}-q_i\big),\ \min\big(v_{\max,i}v_{scale}dt,\ q_{\max,i}-q_i\big)\Big]
}
$$

**$lo>hi$ 的兜底**（当前 $q_i$ 已越界时）：

$$
\Delta q_i=\begin{cases}-v_{\max,i}v_{scale}dt, & q_i>q_{\max,i}\\ +v_{\max,i}v_{scale}dt, & \text{否则}\end{cases}
$$

方向朝"回界内"，安全优先。

### 8.2 通用不等式 $C_{\text{ineq}}z\le d_{\text{ineq}}$（段 12）

$$
A=\begin{bmatrix}I_{12}&0\\C_{\text{ineq}}\end{bmatrix},\quad
l=\begin{bmatrix}l_{\Delta q}\\-\infty\end{bmatrix},\quad
u=\begin{bmatrix}u_{\Delta q}\\d_{\text{ineq}}\end{bmatrix}
$$

- 顶部 12 行：box 约束，$[I_{12},0,0]$ 作用在 $z$ 上就是 $\Delta q$；
- 底部 $k$ 行：$C_{\text{ineq}}\in\mathbb R^{k\times24}$，$d_{\text{ineq}}\in\mathbb R^k$，写成 $l=-\infty,u=d_{\text{ineq}}$ 即 $C_{\text{ineq}}z\le d_{\text{ineq}}$。

**这是通用槽位，不生成 CBF**。CBF 的一种填法（第七课）：

$$
C_{\text{CBF}}=\begin{bmatrix}-\nabla h(q)^\top&0_{1\times6}&0_{1\times6}\end{bmatrix},\quad
d_{\text{CBF}}=\gamma\Delta t\,h(q)
$$

**slack 列为 0**：CBF 不能被 slack 放松。但 QP 是耦合的，slack 变化会通过主任务行间接影响 $\Delta q$，所以"slack 列为 0"是必要条件，不是绝对隔离。

**识别条件**：`params_.C_ineq.cols() == kNumVars`（24）。**未完整验证** `d_ineq` 长度与 finite（第七课指出）。

---

## 九、求解、拆解与 warm start（段 13–16）

### 9.1 组装与求解

```cpp
last_problem_.P = P_;
last_problem_.q = q_cost_;   // 数学符号 c
last_problem_.A = A_scratch_;
last_problem_.l = l_scratch_;
last_problem_.u = u_scratch_;
if (warm_start_enabled_ && warm_start_.size() == kNumVars) {
    last_problem_.x0 = warm_start_;
}
const QpSolution solution = solver_->solve(last_problem_);
```

`solver_` 多态：`OsqpSolver` 或 `AdmmSolver`（`makeDefaultSolver`）。

### 9.2 拆解

```cpp
if (solution.x.size() == kNumVars) {   // 24
    result.dq = solution.x.segment<12>(0);
    result.slack = solution.x.segment<6>(kSRel0);      // 12..17
    result.slack_object = solution.x.segment<6>(kSObj0); // 18..23
} else if (solution.x.size() == 12) {
    result.dq = solution.x;   // 兼容只返回 dq 的 solver
}
```

### 9.3 安全 clamp（仅 $k=0$）

```cpp
if (k == 0) {
    for (int i = 0; i < 12; ++i) {
        result.dq[i] = std::min(upper_[i], std::max(lower_[i], result.dq[i]));
    }
}
```

**仅当没有通用不等式时**才 clamp $\Delta q$。有 $C_{\text{ineq}}$ 时强行 clamp 会破坏 $C_{\text{ineq}}z\le d_{\text{ineq}}$（$\Delta q$ 变了但 $s$ 没动，线性组合变了）。此时必须**独立复算** $\max_i(C_iz-d_i)_+$（第七课）。

### 9.4 warm start

```cpp
if (warm_start_enabled_) {
    if (warm_start_.size() != kNumVars) warm_start_.setZero(kNumVars);
    warm_start_.head<12>() = result.dq;
    warm_start_.segment<6>(kSRel0) = result.slack;
    warm_start_.segment<6>(kSObj0) = result.slack_object;
}
```

存**clamp 后**的 `dq`，与最终返回一致。

---

## 十、故障保护与输入校验（本文件内的部分）

### 10.1 显式失败 `rejectResult`（段 0）

```cpp
DualArmQpResult rejectResult(ObjectTaskRowCheck code) {
    DualArmQpResult r;
    r.dq.setZero(); r.slack.setZero(); r.slack_object.setZero();
    r.converged = false; r.iterations = 0; r.solve_time_ms = 0.0;
    r.object_task_external = true; r.object_task_check = code;
    return r;
}
```

**关键性质**：不分配、不抛异常、**不推进 warm start**、不修改成员。这是"显式失败优于静默回退"的工程原则。

### 10.2 外部行校验 `validateObjectTaskRow`

```cpp
if (!row.valid) return ObjectTaskRowCheck::NotValid;
if (!row.J.allFinite() || !row.e.allFinite() || !row.body_twist_ff.allFinite()) {
    return ObjectTaskRowCheck::NonFinite;
}
```

### 10.3 输入合法性检查

```cpp
if (!q.allFinite() || !left_T_right_target.matrix().allFinite() ||
    !dual_arm::isRigidTransform(left_T_right_target)) {
    return rejectResult(ObjectTaskRowCheck::BadInput);
}
```

### 10.4 box 约束矛盾的兜底

见 8.1 的 `lo>hi` 分支——当前关节越界时强制朝回界方向。

### 10.5 **不在本文件的保护**

| 机制 | 位置 | 作用 |
|---|---|---|
| D7 健康监控（非有限状态、stall 注入、跟踪误差、求解失败计数） | `stream_health_monitor.hpp` | 检测单臂/全局异常 |
| D7 HOLD（重发旧命令，冻结新增量） | `dual_arm_qp_controller.cpp` | 软件 hold，**≠ 急停** |
| 碰撞硬停（`collision_d_stop`） | `dual_arm_qp_controller.cpp` | 独立于 QP 的安全层 |
| `QpProblem::valid()`（矩阵尺寸、$l\le u$） | `admm_solver.cpp` | solver 层校验 |
| 设备急停 | 硬件安全系统 | 不应由 QP 承担 |

**`result.converged=false` 时**：controller 增加失败计数、重发旧命令、返回 `OK`（不是 `ERROR`）。所以"update 返回 OK"≠"没故障"（第七课）。

---

## 十一、完整 QP 汇总

把上面所有部分拼起来，`solveImpl` 构建的 QP 是：

$$
\boxed{
\begin{aligned}
\min_{z=[\Delta q;s_{rel};s_{obj}]}\quad
& \tfrac12 z^\top Pz+c^\top z\\
\text{s.t.}\quad
& \underbrace{\begin{bmatrix}I_{12}&0&0\\C_{\text{ineq}}\end{bmatrix}}_{A}z
\in
\underbrace{\begin{bmatrix}l_{\Delta q}\\-\infty\end{bmatrix}}_{l}
\le Az\le
\underbrace{\begin{bmatrix}u_{\Delta q}\\d_{\text{ineq}}\end{bmatrix}}_{u}
\end{aligned}}
$$

其中：

$$
\begin{aligned}
P&=\underbrace{w_{rel}A_{rel}^\top A_{rel}+w_{obj}A_{obj}^\top A_{obj}}_{\text{主任务}}
+\underbrace{(w_{center}+w_{reg,eff})I_{12}}_{\text{中心+正则}}\\
&\quad+\underbrace{w_{slack,eff}I_{s_{rel}}+w_{slack,obj}I_{s_{obj}}}_{\text{松弛惩罚}}\\
c&=\underbrace{w_{rel}A_{rel}^\top b_{rel}+w_{obj}A_{obj}^\top b_{obj}}_{\text{主任务线性项}}
-\underbrace{w_{center}\,dq_{\text{pref}}}_{\text{中心偏好}}
\end{aligned}
$$

**主任务关掉的机制**：$w_{obj}=0$ 时 $A_{\text{task}}^{\text{obj}}$ 不进 $P,c$（`if (object_on)`），但决策变量仍是 24 维、$A$ 形状不变。

---

## 十二、与代码变量对齐

| 数学量 | 代码变量 | 位置 |
|---|---|---|
| $P$ | `P_` | 段 8、10 |
| $c$ | `q_cost_` | 段 8、10 |
| $A_{\text{task}}^{\text{rel}}$ | `A_task_rel_` | 段 6 |
| $A_{\text{task}}^{\text{obj}}$ | `A_task_obj_` | 段 6 |
| $b_{rel},b_{obj}$ | `bias_rel, bias_obj` | 段 7 |
| $e_{rel},J_{rel}$ | `e_rel, J_rel` | 段 2 |
| $e_{obj},J_{obj}$ | `e_obj, J_obj` | 段 2、6 |
| $f_\sigma,f_{e,rel},f_{e,obj},f$ | `f_sigma, f_err_rel, f_err_obj, f` | 段 3 |
| $w_{reg,eff},w_{slack,eff},v_{scale}$ | `w_reg_eff, w_slack_eff, vel_scale` | 段 4 |
| $dq_{\text{pref}}$ | `dq_center_scratch_` | 段 9–10 |
| $J_{\text{task}},N_\lambda$ | `J_task, N` | 段 9 |
| $l_{\Delta q},u_{\Delta q}$ | `lower_, upper_` | 段 11 |
| $A,l,u$ | `A_scratch_, l_scratch_, u_scratch_` | 段 12 |
| 完整问题 | `last_problem_` | 段 13 |
| warm start | `warm_start_` | 段 16 |
| 显式失败 | `rejectResult` | 段 0 |
| 外部行校验 | `validateObjectTaskRow` | 段 0 |

---

## 十三、常见误区

**① 任务不是约束，是代价。** 主任务进 $P,c$（软），硬约束只有 box 和 $C_{\text{ineq}}$。$A_{\text{task}}$（任务行）与 $A$（约束矩阵）不是一回事。

**② 线性项符号是 $c$ 不是 $q$。** 代码变量名 `q_cost_` 是历史命名；数学上 $q$ 是关节角，线性项用 $c$。

**③ $w_{obj}=0$ 关任务，但 $f_{e,obj}$ 仍算。** "关物体任务代价"≠"排除物体误差对退化因子的影响"。

**④ 零空间是软偏好，不是严格零空间。** $N_\lambda$ 有阻尼泄漏，且最终 $\Delta q$ 由 QP 权衡，不保证 $J\Delta q$ 严格不变。

**⑤ `C_ineq` 是通用接口，不是 CBF 生成器。** 本文件只把行送进 solver，谁计算 $\nabla h$、谁判断最近碰撞点，在外部。

**⑥ 有 $C_{\text{ineq}}$ 时跳过 clamp，必须独立复算残差。** $\max_i(C_iz-d_i)_+$ 是约束最大违反量，不是关节速度 `v_max`。

**⑦ D7 HOLD ≠ 急停。** 重发旧命令，若 `q_cmd_` 已领先真实反馈，机器人仍可能向旧目标运动。软件 hold 不等于物理停机。

**⑧ 连续退化不降 $k_p$。** 降的是权威（阻尼、slack、速度），不是纠偏意愿。降 $k_p$ 会冻结误差。

**⑨ 本文件不是完整安全系统。** QP 收敛 ≠ 机器人安全。CBF 生成、D7、碰撞硬停、设备急停都在别处。

---

**一句话总结**：`solveImpl` 把"双臂相对位姿 + 物体位姿"两个 SE(3) 任务线性化成带松弛的二次代价，叠加连续奇异退化（调权重/速度）、零空间次级优化（三方向投影后软惩罚）、正则与松弛，再拼上 box 约束和通用不等式 $C_{\text{ineq}}$，解 24 维 QP 并拆回 $\Delta q/s_{rel}/s_{obj}$；输入非法时显式失败不推进 warm start，有通用不等式时跳过 clamp 需独立复算残差；CBF 生成、D7 健康监控、碰撞硬停、设备急停都不在本文件。
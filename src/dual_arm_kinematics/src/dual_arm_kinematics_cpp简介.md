# `dual_arm_kinematics.cpp` 逐函数讲解 + 全部雅可比清单

先给结论：**这份文件是"运动学层"的全部函数，包含 6 个位姿/FK 函数、7 个雅可比函数、2 个误差函数、4 个工具函数。但有一个关键发现——代码里的 `objectJacobianBody` 用的是"两臂中点法"，不是 5.8 节推导的"左锚 $J_{O,L}$"。** 这是与文档最大的差异，下面重点讲。

---

## 一、先分类：这个文件里有什么

| 类别 | 函数 | 作用 |
|---|---|---|
| **构造/安装** | `defaultWorldTleftBase`, `defaultWorldTrightBase`, 3 个构造函数 | 默认 base 安装 + DH + 工具 |
| **正运动学（FK）** | `baseTgrasp`, `worldTgrasp`, `baseTframe`, `worldTframe` | 单臂 FK（grasp / 任意 frame） |
| **位姿组合** | `worldTobject`, `leftTright`（2 重载） | 物体位姿（中点）、相对位姿 |
| **单臂雅可比** | `jacobianBase`, `jacobianBody`, `spatialJacobianWorld`, `jacobianWorldPoint` | 4 种单臂雅可比 |
| **物体雅可比** | `objectJacobianWorld`, `objectJacobianBody` | 中点法物体雅可比 |
| **相对雅可比** | `relativeJacobian`（3 种 frame） | 相对任务雅可比 |
| **误差** | `objectError`, `relativeError` | SE(3) 对数误差 |
| **误差雅可比** | `objectErrorJacobian`, `relativeErrorJacobian` | 小误差近似 |
| **工具** | `toIso`, `stack`, `split` | 类型转换、12 维拼接拆分 |

---

## 二、逐函数讲解

### 2.1 构造与安装

**`toIso`**：Eigen 4×4 矩阵 → `Iso3`（Eigen 的 SE(3) 类型）。纯类型转换。

**`defaultWorldTleftBase`**：
$$
{}^WT_{B_L}: p=(0.35,0,0),\ R=R_z(\pi)
$$
左 base 在世界系 $x=0.35$ 处，**绕 $z$ 转 180°**。

**`defaultWorldTrightBase`**：
$$
{}^WT_{B_R}: p=(-0.35,0,0),\ R=I
$$
右 base 在 $x=-0.35$ 处，**不转**。

**关键**：两臂 base 朝向**不同**（左转 180°，右不转）。这就是文档 1.3 节强调的"两臂 base 轴向并不相同，直接把两侧 base 中表达的 Jacobian 相减是错误的"。

**构造函数**：默认用上述安装 + `DHParams{}` + `ToolParams::gripper()`，建立两个 `DHKinematics`（左右各一）。

---

### 2.2 正运动学（FK）

**`baseTgrasp(Arm a, q)`**：
$$
{}^{B_a}T_L=\text{DH FK}(q)
$$
单臂 base 到 grasp 的 FK。

**`worldTgrasp(Arm a, q)`**：
$$
{}^WT_L={}^WT_{B_a}\cdot{}^{B_a}T_L
$$
world 到 grasp，**左乘 base 安装**。这是所有 world 系量的地基。

**`baseTframe(Arm a, q, idx)`**：base 到第 `idx` 个 frame（0~6）的 FK。`fkineAll` 返回所有中间 frame，用于零空间 `clearanceProxy` 比较关键点距离。

**`worldTframe(Arm a, q, idx)`**：world 到第 `idx` 个 frame，`base_[a] * baseTframe`。

---

### 2.3 位姿组合

**`leftTright(q12)`**（两重载，最终都调 6 维版本）：
$$
T_{LR}={}^WT_L^{-1}\,{}^WT_R
$$
**左臂看右臂的相对位姿**，任务一的目标就是这个。

**`worldTobject(q12)`**：**这是中点法，不是左锚**：
$$
\begin{aligned}
p_O&=\tfrac12(p_L+p_R) &&\text{平移中点}\\
R_O&=R_L\exp\!\Big(\tfrac12\log(R_L^\top R_R)\Big) &&\text{旋转测地中点}
\end{aligned}
$$
这正是文档 5.1 节说的"**历史 C2 协调/中点物体系**"，**不是 C4.3 的左锚 $T_O=T_LG_L^{-1}$**。文档明确警告：**当相对旋转接近 $\pi$ 时，测地中点不唯一，Jacobian 附近会失真**。

---

### 2.4 单臂雅可比（4 种）

**`jacobianBase(Arm a, q)`**：
$$
J_{\text{base}}=\text{DH jacobian}(q)\in\mathbb R^{6\times6}
$$
DH 直接给出的 base 系雅可比。

**`jacobianBody(Arm a, q)`**：**这就是 $J_{bL},J_{bR}$**：
$$
J_{b}=R6_{gb}\cdot J_{\text{base}},\qquad
R6_{gb}=\mathrm{diag}(R_{gb},R_{gb}),\ R_{gb}=R_{\text{grasp}}^\top
$$
把 base 系 twist 旋转到 **grasp 体坐标系**，即 $V_b=J_b\dot q$。这就是 5.8 节的 $J_{bL}$。

**`spatialJacobianWorld(Arm a, q)`**：
$$
J_s=\operatorname{Ad}_{{}^WT_L}\cdot J_b
$$
**spatial twist，世界轴，参考世界原点**。注意参考点是世界原点，不是 grasp 原点——这是 spatial twist 的标准定义（$\dot TT^{-1}$）。

**`jacobianWorldPoint(Arm a, q)`**：
$$
J_w=R6_{wg}\cdot J_b,\qquad R6_{wg}=\mathrm{diag}(R_{wg},R_{wg})
$$
**世界轴，但参考点仍是 grasp 原点**。它把 body twist 的两个 3 维分量分别旋转到世界轴，**不加平移偏置**。所以它**不是** spatial twist——不要和 `spatialJacobianWorld` 混淆。

**四者区别表**：

| 函数 | 参考点 | 轴 | 对应 twist |
|---|---|---|---|
| `jacobianBase` | base 原点 | base 轴 | base 系 twist |
| `jacobianBody` | grasp 原点 | grasp 轴 | **body twist** $V_b$ |
| `spatialJacobianWorld` | **世界原点** | 世界轴 | **spatial twist** $V_s=\operatorname{Ad}_{{}^WT}V_b$ |
| `jacobianWorldPoint` | **grasp 原点** | 世界轴 | "world-aligned body twist" |

---

### 2.5 物体雅可比（中点法，不是左锚）

**`objectJacobianWorld(q12)`**：**核心差异在这里**。

它用 `jacobianWorldPoint`（世界轴、grasp 原点参考）组合：

$$
J_{\text{obj,world}}=\begin{bmatrix}
\tfrac12 J_{L,\text{lin}} & \tfrac12 J_{R,\text{lin}}\\[4pt]
(I-G)J_{L,\text{ang}} & G J_{R,\text{ang}}
\end{bmatrix}
$$

其中

$$
G=R_L\cdot J_l^{SO(3)}(\tfrac12\xi)\cdot\tfrac12 J_l^{-1}(\xi)\cdot R_L^\top,\qquad
\xi=\log(R_L^\top R_R)
$$

**这就是中点法的 Jacobian**：
- 平移：两 grasp 原点速度的**平均**（$0.5 J_L+0.5 J_R$）；
- 旋转：$w_O=w_L+G(w_R-w_L)$，$G$ 是从相对旋转的 log 修正来的。当 $\xi\to0$（两臂旋转一致）时 $G\to\tfrac12 I$，退化为平均；$\xi$ 大时 $G$ 偏离 $\tfrac12 I$。

**对比 5.8 节的左锚**：

$$
J_{O,L}^{\text{5.8}}=\big[\operatorname{Ad}_{G_L}J_{bL}\ \big|\ 0\big]
$$

**这是完全不同的公式**：

| 维度 | 5.8 左锚 | 代码中点法 |
|---|---|---|
| 依赖 | 只依赖左臂（右半零） | 两臂都依赖 |
| 物体位姿 | $T_O=T_LG_L^{-1}$ | $T_O=$ 两臂中点 |
| 旋转处理 | 无（纯 Adjoint） | 测地中点 + $G$ 修正 |
| $\pi$ 附近 | 无问题 | **失真**（5.1 节警告） |
| 用在哪 | C4.3 外部 `ObjectTaskRow` | 历史 C2 旧接口 |

**所以：代码里的 `objectJacobianBody` 不是 5.8 节的 $J_{O,L}$。** 5.8 节的左锚 Jacobian **不在这份文件里**——它是 C4.3 通过外部 `ObjectTaskRow.J` 提供的，或者未来实现。

**`objectJacobianBody(q12)`**：

```cpp
Mat3 R_wo_T = worldTobject(q12).linear().transpose();
J.topRows<3>()    = R_wo_T * J.topRows<3>();
J.bottomRows<3>() = R_wo_T * J.bottomRows<3>();
```

把 `objectJacobianWorld` 的**世界轴**旋转到**物体体坐标系**（乘 $R_{wo}^\top$）。参考点仍是物体原点（中点）。这就是任务二 QP 里用的 `J_obj` 的默认来源——**但它基于中点法，不是左锚**。

---

### 2.6 误差与误差雅可比

**`objectError(q12, target)`**：
$$
e_{obj}=\log\big(T_O^{-1}T_O^*\big)^\vee
$$
`worldTobject`（中点）的逆乘目标，取 log。与文档 5.9 节一致。

**`relativeError(q12, target)`**：
$$
e_{rel}=\log\big(T_{LR}^{-1}T_{LR}^*\big)^\vee
$$
与文档 2.2 节一致。

**`objectErrorJacobian` 和 `relativeErrorJacobian`**：都是小误差近似：

```cpp
return -adjoint(T_err.inverse()) * J;
```

数学形式：

$$
\frac{\partial e}{\partial q}\approx-\operatorname{Ad}_{T_{\text{err}}^{-1}}J
$$

**对比 5.11 节的严格式**：

$$
\frac{\partial e}{\partial q}=-[J_l^{SE(3)}(e)]^{-1}J_{O,L}
$$

代码用 $\operatorname{Ad}_{T_{\text{err}}^{-1}}$ 近似 $J_l^{-1}$。严格说两者一阶项差 2 倍（$\operatorname{Ad}_{\exp(-e)}\approx I-\operatorname{ad}_e$，$J_l^{-1}\approx I-\tfrac12\operatorname{ad}_e$），所以这也是**工程近似**，代码注释明确写"small-error approximation, validated numerically"。

**注意**：这两个误差雅可比函数**当前 `dual_arm_qp.cpp` 没有调用**。QP 里用的是 `J_rel`（通过 `relativeJacobian`）和 `J_obj`（`objectJacobianBody` 或外部行），然后自己乘 $\alpha e$。误差雅可比是**备用/诊断**接口。

---

### 2.7 相对雅可比：3 种 frame

**`relativeJacobian(q12, frame)`**：

**BodyR**（QP 用的就是这个）：
$$
J_{rel}^{BodyR}=\big[-\operatorname{Ad}_{T_{LR}^{-1}}J_{bL}\ \big|\ J_{bR}\big]
$$
与文档 3.1 节完全一致。

**LeftL**：
$$
J_{rel}^{LeftL}=\big[-J_{bL}\ \big|\ \operatorname{Ad}_{T_{LR}}J_{bR}\big]
$$
在左 grasp 系表达。相对 BodyR 左乘了一个 $\operatorname{Ad}_{T_{LR}}$（换系）。

**WorldSpatial**：
$$
J_{rel}^{WorldSpatial}=\big[-J_{sL}\ \big|\ J_{sR}\big],\quad J_s=\texttt{spatialJacobianWorld}
$$
**spatial twist 相减**，世界系、世界原点参考。

三者是**同一个相对运动在不同参考系下的表达**，物理等价（差一个 Adjoint），数学上 $J_{rel}^{BodyR}$ 和 $J_{rel}^{LeftL}$ 满足 $J^{LeftL}=\operatorname{Ad}_{T_{LR}}J^{BodyR}$。

---

### 2.8 工具

**`stack`**：左 6 + 右 6 → 12 维。
**`split`**：12 维 → 左右各 6。

---

## 三、全部雅可比清单（这个文件里）

| # | 函数 | 尺寸 | 映射 | 参考点/轴 | 用在哪 |
|---|---|---|---|---|---|
| 1 | `jacobianBase` | 6×6 | $\dot q_L\to$ base twist | base 原点/base 轴 | 内部基础 |
| 2 | `jacobianBody` | 6×6 | $\dot q_L\to V_{bL}$ | **grasp 原点/grasp 轴** | **$J_{bL},J_{bR}$ 基础块** |
| 3 | `spatialJacobianWorld` | 6×6 | $\dot q_L\to V_{sL}$ | **世界原点/世界轴** | `relativeJacobian` 的 WorldSpatial 分支 |
| 4 | `jacobianWorldPoint` | 6×6 | $\dot q_L\to$ world-aligned body twist | grasp 原点/世界轴 | `objectJacobianWorld` 内部 |
| 5 | `objectJacobianWorld` | 6×12 | $\dot q\to$ 物体 twist（中点） | 物体中点/世界轴 | `objectJacobianBody` 内部 |
| 6 | `objectJacobianBody` | 6×12 | $\dot q\to$ 物体 twist（中点） | **物体中点/物体体轴** | **任务二 $J_{obj}$ 默认来源** |
| 7 | `objectErrorJacobian` | 6×12 | $\dot q\to\dot e_{obj}$（近似） | — | 备用/诊断（QP 未用） |
| 8 | `relativeJacobian(BodyR)` | 6×12 | $\dot q\to V_{rel}^R$ | 右 grasp 体轴 | **任务一 $J_{rel}$** |
| 9 | `relativeJacobian(LeftL)` | 6×12 | $\dot q\to V_{rel}^L$ | 左 grasp 体轴 | 备用 |
| 10 | `relativeJacobian(WorldSpatial)` | 6×12 | $\dot q\to V_{rel}^s$ | 世界原点/世界轴 | 备用 |
| 11 | `relativeErrorJacobian` | 6×12 | $\dot q\to\dot e_{rel}$（近似） | — | 备用/诊断（QP 未用） |

**共 11 个雅可比函数，但很多是内部工具或备用**。QP 实际只用：
- **任务一**：`relativeJacobian(BodyR)` → $J_{rel}$
- **任务二**：`objectJacobianBody` → $J_{obj}$（中点法默认），或外部 `ObjectTaskRow.J`

---

## 四、关键发现：代码 vs 文档 5.8 节的差异

这是必须强调的：

**① 代码的 `objectJacobianBody` 是"中点法"，不是 5.8 节的左锚 $J_{O,L}$**

- 代码：$T_O$ = 两臂中点（平移平均、旋转测地中点），$J_{obj}$ 依赖两臂。
- 5.8 节：$T_O=T_LG_L^{-1}$，$J_{O,L}=[\operatorname{Ad}_{G_L}J_{bL},0]$，只依赖左臂。

**两者完全不同**。代码用的是文档 5.1 节警告过的"历史 C2 中点法"，**不是 C4.3 的左锚**。

**② 5.8 节的 $J_{O,L}$ 不在这份文件里**

这份文件只提供中点法。C4.3 的左锚 Jacobian 需要通过：
- **外部 `ObjectTaskRow.J`** 提供（调用方自己算左锚）；
- 或**未来实现**（当前代码没有）。

**③ 文档 5.1 节明确说"旧中点不能作为 C4.3 默认"**

> "此前 S0 审计发现：当两手的相对旋转恰好 $\pi$ 时，旧对称旋转测地中点不唯一，旧物体 Jacobian 在附近可能失真。**因此旧中点不能作为 C4.3 双端抓持的默认物体控制定义。**"

代码里 `objectJacobianBody` 就是"旧中点"。所以 C4.3 用 `ObjectTaskRow` 外部行绕过它——这解释了为什么需要外部接口。

**④ `objectErrorJacobian` 和 `relativeErrorJacobian` 是近似，QP 未用**

QP 用的是运动 Jacobian（`J_rel`, `J_obj`）配 $\alpha e$，不是误差 Jacobian。误差 Jacobian 用 $\operatorname{Ad}_{T_{err}^{-1}}$ 近似 $J_l^{-1}$，一阶项差 2 倍，是"validated numerically"的工程近似。

---

## 五、回答"是不是全部"

**是，也不是**：

**是**：从**运动学层**看，这份文件提供了所有"单臂/物体/相对/误差"的雅可比，没有遗漏。

**不是**：有几点要注意：

1. **5.8 节的左锚 $J_{O,L}$ 不在**——这是 C4.3 需要的外部行，不在这份文件。
2. **上层可能再组合**：`dual_arm_qp.cpp` 里的 $A_{\text{task}}^{\text{rel}}=[J_{rel},-I,0]$ 是把 $J_{rel}$ 拼上松弛块；$A_{\text{task}}^{\text{obj}}=[J_{obj},0,-I]$ 同理。这些"任务行"不是雅可比，但由雅可比拼成。
3. **外部 `ObjectTaskRow.J` 可能不是这个文件算的**——调用方可以提供任意 6×12（左锚、右锚、融合、数值差分）。
4. **零空间用的 $J_{\text{task}}$** 是把 `J_rel` 和 `J_obj` 上下拼成 $6\times12$ 或 $12\times12$，不是新雅可比，是组合。

---

## 六、一张图总结

```
单臂 FK
  baseTgrasp → worldTgrasp
        │
        ├── jacobianBase（base 系）
        │       │
        │       ▼
        │   jacobianBody = J_bL, J_bR（grasp 体轴）
        │       │
        │       ├── spatialJacobianWorld = Ad_{WT} J_b（spatial twist）
        │       └── jacobianWorldPoint = R6_{wg} J_b（世界轴，grasp 原点）
        │
        ├── relativeJacobian(BodyR) = [-Ad_{T_LR^{-1}} J_bL | J_bR]  ← 任务一
        │
        ├── objectJacobianWorld（中点法：0.5 平移平均 + G 旋转修正）
        │       │
        │       ▼
        │   objectJacobianBody（转到物体体轴）  ← 任务二默认（中点，非左锚）
        │
        └── 误差：objectError / relativeError
                │
                ▼
            objectErrorJacobian / relativeErrorJacobian（近似，QP 未用）

5.8 节的左锚 J_O,L = [Ad_{G_L} J_bL | 0] —— 不在这份文件，靠外部 ObjectTaskRow.J
```

---

## 七、一句话回答

> **这份文件是运动学层的全部函数：4 个 FK（`baseTgrasp/worldTgrasp/baseTframe/worldTframe`）、2 个位姿组合（`worldTobject` 中点法、`leftTright` 相对）、11 个雅可比（4 种单臂 + 2 种物体 + 3 种相对 frame + 2 个误差雅可比）、2 个误差、3 个工具。关键发现：① 代码的 `objectJacobianBody` 是"两臂中点法"（平移平均 + 旋转测地中点 + $G$ 修正），**不是** 5.8 节推导的左锚 $J_{O,L}=[\operatorname{Ad}_{G_L}J_{bL},0]$——左锚不在这份文件，靠外部 `ObjectTaskRow.J` 提供；② 这正是文档 5.1 节警告的"旧中点"，$\pi$ 附近会失真，所以 C4.3 要用外部行绕过它；③ QP 实际只用 `relativeJacobian(BodyR)` 和 `objectJacobianBody`（或外部行），其余雅可比是内部工具或备用/诊断；④ 误差雅可比（`objectErrorJacobian`/`relativeErrorJacobian`）用 $\operatorname{Ad}_{T_{err}^{-1}}$ 近似 $J_l^{-1}$，是工程近似，QP 未调用。**
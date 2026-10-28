# 第七课：双臂安全约束、CBF 与故障保护

**S622 双臂 QP 系统 · 第七课专题讲义**

> 本文是独立数学与工程原理课程，不是 C++ 函数参数字典，也不是已经完成真机安全验收的报告。
>
> 参考已核对的仓库 `Ahhyp/s622_dual` `public` 分支：
> - [`dual_arm_qp.hpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/dual_arm_qp.hpp)
> - [`dual_arm_qp.cpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/dual_arm_qp.cpp)
> - [`stream_health_monitor.hpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_controller/include/dual_arm_qp_controller/stream_health_monitor.hpp)
> - [`dual_arm_qp_controller.cpp`](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_controller/src/dual_arm_qp_controller.cpp)
>
> 为避免混淆，本文用**[源码事实]**标记能从上述文件核对的行为，用**[数学推导]**表示通用理论，用**[建议验证]**表示尚需测试的内容。

## 学习目标

读完后应能：独立从安全距离推导 CBF 条件、把条件变成当前 24 维 QP 的约束行；理解碰撞距离 Jacobian、离散时间误差；解释 D7 软件 hold 的能力与局限；制定测试计划。

---

# 第一章：为什么相对位姿控制不等于安全控制？

你的双臂协同任务希望两只夹爪维持：

$$
{}^LT_R(q)\approx{}^LT_R^*.
$$

但两手保持固定相对位姿，并不保证：

- 左右两臂连杆互不碰撞；
- 箱子不碰桌面；
- 关节跟踪没有滞后；
- 某一臂 ServoJ stall 时另一臂不会继续追赶旧目标；
- 气动夹爪的抓持力足够。

因此要区分三个层次：

| 机制 | 目标 | 不能替代 |
|---|---|---|
| 碰撞 CBF | 通过不等式约束已建模的几何危险 | 接触力反馈与完整设备安全 |
| D7 健康互锁 | 发现单臂或全局执行异常，冻结正常目标更新 | 设备级急停 |
| 硬件安全系统 | 限位、保护停止和故障安全机制 | 不应由普通 QP 独自承担 |

**[源码事实]**`dual_arm_qp.cpp` 有 `clearanceProxy()`：只测左右各四个关键帧原点的 16 组距离。这是零空间的舒适度偏好，源码明确说不是安全判据。偏好权重即使很大，也不能保证所有真实碰撞几何都安全。

---

# 第二章：用函数表示“安全”

## 2.1 定义安全集合

**[数学推导]**令 $d(q)$ 是机械臂与障碍之间经过验证的安全距离，$d_{\rm safe}$ 是要求的最小裕度。定义：

$$
\boxed{h(q)=d(q)-d_{\rm safe}}.
$$

安全集合：

$$
\boxed{\mathcal C=\{q\mid h(q)\ge0\}}.
$$

于是：

- $h>0$：模型认为还有裕量；
- $h=0$：恰好在安全边界；
- $h<0$：违反定义的安全裕量。

这不是全系统安全证明：它只涵盖 **h 这个函数实际建模的碰撞类型**。

## 2.2 一维例子

假设障碍物在右侧，机器人只能位于 $x\le0.9$ m。取：

$$
h(x)=0.9-x.
$$

若 $x=0.85$，则 $h=0.05$ m。如果继续朝右运动，h 下降。

仅检查“这一刻 h 大于 0”不够，因为很快就可能越界。因此还要约束 **h 的变化速度**。

---

# 第三章：推导 Control Barrier Function

**[数学推导]**假定控制输入是关节速度：

$$
\dot q=u.
$$

对可微函数 $h(q)$ 用链式法则：

$$
\dot h(q)=\nabla h(q)^\top \dot q.
$$

常见的一阶 CBF 条件为：

$$
\boxed{\dot h(q)+\gamma h(q)\ge0},\qquad\gamma>0.
$$

于是：

$$
\boxed{\nabla h(q)^\top u\ge-\gamma h(q)}. \tag{1}
$$

### 3.1 为什么它可能保证不越过安全边界？

如果每个时刻都能确保：

$$
\dot h\ge-\gamma h,
$$

则比较原理给出：

$$
h(t)\ge h(0)e^{-\gamma t}.
$$

当初始 $h(0)\ge0$ 时，理想连续系统始终满足 $h(t)\ge0$。

但这个结论依赖真实运动满足模型、h 可微、控制输入可行、无未建模扰动与采样问题等前提。**把公式放进 QP 不等于自动获得真实机器人的安全保证。**

### 3.2 gamma 是什么？

$\gamma$ 的单位是 $\mathrm{s}^{-1}$。它控制安全裕量允许多快下降：

- $\gamma$ 小：通常更保守；
- $\gamma$ 大：在安全区内部允许更快靠近边界。

需要实际评估选择，不是越大越安全。

---

# 第四章：S622 优化的是关节增量，怎么变成 QP 行？

你的 QP 不是直接解 $\dot q$，而是求一个周期内的增量：

$$
\Delta q\in\mathbb R^{12},\qquad \dot q\approx\frac{\Delta q}{\Delta t}.
$$

将其代入 (1)：

$$
\nabla h(q)^\top\frac{\Delta q}{\Delta t}\ge-\gamma h(q).
$$

因为 $\Delta t>0$，所以：

$$
\boxed{\nabla h(q)^\top\Delta q\ge-\gamma\Delta t\,h(q)}. \tag{2}
$$

**[源码事实]**当前 QP 统一使用 24 维决策向量：

$$
z=
\begin{bmatrix}
\Delta q_{12}\\s_{\rm rel,6}\\s_{\rm obj,6}
\end{bmatrix}
\in\mathbb R^{24}.
$$

`DualArmQpParams` 提供额外不等式：

$$
C_{\rm ineq}z\le d_{\rm ineq}.
$$

这个不等式， `C_ineq` 是一个通用的 $k \times 24$的槽位， 任何关于 $[ \Delta q; s_{rel}; s_{obj}]$的线性不等式都能装下。举例子（仅仅是实例， 而不是当前实现）：
| 用途 | `C_ineq`行 | `d_ineq` |
| -- | -- | -- |
| 碰撞 CBF | $[-\Delta h^T, 0, 0]$ | $\gamma \Delta t h$ | 
|任务误差上界 | $[J_{rel}, -I, 0]$ | 与 $e$ 相关的界 |
|关节耦合约束 | 任意 12 维的线性组合 + 0 | 常数 |
所以， Core 只是保证这些行能够被送进 solver， 而不假设予语义。语义的正确性是调用方的责任。

将式 (2) 两边乘 -1：

$$
-\nabla h(q)^\top\Delta q\le\gamma\Delta t\,h(q).
$$

装成一行 24 列：

$$
\boxed{
C_{\rm CBF}=
\begin{bmatrix}
-\nabla h(q)^\top &0_{1\times6}&0_{1\times6}
\end{bmatrix}
}
$$

$$
\boxed{d_{\rm CBF}=\gamma\Delta t\,h(q)}.
$$

然后：

$$
\boxed{C_{\rm CBF}z\le d_{\rm CBF}}.
$$

**为什么 slack 列为 0？**因为此处希望碰撞条件是硬约束；不应该悄悄允许相对位姿或物体任务的 slack 放松安全要求。

### 4.1 多个碰撞对

每个可靠、可微的安全函数 $h_i(q)$ 各生成一行。若总共 k 行：

$$
C_{\rm ineq}\in\mathbb R^{k\times24},\quad d_{\rm ineq}\in\mathbb R^k.
$$

注意重点不在拼矩阵，而在安全距离、梯度与有效碰撞对的可靠性。

---

# 第五章：手算例子，看清 CBF 正负号

## 5.1 一维接近障碍物

要求：

$$
x\le0.9\ \mathrm m,\quad h(x)=0.9-x,\quad\nabla h=-1.
$$

当前 $x=0.85$ m，因此 $h=0.05$ m。令：

$$
\Delta t=0.008\ \mathrm s,\quad\gamma=2\ \mathrm s^{-1}.
$$

式 (2) 成为：

$$
-\Delta x\ge-2(0.008)(0.05).
$$

得到：

$$
\boxed{\Delta x\le0.0008\ \mathrm m}.
$$

如果任务希望本轮前进 0.003 m，就不能同时满足这个硬 CBF；QP 必须减小前进量或修改任务。

这只是**一阶控制约束教学示例**，不是 S622 的实际避障测试。

## 5.2 双滑块相对距离

令左、右位置为 $x_L,x_R$，要求距离：

$$
d=x_R-x_L\ge0.05\ \mathrm m.
$$

定义：

$$
h=x_R-x_L-0.05,\qquad
\nabla h=\begin{bmatrix}-1\\1\end{bmatrix}.
$$

当前间距 $0.08$ m，$h=0.03$ m，取相同 $\gamma=2,\Delta t=0.008$，则：

$$
-\Delta x_L+\Delta x_R\ge-0.00048
$$

等价于：

$$
\boxed{\Delta x_L-\Delta x_R\le0.00048}.
$$

即一轮最多允许距离缩小 0.48 mm 的**线性近似**。

若把它扩展为 24 维教学矩阵，前两列是 $[1,-1]$，余下列为 0。但真实 12 轴机器人前两维是**关节角**而不是滑块位移，实际 CBF 必须先做链式法则变换。

---

# 第六章：双臂距离梯度怎样从 Jacobian 得到？

这是第三课“线速度参考点”最直接的应用。

在某个局部构型上，假设左右连杆的一对最近点分别为：

$$
p_L(q_L),\qquad p_R(q_R)
$$

并且最近点对应关系在附近不变。

定义：

$$
r=p_L-p_R,\quad d=\|r\|,\quad n=\frac r{\|r\|}.
$$

则距离导数：

$$
\dot d=n^\top(\dot p_L-\dot p_R).
$$

对应两个**最近碰撞点的位置 Jacobian**：

$$
\dot p_L=J_{pL}\dot q_L,\qquad
\dot p_R=J_{pR}\dot q_R.
$$

于是：

$$
\boxed{
\nabla_q d^\top=
\begin{bmatrix}
n^\top J_{pL}&-n^\top J_{pR}
\end{bmatrix}.
}
$$

这就是能用于 $h(q)=d(q)-d_{\rm safe}$ 的关节空间梯度。

### 6.1 为什么不能直接使用 TCP Jacobian？

因为最近危险点可能在肘部或腕部，那里与 TCP 并不是同一个位置。

刚体上不同点的速度满足：

$$
v_{\rm point}=v_{\rm ref}+\omega\times r_{\rm ref\rightarrow point}.
$$

所以必须使用**碰撞点自己的点速度 Jacobian**。用错误参考点的 Jacobian 会直接导致距离约束求导错误。

### 6.2 几何模型还会遇到哪些困难？

- 最近点发生切换，梯度可能不连续；
- 多个最近点同距，经典梯度可能不唯一；
- 如果已经穿透，某些“距离”返回值定义可能变化；
- 环境、夹爪、箱子和完整连杆几何必须纳入覆盖范围；
- 抓取接触需要允许碰撞矩阵 ACM 特别处理。

因此 `[源码事实]` `clearanceProxy()` 测的关键点距离**不能作为安全级 CBF 的真实距离函数**。

---

# 第七章：当前源码有哪些 CBF 接口？哪些没有？

**[源码事实]**`dual_arm_qp.hpp` 中确实有：

```cpp
Eigen::MatrixXd C_ineq; // k x 24
Eigen::VectorXd d_ineq; // k
```

其意义为：

$$
C_{\rm ineq}z\le d_{\rm ineq}.
$$

`dual_arm_qp.cpp` 在关节位置/速度限制之后，将额外行追加至 QP：

$$
A=\begin{bmatrix}I_{12}&0\\C_{\rm ineq}\end{bmatrix},
\quad
l=\begin{bmatrix}l_{\Delta q}\\-\infty\end{bmatrix},
\quad
u=\begin{bmatrix}u_{\Delta q}\\d_{\rm ineq}\end{bmatrix}.
$$

但**不能因此宣称当前控制器已经真正生成了碰撞 CBF**。

这几份已核对文件能支持的是：

- 存在通用的额外线性约束接口；
- 额外行可以被送入 QP 求解器；
- 并未从此接口直接证实实时网格碰撞距离、梯度计算和 CBF 自动装配链路。

### 7.1 还有两个工程上要注意的实现边界

**输入合法性：**当前 Core 通过 `C_ineq.cols()==24` 识别额外行，但在这段装配路径中未完整验证 `d_ineq` 长度与各元素 finite。因此未来接入 CBF 需要额外验证矩阵尺寸和数值合法性。

**结果后处理：**有额外不等式行时，Core 会跳过原有逐轴 box clamp，因为简单 clamp 可能破坏耦合约束。但这也意味着不能靠后处理保证输出满足全部 CBF 行，需要独立复算：

$$
\boxed{
v_{\max}=\max_i\big(C_i z-d_i\big)_+.
}
$$

该值必须和求解器的收敛状态、约束容差一起记录。上式中的 $v_{\max}$ 是约束最大违反量，**不是关节最大速度的那个 `v_max` 参数**。

---

# 第八章：离散时间的 CBF 陷阱

连续时间 CBF：

$$
\dot h+\gamma h\ge0
$$

的理论保证不能原封不动搬到采样控制器。

真正的离散安全条件可以写为：

$$
\boxed{
h(q_{k+1})\ge(1-\eta)h(q_k),\qquad0<\eta\le1.
}
$$

若 $q_{k+1}=q_k+\Delta q$，且 h 非线性：

$$
h(q+\Delta q)\approx h(q)+\nabla h(q)^\top\Delta q.
$$

代入可得到：

$$
\boxed{\nabla h^\top\Delta q\ge-\eta h(q).}
$$

当 $\eta\approx\gamma\Delta t$，它与第四章的线性行相近。

但一阶展开遗漏二阶项，**不能保证真实非线性距离变化一定满足同样条件**。

### 8.1 怎么让约束更保守？

若能够证明 Taylor 余项的绝对值：

$$
|R_2|\le\epsilon_{\rm model},
$$

那么可以要求：

$$
\nabla h^\top\Delta q
\ge-\eta h(q)+\epsilon_{\rm model}.
$$

这给线性化误差预留裕量，但 $\epsilon_{\rm model}$ 必须由模型或测试支持，不能随意拍脑袋设置。

### 8.2 对你的 position control 更重要的问题

S622 的 QP 求的是**目标关节位置增量**，真实位置接口执行时：

$$
q_{\rm actual}(t)\ne q_{\rm cmd}(t)
$$

可能存在跟踪滞后和调度抖动。

因此要证明真实 CBF 安全，需要同时处理：

- 控制周期变化；
- 硬件位置跟踪误差界；
- 命令延迟、反馈延迟；
- 一臂失步造成闭链不一致；
- 碰撞距离估计误差。

这些不能用数学上假定的 $q_{k+1}=q_k+\Delta q_k$ 直接代替。

---

# 第九章：CBF 也可能让 QP 无解

例如同时要求：

$$
\Delta q_1\ge0.01,\quad \Delta q_1\le0.005.
$$

显然不可行。

真实场景也可能出现：为了离开障碍需要一个方向，而关节限位、固定双端抓持几何或其他障碍约束禁止这个方向。

因此 QP 失败时不能：

- 继续执行未验证的解；
- 静默删除 CBF 行并继续正常搬运；
- 认为“保持旧位置命令”等于风险已经消失。

需要定义分层故障策略、记录冲突约束，并在必要时借助独立的设备级保护。

---

# 第十章：D7 健康监控的真实检查内容

**[源码事实]**`StreamHealthMonitor` 是独立于 ROS 的可测试类，包含以下检查。

| 类别 | 具体检查 | 注意事项 |
|---|---|---|
| 非有限数据 | `q_state` / `q_cmd` 出现 NaN 或 Inf | 不等于碰撞检测 |
| 单臂 stall 注入 | 指定臂、开始时刻、持续时间 | 是仿真故障刺激 |
| 跟踪误差 | 每臂最大 $|q_i-q_{{cmd},i}|$ | `tracking_error_limit<=0` 时关闭，当前默认 0 |
| 求解器故障 | 连续 solve 失败次数达到门槛 | 是全局 unhealthy |

每臂跟踪误差：

$$
E_a=\max_{i\in a}|q_i-q_{{cmd},i}|.
$$

这只能作为状态/命令偏差的代理，不能据此精确推断真实关节扭矩或物体接触力。

---

# 第十一章：D7 HOLD 究竟是什么？为什么不等于急停？

**[源码事实]**在 `dual_arm_qp_controller.cpp` 里：

```cpp
if (!health.healthy) {
    hold_active_ = true;
    writeCommand(q_cmd_);
    return return_type::OK;
}
```

它的准确含义是：

> **停止正常生成新的 QP 关节增量；给双臂重新发送同一组旧位置目标。**

如果当前 `q_cmd_` 已经领先真实反馈，机器人仍可能向那组旧目标继续运动。因此：

- **软件 hold ≠ 立刻停止真实关节**；
- **软件 hold ≠ 设备急停**；
- **软件 hold ≠ 内部作用力得到控制**。

### 11.1 有一个特别值得你注意的“自动恢复”边界

当后续 `health_.evaluate()` 恢复为 healthy，当前控制器可以把 `hold_active_` 设回 false 并继续 QP。对于暂时性 stall，这可能形成自动退出 HOLD 的行为。

在真实刚性双端抓持搬运中，**是否允许自动恢复**应作为专门的系统安全决策，可能需要锁存故障和显式复位；本课程不擅自修改现有控制器。

### 11.2 为什么 `update()` 返回 OK 不表示没故障？

QP 求解失败时，控制器一般增加失败计数并重发旧命令，再返回 `OK`。因此要分别看：

- `result.converged`；
- 连续失败计数；
- D7 健康状态与 hold；
- 两臂 command 与 feedback；
- 实际运动是否停止。

仅查看生命周期组件是否 active，不能完成故障验收。

---

# 第十二章：C4.3 为什么更需要分层安全？

C4.3 的双端理想固定抓取关系为：

$$
{}^WT_L={}^WT_OG_L,\qquad
{}^WT_R={}^WT_OG_R.
$$

这意味着双臂与同一物体形成名义闭链。因此至少要区分：

1. 机械臂与机械臂的碰撞；
2. 机械臂与箱子的非预期碰撞；
3. 物体与桌面/环境的碰撞；
4. 夹爪与物体的**预期抓取接触**；
5. 两臂运动失同步导致的闭链几何失配。

抓取接触应当根据预期几何单独允许，但不能因此关闭整只机械臂或所有物体的碰撞检测。

**目前的开关式气动夹爪没有夹持力反馈**，即使数学 CBF 与相对 QP 都工作，仍不能证明真实物体不会滑落或受到过大内部作用力。

---

# 第十三章：如何验证本课公式和工程行为？

**[建议验证]**将测试分为从纯数学到执行层的不同层级：

| 编号 | 测试 | 最关键的检查 |
|---|---|---|
| T1 | 一维 CBF | 边界内外、梯度方向与正负号 |
| T2 | 双滑块 CBF | 左右靠近时生成正确不等式 |
| T3 | 距离梯度对拍 | $\nabla_q d$ 与中心有限差分一致 |
| T4 | 多碰撞对 | 每个有效碰撞对各生成正确 QP 行 |
| T5 | 非法参数 | `C_ineq/d_ineq` 尺寸或数值错误明确拒绝 |
| T6 | 求解残差 | $\max(Cz-d)_+$ 在规定容差内 |
| T7 | 不可行注入 | 不将未验证的解交给执行器 |
| T8 | 离散化误差 | 检验较大步长、调度波动时的约束失效 |
| T9 | 单臂 stall | 记录左右臂命令冻结时间和实际停止时间 |
| T10 | 暂时故障恢复 | 检验自动恢复是否符合安全策略 |
| T11 | 夹爪和箱子接触 | ACM 只允许预期抓取接触 |
| T12 | 物体/环境碰撞 | 检查箱子与地面、桌面和障碍覆盖 |

### 13.1 为什么要用独立有限差分验证？

对安全距离函数 $h(q)$：

$$
g_i^{FD}=
\frac{h(q+\epsilon e_i)-h(q-\epsilon e_i)}{2\epsilon}.
$$

和代码提供的梯度比较：

$$
\frac{\|g^{FD}-g\|}
{\max(1,\|g^{FD}\|,\|g\|)}.
$$

还要单独测试最近点切换和不可微情况，不能强制要求不存在的唯一经典梯度。

---

# 第十四章：源码能力边界一览

| 功能 | 可从已核对公开源码确认 | 不能据此宣称 |
|---|---|---|
| 关节增量位置/速度 box 限制 | QP 已装配 | 实际硬件完全跟踪 |
| 24 列通用 `C_ineq` | QP Core 支持 | 碰撞 CBF 已完整自动生成 |
| 关键点间距次级偏好 | 存在 | 完整网格无碰撞 |
| 附加行下跳过 box clamp | 已实现 | 求解后绝不违反约束 |
| 非有限状态检查 | D7 支持 | 所有通信故障已检测 |
| 单臂 stall 注入 | D7 支持 | 真机 stall 均可检出 |
| tracking error 监控 | 支持，但默认阈值 0 关闭 | 所有 profile 已启用 |
| 两臂位置目标冻结 | 已实现 | 机械臂立即物理停机 |
| 健康后退出 hold | 当前代码允许 | 真机搬运无需显式复位 |

---

# 第十五章：课后练习

1. 为什么 `clearanceProxy()` 不能代替硬安全约束？
2. 从 $h(q)\ge0$ 开始推导 $\dot h+\gamma h\ge0$ 的数学用途。
3. 把连续时间 CBF 写成以 $\Delta q$ 为变量的离散 QP 行。
4. 为什么负号变化时一定要翻转不等式？
5. 为什么不应默认让任务 slack 放松 CBF？
6. 最近碰撞点不是 TCP 时，为什么不能直接使用 TCP Jacobian？
7. 为什么一阶 CBF 不能证明真实离散机器人绝不碰撞？
8. QP 返回 `converged=true`，为什么仍要独立检查额外约束残差？
9. D7 的 HOLD 当前冻结什么、没有控制什么？
10. 双端夹持箱子时，为什么既需要允许抓取接触又不能关闭所有碰撞检测？

### 答案要点

1. 它只比较少数关键点，并且是软偏好。
2. 在成立的连续动力学假设下，控制安全裕量的下降速度。
3. $C=[-\nabla h^\top,0,0]$，$d=\gamma dt\,h$。
4. 乘负数会翻转不等式方向。
5. slack 使原本硬限制变成可违反的软限制。
6. 刚体各点的线速度不同。
7. 因为离散化、非线性余项和执行跟踪误差。
8. 数值优化的停止条件允许容差；源码不是独立安全验证器。
9. 冻结新 QP 增量、重发旧位置目标；不执行设备急停。
10. 抓持接触是合法接触，其他碰撞仍需要检查。

---

# 第十六章：本课验收标准与下一课

如果能够独立完成下列事项，本课基本掌握：

- 从 $h(q)=d(q)-d_{\rm safe}$ 解释安全集合；
- 推导连续 CBF 与以 $\Delta q$ 为变量的 QP 线性不等式；
- 推导双臂之间一对最近点的关节空间距离梯度；
- 解释离散近似与位置跟踪如何削弱严格保证；
- 正确描述 Core 中的 `C_ineq/d_ineq` 能力与当前尚未验证的 CBF 接线；
- 描述 D7 的故障检测、命令冻结和自动退出 HOLD 的边界。

**一句话总结：**CBF 将可信的安全函数转化为对控制输入的约束；但是“约束数学正确”不等于“整个执行系统安全”。在 S622 双臂中还必须考虑几何覆盖、采样误差、位置跟踪、故障响应和设备级保护。

**第八课预告：**《ROS 2 / Gazebo 系统集成与控制接口》。重点是 launch → xacro → URDF → ros2_control → controller_manager，以及 C42/JTC/QP profile 的接口匹配与排障。

---

## 已核对的公开源码

- https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/dual_arm_qp.hpp
- https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/dual_arm_qp.cpp
- https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_controller/include/dual_arm_qp_controller/stream_health_monitor.hpp
- https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_controller/src/dual_arm_qp_controller.cpp

> 本课不修改控制器代码，也不宣称真机安全验收完成。任何安全约束用于真实机械臂前都需经过独立风险评估、仿真与受控验证。

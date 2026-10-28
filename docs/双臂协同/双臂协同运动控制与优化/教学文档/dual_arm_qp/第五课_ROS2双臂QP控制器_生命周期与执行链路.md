# 第五课：ROS 2 双臂 QP 控制器——生命周期与执行链路

**S622 双臂 QP 系统源码与数学学习课程 · 第五课**

> 这是一份**控制器工作原理讲义**，不是 `dual_arm_qp_controller.cpp` 变量与形参字典。课程遵守既定分工：理论/架构用独立课程；变量和逐行注释另作伴读手册。
>
> 核对基准：`Ahhyp/s622_dual` GitHub `public` 分支（2026-10-20）。
> - [控制器实现](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_controller/src/dual_arm_qp_controller.cpp)
> - [控制器头文件](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_controller/include/dual_arm_qp_controller/dual_arm_qp_controller.hpp)
> - [D7 健康监控](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_controller/include/dual_arm_qp_controller/stream_health_monitor.hpp)
>
> 以下的“已实现”只指上述公开代码，**不代表本地开发分支、仿真或真机验收**。本文没有修改代码或运行 Gazebo。

## 一、为什么这课很重要？

前三课已经解释了运动学、QP 的数学模型和求解器。现在问题是：**QP 给出一组关节增量，机器人为什么会真的运动？**

一次真实控制周期的逻辑是：

```text
controller_manager
    ↓ 调用 update()
读取 12 个关节的 position 状态
    ↓
检查 D7 健康状态
    ↓
用当前关节角与相对位姿目标调用 QP
    ↓
从求解结果取 dq[12]
    ↓
q_cmd ← q_cmd + dq
    ↓
限制位置范围
    ↓
写入 12 个 position command interfaces
    ↓
硬件或 Gazebo 执行（结果要通过状态反馈核验）
```

**目标：**学完后，能区分 QP 数学正确、控制器正常激活、真实机器人准确跟踪这三件不同的事。

---

# 第一章：ROS 2 controller 与普通 C++ 函数有什么区别？

普通函数是调用一次、运行一次；ROS 2 控制器是由 `controller_manager` 管理、按生命周期激活、然后周期性调用 `update()` 的组件。

核心步骤：

```text
创建/加载
   ↓
on_init()
   ↓
on_configure() → inactive
   ↓
on_activate()  → active
   ↓
update() → update() → update() ...
   ↓
on_deactivate()
```

几个回调不能交换顺序理解：

| 回调 | 负责什么 | 当前源码做的事 |
|---|---|---|
| `on_init()` | 参数声明、接口名准备 | 声明关节及控制参数，要求 `joints` 数量为 12，建立接口名 |
| `on_configure()` | 把配置变成可用资源 | 读取控制权重、初始化限位、健康监控和 QP |
| `on_activate()` | 从当前机器人状态开始工作 | 读取状态、令命令等于当前状态、捕获当前双手相对位姿 |
| `update()` | 每个周期运行一次控制逻辑 | 健康检查、求 QP、处理失败、积分关节命令 |
| `on_deactivate()` | 停止这个控制器的正常执行 | 当前实现设置 `activated_ = false` |

**别把 `deactivate` 等同于硬件急停**。当前 `on_deactivate()` 没有单独发送设备级安全制动命令。

---

# 第二章：state interface 和 command interface

## 2.1 最核心的区别

- **State interface**：硬件向软件报告**目前在哪里**。
- **Command interface**：软件向硬件写入**希望到哪里**。

即使两个接口都叫 `position`，它们也不是同一个数据方向。

设关节实际角度为 `0.1 rad`，位置命令为 `0.12 rad`：这不矛盾。机器人可能还没追上目标。

本控制器请求 12 个位置状态接口和 12 个位置命令接口：

```text
left_joint_1/position ... left_joint_6/position
right_joint_1/position ... right_joint_6/position
```

实际关节名称取决于配置中的 `joints` 列表；上面仅示意格式。

它不领取夹爪指关节资源，因此**C4.3 的气动夹爪 OPEN/CLOSE 不在这个控制器内完成**。

## 2.2 为什么接口类型必须匹配？

若硬件提供 `position`，控制器却要求 `velocity`，资源管理器不能把它们当成相同接口。即使两者都有 12 个，**维度相同也不等于接口契约相同**。

这也是之前 C42 profile 配置排查中的关键。以后检查启动失败，优先看硬件接口、控制器请求和资源冲突，而不是先调整 QP 权重。

---

# 第三章：`on_init()` 究竟初始化了什么？

源码声明的参数包括：

- 关节和 QP：`joints`、`dt`、`kp_rel`、`w_rel`、`w_slack`、`w_center`、`w_reg`、`v_max`；
- 健康监控：`tracking_error_limit`、`solve_failure_limit`、`stall_inject_arm`、`stall_inject_start_s`、`stall_inject_duration_s`；
- 测试刺激：`test_sweep_amp_m`、`test_sweep_period_s`。

然后检查 `joints.size()==12`，生成 `joint_name + "/position"` 的状态和命令接口名称。

**数学层还没有开始真正求解。**这一步只是明确：控制谁、向 controller_manager 要什么资源。

---

# 第四章：`on_configure()` 为什么要创建运动学和 QP？

这一阶段从参数系统读取反馈系数和权重，初始化软件关节限制，构造：

```text
DualArmKinematics
       ↓ 提供 FK 与 Jacobian
DualArmQpParams
       ↓ 提供 dt、权重与限位
DualArmQp
       ↓ 每次求解使用
QpSolver
```

源码先构建 `kinematics_`，再让 `qp_` 引用它。这样 QP 的运动学引用始终有有效的对象可用。

最后调用 `qp_->setWarmStart(true)`：**启用使用上一轮解作为新一轮初值的机制**，但这不代表 OSQP 内部 workspace 一定持久复用（第四课已解释其区别）。

### 当前已经接入哪些 QP 参数？

`kp_rel`、`w_rel`、`w_slack`、`w_center`、`w_reg`、`dt`、位置限位和速度上限都在配置流程中实际传入 `DualArmQpParams`。

**当前 `on_init/on_configure` 没有声明或读取物体任务的 `w_obj`、`kp_obj` 等参数。**因此，QP Core 的数学功能存在，并不自动意味着这个 ROS 控制器已启用物体轨迹跟踪。

---

# 第五章：`on_activate()` 为什么要记录当前相对位姿？

当前源码有两个重要操作：

```cpp
q_cmd_ = q_state_;
target_ = kinematics_->leftTright(q_state_);
```

## 5.1 `q_cmd_ = q_state_`

激活时把首个命令目标设置为当前读到的位置，降低突然跳转到旧命令值的风险。

$$
q_{\mathrm{cmd},0}=q_{\mathrm{measured},0}.
$$

这是一种平稳初始化策略，不是任何硬件条件下“立即停止运动”的证明。

## 5.2 捕获目标 `target_`

计算当时左手相对于右手的关系，更准确地说 `leftTright(q)` 返回：

$$
{}^LT_R(q)=({}^WT_L(q))^{-1}{}^WT_R(q).
$$

激活时记录：

$$
T_{LR}^* = T_{LR}(q_0).
$$

这就解释当前控制器的基础任务：

**保持激活瞬间的双手相对位姿。**

它没有凭空知道箱子的目标位置，也没有自动产生抬升、平移、下降轨迹。

---

# 第六章：`update()` 的完整工作逻辑

这是本课最重要的一章。

## 6.1 读取反馈

每轮先检查是否 active，再调用 `readState()` 读取 12 轴 position 状态。

若状态接口数量不为 12，或某个值是 `NaN/Inf`，就返回错误。这些问题发生在 QP 求解前。

## 6.2 检查 D7 健康状态

调用：

```cpp
health_.evaluate(elapsed, q_state_, q_cmd_, solve_fail_streak_);
```

检查单臂故障注入、跟踪误差和连续求解失败等。

若 `health.healthy == false`：

```cpp
hold_active_ = true;
writeCommand(q_cmd_);
return return_type::OK;
```

这里“hold”的真实实现含义是：

**不再生成新关节增量，并向双臂重新写入上一组位置目标。**

它不是设备急停命令，也不是把两臂目标改成最新实测位置。因此不能仅凭这个软件分支保证物理上的立即静止。

## 6.3 构造相对目标

正常情况下，使用激活时捕获的 `target_`。

如果 `test_sweep_amp_m > 0`，则为测试沿左抓取坐标系 X 添加一个正弦变化：

$$
x_{\mathrm{target}}(t)=x_0+
A\sin\left(\frac{2\pi t}{T}\right).
$$

这用于测试控制器响应和 D7 监控。**它不是 C4.3 正式的物体轨迹接口。**

## 6.4 调用 QP

当前代码的关键调用是：

```cpp
const auto result = qp_->solve(q_state_, target);
```

也就是说，只传入：

- 当前实测关节角；
- 双手目标相对位姿。

它没有传入显式物体目标变换与物体速度前馈。因此这里仍是**相对位姿控制主线**。

## 6.5 求解失败

如果 `result.converged == false`，代码增加 `solve_fail_streak_`，保留上一组位置命令，并跳过 `dq` 更新。

**当前失败分支一般返回 `return_type::OK`**，表示控制器这一调用已按自身策略处理失败，不等于数学求解成功。连续失败达到阈值后，健康监控才会将其视为全局 unhealthy。

## 6.6 求解成功

核心三步：

```cpp
q_cmd_ += result.dq;
clampToLimits(&q_cmd_);
writeCommand(q_cmd_);
```

对应：

$$
\boxed{
q_{\mathrm{cmd},k+1}
=q_{\mathrm{cmd},k}+\Delta q_k.
}
$$

**注意不是：**

$$
q_{\mathrm{cmd},k+1}
=q_{\mathrm{state},k}+\Delta q_k.
$$

区别非常大：QP 用实测 `q_state_` 求解，却把增量累加在**上一条命令轨迹**上。

假设：

- 上轮命令第一关节是 `0.11 rad`；
- 本轮实测是 `0.10 rad`；
- QP 本轮给出 `dq=0.005 rad`。

那么实际写入的下一条命令是：

$$
0.11+0.005=0.115\,\mathrm{rad},
$$

而不是 $0.105\,\mathrm{rad}$。

这是一个值得理解和测试的控制架构选择。

---

# 第七章：`dt` 和真实控制周期是一回事吗？

**不是。**

- QP 的 `params.dt` 是算法使用的**标称时间步长**，默认 `0.008 s`。
- `update(time, period)` 的 `period` 是 controller manager 提供的当前调用间隔。

当前公开版 `update()` 把 `period` 标记为未使用，QP `dt` 仍采用配置的固定值。

所以不能因为设置了 `dt=0.008`，就说实际系统一定稳定运行在 125 Hz。

数学上速度约束通过：

$$
|\Delta q_i|\le v_{\max,i}\,dt
$$

换算成单步增量限制。

如果真实 `update` 周期出现波动，位置命令的实际时间间隔与 QP 的名义限制可能不一致。因此应独立测量控制周期、求解时延及执行跟踪误差。

---

# 第八章：D7 健康监控到底检查哪些问题？

当前 `StreamHealthMonitor` 位于头文件中，能够脱离 ROS 直接进行单元测试。

## 8.1 非有限关节状态或命令

若任意关节为 `NaN` 或 `Inf`，报告该臂 unhealthy。

## 8.2 单臂 stall 故障注入

`stall_inject_arm`：

- `-1`：关闭；
- `0`：左臂；
- `1`：右臂。

配合激活后起始时间和持续时长，模拟某一侧异常。目标是测试**一臂异常时双臂停止生成正常目标更新**，并非证明真正通信层每种 stall 已能可靠检出。

## 8.3 跟踪误差检查

对每条臂取六个关节的最大位置跟踪误差：

$$
E_a=\max_{i\in a}|q_i-q_{\mathrm{cmd},i}|.
$$

若 `tracking_error_limit > 0` 且超过阈值，判 unhealthy。

**源码默认 `tracking_error_limit=0`，意味着该阈值检查默认关闭。**

因此不能说“代码写了监控，所以所有运行 profile 一定启用了监控”。

## 8.4 连续 solver 失败

达到 `solve_failure_limit` 时，健康状态报告全局异常（不归咎于某一条臂），后续进入共同 hold 分支。

---

# 第九章：安全与实时性的真实边界

### 9.1 “写入位置命令”不等于“物理跟踪成功”

每轮 `writeCommand(q_cmd_)` 只是向接口写目标角。真机 ServoJ 或 Gazebo 内部的执行系统能否跟上，需要看反馈。

### 9.2 “重发旧位置”不等于“硬件停机”

如果旧命令本身比实测位置超前，机器人仍可能继续追踪旧目标。要证明双臂安全故障响应，需要单独测故障触发时刻、反馈响应、实际停止行为和设备级安全机制。

### 9.3 “solver 收敛”不等于“控制周期满足时限”

QP 核心耗时、OSQP setup/cleanup、数据转换、ROS 调度和硬件响应都是不同时间尺度。测量控制器时要看端到端时间，而不只是 `solve_time_ms`。

### 9.4 “QP Core 物体任务已实现”不等于“控制器已接入”

源码中 `update()` 调用两参数 `solve()`，也没有面向 C4.3 的物体轨迹 ROS 输入或抓持状态机。

**后续扩展必须首先设计物体目标入口与坐标系契约，而不是假设现成 YAML 权重已经能让箱子沿轨迹移动。**

---

# 第十章：排障时应该先怀疑哪一层？

| 症状 | 优先检查 | 不要立刻做什么 |
|---|---|---|
| Controller 无法 active | URDF 接口、资源 ownership、生命周期 | 调整 QP 权重 |
| Active 但几乎不动 | 激活时目标已等于当前位姿，是否施加了真实任务或测试刺激 | 宣称 QP 坏了 |
| QP 收敛、反馈落后 | dt、控制周期、position 执行器跟踪能力、tracking-error 配置 | 仅提高 `kp_rel` |
| 一侧 stall，另一侧仍有实际运动 | hold 时写的旧目标、底层执行状态、安全停机链路 | 只看 `hold_active_` |
| 物体目标参数没效果 | 控制器是否声明、读取并用于正确的 solve 重载 | 只修改 YAML |
| 旧 ROS 图读数互相矛盾 | 仿真残留进程、ROS_DOMAIN_ID、控制器实际接口 | 判断 launch 参数反转 |

---

# 第十一章：本课的源码阅读任务

按顺序阅读三个文件：

1. **`dual_arm_qp_controller.hpp`**：只认清组件拥有哪几类资源；注意 `q_state_`、`q_cmd_`、`target_` 三者角色。
2. **`dual_arm_qp_controller.cpp` 前半部分**：跟踪参数从 `on_init()` 到 `on_configure()` 的流动。
3. **`on_activate()` 和 `update()`**：追踪目标捕获、求解、位置命令累加。
4. **`stream_health_monitor.hpp`**：逐个理解 unhealthy 的判据和检测默认开关。
5. 返回 `dual_arm_qp.cpp`：看求解结果 `dq` 到这里之后如何被消费。

建议给自己画出下图，并能解释每一根箭头：

```text
q_state_ ──> 运动学 / QP ──> result.dq
    │                           │
    ├──> 健康监控                ▼
    │                        q_cmd_ += dq
    │                           │
    └────> 跟踪误差检测 <───────┤
                                ▼
                         position command
```

---

# 第十二章：课堂练习

**练习 1**：`position state` 和 `position command` 为什么名字相同却不能互换？

**练习 2**：为什么 `on_activate()` 要先令 `q_cmd_=q_state_`？它是不是紧急停止命令？

**练习 3**：控制器激活时若捕获了当前 `leftTright`，又没有任何任务变化，QP 是否一定产生明显的双臂平移？

**练习 4**：证明 `q_cmd_ += dq` 与 `q_state_ + dq` 在跟踪滞后时会产生不同的下一周期目标。

**练习 5**：当前 `tracking_error_limit` 默认 0，这意味着什么？

**练习 6**：为什么 `update()` 不使用 `period` 可能影响对关节速度限制的理解？

**练习 7**：若 solver 连续 5 次失败，健康逻辑和正常求解逻辑分别做什么？

**练习 8**：C4.3 要实现“搬运箱子”，为什么不能只在 YAML 添加 `w_obj`？

### 参考答案要点

1. 一个来自反馈，一个用于发送目标。
2. 为了从当前状态开始生成命令；不是设备急停。
3. 不一定。保持相对位姿不等于指定物体的世界运动。
4. 前者基于旧命令积分，后者基于当前实测反馈。
5. 对应跟踪误差监控分支默认关闭。
6. QP 单步增量基于名义 dt，实际执行周期另有时间尺度。
7. 失败时不更新 dq，累计计数；达到门槛后进入全局 unhealthy。
8. 需要 ROS 参数读取、物体目标输入、正确坐标系、QP 重载调用和验证。

---

# 第十三章：验收与下一课

学完后，你应能独立回答：

- 生命周期四个关键回调各负责什么？
- 控制器如何领取 12 轴 position 资源？
- 实测 `q_state_` 怎样进入 QP，而 `q_cmd_` 怎样由旧命令和 dq 得到？
- 为什么 D7 hold 是软件命令冻结，不自动等于物理停机？
- 为什么当前控制器还不能直接实现 C4.3 的完整物体搬运？

**下一课（第六课）：《零空间、奇异性与高级任务分配》**。将深入研究阻尼零空间投影、关节限位裕量、可操作度、次级任务梯度与奇异性处理。

> 若未来更新了 ROS 2 版本、控制器接口或 C4.3 适配器，请先重新核对源码，再把新的实现边界补充到讲义；不要以课程文档替代测试。

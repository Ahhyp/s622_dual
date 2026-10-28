# 第八课：ROS 2 / Gazebo 系统集成与控制接口

> S622 双臂 QP 系列 · 第 8 / 9 课  
> 主题：从 QP 数学解到 12 轴执行命令  
> 学习前提：已学习双臂任务模型、QP、零空间、CBF 和故障 HOLD。

## 阅读说明：证据等级

- **项目历史已确认**：来自此前讨论中实际核对过的仓库快照与实现记录；不保证等于今天最新提交。
- **官方机制**：来自 ROS 2 Humble / ros2_control 的官方接口文档。
- **推荐设计**：针对 C4.3 的建议架构；除非另有项目测试证据，不表示已实现。
- **待验证**：需要在当前仓库、实际运行日志或设备上确认的事项。

本讲义不是当前 `master` 最新源码的逐行审计。若仓库已修改，请以最新接口、配置与测试为准。

---

## 一、学习目标

学完后，应能独立解释并检查以下链路：

```text
C4.3 物体轨迹 / C2 相对位姿任务
                 │
                 ▼
        任务误差与 Jacobian
                 │
                 ▼
           QP Core 求解
              Δq(12)
                 │
                 ▼
       q_cmd ← q_cmd + Δq
                 │
                 ▼
   ros2_control ControllerInterface
                 │
                 ▼
      12 个 position command
                 │
       ┌─────────┴─────────┐
       ▼                   ▼
 Gazebo 仿真硬件       fairino_hardware
       │                   │
 Gazebo 关节执行         ServoJ I/O
       │                   │
       └─────────┬─────────┘
                 ▼
        q_state / 反馈监控
                 │
                 └────→ 下一轮 QP
```

这张图是**推荐的统一语义**，不是声称两个运行后端的底层细节完全一致。

## 二、先分清五个软件角色

### 2.1 QP Core：算什么，不负责怎么发

核心的输入通常包括：当前关节状态、左右末端 Jacobian、相对任务误差、物体任务误差、限位以及附加约束。输出是 12 轴关节增量 `Δq`（以及求解状态、残差、slack 等诊断量）。

它应该可以脱离 Gazebo 和真机独立测试：给定数值输入，检查输出的可行性与任务效果。

### 2.2 ControllerInterface：把数学输出变成设备命令

`ControllerInterface` 是控制器与 `ros2_control` 的结合层。它负责从已分配的 state interfaces 读取状态，运行控制算法或调用核心库，并写入已分配的 command interfaces。

不要把它与机械臂的物理驱动混为一谈：它是**控制算法的运行容器**。

### 2.3 controller_manager（CM）：管理运行周期和资源

CM 负责控制器装载、配置、激活、停用与资源管理，并调用硬件读写和控制器更新。

典型概念流程：

```text
read() → update() → write()
```

这不是“保证每个硬件驱动都在同一线程、同一节拍完成网络发送”的意思。对于带独立 I/O 线程的 Fairino 驱动，还需要单独分析其队列和发送时序。

### 2.4 HardwareInterface：把抽象命令连接到执行端

硬件层把 `position` / `velocity` / `effort` 等命令接口映射到具体的仿真关节或真机驱动，并返回关节状态。

项目历史中曾确认：

- 真机 `fairino_hardware` 使用 `position` command interface。
- 机械臂最终通过 ServoJ 路径执行位置目标。
- 某一历史 Gazebo JTC baseline 使用 `velocity` command interface。

因此不能直接把一个只写 `velocity` 的 QP 控制器视为可无缝迁移真机。

### 2.5 Gazebo：提供物理环境，不负责定义 QP

Gazebo 负责机器人、物体、关节、接触和动力学环境。它与 `ros2_control` 的对接取决于所用 Gazebo 版本及其系统插件。

- Gazebo Classic 使用 `gazebo_ros2_control`。
- 新版 Gazebo Sim 使用 `gz_ros2_control`。

**版本提醒**：Gazebo Classic 已于 2025 年 1 月结束生命周期；这不意味着现有 Humble 仿真项目必须立即迁移，但规划新平台时应考虑维护状态。

官方参考：
- https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html
- https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html

---

## 三、ROS 2 控制器生命周期和资源所有权

理解五个关键词：

1. **unconfigured**：尚未完成配置。
2. **inactive**：已配置，但未主动占用执行所需 command interfaces。
3. **active**：参与更新，拥有分配到的命令资源。
4. **deactivate**：解除控制器的运行状态，准备切换。
5. **switch**：按照控制器管理器的规则停用旧控制器、启用新控制器。

这些状态本身不保证“机械臂已经物理停止”。停用控制器之后硬件如何处理上一条命令，取决于具体后端与配置。

### 为什么不能让左右 JTC 与双臂 QP 同时写相同关节？

如果两套控制算法同时发布同一组关节的目标，将出现命令来源争夺。更隐蔽的是：**不同 command interface 类型的资源声明，不一定能替代你自己做的运行模式互斥设计**。

C4.3 推荐模式：

```text
Mode A: 经典轨迹 baseline
  left JTC + right JTC
  QP inactive

Mode B: 双臂协调 QP
  left JTC inactive
  right JTC inactive
  dual_arm_qp_controller active
```

切换时应保证两臂都受到同一个协调策略管理；避免“左臂 QP，右臂还在执行旧 JTC 轨迹”。

### 推荐的切换事务

1. 停止发送新的物体轨迹目标。
2. 确认当前系统处于允许切换的安全状态。
3. 请求停用两个 JTC，启用双臂 QP。
4. 检查控制器实际状态以及被占用的 interfaces。
5. 以当前实际关节状态初始化 QP 位置指令，避免命令突跳。
6. 清除旧目标、旧积分状态以及过期任务数据。
7. 校验第一帧输出连续后才允许执行新搬运轨迹。

第 5 步和第 7 步属于**安全设计要求**，并非只要调用 ROS 2 switch 服务就自动得到。

可用 CLI（不同安装环境请先看帮助）：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control switch_controllers \
  --deactivate left_arm_controller right_arm_controller \
  --activate dual_arm_qp_controller \
  --strict
```

上面的三个控制器名称仅为**示例占位名**，不要原样当作项目真实名称使用。运行前先确认目标控制器确实存在，且硬件模式兼容。

官方 CLI：https://control.ros.org/humble/doc/ros2_control/ros2controlcli/doc/userdoc.html

---

## 四、QP 为什么求 Δq，却要写 position？

项目采用离散时刻 `k` 的关节增量形式：

\[
\Delta q_k \in \mathbb{R}^{12}
\]

位置级控制器使用：

\[
q_{\mathrm{cmd},k+1}
=
q_{\mathrm{cmd},k}+\Delta q_k
\]

然后把 `q_cmd` 写入 position command interfaces。

若核心求解的是关节速度而不是增量，则正确关系是：

\[
q_{\mathrm{cmd},k+1}
=
q_{\mathrm{cmd},k}+\dot q_k \Delta t
\]

**不要重复乘 `Δt`**：必须先确认 QP 决策变量到底是 `Δq` 还是 `qdot`。数学层速度 QP 与项目实际增量 QP 的数值单位不可混用。

### 两种积分基准为什么不同？

取一轴：

- 当前命令 `q_cmd=0.200 rad`
- 实际反馈 `q_state=0.180 rad`
- 本周期 QP 增量 `Δq=0.004 rad`

命令基准积分：

\[
q_{\mathrm{cmd,new}}=0.204\ \text{rad}
\]

反馈基准积分：

\[
q_{\mathrm{cmd,new}}=0.184\ \text{rad}
\]

两个方案不会产生相同效果。

命令基准可保持规划命令的时间连续性，但在硬件停止跟随或出现 I/O stall 时，**命令可能越来越领先于实际关节**。

反馈基准能限制某些形式的命令漂移，但可能受采样噪声、延迟和误差注入影响。不能不经分析就“改用反馈积分”。

### 针对 C4.3 的命令-反馈保护

推荐持续监测：

\[
e_{\mathrm{follow}}=q_{\mathrm{cmd}}-q_{\mathrm{state}}
\]

至少包括：

- 12 轴跟随误差最大值和变化趋势；
- 左右双臂是否同步推进；
- 状态时间戳与数据年龄；
- 关节位置、速度上限；
- 有效命令与执行确认是否一致。

**关键边界**：`q_cmd` 与 `q_state` 差小，也不能直接证明物体接触力安全。

---

## 五、125 Hz、500 Hz 和 8 ms：三个概念

项目历史中记录的真机架构：

| 部分 | 历史记录的节拍 | 意义 |
|---|---:|---|
| CM | 500 Hz | 控制器管理器的 update 周期 |
| QP | 目标 125 Hz | 解算和指令推进节拍 |
| ServoJ I/O | 125 Hz | 真机位置目标下发节拍 |

它们不是一个统一的“机器人控制频率”。

算术验证：

\[
\Delta t_{\mathrm{QP}}=\frac{1}{125}=0.008s=8ms
\]

\[
\frac{500}{125}=4
\]

因此，在理想的同步情况下，一个 QP 解对应四次 CM 周期。但**这只是名义比例**，并不证明线程之间严格无抖动或无延迟。

推荐设计：

```text
CM tick 0: 计算一次 QP，推进 q_cmd
CM tick 1: 保持 q_cmd，不积分
CM tick 2: 保持 q_cmd，不积分
CM tick 3: 保持 q_cmd，不积分
CM tick 4: 下一次 QP，推进 q_cmd
```

真正实施时应使用合适的时钟与调度设计，而不是仅靠“每 4 次调用求一次”就认定实时性合格。

### 如何做时序验收？

记录至少四类时间戳：

1. 机器人状态采样时间。
2. QP 求解开始/结束时间。
3. 控制器写入命令时间。
4. 驱动发送与反馈确认时间（若当前接口可观测）。

统计求解耗时平均值、P95、P99、最大值；CM update 抖动；QP 漏周期次数；状态年龄；两臂命令失同步事件。

### 为什么平均计算耗时不足以验收？

因为系统危险通常发生在长尾事件：某一帧极慢，左臂动作停下，右臂却继续。对于共同夹住刚体的双臂系统，**最慢的那一侧**可能决定整个任务是否仍可继续。

---

## 六、ControllerInterface 内部应该完成什么？

建议把控制器 update 的内容拆成明确阶段：

```text
1. read state
2. validate timestamp and state quality
3. check coordinated safety state
4. compute FK / Jacobian / task error
5. build QP
6. solve QP
7. validate solver result
8. update bounded q_cmd
9. write twelve position commands
10. publish or buffer diagnostics
```

这只是职责划分示意，不表示所有诊断消息都应在实时 update 线程内发布。高频实时路径应避免不可控的动态分配、阻塞操作和不确定性 I/O。

### QP solve 失败时怎么办？

不可采用“无解就继续沿用最后一个 `Δq` 无限运动”。

建议区分：

- 轻微暂态问题：有限次数的受约束退化策略；
- 状态不可信/控制周期失效：双臂协调冻结命令；
- 重复或严重故障：进入锁存故障状态，需要明确复位；
- 真机设备安全事件：交给独立设备级保护机制执行。

具体阈值和停机行为必须通过安全评估确定，不应直接照搬讲义中的示意流程到真机。

---

## 七、D7 HOLD 与真正安全停机的界限

此前讨论中核对到的一个历史行为是：D7 异常时控制器可能重发上一组位置命令，健康恢复后可能退出 HOLD。

它在软件上意味着“暂停推进目标”，**并不自动等于**：

- 机械臂已经停稳；
- 两个独立控制箱同时停止；
- 气动夹爪保持安全抓持；
- 故障消失后继续运动一定安全；
- 已满足设备级安全停机要求。

### 为什么 C4.3 比单臂更敏感？

左右气动夹爪分别固定抓住同一个物体。它们构成一个闭链协同系统：

```text
Left S622 ── left gripper ══ rigid object ══ right gripper ── Right S622
```

若左臂 ServoJ 停滞、右臂仍执行新位置命令，就可能出现抓取变换破坏与内部作用力升高。即使没有末端力传感器，这个风险仍然存在。

### 推荐双臂一致故障状态机

```text
IDLE
  │ valid grasp / valid task
  ▼
READY
  │ coordinated start
  ▼
RUNNING ── anomaly ──→ HOLD_LATCHED
  │                       │
  ▼                       ├── explicit recovery validation
COMPLETED                 ▼
                      RECOVERY_CHECK
                           │ if safe and reinitialized
                           ▼
                         READY
```

对 C4.3，建议 **HOLD_LATCHED 默认不自动恢复搬运**，而是明确记录原因、检查抓持/相对位姿状态、重置参考后由操作者批准重启。设备级安全停机链仍独立存在。

这是一项**建议的改进目标**，不能仅凭历史软件 HOLD 行为宣称已经实现。

---

## 八、Gazebo 仿真与真机不能混为一谈

| 项目 | Gazebo | 真机 |
|---|---|---|
| 关节响应 | 由仿真和控制模型决定 | 由驱动、伺服和网络 I/O 决定 |
| 命令接口 | 可配置，必须核对当前 profile | 历史验证为 position |
| 抓取关系 | 可采用理想附着模型 | 开关式气动夹爪，无法软件调力 |
| 物体接触 | 可选择理想化或真实摩擦接触 | 依赖实际接触与夹持条件 |
| 单臂 stall | 可注入故障 | 历史出现过 ServoJ 长阻塞 |
| 安全结论 | 验证算法与仿真边界 | 还需独立硬件安全论证 |

### 一个危险的假象

如果 Gazebo 让物体“自动跟随协调坐标系”，那么物体永远不会因为抓持力不足而掉落。它可以有效验证运动学与轨迹跟踪，却不能作为夹爪抓持可靠性证据。

在 C4.3 第一阶段可采用理想固定抓持，但测试报告必须明确**“物体轨迹是独立仿真状态、由约束计算得到，还是由真实接触动力学产生”**。

---

## 九、把 C4.3 接入控制器前应明确的接口

### 9.1 输入

```text
joint_state:
  q[12], optional dq[12], timestamp

relative_task:
  T_left_right_desired
  weights / gains

object_task:
  T_object_desired
  reference twist / feedforward
  task validity and timestamp

safety:
  limits
  collision status
  synchronous execution health
```

上面的结构是**接口建议**，不是项目中已经存在的消息定义。

### 9.2 任务构造

从左右末端 FK 和固定抓取变换得到物体位姿估计：

\[
{}^WT_{O,L}={}^WT_L {}^LT_O
\]

\[
{}^WT_{O,R}={}^WT_R {}^RT_O
\]

理想固定抓持下，它们应近似相等。差值可以形成协同一致性诊断，但不能被当作真实滑移传感器。

已有讨论指出 C4.3 在 QP Core 添加了 `ObjectTaskRow` 外部任务入口；这意味着需要把对象 Jacobian、误差与前馈映射到实际求解器，而不等于当前运行控制器已完成全链路集成。

**尤其检查误差约定**：若物体误差定义为 current − desired，注入的期望纠偏方向必须与 QP Core 的残差符号契约一致。这个问题在第二课专题已深入推导。

### 9.3 输出与诊断

建议至少保留：

- 相对位姿误差（位置与姿态分开）；
- 物体轨迹位置/姿态误差；
- QP 状态、迭代次数、解算耗时；
- slack 范数；
- 12 轴 `q_cmd`、`q_state` 和跟随误差；
- 左右命令时序偏差；
- HOLD 原因、持续时间、锁存状态；
- 关节限位和碰撞安全监控结果。

---

## 十、动手时的最小验证顺序

### 实验 A：接口盘点（零运动）

确认当前运行平台的：

- `controller_manager` 命名空间；
- 12 轴 hardware command/state interface 列表；
- JTC/QP 控制器名称、active/inactive 状态；
- Gazebo 插件类型和机器人模型；
- QP 决策变量的单位；
- 实际使用的 `update_rate`。

输出：**控制栈接口清单**。

### 实验 B：无物体、无轨迹的 QP HOLD

让 QP 获得全部 12 轴资源，保持当前关节位置，不执行新轨迹。

验收：无明显切换跳变、无另一控制器双写、命令来源唯一、状态持续更新。

### 实验 C：小幅双臂相对位姿保持

在安全的仿真构型下运行小幅相对位姿任务。检查 `T_LR` 残差、关节限位和 QP 诊断。

验收：命令有界、无控制器资源冲突、任务错误方向正确。

### 实验 D：物体轨迹接口集成

加入理想固定抓持的 `ObjectTaskRow`；先通过静止物体目标，再测小幅平移和抬升。

验收：物体误差收敛、左右抓取位姿一致性受控、QP 无持续不可行。

### 实验 E：故障注入

分别注入：
- 左臂状态过期；
- 右臂命令执行停顿；
- QP 超时/失败；
- 物体目标时间戳过期；
- 控制器激活或切换失败。

验收：任何一臂失去可信执行状态时，不能允许另一臂继续推进共同物体搬运参考；故障原因可追踪，恢复有明确批准流程。

---

## 十一、现场排障思路

**现象 1：QP 算出合理的 `Δq`，机械臂不动。**

按顺序检查：控制器是否 active；接口是否已被 claim；位置命令是否实际写入；仿真或驱动是否接收；状态是否更新。不要直接调整任务权重。

**现象 2：仿真正常、真机完全不跟随。**

优先检查 command interface 类型、QP 更新节拍、ServoJ I/O 和单位约定，再看控制器数学问题。

**现象 3：切换到 QP 瞬间关节跳一下。**

优先检查是否用当前关节状态初始化命令、旧轨迹是否彻底取消、零空间偏好是否产生突变、第一帧是否错误积分。

**现象 4：轨迹越走越偏但相对位姿还好。**

可能只有相对任务起效、物体任务未接入、目标 frame 不一致、前馈/误差符号错误，或者与外部物体运动学假设不一致。

**现象 5：左臂暂停时右臂继续。**

这是系统级协调故障，不应通过单独调大相对位姿任务权重来“解决”。应检查共享健康状态、命令推进与故障 HOLD 仲裁。

---

## 十二、练习与参考答案

### 题 1：位置积分

给 `q_cmd=0.200 rad`、`q_state=0.180 rad`、`Δq=0.004 rad`。分别计算从命令和反馈出发的下一帧目标。

**答案**：`0.204 rad` 与 `0.184 rad`。两者控制语义不同。

### 题 2：频率

若 QP 125 Hz，CM 500 Hz，求每帧 QP 目标间的名义 CM tick 数以及 QP 周期。

**答案**：4 tick；8 ms。这并不证明实际物理发送无抖动。

### 题 3：资源争夺

假设左、右 JTC 已 active，QP 想同时控制全部 12 轴，应首先检查什么？

**答案**：两个 JTC 的 command interface 声明、QP 的资源需求、切换事务与硬件接口兼容性。目标是在同一协同模式下无重复命令所有者。

### 题 4：固定抓持验证边界

Gazebo 中把物体绑到虚拟物体系并沿轨迹移动，是否证明气动夹爪的真实夹持稳定性？

**答案**：不能。只能证明所建理想抓持模型下的相关运动学或控制效果。

### 题 5：故障恢复

左臂发生约 1 秒 ServoJ stall，停止后网络恢复，是否应该立即恢复 C4.3 原轨迹？

**答案**：不应仅凭通信恢复就自动继续。需检查两侧位置、物体抓持情况、参考时序、命令状态与故障原因，再经安全恢复流程。

---

## 十三、本课验收清单

- [ ] 能准确画出 `QP Core → ControllerInterface → hardware interface → ServoJ/Gazebo` 的数据流。
- [ ] 能区分 `Δq`、`qdot`、`q_cmd`、`q_state` 的物理含义与单位。
- [ ] 能通过 ROS 2 CLI 确认控制器状态和命令接口所有权。
- [ ] 能说明 125 Hz QP、500 Hz CM 和 125 Hz ServoJ 的差异。
- [ ] 能解释为何两个 JTC 不应与一个双臂 QP 控制器争夺相同关节。
- [ ] 能给出无物体、理想固定抓持、故障注入的递进验证顺序。
- [ ] 能说清 D7 HOLD、机械臂停稳和设备安全停机的区别。
- [ ] 能为 C4.3 提供物体任务、误差、输出、诊断及故障边界的接口设计。

## 十四、与前七课的连接

| 已学课程 | 在本课的位置 |
|---|---|
| 运动学与 Jacobian | 控制器更新时构造任务 |
| 双臂任务数学模型 | 相对任务、物体任务的目标行 |
| QP 最优化 | 由 Core 求出关节增量 |
| 零空间和奇异性 | QP 的次目标与退化机制 |
| CBF 与 D7 HOLD | 安全监控、故障状态与执行保护 |
| 本课系统集成 | 让上述结果成为有效设备命令 |

## 第九课预告：C4.3 双臂固定抓持协同搬运综合实践

最后一课会把模型、控制器、物体轨迹、理想抓持和验收报告串起来，形成可逐步实施的双臂搬运 demo；同时明确与原计划“支撑式欠约束搬运”的范围区别。

---

### 外部参考（ROS 2 官方文档）

1. ros2_control Humble Controller Manager：  
   https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html
2. ros2_control Humble CLI：  
   https://control.ros.org/humble/doc/ros2_control/ros2controlcli/doc/userdoc.html
3. Gazebo Classic 接口：  
   https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html
4. Gazebo Sim 接口：  
   https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html

**课程备注**：项目仓库的实时版本、控制器名称、配置和实际诊断字段都必须以现行部署核查为准。理论上合理的方案与实际已通过验收的实现，必须在实验报告中分别表述。

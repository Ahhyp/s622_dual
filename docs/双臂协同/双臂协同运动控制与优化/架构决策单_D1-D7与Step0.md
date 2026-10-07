# 架构决策单：D1–D7 + Step 0 定义

> 配套文档：`修订版执行规划_现状对齐.md`（v2）。本文把 GPT 评审意见落成可执行的架构决策，
> 并把 Step 0 从"选型研究"改写成"验证现有 DH + 建立统一双臂相对运动学"的小里程碑。
> 日期：2026-09-24　适用：`~/my_S622`
> 阅读提示：外部评审无需仓库权限，§1 已含全部事实核对。

---

## 0. 一句话结论

**采纳 D1–D7。** 运动学不引 Pinocchio，先证明并复用现有 `DHKinematics`；
QP 数学层是速度级、执行层统一输出 **position command**；第一版用
"硬约束 + 带 slack 的高权重相对任务 + 加权次任务"；新增 **D7 双臂 stream 健康联锁**。

---

## 1. 事实核对（逐条对照仓库，全部成立）

| GPT 说法 | 核实结果 | 证据 |
|---|---|---|
| `dh_kinematics` 已有 FK / 工具 FK / 解析 Jacobian | ✅ | `fkineFlange/fkine/fkineAll/jacobianFlange/jacobian` |
| DH `d6=0.100` + tool `0.1168` = URDF grasp `0.2168` | ✅ | `dh_kinematics.cpp` 注释明确写出该等式；`ik_params.yaml: tool.gripper.xyz=[0,0,0.1168]` |
| Gazebo 双臂 command interface = velocity | ✅ | `dual_arm_controllers.yaml`：左右臂 `command_interfaces: [velocity]`（手是 position） |
| 真机强制 position command | ✅ | `fairino_hardware_interface.cpp` 校验 `command_interfaces == position`；`s622_real_ros2_control.xacro` 只有 `position` |
| Gazebo CM 100Hz / 真机 CM 500Hz / ServoJ 125Hz | ✅ | `dual_arm_controllers.yaml: update_rate 100`；`real_controllers.yaml: update_rate 500`；插件 `servoj_cmd_t 0.008` |
| 左右 base 一个 yaw=π、一个 yaw=0 | ✅ | `world_T_left_base=(0.35,0,0),yaw=π`；`right=(-0.35,0,0),yaw=0` |

**补充两个影响实现的事实（GPT 未展开）**：
1. `jacobianFlange()` 的列向量是 `[z_i × (o_n − o_i); z_i]`，其中 `z_i`、`o_i` 全部来自 `fkineAll(q)`
   → **Jacobian 表达在各臂自己的 base 系，twist 约定为 `[v; ω]`（spatial）**。
   因此构造 `J_rel` 前，必须先用 **SE(3) adjoint** 把左右统一到同一表达系；`[-J_L, J_R]` 不能原样写。
2. 仿真里"臂=velocity、手=position"是两套接口混用；QP profile 必须单独一套，不与现有 JTC profile 混跑。

---

## 2. 决策 D1–D7

### D1 运动学后端：复用现有 DH，MoveIt/KDL 做 oracle，暂不引 Pinocchio

- **运行时**：`DHKinematics`（base 系 FK + 解析 Jacobian + 工具模型 GRIPPER）。
- **验收**：随机 500–1000 组 q，对 `grasp_frame` 做 FK / J 对拍（对 MoveIt/KDL），再做有限差分验证；
  误差稳定在 **~1e-5 量级以内**即定案。
- **不通过才换** Pinocchio。将来需要 URDF 自动建模 / 任意 frame Jacobian / 动力学 M,C,g 时再引。
- **收益**：关键链自持，不外包；且与现有规划器/IK 同源。

### D2 QP 分层与命令接口

- 三层解耦：
  `dual_arm_kinematics` → `dual_arm_qp_core`（纯 C++，可单测/benchmark） → `dual_arm_qp_controller`（ros2_control）。
- **数学层**：求 `dq*`；**执行层**：`q_cmd,k+1 = q_cmd,k + dq*·Δt`，输出 **position command**。
- **验收**：QP 激活时左右 JTC 必须 inactive；12 个 position interface 只被 QP 占用；切回 JTC 必须原子切换。
- **禁止**：为省事让 Gazebo 同时暴露 position+velocity 两套臂接口（资源互斥未必能阻止同一关节两种命令同时生效）。

### D3 第一版优化语义：硬约束 + slack 相对任务 + 加权次任务

- **硬约束**：关节限位、速度限幅；后续 CBF 安全约束也进这一层。
- **相对位姿**：`J_rel·dq + α·e_rel = s`，slack `s` 重罚 `w_s‖s‖²`
  → 冲突时 QP 仍可解，且能直接量化"为可行性违反了多少相对约束"。
- **次任务**：关节居中、manipulability、双臂舒展度做低权重。
- **不**给相对任务塞 `1e6` 权重；**暂不上** lexicographic / hierarchical QP。
- **纠正**：存在 active 不等式约束时，零空间投影 **不等价于**严格层级 QP；将来要严格优先级应做 HQP。

### D4 frame / anchor

- `world` = 绝对参考；`left_grasp_frame` / `right_grasp_frame` = 协同控制点。
- **无固定 anchor arm**；协同模式启动时记录 `^L T_R* = (^W T_L)^{-1} · ^W T_R` 作为目标，之后保持。
- C2.4 再引入虚拟 `object/coordination_frame`（两 grasp frame 中间）描述整体运动。
- Step 0 必须用 **adjoint** 统一表达，并做"两臂施加同一刚体 twist → 相对 twist = 0"的单元测试。

### D5 三种频率分开

| 频率 | 值 | 说明 |
|---|---|---|
| QP solve/update | **125Hz (Δt=8ms)** | 算法定义 |
| 仿真 QP profile 的 CM | 125Hz | 一拍一算，最简单 |
| 真机 CM | **保持 500Hz 不动** | 已验证配置，不为 QP 改动 |
| 真机 ServoJ | 125Hz | 插件 io_loop |

真机将来让 QP 每 8ms 才推进一次 reference，而不是每个 2ms CM tick 都积分。

### D6 求解器

- **OSQP 首发**，但包一层 **solver backend**，控制器代码不绑死 OSQP。
- 固定稀疏结构 + warm start；典型问题连跑 1 万次，目标 **P99 明显 < 1ms、nominal 失败率 0**；
  达不到再换 ProxQP / qpOASES。

### D7（新增，优先级最高）双臂 stream 健康联锁

- **原则**：协同模式下，任意一臂 command/feedback stream 失去健康 → **两臂一起 hold**，禁止健康那臂继续跑。
- **背景**：ServoJ 每 ~100 次一次 ~1030ms 固有阻塞；两控制箱独立 io_loop，单侧 stall 会让 `T_LR=const` 物理失效。
- **per-arm 健康判据（建议）**：command age、feedback age、io_loop stall flag、跟踪误差超限。
- **状态机**：`NORMAL → DEGRADED → HOLD(双臂) → RESYNC → RESUME | ABORT`。
- **仿真可先行**：在 mock/Gazebo 注入"t=3s 冻结左臂 1s，右臂正常"，正确结果是**双臂冻结并重同步**，
  而不是右臂继续走完。
- 这一条对真机安全的价值，高于 solver 选型。

---

## 3. Step 0 精确定义（小里程碑）

> **目标**：证明现有 DH 可信，并建立"可信的统一双臂相对运动学"。做完 C2 架构不再大返工。
> **范围**：纯 C++ / 单测，不启动 Gazebo。

| # | 任务 | 验收判据 |
|---|---|---|
| S0.1 | DH 对拍 | 随机 500–1000 组 q：FK(`grasp_frame`) 对 MoveIt/KDL，误差 ≤1e-5；解析 J 对有限差分，误差 ≤1e-5 |
| S0.2 | 工具链一致性 | `wrist3 + d6(0.1) + tool(0.1168)` 与 URDF `grasp_frame` 0.2168 一致；GRIPPER 模型位置/姿态一致 |
| S0.3 | 统一到 world | 用 SE(3) adjoint 把左右 base 系 Jacobian 表达统一；构造 `J_rel(6×12)` 与 `e_rel` |
| S0.4 | 关键单元测试 | ① 两臂施加同一刚体 twist → 相对 twist = 0；② `J_rel·dq` 对有限差分相对位姿变化一致 |
| S0.5 | 性能与周期 | 单次 FK+J 耗时；CM 改 125Hz 仿真 profile 后连跑 10min 无失步 |

**产出**：
1. `dual_arm_kinematics`（12 轴 FK/Jacobian/相对位姿/J_rel）；
2. 一组对拍与单测报告（含数字）；
3. 一页结论：DH 是否可信、`J_rel` 约定是否写对。

**非目标**：不装 Pinocchio、不碰 QP、不改真机配置。

---

## 4. 对修订版规划的落地修改

| C 项 | 修改 |
|---|---|
| C2.1 | 由"选型 Pinocchio vs 自研"改为 **"验证现有 DH"**（即本文 Step 0） |
| C2.2 | 明确 **position command profile**；与现有 velocity JTC profile 分离；JTC/QP 互斥 |
| C2.3 | 相对位姿目标定义为 `^L T_R*`，无固定 anchor |
| C2.7 | 并入 **D7 双臂 stream 健康联锁**（不只是距离监控） |
| C2.9 | 明确"单一轨迹源/执行源"，写清 QP 与 JTC、MoveIt Servo、dual_move_server 的边界 |
| C3.4 | 前馈补偿依赖 C1.6 辨识模型，仿真先搭框架 |

---

## 5. 遗留与开放问题

1. OSQP 的 C++ 接口与固定稀疏结构：warm start 后能否稳定 P99 < 1ms（待在 S0.5 之后 benchmark）。
2. `J_rel` 的 twist 约定（spatial vs body）必须在 S0.4 用测试钉死，否则极易写反。
3. D7 在仿真里的 stall 注入方式：mock 侧如何制造"单臂冻结"，需要先设计。
4. 真机 CM 500Hz 下 QP 每 8ms 推进一次的"跳拍"实现细节（S0.5 后再定）。
5. 相对位姿的 `e_rel` 用 SE(3) log 还是位置+四元数误差，需与 OSQP 的稀疏性权衡。

---

*本文为架构决策记录，未修改任何原有规划文件。*

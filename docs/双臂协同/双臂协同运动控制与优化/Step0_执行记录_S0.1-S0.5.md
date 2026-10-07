# Step 0 执行记录：验证现有 DH + 统一双臂相对运动学

> 对应决策单：`架构决策单_D1-D7与Step0.md` §3
> 代码：`src/dual_arm_kinematics/`
> 日期：2026-09-24
> 结论：**D1 成立 —— 现有 DH 可信，作为 C2/C4 的运行时运动学核心；Step 0 完成（S0.5 的 CM 125Hz 部分留到 C2.2）。**

---

## 0. 交付物

| 文件 | 内容 |
|---|---|
| `include/dual_arm_kinematics/se3.hpp` | SE(3)/SO(3)：`adjoint`、`exp/log`、SO(3) Jacobian 及逆（twist 约定 `[v; ω]`） |
| `include/dual_arm_kinematics/dual_arm_kinematics.hpp` | 12 轴统一运动学接口 |
| `src/dual_arm_kinematics.cpp` | 实现（复用 `fairino_planning::DHKinematics`） |
| `test/test_step0_fk_oracle.cpp` | S0.1 / S0.2 / S0.5：DH vs KDL oracle + 工具链 |
| `test/test_step0_relative.cpp` | S0.3 / S0.4 / S0.5：相对位姿 / 相对雅可比 |
| `test/fixtures/s622_dual_arm.urdf` | 由双臂 xacro 生成的 KDL oracle 输入（`scripts/regen_fixture.sh` 可重建） |

---

## 1. S0.1 DH 对拍（对 KDL oracle + 有限差分）

随机 500 组全关节空间构型（关节限位内均匀采样）。

| 指标 | 最大误差 |
|---|---|
| FK(base)，`grasp_frame` | **1.181e-05 m** / **6.282e-04 deg** |
| FK(world) | 1.197e-05 m / 6.292e-04 deg |
| 解析 Jacobian vs KDL | **1.117e-05**（逐元素最大绝对差） |
| 解析 Jacobian vs 有限差分（DH 自洽） | **5.0e-07** |

### ⚠️ 关键发现：1.2e-5 的误差不是约定错误，是 URDF 导出舍入

逐关节核对 URDF 与 DH 表后定位到两处：

| 位置 | URDF | DH | 差 |
|---|---|---|---|
| `j4` 连杆长度（a3） | **-0.24001 m** | -0.240 m | **1e-5 m** |
| `j2/j5/j6` 的 rpy | **1.5708 rad** | π/2 = 1.57079632679 | 3.67e-6 rad/处 |

三个 rpy 舍入累计 ≈ 1.1e-5 rad（对应实测 6.28e-04 deg），再叠加 a3 的 1e-5 m，
正好解释观测到的 ~1.2e-5 m / ~1.1e-5 rad。**其余关节 origin、alpha、d 全部逐位一致。**

因此判定：DH 与 URDF 在物理上一致，误差来自 SolidWorks 导出的数值舍入（10 µm / 微弧度级），
**不影响控制**。测试阈值据此定为 2e-5（真正的 frame/约定错误会是数量级更大）。

---

## 2. S0.2 工具链一致性

- URDF `wrist3_link → grasp_frame` 固定平移 = **0.2168 m**（fixture 两个臂均校验）
- DH：`flange → tool` 平移 = **0.1168 m**，且 `d6(0.1) + tool(0.1168) = 0.2168 m`
- `fkineFlange * toolTransform == fkine(GRIPPER)` 精确一致

---

## 3. S0.3 / S0.4 相对运动学（本 Step 的核心）

约定：`T_rel = (world_T_left_grasp)^-1 · world_T_right_grasp = ^L T_R`；
相对雅可比用 SE(3) adjoint 拼装，**没有**直接写 `[-J_L, J_R]`。

| 测试 | 判据 | 实测 |
|---|---|---|
| `J_rel`(body, R 系) vs 相对位姿有限差分 | <1e-5 | **6.816e-10** |
| `J_rel`(left 系) vs FD | <1e-5 | **7.256e-10** |
| `J_rel`(world spatial) vs FD | <1e-5 | **7.888e-10** |
| 两 base 同时施加共同刚体运动 → 相对位姿/`J_rel` 不变 | <1e-10 | **4.4e-16 / 8.9e-16** |
| **同 body twist → 相对 twist = 0**（adjoint 结构判据） | <1e-10 | **1.831e-14** |
| 误差速率雅可比 vs FD（e=0） | <1e-5 | **6.152e-10** |
| 误差速率雅可比 vs FD（小偏置 e~1e-4） | <1e-3 | 9.565e-05 |

> 说明：误差速率雅可比用了 `J_r^{SE3}(e) ≈ I` 的一阶近似，实测在 e~1e-4 时偏差 9.6e-5，
> 与 O(|e|) 一致。它的精确化留到 C2.2（届时按 QP 需要决定是否补 SE(3) right Jacobian）。

---

## 4. S0.5 性能

| 操作 | 单次耗时 |
|---|---|
| DH FK（单臂） | **15.2 µs** |
| DH Jacobian（单臂） | **8.2 µs** |
| `left_T_right`（双臂） | **43.0 µs** |
| `J_rel(6×12)`（双臂，body） | **112.6 µs** |

对 125Hz（8 ms）预算：`J_rel` 约占 **1.4%**，余量充足。后续 C2.2 会做零分配/预分配优化。

**未完成部分**：S0.5 的「Gazebo CM 改 125Hz profile 连跑 10min 无失步」属于仿真配置改动，
放到 C2.2（QP `ControllerInterface`）一起做。

---

## 5. 结论与对后续的影响

1. **D1 采纳**：运行时运动学用现有 DH；KDL 作独立 oracle。**不引入 Pinocchio。**
2. 相对运动学的 frame/adjoint 约定已被有限差分与结构判据双重钉死，C2 可放心使用。
3. 新增的 `dual_arm_kinematics` 是纯 C++/Eigen 库，可在单测/QP benchmark 中脱离 Gazebo 使用，
   符合决策 D2 的三层解耦。
4. 遗留：C2.2 需要补 `J_r^{SE3}` 精确化（如需）、CM 125Hz 仿真 profile、以及 QP 的零分配优化。

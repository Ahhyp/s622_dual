# 阶段 1：遇到的问题与解决方案

---

## 问题 1：Gazebo Fortress 忽略 prismatic joint limit=0 导致夹爪单指不动作

**日期：** 2026-06-28

**现象：**
hand_controller 发送开夹爪命令 `finger1_joint=0.025, finger2_joint=-0.025`，finger2 正常移动，finger1 纹丝不动。controller 返回 SUCCESS，但 `/joint_states` 和 Gazebo `joint_state` 中 finger1 位置始终为 0。

**原因：**
URDF 中 finger1_joint 的 `<limit lower="0.0" upper="0.0305"/>` — Gazebo Fortress 在解析 SDF 时会忽略值为 0 的 limit 字段，将 `lower=0` 当作"无下限"。控制器发的正方向位移命令被 Gazebo 静默拒绝，关节无法从 0 位置启动。

同理，finger2 的上限 `upper="0"` 也被忽略——但 finger2 的命令是负方向 `-0.025`（由 `lower="-0.0305"` 界定），所以能动。

**解决：**
将 URDF 中 0 值 limit 改为极小非零值：

| 关节 | 字段 | 旧值 | 新值 |
|------|------|------|------|
| finger1_joint | lower | `0.0` | `-1e-6` |
| finger2_joint | upper | `0` | `1e-6` |

改动仅 1 微米，物理无影响。

**涉及文件：** `src/s622_moveit_descriptions/urdf/s622_moveit_descriptions.urdf`

**教训：** Gazebo Fortress 对值为 0 的 joint limit 有特殊处理（视为无限制），prismatic joint 的 limit 必须避开精确 0 值。

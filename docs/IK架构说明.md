# IK 架构说明

## 概述

项目中有三处与 IK 相关的东西，但只有两处在实际运行：

| # | 组件 | 位置 | 状态 |
|---|------|------|------|
| ① | `/fairino/get_all_ik` service | `fairino_planning_core` 独立节点 | ✅ 在用 |
| ② | KDL 数值 IK | `kinematics.yaml` → move_group | ✅ 在用（MoveIt 内部 / fallback） |
| ③ | FairinoIKPlugin | `fairino_planning_ros` | ❌ 未启用 |

---

## 两套 IK 系统详解

### 1. fairino 解析 IK（`/fairino/get_all_ik` service）

| 项目 | 说明 |
|------|------|
| 实现 | `fairino_planning_core/src/ik/fairino_ik.cpp` |
| 方法 | DH 参数 → 腕心点几何法，**闭式解析解**（不是 KDL） |
| 返回 | **所有可能解**（最多 8 组），经 FK 回代验证 |
| 进程 | **独立节点** `fairino_ik_service_node`，不是 move_group |
| DH 来源 | `fairino_planning_core/config/fairino_ik_service.yaml`，yaml 里直接写死 dh_d / dh_a / dh_alpha |
| 调用方 | `moveit_planner.py` → `plan_to_pose_smart()` → `_call_all_ik()` |

**关键**：这个 service 和 move_group 是两个独立进程，互不依赖。DH 参数只存在于这个节点内部，不注入 move_group。

### 2. KDL 数值 IK（MoveIt 内部）

| 项目 | 说明 |
|------|------|
| 配置 | `src/s622_moveit_config/config/kinematics.yaml` |
| 插件 | `kdl_kinematics_plugin/KDLKinematicsPlugin` |
| 方法 | 雅可比迭代，数值逼近 |
| 返回 | **单个解**（依赖种子状态） |
| 参数来源 | **URDF 的 `<joint>` 标签**（关节轴、连杆长度），不需要 DH 参数 |
| 调用方 | `pymoveit2.move_to_pose()` → MoveIt 内部 `/compute_ik` |

**关键**：KDL 从 URDF 自动推导运动学链，不需要手工喂 DH 表。`kinematics.yaml` 里配的 KDL **是真的在用的**。

### 3. FairinoIKPlugin（存在但不可用）

| 项目 | 说明 |
|------|------|
| 实现 | `fairino_planning_ros/src/fairino_ik_plugin.cpp` |
| 注册 | `fairino_planning_ros/plugins/fairino_planning_plugins.xml` |
| 类型 | MoveIt 插件格式的解析 IK（`kinematics::KinematicsBase`） |
| DH 参数 | 期望从 ROS param `fairino_ik.dh.a/alpha/d/theta_offset` 读取 |
| 问题 1 | `kinematics.yaml` 未配置此插件（配的是 KDL） |
| 问题 2 | `fairino_ik.dh.*` 参数从未在任何 yaml 或 launch 文件中设置 |
| 问题 3 | 即使改 `kinematics.yaml` 启用，也会因找不到 DH 参数初始化失败 |

---

## 运行时调用链

```
BT 行为树 MoveToPose target_pose=...
  │
  ▼
move_to_pose_server.py
  │
  ▼
plan_to_pose_smart()
  │
  ├─① _call_all_ik()  →  /fairino/get_all_ik  ← fairino 解析法（闭式解，不是 KDL）
  │    └─ 失败? → fallback 到 plan_to_pose() → moveit2.move_to_pose() → KDL
  │
  ├─② _score_ik()  →  评分排序（关节限位惩罚 + 运动距离）
  │
  └─③ plan_to_joint_positions()  →  moveit2.move_to_configuration()
       └─ MoveIt 只做碰撞检测 + 轨迹插值，不需要再算 IK
```

### 谁会绕过多解 IK

| 调用路径 | 是否走 fairino 解析 IK |
|----------|----------------------|
| BT `MoveToPose target_pose=...` | ✅ 走 `plan_to_pose_smart` → `/fairino/get_all_ik` |
| BT `MoveToPose named_pose=...` | ❌ 直接 `plan_to_joint_positions`（已知关节角，不需要 IK） |
| `yolov8_grasping/arm_executor.py` | ❌ 直接 `moveit2.move_to_pose()`，MoveIt 内部走 KDL |
| `moveit_planner.py` `plan_to_pose()` | ❌ 直接 `moveit2.move_to_pose()`，MoveIt 内部走 KDL |

---

## 死代码

1. `moveit_planner.py` 中 `_compute_ik_kdl()`（line 143-178）：定义了 `/compute_ik` service client，但从未被调用。实际 fallback 是 `plan_to_pose()` → pymoveit2 内置 MoveIt IK。

2. `fairino_planning_ros/FairinoIKPlugin`：MoveIt 插件已注册，但 `kinematics.yaml` 未配置，DH 参数未注入，初始化即失败。

---

## 对双臂的影响

1. `kinematics.yaml` 需要添加 `left_arm:` / `right_arm:` 条目，继续用 KDL 即可，不需要碰 FairinoIKPlugin。
2. `/fairino/get_all_ik` service 当前只支持单臂（`joint_names: [j1..j6]` 无前缀）。双臂时需要决定：加 `left_`/`right_` 前缀支持，或提供两个 service 实例。
3. `moveit_controllers.yaml` 需添加 `left_arm_controller` / `right_arm_controller` / `left_hand_controller` / `right_hand_controller`。

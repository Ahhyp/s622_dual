# 架构迁移：move_group 服务根级 remap → namespaced（对齐 robotarm）

- **日期**：2026-08-23
- **状态**：✅ 已完成（用户仿真验证通过：RViz Plan & Execute 驱动机械臂正常）
- **动机**：详见下方"决策背景"

---

## 1. 决策背景

**迁移前（旧架构）**：
- move_group_fairino 的服务/action 通过 `root_service_remaps` **remap 到根级**
  （`/plan_kinematic_path`、`/compute_ik`、`/move_action` 等 9 服务 + 2 action × 5 topic）
- 现有客户端（visual_servo / arm_actions / collect.py）连根级，无感使用
- **问题**：
  1. KDL 实例服务留在 `/move_group_kdl/*`（未 remap），对现有客户端**不可达**——"双 IK"形同虚设
  2. 根级唯一：未来双臂需要两个 move_group 同时存在，根级 remap 方案直接失效
  3. remap 列表是隐藏魔法（9 服务 + 10 topic），每加一个服务都要同步维护
  4. 与 robotarm 生态（manipulation_common / pymoveit2 `move_group_namespace`）不兼容

**决策**：对齐 robotarm 的 **namespaced 架构**——move_group 服务留在 `/move_group_fairino/*`，
客户端用 `move_group_namespace` 显式连接。理由：
- 服务归属清晰、多实例天然扩展（双臂就绪）
- `arm_clients={"fairino","kdl"}` + `set_ik` 双 IK 切换真正可用
- 阶段 A 已铺垫：robotarm 版 pymoveit2 的 `move_group_namespace` 参数就是为此设计

---

## 2. 实际改动（8 个文件）

### launch 层（服务端）

| 文件 | 改动 |
|---|---|
| `src/gz_launch/launch/s622_gazebo.launch.py` | 删 `root_service_remaps`（fairino 服务留在 `/move_group_fairino/*`）；`monitored_planning_scene` 不再拉到根级（原来两个 move_group 都发布到同一 topic，隐藏冲突）；RViz 节点加 fairino remap（含 action 5-topic 兜底） |
| `src/s622_moveit_config/launch/dual_ik_move_group.launch.py` | 同样删 `root_service_remaps` + `monitored_planning_scene` remap |

### 客户端（显式连 namespace）

| 文件 | 改动 |
|---|---|
| `src/visual_servo/visual_servo/moveit_planner.py` | `MoveItPlanner` 加 `move_group_namespace` 参数（默认 `/move_group_fairino`） |
| `src/visual_servo/visual_servo/visual_servo_node.py` | `_check_reachable` 的 IK 客户端改连 `/move_group_fairino/compute_ik` |
| `src/s622_arm_actions/s622_arm_actions/moveit_planner.py` | 加 namespace 参数 + `_plan_client` 改连 namespaced 服务 |
| `src/s622_arm_actions/scripts/handeye/collect.py` | `MoveIt2` 加 `move_group_namespace="/move_group_fairino"` |
| `src/manipulation_common/manipulation_common/nodes/motion_demo_node.py` | 根级改 `/move_group_fairino` |

### RViz 修复（迁移后发现）

| 文件 | 改动 | 问题 |
|---|---|---|
| `src/gz_launch/launch/s622_gazebo.launch.py` | remap 源名 `query_planner_interfaces`（复数）→ `query_planner_interface`（单数） | move_group 服务端和 RViz 客户端实际服务名都是**单数**，复数 remap 不生效 → "NO PLANNING LIBRARY LOADED" |
| `src/gz_launch/rviz/gz_launch.rviz` | `Move Group Namespace: ""` → `/move_group_fairino` | 面板自己加 namespace 前缀（对齐 robotarm），remap 成为冗余兜底 |

---

## 3. RViz bug 根因（重要经验）

**现象**：迁移后 RViz 正常显示机械臂，但：
- MotionPlanning 面板 Context → Planning Library 显示 "NO PLANNING LIBRARY LOADED"
- Planning 改 Goal State 后橙色机械臂无反应
- Plan & Execute 终端无报错、有成功 log，但不驱动机械臂

**根因**：RViz 面板查询可用规划器列表的服务名是 `query_planner_interface`（**单数**，从
`libmoveit_motion_planning_rviz_plugin_core.so` 和 `libmoveit_move_group_default_capabilities.so`
二进制字符串确认），而我们的 remap 写成了复数 `query_planner_interfaces` → remap 源名不匹配
→ 面板连根级 `/query_planner_interface`（迁移后无服务）→ 面板"瞎了"。

**为什么 robotarm 写复数也能用**：robotarm 的 RViz 配置 `Move Group Namespace: /move_group_fairino`
让面板**自己加前缀**，不依赖 launch remap。我们之前 RViz 配置是空 namespace，全靠 remap，写错就断。

**经验**：
1. RViz MotionPlanning 面板连接 move_group 的**首选方式**是配置 `Move Group Namespace`（非空），
   不是 launch remap（面板源码里服务名是单数，remap 容易写错）
2. 服务名单复数以**二进制字符串**为准（`strings lib*.so | grep service_name`），不要凭 srv 文件名猜
   （srv 文件 `QueryPlannerInterfaces.srv` 是复数，但实际服务名是单数 `query_planner_interface`！）

---

## 4. 验证结果

| 项 | 结果 |
|---|---|
| 语法检查 7 文件 | ✅ |
| manipulation_common 37 单测 | ✅ |
| 双 launch 解析 | ✅ |
| 用户仿真：RViz Plan & Execute 驱动机械臂 | ✅（修复后） |
| 服务位置（用户确认） | ✅ `/move_group_fairino/plan_kinematic_path` 等 namespaced 服务存在 |

---

## 5. 遗留 / 后续

1. **双臂文件未动**（本轮刻意跳过，双臂阶段再处理）：
   - `s622_arm_actions/launch/arm_actions_dual.launch.py`（客户端 remap 到根级服务）
   - `dual_move_server.py`（`/plan_kinematic_path` 根级引用）
   - `s622_dual_arm.launch.py` / `s622_table.launch.py`（单 move_group）
2. **死代码未动**：`visual_servo_node_old.py` / `copy_visual_servo_node.py`（未注册）、`test_learning/ik_solver.py`（学习用途）
3. **阶段 D 待办**：retime（TOTG）验证（trajectory_retime_server 加 launch 节点）、motion_control stop 链路实测
4. **KDL 实例现在真正可达**：客户端可 `arm_clients={"fairino","kdl"}` + `set_ik("kdl")` 切换（未实测，待阶段 D/双臂）

---

## 6. 回滚

- launch：`git checkout -- src/gz_launch/launch/s622_gazebo.launch.py src/s622_moveit_config/launch/dual_ik_move_group.launch.py src/gz_launch/rviz/gz_launch.rviz`
- 客户端：`git checkout -- src/visual_servo src/s622_arm_actions src/manipulation_common`
- 注意：回滚后客户端连根级、launch 恢复 root_service_remaps，二者必须**同时**回滚（配对使用）

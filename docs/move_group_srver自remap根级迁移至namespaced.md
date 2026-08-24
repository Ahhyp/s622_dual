## 架构对比：根级 remap vs namespaced

### 先看清我们当前架构的真实情况

```
我们（现在）：
  /move_group_fairino/*  →  remap 到根级  →  /plan_kinematic_path 等 9 服务 + 2 action
  /move_group_kdl/*      →  保持 namespaced（没 remap！）

  现有客户端（visual_servo / arm_actions / collect.py）全部连根级
  → 实际只能连到 fairino 实例，KDL 实例对现有客户端【不可达】
```

**这是最关键的发现**：我们的"双 move_group"目前对现有客户端是**不对称的**——fairino 借根级可见，kdl 留在 namespace 里无人能连。所谓"双 IK 架构"其实只跑了 fairino 一条腿。

### 对比表

| 维度             | robotarm（namespaced）                              | 我们（remap 根级）                                           |
| ---------------- | --------------------------------------------------- | ------------------------------------------------------------ |
| 服务归属         | 清晰：`/move_group_fairino/*` 一眼可辨              | 隐藏：`/plan_kinematic_path` 不知属于谁                      |
| 多实例扩展       | ✅ 任意多个并存，双臂天然支持                        | ❌ 根级唯一，只能容一个实例                                   |
| 客户端选择 IK    | ✅ `arm_clients={"fairino","kdl"}` + `set_ik` 随时切 | ❌ 根级只能连一个（fairino）                                  |
| remap 魔法       | 无                                                  | 9 服务 + 10 topic 冗长列表，易错（已踩过 action 5-topic 坑） |
| 与 robotarm 生态 | ✅ 零适配（MoveItMotion 就是为它设计的）             | ❌ `move_group_namespace` 参数用不上                          |
| 现有客户端改动   | 需加 1 个参数                                       | 零改动（但这是 M2.6 遗留兼容，非优势）                       |
| 服务发现调试     | `ros2 service list` 直观                            | 需脑内映射                                                   |

### 我的结论：robotarm 的 namespaced 方案更合理

理由按重要性排序：

1. **可扩展性（决定性的）**：根级是全局唯一命名空间。你们项目的既定方向是**双臂协同**（阶段 2-7），双臂必然需要两个 move_group 同时存在——根级 remap 方案在那一刻直接失效。namespaced 是 ROS2 对"多实例"的标准答案（namespace 是一等公民）。

2. **KDL 实例当前是"死配置"**：现有客户端连根级 → 永远碰不到 `/move_group_kdl/*`。要么接受"kdl 只是占位"，要么就是架构没对齐。namespaced 方案下 `set_ik("kdl")` 才能真正生效。

3. **阶段 A 的铺垫恰好指向这个方向**：我们刚替换的 robotarm 版 pymoveit2，`move_group_namespace` 参数**就是为 namespaced 架构设计的**。而且 manipulation_common 的 `PlannerSwitch` / `set_ik` / `arm_clients` dict 设计，全部假设多客户端共存——这暗示 robotarm 整个生态都以 namespaced 为默认。我们用 remap 根级，等于把刚铺好的路又绕开了。

4. **维护成本**：remap 列表是隐藏魔法。以后 move_group 每加一个服务/action，都要同步维护 remap。namespaced 零维护。

### 诚实的反面（为什么当初选了 remap）

根级 remap 是 M2.6 时代"根级单 move_group → 双 move_group"过渡时的**兼容层**：让老客户端（连根级）无感迁移。这个动机当时合理，但它是**过渡态**，不是**终态**。现在阶段 A/B/C1 已经完成，正是把过渡态收拾干净的时机。

### 迁移成本（如果要做）

其实不大，因为阶段 A 已铺垫好：

| 步骤              | 改动                                                                    |
| ----------------- | ----------------------------------------------------------------------- |
| 1. launch         | 删掉 fairino 的 `root_service_remaps`（约 15 行）                       |
| 2. 3 个现有客户端 | 各加 `move_group_namespace="/move_group_fairino"`（1 行/处）            |
| 3. RViz           | 面板 remap 到 fairino namespace（照 robotarm `moveit_stack.py:95-104`） |
| 4. 验证           | 抓取闭环 + motion_demo + 单测回归                                       |

工作量小，但**收益是架构正确 + 双臂就绪 + kdl 真正可用**。

---

开始迁移：

## 架构迁移代码已完成，请启动仿真验证

### 改了什么（7 个文件）

**launch 层（服务端）**：
1. `src/gz_launch/launch/s622_gazebo.launch.py` —— 删 `root_service_remaps`（fairino 服务留在 `/move_group_fairino/*`）；RViz 加 fairino remap（含 action 5-topic 兜底）；`monitored_planning_scene` 不再拉到根级（之前两个 move_group 都发布到同一 topic，隐藏冲突）
2. `src/s622_moveit_config/launch/dual_ik_move_group.launch.py` —— 同样删 `root_service_remaps` + `monitored_planning_scene`

**客户端（显式连 namespace）**：
3. `visual_servo/moveit_planner.py` —— `MoveItPlanner` 加 `move_group_namespace` 参数（默认 `/move_group_fairino`）
4. `visual_servo/visual_servo_node.py` —— `_check_reachable` 的 IK 客户端改连 `/move_group_fairino/compute_ik`
5. `s622_arm_actions/moveit_planner.py` —— 加 namespace 参数 + `_plan_client` 改连 namespaced 服务
6. `s622_arm_actions/scripts/handeye/collect.py` —— `MoveIt2` 加 namespace
7. `manipulation_common/nodes/motion_demo_node.py` —— 根级改 `/move_group_fairino`

**验证已做**：语法 ✓、37 单测 ✓、双 launch 解析 ✓、安装代码确认带 namespace ✓

### 验证步骤（同之前三终端）

**终端 1：仿真**
```bash
source /opt/ros/humble/setup.bash
source ~/my_S622/install/setup.bash
ros2 launch gz_launch s622_gazebo.launch.py
```

**终端 2：YOLO**（同之前命令）→ **终端 3：visual_servo**（同之前命令）→ 触发抓取。

### 观察点（迁移后的关键验证）

1. **visual_servo 抓取闭环**：能跑通 = 客户端 namespace 连接正确（这是最重要的回归）
2. **RViz**：机械臂姿态应正常显示（fairino remap 生效），可试 Plan & Execute
3. **服务位置**：启动后可 `ros2 service list | grep move_group` —— 应看到 `/move_group_fairino/plan_kinematic_path` 等（不再是根级）
4. （可选）motion_demo 也应正常

有任何报错贴给我。验证通过后我更新文档、暂存 git。
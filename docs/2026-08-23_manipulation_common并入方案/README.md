# manipulation_common 并入方案

- **日期**：2026-08-23
- **目录**：`docs/2026-08-23_manipulation_common并入方案/`
- **目标**：将 robotarm 的 `manipulation_common`（通用操作库）正式纳入 my_S622，渐进接入现有体系
- **状态**：✅ **阶段 A、B、C1、D2 + 架构迁移 已完成**（A 记录见 [阶段A_pymoveit2替换记录.md](阶段A_pymoveit2替换记录.md)，B 见 [manipulation_common_API清单.md](manipulation_common_API清单.md)，C1 见 [阶段C1_demo验证记录.md](阶段C1_demo验证记录.md)，架构迁移见 [架构迁移_namespaced记录.md](架构迁移_namespaced记录.md)，D2/RViz 修复见 [阶段D2_RViz修复记录.md](阶段D2_RViz修复记录.md)），阶段 C2/C3 待执行

---

## 0. 背景与前置事实

| 事实                                                                                     | 结论                                           |
| ---------------------------------------------------------------------------------------- | ---------------------------------------------- |
| `manipulation_common` 已复制在 `src/`（2804 行，8 个模块）                               | git 未跟踪（0 文件），已构建过（install 存在） |
| 模块依赖：仅标准 ROS 消息/库 + numpy/scipy                                               | **零外部依赖**，可独立构建                     |
| 自带 4 个测试（abort_manager / motion_control_node / motion_executor / target_selector） | 可验证                                         |
| **MoveItMotion 依赖 robotarm 定制版 pymoveit2**（`move_group_namespace` 支持）           | **并入的前置障碍**                             |
| my_S622 的 pymoveit2 是原版（无 namespace 支持）                                         | 需升级（阶段 A）                               |
| my_S622 有 `trajectory_retime_server` + `RetimeTrajectory.srv`                           | TOTG retime 可用                               |
| MoveItMotion 的 `_plan_fairino_cartesian` 依赖 move_group `compute_cartesian_path`       | 已 remap 到根级 ✅                              |

**已确认决策**：阶段 A 的 pymoveit2 升级采用**整体替换为 robotarm 版**。✅ **已执行（2026-08-23）**，见 [阶段A_pymoveit2替换记录.md](阶段A_pymoveit2替换记录.md)。

---

## 1. 阶段 A：pymoveit2 整体替换（前置）

### 影响面（已核实）

| 文件                                     | 动作    | 差异                                                                                           |
| ---------------------------------------- | ------- | ---------------------------------------------------------------------------------------------- |
| `pymoveit2/moveit2.py`                   | 替换    | 168 行（`move_group_namespace` 参数 + `__resolve_move_group_name()` + 8 个服务/action 名解析） |
| `pymoveit2/moveit2_gripper.py`           | 替换    | 4 行（namespace 透传）                                                                         |
| `pymoveit2/gripper_interface.py`         | 替换    | 4 行（namespace 透传）                                                                         |
| `pymoveit2/test/test_execution_mutex.py` | 新增    | 测试                                                                                           |
| `package.xml`                            | 加 1 行 | `<exec_depend>trajectory_retime_server</exec_depend>`                                          |
| 其余 20+ 文件                            | 不动    | 0 差异                                                                                         |

### 现有调用方（零改动，已验证兼容）

- `s622_arm_actions/moveit_planner.py`
- `visual_servo/moveit_planner.py`
- `s622_arm_actions/scripts/handeye/collect.py`

三者只用 `MoveIt2()` 基础 API（move_to_pose / move_to_configuration / wait_until_executed / max_velocity 等）。robotarm 版 `move_group_namespace` 默认 `""` = 根级，**行为不变**。

### 步骤

| #   | 动作                                                                     |
| --- | ------------------------------------------------------------------------ |
| A1  | 归档当前 pymoveit2 旧版（git 已跟踪，可回滚；或归档到 `src/archive/`）   |
| A2  | 替换 3 个文件 + 新增 test 文件（复制 robotarm 版）                       |
| A3  | `package.xml` 加 `trajectory_retime_server` 依赖                         |
| A4  | `colcon build --packages-select pymoveit2`（ament_cmake，编译后重装）    |
| A5  | **回归验证**：启动仿真，跑一次 visual_servo 抓取闭环（确认根级行为不变） |

### 验收

1. 现有 3 个调用方正常（根级 move_group）
2. 新代码可传 `move_group_namespace="/move_group_fairino"` 连 namespaced move_group
3. 抓取闭环回归通过

---

## 2. 阶段 B：正式纳入 manipulation_common — ✅ 已完成（2026-08-23）

| #   | 动作                                                                                                                                                                     | 结果 |
| --- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---- |
| B1  | `colcon build --packages-select manipulation_common`（确认可构建）                                                                                                       | ✅ 通过（仅 setuptools tests_require 弃用警告） |
| B2  | 跑 4 个单测（test_abort_manager / test_motion_control_node / test_motion_executor / test_target_selector）                                                               | ✅ **37 passed**（需 `ROS_LOG_DIR=/tmp/ros_log`，沙箱拦截 ~/.ros/log） |
| B3  | `git add src/manipulation_common` 纳入版本管理                                                                                                                           | ✅ 27 文件已跟踪 |
| B4  | 文档：8 个模块 API 清单（MoveItMotion / trajectory_scoring / keepout_manager / detection_cache / target_selector / abort_manager / pose_tools / tf_tools / yaml_loader） | ✅ [manipulation_common_API清单.md](manipulation_common_API清单.md) |

### 验收

- 构建通过 + 4 测试全绿 ✅
- git 纳入（可 diff/回滚）✅

---

## 3. 阶段 C：接入 my_S622（渐进，不破坏主路径）

### C1. 第一个接入点（⚠️ 关键决策，见 §6）

**推荐：独立节点验证**——用 manipulation_common 自带的 `motion_control_node` 起独立 demo：
- 验证 MoveItMotion 全链路（plan + execute + retime + namespace 连接 `/move_group_fairino`）
- 不碰现有 visual_servo / arm_actions / BT 主路径
- 跑通后作为"能力证明"，再决定后续替换范围

### C2. 能力对账（现有 moveit_planner.py vs MoveItMotion）

| 能力                             | 现有 moveit_planner.py    | MoveItMotion                        |
| -------------------------------- | ------------------------- | ----------------------------------- |
| 基础 plan/execute                | ✅                         | ✅                                   |
| 多 IK 客户端切换（fairino/kdl）  | ❌                         | ✅（`set_ik`）                       |
| 路径评分选优（select_best_path） | ❌                         | ✅（腕部运动量评分）                 |
| keepout 禁入区                   | ❌                         | ✅（keepout_manager）                |
| 急停/取消                        | ❌                         | ✅（abort_manager）                  |
| TOTG 时间最优重定时              | ❌（pymoveit2 有，未启用） | ✅（`_retime_trajectory_if_needed`） |
| 夹爪控制                         | ❌（走 controller）        | ✅（control_gripper）                |
| planner/pipeline 运行时切换      | 部分                      | ✅                                   |

### C3. 建议接入顺序

1. motion_control_node 独立 demo（验证链路）
2. （可选）s622_bt_manager 的 BT 节点或 arm_actions 逐步用 MoveItMotion
3. （可选，最后）visual_servo 的 MoveItPlanner 副本替换为 MoveItMotion

每个接入点独立验证 + 回归，避免一步到位。

---

## 4. 阶段 D：验证

| #   | 验证项                                                                |
| --- | --------------------------------------------------------------------- |
| D1  | 每个接入点：仿真 plan + execute 成功                                  |
| D2  | retime（TOTG）生效（需 trajectory_retime_server 启动，launch 加节点） |
| D3  | namespace 连接 `/move_group_fairino` 正常                             |
| D4  | 回归：visual_servo 抓取闭环 / arm_actions / BT / 双臂不受影响         |

---

## 5. 风险与回滚

| 风险                                     | 等级 | 应对                              |
| ---------------------------------------- | ---- | --------------------------------- |
| pymoveit2 替换破坏现有调用               | 低   | 已验证 API 兼容；git 历史可回滚   |
| MoveItMotion 依赖 compute_cartesian_path | 低   | 已 remap 到根级                   |
| retime 需要 trajectory_retime_server     | 中   | 用户包已在，launch 加启动节点即可 |
| MoveItMotion 接入改动主路径              | 中   | 渐进式：先独立节点，不碰主路径    |

**回滚**：pymoveit2 / manipulation_common 均有 git（替换后）可回滚；接入点逐个回退。

---

## 6. 待决策项

1. **第一个接入点**：✅ **已定（2026-08-23）——独立节点验证**（`motion_control_node` demo，验证 MoveItMotion 全链路，不碰主路径）
2. **manipulation_common 是否整体保留 vs 能力吸收进 s622_arm_actions**（当前倾向：整体保留为独立包，能力逐步引用）

---

## 7. TODO（暂缓，已记录）

> **规划管线切换**：当前用 OMPL，TODO 为换 robotarm 的 **FairinoPlannerManager**（BiRRT*/Tube-BiRRT*/AAPF 等自研管线）。manipulation_common 并入完成后再评估。相关参数已随 fairino_planning_core 移植就位（common_planning_params.yaml 等 7 个）。

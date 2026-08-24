# 阶段 D2 验证 + RViz bug 修复记录（2026-08-24）

- **日期**：2026-08-24
- **状态**：✅ 完成（用户仿真验证通过）
- **关联**：架构迁移的后续验证与修复

---

## 1. retime（TOTG）验证 —— D2

### 改动

| 文件 | 改动 |
|---|---|
| `src/trajectory_retime_server/src/retime_server.cpp` | 升级为 **robotarm 完整版**（510 行）：3 参数拉取、TOTG 输入校验（joint 集合完全匹配）、scaling (0,1] 校验、service_name 可配、健壮性检查 |
| `src/trajectory_retime_server/launch/retime_server.launch.py` | 传齐 3 参数（URDF/SRDF/kinematics），**use_sim_time 参数化**（由外层传入，不硬编码） |
| `src/gz_launch/launch/s622_gazebo.launch.py` | 启动链加 retime_server（IncludeLaunchDescription，传 `use_sim_time=true`） |
| `src/manipulation_common/.../motion_demo_node.py` | 加 `cartesian` 参数（验证 retime 用） |

### 验证

- retime_server 启动正常：`Parameter 'robot_description' already exists`、`Loaded robot model: s622`
- 服务存在：`/retime_trajectory`
- 触发条件：pymoveit2 robotarm 版仅在 `cartesian=True` 时调用（关节空间路径不触发）

---

## 2. RViz 三个 bug 的修复链（重要经验）

### Bug 1：RViz 段错误崩溃 + "Link [X] does not exist"（64 次）

**现象**：RViz 崩溃（exit -11），机械臂相关 link 全部报不存在。

**排查过程**（逐层排除）：
1. ❌ 不是 launch remap 单复数——改对后仍崩
2. ❌ 不是双 MotionPlanning display——只有 1 个
3. ❌ 不是 spawn_box 每 7 秒重复——TimerAction 只执行一次（源码确认）
4. ❌ 不是 /clock 回跳——实测单调递增
5. ❌ 不是 obb_node 墙钟时间戳——源码确认全部继承输入图像时间戳
6. ✅ **是 retime_server 缺 use_sim_time**——加入后 jump back 归零

**根因**：retime_server 未设 `use_sim_time`，节点内时间用墙钟，与 RViz（sim time）时间基准不一致 → 周期性 TF jump back → RViz reset → namespace reload → 模型消失（闪）→ 崩溃。

**证据**：加入 retime_server 前后对比——之前日志 jump=0，之后 jump=83；补 use_sim_time 后 jump=0。

**修复**：retime_server.launch.py `use_sim_time` 参数化（默认 false），s622_gazebo.launch.py 传入 `true`。

### Bug 2：RViz 白色/彩色机械臂不同步

**现象**：白色机械臂（RobotModel/TF）与彩色机械臂（Scene Robot/planning scene）位置不一致。

**根因**：RViz 配置 `Planning Scene Topic: /monitored_planning_scene`（根级），迁移后 move_group 发布到 `/move_group_fairino/monitored_planning_scene`（namespaced）→ 彩色 Scene Robot 收不到数据，卡在错误位置。

**修复**：`gz_launch.rviz` → `Planning Scene Topic: /move_group_fairino/monitored_planning_scene`（对齐 robotarm）。

### Bug 3：RViz 面板 kinematics 缺失

**现象**：`No kinematics plugins defined`（21 次）+ `No active joints or end effectors found for group 'robot_arm'`。

**根因**：RViz 节点只传 2 参数（description/semantic），缺 kinematics。

**修复**：补 `robot_description_kinematics` + `joint_limits` + `pipeline_params`（fairino_planning），现在与 robotarm 的 RViz 节点参数**完全一致**（5 参数）。

---

## 3. 经验总结（后续项目可复用）

1. **TimerAction 语义**：ROS2 launch 中 `TimerAction(period=X)` 只延迟执行**一次**，不重复——排查"周期性现象"时不要误判
2. **排查 TF jump back**：先实测 `/clock` 单调性（`ros2 topic echo /clock`），排除时钟后再查节点时间基准
3. **use_sim_time 一致性**：仿真中**所有节点必须统一 use_sim_time**，include 的 launch 也要参数化传入，漏一个就可能导致 TF/RViz 连锁问题
4. **二进制字符串验证**：服务名单复数以 `strings lib*.so | grep` 为准（QueryPlannerInterfaces.srv 复数 vs 实际服务名 query_planner_interface 单数）
5. **RViz 连接 move_group 首选 .rviz 配置**：`Move Group Namespace` + `Planning Scene Topic` 都设 namespaced，不要依赖 launch remap（双路径会振荡）

---

## 4. 当前 RViz 配置（与 robotarm 完全对齐）

| 字段 | 值 |
|---|---|
| Move Group Namespace | `/move_group_fairino` |
| Planning Scene Topic | `/move_group_fairino/monitored_planning_scene` |
| Robot Description | `robot_description` |
| 节点参数 | description + semantic + kinematics + joint_limits + fairino_planning + use_sim_time |

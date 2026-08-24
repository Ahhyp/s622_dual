# FairinoIK 插件移植方案（robotarm 双 move_group 方案落地）

> 文档状态：**阶段 1-3 已完成（2026-08-22）；待端到端仿真验收**
> 创建日期：2026-08-22
> 适用项目：`~/my_S622`

---

## 0. 目标与决策记录

**目标**：在 my_S622 采用 fairino_robotarm-main 的 move_group 方案——`FairinoIKPlugin`（真机验证过的解析 IK）作为 move_group 的 kinematics 插件，与 KDL 构成双 move_group 对比/兜底；IK 主路径从 `/fairino/get_all_ik` service 切换为 move_group 插件。

**用户已拍板决策**：

| 决策项 | 选择 | 备注 |
|---|---|---|
| 工具偏移 `gripper_tool` | **0.1168（robotarm 原值）** | ⚠️ 与 S622 URDF `grasp_frame=0.2168` 不一致，风险见 §5.1 |
| IK 主路径 | **切换为 move_group 插件** | service 路径退役或保留兼容 |
| 执行时机 | **只出方案，暂不执行** | 本文件为审阅稿 |

**核查结论（好消粋）**：robotarm 的 `DHParams` 默认值与 S622 DH 完全一致（`d{0.140,0,0,0.102,0.102,0.100}` / `a{0,-0.280,-0.240,0,0,0}`），DH 参数**零改动**。

---

## 1. 差距分析（已核实，2026-08-22）

robotarm 的 `FairinoIKPlugin`（908 行，核心 `solveIK()` 约 440 行）依赖完整版 IK 链路，你的精简版 `fairino_planning_core`（21 文件）与之不兼容：

| 组件 | robotarm 版 | 你的项目 | 动作 |
|---|---|---|---|
| `types.h` | 兼容 facade | 独立完整定义 | **替换** |
| `types/aliases.hpp` | ✅ | ❌ | 新增 |
| `config/planning_params.hpp`（含 `AnalyticalIKParams`） | ✅ | ❌ | 新增 |
| `model/robot_kinematics_config.hpp`（含 `DHParams`） | ✅ | ❌ | 新增 |
| `request/plan_request_core.hpp` | ✅ | ❌ | 新增 |
| `result/plan_result.hpp` | ✅ | ❌ | 新增 |
| `dh_kinematics.h/cpp` | 93 行（`DHKinematics(DHParams, tool)`） | 43 行（旧 API） | **替换** |
| `ik/fairino_ik.h/cpp` | 173/344 行（构造收 `AnalyticalIKParams`） | 29/150 行（构造收 `d,a,alpha`） | **替换** |
| `ik/ik_selector.h/cpp` | 229 行 | 29 行 | **替换** |
| `fairino_ik_plugin.h/cpp` | 908 行（含 `solveIK`） | ~500 行（无 `solveIK`） | **替换** |
| `config/parameter_loader.hpp/cpp` | ✅ | ❌ | 新增 |
| `pipeline/fairino_planning_pipeline.h`（+`.cpp`） | ✅ | ❌ | 新增（.cpp 视编译需求） |
| `config/ik_params.yaml` | ✅ | ❌ | 新增（**工具偏移按决策改**） |

**插件注册名**：原计划"保持你的注册名 `fairino_planning_ros/FairinoIKPlugin`"，实际执行（整体替换）采用了 robotarm 版注册名 **`fairino_planning/FairinoIKPlugin`**（类型 `fairino_planning::FairinoIKPlugin`）——阶段 2 配置 `kinematics_fairino.yaml` 时必须使用该名。

**`/fairino/get_all_ik` 现有调用方**（切换主路径后需改造）：
- `s622_arm_actions/s622_arm_actions/moveit_planner.py:88,138`（主路径 IK 预检）
- `s622_arm_actions/test/moveit_planner.py:70`
- `s622_arm_actions/scripts/handeye/collect.py:78,122`（手眼标定取全解）
- `gz_launch/launch/s622_gazebo.launch.py:183-195`（启动 `fairino_ik_service_node`）

---

## 2. 移植范围（文件级清单）

> ⚠️ **本节为原计划的分析记录**（增量移植清单）。实际执行改为**整体替换**两个包（见 §3 阶段 1 执行调整说明），§2.1/2.2 所列文件均已随 robotarm 完整版整体就位，不再逐文件操作。

### 2.1 `fairino_planning_core`（原计划：新增 6 + 替换 4 + 适配；实际：整体替换）

**新增**：
```
include/fairino_planning_core/types/aliases.hpp
include/fairino_planning_core/config/planning_params.hpp
include/fairino_planning_core/model/robot_kinematics_config.hpp
include/fairino_planning_core/request/plan_request_core.hpp
include/fairino_planning_core/result/plan_result.hpp
config/ik_params.yaml          # 工具偏移改为 0.1168（按用户决策）
```

**替换**：
```
include/fairino_planning_core/types.h
include/fairino_planning_core/dh_kinematics.h
src/dh_kinematics.cpp
include/fairino_planning_core/ik/fairino_ik.h
src/ik/fairino_ik.cpp
include/fairino_planning_core/ik/ik_selector.h
src/ik/ik_selector.cpp
```

**适配**：`src/nodes/fairino_ik_service_node.cpp` + `src/service/fairino_ik_service.cpp`（旧 `FairinoIK(d,a,alpha)` API → 新 API；若 service 退役则删除并清理引用）

**更新**：`CMakeLists.txt`（新源文件、头文件安装）

### 2.2 `fairino_planning_ros`（原计划：替换 1 + 新增 2-3；实际：整体替换）

**替换**：
```
include/fairino_planning_ros/fairino_ik_plugin.h
src/fairino_ik_plugin.cpp       # → robotarm 908 行版
```

**新增**：
```
include/fairino_planning_ros/config/parameter_loader.hpp
src/config/parameter_loader.cpp
include/fairino_planning_ros/pipeline/fairino_planning_pipeline.h
src/pipeline/fairino_planning_pipeline.cpp   # 视编译链接需求
```

**插件注册**：原计划"不动"（保持 `fairino_planning_ros/FairinoIKPlugin`），实际整体替换后采用 robotarm 版 `plugins/fairino_planning_plugins.xml`（注册 `fairino_planning/FairinoIKPlugin` + `fairino_planning/FairinoPlannerManager`）

**更新**：`CMakeLists.txt`

### 2.3 `s622_moveit_config`（新增 3，阶段 2 执行）

```
config/kinematics_fairino.yaml   # kinematics_solver: fairino_planning/FairinoIKPlugin（robotarm 注册名）, group: robot_arm
config/kinematics_kdl.yaml       # kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
launch/dual_ik_move_group.launch.py  # 双 move_group（参照 robotarm moveit_stack.py 的 remap 设计）
```

---

## 3. 实施步骤

### 阶段 1：整体替换 `fairino_planning_core` + `fairino_planning_ros`（含 IK 插件移植）✅ 已完成

> ⚠️ **执行中发现的重要调整**：原方案为"新增 6 文件 + 替换 4 文件"（分阶段 1/2 分别移植 planning_core 与 planning_ros 插件），实际核查发现 robotarm 的类型体系（`JointConfig`=Eigen 向量、`Pose` 语义变化、`plan()` 接口 7 参数化）与用户旧版（`JointArray`/`PlanRequest`）**完全冲突**，旧规划算法（`bi_rrt_star`/`rrt_tree`/`planning_algorithm`）无法共存——适配旧算法 = 重写 plan 实现。因此实际执行改为**一次性整体替换** `fairino_planning_core`（59 文件）+ `fairino_planning_ros`（19 文件）为 robotarm 完整版，**原"阶段 2：移植 fairino_planning_ros 插件"（908 行 `fairino_ik_plugin.cpp`、`parameter_loader`、`pipeline`）已随本阶段一并完成**，不再单独成阶段。

**执行记录（2026-08-22）**：

| 步骤 | 动作 | 状态 |
|---|---|---|
| 1 | 备份用户特有 `fairino_planning.yaml`（OMPL 配置，gz_launch 依赖） | ✅ |
| 2 | 整体替换 `fairino_planning_core` → robotarm 完整版（59 文件） | ✅ |
| 3 | 整体替换 `fairino_planning_ros` → robotarm 完整版（19 文件） | ✅ |
| 4 | 恢复 `fairino_planning.yaml` 到 `fairino_planning_ros/config/` | ✅ |
| 5 | `colcon build --merge-install --packages-select fairino_planning_core fairino_planning_ros` | ✅ 编译通过（1min 46s） |
| 6 | 单测验证：IK 选择器 10 + BiRRT* 10 + RRTTree 4 + AAPF 8 = **32 个测试全部通过**（含工具偏移 0.1168 可配置测试） | ✅ |
| 7 | 插件验证：`install/share/fairino_planning_ros/plugins/` xml 正确安装；库中 `registerPlugin<FairinoIKPlugin, KinematicsBase>` 符号存在 | ✅ |
| 8 | 清理 install 旧残留（旧版 `fairino_ik_service_node`/`test_core`） | ✅ |
| 9 | **用户旧版归档**（用户提议，优于直接删除）：从 git HEAD 提取旧版到 `src/archive/fairino_planning_core`（21 文件）+ `src/archive/fairino_planning_ros`（13 文件）+ `COLCON_IGNORE` | ✅ |

**替换影响（需在后续阶段处理）**：
- 用户旧版规划算法（简化 BiRRT* 等）已被 robotarm 完整版覆盖，旧版已归档至 `src/archive/`（软删除，随时可查/可恢复）
- 用户特有 `fairino_ik_service_node` / `fairino_ik_service`（`/fairino/get_all_ik` service）已随替换删除（旧版在 archive 中）→ 阶段 3 改 gz_launch launch、阶段 4 改 s622_arm_actions 调用
- 插件注册名变为 `fairino_planning/FairinoIKPlugin`（旧 `fairino_planning_ros/FairinoIKPlugin` 作废）

### 阶段 2：配置双 move_group ✅ 已完成

> 原计划为"新增 3 文件 + 改造 gz_launch"，执行中额外发现并修复了 2 个 M2.8 遗留问题（见下）。

**执行记录（2026-08-22）**：

| 步骤 | 动作 | 状态 |
|---|---|---|
| 1 | 复制 robotarm `kinematics_fairino.yaml` / `kinematics_kdl.yaml` 到 `s622_moveit_config/config/`（group `robot_arm` 与 SRDF 匹配） | ✅ |
| 2 | 新增 `s622_moveit_config/launch/dual_ik_move_group.launch.py`（双 move_group + RSP + JSP，自包含；remap 复制 robotarm `moveit_stack.py:126-149`；kinematics 用 MoveIt 标准格式 `robot_description_kinematics.<group>` 注入） | ✅ |
| 3 | 改造 `gz_launch/launch/s622_gazebo.launch.py`：move_group 单实例 → 双实例；**删除已移除的 `fairino_ik_service_node`**（含 fairino_ik_yaml 引用） | ✅ |
| 4 | 修复 `gz_launch/CMakeLists.txt`：移除已删除的 `launch/gazebo.launch.py` 安装项 | ✅ |
| 5 | **修复 M2.8 遗留 bug**：`robot_gazebo.urdf.xacro` 加显式 `<xacro:s622_arm .../>` 实例化（主 xacro `instantiate` 默认已改 false，单臂顶层此前展开为空 robot——仅 10 link 全是相机；修复后 23 link 完整） | ✅ |
| 6 | 编译 + 端到端验证（见下） | ✅ |

**验证结果**：
- 双 move_group 启动：`/move_group_fairino/move_group` + `/move_group_kdl/move_group` ✅
- kinematics 参数：fairino=`fairino_planning/FairinoIKPlugin` / kdl=`kdl_kinematics_plugin/KDLKinematicsPlugin` ✅
- FairinoIKPlugin 加载日志：`initialized: group='robot_arm', joints=6`；工具偏移 `flange_to_tcp=[0,0,0.1168]`；`tool/URDF check ok: wrist3_link->grasp_frame pos_err=0.000000`（插件自带工具一致性检查通过）✅
- **ComputeIK 端到端**：`/move_group_fairino/compute_ik`（GetPositionIK）目标 (0.35,0,0.30) z-朝下 → `error_code=1 (SUCCESS)`，解 `[0.296,-1.618,-1.107,-1.987,1.571,-1.275]` ✅；KDL 同目标也 SUCCESS ✅

**踩坑记录**：
1. IK 一度返回 `-21 (NO_IK_SOLUTION)`，根因不是 IK 而是**独立 launch 缺 TF**（`TF Problem: base_link does not exist`）——补 `robot_state_publisher` + `joint_state_publisher` 后解决
2. `launch Node parameters` 传空 list `source_list: []` 会报 tuple 类型错——去掉该参数即可
3. 调试期间需 `ROS_LOG_DIR=/tmp/...`（沙箱不允许写 `~/.ros/log`）；SHM 警告为沙箱限制，无碍

### 阶段 3：主路径切换与验证 ✅ 已完成（2026-08-22）

**逐文件改动**（用户要求慢速、逐文件、可读）：

| # | 文件 | 改动 |
|---|---|---|
| 1 | `s622_arm_actions/moveit_planner.py`（442→305 行） | 删 `GetAllIK`/`GetPositionIK`/`math`/`Duration` import；删 `_all_ik_client`+`_ik_client`；删 `_call_all_ik`（调 service 拿全解）；删 `_compute_ik_kdl`（死代码）；删 `_score_ik`（被插件内 IKSelector 覆盖）；`plan_to_pose_smart` 简化为直接 `plan_to_pose`——**解评估由 FairinoIKPlugin 内部 IKSelector（S1-S4 四维评分）完成** |
| 2 | `s622_moveit_config/launch/dual_ik_move_group.launch.py` | move_group_fairino 加**服务端 remap 到根级**：11 个服务 + move_action/execute_trajectory action（展开成 5 底层 topic，与 arm_actions 客户端 remap 对接） |
| 3 | `gz_launch/launch/s622_gazebo.launch.py` | 同上（仿真主路径） |
| 4 | `s622_arm_actions/scripts/handeye/collect.py`（284→214 行） | 删 `GetAllIK`/`_call_ik`/`_score_ik`/`JOINT_SAFETY_LIMITS`；`goto` 简化为直接 `move_to_pose` |
| 5 | `s622_arm_actions/test/moveit_planner.py` | **归档**到 `src/archive/`（旧版实验脚本，docstring 自述"保留一个旧的"，依赖已退役 service） |
| 6 | `gz_launch/launch/s622_dual_arm.launch.py` + `s622_table.launch.py` | 删已移除的 `fairino_ik_service_node` 节点及其 return 引用 |

**验证结果**：
- 6 文件语法 + `colcon build` 通过
- **根级服务恢复**（客户端依赖）：`/plan_kinematic_path`、`/compute_ik`、`/get_planning_scene`、`/compute_fk` ✅（move_group_fairino 服务端 remap）
- **根级 action 恢复**：`/move_action`、`/execute_trajectory` ✅（`ros2 action list` 确认——注意 `ros2 service list` 不显示 action 底层服务，用 action list 验证）
- **根级 IK 回归**：`/compute_ik` → move_group_fairino（FairinoIKPlugin）→ `error_code=1` ✅
- move_group_kdl 保持 namespaced（无根级服务冲突）✅

**链路闭环**：客户端相对名 →（arm_actions launch remap）→ 根级 →（move_group_fairino remap）→ `/move_group_fairino/*`

**遗留事项**：
- `fairino_msgs/GetAllIK.srv` 仍存在但无调用方（可留作厂商接口，可选清理）
- `s622_table.launch.py`/`s622_dual_arm.launch.py` 的 move_group 仍为单实例（KDL IK）——如需 Fairino IK 再升级双实例（双臂阶段处理）
- **待端到端验收**：`s622_gazebo.launch.py` 完整启动（Gazebo）跑通一次 pick-place（需用户环境 GUI，见 §6 验收标准 6）

---

## 4. 联动修改汇总

| 文件 | 改动 | 阶段 |
|---|---|---|
| `fairino_planning_core` | 整体替换为 robotarm 完整版（59 文件） | ✅ 阶段 1 |
| `fairino_planning_ros` | 整体替换为 robotarm 完整版（19 文件，含 908 行 `fairino_ik_plugin.cpp`） | ✅ 阶段 1 |
| `fairino_planning_ros/config/fairino_planning.yaml` | 用户 OMPL 配置，已恢复 | ✅ 阶段 1 |
| `s622_moveit_config/config/kinematics_fairino.yaml` | 新增（solver=`fairino_planning/FairinoIKPlugin`） | 阶段 2 |
| `s622_moveit_config/config/kinematics_kdl.yaml` | 新增（KDL） | 阶段 2 |
| `s622_moveit_config/launch/dual_ik_move_group.launch.py` | 新增（双 move_group） | 阶段 2 |
| `gz_launch/launch/s622_gazebo.launch.py` | 双 move_group 节点 + 移除 `fairino_ik_service_node` | 阶段 2 |
| `s622_arm_actions/moveit_planner.py` | IK 预检改走 move_group | 阶段 3 |
| `s622_arm_actions/scripts/handeye/collect.py` | 全解获取改走插件/移除 | 阶段 3 |
| `fairino_ik_service_node` / `fairino_ik_service` | 已随阶段 1 删除（旧版在 `src/archive/`） | ✅ 阶段 1 |

---

## 5. 风险与注意事项

### 5.1 ⚠️ 工具偏移不一致（最高风险，用户决策已选 0.1168）
- 插件内部 FK/IK 用 `gripper_tool=0.1168`，而 MoveIt `robot_description` 里 `wrist3_link→grasp_frame=0.2168`
- 后果：**仿真中** MoveIt 规划的末端位姿与插件求解的关节角可能不一致 → 规划验证失败或执行偏差；**真机**若实际工具是 0.1168 则无此问题（但仿真 URDF 需同步改为 0.1168 保持一致性，夹爪距离参数需重新验证）
- 建议：执行时先跑一次"FK 一致性检查"（插件 FK vs MoveIt FK 输出对比），偏差 > 1mm 就必须统一两处值

### 5.2 `types.h` 替换的连锁影响
`fairino_planning_core` 其他代码（`bi_rrt_star`、`fairino_ik_service`、`test_core` 等）都基于旧 `types.h` API（`JointArray`/`Pose`/`JointLimits`），替换后需一并适配编译；robotarm 版 `types.h` 是 facade，真身在各子模块，`Pose` 等类型定义位置变化

### 5.3 `ik_selector` 229 行版依赖参数文件
新版 `ik_selector` 依赖 `ik_params.yaml` 的评分/连续性/安全裕度参数，参数缺失会运行时异常——`ik_params.yaml` 必须随插件一起就位

### 5.4 `pipeline` 头连锁
`parameter_loader.hpp` include `pipeline/fairino_planning_pipeline.h`，后者依赖 `algorithms/planning_algorithm.h`（你的精简版与 robotarm 版可能 API 不同）——若编译报错需连带移植 `planning_algorithm` 或最小化 pipeline 头

### 5.5 service 退役风险
`/fairino/get_all_ik` 被 `handeye/collect.py` 依赖（标定采集需要全解列表），直接删除会破坏标定流程——切换后该脚本需改用 move_group 插件获取全解，或保留 service 双轨一段时间

### 5.6 真机验证的"延续性"
robotarm 的 IK 在其机器人+工具配置下真机验证过；移植后 DH 参数一致（✅），但**工具偏移按你决策用 0.1168**，真机部署前必须用实际工具确认该值（法兰→夹爪中心实测），否则"验证过"不成立

---

## 6. 验收标准

1. `colcon build`（全量）编译通过，无 `fairino_planning_core`/`fairino_planning_ros` 链接错误 ✅（阶段 1 已验收）
2. 插件注册验证：`install/share/fairino_planning_ros/plugins/` xml 正确安装（`fairino_planning/FairinoIKPlugin`）；库中 `registerPlugin<FairinoIKPlugin, kinematics::KinematicsBase>` 符号存在 ✅（阶段 1 已验收）；阶段 2 启动 move_group 时插件可被加载（无 `KinematicsPlugin ... not found` 报错）
3. 双 move_group 启动：`ros2 node list | grep move_group` 出现 `move_group_fairino` 和 `move_group_kdl`（阶段 2 验收）
4. **IK 正确性**：robotarm 版单测 32 个全部通过（IK 选择器 10 + BiRRT* 10 + RRTTree 4 + AAPF 8）✅（阶段 1 已验收）；阶段 3 对比 move_group 插件 IK 与 KDL 解（容差内）
5. **FK 一致性**：插件 FK vs MoveIt `robot_description` FK，末端偏差 < 1mm（工具偏移不一致会在此暴露；当前 0.1168 vs URDF 0.2168 需重点检查，见 §5.1）
6. 单臂仿真视觉抓取（`s622_gazebo.launch.py` + YOLO + visual_servo）完整跑通一次 pick-place（阶段 3 验收）
7. 双臂仿真（`s622_dual_arm.launch.py`）不受影响（双臂走 OMPL 12DOF 规划，不依赖单臂 IK 插件，但需回归）

---

## 7. 回滚

- 所有替换文件均有 git 历史（`git checkout` 恢复）；旧版另有 `src/archive/` 软删除副本
- 若阶段 1 编译失败：还原 `fairino_planning_core`/`fairino_planning_ros`（从 archive 或 git 恢复）
- 主路径切换（阶段 3）前：`moveit_planner.py` 先备份，确认新路径（move_group 插件 IK）验证通过后再切换

---

## 8. 参考文档

| 文档 | 用途 |
|---|---|
| `docs/IK架构说明.md` | 现有单 move_group + service 主路径的架构说明 |
| `docs/机械臂参数.md` | S622 DH 参数（与 robotarm DHParams 对照依据） |
| `fairino_robotarm-main/src/gazebo_launch/docs/gazebo_launch架构.md` | 双 move_group 设计依据 |
| `fairino_robotarm-main/src/gazebo_launch/launch_utils/moveit_stack.py` | 双 move_group 启动实现参考 |
| `docs/AI_CONTEXT.md` | grasp_frame 偏移 0.2168、抓取参数 |

# 规划管线统一：OMPL → FairinoPlannerManager

> 日期：2026-08-25
> 目标：当前仿真实际跑的是 OMPL（"fairino" 名目下指向 `ompl_interface/OMPLPlanner`，历史遗留错误配置），统一为 robotarm 的 **FairinoPlannerManager**（BiRRT*/Tube-BiRRT*/AAPF/RRT* 自研管线），对齐 robotarm 的 launch 注入方式。
> 对应 TODO：`docs/2026-08-23_manipulation_common并入方案/README.md` §7

---

## 1. 现状分析

### 1.1 已就位（与 robotarm 完全一致，无需改动）

| 项 | 状态 |
|---|---|
| `fairino_planning_ros/src/fairino_planner_manager.cpp`（4 算法：aapf/tube/birrt/rrt*） | ✅ diff 与 robotarm 一致 |
| `fairino_planning_ros/plugins/fairino_planning_plugins.xml` 注册 `fairino_planning/FairinoPlannerManager` | ✅ |
| `fairino_planning_core/config/` 7 个 yaml（common/aapf/tube/birrt/rrt/ik/cartesian） | ✅ 全部与 robotarm 一致 |
| `s622_moveit_config/config/kinematics_fairino.yaml`（FairinoIKPlugin） | ✅ |

### 1.2 问题所在（历史遗留错误）

**`src/fairino_planning_ros/config/fairino_planning.yaml`** 内容错误：
```yaml
fairino:
  planning_plugin: ompl_interface/OMPLPlanner   # ← 错误！不是 FairinoPlannerManager
  ...
  planner_configs: {RRTConnect, RRTstar, LBTRRT, ...}  # OMPL 配置
```
- 顶层是 `planning_pipelines` 包装格式（含 `pipeline_names: [ompl, fairino]`）
- launch 注入该文件后**覆盖** builder 生成的管线声明 → 实际注册了 ompl + fairino 两个管线名，但 fairino 用 OMPL 插件
- 即：**当前仿真跑的一直是 OMPL（RRTConnect 等），FairinoPlannerManager 从未被真正加载**

### 1.3 robotarm 的正确做法（对齐目标）

1. **管线配置文件放 moveit_config 包**：`fairino3_v6_moveit2_config/config/fairino_planning.yaml`
   ```yaml
   planning_plugin: fairino_planning/FairinoPlannerManager
   request_adapters: >- ...
   response_adapters: ""
   # 顶层算法参数（legacy 兜底，实际生效参数在 planning_core 的 fairino.algorithms.*）
   max_iterations: 12000 ...
   ```
2. **builder 声明**：`.planning_pipelines(pipelines=["fairino", "ompl"], default_planning_pipeline="fairino")`
   → MoveItConfigsBuilder 自动加载 `config/fairino_planning.yaml` + `config/ompl_planning.yaml`（文件名规则 `<pipeline>_planning.yaml`）
3. **move_group 参数注入**（`gazebo_launch/launch_utils/moveit_stack.py`）：
   - `moveit_config.to_dict()`（含 planning_pipelines.fairino）
   - `kinematics_fairino` / `kinematics_kdl`
   - `controllers`、`sensors_3d`
   - `fairino_planning`（裸格式，顶层 planning_plugin）
   - `planning_core`（common_planning_params.yaml → `planner.*`、`fairino.optimizer/trajectory/pipeline/safety.*`）
   - `aapf_birrt_star_core` / `tube_birrt_star_core` / `birrt_star_core` / `rrt_star_core`（→ `fairino.algorithms.<name>.*`）
   - `ik_core`（ik_params.yaml → `fairino.ik.*`）
   - fairino 实例额外：`{"fairino": {"ik": {"task_profile": "grasp"}}}` + `{"planner": {"random_seed": N}}`
4. **RViz**：注入 `moveit_config.planning_pipelines` + `fairino_planning` 裸参数（面板显示管线列表）

### 1.4 参数读取路径（FairinoPlannerManager::initialize）

```cpp
loadPlannerConfig(node_, ns, "fairino.algorithms.aapf_birrt_star");  // ← planning_core 的 aapf_birrt__params.yaml
loadPlannerConfig(node_, ns, "fairino.algorithms.tube_birrt_star");
loadPlannerConfig(node_, ns, "fairino.algorithms.birrt_star");
loadPlannerConfig(node_, ns, "fairino.algorithms.rrt_star");
loadPipelineOptions(node_, ns);   // ← common_planning_params.yaml 的 planner.* 段
```
- `gi_pref/gd_pref` 只用 primary_prefix（`fairino.algorithms.<name>`），legacy `fairino` 被 `(void)` 忽略
- 即：**顶层 fairino_planning.yaml 算法参数不生效，真正生效的是 planning_core 7 个 yaml**

---

## 2. 改动方案

### 2.1 新建 `src/s622_moveit_config/config/fairino_planning.yaml`
直接复制 robotarm `fairino3_v6_moveit2_config/config/fairino_planning.yaml`（33 行，planning_plugin + request_adapters + 顶层算法参数）。

### 2.2 修正 `src/fairino_planning_ros/config/fairino_planning.yaml`
改为 robotarm 正确版（裸格式：顶层 `planning_plugin: fairino_planning/FairinoPlannerManager`）。
> 影响面：`s622_table.launch.py` / `s622_dual_arm.launch.py` / `dual_ik_move_group.launch.py` 也从该文件读 `pipeline_params`。
> 安全分析：这些 launch 均 `pipelines=["ompl"]`，注入裸参数后无 `planning_pipelines` 键 → 不影响 builder 声明的 ompl-only 管线；MoveIt 读的是 `planning_pipelines.fairino.planning_plugin`（builder 未注册 fairino 管线时不读）。行为不变，双臂阶段再统一。

### 2.3 修改 `src/gz_launch/launch/s622_gazebo.launch.py`

1. builder：`.planning_pipelines(pipelines=["fairino", "ompl"], default_planning_pipeline="fairino")`
2. 删除从 `fairino_planning_ros/config/fairino_planning.yaml` 读 `pipeline_params` 的逻辑
3. 加载 planning_core 参数：
   ```python
   fairino_planning = load_yaml("s622_moveit_config", "config/fairino_planning.yaml")
   planning_core = load_yaml("fairino_planning_core", "config/common_planning_params.yaml")
   aapf_core = load_yaml("fairino_planning_core", "config/aapf_birrt__params.yaml")
   tube_core = load_yaml("fairino_planning_core", "config/tube_birrt__params.yaml")
   birrt_core = load_yaml("fairino_planning_core", "config/birrt__params.yaml")
   rrt_core = load_yaml("fairino_planning_core", "config/rrt__params.yaml")
   ik_core = load_yaml("fairino_planning_core", "config/ik_params.yaml")
   ```
4. move_group_fairino 参数：
   ```python
   [moveit_config.to_dict(), fairino_planning, planning_core, aapf_core, tube_core,
    birrt_core, rrt_core, ik_core,
    {"fairino": {"ik": {"task_profile": "grasp"}}},
    {"planner": {"random_seed": 0}},
    {"robot_description_kinematics": kinematics_fairino}, {"use_sim_time": True}]
   ```
5. move_group_kdl 参数：同上但**不含** task_profile（robotarm kdl 实例不带 grasp profile，用 ik_params 默认 grasp）
6. RViz 参数：`moveit_config.planning_pipelines` + `fairino_planning`（替换原 pipeline_params）

### 2.4 其他 launch（本轮不动）
- `s622_table.launch.py` / `s622_dual_arm.launch.py` / `dual_ik_move_group.launch.py`：双臂阶段再统一（§2.2 安全分析保证不受破坏）

---

## 3. 验证计划

### V1 构建 + 静态检查
1. `colcon build --merge-install --symlink-install --packages-select s622_moveit_config gz_launch`（s622_moveit_config 的 config 为复制安装，需 rebuild；gz_launch launch 为 symlink 即时生效）
2. 确认 `install/share/s622_moveit_config/config/fairino_planning.yaml` 存在且内容正确
3. grep 确认无残留 `ompl_interface/OMPLPlanner` 在 fairino 配置中

### V2 用户仿真验证（用户启动）
1. 用户启动 `ros2 launch gz_launch s622_gazebo.launch.py`
2. 检查 move_group_fairino 日志：
   - `Fairino planner params loaded: aapf_birrt*_max_iter=...`（初始化成功）
   - 规划时 `group_name=robot_arm, selected_planner=..., tool_model=GRIPPER`（getPlanningContext）
3. RViz 面板 Planning Pipeline 下拉可见 fairino（默认）/ ompl 两条
4. `ros2 param get /move_group_fairino/move_group planning_pipelines` 确认管线列表
5. 跑一次运动（motion_demo 或 arm_actions MoveToPose），确认规划成功且轨迹执行

### V3 回归
- visual_servo 抓取闭环（如用户需要）
- 37 个 manipulation_common 测试

---

## 4. 风险与回滚

| 风险 | 等级 | 应对 |
|---|---|---|
| FairinoPlannerManager 首次真正加载，参数/行为差异 | 中 | 参数全部来自 robotarm 一致文件；日志确认 4 算法 max_iter 加载 |
| 双臂 launch 读修正后的 fairino_planning.yaml | 低 | §2.2 安全分析：注入裸参数无 planning_pipelines 键，ompl 行为不变 |
| KDL move_group 加载 fairino 管线 | 低 | robotarm 同样给 kdl 实例注入 fairino 参数（pipeline 与 IK 独立） |
| RViz 面板管线列表 | 低 | 注入 moveit_config.planning_pipelines 标准做法（demo.launch.py 同款） |

**回滚**：git 历史 + launch 一行改回 `pipelines=["ompl"]`。

---

## 5. 相关文档
- `docs/2026-08-23_manipulation_common并入方案/README.md` §7（TODO 源）
- `docs/FairinoIK插件移植方案.md:102`（插件注册记录）
- robotarm 参考：`fairino_robotarm-main/src/gazebo_launch/launch_utils/moveit_stack.py`、`fairino3_v6_moveit2_config/config/fairino_planning.yaml`、`fairino3_v6_moveit2_config/launch/demo.launch.py`

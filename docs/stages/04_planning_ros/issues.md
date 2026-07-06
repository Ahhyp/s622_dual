# 阶段 4（流程文档阶段四）：MoveIt2 插件（自写）

## 包

`fairino_planning_ros` — 将 `fairino_planning_core` 包装成 MoveIt2 可调用的插件。

## 实现内容

| 模块 | 文件 |
|------|------|
| PlannerManager 插件 | `fairino_planner_manager.h/.cpp` — 注册为 Fairino 规划器 |
| PlanningContext | `fairino_planning_context.h/.cpp` — solve/start/goal/limits/path 转换 |
| IK 插件 | `fairino_ik_plugin.h/.cpp` — `KinematicsBase` 接口 |
| 碰撞检测适配 | `moveit_collision_checker.h/.cpp` — MoveIt2 → CollisionInterface |
| 管线配置 | `config/fairino_planning.yaml` |
| 插件注册 | `plugins/fairino_planning_plugins.xml` |

## 验证

```bash
ros2 run pluginlib list_plugins moveit_core "planning_interface::PlannerManager"
ros2 run pluginlib list_plugins moveit_core "kinematics::KinematicsBase"
ros2 launch gz_launch s622_gazebo.launch.py  # Plan & Execute 成功
```

## 遇到的问题

### 问题 1：ament_target_dependencies 自依赖

**日期：** 2026-05-28

**原因：** CMakeLists.txt 中写了 `ament_target_dependencies(... fairino_planning_ros ...)`，包依赖自身。

**解决：** 删除该行。

### 问题 2：pluginlib 命令格式不对

**日期：** 2026-05-28

**原因：** ROS 2 Humble 中命令是 `ros2 run pluginlib list_plugins [package] [base_class]`，不是 `plugin_tool --plugin-package`。

### 问题 3：插件注册在 moveit_core 名下

**日期：** 2026-05-28

**原因：** `pluginlib_export_plugin_description_file(moveit_core ...)` 把 XML 注册在 `moveit_core` 的 ament 索引下。

**解决：** 查询时第一个参数用 `moveit_core`，不是 `fairino_planning_ros`。

### 问题 4：fairino_ik_plugin.cpp 缺少 PLUGINLIB_EXPORT_CLASS

**日期：** 2026-05-28

**原因：** cpp 文件末尾忘了加导出宏。

**解决：**
```cpp
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fairino_planning_ros::FairinoIKPlugin, kinematics::KinematicsBase)
```

### 问题 5：planning_plugin 参数未生效（管线退化为 CHOMP）

**日期：** 2026-05-28

**原因：** fairino_planning.yaml 中 `fairino:` 段放在 `planning_pipelines:` 内部，MoveIt2 期望它和 `planning_pipelines` 平级（作为顶层命名空间参数）。

**解决：** 把 `fairino:` 移到 yaml 顶层。

### 问题 6：Gazebo 报 no ros2_control tag

**日期：** 2026-05-28

**原因：** 只 `--packages-select fairino_planning_core fairino_planning_ros` 编译，其他包的安装不一致。

**解决：** `pkill -9 -f gz`，然后全量 `colcon build --merge-install --symlink-install`。

### 问题 7：FairinoPlanningContext segfault — jmg_ 空指针

**日期：** 2026-05-28

**原因：** 构造函数中 `jmg_` 只声明了 `= nullptr`，从未赋值。`copyJointGroupPositions(jmg_, ...)` 解引用空指针。

**解决：** 构造函数初始化列表补：
```cpp
jmg_(robot_model->getJointModelGroup(group))
```

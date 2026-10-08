# 问题记录

---

## [阶段二] fairino3_v6 在 Gazebo 中 Plan 通过但 Execute 无反应

**日期：** 2026-05-26

**现象：**
RViz 中 Plan 正常，点 Execute 后机械臂不动。Gazebo 日志无报错，控制器正常加载。

**原因：**
`s622_gazebo.launch.py` 中 controller 名写成了 `robot_arm_controller`，但 `fairino3_v6_moveit_config/ros2_controllers.yaml` 里定义的是 `fairino3_controller`。名字不匹配导致 controller_manager 找不到对应控制器。

**解决：**
spawner 参数改为 `fairino3_controller`，与 ros2_controllers.yaml 一致。

---

## [阶段二] fairino3 controller 激活后机械臂依然不动

**日期：** 2026-05-26

**现象：**
控制器成功激活（Configured and activated），轨迹发送成功（Goal request accepted），轨迹执行成功（Goal reached, success），但 Gazebo 和 RViz 中机械臂关节值始终接近 0。

**原因：**
`fairino3_v6_moveit_config/ros2_controllers.yaml` 中 6 个 joint 的 `ff_velocity_scale` 全部为 `0.0`。速度前馈系数为 0 意味着轨迹规划出的目标速度被归零，控制器只靠 PID 微小纠偏，GazeboSimSystem 几乎收不到速度指令。

**解决：**
所有 `ff_velocity_scale: 0.0` 改为 `1.0`。注意 s622 的配置中此值本身就是 `1.0`，这是 fairino3 配置特有的坑。

---

## [阶段二] s622 Gazebo xacro 宏名不匹配

**日期：** 2026-05-27

**现象：**
```
XacroException: unknown macro name: xacro:s622_moveit_descriptions_ros2_control
```

**原因：**
`robot_gazebo.friction.urdf.xacro` 中 include 了 `robot_gazebo.friction.xacro`（宏名 `fairino3_v6_robot_ros2_control`），但调用时用了 `s622_moveit_descriptions_ros2_control`。两个 friction 文件是 fairino 的，宏名残留 fairino 前缀。

**解决：**
放弃复用 fairino 的 friction 文件。为 s622 单独写 `s622_gz_moveit_descriptions.ros2_control.xacro`（GazeboSimSystem + velocity），`robot_gazebo.urdf.xacro` 直接 include 该文件。launch 中也改用 `robot_gazebo.urdf.xacro`。

---

## [阶段二] s622_gz_moveit_descriptions.ros2_control.xacro 未安装

**日期：** 2026-05-27

**现象：**
```
No such file or directory: .../install/share/s622_moveit_config/config/s622_gz_moveit_descriptions.ros2_control.xacro
```

**原因：**
新文件放在 `s622_moveit_config/config/` 下，但 `--packages-select gz_launch` 不会重装 `s622_moveit_config`。`--symlink-install` 只对已安装过的包创建 symlink，新文件不会自动出现在 install 目录。

**解决：**
`colcon build --packages-select s622_moveit_config gz_launch` 同时重建两个包。

---

## [阶段二] 想跑 s622 却跑了 fairino3 的 launch

**日期：** 2026-05-27

**现象：**
多次修改后混淆，跑了 `gazebo.launch.py`（fairino3 旧版），报宏名不匹配。

**原因：**
两个 launch 文件并存：`gazebo.launch.py`（fairino3）、`s622_gazebo.launch.py`（s622）。

**解决：**
始终用 `ros2 launch gz_launch s622_gazebo.launch.py`。

## [阶段一] MoveIt2 demo.launch.py 找不到 URDF/SRDF

**日期：** 2026-05-26

**现象：**
```
WARNING: Cannot infer URDF from s622_moveit_config -- using config/s622_moveit_descriptions.urdf
ERROR: File s622_moveit_descriptions.urdf doesn't exist
```

**原因：**
1. `.setup_assistant` 指向 `s622_moveit_descriptions` 包的 `urdf/s622_moveit_descriptions.urdf`
2. MoveItConfigsBuilder 读取该配置，但 s622_moveit_descriptions 包中的 URDF 是纯 URDF 格式
3. `s622_moveit_config/config/` 下只有 `.urdf.xacro` 和 `.srdf`，缺 `.urdf`
4. 且 config 目录的文件没有通过 CMakeLists.txt 的 `install(DIRECTORY config ...)` 安装

**解决：**
1. CMakeLists.txt 补上 `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`
2. 从原项目完整复制 `config/` 和 `launch/` 目录

---

## [阶段一] robot_arm_controller 无法激活（velocity command interface 不匹配）

**日期：** 2026-05-26

**现象：**
```
ERROR: Not acceptable command interfaces combination:
Start interfaces: [j1/velocity, j2/velocity, ...]
Not existing: [j1/velocity, j2/velocity, ...]
Failed to activate controller: robot_arm_controller
```

**原因：**
原项目 `ros2_controllers.yaml` 中 `robot_arm_controller` 使用 `velocity` command interface，但 FakeSystem 硬件只导出 `position` interface。这是原项目本身的配置问题。

**影响：**
不影响 Plan（规划），只影响 Execute（执行）。Gazebo 阶段会解决。

---

## [阶段一] COLCON_PREFIX_PATH 残留警告

**现象：**
```
WARNING: The path '/home/yep/my_S622/robot_ws/install' in COLCON_PREFIX_PATH doesn't exist
```

**原因：**
旧工作空间 `robot_ws` 的路径残留在环境变量中。

**解决：**
`unset COLCON_PREFIX_PATH` 或重开终端。

---

## [阶段一] catkin_pkg 缺失 + conda 环境激活方式

**日期：** 2026-05-25

**现象：**
`colcon build` 报 `ModuleNotFoundError: No module named 'catkin_pkg'`

**原因：**
非交互式 shell 中 `conda activate yolov8` 无效，CMake 找到的是 base conda 的 Python，缺少 catkin_pkg。

**解决：**
```bash
eval "$(conda shell.bash hook)" && conda activate yolov8
```

## [阶段一/s622_moveit_descriptions] xacro 命名空间缺失导致 XML 解析失败

**日期：** 2026-05-25

**现象：**
```
XML parsing error: unbound prefix: line 1, column 0
when processing file: .../s622_moveit_descriptions.urdf
```

**原因：**
URDF 中添加了 `<xacro:include>` 和 `<xacro:s622_camera>`，但根 `<robot>` 标签缺少 `xmlns:xacro="http://www.ros.org/wiki/xacro"` 声明。

**解决：**
在 `<robot>` 标签加上 `xmlns:xacro="http://www.ros.org/wiki/xacro"`。

---

## [阶段一/s622_moveit_descriptions] meshes/ 为空导致 RViz 无模型

**日期：** 2026-05-25

**现象：**
RViz 中 RobotModel 无显示，TF 树正常。

**原因：**
从原项目复制 URDF 时只复制了 `.urdf` 文件，`meshes/` 和 `visual/` 目录遗漏，STL 路径全部无效。

**解决：**
```bash
cp -r /home/yep/S622_robotarm/src/s622_moveit_descriptions/meshes src/s622_moveit_descriptions/
cp -r /home/yep/S622_robotarm/src/s622_moveit_descriptions/visual src/s622_moveit_descriptions/
```

---

## [阶段一/s622_moveit_descriptions] RViz 启动时未加载配置文件

**日期：** 2026-05-25

**现象：**
RViz 窗口打开但完全是空白界面，无 RobotModel 显示项。

**原因：**
`display.launch.py` 启动 `rviz2` 时没传 `-d` 参数指定 rviz config 文件。

**解决：**
在 launch 中添加 rviz config 路径：
```python
rviz_config = PathJoinSubstitution([
    FindPackageShare("s622_moveit_descriptions"),
    "rviz",
    "urdf.rviz"
])
# Node 中加 arguments=["-d", rviz_config]
```

---

## [阶段一/s622_moveit_descriptions] urdf.rviz 文件为空

**日期：** 2026-05-25

**现象：**
RViz 加载配置后仍无 RobotModel。

**原因：**
`rviz/urdf.rviz` 是空文件，没有包含 RobotModel 显示配置。

**解决：**
从原项目 `s622_moveit_descriptions/rviz/urdf.rviz` 复制完整配置文件。

---

## [阶段三] rrt_tree.h 残留 JointConfig 类型名

**日期：** 2026-05-28

**现象：**
编译报 `JointConfig does not name a type`，rrt_tree.h 中 4 处方法签名使用了 `JointConfig`。

**原因：**
从原项目复制 rrt_tree.h 时只改了 `TreeNode::state` 为 `JointArray`，遗漏了方法签名的类型名。

**解决：**
头文件 4 处 `JointConfig` → `JointArray`：`addNode`、`nearest`、`nearRadius`、`backtrack` 的参数和返回类型。

---

## [阶段三] rrt_tree.cpp 残留 Eigen 向量语法

**日期：** 2026-05-28

**现象：**
编译报 `no match for operator-`，`std::array` 不支持 `.squaredNorm()` 和 `.norm()`。

**原因：**
原项目 `JointConfig = Eigen::Matrix<double,6,1>` 有向量减法运算符和 norm 方法，用户改为 `JointArray = std::array<double,6>` 后这些语法不可用。

**解决：**
3 处改为手动循环：
- `nearest()` 和 `nearRadius()` 的 fallback 线性搜索
- `propagateCost()` 的代价传播

---

## [阶段三] DHKinematics 无默认构造函数

**日期：** 2026-05-28

**现象：**
`FairinoIK` 构造函数中 `fk_` 成员被默认构造，但 `DHKinematics` 只有 `DHKinematics(const std::array<DHParam, DOF>&)` 单参构造函数。

**原因：**
类成员在初始化列表未显式初始化时，C++ 会尝试默认构造。DHKinematics 无默认构造函数。

**解决：**
用 lambda 在初始化列表中构建 DHParam 数组并传给 fk_：
```cpp
FairinoIK::FairinoIK(...)
    : d_(d), a_(a), alpha_(alpha), fk_([](const auto& d, const auto& a, const auto& al) {
        std::array<DHParam, DOF> dh;
        for (int i = 0; i < DOF; ++i) dh[i] = {a[i], al[i], d[i], 0.0};
        return dh;
    }(d, a, alpha))
```

---

## [阶段三] JointLimits 默认值为零导致 IKSelector 全部过滤

**日期：** 2026-05-28

**现象：**
测试 4 (IK 选择器) 返回 nullopt，所有 IK 解被 `isWithin()` 过滤。

**原因：**
`JointLimits` 结构体没有成员默认值，`IKSelector` 默认构造的 `limits_` 中 lower/upper 全为零，任何非零关节角都不在范围内。

**解决：**
`types.h` 中 `JointLimits` 成员加上 S622 的默认限位值（与原项目 `JointLimits()` 构造函数硬编码值一致）。

---

## [阶段三] test_core 未安装到 install 目录

**日期：** 2026-05-28

**现象：**
`colcon build` 成功但 `./install/lib/fairino_planning_core/test_core` 不存在。

**原因：**
CMakeLists.txt 有 `add_executable(test_core ...)` 但缺少对应的 `install(TARGETS test_core ...)`。CMake 只安装显式声明 install 的目标。

**解决：**
CMakeLists.txt 补 `install(TARGETS test_core DESTINATION lib/${PROJECT_NAME})`。

---

## [阶段三] ik_selectior.cpp 文件名拼写错误

**日期：** 2026-05-28

**现象：**
CMakeLists.txt 引用 `src/ik/ik_selector.cpp`，实际文件名为 `ik_selectior.cpp`。

**解决：**
重命名文件为 `ik_selector.cpp`。

---

## [阶段四] ament_target_dependencies 自依赖

**日期：** 2026-05-28

**现象：**
CMakeLists.txt 中 `ament_target_dependencies(${PROJECT_NAME} ... fairino_planning_ros ...)`，包依赖自身。

**解决：**
删除 `fairino_planning_ros` 行。

---

## [阶段四] pluginlib list_plugins 命令格式不对

**日期：** 2026-05-28

**现象：**
`ros2 run pluginlib plugin_tool list_plugins` 报 No executable found。

**原因：**
ROS 2 Humble 中命令是 `ros2 run pluginlib list_plugins`，参数是 positional 的 `[package] [base_class]`，不是 `--plugin-package` 和 `--base-class`。

**解决：**
```bash
ros2 run pluginlib list_plugins moveit_core "planning_interface::PlannerManager"
```

---

## [阶段四] 插件注册在 moveit_core 名下

**日期：** 2026-05-28

**现象：**
`ros2 run pluginlib list_plugins fairino_planning_ros ...` 无输出。

**原因：**
`pluginlib_export_plugin_description_file(moveit_core ...)` 把插件 XML 注册在 `moveit_core` 的 ament 索引下，查 `fairino_planning_ros` 自然查不到。

**解决：**
查询时第一个参数用 `moveit_core`，不是 `fairino_planning_ros`。

---

## [阶段四] fairino_ik_plugin.cpp 缺少 PLUGINLIB_EXPORT_CLASS

**日期：** 2026-05-28

**现象：**
`pluginlib list_plugins` 只显示 FairinoPlannerManager，不显示 FairinoIKPlugin。

**原因：**
cpp 文件末尾忘了加：
```cpp
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fairino_planning_ros::FairinoIKPlugin, kinematics::KinematicsBase)
```

**解决：**
补上导出宏。

---

## [阶段四] planning_plugin 参数未生效（管线退化为 CHOMP）

**日期：** 2026-05-28

**现象：**
```
Loading planning pipeline 'fairino'
Multiple planning plugins available. You should specify the '~planning_plugin' parameter.
Using 'chomp_interface/CHOMPPlanner' for now.
```

**原因：**
fairino_planning.yaml 中 `fairino:` 段放在 `planning_pipelines:` 内部，MoveIt2 期望它和 `planning_pipelines` 平级（作为顶层命名空间参数）。

**解决：**
```yaml
planning_pipelines:
  pipeline_names: ["ompl", "fairino"]
  default_planning_pipeline: fairino

fairino:                          # ← 移到顶层
  planning_plugin: fairino_planning_ros/FairinoPlannerManager
  ...
```

---

## [阶段四] Gazebo 报 no ros2_control tag

**日期：** 2026-05-28

**现象：**
```
[ign gazebo-1] [ERROR] gz_ros2_control: Error parsing URDF, plugin not active : no ros2_control tag
spawner: Could not contact service /controller_manager/list_controllers
```

**原因：**
阶段四只 `--packages-select fairino_planning_core fairino_planning_ros`，其他包未重编。Gazebo 相关包的安装状态不一致。

**解决：**
`pkill -9 -f gz` 清残留，然后全量 `colcon build --merge-install --symlink-install`。

---

## [阶段六] pixel→3D→base_link 坐标转换存在系统性 y 向偏差

**日期：** 2026-06-15

**现象：**
通过 YOLO OBB 检测 → 深度图中值 → camera_optical_frame 3D 坐标 → TF 到 base_link，得到的物体 3D 坐标在 **y 方向存在系统性正向偏移**，且偏移量随目标位置变化：

| spawn_y | Δy (mm) | pixel v |
|---------|---------|---------|
| 0.0     | +136    | 32      |
| 0.1     | +110    | 71      |
| 0.2     | +84     | 112     |
| 0.3     | +58     | 154     |

- x 方向极准（≤16mm），z 方向对地面目标极准（≤1cm）
- Δy 随 spawn_y 增大而减小（越靠图像边缘偏得越大 → **B 类误差：内参/深度对齐**）
- 投影链路 debug 节点证实 TF 链正确，问题在 pixel→3D 的 y 分量受 79° 相机倾斜 + fx≠fy 不对称放大

**原因：**
相机安装在 base_link 上方 1m、倾斜 79° 俯视。物体 y=0 时在图像中 v≈32（靠近顶部边缘），此时像素误差被 fx≠fy（465.6 vs 625.2）和 79° 光轴倾斜放大，导致 camera_y 计算偏差，经 TF 变换后表现为 base_y 的 +13.6cm 误差。

这不是代码 bug，是固定外置相机 + 倾斜视角 + 内参不对称导致的**几何系统性误差**。纯 3D 伺服（pixel→3D→追坐标）无法规避此误差。

**影响：**
旧版 3D 伺服架构（直接 pixel→3D→base_link→servo 追）会把这个误差直接变成 EE 落点误差，导致抓取偏移。

**架构级解决：**
重新设计视觉伺服系统，改为 **2D 图像空间伺服 + 盲 Cartesian 下降**：
- pixel→3D 仅用于 MoveIt 粗规划（容忍大误差）
- 精确对齐在图像空间闭环：`error_uv = target_uv - projected_ee_uv`
- 对齐后盲降（纯运动学沿 z 下降，不依赖视觉 3D 坐标）
- 详见 `visual_servo_node.py` 新版状态机：COARSE_PLANNING → VISUAL_ALIGN_XY → VISUAL_ALIGN_YAW → DESCEND_WITH_FEEDBACK

**教训：**
固定外置倾斜相机 + 纯 3D 伺服在仿真中不可靠。源项目用 2D 图像伺服避开了这个问题——这不是"源项目算得更准"，而是"源项目根本不需要算得准"。

## [阶段四] FairinoPlanningContext::extractStartState segfault

**日期：** 2026-05-28

**现象：**
```
#2  FairinoPlanningContext::extractStartState()
#0  JointModelGroup::getVariableCount()  ← segfault at 0x390
```

**原因：**
构造函数中 `jmg_` 成员只声明了 `= nullptr`，从未赋值。`copyJointGroupPositions(jmg_, values)` 解引用空指针。

**解决：**

---

## 问题：Gazebo Fortress prismatic joint limit=0 导致夹爪单指不动作

**日期：** 2026-06-28

**现象：**
`finger1_joint` 命令 `0.025` 不执行，`finger2_joint` 命令 `-0.025` 正常。

**原因：**
Gazebo Fortress 解析 SDF 时忽略值为 0 的 joint limit，`lower=0` 被当作"无下限"，prismatic joint 无法从 0 启动正方向运动。

**解决：**
`lower="0.0"` → `lower="-1e-6"`, `upper="0"` → `upper="1e-6"`（1 微米偏差，无物理影响）。
文件：`src/s622_moveit_descriptions/urdf/s622_moveit_descriptions.urdf`
构造函数初始化列表补 `jmg_(robot_model->getJointModelGroup(group))`。

---

# TODO（2026-08-26 双臂现代化收尾后遗留）

> 来源：双臂现代化 S1-S5 + 回归验证完成（BT pick_place_dual SUCCESS）后的遗留项。
> 相关记录：docs/2026-08-25_place位姿标定/、docs/2026-08-25_双臂现代化改造/

## [双臂] 伺服 descend/lift 提速

**状态：** 待办

**背景：**
BT 抓取全流程 SUCCESS，但伺服段（visual_align_server 的 descend/lift）实测
**23-26mm/s**（目标 40-50mm/s），descend 13cm 花 32.9s、lift 10cm 花 24.4s，
整条 BT 约一半时间耗在伺服段。

**方向：**
- visual_align_server 的 descend/lift 循环里伺服实际速度只有目标 60-70%，
  可能与 servo 的 butterworth 平滑或 controller 限速有关
- 可尝试提高 `descend_speed`/`lift_speed`（BT XML）或检查 servo 平滑参数

## [双臂] fairino planner IK 多解碰撞重试

**状态：** 待办

**背景：**
fairino 管线 `setFromIK` 只取一个候选解（连续性优先），选中解碰撞时直接
BiRRT* fast-fail，不做多解碰撞重试。place 位姿曾因此失败（选中的解与桌面/场景碰撞）。

**方向：**
- setFromIK 后对选中解做碰撞检查，碰撞则尝试其他 raw candidates
- 对齐 FairinoIKPlugin 的 selector_candidates 机制（当前只选一个）

## [双臂] right 臂对称验证

**状态：** 待办

**背景：**
place 位姿已对称标定（left→world (0.10,0.20) / right→world (-0.10,0.20)），
但 BT 只跑过 `arm=left`。

**方向：**
- bt_executor 参数 `arm='right'` 跑一遍 pick_place_dual 验证对称性
- 确认 right 的 IK（root→base 变换）与 place 位姿均正常

## [双臂] 手递手交接（阶段 5）

**状态：** 待办

**背景：**
place 位置已放到"另一只机械臂能舒适夹取的位置"（用户拍板），
为递物交接做准备。`pick_handover_place.xml` 已存在但未验证。

**方向：**
- 验证 pick_handover_place.xml（left 抓 → right 接）
- 依赖 TransferObjectNode（scene_nodes.cpp 已实现）

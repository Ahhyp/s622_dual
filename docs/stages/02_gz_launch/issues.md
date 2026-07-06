# 阶段 2：遇到的问题与解决方案

## 问题 1：controller 名不匹配导致 Execute 无反应

**日期：** 2026-05-26

**现象：**
RViz 中 Plan 正常，点 Execute 后机械臂不动。

**原因：**
`s622_gazebo.launch.py` 中 controller 名写成了 `robot_arm_controller`，但 `fairino3_v6_moveit_config/ros2_controllers.yaml` 里定义的是 `fairino3_controller`。

**解决：**
spawner 参数改为 `fairino3_controller`，与 ros2_controllers.yaml 一致。

## 问题 2：ff_velocity_scale=0 导致机械臂不动

**日期：** 2026-05-26

**现象：**
控制器激活成功，轨迹发送/执行成功，但 Gazebo 和 RViz 中关节值始终接近 0。

**原因：**
`fairino3_v6_moveit_config/ros2_controllers.yaml` 中 6 个 joint 的 `ff_velocity_scale` 全部为 `0.0`。速度前馈系数为 0 意味着轨迹速度被归零。

**解决：**
所有 `ff_velocity_scale: 0.0` 改为 `1.0`。s622 的配置此值本身就是 `1.0`，这是 fairino3 配置特有的坑。

## 问题 3：xacro 宏名不匹配

**日期：** 2026-05-27

**现象：**
```
XacroException: unknown macro name: xacro:s622_moveit_descriptions_ros2_control
```

**原因：**
复用 fairino 的 friction 文件，宏名残留 fairino 前缀。

**解决：**
为 s622 单独写 `s622_gz_moveit_descriptions.ros2_control.xacro`，`robot_gazebo.urdf.xacro` 直接 include。

## 问题 4：新 xacro 文件未安装

**日期：** 2026-05-27

**现象：**
No such file: `.../install/share/s622_moveit_config/config/s622_gz_moveit_descriptions.ros2_control.xacro`

**原因：**
`--packages-select gz_launch` 不会重装 `s622_moveit_config`。

**解决：**
`colcon build --packages-select s622_moveit_config gz_launch` 同时重建。

## 问题 5：跑错了 launch 文件

**日期：** 2026-05-27

**现象：**
多次修改后混淆，跑了 `gazebo.launch.py`（fairino3 旧版）。

**解决：**
始终用 `ros2 launch gz_launch s622_gazebo.launch.py`。

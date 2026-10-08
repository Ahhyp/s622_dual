# 阶段 2：Gazebo 仿真（自写）

## 目标

手写 launch 文件和配置文件，在 Gazebo 中启动 S622 机械臂仿真，实现 Plan & Execute。

**自写包：`gz_launch`**

## 关键文件

| 文件 | 说明 |
|------|------|
| `launch/s622_gazebo.launch.py` | 主 launch：Gazebo + ros2_control + MoveIt2 + RViz |
| `config/robot_gazebo.urdf.xacro` | S622 的 URDF，含 gazebo 插件和 ros2_control 标签 |
| `rviz/gz_launch.rviz` | RViz 配置文件 |

## 依赖

- `s622_moveit_config/config/s622_gz_moveit_descriptions.ros2_control.xacro`（自写）
- `s622_moveit_config/config/ros2_controllers.yaml`
- `s622_moveit_config/config/initial_positions.yaml`

## 验收

- `ros2 launch gz_launch s622_gazebo.launch.py` 启动成功
- RViz 中 Plan & Execute 机械臂能在 Gazebo 中运动

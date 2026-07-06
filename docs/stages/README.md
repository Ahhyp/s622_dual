# 阶段索引

按[代码级项目搭建流程](../../代码级项目搭建流程.md)的阶段划分（v2，8 阶段）。

| 目录 | 流程文档节 | 阶段 | 名称 | 方式 | 状态 |
|------|-----------|------|------|------|------|
| [01_robot_model](01_robot_model/) | §4 | 阶段一 | 厂商层 | 复制 | ✅ |
| [02_gz_launch](02_gz_launch/) | §5 | 阶段二 | Gazebo 仿真 | **自写** | ✅ |
| [03_planning_core](03_planning_core/) | §6 | 阶段三 | 核心运动学与规划 | **自写** | ✅ |
| [04_planning_ros](04_planning_ros/) | §7 | 阶段四 | MoveIt2 插件 | **自写** | ✅ |
| [05_perception](05_perception/) | §8 | 阶段五 | 感知与标定 | 部分自写 | ✅ |
| [06_visual_servo](06_visual_servo/) | §9 | 阶段六 | visual_servo | **自写** | 🚧 骨架通，奇异点待根治 |
| [07_real_yolo](07_real_yolo/) | — | 接真 YOLO | 假检测→真实 OBB 推理 | **自写** | ← 进行中 |
| — | §10 | 阶段七 | OctoMap + GraspNet | **自写** | 待做 |
| — | §11 | 阶段八 | 工具层 | 部分自写 | 待做 |

每个阶段目录含 `plan.md`（目标概要）和 `issues.md`（问题记录）。

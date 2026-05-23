# CLAUDE.md

## 项目概述

本项目是 S622 机械臂智能抓取系统的学习复现项目，按照[代码级项目搭建流程](代码级项目搭建流程.md)从零搭建。

**原则：每一步都建包、写代码、编译、运行、验收、记录。**

## 参考文档

| 文件 | 说明 |
|------|------|
| [代码级项目搭建流程](代码级项目搭建流程.md) | **主文档**，13 阶段搭建流程，每步有代码模板和验收标准 |
| [原项目结构](原项目结构.md) | 原项目 26 个包完整目录树 |
| [学习](学习.md) | 原项目运行指南、源码阅读路径 |
| [项目搭建流程](项目搭建流程.md) | 原项目原始搭建文档 |

## 环境

- **ROS 2 Humble**（`/opt/ros/humble/`）
- **Python 3.10**，使用 conda `yolov8` 环境
- 源码位于 `src/`，按分层目录组织

## 构建

```bash
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8

rm -rf build install
colcon build --merge-install --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=$(which python3)"

source install/setup.bash
```

## 目录结构

```
src/
├── vendor/                ← 厂商/外部，直接复制
├── robot_description/     ← S622 专用模型
├── moveit_config/         ← MoveIt2 配置
├── planning/              ← 自研规划核心+插件
├── simulation/            ← Gazebo 仿真
├── perception/            ← YOLO 检测
├── calibration/           ← 手眼标定
├── grasping/              ← 抓取模块
└── tools/                 ← 工具包
```

## 进度跟踪

| 阶段 | 内容 | 自写/复制 | 状态 |
|------|------|-----------|------|
| 第一阶段 | fairino_description + fairino_msgs + fairino_hardware | 复制 | ← 下一步 |
| 第二阶段 | s622_moveit_descriptions | 复制 | 待做 |
| 第三阶段 | MoveIt2 配置 | 复制 | 待做 |
| 第四阶段 | fairino_planning_core | **自写** | 待做 |
| 第五阶段 | fairino_planning_ros | **自写** | 待做 |
| 第六阶段 | gz_launch | **自写** | 待做 |
| 第七阶段 | 感知与标定 | 部分自写 | 待做 |
| 第八阶段 | visual_servo | **自写** | 待做 |
| 第九~十二阶段 | OctoMap/GraspNet/工具/外部包 | 部分自写 | 待做 |
| 第十三阶段 | GraphExecuter | 复制 | 待做 |

分批编译顺序：见代码级项目搭建流程 S16。

## 可用 Skill

| 命令 | 说明 |
|------|------|
| `/record-issue <标题>` | 追加问题记录 |

详细计划和代码模板都在[代码级项目搭建流程](代码级项目搭建流程.md)中。

## 经验教训

1. 构建前先 `eval "$(conda shell.bash hook)" && conda activate yolov8`，非交互式 shell 不能直接用 `conda activate`
2. 用 `-DPython3_EXECUTABLE=$(which python3)` 确保 CMake 找到 conda 环境的 Python
3. 首次编译前必须 `rm -rf build install` 清理缓存
4. 不擅自修改原项目 CMakeLists.txt，构建失败优先检查系统依赖

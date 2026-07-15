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
| [IK 架构说明](docs/IK架构说明.md) | 两套 IK 系统的关系、调用链、死代码说明 |

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
src/                          ← 所有包平铺，不再用子目录分类
├── fairino_description/
├── fairino_msgs/
├── fairino_hardware/
├── s622_moveit_descriptions/
├── fairino3_v6_moveit2_config/
├── s622_moveit_config/
├── gz_launch/
├── fairino_planning_core/
├── fairino_planning_ros/
├── visual_servo/              # 视觉伺服抓取
├── s622_task_interfaces/      # 双臂项目：Action/Service 接口（待建）
├── s622_servo_actions/        # 双臂项目：伺服 Action Server（待建）
├── s622_bt_manager/           # 双臂项目：BT.CPP 任务管理器（待建）
└── yolov8_obb/                # YOLOv8 OBB 检测
```

## 进度跟踪

| 阶段 | 内容 | 自写/复制 | 状态 |
|------|------|-----------|------|
| 阶段一 | 6 个厂商包（description/msgs/hardware/moveit_descriptions/moveit_config） | 复制 | ✅ 已完成 |
| 阶段二 | gz_launch | **自写** | ✅ 已完成 |
| 阶段三 | fairino_planning_core | **自写** | ✅ 已完成 |
| 阶段四 | fairino_planning_ros | **自写** | ✅ 已完成 |
| 阶段五 | 感知与标定 | 部分自写 | ✅ 已完成 |
| 阶段六 | visual_servo | **自写** | ✅ 已完成 |
| 阶段七 | OctoMap + GraspNet | **自写** | 搁置 |
| 阶段八 | 工具层 | 部分自写 | 搁置 |
| **阶段九** | **双臂协同** | **自写** | ← 当前 |

### 双臂协同子阶段

> 总计划：[docs/双臂协同/双臂协同总体计划.md](docs/双臂协同/双臂协同总体计划.md)
> 阶段记录：[dual_arm_stages/](dual_arm_stages/)

| 子阶段 | 内容 | 状态 |
|--------|------|------|
| 阶段 1 | BT.CPP 单臂 Pick-Place 闭环重构 | ← 当前 |
| 阶段 2 | 双臂坐标系统一 | 待开始 |
| 阶段 3 | 双臂独立控制 | 待开始 |
| 阶段 4 | 双臂同步运动 | 待开始 |
| 阶段 5 | 双臂递物交接 | 待开始 |
| 阶段 6 | 双臂协同搬运 | 待开始 |
| 阶段 7 | 实验统计与简历包装 | 待开始 |

## 新对话快速上手

新对话开始时让 AI 读 [docs/AI_CONTEXT.md](docs/AI_CONTEXT.md) 即可获得完整项目上下文。

## 快速启动（双臂协同需要仿真正在运行）

```bash
# 终端 1：仿真
ros2 launch gz_launch s622_gazebo.launch.py

# 终端 2：YOLO（必须在 conda yolov8 环境）
source install/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
ros2 run yolov8_obb yolov8_obb_node --ros-args \
  -p model_path:="$HOME/my_S622/src/yolov8_obb/models/yolo-obb-gazebo.pt" \
  -p image_topic:="/camera/color/image_raw" \
  -p detections_topic:="/yolov8/obb_detections" \
  -p confidence_threshold:=0.05 -p imgsz:=1024 -p publish_empty:=true

# 终端 3：visual_servo
source install/setup.bash
ros2 run visual_servo visual_servo_node --ros-args \
  -p enable_motion:=true -p z_strategy:="table" \
  -p table_z:=0.0 -p grasp_height_above_table:=0.030 \
  -p pregrasp_height_above_table:=0.16 -p lock_check_pixel_tolerance:=12.0 \
  -p descend_speed:=0.04 -p descend_timeout:=30.0 \
  -p lifting_timeout:=30.0 -p lift_height:=0.10 \
  -p visual_align_timeout:=25.0 -p pregrasp_camera_offset:=0.03

# 触发抓取
ros2 topic pub /servo_trigger std_msgs/msg/Bool "data: true" --once
```

## 可用 Skill

| 命令 | 说明 |
|------|------|
| `/record-issue <标题>` | 追加问题记录到 docs/issues.md |

## 经验教训

1. 构建前先 `eval "$(conda shell.bash hook)" && conda activate yolov8`，非交互式 shell 不能直接用 `conda activate`
2. 用 `-DPython3_EXECUTABLE=$(which python3)` 确保 CMake 找到 conda 环境的 Python
3. 首次编译前必须 `rm -rf build install` 清理缓存
4. 不擅自修改原项目 CMakeLists.txt，构建失败优先检查系统依赖
5. 厂商包直接整批复制，不要逐包搭建（配置依赖互相耦合）
6. 复制 s622_moveit_descriptions 时删除 camera 引用，等仿真阶段再引入
7. 🚨 **相机内参 bug**：gz-sim Fortress 的 `rgbd_camera` 传感器会导致 `<horizontal_fov>` 和 `<lens><intrinsics>` 不一致（渲染用 hfov，camera_info 读 lens）。解决方案：分离为独立 `camera` + `depth_camera` 传感器，显式提供 `<lens>`，hfov 使用反推的实际渲染值 82.4°（而非 D435 物理 69°）。详记录于 [gazebo_rgbd_intrinsics_bug_resolve.md](docs/dual_arm_stages/stage_1/gazebo_rgbd_intrinsics_bug_resolve.md)。恢复指南在文档第七章。涉及文件：
   - `src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro`（新建）
   - `src/s622_moveit_descriptions/urdf/camera/camera.xacro`（修改，hfov=82.4°, vfov=52.5°）
   - **真机部署时恢复使用 realsense-ros 驱动即可，不需回滚**
8. **IK 架构**：项目有两套 IK。主路径 `plan_to_pose_smart()` 通过 `/fairino/get_all_ik` service 调 DH 解析法拿全部解并评分，MoveIt 只做碰撞检测+轨迹插值。`kinematics.yaml` 配的 KDL 仅作 fallback。`_compute_ik_kdl()` 是死代码。`fairino_planning_ros/FairinoIKPlugin` 存在但未启用。详见 [IK 架构说明](docs/IK架构说明.md)。

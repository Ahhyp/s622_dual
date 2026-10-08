# 包来源分类

## 一、厂商（法奥/Fairino）提供

| 包名 | 类型 | 说明 |
|------|------|------|
| `fairino_description` | URDF 模型 | Fairino 全系列（3/5/10/16/20/30 v6）的 URDF + STL 网格，厂商原生交付 |
| `fairino_hardware` | ros2_control 插件 | 封装 `libfairino.so` SDK，实现 `SystemInterface`。**同学修补了慧灵夹爪 finger1/finger2 支持、GripperState 状态机** |
| `fairino_msgs` | 消息定义 | 机器人通信协议：`RobotNonrtState`、`RemoteCmdInterface`、`RemoteScriptContent` |
| `fairino3_v6_moveit2_config` | MoveIt2 配置 | Fairino3 v6 的 SRDF、碰撞矩阵、控制器参数、kinematics.yaml |
| `s622_moveit_config` | MoveIt2 配置 | S622 主力配置包（规划组、末端执行器、虚拟关节） |
| `s622_moveit_descriptions` | URDF 模型 | S622 专用 URDF/SRDF（SolidWorks 导出），含更详细惯性/碰撞参数 |

**关键认知：** 厂商代码并非拿来即用。同学在 `fairino_hardware` 上做了：
- 添加慧灵夹爪的 `finger1`/`finger2` 两个关节的读写逻辑
- 添加 `GripperState` 状态机（OPEN/CLOSE/UNKNOWN）
- DO0 电磁阀控制 → 气缸，带滞回逻辑
- 修正厂商代码中的 bug（如 `path_shortcut.h` 的 Python 三引号语法错误）

---

## 二、同学自研

| 包名 | 类型 | 说明 |
|------|------|------|
| `fairino_planning_core` | C++ 算法库 | 纯 C++17 实现，**无 ROS 依赖**。DH 运动学、解析 IK、RRT*、BiRRT*、路径后处理。依赖 Eigen3 + nanoflann |
| `fairino_planning_ros` | MoveIt2 插件 | 将 planning_core 封装为 MoveIt2 PlannerManager + IK 插件。含 `FairinoPlanningContext::solve()` 和 `FairinoIKPlugin` |
| `gz_launch` | 仿真环境 | Gazebo 仿真 launch 文件、场景 SDF、物体模型、演示脚本 `demo_pathplanning_node.py` |
| `visual_servo` | 视觉伺服 | 13 状态状态机 `servo_yolo_grasping_node.py`：IDLE → SEARCHING → … → COMPLETED。PD 控制器、多坐标系变换 |
| `yolov8_grasping` | 视觉检测 | YOLOv8 目标检测抓取节点（Python），基于 OAK-D 相机 |
| `yolov8_obb` | 视觉检测 | YOLOv8 有向边界框检测（C++ GPU 推理） |
| `yolov8_obb_msgs` | 消息定义 | OBB 检测结果的自定义 ROS 2 消息 |
| `octomap_yolo_grasping` | 融合抓取 | OctoMap 三维障碍物地图 + YOLOv8 检测融合 |
| `graspnet_grasping` | 抓取规划 | GraspNet 抓取位姿规划 |
| `hand_eye_calibration` | 标定 | 手眼标定集成（launch + TF 发布），依赖 easy_handeye2 |
| `trajectory_retime_server` | 轨迹服务 | 轨迹重新定时/时间缩放 |
| `control_servers` | Web 控制 | FastAPI + Jinja2 Web 接口（`joint_control_app.py`），HTTP API 控制机械臂 |
| `data_monitor` | 数据监控 | 关节轨迹监控与数据记录 |
| `panda_arm_msg` | 消息定义 | Panda 机械臂自定义消息 |
| `GraphExecuter` | 工作流编排 | 独立项目（非 ROS 包），基于 PySide6 + NodeGraphQt 的可视化工作流系统 |

---

## 三、外部开源项目（GitHub 直接引用）

| 包名 | 来源 | 说明 |
|------|------|------|
| `pymoveit2` | Andrej Orsula | MoveIt2 的 Python 接口库，基于 ROS 2 actions/services |
| `easy_handeye2` | IFL-CAMP | 硬件无关的手眼标定库，含 GUI + 评估器 |
| `ros2_aruco` | ravijo | ArUco 标记检测节点 |
| `realsense2_gz_description` | Marq Rasmussen | Intel RealSense 相机 Gazebo 仿真 URDF |

---

## 四、厂商原材料（非代码包，同学加工后生成上述包）

| 原材料 | 由谁提供 | 产出 |
|------|------|------|
| CAD 模型 / STL 网格 | 法奥 | `fairino_description`、`s622_moveit_descriptions` 中的 meshes/ |
| `libfairino.so.2.2.5` SDK | 法奥 | `fairino_hardware` 封装调用 |
| 通信协议文档 | 法奥 | `fairino_msgs` 消息定义 |

---

## 五、统计

| 来源 | 数量 | 占比 |
|------|------|------|
| 厂商提供（含同学修补） | 6 个包 | 23% |
| 同学自研 | 16 个包 + 1 个独立项目 | 62% |
| 外部开源 | 4 个包 | 15% |

**核心结论：** 厂商只给了模型、SDK、通信协议。项目的核心竞争力——运动学算法、规划算法、视觉伺服、仿真环境、Web 控制——全部是同学自研。

---


# 包来源分类

## 同学自写核心代码（8 个）

这些是项目的核心原创代码，需要深入阅读和理解。

| 包名 | 说明 | 阶段 |
|------|------|------|
| `fairino_planning_core` | 核心运动学与规划算法（FK/IK/BiRRT*/RRT*），纯 C++，无 ROS 依赖 | 2 |
| `fairino_planning_ros` | MoveIt2 规划器插件 + IK 插件 + 碰撞检测包装 | 3 |
| `gz_launch` | Gazebo 仿真 launch 文件、场景模型、演示脚本 | 4 |
| `yolov8_grasping` | YOLOv8 目标检测抓取节点（Python） | 6 |
| `trajectory_retime_server` | 轨迹重新定时服务 | 6 |
| `visual_servo` | 视觉伺服 + 13 状态抓取状态机 + PD 控制器 | 8 |
| `octomap_yolo_grasping` | OctoMap 三维障碍物地图 + YOLO 融合抓取 | 8 |
| `graspnet_grasping` | GraspNet 抓取位姿规划 | 8 |

---

## 基础支撑功能（7 个）

这些是必要的配套代码，相对简单，多为模板/示例修改、消息定义或配置包装。

| 包名 | 说明 | 阶段 |
|------|------|------|
| `fairino_msgs` | 自定义 ROS 2 消息/服务定义（RobotNonrtState, RemoteCmdInterface, RemoteScriptContent） | 1 |
| `fairino_hardware` | ros2_control 硬件接口插件（SystemInterface），含 command_server | 5 |
| `yolov8_obb` | YOLOv8 有向边界框检测（C++，GPU 推理） | 6 |
| `yolov8_obb_msgs` | OBB 检测结果消息定义（package.xml 中名为 yolov8_msgs） | 6 |
| `hand_eye_calibration` | 手眼标定集成包（launch + TF 发布），包装 easy_handeye2 | 7 |
| `control_servers` | FastAPI Web 控制接口 + Jinja2 前端页面 | 9 |
| `data_monitor` | 关节轨迹监控与数据记录 | 9 |

---

## 工具生成 / 配置型（4 个）

这些由 SolidWorks URDF Exporter 或 MoveIt Setup Assistant 自动生成，再手动微调。

| 包名 | 说明 | 阶段 |
|------|------|------|
| `fairino_description` | URDF 机械臂模型，由 SolidWorks CAD 导出 | 1 |
| `s622_moveit_descriptions` | S622 URDF/SRDF，SolidWorks 导出 | 1 |
| `fairino3_v6_moveit2_config` | Fairino3 v6 的 MoveIt2 全套配置（SRDF + kinematics + controllers yaml） | 3 |
| `s622_moveit_config` | S622 的 MoveIt2 全套配置（主力配置） | 3 |

---

## 外部开源项目（5 个）

直接复制或 git clone 使用，非本项目原创。

| 包名 | 来源 | 说明 | 阶段 |
|------|------|------|------|
| `pymoveit2` | [Andrej Orsula](https://github.com/AndrejOrsula/pymoveit2) | MoveIt2 Python 接口封装 | 3 |
| `easy_handeye2` | [IFL-CAMP](https://github.com/IFL-CAMP/easy_handeye2) | 手眼标定库（Tsai/Lenoy 算法 + GUI） | 7 |
| `ros2_aruco` | [ravijo](https://github.com/ravijo/ros2_aruco) | ArUco 标记检测 | 6 |
| `realsense2_gz_description` | 开源 | Intel RealSense D435 相机仿真 URDF | 4 |
| `GraphExecuter` | 独立项目 | PySide6 + NodeGraphQt 可视化工作流编辑器 | 10 |

---

## 其他

| 包名 | 说明 |
|------|------|
| `panda_arm_msg` | Panda 机械臂消息定义，可能是遗留/测试用 |

---

## 厂家提供（非 ROS 包）

| 文件 | 说明 | 阶段 |
|------|------|------|
| `libfairino.so`（SDK） | Fairino 厂家 C++ SDK，预编译 .so 文件 | 5 |
| CAD 模型 / STL 网格 | Fairino S622 机械臂 SolidWorks 模型 | 1 |


fairino_description
s622_moveit_descriptions
fairino3_v6_moveit2_config
s622_moveit_config
fairino_msgs
fairino_hardware

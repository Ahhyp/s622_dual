# 阶段 1：机器人模型定义

## 目标

创建 S622 机械臂的 URDF 模型描述，使机器人模型能在 RViz 中可视化。

## 前提

- 无（不依赖其他阶段）
- 从原项目复制 URDF 和 STL 网格文件（学习阶段无需厂家 CAD）

## 待创建的包

| 序号 | 包名 | 说明 | 来源 |
|------|------|------|------|
| 1 | `fairino_description` | URDF + STL 网格 + launch/rviz 文件 | 从原项目复制 |
| 2 | `s622_moveit_descriptions` | S622 专用 URDF（SolidWorks 导出版） | 从原项目复制 |
| 3 | `fairino_msgs` | 自定义 ROS 2 消息/服务定义 | 从原项目复制 |

## 步骤

### 1. fairino_description

- 复制 URDF 文件（多型号，S622 使用 `fairino3_v6.urdf`）
- 复制 STL 网格文件（各关节连杆视觉模型）
- 复制 launch 文件（`display.launch.py`）和 RViz 配置
- `package.xml`：ament_cmake 构建类型，依赖 joint_state_publisher、robot_state_publisher、rviz2
- `CMakeLists.txt`：安装 urdf、meshes、launch、rviz 到 share 目录

### 2. s622_moveit_descriptions

- 从原项目复制（SolidWorks URDF Exporter 导出的版本）
- 包含更详细的惯性参数和碰撞几何

### 3. fairino_msgs

- 定义自定义消息：`RobotNonrtState`、`RemoteCmdInterface`、`RemoteScriptContent`

## 验证

```bash
source install/setup.bash
ros2 launch fairino_description display.launch.py
```

RViz 中应正确显示 S622 机械臂的 URDF 模型。

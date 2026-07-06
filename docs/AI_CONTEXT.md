# AI 上下文：S622 机械臂智能抓取系统

> 本文档供 AI 快速理解项目全貌。每次新对话开始时加载此文档。

---

## 1. 项目概况

- **项目**：S622（Fairino）6-DOF 机械臂智能抓取系统的学习复现
- **仿真**：Gazebo Fortress 6.16 + RViz2 + MoveIt2 + ros2_control
- **视觉**：Intel RealSense D435 RGBD 相机（仿真），YOLOv8 OBB 目标检测
- **控制**：MoveIt Servo 笛卡尔速度控制 + 2D 图像空间视觉伺服（IBVS）
- **操作系统**：WSL2 Ubuntu 22.04，ROS 2 Humble
- **源项目**：`~/S622_robotarm/`（26 包完整项目，本项目的参考）

## 2. 环境设置

```bash
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8

# 构建
rm -rf build install
colcon build --merge-install --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=$(which python3)"
source install/setup.bash
```

**注意**：
- 必须用 conda `yolov8` 环境（Python 3.10 + ultralytics + torch）
- `ros2 launch` 的子进程不会继承 conda 环境 → YOLO 节点需在 conda 终端单独启动
- WSL2 重启后需 `wsl --shutdown`（Windows PowerShell）恢复图形

## 3. 目录结构

```
src/
├── fairino_description/      # Fairino S622 URDF（厂商复制）
├── fairino_msgs/              # 自定义消息（厂商复制）
├── fairino_hardware/          # 硬件接口（厂商复制）
├── s622_moveit_descriptions/  # MoveIt URDF/SRDF（厂商复制）
├── s622_moveit_config/        # MoveIt 运动学/控制器配置
├── gz_launch/                 # Gazebo 仿真启动 + 相机配置 + 目标模型
│   ├── launch/s622_gazebo.launch.py  # 主 launch 文件
│   ├── config/robot_gazebo.urdf.xacro # 机器人 + 相机 URDF
│   └── models/target_box/     # 抓取目标 SDF 模型
├── fairino_planning_core/     # 规划核心
├── fairino_planning_ros/      # 规划 ROS 接口
├── visual_servo/              # 视觉伺服抓取（自写核心包）
│   ├── visual_servo_node.py   # 主状态机
│   ├── moveit_planner.py      # MoveIt 规划封装
│   ├── error_estimator.py     # 深度图处理 / 像素反投影
│   └── scripts/
│       ├── calibrate_jacobian.py      # j_img_to_base 标定
│       ├── collect_calib_data.py      # 坐标转换链数据采集
│       ├── verify_coordinate_chain.py # 坐标链诊断实验
│       └── calibrate_table_z.py       # table_z 标定辅助
└── yolov8_obb/                # YOLOv8 OBB 检测节点
```

## 4. 机器人配置

### 4.1 关节极限

| 关节 | 下限    | 上限    |
| ---- | ------- | ------- |
| j1   | -3.0543 | +3.0543 |
| j2   | -4.6251 | +1.4835 |
| j3   | -2.8274 | +2.8274 |
| j4   | -4.6251 | +1.4835 |
| j5   | -3.0543 | +3.0543 |
| j6   | -3.0543 | +3.0543 |

工作区：以 base 为中心，半径约 0.3~0.9m，中间位置 `(0.35, 0)` 较好。

### 4.2 相机配置（当前）

```xml
<!-- robot_gazebo.urdf.xacro -->
<origin xyz="0.35 0.5 0.9" rpy="0 ${58*M_PI/180} ${-M_PI/2}"/>
```

- D435 仿真（`realsense2_description` + `realsense2_gz_description`）
- 640×480, 60fps, h_fov=69°, v_fov=42°
- 内参：fx=465.6, fy=625.2, cx=320, cy=240
- camera_link → camera_color_optical_frame：标准 ROS optical 约定（rpy="-1.5708 0 -1.5708"）
- 相机原点在 base_link 系约 `(0.3825, 0.4838, 0.8976)`

### 4.3 末端执行器

- 二指平行夹爪（prismatic joint），开合范围 ±0.0305m（总约 61mm）
- `grasp_frame`：两指之间的中心点，wrist3_link + 偏移 (0, 0, 0.2168)
- 夹爪指向下时，grasp_frame 到指尖约 10mm

### 4.4 抓取目标

- **模型**：绿色立方体，4cm×4cm×4cm（`gz_launch/models/target_box/model.sdf`）
- **动态**（`static=false`），落地面后中心 z=0.02
- **YOLO 类别**：`cube`（YOLO 模型 `yolo-obb-gazebo.pt` 输出）

### 4.5 机械臂物理尺寸

#### 连杆长度（URDF 关节 origin 累加）

```
base_link
  │ j1: origin (0, 0, 0), axis Z
  ↓
shoulder_link
  │ j2: origin (0, 0, 0.14), axis Z（前置 X 转 90°）
  ↓
upperarm_link      ← 上臂，长 0.28m
  │ j3: origin (-0.28, 0, 0), axis Z
  ↓
forearm_link       ← 前臂，长 0.24m
  │ j4: origin (-0.24001, 0, 0), axis Z
  ↓
wrist1_link        ← 手腕 1
  │ j5: origin (0, 0, 0.102), axis Z（前置 X 转 90°）
  ↓
wrist2_link        ← 手腕 2，长 0.102m
  │ j6: origin (0, 0, 0.102), axis Z（前置 X 转 -90°）
  ↓
wrist3_link        ← 手腕 3，长 0.102m
  │ grasp_frame: fixed, origin (0, 0, 0.2168)
  ↓
grasp_frame        ← 夹爪中心（两指之间）
```

| 段                                 | 长度         |
| ---------------------------------- | ------------ |
| 上臂 (upperarm)                    | 0.28 m       |
| 前臂 (forearm)                     | 0.24 m       |
| 手腕段 1                           | 0.102 m      |
| 手腕段 2                           | 0.102 m      |
| wrist3 → grasp_frame               | 0.2168 m     |
| **j2 原点 → grasp_frame 最大伸展** | **≈ 0.94 m** |

#### 工作区

- j2 原点在 base_link 系 `(0, 0, 0.14)`
- 最大水平伸出半径 ≈ 0.94m（从 j2 原点算）
- 实际可达球壳半径：**0.3 ~ 0.9m**（base 系 XY 平面投影）
- 最大高度：**≈ 1.1m**（含 j2 原点偏移 0.14m）
- **最佳工作点**：`(0.35, 0)` 附近，臂姿态舒适，不奇异
- **奇异点**：臂完全伸直（j3≈0）时，j2/j4 速度方向退化

#### 底座

- base_link 原点在 `(0, 0, 0)`，Gazebo world 原点
- 底座大致直径 0.15m，高 0.14m（到 j2 原点）

### 4.6 夹爪物理参数

- **类型**：二指平行夹爪（prismatic joints: finger1_joint, finger2_joint）
- **单指行程**：0.0305m（单边）
- **开合总范围**：≈ 61mm（两指各 30.5mm）
- **闭合时两指间距**：0mm（`gripper_close_pos: [0.0, 0.0]`）
- **张开时两指间距**：≈ 50mm（`gripper_open_pos: [0.025, -0.025]`）
- **grasp_frame → 指尖 z 偏移**：≈ 10mm（指尖在 grasp_frame 下方）
- **grasp_frame → fingertip XY**：两指沿 ±Y 各 25mm（张开时）

### 4.7 桌面和工作面

- **Gazebo 地面**：z = 0（`empty.sdf` 默认地面平面）
- **目标方块落地后中心 z**：0.02（半高 0.02，底面触地）
- **方块上表面 z**：0.04
- **无物理桌面模型**——方块直接落在地面上，抓取时需注意下方无桌面可做碰撞检测参考

### 4.8 相机物理位置

- **相机原点在 base_link 系**：`(0.3825, 0.4838, 0.8976)`
- **相机视线方向**（optical Z）：大致沿 base -X 方向，俯角 58°
- **相机在地面上的投影**：`(0.3825, 0.4838)` 即机器人右前方
- **相机到工作区中心 (0.35, 0, 0.07) 距离**：≈ 0.93m（斜距）
- **图像中工作区像素**：约 `(336, 254)`——接近画面中心，略偏下

## 5. 抓取状态机

```
IDLE → DETECTING → COARSE_PLANNING → MOVING_TO_PREGRASP
→ VISUAL_ALIGN_XY → VISUAL_ALIGN_YAW → ALIGN_LOCK_CHECK
→ BLIND_DESCEND → GRASPING → LIFTING → VERIFY_GRASP → DONE
```

### 关键设计决策

1. **锁目标像素（lock_target_during_alignment）**：进入 VISUAL_ALIGN_XY 时将 YOLO 检测像素拍快照（`locked_target_uv`），后续对齐全程不读 YOLO，天然抗遮挡
2. **盲降（BLIND_DESCEND）**：纯 Z 下降，不读 YOLO，避免夹爪进入画面导致遮挡/检测失效
3. **远离相机偏移（pregrasp_camera_offset）**：pregrasp 沿"远离相机水平投影"方向偏移 3-5cm，EE 在目标的远相机侧，减少对齐阶段的 arm 自遮挡

### 启动命令

```bash
# 终端 1：启动仿真（不含 YOLO）
ros2 launch gz_launch s622_gazebo.launch.py

# 终端 2：YOLO（必须在 conda yolov8 环境）
source install/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
ros2 run yolov8_obb yolov8_obb_node --ros-args \
  -p model_path:="$HOME/my_S622/src/yolov8_obb/models/yolo-obb-gazebo.pt" \
  -p image_topic:="/camera/color/image_raw" \
  -p detections_topic:="/yolov8/obb_detections" \
  -p confidence_threshold:=0.05 -p imgsz:=1024 -p publish_empty:=true

# 终端 3：视觉伺服抓取
source install/setup.bash
ros2 run visual_servo visual_servo_node --ros-args \
  -p enable_motion:=true \
  -p z_strategy:="table" \
  -p table_z:=0.0 \
  -p grasp_height_above_table:=0.030 \
  -p pregrasp_height_above_table:=0.16 \
  -p lock_check_pixel_tolerance:=12.0 \
  -p descend_speed:=0.04 \
  -p descend_timeout:=30.0 \
  -p lifting_timeout:=30.0 \
  -p lift_height:=0.10 \
  -p visual_align_timeout:=25.0 \
  -p pregrasp_camera_offset:=0.03

# 终端 4：触发抓取
ros2 topic pub /servo_trigger std_msgs/msg/Bool "data: true" --once
```

## 6. 标定值

### j_img_to_base（pos1, 相机 0.35/0.5/0.9）

```
j_img_to_base = [[-0.001939,  0.000069],
                 [-0.000000,  0.001957]]   (m/px)
```

标定于 2026-06-23。EE 在 `(0.35, 0.0, 0.20)`，夹爪垂直向下。RMSE=0px, N=32。

`use_adaptive_jacobian=true` 时运行时自动计算 live Jacobian，此值作为 fallback。

### 方块常量

```
BLOCK_HALF = (0.020, 0.020, 0.020)   # 4cm 立方体半边长
CENTER_Z = 0.020                       # 落地后中心 z
```

### grasp_height_above_table

```
table_z (0.0) + 半高 (0.02) + grasp_frame→指尖偏移 (0.01) = 0.030
```

## 7. 坐标转换链（已验证）

- **E1（理论闭环）**：0μm 误差，证明 `base↔camera↔pixel` 数学链无 bug
- **pixel→base 精度**：X 方向 ~1.6mm，Y 方向 ~25mm（YOLO OBB 中心 ≠ 3D 几何中心投影）
- **YOLO 像素→base 不能做盲降定位**：误差 25-40mm（物理限制，非代码问题）
- **结论**：2D 图像空间伺服是正确的方案，3D 盲降不可靠

## 8. 已知问题和解决方案

| 问题                                 | 原因                                       | 解决                                                            |
| ------------------------------------ | ------------------------------------------ | --------------------------------------------------------------- |
| VISUAL_ALIGN_XY 时遮挡导致 YOLO 失效 | arm 经过 camera→target 视线                | lock_target + pregrasp_camera_offset                            |
| BLIND_DESCEND 降幅不足               | servo 执行慢                               | descend_speed=0.04, timeout=30s                                 |
| Ctrl+C 后 RViz Plan&Execute 不动     | servo 没释放控制权                         | `ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger` |
| MoveIt 规划 ERROR -2                 | pregrasp_camera_offset 太大导致目标不可达  | 减小 offset（0.05→0.03→0.02）                                   |
| YOLO conf 偏低（0.1-0.3）            | 4cm 方块在 640×480 画面上占比小            | 调低 min_confidence=0.10，detection 仍然稳定                    |
| ros2 launch 中 YOLO 崩溃             | 子进程不继承 conda 环境                    | YOLO 单独在 conda 终端启动                                      |
| cv_bridge NumPy 版本冲突             | ultralytics 依赖 numpy≥2，ROS 需要 numpy<2 | `pip install "numpy<2"` 锁定                                    |
| WSL 图形窗口不显示                   | WSLg 桥崩溃                                | Windows PowerShell: `wsl --shutdown`                            |

## 9. 代码风格

- 参数优先：所有可调值通过 `self.declare_parameter`，用 `--ros-args -p key:=value` 覆盖
- 状态机：`ServoState` Enum + `_enter_state()` + 每个 `_state_xxx()` 返回 twist
- TF 方向：`lookup_transform(base, camera)` → `P_base = R @ P_cam + t`，`P_cam = R.T @ (P_base - t)`
- 深度图：单位米，5×5 中值采样，`pixel_depth_to_camera` 用标准 pinhole 反投影

## 10. 文档索引

| 文档                                             | 内容                   |
| ------------------------------------------------ | ---------------------- |
| [CLAUDE.md](../CLAUDE.md)                        | 项目级 AI 指令         |
| [2026_6_22_debug报告.md](2026_6_22_debug报告.md) | 坐标转换链完整诊断报告 |
| [代码级项目搭建流程](代码级项目搭建流程.md)      | 13 阶段搭建规范        |
| [原项目结构](原项目结构.md)                      | 源项目 26 包目录树     |
| [issues.md](issues.md)                           | 问题记录               |

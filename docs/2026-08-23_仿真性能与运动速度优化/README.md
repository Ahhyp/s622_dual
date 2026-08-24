# 仿真性能与运动速度优化记录

- **日期**：2026-08-23 14:36
- **目录**：`docs/2026-08-23_仿真性能与运动速度优化/`
- **背景**：用户反馈机械臂运动速度慢（对比 robotarm 项目同机器快），且仿真 RTF 低

---

## 1. 问题诊断过程

| 阶段 | 发现 | 结论 |
|---|---|---|
| 1. 速度参数对比 | robotarm 与 my_S622 的 controller gains、joint_limits、velocity scaling **几乎相同**（MoveItMotion 0.3 vs moveit_planner 0.2） | 配置差异不足以解释"快很多" |
| 2. 实测 RTF | my_S622 仿真 RTF = **0.07**（后经优化到 0.3） | **仿真比真实慢 14 倍**，机械臂墙钟速度慢是 RTF 主导 |
| 3. CPU 分析 | Gazebo 吃 713% CPU（7 核）、RViz 223%、YOLO 43.8% | 仿真计算/渲染负载大 |
| 4. 相机配置对比 | my_S622：60fps @ 960×540 ×2 传感器；robotarm：60fps @ 640×480 | 相机渲染负载约为 robotarm 的 3 倍 |
| 5. RViz 面板 | `gz_launch.rviz` 的 **Velocity_Scaling_Factor: 0.1**（10% 速度） | **Plan&Execute 慢的直接原因**（与代码 scaling 无关） |
| 6. 物理步长 | gz 默认 `max_step_size=0.001`（1000Hz 物理） | 单步计算量大，RTF 上不去 |

---

## 2. 全部修改清单

### 2.1 RViz Plan&Execute 速度（直接快 10 倍）

| 文件 | 修改 |
|---|---|
| `src/gz_launch/rviz/gz_launch.rviz` | `Velocity_Scaling_Factor: 0.1 → 1.0`；`Acceleration_Scaling_Factor: 0.1 → 1.0` |

### 2.2 物理步长（RTF 核心优化，1000Hz → 200Hz）

| 文件 | 修改 |
|---|---|
| `src/gz_launch/worlds/s622_world.sdf`（**新建**） | 复制 gz 默认 empty.sdf，`max_step_size: 0.001 → 0.005`（200Hz 物理，匹配 controller 200Hz）；world 名保持 "empty"（话题不变） |
| `src/gz_launch/launch/s622_gazebo.launch.py` | gz_args 从 `"empty.sdf -r"` 改为指向 `worlds/s622_world.sdf -r` |

### 2.3 相机渲染负载（RTF 提升，约降 7 倍）

| 文件 | 修改 |
|---|---|
| `src/s622_moveit_descriptions/urdf/camera/camera.xacro` | `fps: 60 → 20`；`image_width: 960 → 640`；`image_height: 540 → 480` |
| `src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro` | 宏默认参数同步 `fps:=20`、`image_width:=640`、`image_height:=480` |

> 注：分辨率变化后 `camera_info` 自动更新内参；visual_servo `use_adaptive_jacobian=true` 运行时自适应，`j_img_to_base` 仅 fallback，无影响。

### 2.4 MoveIt 粗规划速度（0.2 → 1.0 全速）

| 文件 | 修改 |
|---|---|
| `src/visual_servo/visual_servo/moveit_planner.py` | `max_vel/max_acc: 0.2 → 1.0` |
| `src/s622_arm_actions/s622_arm_actions/moveit_planner.py` | `max_vel/max_acc: 0.2 → 1.0` |
| `src/s622_arm_actions/s622_arm_actions/move_to_pose_server.py` | `default_velocity_scale/acceleration: 0.2 → 1.0` |
| `src/s622_arm_actions/config/{arm,dual_arm,left_arm,right_arm}_config.yaml` | `default_velocity_scale/acceleration: 0.2 → 1.0` |

### 2.5 伺服阶段速度（visual_servo_node.py，约 4 倍）

| 参数 | 原值 → 新值 | 说明 |
|---|---|---|
| `control_rate` | 50 → **100** | 伺服循环 2 倍 |
| `max_linear_vel` | 0.08 → **0.3** | 伺服 XY 速度 |
| `max_angular_vel` | 0.45 → **1.5** | 伺服角速度 |
| `visual_align_gain` | 0.45 → **1.0** | 对齐增益 |
| `visual_align_max_step` | 0.0015 → **0.004** | 每周期步进（对齐速度关键瓶颈） |
| `visual_align_max_vel` | 0.045 → **0.2** | 对齐速度 |
| `descend_speed` | 0.018 → **0.12** | 盲降速度 |
| z 下降硬编码（`visual_servo_node.py` 内 `min(...,0.05)`） | 0.05 → **0.20** | z 速度封顶 |

### 2.6 MoveIt Servo 采样率

| 文件 | 修改 |
|---|---|
| `src/s622_moveit_config/config/servo.yaml` | `publish_period: 0.034 → 0.02`（30Hz → 50Hz） |

---

## 3. 生效方式与验证

- 上述改动除 `s622_arm_actions`/`visual_servo`（Python 拷贝式）需 `colcon build --packages-select` 重装外，xacro/launch/config 均经 symlink-install **即时生效**
- **所有改动需重启仿真生效**（world、camera、servo.yaml、RViz 配置均在 launch 启动时加载）
- 重启命令：`ros2 launch gz_launch s622_gazebo.launch.py`
- visual_servo 启动时 `descend_speed` 建议 `-p descend_speed:=0.12`（或删掉用默认）

**预期**：
- RViz Plan&Execute 全速（scaling 1.0）
- RTF 从 0.07 → 0.3 → **0.8+**（物理 200Hz + 相机降载）
- 机械臂墙钟运动速度显著提升

---

## 3.5 修复记录（2026-08-23 14:41）

**问题**：修改 world 后仿真无法启动（gz 报 `Unable to find or download file`，exit 255 → launch 自动关停）

**根因**：`install/share/gz_launch/worlds` 是旧的**拷贝目录**（非 symlink），新建的 `s622_world.sdf` 只在 src 里、install 里没有 → gz 找不到 world 文件

**修复**：重装 `gz_launch`（symlink-install）→ `s622_world.sdf` 进入 install → gz 正常加载

**经验**：`colcon build` 若曾用非 symlink-install 构建，`install/share/<pkg>` 下的 DIRECTORY 会变成拷贝而非 symlink——之后新增文件必须重装该包。

## 4. 验证结果（2026-08-23 14:5x）

✅ **仿真正常启动**（自定义 world 加载、机械臂 spawn 成功）
✅ **visual_servo 抓取闭环成功**（检测 → 粗规划 → 伺服对齐 → 下降 → 抓取 → 提升完整走通）
✅ Plan&Execute 全速（RViz scaling 1.0 生效）
✅ 机械臂运动速度显著提升（RTF 提升 + 全速轨迹）

---

## 4. ⚠️ 真机注意事项

**以上速度均为仿真优化值（Gazebo 下安全），真机部署必须降回保守值**：

| 参数 | 仿真值 | 真机建议 |
|---|---|---|
| MoveIt scaling | 1.0 | 0.1 ~ 0.3 |
| `max_linear_vel` / `max_angular_vel` | 0.3 / 1.5 | 0.08 / 0.45 |
| `descend_speed` | 0.12 | 0.02 ~ 0.04 |
| `visual_align_max_step` | 0.004 | 0.0015 |
| 物理步长 | 0.005（200Hz） | 真机无物理仿真，不适用 |

代码中相关参数旁已加注释"仿真值，真机建议降回"。

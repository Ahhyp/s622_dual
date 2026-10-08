# 阶段六：视觉伺服 visual_servo（自写）

## 目标

用视觉反馈形成闭环控制——摄像头持续看目标位置，PD 控制器把像素误差转成末端速度指令（TwistStamped），通过 MoveIt Servo 驱动机械臂实时追踪目标。

## 涉及包

| 包名 | 方式 | 说明 |
|------|------|------|
| visual_servo | **自写** | 视觉伺服节点（PD + 状态机 + 误差估计） |
| pymoveit2 | 复制 | MoveIt2 Python 接口（阶段五已复制） |
| trajectory_retime_server | 复制 | pymoveit2 依赖（阶段五已复制） |

## 分阶段实施

### Phase 6.1：PD 控制器骨架
- 2D/3D 误差估计（ErrorEstimator）
- PD 控制器（kp=0.8, kd=0.05, max_v=0.10）
- 状态机骨架：IDLE → SERVOING → DONE
- `enable_motion=False` 安全开关，先验证管道再开

### Phase 6.2：闭环伺服 + MoveIt Servo
- 发布 `TwistStamped` 到 `/servo_node/delta_twist_cmds`
- servo.yaml 配置，对接 MoveIt Servo
- 线上热切换参数（ros2 param set）

### Phase 6.3：完整抓取状态机
- IDLE → APPROACHING → DESCENDING → GRASPING → RETREATING → DONE
- 夹爪自动控制（JointTrajectory → hand_controller）
- debug_target 模式跳过像素管线，直接硬编码目标点

## 核心链路

```
YOLO检测 → ErrorEstimator(像素→3D→base_link误差)
  → PDController → TwistStamped
  → /servo_node/delta_twist_cmds
  → MoveIt Servo (Jacobian IK → joint velocities)
  → /robot_arm_controller/joint_trajectory
  → Gazebo机械臂
```

## 关键设计决策

- **PD 不用 I**：视觉伺服 50Hz 更新，积分容易累积过冲
- **笛卡尔直线追踪**：PD 按 target-ee 笛卡尔误差方向驱动，Jacobian 逆解转关节速度
- **奇异点保护**：初始姿态设弯曲位姿（J2/J3 ~±60°），配合 `hard_stop_singularity_threshold`

## 验收标准

- [x] Phase 6.1：PD 管道跑通，误差日志合理，motion=OFF 验证
- [x] Phase 6.2：MoveIt Servo 收 twist 并驱动机械臂（status 1/2）
- [x] Phase 6.3：完整抓取状态机，夹爪自动控制
- [ ] 奇异点问题根治（PD 推手臂至拉直）
- [ ] 目标可达性预检

## 后续

→ [阶段七](#)（OctoMap + GraspNet）

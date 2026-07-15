# 阶段 1：BT.CPP 单臂 Pick-Place 闭环重构

> 详细设计：[docs/双臂协同/阶段1.md](../../docs/双臂协同/阶段1.md)

## 目标

用 BehaviorTree.CPP v4 重构单臂抓取-放置任务层：
- 保留原有 2D/2.5D 抓取稳定性（IBVS + blind descend）
- 新增放置、撤离、回 home
- 使用 6D-compatible GraspCandidate 接口
- 为后续双臂任务预留扩展空间

## 前提

- BT.CPP v4 已安装（`ros-humble-behaviortree-cpp`）
- 原有 `visual_servo` 单臂抓取已验证通过
- YOLOv8 OBB 检测正常

## 待创建的包

| 序号 | 包名 | 说明 | 来源 |
|------|------|------|------|
| 1 | `s622_task_interfaces` | Action/Service/msg 接口定义 | 自写 |
| 2 | `s622_servo_actions` | 视觉伺服 Action Server（封装原 visual_servo 能力） | 部分复用 |
| 3 | `s622_bt_manager` | BT.CPP 任务管理器 + 自定义 BT 节点 | 自写 |

## 开发里程碑

### M1.1：BT 空框架跑通
- 创建 `s622_bt_manager` 包
- 加载 `single_arm_pick_place.xml`
- 注册 Dummy BT Nodes
- 验收：BT 能正常启动，节点返回 SUCCESS/FAILURE

### M1.2：接入 YOLO 检测
- `DetectObject` 订阅 `/yolov8/obb_detections`
- 输出 cube 的 uv、yaw、confidence、coarse pose
- 验收：YOLO 启动时 SUCCESS，无 cube 时 FAILURE

### M1.3：生成 6D-compatible GraspCandidate
- `GenerateGraspCandidate` 生成抓取候选
- RViz 可视化 pregrasp/grasp pose
- 验收：pose frame_id、z 高度、rp yaw 正确，pregrasp_camera_offset 生效

### M1.4：接入 MoveIt2 粗定位
- `MoveToPregrasp` 调用 MoveIt2
- 验收：规划成功到达 pregrasp，失败时 Recovery 触发

### M1.5：接入视觉伺服抓取
- LockTargetPixel → VisualAlignXY → VisualAlignYaw → AlignLockCheck
- BlindDescend → CloseGripper → Lift → VerifyGrasp
- 验收：恢复原有单臂抓取能力，旧 visual_servo_node.py 不再作为主入口

### M1.6：补齐放置流程
- GeneratePlaceCandidate → MoveToPrePlace → MoveToPlace
- OpenGripper → DetachObject → Retreat → GoHome
- 验收：完整 pick-place 闭环

### M1.7：Recovery 和日志
- 失败 → Recovery（StopServo → OpenGripper → GoSafePose）
- 记录 CSV：`experiments/results/phase1_pick_place_trials.csv`
- 验收：连续 10 次，成功率 ≥ 70%

## 验收指标

| 指标 | 目标 |
|------|------|
| 连续测试次数 | 10 次 |
| 完整 pick-place 成功率 | ≥ 70% |
| 抓取成功率 | ≥ 80% |
| Recovery 触发后安全退出率 | 100% |

## 最终交付物

1. `s622_bt_manager` 包
2. `s622_servo_actions` 包
3. `s622_task_interfaces` 包
4. `single_arm_pick_place.xml`
5. GraspCandidate 抽象
6. 全部 BT 节点（DetectObject / MoveToPose / VisualAlign / Gripper 等）
7. 实验 CSV 日志

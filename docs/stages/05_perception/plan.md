# 阶段五：感知与标定（部分自写）

## 目标

打通"视觉检测 → 抓取位姿估计 → 机械臂执行"完整链路。

## 涉及包

| 包名 | 方式 | 说明 |
|------|------|------|
| yolov8_obb_msgs | **自写** | YOLOv8 OBB 检测结果消息定义（InferenceResult, Yolov8Inference） |
| yolov8_obb | **自写** | YOLO 检测节点（先发假数据，后续换真模型） |
| yolov8_grasping | **自写** | 抓取核心：位姿估计 + 执行器 |
| pymoveit2 | 复制 | MoveIt2 Python 接口（从源项目复制） |
| trajectory_retime_server | 复制 | pymoveit2 的依赖（从源项目复制） |
| realsense2_gz_description | 复制 | Gazebo D435 仿真传感器定义（已提前完成） |
| gz_launch（相机扩展） | **自写** | ros_gz_bridge 完整桥接（RGB + 深度 + camera_info） |

## 分阶段实施（Phase 1-4）

### Phase 1：消息定义 + 回环测试
- `yolov8_obb_msgs` 建包，`rosidl_generate_interfaces()` + `member_of_group`
- `yolov8_obb_node` 发假检测框
- `grasping_node` 订阅并打印

### Phase 2：目标选择 + 假抓取位姿
- `grasping_node` 选置信度最高的检测框
- 发写死的 `/grasp_pose`（PoseStamped, base_link）

### Phase 3：像素 → 3D
- 订阅 `/camera/color/camera_info`（内参）
- 订阅 `/camera/depth/image_raw`（深度图，16UC1 mm）
- `PoseEstimator.pixel_to_camera()`：窗口取中位深度，针孔模型
- tf2 变换 `camera_color_optical_frame` → `base_link`
- 俯视抓取姿态：`grasp_quat_top_down(yaw)`
- 发布真实 `/grasp_pose`

### Phase 4：接 ArmExecutor（MoveIt2 执行）
- `ArmExecutor`：MoveIt2 运动 + 夹爪控制封装
- `GraspExecutorNode`：pregrasp → 笛卡尔下降 → 闭合 → 笛卡尔上升
- 夹爪通过 `joint_trajectory_controller` action 控制（`hand_controller`）
- 打开：`[0.025, -0.025]`，闭合：`[0.0, 0.0]`

## 仿真相机集成

```
Gazebo rgbd_camera sensor
  → gz topic /camera (image / depth_image / camera_info)
  → ros_gz_bridge (parameter_bridge)
  → ROS 2 topics:
      /camera/color/image_raw
      /camera/depth/image_raw
      /camera/color/camera_info
```

## 验收标准

- [x] YOLO 假检测 → grasping 订阅链路通
- [x] 相机桥接三话题全通（RGB + 深度 + 内参）
- [x] TF `base_link` → `camera_link` 稳定
- [x] 像素 → 3D 位姿转换正确
- [x] MoveIt2 规划执行成功
- [x] 夹爪开合控制通
- [x] 完整抓取流程（pregrasp → 下降 → 抓取 → 上升）跑通

## 后续

→ [阶段六](#)（visual_servo）

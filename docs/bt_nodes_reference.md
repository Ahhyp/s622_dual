# s622_bt_manager BT 节点说明

> 不含 `dummy_nodes.cpp`（M1.7 早期验证用，已弃用）

---

## 1. bt_executor_node.cpp — BT 执行器主节点

| 项目       | 说明                                       |
| ---------- | ------------------------------------------ |
| ROS 节点名 | `bt_executor`                              |
| 类型       | 主程序入口（非 BT 节点，是 BT 运行的容器） |

**功能**：加载 XML 行为树并 tick 执行。支持 Groot2 实时监控、TreeObserver 失败节点追踪、`/bt_trigger` 话题触发启动。

**参数**：

| 参数           | 默认值           | 说明                                        |
| -------------- | ---------------- | ------------------------------------------- |
| `tree_file`    | `dummy_tree.xml` | 行为树 XML 文件名（位于 `behavior_trees/`） |
| `tree_id`      | `DummyTree`      | 行为树 ID                                   |
| `tick_rate_hz` | `10`             | tick 频率                                   |
| `auto_start`   | `false`          | 是否启动即执行                              |
| `groot2_port`  | `1667`           | Groot2 监控端口                             |

**外部接口**：
- **订阅** `/bt_trigger`（Bool）— 触发启动
- **订阅** YOLO/depth/camera_info（通过 RosContext）
- **发布** `/grasp_visualization`（PoseArray）

**运行流程**：
1. `init()` 构建 BehaviorTreeFactory，注册所有 BT 节点，加载 XML
2. 收到 `/bt_trigger` data=true 后在独立线程中 `tree.tickOnce()` 循环
3. TreeObserver 在 FAILURE 时自动记录失败节点名到 blackboard `last_failure_node`

---

## 2. motion_nodes.cpp — 运动控制

注册节点：**1 个**

### MoveToPose（StatefulActionNode）

让机械臂运动到指定位姿或命名位姿。调用 `move_to_pose` action server。

**外部接口**：
- **Action Client** `move_to_pose`（类型 `MoveToPose`）

**Input Ports**：

| Port                   | 类型        | 默认值 | 说明                                 |
| ---------------------- | ----------- | ------ | ------------------------------------ |
| `named_pose`           | string      | `""`   | 命名位姿（home/safe 等），非空时优先 |
| `target_pose`          | PoseStamped | —      | 目标位姿（named_pose 为空时必填）    |
| `velocity_scale`       | double      | `0.2`  | 速度比例                             |
| `acceleration_scale`   | double      | `0.2`  | 加速度比例                           |
| `timeout_sec`          | double      | `20.0` | 超时时间                             |
| `ensure_servo_stopped` | bool        | `true` | 执行前是否先停 servo                 |

---

## 3. gripper_nodes.cpp — 夹爪控制

注册节点：**2 个**

### SetGripper（SyncActionNode）

控制夹爪开/合。调 `set_gripper` service。

**外部接口**：
- **Service Client** `set_gripper`（类型 `SetGripper`）

**Input Ports**：

| Port          | 类型   | 默认值 | 说明                  |
| ------------- | ------ | ------ | --------------------- |
| `command`     | string | —      | `"open"` 或 `"close"` |
| `timeout_sec` | double | `5.0`  | 超时时间              |

**Output Ports**：

| Port              | 类型   | 说明                   |
| ----------------- | ------ | ---------------------- |
| `finger_position` | double | 执行后 finger 实际位置 |

### VerifyGrasp（SyncActionNode）

验证是否抓到物体。订阅 `/joint_states`，监测 finger 位置，用三段判定法：
- finger 位置 < `finger_min_position` → **EMPTY**（完全闭合，没抓到）
- finger 位置 > 0.022 → **MISS**（几乎没动，未接触）
- 中间 → **GRASPED**（抓到物体）

**外部接口**：
- **订阅** `/joint_states`（JointState）

**Input Ports**：

| Port                  | 类型   | 默认值            | 说明                       |
| --------------------- | ------ | ----------------- | -------------------------- |
| `finger_min_position` | double | `0.005`           | 判定为空的最小 finger 位置 |
| `feedback_joint`      | string | `"finger1_joint"` | 监测的关节名               |
| `timeout_sec`         | double | `5.0`             | 超时                       |

**Output Ports**：

| Port              | 类型   | 说明             |
| ----------------- | ------ | ---------------- |
| `finger_position` | double | 当前 finger 位置 |

---

## 4. scene_nodes.cpp — 场景物体管理

注册节点：**2 个**

### AttachObject（SyncActionNode）

将物体附着到机械臂 link 上（抓取后）。调 `attach_object` service。

**外部接口**：
- **Service Client** `attach_object`（类型 `AttachObject`）

**Input Ports**：

| Port          | 类型   | 默认值          | 说明            |
| ------------- | ------ | --------------- | --------------- |
| `object_name` | string | `"cube"`        | 物体名          |
| `link_name`   | string | `"grasp_frame"` | 附着到哪个 link |
| `size_x/y/z`  | double | `0.04`          | 物体尺寸        |
| `offset_z`    | double | `0.02`          | Z 方向偏移      |
| `timeout_sec` | double | `3.0`           | 超时            |

### DetachObject（SyncActionNode）

将物体从机械臂释放（放置后）。调 `detach_object` service。

**外部接口**：
- **Service Client** `detach_object`（类型 `DetachObject`）

**Input Ports**：

| Port                | 类型        | 默认值   | 说明         |
| ------------------- | ----------- | -------- | ------------ |
| `object_name`       | string      | `"cube"` | 物体名       |
| `put_back_in_world` | bool        | `true`   | 是否放回世界 |
| `drop_pose`         | PoseStamped | 原点     | 放置位姿     |
| `timeout_sec`       | double      | `3.0`    | 超时         |

---

## 5. servo_nodes.cpp — 视觉伺服

注册节点：**2 个**

### VisualAlign（StatefulActionNode）

执行视觉伺服对齐（descend/lift/align_xy 等模式）。支持 `arm_prefix` 动态切换左右臂 action。

**外部接口**：
- **Action Client** `visual_align` 或 `/{prefix}/visual_align`（类型 `VisualAlign`）

**Input Ports**：

| Port                   | 类型   | 默认值  | 说明                                   |
| ---------------------- | ------ | ------- | -------------------------------------- |
| `mode`                 | string | —       | 模式（`descend`/`lift`/`align_xy` 等） |
| `arm_prefix`           | string | `""`    | 左右臂前缀（`"left"`/`"right"`）       |
| `target_x_base`        | double | `0.0`   | 目标 X（base 系）                      |
| `target_y_base`        | double | `0.0`   | 目标 Y（base 系）                      |
| `tolerance_m`          | double | `0.005` | 位置容差                               |
| `target_yaw`           | double | `0.0`   | 目标 yaw                               |
| `tolerance_rad`        | double | `0.05`  | yaw 容差                               |
| `distance`             | double | `0.0`   | 运动距离（descend/lift 用）            |
| `speed`                | double | `0.04`  | 运动速度                               |
| `timeout_sec`          | double | `25.0`  | 超时                                   |
| `ensure_servo_started` | bool   | `true`  | 是否先启动 servo                       |

### StopServo（SyncActionNode）

停止视觉伺服。调 `/servo_node/stop_servo` service，也支持 `arm_prefix`。

**外部接口**：
- **Service Client** `/{prefix}/servo_node/stop_servo`（类型 `Trigger`）

**Input Ports**：

| Port         | 类型   | 默认值 | 说明       |
| ------------ | ------ | ------ | ---------- |
| `arm_prefix` | string | `""`   | 左右臂前缀 |

---

## 6. grasp_nodes.cpp — 抓取/放置候选位姿生成

注册节点：**2 个**

### GenerateGraspCandidate（SyncActionNode）

从 YOLO 检测的像素坐标 (u, v, yaw) 计算 TCP 抓取和预抓取位姿。优先用**桌面平面交线法**（射线与 z=table_z 平面求交），失败时回退到深度传感器反投影。

**核心算法**：
1. 读 camera_info 获取内参
2. 查 TF 获取相机在 base 系的位姿
3. 主路径：射线与桌面平面（z=table_z+grasp_dz）求交 → 得到物体 base 坐标
4. 回退路径：深度图中值采样 → 像素反投影 → TF 变换到 base 系
5. 从立方体 4 个等价 yaw 中选离当前 EE yaw 最近的
6. 输出 grasp_pose + pregrasp_pose（沿相机方向偏移 `pregrasp_camera_offset`）

**外部接口**：
- **依赖** RosContext 中的 TF、camera_info、深度图、grasp_viz 发布器

**Input Ports**：

| Port                          | 类型   | 默认值          | 说明                   |
| ----------------------------- | ------ | --------------- | ---------------------- |
| `u`                           | double | —               | 像素 u 坐标            |
| `v`                           | double | —               | 像素 v 坐标            |
| `object_yaw`                  | double | —               | 物体 OBB 角度          |
| `table_z`                     | double | `0.0`           | 桌面在 base 系的 Z     |
| `grasp_height_above_table`    | double | `0.030`         | 抓取高度（桌上）       |
| `pregrasp_height_above_table` | double | `0.16`          | 预抓取高度             |
| `pregrasp_camera_offset`      | double | `0.03`          | 预抓取偏离相机方向距离 |
| `base_frame`                  | string | `"base_link"`   | 基坐标系               |
| `ee_frame`                    | string | `"grasp_frame"` | 末端坐标系             |
| `camera_x_in_base`            | double | `0.3825`        | 相机在 base 系的 X     |
| `camera_y_in_base`            | double | `0.4838`        | 相机在 base 系的 Y     |

**Output Ports**：

| Port                | 类型        | 说明           |
| ------------------- | ----------- | -------------- |
| `tcp_grasp_pose`    | PoseStamped | TCP 抓取位姿   |
| `tcp_pregrasp_pose` | PoseStamped | TCP 预抓取位姿 |
| `grasp_yaw`         | double      | 选定的抓取 yaw |
| `grasp_pose_x`      | double      | 抓取点 X       |
| `grasp_pose_y`      | double      | 抓取点 Y       |

### GeneratePlaceCandidate（SyncActionNode）

从 ROS 参数加载放置位姿。构造时读 `place_*` 和 `pre_place_*` 参数。

**参数**（roscpp）：

| 参数                 | 说明                           |
| -------------------- | ------------------------------ |
| `place_frame_id`     | 放置坐标系（默认 `base_link`） |
| `place_position`     | 放置位置 `[x, y, z]`           |
| `place_rpy`          | 放置姿态 `[r, p, y]`           |
| `pre_place_frame_id` | 预放置坐标系                   |
| `pre_place_position` | 预放置位置                     |
| `pre_place_rpy`      | 预放置姿态                     |

**Output Ports**：

| Port             | 类型        | 说明       |
| ---------------- | ----------- | ---------- |
| `place_pose`     | PoseStamped | 放置位姿   |
| `pre_place_pose` | PoseStamped | 预放置位姿 |

---

## 7. perception_nodes.cpp — 感知

注册节点：**3 个**（+ RosContext 辅助类）

### RosContext（辅助类，非 BT 节点）

管理感知相关 ROS 订阅器的共享上下文，在各感知 BT 节点间共享。

**成员**：
| 成员                             | 说明                        |
| -------------------------------- | --------------------------- |
| `yolo_sub` / `latest_yolo`       | YOLO OBB 检测结果缓存       |
| `depth_sub` / `latest_depth`     | 深度图缓存                  |
| `caminfo_sub` / `latest_caminfo` | 相机内参缓存                |
| `tf_buffer` / `tf_listener`      | TF2 变换                    |
| `grasp_viz_pub`                  | 抓取可视化 PoseArray 发布器 |

### DetectObject（SyncActionNode）

从缓存的 YOLO 检测结果中找指定类别的置信度最高的目标。

**外部接口**：
- **依赖** RosContext 中的 YOLO 缓存

**Input Ports**：

| Port             | 类型   | 默认值   | 说明                 |
| ---------------- | ------ | -------- | -------------------- |
| `object_name`    | string | `"cube"` | 目标类别名           |
| `min_confidence` | double | `0.10`   | 最小置信度           |
| `max_age_sec`    | double | `1.0`    | 检测结果最大有效时间 |

**Output Ports**：

| Port                | 类型   | 说明       |
| ------------------- | ------ | ---------- |
| `output_u`          | double | 中心像素 u |
| `output_v`          | double | 中心像素 v |
| `output_yaw`        | double | OBB 角度   |
| `output_confidence` | double | 置信度     |

### LockTargetPixel（SyncActionNode）

锁定目标像素坐标（pass-through）。`input_u/input_v` → `locked_u/locked_v`。用于在视觉对齐过程中保持目标不变。

**Input Ports**：`input_u`, `input_v`
**Output Ports**：`locked_u`, `locked_v`

### AlignLockCheck（ConditionNode）

检查当前检测到的目标是否仍在锁定像素的容差范围内。用于视觉伺服对齐过程中的闭环验证。

**外部接口**：
- **依赖** RosContext 中的 YOLO 缓存

**Input Ports**：

| Port             | 类型   | 默认值   | 说明       |
| ---------------- | ------ | -------- | ---------- |
| `locked_u`       | double | —        | 锁定的 u   |
| `locked_v`       | double | —        | 锁定的 v   |
| `tolerance_px`   | double | `12.0`   | 像素容差   |
| `min_confidence` | double | `0.05`   | 最小置信度 |
| `object_name`    | string | `"cube"` | 目标类别名 |

---

## 节点注册汇总

| 注册函数                  | 文件                 | 节点名                   | 类型           |
| ------------------------- | -------------------- | ------------------------ | -------------- |
| `registerMotionNodes`     | motion_nodes.cpp     | `MoveToPose`             | StatefulAction |
| `registerGripperNodes`    | gripper_nodes.cpp    | `SetGripper`             | SyncAction     |
|                           |                      | `VerifyGrasp`            | SyncAction     |
| `registerSceneNodes`      | scene_nodes.cpp      | `AttachObject`           | SyncAction     |
|                           |                      | `DetachObject`           | SyncAction     |
| `registerServoNodes`      | servo_nodes.cpp      | `VisualAlign`            | StatefulAction |
|                           |                      | `StopServo`              | SyncAction     |
| `registerGraspNodes`      | grasp_nodes.cpp      | `GenerateGraspCandidate` | SyncAction     |
|                           |                      | `GeneratePlaceCandidate` | SyncAction     |
| `registerPerceptionNodes` | perception_nodes.cpp | `DetectObject`           | SyncAction     |
|                           |                      | `LockTargetPixel`        | SyncAction     |
|                           |                      | `AlignLockCheck`         | Condition      |

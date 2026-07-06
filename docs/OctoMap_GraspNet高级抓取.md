# Phase 8 规划：OctoMap + GraspNet 高级抓取

## 1. 阶段定位

本阶段目标是将 S622 机械臂抓取系统从当前的“2D OBB 检测 + 深度中心点 + 俯视抓取”升级为“3D 场景理解 + 6DoF 抓取候选生成 + MoveIt2 碰撞验证 + 可靠抓取执行”。

建议阶段命名：

```text
Phase 7：YOLOv8 OBB 真检测
Phase 8：OctoMap + GraspNet 高级抓取
```

原因是 OctoMap 和 GraspNet 的改造范围较大，不只是替换检测节点，而是会引入新的 3D 感知、抓取生成、候选筛选和碰撞验证模块。

---

## 2. 当前系统基础

当前系统已经具备：

1. Gazebo + MoveIt2 + RViz + Servo 启动链路
2. 相机彩色图像、深度图和相机内参话题
3. YOLO OBB 检测消息接口
4. 像素点 + 深度 → camera frame → base_link 的 3D 转换能力
5. MoveIt2 pregrasp 规划能力
6. Servo 闭环接近能力
7. JointTrajectory 夹爪控制能力
8. 完整抓取状态机

当前抓取逻辑属于简化抓取：

```text
OBB 中心点 → 深度取 3D 点 → roll=π, pitch=0, yaw=OBB.angle → 抓取
```

该方法适合：

* 桌面上单个规则物体
* 顶视或近似顶视抓取
* 长方体、盒子等简单目标
* 环境障碍较少的场景

但对以下情况不够强：

* 物体堆叠
* 多物体杂乱场景
* 非规则形状物体
* 需要侧抓、斜抓的物体
* 需要严格避障的场景
* 桌面、墙体、障碍物需要进入规划场景的情况

---

## 3. OctoMap 是什么

OctoMap 是一种三维占据地图。它将空间划分成许多 3D 体素，并用概率表示每个体素是否被占用。

在机械臂抓取系统中，OctoMap 的作用是让 MoveIt2 知道环境中有哪些障碍物。

典型数据流：

```text
Depth Image / PointCloud2
        ↓
Occupancy Map Updater
        ↓
OctoMap
        ↓
MoveIt Planning Scene
        ↓
FCL Collision Checking
        ↓
MoveIt 规划避障
```

加入 OctoMap 后，MoveIt2 规划时可以考虑：

* 桌面
* 物体
* 障碍物
* 相机看到的环境点云
* 未知或被占用空间

对本项目的意义：

1. pregrasp 规划更安全
2. retreat 规划更安全
3. 机械臂不容易穿过桌面或障碍物
4. 可以为 GraspNet 候选抓取做碰撞验证
5. 后续支持更复杂场景抓取

---

## 4. GraspNet 是什么

GraspNet 是通用物体 6DoF 抓取检测方法和数据集体系。

与 YOLO OBB 不同，YOLO OBB 主要输出 2D 图像平面里的旋转框：

```text
center_x, center_y, width, height, angle
```

GraspNet 直接从 RGB-D 或点云中预测多个三维抓取候选：

```text
grasp position: x, y, z
grasp orientation: rotation / quaternion
grasp width: gripper opening width
grasp score: grasp quality
```

它输出的抓取不局限于俯视方向，可以产生真正的 6DoF 抓取姿态。

对于 S622 项目，GraspNet 的作用是：

1. 替代或增强当前 OBB yaw 抓取策略
2. 从点云中自动寻找可抓位置
3. 支持非规则物体
4. 支持多物体杂乱场景
5. 输出多个候选抓取，交给 MoveIt2 逐个筛选
6. 提高抓取泛化能力

---

## 5. 总体架构规划

Phase 8 推荐架构如下：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
        ↓
PointCloud 生成 / RGB-D 同步
        ↓
场景点云预处理
        ├── OctoMap → MoveIt Planning Scene → 碰撞检测
        └── GraspNet → grasp candidates
                          ↓
                  候选抓取筛选
                          ↓
                  TF 转 base_link
                          ↓
                  IK 预检 + 碰撞检测
                          ↓
                  MoveIt 规划到 pregrasp
                          ↓
                  Servo 精修接近
                          ↓
                  夹爪闭合
                          ↓
                  retreat
```

其中：

```text
OctoMap 负责环境建模和避障
GraspNet 负责抓取姿态生成
MoveIt2 负责 IK、规划和碰撞验证
visual_servo 负责末端接近阶段闭环修正
arm_executor 负责夹爪和流程执行
```

---

## 6. 推荐分阶段实施

## Phase 8.1：OctoMap 接入 MoveIt2

### 目标

先不接 GraspNet，只让 MoveIt2 能看到环境点云并生成 OctoMap。

### 输入

使用已有深度相机：

```text
/camera/depth/image_raw
/camera/color/camera_info
```

或生成点云：

```text
/camera/depth/points
```

### 工作内容

1. 确认 Gazebo 深度相机是否能提供点云
2. 如果没有点云，则增加 depth image → PointCloud2 转换节点
3. 在 `s622_moveit_config` 中增加 3D sensor 配置
4. 配置 MoveIt2 occupancy map monitor
5. 设置 OctoMap 参数：

   * map frame
   * resolution
   * max range
   * point subsample
   * padding
   * filtered cloud topic
6. 在 RViz 中显示 MotionPlanning 的 Planning Scene
7. 验证 MoveIt2 是否能看到 OctoMap 障碍物

### 验证标准

1. RViz 中能看到环境占据地图
2. 桌面或障碍物能进入 Planning Scene
3. MoveIt 规划不会穿过明显障碍
4. 不影响当前 pregrasp 规划和 Servo 流程

### 风险

1. TF 不完整导致点云无法进入 planning scene
2. 点云 frame 与 base_link 转换错误
3. OctoMap 分辨率过小导致计算慢
4. 深度图噪声导致地图抖动
5. 机械臂自身点云没有过滤，导致把机器人身体也建进地图

### 应对

1. 优先检查 TF：camera frame → base_link
2. 先用较粗分辨率，例如 0.03 m 或 0.05 m
3. 限制 max range
4. 配置 self filter 或传感器滤波
5. 先在简单桌面场景验证

---

## Phase 8.2：点云预处理节点

### 目标

为 GraspNet 准备稳定、干净、坐标一致的点云输入。

### 新增包建议

```text
graspnet_perception
```

或：

```text
s622_graspnet
```

### 节点建议

```text
pointcloud_preprocessor_node
```

### 输入

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/tf
```

### 输出

```text
/s622/graspnet/scene_cloud
/s622/graspnet/object_cloud
/s622/graspnet/workspace_mask
```

### 处理内容

1. RGB-D 时间同步
2. 深度图转点云
3. 去除无效深度
4. 限制 workspace 范围
5. 去除桌面平面
6. 降采样
7. 可选：结合 YOLO OBB 做 ROI 裁剪
8. 输出 GraspNet 可用格式

### 推荐策略

初期不要完全丢掉 YOLO OBB。YOLO OBB 可以作为 GraspNet 的 ROI 约束：

```text
YOLO OBB 找目标区域
        ↓
深度图裁剪目标点云
        ↓
GraspNet 只在目标区域生成抓取
```

这样比全场景 GraspNet 更稳定，速度也更快。

---

## Phase 8.3：GraspNet 推理节点

### 目标

接入 GraspNet 或 GraspNet baseline，输入点云，输出多个抓取候选。

### 新增节点

```text
graspnet_inference_node
```

### 输入

```text
/s622/graspnet/scene_cloud
```

或：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
```

### 输出

建议新建消息：

```text
s622_grasp_msgs/GraspCandidate.msg
s622_grasp_msgs/GraspCandidateArray.msg
```

### GraspCandidate 字段建议

```text
std_msgs/Header header
geometry_msgs/Pose pose
float32 score
float32 width
float32 depth
string source
```

### GraspCandidateArray 字段建议

```text
std_msgs/Header header
GraspCandidate[] grasps
```

### 输出话题

```text
/s622/grasp_candidates
```

### 推理结果含义

每个候选代表一个夹爪抓取位姿：

```text
pose.position：夹爪目标位置
pose.orientation：夹爪目标姿态
score：抓取质量评分
width：推荐夹爪开口
```

### 初版策略

1. 使用预训练 GraspNet 权重
2. 先在仿真点云上跑通
3. 只取 score 最高的前 N 个抓取
4. 发布到 RViz 可视化
5. 暂时不直接执行，先人工检查姿态是否合理

---

## Phase 8.4：抓取候选筛选节点

### 目标

GraspNet 会输出多个候选，但不能直接执行。必须经过筛选。

### 新增节点

```text
grasp_candidate_filter_node
```

### 输入

```text
/s622/grasp_candidates
```

### 输出

```text
/s622/selected_grasp_pose
```

### 筛选条件

1. 分数大于阈值
2. 位姿在机器人可达范围内
3. 夹爪宽度在物理范围内
4. 姿态符合夹爪运动学约束
5. IK 可解
6. 与 OctoMap 无碰撞
7. pregrasp 可规划
8. approach 方向不会撞桌面
9. retreat 方向安全

### 筛选流程

```text
候选抓取列表
    ↓
按 score 降序排序
    ↓
转换到 base_link
    ↓
计算 pregrasp pose
    ↓
IK 检查
    ↓
MoveIt 碰撞检查
    ↓
MoveIt 规划检查
    ↓
选第一个可执行候选
```

### 关键思想

GraspNet 负责“想象怎么抓”，MoveIt2 负责“判断能不能抓”。

不能只看 GraspNet score。

---

## Phase 8.5：接入现有执行链路

### 目标

让 GraspNet 选出的抓取姿态复用现有 Phase 6.6 执行流程。

当前 Phase 6.6 流程：

```text
trigger
  ↓
IK 预检
  ↓
stop_servo
  ↓
MoveIt 规划到 pregrasp
  ↓
start_servo
  ↓
APPROACHING
  ↓
DESCENDING
  ↓
GRASPING
  ↓
RETREATING
  ↓
DONE
```

Phase 8 中保持这个流程，但目标来源变化：

```text
原来：YOLO OBB → grasp_pose
现在：GraspNet → selected_grasp_pose
```

### 接入方式

保留 `/grasp_pose` 作为兼容接口，新增一个桥接节点：

```text
graspnet_to_grasp_pose_node
```

输入：

```text
/s622/selected_grasp_pose
```

输出：

```text
/grasp_pose
```

这样现有 `visual_servo_node` 和 `moveit_planner` 可以少改或不改。

---

## Phase 8.6：OctoMap + GraspNet 联合验证

### 目标

验证高级抓取系统在复杂环境下是否有效。

### 场景 1：单物体桌面抓取

目标：

```text
验证 GraspNet 输出是否比 OBB 抓取更稳定
```

标准：

1. 能生成抓取候选
2. 候选姿态合理
3. IK 可解
4. MoveIt 能规划到 pregrasp
5. 抓取成功

### 场景 2：多物体抓取

目标：

```text
验证系统能从多个候选中选出可执行抓取
```

标准：

1. GraspNet 输出多个候选
2. filter 能剔除不可达或碰撞候选
3. 最终选择一个可执行抓取

### 场景 3：障碍物避障抓取

目标：

```text
验证 OctoMap 对规划避障有效
```

标准：

1. 障碍物进入 Planning Scene
2. MoveIt 规划绕开障碍物
3. 不发生明显碰撞

### 场景 4：杂乱堆叠抓取

目标：

```text
验证 6DoF 抓取相对 OBB 俯视抓取的优势
```

标准：

1. 能找到非俯视抓取姿态
2. 能筛选出安全抓取
3. 抓取成功率高于简单 OBB 策略

---

## 7. 推荐包结构

建议新增：

```text
~/my_S622/src/
├── s622_grasp_msgs
│   ├── msg/GraspCandidate.msg
│   └── msg/GraspCandidateArray.msg
│
├── s622_octomap_config
│   ├── config/sensors_3d.yaml
│   └── launch/octomap_moveit.launch.py
│
├── s622_graspnet
│   ├── s622_graspnet/pointcloud_preprocessor_node.py
│   ├── s622_graspnet/graspnet_inference_node.py
│   ├── s622_graspnet/grasp_candidate_filter_node.py
│   └── s622_graspnet/graspnet_to_grasp_pose_node.py
│
└── visual_servo
    └── 复用现有状态机
```

也可以先不单独建 `s622_octomap_config`，直接把 OctoMap 配置放进 `s622_moveit_config/config/`。

---

## 8. 推荐接口设计

### 8.1 GraspNet 候选输出

话题：

```text
/s622/grasp_candidates
```

消息：

```text
GraspCandidateArray
```

用途：

```text
保存所有 GraspNet 候选，方便 RViz 可视化和调试
```

### 8.2 筛选后抓取输出

话题：

```text
/s622/selected_grasp_pose
```

消息：

```text
geometry_msgs/PoseStamped
```

用途：

```text
表示最终可执行的抓取位姿
```

### 8.3 兼容现有系统

话题：

```text
/grasp_pose
```

消息：

```text
geometry_msgs/PoseStamped
```

用途：

```text
继续给 visual_servo 和 moveit_planner 使用
```

---

## 9. 与 YOLO OBB 的关系

加入 GraspNet 后，YOLO OBB 不一定要删除。

推荐保留 YOLO OBB，作为 GraspNet 的辅助模块。

可选模式：

### 模式 A：YOLO OBB 单独抓取

适合简单场景：

```text
YOLO OBB → grasp_pose → 当前抓取流程
```

优点：

```text
快、简单、稳定、易调试
```

### 模式 B：YOLO OBB + GraspNet ROI

适合目标明确的高级抓取：

```text
YOLO OBB → ROI 裁剪点云 → GraspNet → 候选筛选 → 抓取
```

优点：

```text
速度更快，减少 GraspNet 在无关区域生成候选
```

### 模式 C：全场景 GraspNet

适合杂乱抓取：

```text
完整点云 → GraspNet → 多候选 → MoveIt 筛选 → 抓取
```

优点：

```text
泛化能力强，不依赖类别检测
```

建议路线：

```text
先做模式 B，再做模式 C
```

---

## 10. 执行优先级

推荐优先级如下：

### 优先级 1：OctoMap 先接入

原因：

```text
没有环境碰撞地图，GraspNet 输出再好也可能规划撞障碍
```

先完成：

1. 深度点云进入 MoveIt2
2. RViz 能看到 OctoMap
3. MoveIt 规划能避障

### 优先级 2：GraspNet 离线测试

原因：

```text
GraspNet 依赖 Python、CUDA、点云格式，直接接 ROS 风险较高
```

先完成：

1. 保存一帧 Gazebo RGB-D
2. 转成 GraspNet 输入格式
3. 离线跑 GraspNet demo
4. 可视化抓取候选

### 优先级 3：GraspNet ROS 节点化

将离线流程封装成 ROS2 节点。

### 优先级 4：MoveIt 筛选抓取候选

不要直接执行 GraspNet 分数最高的抓取，必须经过 IK 和碰撞验证。

### 优先级 5：接入现有闭环抓取

最后再接入 `visual_servo` 和 `arm_executor`。

---

## 11. 关键风险

### 风险 1：GraspNet 环境依赖复杂

GraspNet 常见依赖包括：

* PyTorch
* CUDA
* Open3D
* MinkowskiEngine 或类似稀疏卷积库
* 自定义 CUDA op
* graspnetAPI

这些依赖可能和 ROS2 Humble 的 Python 环境冲突。

应对：

```text
优先保持 conda yolov8 环境
必要时单独创建 graspnet conda 环境
ROS2 节点通过 conda 环境运行
```

### 风险 2：Gazebo 深度图和真实深度图差异

仿真深度图可能太干净，真实相机深度图会有噪声和缺失。

应对：

```text
先仿真跑通
再增加噪声模拟
最后迁移真实相机
```

### 风险 3：GraspNet 坐标系和机器人坐标系不一致

GraspNet 输出通常在相机坐标系下，需要准确转换到 `base_link`。

应对：

```text
严格检查 camera optical frame
确认 x/y/z 方向
用 RViz 可视化每个抓取坐标轴
```

### 风险 4：夹爪模型与 GraspNet 默认夹爪不同

GraspNet 通常假设两指平行夹爪，但宽度、深度、手爪坐标定义可能与 S622 夹爪不同。

应对：

```text
定义 grasp_frame 与 GraspNet gripper frame 的变换
限制 grasp width 到 S622 夹爪范围
必要时加固定姿态修正
```

### 风险 5：OctoMap 把机器人自身建进地图

如果没有 self-filter，深度相机可能看到机械臂自身，导致 MoveIt 认为自己的手臂是障碍。

应对：

```text
配置 self filter
限制相机视野和工作空间
先用固定桌面障碍验证
```

---

## 12. 成功标准

Phase 8 初步成功标准：

1. OctoMap 能进入 MoveIt2 Planning Scene
2. MoveIt 规划能避开点云障碍
3. GraspNet 能从相机点云生成多个抓取候选
4. 抓取候选能在 RViz 中可视化
5. 候选能转换到 `base_link`
6. 候选能经过 IK 检查
7. 候选能经过碰撞检查
8. 系统能选出一个可执行抓取
9. MoveIt 能规划到该抓取的 pregrasp
10. Servo 能完成最后接近
11. 夹爪能闭合并 retreat
12. 在简单物体场景下抓取成功

---

## 13. 最小可行版本

Phase 8 的 MVP 不建议一开始就做全功能。

最小可行版本如下：

```text
1. OctoMap 接入 MoveIt2
2. 保存一帧 RGB-D
3. 离线 GraspNet 生成 grasp candidates
4. 手动选择一个候选
5. 转成 PoseStamped
6. 复用现有 MoveIt + Servo 执行
```

MVP 跑通后，再逐步自动化：

```text
离线 GraspNet
    ↓
ROS GraspNet 节点
    ↓
自动候选筛选
    ↓
OctoMap 碰撞验证
    ↓
闭环抓取
```

---

## 14. 推荐最终路线

最终推荐路线：

```text
Phase 7：YOLOv8 OBB 真检测
    ↓
Phase 8.1：MoveIt2 OctoMap 环境建图
    ↓
Phase 8.2：点云预处理与 ROI 裁剪
    ↓
Phase 8.3：GraspNet 离线推理验证
    ↓
Phase 8.4：GraspNet ROS2 节点化
    ↓
Phase 8.5：候选抓取 IK + 碰撞筛选
    ↓
Phase 8.6：复用现有 MoveIt + Servo 抓取执行
    ↓
Phase 8.7：多物体、杂乱场景、避障抓取验证
```

结论：

```text
OctoMap 负责“安全地走过去”
GraspNet 负责“智能地决定怎么抓”
MoveIt2 负责“判断能不能到达并避障”
visual_servo 负责“最后几厘米闭环修正”
arm_executor 负责“执行夹爪和抓取流程”
```

这会把 S622 从规则物体的 2D 俯视抓取，升级到更接近真实机器人系统的 3D 智能抓取架构。

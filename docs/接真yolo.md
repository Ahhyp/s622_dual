# Phase 7 规划：将 `yolov8_obb` 从假检测替换为真实 YOLOv8 OBB 推理节点

## 1. 阶段目标

Phase 7 的目标是将当前 `yolov8_obb` 包中的固定假检测节点，改造为基于 Ultralytics YOLOv8 OBB 模型的真实视觉检测节点。

改造后，节点应完成以下功能：

1. 订阅 Gazebo 相机彩色图像话题 `/camera/color/image_raw`
2. 使用 YOLOv8 OBB 模型进行实时目标检测
3. 将检测结果转换为已有消息类型 `yolov8_obb_msgs/Yolov8Inference`
4. 继续发布到 `/yolov8/obb_detections`
5. 保持下游 `grasping_node` 和 `visual_servo_node` 不需要改动
6. 输出的 OBB angle 保持为弧度，直接使用 Ultralytics 结果
7. 为空检测场景提供明确处理方式，避免下游误用旧目标

本阶段完成后，视觉伺服和抓取执行链路将从“固定假目标”升级为“相机图像真实检测目标”。

---

## 2. 当前基础状态

当前系统已经完成以下能力：

* Gazebo + MoveIt2 + RViz + Servo 已能正常启动
* 相机话题已经桥接完成：

  * `/camera/color/image_raw`
  * `/camera/depth/image_raw`
  * `/camera/color/camera_info`
* `grasping_node` 已经能够根据 OBB 检测、深度和相机内参计算目标 3D 位姿
* `visual_servo_node` 已经完成闭环抓取状态机
* `moveit_planner` 已经支持触发时先 IK 预检，再规划到 pregrasp
* 机械臂已能完成完整抓取动作

当前 `yolov8_obb` 节点仍然发布固定假数据：

```bash
center_x = 320
center_y = 240
angle = 0.0
confidence = 0.9
```

发布话题为：

```bash
/yolov8/obb_detections
```

Phase 7 的核心就是只替换这个检测来源，不破坏下游接口。

---

## 3. 技术路线

本阶段采用“最小改动、先跑通、后优化”的策略。

### 3.1 节点输入

订阅相机彩色图像：

```bash
/camera/color/image_raw
```

消息类型：

```bash
sensor_msgs/msg/Image
```

图像通过 `cv_bridge` 转换为 OpenCV BGR 图像，再传入 Ultralytics YOLO 模型。

### 3.2 模型加载

模型必须只在节点初始化时加载一次，不允许在图像回调函数中重复加载。

计划参数：

```bash
model_path
device
imgsz
confidence_threshold
max_det
```

默认模型可先使用：

```bash
yolov8n-obb.pt
```

后续正式抓取应替换为自训权重，例如：

```bash
s622_block_obb.pt
```

### 3.3 推理方式

Phase 7 初版采用同步推理：

```text
收到图像 → cv_bridge 转图 → YOLO predict → 解析 OBB → 发布消息
```

同步推理实现简单，便于调试。

后续如出现延迟问题，再考虑：

* 单独推理线程
* 只保留最新图像帧
* 降低相机 FPS
* 调整 `imgsz`
* 使用 GPU
* 导出 TensorRT / ONNX

---

## 4. 权重选择规划

### 4.1 短期测试权重

短期使用官方：

```bash
yolov8n-obb.pt
```

用途：

* 验证节点能否启动
* 验证模型能否加载
* 验证图像能否进入 YOLO
* 验证 OBB 消息格式是否正确
* 验证 `/yolov8/obb_detections` 是否能被下游继续订阅

注意：该权重主要面向 DOTA 航拍数据集，识别类别与机械臂抓取物体并不完全匹配。因此 Gazebo 中普通方块、盒子、工件不一定能被检测到。

### 4.2 正式抓取权重

正式抓取阶段应使用自训 OBB 权重。

建议训练一个单类或少类 OBB 模型，例如：

```text
class 0: grasp_object
```

训练目标应与 Gazebo 中实际抓取物体一致，例如：

* 长方体工件
* 方块
* 盒子
* 圆柱类物体
* 自定义夹取目标

自训权重输出的 `angle` 才能稳定用于夹爪 yaw 对齐。

---

## 5. 设备规划

节点参数 `device` 默认设置为：

```bash
auto
```

逻辑：

```text
如果 torch.cuda.is_available() 为 True，则使用 GPU 0
否则使用 CPU
```

运行前检查 conda 环境：

```bash
eval "$(conda shell.bash hook)" && conda activate yolov8
python - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
print("torch cuda:", torch.version.cuda)
if torch.cuda.is_available():
    print("gpu:", torch.cuda.get_device_name(0))
PY
```

推荐策略：

* 有 NVIDIA GPU：使用 `device:=0`
* 没有 GPU：使用 `device:=cpu`
* 不确定：使用 `device:=auto`

---

## 6. 仿真物体规划

### 6.1 使用官方权重时

如果使用 `yolov8n-obb.pt`，Gazebo 中应临时放置较接近 DOTA 类别外观的物体，例如：

* 船形贴图
* 飞机贴图
* 车辆俯视图贴图
* 长条形航拍目标贴图

这一步只用于验证 OBB 检测节点，不作为最终抓取方案。

### 6.2 正式抓取时

正式抓取建议使用 Gazebo 中的真实抓取物体，并基于该物体采集数据、自训 OBB 模型。

推荐物体：

```text
s622_block
```

或类似长方体工件。

要求：

* 物体在相机图像中边界清晰
* 顶视或斜视角下能稳定标注旋转框
* 长边方向明确
* OBB angle 能代表夹爪 yaw 对齐方向

---

## 7. 消息格式保持兼容

继续使用已有消息：

```bash
yolov8_obb_msgs/msg/Yolov8Inference
```

其中：

```text
results: InferenceResult[]
```

每个 `InferenceResult` 字段保持不变：

```text
class_name
confidence
center_x
center_y
width
height
angle
```

映射关系如下：

| YOLOv8 OBB 输出 | ROS 消息字段 |
| --------------- | ------------ |
| class name      | class_name   |
| confidence      | confidence   |
| center x        | center_x     |
| center y        | center_y     |
| OBB width       | width        |
| OBB height      | height       |
| OBB rotation    | angle        |

其中 `angle` 直接使用 Ultralytics 输出的弧度值，不做单位转换，不做正负号转换。

---

## 8. 没检测到目标时的处理策略

采用策略：

```text
发布空 results
```

即：

```yaml
results: []
```

不采用“不发布消息”的方式。

原因：

1. 下游可以明确知道当前帧没有检测目标
2. 避免 `grasping_node` 或 `visual_servo_node` 继续使用上一帧旧目标
3. 更适合闭环视觉伺服系统
4. 状态机可以根据空检测进入等待、丢失目标或失败恢复逻辑

节点中设置参数：

```bash
publish_empty:=true
```

默认值为 `true`。

---

## 9. 节点参数设计

计划支持以下 ROS 参数：

```bash
model_path
image_topic
detections_topic
confidence_threshold
device
imgsz
max_det
publish_empty
yolo_verbose
drop_if_busy
```

默认值：

```bash
model_path:=yolov8n-obb.pt
image_topic:=/camera/color/image_raw
detections_topic:=/yolov8/obb_detections
confidence_threshold:=0.25
device:=auto
imgsz:=640
max_det:=20
publish_empty:=true
yolo_verbose:=false
drop_if_busy:=false
```

其中：

* `confidence_threshold` 用于过滤低置信度结果
* `device` 控制 CPU/GPU
* `imgsz` 控制推理输入尺寸
* `max_det` 控制最大检测数量
* `publish_empty` 控制无目标时是否发布空结果
* `drop_if_busy` 为后续多线程优化预留

---

## 10. 实施步骤

### Step 1：确认环境

进入 conda 环境：

```bash
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
```

检查依赖：

```bash
python - <<'PY'
import torch
import ultralytics
from ultralytics import YOLO
print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
print("ultralytics ok")
PY
```

### Step 2：编写 `yolov8_obb_node.py`

将原来的假检测逻辑替换为真实 YOLOv8 OBB 推理逻辑。

关键要求：

* `YOLO(model_path)` 只在 `__init__` 中执行
* 图像回调中只做推理，不加载模型
* 使用 `cv_bridge` 转图
* 使用 `model.predict(...)`
* 解析 `result.obb.xywhr`
* 发布 `Yolov8Inference`

### Step 3：确认 package 配置

检查 `setup.py` 中是否有 entry point，例如：

```python
entry_points={
    'console_scripts': [
        'yolov8_obb_node = yolov8_obb.yolov8_obb_node:main',
    ],
},
```

如果没有，需要补上。

### Step 4：重新构建

```bash
cd ~/my_S622
colcon build --merge-install --symlink-install --packages-select yolov8_obb
source install/setup.bash
```

### Step 5：启动仿真

```bash
ros2 launch gz_launch s622_gazebo.launch.py
```

### Step 6：单独启动 YOLO OBB 节点

```bash

ros2 run yolov8_obb yolov8_obb_node \
  --ros-args \
  -p model_path:=/home/yep/my_S622/src/yolov8_obb/models/yolo-obb-gazebo.pt \
  -p image_topic:=/camera/color/image_raw \
  -p detections_topic:=/yolov8/obb_detections \
  -p confidence_threshold:=0.25 \
  -p device:=auto \
  -p imgsz:=640 \
  -p publish_empty:=true
```

### Step 7：检查输出话题

```bash
ros2 topic echo /yolov8/obb_detections
```

预期情况有两种：

检测到目标时：

```yaml
results:
- class_name: ...
  confidence: ...
  center_x: ...
  center_y: ...
  width: ...
  height: ...
  angle: ...
```

没有检测到目标时：

```yaml
results: []
```

---

### Step8: 检查相机是否正常工作
```bash
ros2 topic list | grep camera
```
确认有
/camera/color/image_raw

看频率：
```bash
ros2 topic hz /camera/color/image_raw
```
看 header：
```bash
# ros2 topic echo --once /camera/color/image_raw/header
ros2 topic echo --once /camera/color/image_raw --field header

```

然后用 RViz 或 rqt 看图像：
rqt_image_view

选择：
/camera/color/image_raw
确认画面里真的能看到你的目标物体，而且目标没有太小、太暗、被遮挡、超出画面。

### Step 9 先向仿真环境里面添加一个正方体，以便后续 yolo 识别他
#### Step 1：先 spawn 一个长方体

例如先做一个简单 SDF：
```bash
mkdir -p ~/my_S622/src/gz_launch/models/target_box
gedit ~/my_S622/src/gz_launch/models/target_box/model.sdf
```

写入：
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="target_box">
    <static>false</static>

    <link name="link">
      <pose>0 0 0.02 0 0 0</pose>

      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00005</ixx>
          <iyy>0.00005</iyy>
          <izz>0.00005</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.10 0.06 0.04</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.10 0.06 0.04</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

再建 model.config：
```bash
gedit ~/my_S622/src/gz_launch/models/target_box/model.config
```

写入：
```xml
<?xml version="1.0"?>
<model>
  <name>target_box</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>S622</name>
  </author>
  <description>Simple target box for YOLO OBB grasping test</description>
</model>
```


#### Step 2：启动仿真后 spawn 它

如果你用的是 Gazebo Classic ，可以试：
```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity target_box \
  -file ~/my_S622/src/gz_launch/models/target_box/model.sdf \
  -x 0.45 -y 0.00 -z 0.04 \
  -R 0 -P 0 -Y 0.5
```

这里：
```
-x 0.45
-y 0.40
-z 0.04
-Y 0.5
```

表示放在机械臂前方，并且绕 z 轴旋转 0.5 rad。这样你可以测试 OBB angle 是否变化。

✔ 我的是这个
如果你用的是 Ignition/Gazebo Harmonic 或 ros_gz_sim，命令可能是：
```bash
ros2 run ros_gz_sim create \
  -name target_box \
  -file ~/my_S622/src/gz_launch/models/target_box/model.sdf \
  -x 0.45 -y -0.40 -z 0.04 \
  -R 0 -P 0 -Y 0.5
```
没有在画面里面就直接在gazebo拖动



#### 在 RViz 看相机的图片，确认有这个东西。


#### Step 4：降低阈值测试检测

启动 YOLO 节点时先用低阈值：
```bash
MODEL=$HOME/my_S622/src/yolov8_obb/models/yolo-obb-gazebo.pt

ros2 run yolov8_obb yolov8_obb_node \
  --ros-args \
  -p model_path:="$MODEL" \
  -p image_topic:=/camera/color/image_raw \
  -p detections_topic:=/yolov8/obb_detections \
  -p confidence_threshold:=0.05 \
  -p device:=auto \
  -p imgsz:=1024 \
  -p publish_empty:=true
```

然后看：
```bash
ros2 topic echo /yolov8/obb_detections
```

如果有检测，应该看到：

results:
- class_name: ...
  confidence: ...
  center_x: ...
  center_y: ...
  width: ...
  height: ...
  angle: ...


#### Step 5: 接 visual_servo

先不要直接完整抓取，先开安全模式：
```bash
ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=false \
  -p enable_grasp_sequence:=false
```

然后触发：

```bash
ros2 topic pub --once /servo_trigger std_msgs/msg/Bool "{data: true}"
```

看 visual_servo_node 日志里是否出现：
```
trigger: enter PLANNING
```

或者是否报：
```
trigger refused: no valid target
TF target→base failed
not reachable
```

```bash
ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=true \
  -p enable_grasp_sequence:=true


ros2 topic pub --once /servo_trigger std_msgs/msg/Bool "{data: true}"
```

### 这里进行调试检测， 需要很多步骤， 故而创建 md 文档。
[检测可视化](./检测可视化.md)

## 11. 验证标准

Phase 7 初步完成的标准：

1. `yolov8_obb_node` 能正常启动
2. 模型只加载一次
3. 节点能订阅 `/camera/color/image_raw`
4. 节点能发布 `/yolov8/obb_detections`
5. 消息类型仍为 `Yolov8Inference`
6. 下游节点不需要改代码
7. 无检测时发布空 `results`
8. 有检测时能正确输出：

   * 类别名
   * 置信度
   * OBB 中心点
   * OBB 宽高
   * OBB 角度
9. `angle` 单位保持 rad
10. 不影响 Phase 6.6 已完成的抓取闭环逻辑

---

## 12. 风险与应对

### 风险 1：官方权重检测不到 Gazebo 物体

原因：`yolov8n-obb.pt` 与抓取物体类别不匹配。

应对：

* 短期放置类似 DOTA 类别的测试目标
* 长期训练自定义 OBB 权重

### 风险 2：CPU 推理太慢

应对：

* 降低 `imgsz`
* 降低相机 FPS
* 使用 `device:=0`
* 后续增加推理线程
* 后续导出 TensorRT / ONNX

### 风险 3：OBB angle 与夹爪 yaw 方向存在 90 度偏差

原因：

* 模型宽高定义与夹爪夹取方向可能不一致
* 物体长边方向和夹爪闭合方向需要确认

应对：

* 先直接使用 `angle`
* 实测抓取方向
* 如有必要，在下游或检测节点中增加 `angle_offset` 参数，例如 `π/2`

### 风险 4：下游使用旧目标

应对：

* 无检测时发布空 `results`
* 下游状态机后续可增加目标超时判断

### 风险 5：YOLO 输出多个目标

应对：

* Phase 7 初版全部发布
* 下游继续按现有逻辑处理
* 后续可增加目标选择策略：

  * 最高置信度
  * 离图像中心最近
  * 指定 class_name
  * 指定 ROI 区域

---

## 13. 后续优化方向

Phase 7 跑通后，可继续进入以下优化：

1. 自训 S622 抓取物体 OBB 权重
2. 增加 `target_class` 参数，只发布指定类别
3. 增加 `angle_offset` 参数，修正夹爪方向
4. 增加检测可视化图像发布
5. 增加 FPS 和推理耗时日志
6. 增加目标丢失超时机制
7. 增加图像队列，只保留最新帧
8. 增加 GPU/TensorRT 加速
9. 与 `visual_servo` 状态机增加更严格的目标有效性判断
10. 整理为 launch 文件统一启动

---

## 14. 本阶段结论

Phase 7 不改变抓取节点、视觉伺服节点和消息接口，只替换检测源。

推荐执行路线：

```text
先用 yolov8n-obb.pt 跑通节点和消息链路
再准备 Gazebo 真实抓取物体数据
然后训练自定义 OBB 权重
最后接入完整抓取闭环
```

完成 Phase 7 后，S622 项目将从基于假目标的闭环抓取，升级为基于真实视觉检测的智能抓取流程。

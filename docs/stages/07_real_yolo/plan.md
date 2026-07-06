# 接真 YOLO：假检测 → 真实 YOLOv8 OBB 推理

> 详见 [接真yolo.md](../../接真yolo.md)

## 目标

将 `yolov8_obb` 节点从固定假检测替换为基于 Ultralytics YOLOv8 OBB 模型的真实视觉检测。

## 设计原则

- **只换检测源，不破坏下游**：消息仍是 `yolov8_obb_msgs/Yolov8Inference`
- `grasping_node` 和 `visual_servo_node` 不需要改代码
- 同步推理起步，先跑通再优化

## 涉及文件

| 文件 | 改动 |
|------|------|
| `yolov8_obb/yolov8_obb/yolov8_obb_node.py` | 替换：假检测 → Ultralytics YOLO 推理 |
| `yolov8_obb/models/` | 新增：pt 权重文件（从源项目复制） |

## 技术路线

```
收到图像 → cv_bridge 转 OpenCV → YOLO predict → 解析 OBB → 发布 Yolov8Inference
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| model_path | `yolo-obb-gazebo.pt` | 源项目 Gazebo 权重 |
| confidence_threshold | 0.25 | 置信度过滤 |
| device | auto | CPU/GPU 自动 |
| imgsz | 640 | 推理尺寸 |
| publish_empty | true | 无目标时发空 results |

## OBB 输出 → ROS 消息映射

| YOLO 输出 | 消息字段 |
|-----------|----------|
| class name | class_name |
| confidence | confidence |
| center x/y | center_x / center_y |
| OBB width/height | width / height |
| OBB rotation (rad) | angle |

## 模型文件

从源项目复制到 `src/yolov8_obb/models/`：

| 文件 | 用途 |
|------|------|
| `yolo-obb-gazebo.pt` | 首选，源项目 Gazebo 仿真用 |
| `yolov8n-obb.pt` | 备选，官方 DOTA 预训练 |
| `yolo-obb1/2/3.pt` | 迭代训练版本 |
| `best_stone.pt` | 石头/特定物体 |

## 验证标准

1. 节点正常启动，模型只加载一次
2. 能订阅 `/camera/color/image_raw` 并推理
3. 发布 `/yolov8/obb_detections`，消息类型不变
4. 无检测时发空 `results`
5. 有检测时 angle 为弧度，OBB 字段完整
6. grasping_node / visual_servo_node 不改也能消费

## 后续

→ 自训 S622 抓取物体 OBB 权重
→ 接入完整抓取闭环

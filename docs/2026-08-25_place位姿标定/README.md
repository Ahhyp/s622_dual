# place 位姿标定（2026-08-25）

## 背景

BT `pick_place_dual.xml` 全流程跑到 **place 失败**：

```
[bt_executor] MoveToPose: sending goal xyz (0.3000, -0.2000, 0.0300) rpy (3.142, 0.000, 0.000)
[FairinoIKPlugin] [IK][call=3] ... raw_candidates=4 → selected
[fairino_planner] Fairino plan failure: planner=birrt* message=BiRRT*: goal configuration in collision
[left.move_to_pose_server] Planning failed! Error code: -1
```

抓取侧全部成功（检测→pregrasp→对齐→descend→close→verify GRASPED→lift→safe），
唯一失败点就是 place 目标位姿的 IK 选中解与场景碰撞。

## 根因分析

### 1. place 位置是从单臂复制来的，双臂坐标系下不合理

- 单臂：`base_link` 在 world 原点，`place=(0.30,-0.20,0.03)` 在自身前方偏左，舒适。
- 双臂 left：`left_base_link` 在 world (0.35,0,0) yaw=π，
  `place=(0.30,-0.20,0.03)` → world **(0.05, 0.20, 0.03)**，z 太低 + 位置贴桌面中心。
- 此时指尖/手腕贴近桌面，attach 的 cube（grasp_frame 上方 2cm）底边距桌面仅 ~1cm，
  move_group 严格查碰撞 → 选中解 goal in collision。

### 2. fairino planner 只取一个 IK 解、不做多解碰撞重试

IK call=3 有 4 个 raw candidates，选中连续性最优（dq_norm=0.93）的解，
但该解在场景中碰撞；planner 不尝试其他候选（如 wrist_fold 被拒的 q1=129.88° 分支），
直接 BiRRT* fast-fail → code=-1。

### 3. 潜在 bug：detach 落点 frame 错位（本次未触发，place 修好后必踩）

- BT `DetachObject drop_pose="{place_pose}"`，place_pose 是 **left_base_link 系**。
- 服务端 `planning_scene_service` 以 `base_link="world"` 发布，
  `co.header.frame_id = self._base`（world）+ `primitive_poses = drop_pose`（left_base_link 系）→ **错位**。
- cube 会被放到 world 系 (0.30,-0.20,0.03) 附近 —— 超出 right 臂工作空间，且与 left 臂 home 冲突。

## 修复方案（用户拍板：place 放到"另一只机械臂能舒适夹取的位置"）

### 坐标系换算

- left：`left_base_link (x_l, y_l)` → world `(0.35 - x_l, -y_l)`
- right：`right_base_link (x_r, y_r)` → world `(x_r - 0.35, y_r)`

### 新位姿（两臂交接区，world 对称）

| 参数 | left | right |
|---|---|---|
| place | `(0.25, -0.20, z)` → world **(0.10, 0.20)** | `(0.25, 0.20, z)` → world **(-0.10, -0.20)** |
| right 系视角 | right 系 (0.45, 0.20) 前方偏左，舒适 | — |
| left 系视角 | — | left 系 (0.45, 0.20) 前方偏左，舒适 |
| pre_place z | 0.20（不变） | 0.20 |

z 抬到 **0.06**（指尖离桌面留余量，避免 goal in collision；cube 靠 detach 落桌）。

> 注：right_place 对称式见 `bt_dual_config.yaml` 注释，本次先改 left（当前 BT 只跑 left）。

### detach drop_pose frame 修复

在 `DetachObjectNode::tick()` 里用 `tf_buffer` 把 drop_pose 从 left_base_link 变换到 world
再传给服务端（对齐服务端 `base_link="world"`）。

## 验证

1. 改 `bt_dual_config.yaml`（left place/pre_place 位姿 + z）
2. 改 `scene_nodes.cpp`（drop_pose world 变换，用 `arm_prefix` 的 base frame）
3. 重启仿真 → `ros2 topic pub --once /bt_trigger std_msgs/msg/Bool "{data: true}"`
4. 期望：place SUCCESS → SetGripper open → Detach → lift → home → **BT SUCCESS**

# 阶段六：遇到的问题与解决方案

---

## 1. MoveIt Servo 始终 INACTIVE（status=0）

**现象**：`ros2 topic echo /servo_node/status` 始终输出 `data: 0`，servo 收到 twist 但不处理。

**原因**：
- servo 启动后默认 paused，需要 `start_servo` service 激活
- QoS 不匹配：servo subscriber 用 BEST_EFFORT+VOLATILE，`ros2 topic pub` 默认发 RELIABLE+TRANSIENT_LOCAL
- `ros2 topic pub` 发的时间戳是全零（wall clock），被 servo 当作过期数据丢弃
- `incoming_command_timeout: 0.5`（0.5 秒没收到有效 twist 就停）

**修复**：
1. 每次仿真重启必须调 `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}`
2. 写 test_servo.py 用 `self.get_clock().now().to_msg()` 打正确时间戳
3. 放宽 `incoming_command_timeout` 到 5.0 秒
4. QoS 匹配：`--qos-reliability best_effort`

**教训**：MoveIt Servo 的激活链路比看上去复杂——它不是启动就能用，需要 service call + 正确时间戳 + QoS 匹配三样都对。

---

## 2. 碰撞检测参数名不匹配

**现象**：`servo.yaml` 里设 `collision_check: false`，但 `ros2 param get /servo_node moveit_servo.check_collisions` 返回 `True`。

**原因**：servo.yaml 的 key 名是 `collision_check`，但 MoveIt Servo 内部读的 key 是 `check_collisions`。名字对不上，配置不生效。

**修复**：servo.yaml 改用 `check_collisions: false`。

**教训**：yaml key 名不等于代码参数名，需要 `ros2 param list` 验证。

---

## 3. 全零默认姿态 = 奇异点

**现象**：仿真启动机械臂全伸直（所有关节 0），MoveIt Servo 一激活就急停：`Very close to a singularity, emergency stop`。

**原因**：全零姿态（手臂完全伸直）是 Jacobian 条件数最大的奇异姿态之一，末端在某些方向无法运动。

**修复**：改 `initial_positions.yaml` 为弯曲姿态，J2=-1.05(-60°), J3=1.05(60°)，肘部显著弯曲避开奇异。

**教训**：机械臂默认姿态不能随便设全零，要考虑奇异点。

---

## 4. PD 笛卡尔直追 → 手臂拉直 → 奇异

**现象**：不管 debug_target 设在哪里（[0.45,0.1,0.25]、[0.3,0.0,0.35]、[0.25,0.0,0.4]），servo 跟踪几十毫米后机械臂被拉直，进入奇异点急停。

**根本原因**：纯笛卡尔 PD 没有关节空间约束。PD 沿 `target - ee` 直线方向驱动，Jacobian 逆解不关心"保持弯曲姿态"。手臂为够到目标自然被拉直。

**影响**：这是纯 PD + servo 架构的固有局限，不是具体参数问题。

**待解决方向**：
- null-space 投影：在关节速度上叠加"保持弯曲姿态"的次要目标
- 奇异感知减速：status 变 2 时自动降速
- 可达性预检：APPROACHING 前用 IK 算目标是否可达
- 降低 `max_linear_vel` 和 `linear_scale`

---

## 5. `enable_motion` 参数缓存 bug

**现象**：`ros2 param set /visual_servo_node enable_motion true` 显示成功，但节点仍发零 twist，日志显示 `motion=OFF`。

**原因**：`__init__` 里 `self.enable_motion = self.get_parameter(...).value` 只读了一次，控制循环里用 `self.enable_motion` 而不是实时 `get_parameter()`。

**修复**：控制循环改为 `self.get_parameter("enable_motion").value`。

**教训**：ROS 2 参数是动态的，但 Python 变量不是。`ros2 param set` 只改参数服务器里的值，不更新 Python 对象属性。

**同样受影响的参数**：`enable_grasp_sequence`、`grasp_duration`，已同步修复。

---

## 6. `tf_transformations` 依赖链

**现象**：`ros2 run visual_servo` → `ModuleNotFoundError: tf_transformations` → 装了 apt 后 → `ModuleNotFoundError: transforms3d`。

**修复**：
```bash
sudo apt install ros-humble-tf-transformations
pip install transforms3d -i https://pypi.org/simple/
```

**教训**：conda yolov8 环境缺少一些 ROS Python 的传递依赖，apt + pip 补。

---

## 7. visual_servo_node 缺少 main()

**现象**：`ros2 run visual_servo visual_servo_node` → `AttributeError: module has no attribute 'main'`。

**原因**：setup.py entry_point 指向 `visual_servo.visual_servo_node:main`，但代码里只有 `class VisualServoNode`，没有 `def main()`。

**修复**：加 `main()` + `if __name__ == "__main__"`，用 `MultiThreadedExecutor` spin。

---

## 8. Robot arm controller 卡死（sim 重启后）

**现象**：仿真重启后 `ros2 action send_goal` 被接受但机械臂不动，feedback 里 `desired == actual`，误差始终为零。

**原因**：`ros_gz_sim create` 没 spawn 成功（`Requesting list of world names` 循环），controller 连不上 Gazebo。

**修复**：`pkill -9 -f gz && pkill -9 -f ros_gz` 后重新启动。

**教训**：WSL2 下 Gazebo 偶尔启动卡住，kill 重来一般就好。

---

## 9. `servo_node` 僵尸进程混入

**现象**：`ros2 topic info /servo_node/delta_twist_cmds -v` 发现 subscriber 是 `servo_node`，但状态始终 INACTIVE，参数未设置。

**原因**：`fairino3_v6_moveit2_config` 的 launch 文件带起了另一个 `servo_node`，用的是 fairino3 的配置，跟 S622 对不上。

**修复**：确认 launch 文件里 servo_node 来自 `s622_moveit_config/config/servo.yaml`，把 fairino3 版本排除。

---

## 10. MultiThreadExecutor + pymoveit2 + spin_until_future_complete 崩溃

**现象**：在 `visual_servo_node` 里加了 `MoveItPlanner`（内部用 pymoveit2）后，`ros2 run` 启动即崩：

```
RCLError: Failed to get number of ready entities for action client:
  wait set index for status subscription is out of bounds,
  at ./src/rcl_action/action_client.c:623
```

**触发条件**：三个因素同时存在才崩：
1. `MultiThreadedExecutor`
2. 在回调里调用 `spin_until_future_complete`
3. pymoveit2 的 `MoveIt2` 内部建了 `move_action` 和 `follow_joint_trajectory` 两个 **action client**

三缺一不崩。pymoveit2 的 action client 在 MultiThreadedExecutor 里和回调的同步等待产生竞态，ROS 2 Humble 的 `rcl_action` wait set 管理在这种场景下有 bug。

**影响**：加 MoveIt 粗对齐规划（在 `_on_trigger` 回调里同步调 `spin_until_future_complete` 等待规划结果）时必崩。

**状态**：**未修复**。潜在方向：
- 换 `SingleThreadedExecutor` + 异步回调
- 把 MoveIt 规划放到独立节点，通过 topic/service 通信
- pymoveit2 的 action client 放到独立的 callback group 里

**教训**：ROS 2 Humble 的 MultiThreadedExecutor 和同步 action client 调用不兼容。学习项目先记录，后续排查后再修。

---

## 11. pixel→3D→base_link 坐标转换 y 向系统偏差 → 架构重设计

**日期**：2026-06-15

**现象**：通过 YOLO + 深度 + TF 算出的 base_link 3D 坐标在 y 方向系统性偏 +6~+14cm，偏移量随物体位置变化。

**原因**：固定外置倾斜相机（79°俯视）+ fx≠fy 内参不对称导致的几何系统性误差，不是代码 bug。

**解决**：放弃纯 3D 伺服架构，重新设计为 2D 图像空间闭环：
- pixel→3D 降级为"rough target"，仅用于 MoveIt 粗规划
- 精确对齐用 `_compute_image_error_uv()`：target 像素 − 末端投影像素
- 对齐后盲 Cartesian 下降（不依赖视觉 3D 坐标）
- 新状态机：COARSE_PLANNING → VISUAL_ALIGN_XY → VISUAL_ALIGN_YAW → DESCEND_WITH_FEEDBACK

**教训**：固定外置倾斜相机的纯 3D 伺服在仿真中有原理性限制。源项目用 2D 图像伺服规避了此问题。

# 执行记录：双臂 launch 时序 + 相机性能优化（对齐 robotarm）

> 日期：2026-08-25
> 场景：s622_dual_arm.launch.py / s622_dual_arm_gazebo.urdf.xacro

---

## 问题（用户反馈）

1. **TimerAction 魔法数字**：9 个 TimerAction（7/8/12/15/15/25/28s），启动慢、难维护
2. **RViz 卡顿/帧率低**：双臂 xacro 调用 `camera_v0` 未传 use_camera_visual
   → 默认 true（真实 d435.dae mesh）→ RViz 加载完整相机模型
3. **相机像素**：✅ 已对齐（20fps/640×480/hfov 82.4°，camera.xacro 统一配置），无需改

## robotarm 参考方案（对齐目标）

- **时序**：仅 3 个 TimerAction——`robot_spawn_delay=5s`（参数化）、
  `controller_spawn_delay=8s`（参数化）、rviz=controller+1s；
  **move_group 立即启动**（不延迟）
- **相机**：`enable_camera_model` launch 参数控制 mesh 开关

## 改动

### 1. `src/gz_launch/config/s622_dual_arm_gazebo.urdf.xacro`
- 加 `include_camera_visual` arg（默认 false）
- `camera_v0 use_camera_visual="$(arg include_camera_visual)"`（对齐单臂 robot_gazebo.urdf.xacro）

### 2. `src/gz_launch/launch/s622_dual_arm.launch.py`
- 加 launch 参数：`robot_spawn_delay`（默认 5.0）、`controller_spawn_delay`（默认 8.0）
- builder mappings 加 `include_camera_visual: "false"`
- 时序重构（对齐 robotarm）：
  | 组件 | 旧 | 新 |
  |---|---|---|
  | spawn_robot | 立即 | 5s（robot_spawn_delay 参数化）|
  | spawn_box | 7s | 7s（= robot 5 + 2，注释说明）|
  | JSB spawner | 8s | 8s（controller_spawn_delay 参数化）|
  | arm/hand spawner | 12s | 9s（= ctrl 8 + 1）|
  | planning_scene | **25s** | 10s（= ctrl 8 + 2）|
  | move_group × 2 | **15s 延迟** | **立即启动**（对齐 robotarm）|
  | RViz | **28s** | 9s（= ctrl 8 + 1）|
  | servo × 2 | 17s | 10s（= ctrl 8 + 2）|
  | arm_actions | 18s | 11s（= servo 10 + 1）|
  | bt_executor | **22s** | 13s（= arm_actions 11 + 2）|
- **TimerAction 从 9 个 → 8 个，但 move_group 不再延迟**，全链路启动时间从 ~28s 降到 ~13s

## 验证

- ✅ launch 语法 OK
- ✅ launch 描述生成：spawn_robot(参数化) → cube(7) → JSB(8) → arm/hand(9) →
  planning_scene(10) → **move_group×2 立即** → rviz(9) → servo×2(10) → arm_actions(11) → bt(13)
- ✅ xacro：`include_camera_visual=true` 有 d435.dae；`false` 无（0 匹配）→ 相机简化生效
- ✅ gz_launch 构建通过，install 同步

## 下一步

用户重启仿真验证：启动更快、RViz 不卡、功能不受影响

---

## 运动提速（2026-08-25 追加）

### 问题
BT `pick_place_dual.xml` 显式 `velocity_scale=0.2~0.3`（覆盖 config 的 1.0）
+ WSL2 RTF 低 → 机械臂运动慢（与单臂早期同症状）。

### 改动（src/s622_bt_manager/behavior_trees/pick_place_dual.xml）
| 节点 | 旧 | 新 |
|---|---|---|
| home / pregrasp / safe / pre_place / 收尾 home | 0.2~0.3 | **0.8/0.8** |
| place（放置） | 0.10 | 0.10（保留，放置要稳）|
| Recovery safe / home | 0.2 | **0.6/0.6** |

### 验证
- ✅ XML 解析 OK
- ✅ install 为 symlink 即时生效（无需构建）

### 待办（用户已确认后续做）
- place 目标位置重新标定：放到**另一只机械臂能舒适夹取的位置**
  （当前 left_place=[0.30,-0.20,0.03] 是单臂复制值，双臂坐标系下 IK 解碰撞）

### 提速链路核对（2026-08-25）
- BT XML velocity_scale=0.8 → MoveToPose action goal → move_to_pose_server `v=goal.velocity_scale`
  → MoveItMotion `arm.max_velocity=0.8` → pymoveit2 `max_velocity_scaling_factor`
  → fairino planner TOTG `velocity_scaling=0.8`（日志已证 0.3 时显示 velocity_scaling=0.300）
- **链路正确，install XML 已更新（symlink），需重启仿真生效**
- 注意：descend/lift 是 servo 速度（0.04/0.05 m/s），不走 velocity_scale，属安全接近阶段

---

## 🔴 提速未生效的真正根因：MoveItMotion._wait 提前返回（2026-08-25 追加修复）

### 现象（用户重启仿真验证后）
- BT 提速到 0.8 但仍慢
- 且机械臂**未到位就 descend/关夹爪**（与"没降到方块位置就关夹爪"同源）

### 日志铁证
```
MoveToPose: sending goal (to=30.0s)  [t=+0.0s]
MoveToPose: SUCCESS                  [t=+0.8s]   ← BT 以为完成了
left_arm_controller: Goal reached    [t=+126s]   ← 实际才执行完！
```

### 根因
`manipulation_common/planning/motion_executor.py::_wait()`：
```python
def _wait(self, moveit_obj, action_name, timeout_sec):
    if self.abort is not None:
        return self.abort.wait_idle_or_abort(...)   # 真正等执行完成
    time.sleep(min(timeout_sec, 0.5))                # ← 无 abort 时只睡 0.5s 就返回 True！
    return True
```
- robotarm 的 `fairino_pose_control_server` 构造 MoveItMotion 时传了 `abort=AbortManager(...)`
- **我们的 move_to_pose_server / gripper_service 没传 abort → _wait 只 sleep 0.5s 提前 SUCCESS**
- → BT 立即进入下一步（VisualAlign/descend），此时机械臂还在执行上一步轨迹 → servo 与轨迹冲突 → 未到位、z 不准、姿态乱

### 修复
- `move_to_pose_server.py`：`self.abort = AbortManager(self, arm=self.moveit2, gripper=None)` + `abort=self.abort`
- `gripper_service.py`：`self.abort = AbortManager(self, arm=self.moveit2_arm, gripper=self.moveit2_gripper)` + `abort=self.abort`
- `query_state()` 确认存在（MoveIt2State.IDLE=0 / REQUESTING=1 / EXECUTING=2，与 wait_idle_or_abort 匹配）
- 构建通过

### 预期效果（重启仿真后）
1. MoveToPose 真正等机械臂执行完才 SUCCESS
2. 提速 0.8 真正生效（不会再被"提前返回"掩盖）
3. descend/关夹爪在正确时机触发（到位之后）

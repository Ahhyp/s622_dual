# C2 接入规划：s622_arm_actions 改用 manipulation_common.MoveItMotion

- **日期**：2026-08-24
- **目录**：`docs/2026-08-24_C2_arm_actions接入MoveItMotion/`
- **状态**：规划稿（未执行）
- **前置**：阶段 A/B/C1/架构迁移/D2 已完成（见 `docs/2026-08-23_manipulation_common并入方案/`）

---

## 0. 决策背景（用户意图）

> manipulation_common 功能已验证充分（C1 demo 跑通），**直接用它的接口，不重复造轮子**。
> 因此不搞 adapter 兼容层、不留双路径开关——**接口层（arm_actions）内部直接换用 MoveItMotion**。

---

## 1. robotarm 考察结果（怎么调用 manipulation_common）

### 1.1 核心模式：接口层（ROS）内部直接 new MoveItMotion

```
用户/LLM
   ↓ ExecutePreview.action
llm_yolo_task_server（继承 FairinoPoseControlServer）   ← 顶层 Action Server
   ├── ControlPose.srv（Service Server）
   └── 内部 self.motion = MoveItMotion(...)              ← 直接用 manipulation_common
        ├── move_to_pose() / move_to_joints()
        └── control_gripper()
```

### 1.2 robotarm 关键文件与用法

| 文件 | 角色 | MoveItMotion 用法 |
|---|---|---|
| `llm_arm_control/fairino_pose_control_server.py` | **Service Server**（`ControlPose.srv`） | `MoveItMotion(node, arm_clients={"fairino": ...}, gripper=..., abort=..., pose_tools=..., select_best_path=..., score_cfg=...)`；`move_to_pose` / `control_gripper` / `wait_client_ready` |
| `llm_arm_control/llm_yolo_task_server.py` | **Action Server**（`ExecutePreview.action`），**继承** pose control server | 复用 `self.motion`：`move_to_pose` / `move_to_joints` |
| `yolov8_grasping/demo_node*.py` | 独立 demo 节点 | 同款构造，`gripper=None`（无夹爪版） |
| `yolov8_grasping/visual_grasping_node.py` | 抓取主节点 | 同款 |

### 1.3 关键结论

1. **robotarm 没有"BT 层"**，顶层是 LLM/CLI；接口层是 `llm_arm_control`（等价我们的 `s622_arm_actions`）
2. **接口层内部直接 new MoveItMotion**——不是绕过接口层，而是**换内部引擎**
3. **gripper 是 MoveItMotion 一等公民**：`MoveItMotion(..., gripper=self.moveit2_gripper, open_positions=..., close_positions=...)` → `control_gripper()` 直接开合夹爪
4. 参数标配：`arm_clients={"fairino": moveit2}` + `select_best_path` + `score_cfg` + `action_delay`

### 1.4 层级对照

| 层 | robotarm | 我们（当前） | C2 目标 |
|---|---|---|---|
| 顶层 | LLM CLI → ExecutePreview.action | s622_bt_manager（C++ BT） | 不变 |
| 接口层 | llm_arm_control（Action/Srv Server） | **s622_arm_actions** | 保留 |
| 运动库 | **MoveItMotion** | MoveItPlanner（自写） | **换 MoveItMotion** |
| MoveIt | move_group + fairino_planning_ros | 同 | 不变 |

---

## 2. 现状对账（已读代码确认）

### 2.1 s622_arm_actions 包功能

**本质 = ROS 接口层**（Action/Service Server，供 C++ BT 调用），**不被 manipulation_common 覆盖**（后者是 Python 库，无 ROS 接口）。但**内部运动实现被 MoveItMotion 覆盖**。

| server | 接口（来自 s622_bt_manager msg/srv） | 内部实现 | 是否换 MoveItMotion |
|---|---|---|---|
| move_to_pose_server | `MoveToPose.action` | MoveItPlanner | ✅ **换** |
| gripper_service | `SetGripper.srv` | 直接发 JointTrajectory | ✅ **换**（`control_gripper`） |
| visual_align_server | `VisualAlign.action` | servo Twist 视觉对齐 | ❌ 不换（MoveItMotion 无此能力，属视觉伺服） |
| planning_scene_service | `AttachObject.srv` 等 | planning_scene topic | ❌ 不换（MoveItMotion 无 attach/detach） |
| dual_move_server | `DualMoveToJointState.action` | 双臂 | ❌ 不换（双臂阶段再处理） |

### 2.2 BT 调用方式（为什么不能"绕过" arm_actions）

s622_bt_manager（C++ BT）通过 **ROS Action/Service 接口**调用，不直接接触 MoveIt：

| BT 节点 | 调用目标 | 是否经 arm_actions |
|---|---|---|
| MoveToPose | action `move_to_pose`（arm_prefix 前缀） | ✅ 是 |
| DualMoveToJointState | action `/dual/move_to_joint_state` | ✅ 是 |
| SetGripper | srv `set_gripper` | ✅ 是 |
| VisualAlign | action `visual_align` | ✅ 是 |
| Attach/Detach/TransferObject | srv `attach_object` 等 | ✅ 是 |
| StopServo/StartServo | srv `/servo_node/stop_servo` | ❌ 直连 servo_node |
| DetectObject/LockTargetPixel | topic `/yolov8/obb_detections` 等 | ❌ 直连感知话题 |

**结论**：BT 是 C++、manipulation_common 是 Python，**无法绕过接口层直接调用**。正确做法 = 保留 arm_actions 接口层，换内部引擎（MoveItPlanner → MoveItMotion）。这与 robotarm 的做法完全一致。

---

## 3. 改动文件清单（2 个 + 1 个测试）

| # | 文件 | 动作 | 说明 |
|---|---|---|---|
| 1 | `src/s622_arm_actions/s622_arm_actions/move_to_pose_server.py` | 改 | 内部 `MoveItPlanner` → `MoveItMotion`（`move_to_pose` / `move_to_joints`）；**不留开关，直接换** |
| 2 | `src/s622_arm_actions/s622_arm_actions/gripper_service.py` | 改 | 内部直发 JointTrajectory → `MoveItMotion.control_gripper`（`SetGripper.srv` 接口不变） |
| 3 | `src/s622_arm_actions/test/test_c2_servers.py` | 新增 | mock 测试：接口契约（Action/Srv 名、字段）不变 |
| 4 | `src/s622_arm_actions/s622_arm_actions/moveit_planner.py` | 归档（后续） | 两个 server 不再引用后移入 `src/archive/` + COLCON_IGNORE |

> 明确**不改**：visual_align_server / planning_scene_service / dual_move_server / s622_bt_manager / visual_servo（有独立 MoveItPlanner 副本，不受影响）/ fairino_planning_ros（在 move_group 内部，不同层）。

---

## 4. 具体改法

### 4.1 move_to_pose_server.py

```python
# 构造：替换 MoveItPlanner 部分
from manipulation_common.utils.pose_tools import PoseTools
from manipulation_common.planning.motion_executor import MoveItMotion, PlanScoreConfig
from manipulation_common.planning.trajectory_scoring import select_best_path
from pymoveit2 import MoveIt2

self.moveit2 = MoveIt2(node=self, joint_names=joint_names,
    base_link_name=base_link, end_effector_name=end_effector,
    group_name=group_name, callback_group=cb,
    move_group_namespace="/move_group_fairino")
self.moveit2.pipeline_id = "ompl"   # 对齐现有（现有 pipeline_id="fairino" 实为 OMPL 段）
self.motion = MoveItMotion(self, arm_clients={"fairino": self.moveit2},
    gripper=None, pose_tools=PoseTools(self, base_frame=base_link),
    select_best_path=select_best_path,
    score_cfg=PlanScoreConfig(num_candidates=8), action_delay=0.0)
self._vel, self._acc = self._default_v, self._default_a   # set_speed 存参数

# _execute 里：
#   set_speed(v,a)  → self._vel, self._acc = v, a
#   named_pose      → self.motion.move_to_joints(poses, action_name=f"named:{name}", timeout_sec=60.0)
#   target_pose     → Pose 构造 + self.motion.move_to_pose(pose,
#                        max_velocity=self._vel, max_acceleration=self._acc,
#                        timeout_sec=60.0, action_name="move_to_pose")
```

⚠️ **待实现时确认**：`move_to_joints` 签名无 max_velocity/max_acceleration 参数 → 速度需通过 `self.moveit2.max_velocity` 属性在调用前设置，或接受 MoveItMotion 默认值。

### 4.2 gripper_service.py

```python
# 构造：带 gripper 客户端
self.moveit2_arm = MoveIt2(node=self, joint_names=JOINT_NAMES,
    base_link_name=base_link, end_effector_name=end_effector,
    group_name="robot_arm", callback_group=cb,
    move_group_namespace="/move_group_fairino")
self.moveit2_gripper = MoveIt2(node=self,
    joint_names=["finger1_joint", "finger2_joint"],
    base_link_name=base_link, end_effector_name=end_effector,
    group_name="hand", callback_group=cb,
    move_group_namespace="/move_group_fairino",
    follow_joint_trajectory_action_name="/hand_controller/follow_joint_trajectory")
self.motion = MoveItMotion(self, arm_clients={"fairino": self.moveit2_arm},
    gripper=self.moveit2_gripper,
    open_positions=self._open, close_positions=self._close, action_delay=0.0)

# _on_set_gripper 里：
#   open  → self.motion.control_gripper(open_gripper=True, action_name="open")
#   close → self.motion.control_gripper(open_gripper=False, action_name="close")
#   保持 response 字段（success / finger_position / error_msg）不变
```

⚠️ **行为变化说明**：现有 gripper_service = "直发轨迹 + sleep 1.2s"（无规划、无执行确认）；MoveItMotion.control_gripper = "plan + execute"（走 move_group → controller，**带执行结果确认**）。这是**行为增强**，需 V2 验证确认 hand group 规划正常。

---

## 5. 逐步验证与验证标准

### V1 离线（无需仿真）

| # | 动作 | 通过标准 |
|---|---|---|
| V1.1 | `colcon build --packages-select s622_arm_actions manipulation_common` | 编译通过 |
| V1.2 | 新增 `test_c2_servers.py` 跑通 | 接口契约（Action/Srv 名、字段）不变 |
| V1.3 | `git grep MoveItPlanner`（s622_arm_actions 内） | 仅 moveit_planner.py 自身 |
| V1.4 | `git diff src/s622_bt_manager` 为空 | BT 接口层零变化 |

### V2 仿真（用户启动）

| # | 动作 | 通过标准 |
|---|---|---|
| V2.1 | 启动仿真 + arm_actions.launch.py | 两个 server 正常启动 |
| V2.2 | 发 MoveToPose（named_pose=home） | 到 home；日志 `planner_mode=ompl_global_candidate_scored` |
| V2.3 | 发 MoveToPose（target_pose 可达位姿） | 成功；日志 `selected best trajectory from N candidates` |
| V2.4 | 发 SetGripper open/close | 夹爪开合正常；response.success=true |
| V2.5 | 连续 3 次混合动作（位姿+夹爪） | 全部成功，无超时/异常 |

### V3 回归

| # | 动作 | 通过标准 |
|---|---|---|
| V3.1 | visual_servo 抓取闭环 | 正常 |
| V3.2 | RViz Plan & Execute | 正常 |
| V3.3 | manipulation_common 37 单测 | 全绿 |

---

## 6. 风险与缓解

| 风险 | 等级 | 缓解 |
|---|---|---|
| control_gripper 从"直发轨迹"变"plan+execute"，行为差异 | 中 | V2.4 专门验证夹爪；若 hand group 规划有问题，收缩范围为只改 move_to_pose_server |
| MoveItMotion 超时/等待方式与现有不同 | 低 | V2.2/2.3 逐项验证；timeout_sec=60 对齐 |
| moveit_planner.py 归档影响 visual_servo？ | 低 | visual_servo 有**独立副本**（`visual_servo/moveit_planner.py`），不 import s622 的（已确认） |
| `move_to_joints` 无速度参数 | 低 | 实现时用 `self.moveit2.max_velocity` 属性前置设置 |
| retime 依赖 `/retime_trajectory` | 低 | 已在 launch 启动（D2）；服务不可用 MoveItMotion warn 降级 |

---

## 7. 代码回滚

| 场景 | 回滚方式 |
|---|---|
| V2 仿真异常 | `git revert <commit>`（两 server 改动各自 commit，可局部回滚） |
| moveit_planner.py 归档后想恢复 | `git checkout -- src/s622_arm_actions/moveit_planner.py` |

**提交策略**（2 个 commit）：
1. `feat(c2): move_to_pose_server 改用 MoveItMotion`（含 moveit_planner.py 归档）
2. `feat(c2): gripper_service 改用 MoveItMotion.control_gripper`

---

## 8. 待确认决策点

1. **范围**：move_to_pose_server + gripper_service 一起改，还是先只 move_to_pose_server 验证通过后再 gripper？
2. **gripper 行为增强**：control_gripper 从"直发轨迹"变"plan+execute"，接受吗？
3. **不留开关**：直接替换，靠 git revert 回滚——确认？

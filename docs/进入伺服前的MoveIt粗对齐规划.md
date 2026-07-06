# Phase 6.6：进入伺服前的 MoveIt 粗对齐规划

先把这一步在整体里的角色定清楚，再给实现。

## 一、这一步到底在做什么

整体策略是"**规划做粗、伺服做精**"，分工明确：

```
trigger
  │
  ▼
[可达性预检]              IK 查一下目标到底能不能到（Phase 6.2 已做）
  │
  ▼
[PLANNING] ← 本阶段       MoveIt 规划执行到「物体上方 + 夹爪朝下 + yaw对齐」
  │                       这一步全局规划，自动避奇异、避碰撞、含完整姿态
  ▼
[APPROACHING]            切 servo，只做 3D 位置精调（消除标定残差）
  │
  ▼
[DESCENDING → GRASP → RETREAT → DONE]
```

**关键认知**：姿态（朝下 + yaw）在 PLANNING 阶段一次性摆好，靠的是 MoveIt 全局规划器——它天然会绕开奇异和碰撞。伺服阶段就**不碰姿态了**，只管位置，保留 3 维冗余。这正是上一轮定的折中方案。

## 二、最大的技术难点：servo 和 move_group 抢控制权

这是这一步唯一的坑，必须先讲清楚。

MoveIt Servo 和 move_group **都往同一个控制器**（`robot_arm_controller`）发 `JointTrajectory`。同一时刻只能有一个在写，否则指令打架，机械臂抽搐。

协调办法用 servo 自带的服务：

| 时机          | 动作                         | 谁控制机械臂           |
| ------------- | ---------------------------- | ---------------------- |
| PLANNING 开始 | 调 `/servo_node/stop_servo`  | move_group（规划执行） |
| PLANNING 结束 | 调 `/servo_node/start_servo` | servo（伺服精调）      |

**所以 servo 不能在 launch 里自动 start，要交给状态机按需切换。**

## 三、模块设计

新建 `visual_servo/moveit_planner.py`，封装 MoveIt 规划 + servo 开关。沿用 Phase 4 的 pymoveit2 经验。

```python
#!/usr/bin/env python3
"""MoveIt 粗对齐规划 + servo 开关协调。

职责:
  1. 规划执行到指定位姿（位置 + 朝下姿态 + yaw）
  2. 规划前 stop servo、规划后 start servo，避免抢控制器
不依赖 visual_servo_node 的状态机，纯执行模块。
"""
import math
from typing import Optional

import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler

from pymoveit2 import MoveIt2


class MoveItPlanner:
    def __init__(
        self,
        node: Node,
        joint_names,
        base_link: str,
        end_effector: str,
        group_name: str,
        callback_group=None,
        max_vel: float = 0.2,
        max_acc: float = 0.2,
    ):
        self.node = node
        cb = callback_group or ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=node,
            joint_names=joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name,
            callback_group=cb,
        )
        self.moveit2.max_velocity = max_vel
        self.moveit2.max_acceleration = max_acc

        # servo 开关服务客户端
        self.start_servo_cli = node.create_client(
            Trigger, "/servo_node/start_servo", callback_group=cb)
        self.stop_servo_cli = node.create_client(
            Trigger, "/servo_node/stop_servo", callback_group=cb)

    # ---------------- servo 开关 ----------------
    def _call_servo_switch(self, client, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warning(f"{name} service unavailable")
            return False
        future = client.call_async(Trigger.Request())
        # 调用方在多线程 executor 下用 spin_until_future_complete
        import rclpy
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        ok = future.result() is not None and future.result().success
        self.node.get_logger().info(f"{name}: {'ok' if ok else 'failed'}")
        return ok

    def stop_servo(self) -> bool:
        return self._call_servo_switch(self.stop_servo_cli, "stop_servo")

    def start_servo(self) -> bool:
        return self._call_servo_switch(self.start_servo_cli, "start_servo")

    # ---------------- 粗对齐规划 ----------------
    def plan_to_pregrasp(
        self,
        position: np.ndarray,
        yaw: float = 0.0,
    ) -> bool:
        """规划执行到「指定位置 + 夹爪朝下 + 指定 yaw」的位姿。

        Args:
            position: base_link 系 (x,y,z)，已含 hover 偏移。
            yaw: 绕 base z 轴偏航（对齐 OBB），弧度。
        Returns:
            规划+执行是否成功。
        过程: stop servo → MoveIt 规划执行 → start servo
        """
        # 朝下姿态：roll=π 让夹爪 z 轴朝下，叠加 yaw
        quat = quaternion_from_euler(math.pi, 0.0, yaw)

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        # 1. 让出控制权
        self.stop_servo()

        # 2. MoveIt 规划执行（全局规划，自动避奇异/碰撞）
        self.node.get_logger().info(
            f"planning to pregrasp ({pose.position.x:.3f}, "
            f"{pose.position.y:.3f}, {pose.position.z:.3f}), yaw={yaw:.2f}")
        self.moveit2.move_to_pose(
            position=[pose.position.x, pose.position.y, pose.position.z],
            quat_xyzw=[quat[0], quat[1], quat[2], quat[3]],
            cartesian=False,   # 关节空间规划，避奇异能力强
        )
        success = self.moveit2.wait_until_executed()

        # 3. 收回控制权给 servo（无论成功失败都要 start，
        #    否则后续伺服阶段拿不到控制权）
        self.start_servo()

        if not success:
            self.node.get_logger().error("pregrasp planning failed")
        return success
```

## 四、状态机集成

`ServoState` 加一个 `PLANNING`：

```python
class ServoState(Enum):
    IDLE = 0
    PLANNING = 1      # ← 新增：MoveIt 粗对齐
    APPROACHING = 2   # 3D 位置精调
    DESCENDING = 3
    GRASPING = 4
    RETREATING = 5
    DONE = 6
```

`cb_trigger` 改成进 PLANNING：

```python
def cb_trigger(self, msg: Bool):
    if msg.data:
        if self.state not in (ServoState.IDLE, ServoState.DONE):
            self.get_logger().warning(
                f"trigger ignored, state={self.state.name}")
            return

        target = self._compute_target_in_base()
        if target is None:
            self.get_logger().warning("trigger refused: no valid target")
            return

        hover_pt = target + np.array([0.0, 0.0, self.hover_z])
        if not self._check_reachable(hover_pt):
            self.get_logger().warning("trigger refused: not reachable")
            return

        # 锁定目标 + yaw，PLANNING 阶段用
        self.target_locked = target
        # yaw 从检测取（OBB angle）；先取最高置信度目标
        det = max(self.latest_det.results, key=lambda r: r.confidence)
        self.planned_yaw = float(det.angle)

        self.get_logger().info(">>> trigger: enter PLANNING")
        self._enter_state(ServoState.PLANNING)
    else:
        self.get_logger().info(">>> trigger: back to IDLE")
        self._enter_state(ServoState.IDLE)
```

PLANNING 的执行**不放在 control_loop 里**（规划是阻塞的，会卡死 50Hz 循环）。放在 `_enter_state` 触发一个独立处理，或用单独 timer 跑一次。这里给个简单做法——在 `_enter_state` 切到 PLANNING 时启动一个一次性任务：

```python
def _enter_state(self, new_state: ServoState):
    if new_state == self.state:
        return
    self.get_logger().info(f"state: {self.state.name} → {new_state.name}")
    self.state = new_state

    if new_state in (ServoState.APPROACHING, ServoState.DESCENDING,
                     ServoState.RETREATING):
        self.pd.reset()

    # PLANNING 是阻塞动作，用 one-shot timer 异步执行，
    # 避免卡住控制循环
    if new_state == ServoState.PLANNING:
        self._planning_timer = self.create_timer(
            0.01, self._do_planning_once)

def _do_planning_once(self):
    # one-shot：立刻销毁定时器，只跑一次
    self._planning_timer.cancel()
    self._planning_timer.destroy()

    hover_pt = self.target_locked + np.array([0.0, 0.0, self.hover_z])
    ok = self.planner.plan_to_pregrasp(hover_pt, yaw=self.planned_yaw)

    if ok:
        self.get_logger().info("pregrasp reached, enter APPROACHING")
        self._enter_state(ServoState.APPROACHING)
    else:
        self.get_logger().warning("planning failed, back to IDLE")
        self._enter_state(ServoState.IDLE)
```

control_loop 在 PLANNING 状态下只发零 twist（servo 此时已 stop，发了也没人收，但保持心跳习惯）：

```python
def control_loop(self):
    twist = self._zero_twist()

    if self.state == ServoState.APPROACHING:
        twist = self._servo_step(self.hover_z, self._on_approach_done)
    elif self.state == ServoState.DESCENDING:
        twist = self._servo_step(self.descend_z,
                                 lambda: self._enter_state(ServoState.GRASPING))
    elif self.state == ServoState.GRASPING:
        self._handle_grasping()
    elif self.state == ServoState.RETREATING:
        twist = self._retreat_step()
    # PLANNING / IDLE / DONE：零 twist

    self.twist_pub.publish(twist)
```

## 五、`__init__` 里初始化 planner

```python
from visual_servo.moveit_planner import MoveItPlanner

# 在 __init__ 里，cb_group 创建之后
self.declare_parameter("joint_names",
                       ["j1", "j2", "j3", "j4", "j5", "j6"])  # 按 SRDF 改
joint_names = self.get_parameter("joint_names").value

self.planner = MoveItPlanner(
    node=self,
    joint_names=joint_names,
    base_link=self.base_frame,
    end_effector=self.ee_frame,
    group_name=self.move_group_name,
    callback_group=self.cb_group,
)
self.planned_yaw = 0.0
```

## 六、launch 改动：servo 不要自动 start

如果你之前在 launch 里加了自动 `start_servo`（方式 B），**删掉**。现在 servo 由状态机控制开关，启动时应该是 stop 状态。

## 七、验证步骤

```bash
colcon build --merge-install --symlink-install --packages-select visual_servo
source install/setup.bash
```

**步 1：单独测 planner 不接伺服**

先确认"stop servo → MoveIt 规划 → start servo"这条链通：

```bash
ros2 run visual_servo visual_servo_node --ros-args \
  -p enable_motion:=true \
  -p debug_target:=[0.0,-0.4,0.2]

# 或者
ros2 run visual_servo visual_servo_node --ros-args \
  -p enable_motion:=true \
  -p enable_grasp_sequence:=true \
  -p debug_target:=[0.0,-0.4,0.2]

# 启动上面这个结点之后要等一会， 3s ~ 5s
测试用数据
# 远点
debug_target:='[0.5, 0.0, 0.3]'
# 高点
debug_target:='[0.3, 0.0, 0.5]'
# 侧远点
debug_target:='[0.0, -0.45, 0.3]'
# 近点
debug_target:='[0.3, 0.0, 0.15]'


ros2 run yolov8_obb yolov8_obb_node
ros2 topic pub --once /servo_trigger std_msgs/msg/Bool "{data: true}"
```

期望日志：

```
>>> trigger: enter PLANNING
stop_servo: ok
planning to pregrasp (...), yaw=...
[MoveIt 规划执行]
start_servo: ok
pregrasp reached, enter APPROACHING
```

观察机械臂：先用 MoveIt **平滑地**飞到物体上方、夹爪朝下，然后切伺服微调。

**步 2：重点看夹爪姿态**

到达 pregrasp 后，RViz 里确认夹爪 z 轴朝下。这一步通了，**问题一就解决了**。

## 八、易踩坑

| 现象                             | 原因                                                  |
| -------------------------------- | ----------------------------------------------------- |
| `stop_servo service unavailable` | servo_node 没起，检查 launch                          |
| 规划成功但机械臂不动             | servo 没 stop 干净，move_group 和 servo 抢控制器      |
| 切 APPROACHING 后机械臂不微调    | start_servo 没成功，servo 还是 stop 状态              |
| PLANNING 卡死整个节点            | 规划放进了 control_loop（必须用 one-shot timer 异步） |
| 规划失败 INVALID_GROUP           | joint_names/group_name 跟 SRDF 不一致                 |

## 九、这一步完成后

* 问题一（夹爪朝下）解决
* 伺服阶段简化为纯 3D 位置，留 3 维冗余
* 为 Phase 6.7（null-space）铺好路：3 维冗余足够做 null-space 避奇异

**下一步**是 Phase 6.8（奇异减速，简单兜底）还是 Phase 6.7（null-space）？按之前讨论，建议先 6.8。不过先把这个 6.6 跑通，确认夹爪朝下 + servo/moveit 切换顺畅，再往下走。

要我把这一步的所有改动整理成一个**完整的 visual_servo_node.py**给你，还是你照着片段自己集成？片段集成能让你更清楚每块的位置，但完整文件不容易出错，你定。

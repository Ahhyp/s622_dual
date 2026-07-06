# Phase 6 全景 + 接下来怎么走

恭喜，Phase 6.1 + 6.2 都过了，**视觉伺服闭环正式打通**。这是你整个项目里最难的一关，过了之后剩下的都是往骨架上加肉。

## 一、Phase 6 全景

完整的视觉伺服阶段我把它拆成 5 个子阶段，按难度递增：

| 阶段    | 内容                                                      | 状态     |
| ------- | --------------------------------------------------------- | -------- |
| **6.1** | 骨架：状态机 + PD + 误差估计 + zero twist                 | ✅ 完成   |
| **6.2** | 真伺服：`enable_motion=true`，机械臂跟着假目标走到 hover  | ✅ 完成   |
| **6.3** | 完整闭环抓取：APPROACH → DESCEND → GRASP → RETREAT → DONE | ⏭️ 下一步 |
| **6.4** | yaw 通道：让夹爪也对齐 OBB 角度（angular.z）              | 待做     |
| **6.5** | 鲁棒性：丢失目标超时、动态目标跟踪、安全停机              | 待做     |

后面还有 Phase 7（接真 YOLO 模型）、Phase 8（端到端 demo），但那是 Phase 6 完了之后的事。

## 二、Phase 6.3：你接下来要做什么

### 1. 目标

把现有的"伺服到 hover 点然后停"扩展成完整抓取流程：

```
IDLE
 │ trigger
 ▼
APPROACHING       伺服到物体上方 hover_z=10cm 处
 │ |e| < 5mm
 ▼
DESCENDING        hover_z 改为 0，继续伺服往下扎
 │ |e| < 5mm
 ▼
GRASPING          停 servo（发零 twist），调夹爪闭合，等 1.5s
 │ done
 ▼
RETREATING        hover_z 改为 10cm，伺服往上抬
 │ |e| < 5mm
 ▼
DONE              发零 twist 待命
```

### 2. 关键设计点

**状态切换判据**：只用"位置收敛"一个条件，简单可靠。

**hover_z 是动态的**：不再写死 0.10，而是根据状态切换。我建议在节点里维护 `self.current_hover_z`，每次切状态就更新。

**GRASPING 不用 servo**：servo 是连续运动接口，不适合"停下做动作"。这一步直接发零 twist 保持机械臂不动，然后用 publisher 发夹爪指令（沿用 Phase 4 的 `JointTrajectory` 接口）。

**RETREATING 别用 DESCENDING 的位置**：要回到 APPROACHING 时记下来的 hover 点，不是物体当前位置（万一夹起来后物体跟着动了呢）。

### 3. 需要新增的字段

`VisualServoNode.__init__` 里新增：

```python
# 夹爪发布（沿用 arm_executor 的接口，但这里不引 arm_executor 包）
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
self.gripper_pub = self.create_publisher(
    JointTrajectory, "/hand_controller/joint_trajectory", 10)

# 状态相关
self.current_hover_z = self.hover_z  # 当前期望 z 偏移
self.grasp_start_time = None         # GRASPING 开始时间
self.retreat_target = None           # 退回点缓存
self.declare_parameter("descend_offset_z", 0.0)  # 下扎多深
self.declare_parameter("grasp_duration", 1.5)    # 夹爪闭合等待
```

`ServoState` 枚举扩展：

```python
class ServoState(Enum):
    IDLE = 0
    APPROACHING = 1
    DESCENDING = 2
    GRASPING = 3
    RETREATING = 4
    DONE = 5
```

### 4. control_loop 改造思路

把现在 SERVOING 那一坨改成 dispatch：

```python
def control_loop(self):
    twist = self._zero_twist()
    
    if self.state == ServoState.APPROACHING:
        twist = self._servo_step(target_offset_z=self.hover_z,
                                 next_state=ServoState.DESCENDING)
    elif self.state == ServoState.DESCENDING:
        twist = self._servo_step(target_offset_z=self.descend_z,
                                 next_state=ServoState.GRASPING)
    elif self.state == ServoState.GRASPING:
        self._handle_grasping()  # 内部计时 + 切状态
    elif self.state == ServoState.RETREATING:
        twist = self._retreat_step(...)
    
    self.twist_pub.publish(twist)
```

`_servo_step` 是把现在 SERVOING 的逻辑抽出来的通用伺服函数，参数化"目标偏移"和"收敛后下个状态"。

## 三、做完整流程之前先验证 6.3a 子任务

不要一口气写完整状态机，分两步：

### 6.3a：先加 DESCENDING

只加一个状态，验证"hover_z 变化时机械臂会不会重新下扎"。

最简改法：在 `cb_trigger` 里加个开关，或者临时硬编码：APPROACHING 收敛后 → DESCENDING（hover_z=0）→ DONE。**不接夹爪，不退回**。

通过标准：机械臂走到 hover → 继续往下扎到物体表面 → 停。

### 6.3b：加夹爪 + 退回

6.3a 通过后再加 GRASPING 和 RETREATING。

通过标准：完整 5 状态走完，机械臂回到 hover 点，夹爪闭合状态。

**两段验证比一段验证省一倍调试时间**，因为出问题时知道是哪一段引入的。

## 四、需要改到 obb_node 假数据的地方

为了让 6.3 验证有意义，最好让假目标的深度**接近真实物体**：

* 当前你 obb_node 应该是发 (320, 240) 像素，深度图给的是仿真里某个固定值
* DESCENDING 时机械臂会往那个深度扎
* **确保深度对应的物体位置在机械臂工作空间内**，否则 IK 解不出来 servo 报奇异

简单做法：在仿真里 spawn 一个 0.5m 高的方块在机械臂前方 (0.4, 0, 0.5)，相机正对着，假检测的像素就是块的中心。

## 五、安全提醒

DESCENDING 是这次 Phase 里**最危险**的状态：机械臂会主动往下扎。建议：

1. `descend_offset_z` 设保守一点，比如 `0.02`（离物体表面还差 2cm 就停），不真扎到物体
2. 第一次跑把 `max_linear_vel` 降到 `0.05`
3. RViz 盯着看，不对立刻 trigger=false 切 IDLE

## 六、要我直接给代码吗

如果你对状态机扩展有把握，可以自己照思路改。如果想要现成代码，告诉我两件事我直接出 6.3a 的版本：

1. **descend 多深？** 物体表面 (`descend_offset_z=0`) 还是表面上方 2cm (`descend_offset_z=0.02`)？
2. **当前 obb_node 假目标对应的深度**是多少？（`ros2 topic echo /camera/depth/image_raw` 看一下 (240, 320) 那个像素附近的值）

知道这两个数才能把参数默认值给得合理。


# Phase 6.3：完整闭环抓取实现


按上面说的设计来。给你**完整版代码**，但用 `enable_grasp_sequence` 参数控制是否走完整流程：

* 默认 `False`：行为和 Phase 6.2 一样，APPROACHING 收敛就 DONE（**先用这个回归测试，确保 6.2 没破坏**）
* 改 `True`：走完整 5 状态流程

## 一、`pd_controller.py` 和 `error_estimator.py` 不动

继续用 Phase 6.1 的版本。

## 二、`visual_servo_node.py` 完整替换

文件较长，分两段贴。**两段拼一起就是完整文件**。

### 第 1 段（imports + 类定义 + **init** + 回调）

```python
#!/usr/bin/env python3
"""视觉伺服节点（Phase 6.3）

完整闭环抓取状态机:
  IDLE → APPROACHING → DESCENDING → GRASPING → RETREATING → DONE

兼容开关:
  enable_grasp_sequence=False (默认): APPROACHING 收敛即 DONE，等于 Phase 6.2
  enable_grasp_sequence=True:         走完整 5 状态
"""
import math
from enum import Enum
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped, PointStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from yolov8_obb_msgs.msg import Yolov8Inference

from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

from visual_servo.pd_controller import PDController
from visual_servo.error_estimator import ErrorEstimator


class ServoState(Enum):
    IDLE = 0
    APPROACHING = 1   # 伺服到物体上方 hover_z
    DESCENDING = 2    # 伺服到物体上方 descend_z（接近物体）
    GRASPING = 3      # 停 servo，闭合夹爪
    RETREATING = 4    # 伺服回 hover 点
    DONE = 5


class VisualServoNode(Node):
    def __init__(self):
        super().__init__("visual_servo_node")

        # ---------- 控制参数 ----------
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("kd", 0.05)
        self.declare_parameter("max_linear_vel", 0.10)
        self.declare_parameter("position_tolerance", 0.005)

        # ---------- 抓取参数 ----------
        # APPROACHING 阶段悬停高度（物体上方 10cm）
        self.declare_parameter("hover_offset_z", 0.10)
        # DESCENDING 阶段下扎到物体上方多高（2cm 留余量，不真撞）
        self.declare_parameter("descend_offset_z", 0.02)
        # GRASPING 等夹爪闭合的时间（秒）
        self.declare_parameter("grasp_duration", 1.5)
        # 是否走完整抓取流程（False 时退化为 Phase 6.2 行为）
        self.declare_parameter("enable_grasp_sequence", False)

        # ---------- 坐标系 ----------
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("ee_frame", "tool0")

        # ---------- 安全 ----------
        self.declare_parameter("enable_motion", False)

        # ---------- 夹爪 ----------
        self.declare_parameter("gripper_topic",
                               "/hand_controller/joint_trajectory")
        self.declare_parameter("gripper_joint_names",
                               ["finger1_joint", "finger2_joint"])
        self.declare_parameter("gripper_open_pos", [0.025, -0.025])
        self.declare_parameter("gripper_close_pos", [0.0, 0.0])

        # ---------- 读参数 ----------
        self.control_rate = float(self.get_parameter("control_rate").value)
        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        max_v = float(self.get_parameter("max_linear_vel").value)
        self.tol = float(self.get_parameter("position_tolerance").value)
        self.hover_z = float(self.get_parameter("hover_offset_z").value)
        self.descend_z = float(self.get_parameter("descend_offset_z").value)
        self.grasp_duration = float(self.get_parameter("grasp_duration").value)
        self.enable_grasp_seq = bool(
            self.get_parameter("enable_grasp_sequence").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.enable_motion = bool(self.get_parameter("enable_motion").value)

        gripper_topic = self.get_parameter("gripper_topic").value
        self.gripper_joint_names = list(
            self.get_parameter("gripper_joint_names").value)
        self.gripper_open_pos = list(
            self.get_parameter("gripper_open_pos").value)
        self.gripper_close_pos = list(
            self.get_parameter("gripper_close_pos").value)

        # ---------- 模块 ----------
        self.pd = PDController(kp=kp, kd=kd, max_output=max_v)
        self.estimator = ErrorEstimator()
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- 缓存 ----------
        self.depth_img: Optional[np.ndarray] = None
        self.depth_stamp = None
        self.latest_det: Optional[Yolov8Inference] = None

        # ---------- 状态机变量 ----------
        self.state = ServoState.IDLE
        # GRASPING 开始时刻（秒），用于判定夹爪是否到位
        self.grasp_start_time: Optional[float] = None
        # APPROACHING 收敛时记下来的 base 系物体位置 (np.array shape=(3,))
        # RETREATING 用它算退回点，避免抓起来后物体动了带偏机械臂
        self.target_locked: Optional[np.ndarray] = None

        # ---------- 订阅 ----------
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.cb_info, 10)
        self.create_subscription(
            Image, "/camera/depth/image_raw", self.cb_depth, 10)
        self.create_subscription(
            Yolov8Inference, "/yolov8/obb_detections", self.cb_det, 10)
        self.create_subscription(
            Bool, "/servo_trigger", self.cb_trigger, 10)

        # ---------- 发布 ----------
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, gripper_topic, 10)

        # ---------- 控制循环 ----------
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            f"visual_servo started | rate={self.control_rate}Hz | "
            f"kp={kp} kd={kd} max_v={max_v} | motion={self.enable_motion} | "
            f"grasp_seq={self.enable_grasp_seq}"
        )

    # ==================================================================
    # 回调
    # ==================================================================
    def cb_info(self, msg: CameraInfo):
        self.estimator.set_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_stamp = msg.header.stamp

    def cb_det(self, msg: Yolov8Inference):
        self.latest_det = msg

    def cb_trigger(self, msg: Bool):
        if msg.data:
            if self.state in (ServoState.IDLE, ServoState.DONE):
                self.get_logger().info(">>> trigger: enter APPROACHING")
                self._enter_state(ServoState.APPROACHING)
        else:
            self.get_logger().info(">>> trigger: back to IDLE")
            self._enter_state(ServoState.IDLE)
```

### 第 2 段（control_loop + 状态处理 + main）

```python
    # ==================================================================
    # 状态切换辅助
    # ==================================================================
    def _enter_state(self, new_state: ServoState):
        """统一的状态切换入口，做必要的清理工作"""
        if new_state == self.state:
            return
        self.get_logger().info(
            f"state: {self.state.name} → {new_state.name}")
        self.state = new_state

        # 切到任何伺服状态都要 reset PD，避免微分项乱跳
        if new_state in (ServoState.APPROACHING,
                         ServoState.DESCENDING,
                         ServoState.RETREATING):
            self.pd.reset()

        if new_state == ServoState.GRASPING:
            self.grasp_start_time = self._now_sec()
        else:
            self.grasp_start_time = None

        if new_state == ServoState.IDLE:
            self.target_locked = None

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _zero_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        return t

    # ==================================================================
    # 控制循环
    # ==================================================================
    def control_loop(self):
        # 默认零 twist 心跳。即便没在伺服也要持续发，
        # 否则 servo_node 0.5s 后会自动停（incoming_command_timeout）
        twist = self._zero_twist()

        if self.state == ServoState.APPROACHING:
            twist = self._servo_step(
                target_offset_z=self.hover_z,
                on_converge=self._on_approach_done,
            )
        elif self.state == ServoState.DESCENDING:
            twist = self._servo_step(
                target_offset_z=self.descend_z,
                on_converge=lambda: self._enter_state(ServoState.GRASPING),
            )
        elif self.state == ServoState.GRASPING:
            self._handle_grasping()
            # twist 保持零，机械臂悬停
        elif self.state == ServoState.RETREATING:
            twist = self._retreat_step()

        self.twist_pub.publish(twist)

    # ==================================================================
    # 各状态行为
    # ==================================================================
    def _servo_step(self, target_offset_z: float,
                    on_converge) -> TwistStamped:
        """通用伺服步骤：算误差 → PD → twist。
        target_offset_z: 期望停在物体上方多高
        on_converge: 收敛时调用的回调（无参）
        """
        twist = self._zero_twist()
        error = self._compute_error_in_base(z_offset=target_offset_z)
        if error is None:
            return twist

        err_norm = float(np.linalg.norm(error))
        if err_norm < self.tol:
            self.get_logger().info(
                f"converged in {self.state.name}, |e|={err_norm*1000:.1f}mm")
            on_converge()
            return twist

        v = self.pd.compute(error, self._now_sec())

        # 节流日志（5Hz）
        now = self._now_sec()
        if int(now * self.control_rate) % 10 == 0:
            self.get_logger().info(
                f"[{self.state.name}] |e|={err_norm*1000:.1f}mm  "
                f"v=({v[0]:+.3f},{v[1]:+.3f},{v[2]:+.3f}) m/s"
            )

        if self.enable_motion:
            twist.twist.linear.x = float(v[0])
            twist.twist.linear.y = float(v[1])
            twist.twist.linear.z = float(v[2])
        return twist

    def _on_approach_done(self):
        """APPROACHING 收敛回调：锁定目标位置 + 切下一状态"""
        # 锁住此刻的物体位置，RETREATING 会用到
        self.target_locked = self._compute_target_in_base()

        if not self.enable_grasp_seq:
            # 兼容 Phase 6.2 行为
            self._enter_state(ServoState.DONE)
        else:
            # 进入完整抓取流程，先张开夹爪
            self._send_gripper(self.gripper_open_pos)
            self._enter_state(ServoState.DESCENDING)

    def _handle_grasping(self):
        """GRASPING：发夹爪闭合，等到位后切 RETREATING"""
        if self.grasp_start_time is None:
            return
        elapsed = self._now_sec() - self.grasp_start_time

        # 进入状态后立刻发一次闭合指令（不重复发）
        if elapsed < 0.05:
            self._send_gripper(self.gripper_close_pos)
            self.get_logger().info("gripper close cmd sent")

        if elapsed >= self.grasp_duration:
            self.get_logger().info("gripper closed, retreating")
            self._enter_state(ServoState.RETREATING)

    def _retreat_step(self) -> TwistStamped:
        """RETREATING：伺服回锁定的目标位置上方 hover_z 处。
        用 target_locked 而不是当前检测，避免抓起后物体跟着动带偏。
        """
        twist = self._zero_twist()
        if self.target_locked is None:
            self.get_logger().warning("no target_locked, abort retreat")
            self._enter_state(ServoState.DONE)
            return twist

        # 退回点 = 锁定位置 + hover_z
        retreat_pt = self.target_locked + np.array([0.0, 0.0, self.hover_z])

        ee = self._lookup_ee_in_base()
        if ee is None:
            return twist
        error = retreat_pt - ee
        err_norm = float(np.linalg.norm(error))

        if err_norm < self.tol:
            self.get_logger().info(
                f"retreat converged, |e|={err_norm*1000:.1f}mm → DONE")
            self._enter_state(ServoState.DONE)
            return twist

        v = self.pd.compute(error, self._now_sec())
        now = self._now_sec()
        if int(now * self.control_rate) % 10 == 0:
            self.get_logger().info(
                f"[RETREATING] |e|={err_norm*1000:.1f}mm  "
                f"v=({v[0]:+.3f},{v[1]:+.3f},{v[2]:+.3f}) m/s"
            )
        if self.enable_motion:
            twist.twist.linear.x = float(v[0])
            twist.twist.linear.y = float(v[1])
            twist.twist.linear.z = float(v[2])
        return twist

    # ==================================================================
    # 辅助：误差计算 / TF / 夹爪
    # ==================================================================
    def _compute_target_in_base(self) -> Optional[np.ndarray]:
        """像素 → 相机系 → base_link，返回目标物体位置（不含 hover_z）"""
        if self.depth_img is None:
            return None
        if self.latest_det is None or len(self.latest_det.results) == 0:
            return None
        if not self.estimator.has_intrinsics():
            return None

        target = max(self.latest_det.results, key=lambda r: r.confidence)
        xyz_cam = self.estimator.pixel_to_camera(
            target.center_x, target.center_y, self.depth_img)
        if xyz_cam is None:
            return None

        pt = PointStamped()
        pt.header.frame_id = self.camera_frame
        pt.header.stamp = self.depth_stamp
        pt.point.x, pt.point.y, pt.point.z = (
            float(xyz_cam[0]), float(xyz_cam[1]), float(xyz_cam[2]))
        try:
            pt_base = self.tf_buffer.transform(
                pt, self.base_frame, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warning(
                f"TF target→base failed: {e}", throttle_duration_sec=2.0)
            return None
        return np.array(
            [pt_base.point.x, pt_base.point.y, pt_base.point.z])

    def _lookup_ee_in_base(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warning(
                f"TF base→ee failed: {e}", throttle_duration_sec=2.0)
            return None
        return np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ])

    def _compute_error_in_base(self,
                               z_offset: float) -> Optional[np.ndarray]:
        """误差 = (物体位置 + z_offset) - 末端位置"""
        target = self._compute_target_in_base()
        if target is None:
            return None
        ee = self._lookup_ee_in_base()
        if ee is None:
            return None
        goal = target + np.array([0.0, 0.0, z_offset])
        return goal - ee

    def _send_gripper(self, positions):
        """发一个 1s 走到位的 trajectory 点"""
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start.sec = 1
        msg.points.append(pt)
        self.gripper_pub.publish(msg)


def main():
    rclpy.init()
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```


### 夹爪要是非垂直向下姿态而结束 approching ， 怎么办？

因此在这里，需要做出对任务的调整！

[进入伺服前的 MoveIt 粗对齐规划](./进入伺服前的MoveIt粗对齐规划.md)


### 频繁奇异值怎么办？ 躲避奇异值

数学上：
冗余自由度 = 关节数 - 任务维度

当前的任务维度是 3维， 源项目是二位。

|          | 你的实现                 | 源项目 servo 阶段                           |
| -------- | ------------------------ | ------------------------------------------- |
| 控制维度 | 3D 位置 (x, y, z 全伺服) | 2D 平面 + Z 锁定（XY 伺服，Z 保持当前高度） |
| 姿态控制 | 无                       | yaw 独立伺服                                |
| 误差来源 | debug_target / 像素→3D   | 检测框 → base_link 3D 点                    |

源项目的 servo 只追 XY 平面、不追 Z——高度靠 MoveIt 规划器先飞到目标上方，servo 只管水平对准。这样做有三个好处：
更安全：高度不变，不会 servo 着就撞桌子
更少奇异：少一个自由度，Jacobian 冗余度更大
更容易收敛：2D 误差比 3D 误差收敛快
你的代码追了 xyz 三个方向的误差，手臂多一个维度被"拉"，更容易偏到奇异方向。改成 2D 平面伺服是立竿见影的改进。



#### 方案1： null-space 避奇异值
这个方法要求任务有冗余自由度， 我的是3D，源项目是2D， 机械臂有六个关节，都是由冗余的，可以用这个方法，但是如果改造成 6D 的就不能用这个方法了！ 这个优化方法优先级放在后面。

[null-space_奇异值感知减速_夹爪]()

#### 方案2: 目标可达性预检 
这个方案就是工业系统的标配。MoveIt 现成的 IK 服务能直接用。
实现方式
在 cb_trigger 里收到 trigger 之后、切到 APPROACHING 之前，调用 /compute_ik 服务:
```python
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

# __init__ 里
self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

def _check_reachable(self, target_pose_in_base) -> bool:
    """用 MoveIt IK 服务预检目标是否可达 + 有解"""
    if not self.ik_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().warning("IK service unavailable, skip check")
        return True   # 服务不在就跳过检查（不要因为服务挂了拒绝抓取）

    req = GetPositionIK.Request()
    req.ik_request.group_name = "robot_arm"
    req.ik_request.pose_stamped.header.frame_id = self.base_frame
    req.ik_request.pose_stamped.pose = target_pose_in_base
    req.ik_request.timeout.sec = 0
    req.ik_request.timeout.nanosec = int(0.1 * 1e9)
    req.ik_request.avoid_collisions = True

    future = self.ik_client.call_async(req)
    rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
    
    if future.result() is None:
        return False
    return future.result().error_code.val == 1  # SUCCESS
```
注意事项
- **检查 hover 点而不是物体本身**：物体高度可能在 servo 工作空间内但 hover 点（+10cm）不在。检查最远那个点最严格。
- **同时检查 manipulability**：IK 有解不代表姿态好。可以解出关节角后算一下雅可比条件数，太差也拒绝。
- **失败处理要明确**：拒绝后机械臂应该回什么状态？建议直接回 IDLE 并打印 "target not reachable, please reposition"，让上层重新决策。
- **这里与代码真正的实现出入比较大**
设计：
| 设计点                                 | 原因                                                 |
| -------------------------------------- | ---------------------------------------------------- |
| 多线程 executor                        | 服务调用阻塞 + 单线程 spin = 死锁                    |
| 检查 hover 点不是物体本身              | hover z 更高，离 base 更远，更容易越界               |
| 服务不在时 return True                 | 基础设施问题不应该卡住抓取流程                       |
| 状态过滤只接受 IDLE/DONE               | 防止伺服中途收到 trigger 重启混乱                    |
| `pose_stamped.orientation = (1,0,0,0)` | roll=π 的俯视姿态固定值；姿态不影响位置可达性判断    |
| timeout 100ms                          | IK 用 KDL/TRAC-IK 都很快，100ms 够；太长会卡触发响应 |


#### 方案3： 奇异感知减速


## 三、验证步骤（**严格按顺序**）

### 步 1：回归测试 6.2 行为

不开 grasp_sequence，确认改动没破坏 Phase 6.2：

```bash
colcon build --merge-install --symlink-install --packages-select visual_servo
source install/setup.bash

ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=true \
  -p debug_target:=[0.0,-0.4,0.2]

# 后面这个 debug_target 完全是为了不靠近奇异值，让机械臂够得着。
# 默认 enable_grasp_sequence=false

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {} && ros2 topic pub --once /servo_trigger std_msgs/msg/Bool "{data: true}"
```

```bash
# P，而不是PD， 并且调小 kp
ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=true \
  -p kd:=0.0 \
  -p kp:=0.3 \
  -p max_linear_vel:=0.05 \
  -p debug_target:=[0.0,-0.4,0.3]
```

```bash
# 测试目标原理工作空间时， IK 检测 能不能检测到。
ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=true \
  -p kd:=0.0 \
  -p kp:=0.3 \
  -p max_linear_vel:=0.05 \
  -p debug_target:=[0.0,-0.4,2.0]
```

期望：和昨天一样，到 hover 点停 → DONE。看到的状态切换日志应该是 `IDLE → APPROACHING → DONE`。

**这步不通过，先不要往下走**。

### 步 2：6.3a — 打开 grasp_sequence，但**先不接夹爪**

把 `grasp_duration` 设成 0.1 让 GRASPING 一闪而过，重点看 DESCENDING 和 RETREATING：

```bash
ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=true \
  -p enable_grasp_sequence:=true \
  -p grasp_duration:=0.1 \
  -p debug_target:=[0.25,0.0,0.4]

# 或者线上切到完整抓取模式，不重启节点
ros2 param set /visual_servo_node enable_grasp_sequence true
ros2 param set /visual_servo_node grasp_duration 0.1

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {} && ros2 topic pub --once /servo_trigger std_msgs/msg/Bool "{data: true}"

```

期望状态切换：

```
IDLE → APPROACHING → DESCENDING → GRASPING → RETREATING → DONE
```

观察机械臂动作：先到 hover → 往下扎到 hover-8cm（hover_z=0.10, descend_z=0.02 差 8cm）→ 短暂停顿 → 抬回 hover → 停。

**容易出问题的点**：

* **DESCENDING 撞到桌面**：descend_offset_z 太小。Gazebo 仿真里桌面会反作用，机械臂震荡。调大 `descend_offset_z` 到 0.05。
* **RETREATING 飞错方向**：`target_locked` 没拿到（深度无效或 TF 失败）。日志会刷 warning。
* **状态卡在 DESCENDING**：物体位置在工作空间外，机械臂够不到。给 obb 假目标改一下像素，让对应深度合理。

### 步 3：6.3b — 加正常的 grasp_duration

```bash
ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=true \
  -p enable_grasp_sequence:=true
# grasp_duration 用默认 1.5s
```

观察 GRASPING 阶段夹爪是不是真的合上了。可以用 RViz 显示模型，或：

```bash
ros2 topic echo /joint_states | grep -A 2 finger
```







### 验证

## 四、几个调参经验值

| 参数                 | 默认        | 调整建议                              |
| -------------------- | ----------- | ------------------------------------- |
| `descend_offset_z`   | 0.02        | 仿真物体高度 ≥ 5cm 时，先用 0.05 安全 |
| `grasp_duration`     | 1.5         | 真实硬件可能要 2~3s                   |
| `position_tolerance` | 0.005 (5mm) | 太紧机械臂在阈值附近抖；太松抓不准    |
| `kp`                 | 0.8         | 走得慢就加，超调就减                  |
| `max_linear_vel`     | 0.10        | 第一次跑改成 0.05 更安全              |

## 五、下一步

Phase 6.3 通过后（5 状态完整跑下来，机械臂带着夹爪闭合状态退回 hover），你就完成了**端到端的视觉伺服抓取**。

继续往下选：

* **Phase 6.4**：加 yaw 通道，让夹爪对齐 OBB 角度（angular.z）
* **Phase 6.5**：鲁棒性，丢失目标超时、安全停机
* **Phase 7**：接真 YOLO 模型，丢掉假数据

我建议**先做 Phase 7**，因为前面所有调试都是在假数据上做的，接真模型会暴露很多新问题（检测抖动、丢帧、角度噪声），那时候再做 6.4 / 6.5 才有真实场景驱动调参。

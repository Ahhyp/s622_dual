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

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


from visual_servo.moveit_planner import MoveItPlanner
from sensor_msgs.msg import JointState

# --------------------------------------------------------------------
# 状态枚举
# --------------------------------------------------------------------
class ServoState(Enum):
    IDLE = 0
    PLANNING = 1      # ← 新增：MoveIt 粗对齐
    APPROACHING = 2   # 伺服到物体上方 hover_z
    DESCENDING = 3    # 伺服到物体上方 descend_z（接近物体）
    GRASPING = 4      # 停 servo，闭合夹爪
    RETREATING = 5    # 伺服回 hover 点
    DONE = 6


# --------------------------------------------------------------------
# 节点
# --------------------------------------------------------------------
class VisualServoNode(Node):
    def __init__(self):
        super().__init__("visual_servo_node")

        # ---------- 控制参数 ----------
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("kd", 0.05)
        self.declare_parameter("max_linear_vel", 0.10)
        self.declare_parameter("position_tolerance", 0.005)
        self.declare_parameter("avoid_collisions", True)
        
        # ---------- 抓取参数 ----------
        # APPROACHING 阶段悬停高度（物体上方 10cm）
        self.declare_parameter("hover_offset_z", 0.10)
        # DESCENDING 阶段下扎到物体上方多高（2cm 留余量，不真撞）
        self.declare_parameter("descend_offset_z", 0.02)
        # GRASPING 等夹爪闭合的时间（秒）
        self.declare_parameter("grasp_duration", 5)
        # 是否走完整抓取流程（False 时退化为 Phase 6.2 行为）
        self.declare_parameter("enable_grasp_sequence", False)

        # ---------- 坐标系 ----------
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("ee_frame", "grasp_frame")

        # ---------- 安全 ----------
        self.declare_parameter("enable_motion", False)

        # ---------- 调试：跳过像素→3D，用 base_link 硬编码目标 ----------
        # 参数值 [x, y, z] 米，如 [0.4, 0.0, 0.3]。数组为空则走正常管线
        self.declare_parameter("debug_target", [0.0, 0.0, 0.0])

        # ---------- 夹爪 ----------
        self.declare_parameter("gripper_topic",
                               "/hand_controller/joint_trajectory")
        self.declare_parameter("gripper_joint_names",
                               ["finger1_joint", "finger2_joint"])
        self.declare_parameter("gripper_open_pos", [0.025, -0.025])
        self.declare_parameter("gripper_close_pos", [0.0, 0.0])

        # 如果 YOLO 很久没更新，visual_servo_node 仍然可能使用老目标 添加超时保护        
        # 检测消息超过多久算过期
        self.declare_parameter("det_timeout", 2.0)
        self.det_timeout = float(self.get_parameter("det_timeout").value)
        self.latest_det_time = None

        # -------------调试信息------------------------
        self.declare_parameter("debug_print_target", True)
        self.declare_parameter("debug_print_ik", True)
        
        # ---------- 目标管理策略 ----------
        # locked: trigger 后锁定目标，执行阶段一直用锁定目标。推荐当前静态红盒子使用。
        # realtime: 每次都用实时检测。目标一遮挡就停，不推荐当前场景。
        # hybrid: 可见时可小范围刷新，遮挡时用 locked。
        self.declare_parameter("target_policy", "locked")
        # APPROACHING 收敛时是否尝试刷新 target_locked
        # 当前遮挡问题下，建议默认 False，避免把有效 locked target 覆盖掉。
        self.declare_parameter("refresh_target_at_approach_done", False)
        # 如果允许刷新，刷新后的目标与旧目标距离不能超过这个阈值，单位 m
        self.declare_parameter("target_refresh_max_jump", 0.05)

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
        self.debug_print_target = bool(
            self.get_parameter("debug_print_target").value)
        self.debug_print_ik = bool(
            self.get_parameter("debug_print_ik").value)
        self.det_timeout = float(
            self.get_parameter("det_timeout").value)

        gripper_topic = self.get_parameter("gripper_topic").value
        self.gripper_joint_names = list(
            self.get_parameter("gripper_joint_names").value)
        self.gripper_open_pos = list(
            self.get_parameter("gripper_open_pos").value)
        self.gripper_close_pos = list(
            self.get_parameter("gripper_close_pos").value)

        self.avoid_collisions = bool(
            self.get_parameter("avoid_collisions").value)
        
        self.target_policy = str(self.get_parameter("target_policy").value)
        self.det_timeout = float(self.get_parameter("det_timeout").value)
        self.refresh_target_at_approach_done = bool(
            self.get_parameter("refresh_target_at_approach_done").value)
        self.target_refresh_max_jump = float(
            self.get_parameter("target_refresh_max_jump").value)

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
        self.latest_joint_state: Optional[JointState] = None
        self.latest_det_time: Optional[float] = None
        # 最近一次“真实可见”的目标位置
        self.last_visible_target: Optional[np.ndarray] = None
        self.last_visible_target_time: Optional[float] = None
        
        # ---------- 状态机变量 ----------
        self.state = ServoState.IDLE
        # GRASPING 开始时刻（秒），用于判定夹爪是否到位
        self.grasp_start_time: Optional[float] = None
        # APPROACHING 收敛时记下来的 base 系物体位置 (np.array shape=(3,))
        # RETREATING 用它算退回点，避免抓起来后物体动了带偏机械臂
        self.target_locked: Optional[np.ndarray] = None

        
        # ------------ IK 预检测 --------------------
        # 用 SRDF 里的 group 名
        self.declare_parameter("move_group_name", "robot_arm")
        self.move_group_name = self.get_parameter("move_group_name").value

        # 多线程执行器配套的 callback group
        # Reentrant 允许回调内部再调阻塞操作（IK service）
        self.cb_group = ReentrantCallbackGroup()
        
        # region: 这个是 planner 的创建。 不是 IK 预检
        self.declare_parameter("joint_names",
                            ["j1", "j2", "j3", "j4", "j5", "j6"])  # 按 SRDF 改
        joint_names = self.get_parameter("joint_names").value

        # 这一个是让状态机能够自己 控制 servo
        self.planner = MoveItPlanner(
            node=self,
            joint_names=joint_names,
            base_link=self.base_frame,
            end_effector=self.ee_frame,
            group_name=self.move_group_name,
            callback_group=self.cb_group,
        )
        self.planned_yaw = 0.0

        # IK 服务客户端
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik",
            callback_group=self.cb_group)
        # endregion

        # ---------- 订阅 ----------
        self.create_subscription(
            JointState, "/joint_states", self.cb_joint_state, 10)
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.cb_info, 10)
        self.create_subscription(
            Image, "/camera/depth/image_raw", self.cb_depth, 10)
        self.create_subscription(
            Yolov8Inference, "/yolov8/obb_detections", self.cb_det, 10)
        # self.create_subscription(
        #     Bool, "/servo_trigger", self.cb_trigger, 10)
        self.create_subscription(
            Bool, "/servo_trigger", self.cb_trigger, 10,
            callback_group=self.cb_group)

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
            f"grasp_seq={self.enable_grasp_seq} | "
            f"avoid_collisions = {self.avoid_collisions}"
        )

    # ==================================================================
    # 回调
    # ==================================================================
    def cb_info(self, msg: CameraInfo):
        self.estimator.set_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def cb_joint_state(self, msg: JointState):
        self.latest_joint_state = msg

    def cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_stamp = msg.header.stamp

    def cb_det(self, msg: Yolov8Inference):
        self.latest_det = msg
        self.latest_det_time = self._now_sec()

        if len(msg.results) == 0:
            self.get_logger().info(
                "det received: empty results",
                throttle_duration_sec=1.0,
            )
            return

        target = max(msg.results, key=lambda r: r.confidence)

        self.get_logger().info(
            f"det received | n={len(msg.results)} "
            f"best={target.class_name} "
            f"conf={target.confidence:.3f} "
            f"px=({target.center_x:.1f},{target.center_y:.1f}) "
            f"size=({target.width:.1f},{target.height:.1f}) "
            f"angle={target.angle:.3f} "
            f"frame={msg.header.frame_id}",
            throttle_duration_sec=1.0,
        )

    def cb_trigger(self, msg: Bool):
        if msg.data:
            if self.state not in (ServoState.IDLE, ServoState.DONE):
                self.get_logger().warning(
                    f"trigger ignored, state={self.state.name}")
                return

            target = self._observe_target_in_base()
            if target is None:
                self.get_logger().warning("trigger refused: no valid target")
                return


            # IK 预检测， 检测目标点是否可达。
            hover_pt = target + np.array([0.0, 0.0, self.hover_z])
            if not self._check_reachable(hover_pt):
                self.get_logger().warning("trigger refused: not reachable")
                return

            # 锁定目标 + yaw，PLANNING 阶段用  
            # 锁定目标位置。后续执行阶段优先使用这个 target_locked，
            # 避免机械臂遮挡目标后 YOLO 丢失导致停止。
            self.target_locked = target.copy()
            self.target_locked_time = self._now_sec()

            self.planned_yaw = 0.0

            if self.latest_det is not None and len(self.latest_det.results) > 0:
                det = max(self.latest_det.results, key=lambda r: r.confidence)
                self.planned_yaw = float(det.angle)
            else:
                self.get_logger().warning(
                    "no detection angle available, use planned_yaw=0.0"
                )

            self.get_logger().info(
                f">>> trigger: lock target and enter PLANNING | "
                f"target=({self.target_locked[0]:+.4f}, "
                f"{self.target_locked[1]:+.4f}, "
                f"{self.target_locked[2]:+.4f}) "
                f"hover=({hover_pt[0]:+.4f}, "
                f"{hover_pt[1]:+.4f}, "
                f"{hover_pt[2]:+.4f}) "
                f"yaw={self.planned_yaw:+.3f} "
                f"policy={self.target_policy}"
            )
            self._enter_state(ServoState.PLANNING)

        else:
            self.get_logger().info(">>> trigger: back to IDLE")
            self._enter_state(ServoState.IDLE)

    def _enter_state(self, new_state: ServoState):
        if new_state == self.state:
            return
        self.get_logger().info(f"state: {self.state.name} → {new_state.name}")
        self.state = new_state

        # 切到任何伺服状态都要 reset PD，避免微分项乱跳
        if new_state in (ServoState.APPROACHING, ServoState.DESCENDING, ServoState.RETREATING):
            self.pd.reset()

        # PLANNING 是阻塞动作，用 一次 timer 异步执行，
        # 避免卡住控制循环
        if new_state == ServoState.PLANNING:
            self._planning_timer = self.create_timer(
                0.01, self._do_planning_once)

        if new_state == ServoState.GRASPING:
            self.grasp_start_time = self._now_sec()
        else:
            self.grasp_start_time = None

        if new_state == ServoState.IDLE:
            self.target_locked = None
            self.target_locked_time = None
    
    def _do_planning_once(self):
        self._planning_timer.cancel()
        self._planning_timer.destroy()

        if self.target_locked is None:
            self.get_logger().warning(
                "planning aborted: target_locked is None"
            )
            self._enter_state(ServoState.IDLE)
            return

        hover_pt = self.target_locked + np.array([0.0, 0.0, self.hover_z])
        ok = self.planner.plan_to_pregrasp(hover_pt, yaw=self.planned_yaw)

        if ok:
            self.get_logger().info("pregrasp reached, enter APPROACHING")
            self._enter_state(ServoState.APPROACHING)
        else:
            self.get_logger().warning("planning failed, back to IDLE")
            self._enter_state(ServoState.IDLE)

        
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
        """APPROACHING 收敛回调。

        关键原则：
        - 不要因为当前 YOLO 遮挡就丢掉 target_locked
        - 默认继续使用 trigger 时锁定的目标
        - 如需刷新，只有在当前能看见目标且新旧目标距离很近时才刷新
        """

        if self.refresh_target_at_approach_done:
            observed = self._observe_target_in_base()

            if observed is not None:
                if self.target_locked is None:
                    self.target_locked = observed.copy()
                    self.target_locked_time = self._now_sec()
                    self.get_logger().info(
                        "target_locked created at approach done"
                    )
                else:
                    jump = float(np.linalg.norm(observed - self.target_locked))

                    if jump <= self.target_refresh_max_jump:
                        self.target_locked = observed.copy()
                        self.target_locked_time = self._now_sec()

                        self.get_logger().info(
                            f"target_locked refreshed at approach done | "
                            f"jump={jump*1000:.1f}mm "
                            f"x={self.target_locked[0]:+.4f}, "
                            f"y={self.target_locked[1]:+.4f}, "
                            f"z={self.target_locked[2]:+.4f}"
                        )
                    else:
                        self.get_logger().warning(
                            f"target refresh rejected at approach done: "
                            f"jump={jump*1000:.1f}mm > "
                            f"{self.target_refresh_max_jump*1000:.1f}mm, "
                            f"keep previous target_locked"
                        )
            else:
                self.get_logger().warning(
                    "target not visible at approach done, keep previous target_locked"
                )
        else:
            self.get_logger().info(
                "approach done: keep existing target_locked"
            )

        if self.target_locked is None:
            self.get_logger().warning(
                "no target_locked available at approach done, abort to DONE"
            )
            self._enter_state(ServoState.DONE)
            return

        if not self.get_parameter("enable_grasp_sequence").value:
            self._enter_state(ServoState.DONE)
        else:
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

        # if elapsed >= self.grasp_duration:
        # 这里改成实时读取， 也是为了方便
        if elapsed >= self.get_parameter("grasp_duration").value:
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

    def _debug_depth_at_pixel(self, u: float, v: float):
        """打印检测中心附近的深度原始值，帮助判断深度是否正常。"""
        if self.depth_img is None:
            return

        h, w = self.depth_img.shape[:2]
        x = int(round(u))
        y = int(round(v))

        if x < 0 or x >= w or y < 0 or y >= h:
            self.get_logger().warning(
                f"target pixel out of depth image: "
                f"u={u:.1f}, v={v:.1f}, depth_size=({w},{h})",
                throttle_duration_sec=1.0,
            )
            return

        # 取中心附近 5x5 小窗口，避免单个深度点刚好无效
        x0 = max(0, x - 2)
        x1 = min(w, x + 3)
        y0 = max(0, y - 2)
        y1 = min(h, y + 3)

        patch = self.depth_img[y0:y1, x0:x1]
        finite = patch[np.isfinite(patch)]

        if finite.size == 0:
            self.get_logger().warning(
                f"depth patch invalid at pixel ({x},{y})",
                throttle_duration_sec=1.0,
            )
            return

        raw_center = self.depth_img[y, x]
        raw_min = np.min(finite)
        raw_max = np.max(finite)
        raw_mean = np.mean(finite)

        self.get_logger().info(
            f"depth debug | pixel=({x},{y}) "
            f"dtype={self.depth_img.dtype} "
            f"center={raw_center} "
            f"patch_mean={raw_mean:.4f} "
            f"patch_min={raw_min:.4f} "
            f"patch_max={raw_max:.4f} "
            f"image_size=({w},{h})",
            throttle_duration_sec=1.0,
        )

    # ==================================================================
    # 辅助：误差计算 / TF / 夹爪
    # ==================================================================
    # region 旧函数
    # def _compute_target_in_base(self) -> Optional[np.ndarray]:
        # """像素 → 相机系 → base_link，返回目标物体位置（不含 hover_z）。
        # 如果 debug_target 非零则直接用硬编码目标，跳过像素管线。"""
        # debug = list(self.get_parameter("debug_target").value)
        # if any(abs(v) > 1e-6 for v in debug):
        #     return np.array(debug, dtype=float)

        # if self.depth_img is None:
        #     return None
        # if self.latest_det is None or len(self.latest_det.results) == 0:
        #     return None
        # if not self.estimator.has_intrinsics():
        #     return None

        # # 超时保护
        # if self.latest_det_time is None:
        #     return None

        # if self._now_sec() - self.latest_det_time > self.det_timeout:
        #     self.get_logger().warning(
        #         "detection timeout, target is stale",
        #         throttle_duration_sec=2.0
        #     )
        #     return None

        # target = max(self.latest_det.results, key=lambda r: r.confidence)
        # xyz_cam = self.estimator.pixel_to_camera(
        #     target.center_x, target.center_y, self.depth_img)
        # if xyz_cam is None:
        #     return None

        # pt = PointStamped()
        # pt.header.frame_id = self.camera_frame
        # pt.header.stamp = self.depth_stamp
        # pt.point.x, pt.point.y, pt.point.z = (
        #     float(xyz_cam[0]), float(xyz_cam[1]), float(xyz_cam[2]))
        # try:
        #     pt_base = self.tf_buffer.transform(
        #         pt, self.base_frame, timeout=Duration(seconds=0.1))
        # except Exception as e:
        #     self.get_logger().warning(
        #         f"TF target→base failed: {e}", throttle_duration_sec=2.0)
        #     return None
        # return np.array(
        #     [pt_base.point.x, pt_base.point.y, pt_base.point.z])
    # endregion
    
    def _observe_target_in_base(self) -> Optional[np.ndarray]:
        """像素 → 相机系 → base_link，返回目标物体位置（不含 hover_z）。
        如果 debug_target 非零则直接用硬编码目标，跳过像素管线。
        """
        debug = list(self.get_parameter("debug_target").value)
        if any(abs(v) > 1e-6 for v in debug):
            target_debug = np.array(debug, dtype=float)
            self.get_logger().info(
                f"use debug_target in base_link: "
                f"({target_debug[0]:+.4f}, {target_debug[1]:+.4f}, {target_debug[2]:+.4f})",
                throttle_duration_sec=1.0,
            )
            return target_debug

        if self.depth_img is None:
            self.get_logger().warning(
                "no depth image yet",
                throttle_duration_sec=1.0,
            )
            return None

        if self.latest_det is None:
            self.get_logger().warning(
                "no detection msg yet",
                throttle_duration_sec=1.0,
            )
            return None

        if self.latest_det_time is not None:
            age = self._now_sec() - self.latest_det_time
            if age > self.det_timeout:
                self.get_logger().warning(
                    f"detection timeout: age={age:.2f}s > {self.det_timeout:.2f}s",
                    throttle_duration_sec=1.0,
                )
                return None

        if len(self.latest_det.results) == 0:
            self.get_logger().warning(
                "latest detection has empty results",
                throttle_duration_sec=1.0,
            )
            return None

        if not self.estimator.has_intrinsics():
            self.get_logger().warning(
                "camera intrinsics not ready",
                throttle_duration_sec=1.0,
            )
            return None

        # 取最高置信度目标
        target = max(self.latest_det.results, key=lambda r: r.confidence)

        if self.debug_print_target:
            self.get_logger().info(
                f"target pixel | class={target.class_name} "
                f"conf={target.confidence:.3f} "
                f"u={target.center_x:.1f} v={target.center_y:.1f} "
                f"w={target.width:.1f} h={target.height:.1f} "
                f"angle={target.angle:.3f}",
                throttle_duration_sec=1.0,
            )
            self._debug_depth_at_pixel(target.center_x, target.center_y)

        # 像素 + 深度 → 相机坐标
        xyz_cam = self.estimator.pixel_to_camera(
            target.center_x,
            target.center_y,
            self.depth_img,
        )

        if xyz_cam is None:
            self.get_logger().warning(
                "pixel_to_camera failed: invalid depth or intrinsics",
                throttle_duration_sec=1.0,
            )
            return None

        if self.debug_print_target:
            self.get_logger().info(
                f"target in camera | "
                f"x={xyz_cam[0]:+.4f}, y={xyz_cam[1]:+.4f}, z={xyz_cam[2]:+.4f} "
                f"frame={self.camera_frame}",
                throttle_duration_sec=1.0,
            )

        pt = PointStamped()
        pt.header.frame_id = self.camera_frame

        # 如果 TF 时间对不上，可以先用 Time() 获取最新 TF。
        # 你现在用 depth_stamp 也可以，但调试阶段 latest TF 更容易排除时间同步问题。
        # 调试完成之后改为pt.header.stamp = self.depth_stamp
        pt.header.stamp = rclpy.time.Time().to_msg()
        # pt.header.stamp = self.depth_stamp
        
        pt.point.x = float(xyz_cam[0])
        pt.point.y = float(xyz_cam[1])
        pt.point.z = float(xyz_cam[2])

        try:
            pt_base = self.tf_buffer.transform(
                pt,
                self.base_frame,
                timeout=Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warning(
                f"TF target camera→base failed: {e}",
                throttle_duration_sec=1.0,
            )
            return None

        target_base = np.array([
            pt_base.point.x,
            pt_base.point.y,
            pt_base.point.z,
        ])
        
        self.last_visible_target = target_base.copy()
        self.last_visible_target_time = self._now_sec()


        if self.debug_print_target:
            hover_pt = target_base + np.array([0.0, 0.0, self.hover_z])
            descend_pt = target_base + np.array([0.0, 0.0, self.descend_z])

            self.get_logger().info(
                f"target in base | "
                f"x={target_base[0]:+.4f}, y={target_base[1]:+.4f}, z={target_base[2]:+.4f}",
                throttle_duration_sec=1.0,
            )

            self.get_logger().info(
                f"hover target | "
                f"x={hover_pt[0]:+.4f}, y={hover_pt[1]:+.4f}, z={hover_pt[2]:+.4f} "
                f"hover_z={self.hover_z:.3f}",
                throttle_duration_sec=1.0,
            )

            self.get_logger().info(
                f"descend target | "
                f"x={descend_pt[0]:+.4f}, y={descend_pt[1]:+.4f}, z={descend_pt[2]:+.4f} "
                f"descend_z={self.descend_z:.3f}",
                throttle_duration_sec=1.0,
            )

        return target_base
    
    def _compute_target_in_base(self) -> Optional[np.ndarray]:
        """兼容旧接口。

        执行阶段根据 target_policy 返回目标：
        - locked: 使用 target_locked
        - realtime: 使用实时 YOLO
        - hybrid: 可见时刷新，不可见时用 locked
        """
        return self._get_execution_target_in_base()

    def _get_execution_target_in_base(self) -> Optional[np.ndarray]:
        """返回当前执行阶段应该使用的目标位置。

        target_policy:
        - locked:
            trigger 后锁定目标，执行阶段永远使用 target_locked。
            适合静态目标，能解决机械臂遮挡导致 YOLO 丢失的问题。

        - realtime:
            每次都用当前 YOLO 实时检测。
            遮挡时会停，适合后续测试动态目标，不适合当前红盒子。

        - hybrid:
            如果当前看得见目标，并且新目标离 locked target 不远，则刷新；
            如果看不见，则继续使用 locked target。
        """

        policy = self.target_policy

        # 非执行阶段，直接实时观察即可。
        # trigger 时也会走这里，此时 state 通常是 IDLE 或 DONE。
        if self.state not in (
            ServoState.PLANNING,
            ServoState.APPROACHING,
            ServoState.DESCENDING,
            ServoState.GRASPING,
            ServoState.RETREATING,
        ):
            return self._observe_target_in_base()

        # locked 策略：执行阶段只用锁定目标
        if policy == "locked":
            if self.target_locked is not None:
                self.get_logger().info(
                    f"use locked target | "
                    f"x={self.target_locked[0]:+.4f}, "
                    f"y={self.target_locked[1]:+.4f}, "
                    f"z={self.target_locked[2]:+.4f}",
                    throttle_duration_sec=1.0,
                )
                return self.target_locked.copy()

            self.get_logger().warning(
                "execution target unavailable: target_policy=locked but target_locked is None",
                throttle_duration_sec=1.0,
            )
            return None

        # realtime 策略：始终依赖当前检测
        if policy == "realtime":
            return self._observe_target_in_base()

        # hybrid 策略：看得见则小范围刷新，看不见则用 locked
        if policy == "hybrid":
            observed = self._observe_target_in_base()

            if observed is not None:
                if self.target_locked is None:
                    self.target_locked = observed.copy()
                    self.target_locked_time = self._now_sec()
                    return observed

                jump = float(np.linalg.norm(observed - self.target_locked))

                if jump <= self.target_refresh_max_jump:
                    self.target_locked = observed.copy()
                    self.target_locked_time = self._now_sec()

                    self.get_logger().info(
                        f"hybrid target refreshed | jump={jump*1000:.1f}mm "
                        f"x={observed[0]:+.4f}, "
                        f"y={observed[1]:+.4f}, "
                        f"z={observed[2]:+.4f}",
                        throttle_duration_sec=1.0,
                    )
                    return observed

                self.get_logger().warning(
                    f"hybrid observed target jump too large: "
                    f"{jump*1000:.1f}mm > {self.target_refresh_max_jump*1000:.1f}mm, "
                    f"keep locked target",
                    throttle_duration_sec=1.0,
                )

            if self.target_locked is not None:
                self.get_logger().warning(
                    "hybrid target not visible, use locked target",
                    throttle_duration_sec=1.0,
                )
                return self.target_locked.copy()

            return None

        self.get_logger().warning(
            f"unknown target_policy={policy}, fallback to locked behavior",
            throttle_duration_sec=1.0,
        )

        if self.target_locked is not None:
            return self.target_locked.copy()

        return self._observe_target_in_base()

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
        """误差 = (执行目标位置 + z_offset) - 末端位置"""

        target = self._get_execution_target_in_base()

        if target is None:
            self.get_logger().warning(
                f"servo target unavailable in state={self.state.name}, output zero twist",
                throttle_duration_sec=1.0,
            )
            return None

        ee = self._lookup_ee_in_base()
        if ee is None:
            return None

        goal = target + np.array([0.0, 0.0, z_offset])
        error = goal - ee

        self.get_logger().info(
            f"servo goal | state={self.state.name} "
            f"policy={self.target_policy} "
            f"target=({target[0]:+.4f}, {target[1]:+.4f}, {target[2]:+.4f}) "
            f"goal=({goal[0]:+.4f}, {goal[1]:+.4f}, {goal[2]:+.4f}) "
            f"ee=({ee[0]:+.4f}, {ee[1]:+.4f}, {ee[2]:+.4f}) "
            f"err=({error[0]:+.4f}, {error[1]:+.4f}, {error[2]:+.4f})",
            throttle_duration_sec=1.0,
        )

        return error

    def _send_gripper(self, positions):
        """发一个 1s 走到位的 trajectory 点"""
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start.sec = 1
        msg.points.append(pt)
        self.gripper_pub.publish(msg)
    
    def _check_reachable(self, position_in_base: np.ndarray) -> bool:
        """用 MoveIt /compute_ik 检查 base_link 系下某个位置是否可达。

        返回 True 表示有 IK 解（且无碰撞）；False 表示不可达或服务超时。
        服务本身不可用时返回 True 跳过检查，不因为基础设施问题拒绝抓取。
        """
        if not self.ik_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning(
                "/compute_ik unavailable, skip reachability check")
            return True

        # IK 必须知道"当前姿态"才能解算，没收到 /joint_states 就跳过
        # if self.latest_joint_state is None:
        #     self.get_logger().warning(
        #         "no /joint_states yet, skip reachability check")
        #     return True
    
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.move_group_name
        # req.ik_request.robot_state.joint_state = self.latest_joint_state
        req.ik_request.pose_stamped.header.frame_id = self.base_frame
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose.position.x = float(position_in_base[0])
        req.ik_request.pose_stamped.pose.position.y = float(position_in_base[1])
        req.ik_request.pose_stamped.pose.position.z = float(position_in_base[2])
        # 俯视抓取姿态：roll=π → quat (1,0,0,0)
        # 姿态在 IK 检查阶段固定，不影响"位置是否在工作空间内"的判断
        req.ik_request.pose_stamped.pose.orientation.x = 1.0
        req.ik_request.pose_stamped.pose.orientation.y = 0.0
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 0.0
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = int(0.5 * 1e9)   # 500ms IK 超时
        req.ik_request.avoid_collisions = self.avoid_collisions

        if self.debug_print_ik:
            self.get_logger().info(
                f"IK request | group={self.move_group_name} "
                f"frame={self.base_frame} "
                f"pos=({position_in_base[0]:+.4f}, "
                f"{position_in_base[1]:+.4f}, "
                f"{position_in_base[2]:+.4f}) "
                f"quat=(x=1.0, y=0.0, z=0.0, w=0.0) "
                f"avoid_collisions={self.avoid_collisions}",
                throttle_duration_sec=1.0,
            )

        # 发送异步请求
        future = self.ik_client.call_async(req)
        if not self._wait_future(future, timeout_sec=1.0):
            self.get_logger().warning("IK service call timed out")
            return False
        
        code = future.result().error_code.val

        if code != 1:
            self.get_logger().warning(
                f"IK failed | error_code={code} "
                f"target=({position_in_base[0]:+.4f}, "
                f"{position_in_base[1]:+.4f}, "
                f"{position_in_base[2]:+.4f}) "
                f"avoid_collisions={self.avoid_collisions}"
            )
            return False

        self.get_logger().info(
            f"IK success | target=({position_in_base[0]:+.4f}, "
            f"{position_in_base[1]:+.4f}, "
            f"{position_in_base[2]:+.4f})",
            throttle_duration_sec=1.0,
        )

        return True
    
    def _wait_future(self, future, timeout_sec: float) -> bool:
        """轮询等待 future 完成。
        避免 spin_until_future_complete 在回调里造成的嵌套 spin 问题。
        依赖 MultiThreadedExecutor 的其他线程处理服务响应。
        """
        import time
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return False




def main():
    rclpy.init()
    node = VisualServoNode()
    # 必须用多线程，否则 cb_trigger 里 spin_until_future_complete 会死锁
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
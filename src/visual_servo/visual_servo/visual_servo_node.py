#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visual_servo_node.py - closed-loop visual servo grasp architecture

Pipeline:
  IDLE
  -> DETECTING                # YOLO 稳定确认 + 锁定 OBB 快照
  -> COARSE_PLANNING
  -> MOVING_TO_PREGRASP       # MoveIt 粗定位到 target 上方(已偏移)
  -> VISUAL_ALIGN_XY          # 锁定 target_uv,闭环 EE 投影到 target_uv
  -> VISUAL_ALIGN_YAW         # 用锁定的 target_yaw 闭环
  -> ALIGN_LOCK_CHECK         # 严格收敛复核
  -> BLIND_DESCEND            # 纯 Z 下降,不读 YOLO
  -> GRASPING
  -> LIFTING
  -> VERIFY_GRASP
  -> DONE / FAILED

Key idea:
  对于固定外置相机 + 静态物体,真实 target_uv 是不变的. 因此进入 VISUAL_ALIGN_XY
  时一次性锁定 target_uv,后续闭环只追这个固定参考即可,完全不依赖 YOLO,
  天然抗遮挡和检测抖动.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import TwistStamped, PointStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from yolov8_obb_msgs.msg import Yolov8Inference

from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

from moveit_msgs.srv import GetPositionIK
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from visual_servo.error_estimator import ErrorEstimator
from visual_servo.moveit_planner import MoveItPlanner


class ServoState(Enum):
    IDLE = 0
    DETECTING = 1
    COARSE_PLANNING = 2
    MOVING_TO_PREGRASP = 3
    VISUAL_ALIGN_XY = 4
    VISUAL_ALIGN_YAW = 5
    ALIGN_LOCK_CHECK = 6
    BLIND_DESCEND = 7
    GRASPING = 8
    LIFTING = 9
    VERIFY_GRASP = 10
    CALIBRATION_HOLD = 11      # NEW
    DONE = 12
    FAILED = 13

@dataclass
class TrackedDetection:
    seq: int
    stamp_sec: float
    class_name: str
    confidence: float
    u: float
    v: float
    width: float
    height: float
    angle: float


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def normalize_angle_pi_symmetry(a: float) -> float:
    """Parallel gripper: yaw and yaw+pi are equivalent."""
    a = normalize_angle(a)
    if a > math.pi / 2.0:
        a -= math.pi
    elif a < -math.pi / 2.0:
        a += math.pi
    return a


def limit_vector_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > max_norm and n > 1e-12:
        return v * (max_norm / n)
    return v


def yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class VisualServoNode(Node):
    def __init__(self):
        super().__init__("visual_servo_node")

        # ------------------------------------------------------------------
        # Core control and safety
        # ------------------------------------------------------------------
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("max_linear_vel", 0.08)
        self.declare_parameter("max_angular_vel", 0.45)
        self.declare_parameter("avoid_collisions", True)

        # State timeouts (sec)
        self.declare_parameter("detecting_timeout", 5.0)
        self.declare_parameter("coarse_planning_timeout", 20.0)
        self.declare_parameter("visual_align_timeout", 10.0)
        self.declare_parameter("yaw_align_timeout", 6.0)
        self.declare_parameter("lock_check_timeout", 4.0)
        self.declare_parameter("descend_timeout", 10.0)
        self.declare_parameter("lifting_timeout", 8.0)
        self.declare_parameter("verify_timeout", 3.0)

        # Frames
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("ee_frame", "grasp_frame")

        # YOLO detection gating
        self.declare_parameter("target_class_name", "")
        self.declare_parameter("min_confidence", 0.10)
        self.declare_parameter("det_timeout", 0.50)
        self.declare_parameter("stable_detection_frames", 5)
        self.declare_parameter("same_target_px_gate", 35.0)
        self.declare_parameter("track_px_gate", 80.0)
        self.declare_parameter("edge_margin_px", 25.0)
        self.declare_parameter("min_obb_width_px", 5.0)
        self.declare_parameter("min_obb_height_px", 5.0)
        self.declare_parameter("max_obb_width_px", 10000.0)
        self.declare_parameter("max_obb_height_px", 10000.0)

        # Rough 3D estimate for coarse MoveIt planning ONLY
        self.declare_parameter("debug_target", [0.0, 0.0, 0.0])
        self.declare_parameter("hover_offset_z", 0.12)
        self.declare_parameter("coarse_xy_margin", 0.00)
        # 远离相机偏移 pregrasp,降低 EE 自遮挡 (方案 A 改进版:在图像空间挑方向)
        self.declare_parameter("pregrasp_camera_offset", 0.05)
        self.declare_parameter("pregrasp_candidate_dirs", 16)
        self.declare_parameter("pregrasp_min_pixel_separation", 60.0)
        self.declare_parameter("move_group_name", "robot_arm")
        self.declare_parameter("joint_names", ["j1", "j2", "j3", "j4", "j5", "j6"])

        # Z strategy
        self.declare_parameter("z_strategy", "rough_target")  # rough_target / table
        self.declare_parameter("table_z", 0.0)
        self.declare_parameter("pregrasp_height_above_table", 0.16)
        self.declare_parameter("grasp_height_above_table", 0.025)
        self.declare_parameter("descend_offset_z", 0.025)
        self.declare_parameter("lift_height", 0.16)

        # Image-space visual servo
        self.declare_parameter("visual_reference_mode", "ee_projection")
        self.declare_parameter("desired_center_x", -1.0)
        self.declare_parameter("desired_center_y", -1.0)
        # 标定后写入,用于补偿 base->camera TF 误差导致的 EE 投影偏差
        self.declare_parameter("grasp_projection_offset_px", [0.0, 0.0])
        
        # 把 EE 投影到 "target 所在平面" 而不是 EE 当前位置.
        # 这是消除视差(parallax)误差的关键. 推荐始终开启.
        self.declare_parameter("use_target_plane_projection", True)
        # target 实际所在水平面相对 grasp_goal_z 的微调 (米).
        # 默认 0 表示用 grasp_goal_z (即夹爪指尖最终接触点的高度).
        # 若 target 几何中心略高于该高度, 可调 +0.005~+0.01.
        self.declare_parameter("target_plane_z_offset", 0.0)

        # Calibration mode: BLIND_DESCEND 到位后不闭爪,保持位置等待人工测量
        self.declare_parameter("calibration_mode", False)



        self.declare_parameter("j_img_to_base", [0.002544, 0.002325, 0.001149, 0.003549])
        self.declare_parameter("visual_align_gain", 0.45)
        self.declare_parameter("visual_align_pixel_tolerance", 8.0)
        self.declare_parameter("visual_align_max_step", 0.0015)
        self.declare_parameter("visual_align_max_vel", 0.045)
        self.declare_parameter("use_adaptive_jacobian", True)

        # 锁定 target_uv 模式 (推荐): 进入 VISUAL_ALIGN_XY 时拍快照,后续不读 YOLO
        self.declare_parameter("lock_target_during_alignment", True)
        # 锁定模式下用 50Hz control tick 计数,而非 YOLO seq
        self.declare_parameter("visual_align_stable_ticks", 30)   # 0.6s
        self.declare_parameter("yaw_align_stable_ticks", 20)      # 0.4s
        self.declare_parameter("lock_check_stable_ticks", 40)     # 0.8s
        # 非锁定模式下的旧帧计数 (回退路径)
        self.declare_parameter("visual_align_stable_frames", 6)
        self.declare_parameter("yaw_stable_frames", 4)
        self.declare_parameter("lock_check_stable_frames", 10)

        # ALIGN_LOCK_CHECK 严格阈值
        self.declare_parameter("lock_check_pixel_tolerance", 4.0)
        self.declare_parameter("lock_check_yaw_tolerance", 0.05)
        self.declare_parameter("lock_check_max_retries", 2)
        # 在 lock check 阶段如果 fresh detection 可见,
        # 顺便监测锁定值是否漂移过大 (像素)
        self.declare_parameter("lock_drift_warning_px", 25.0)

        # BLIND_DESCEND: pure Z, no YOLO
        self.declare_parameter("descend_speed", 0.018)

        # YOLO OBB to gripper yaw mapping
        self.declare_parameter("obb_to_gripper_yaw_sign", 1.0)
        self.declare_parameter("obb_to_gripper_yaw_offset", 0.0)
        self.declare_parameter("yaw_pi_symmetry", True)
        self.declare_parameter("yaw_gain", 1.2)
        self.declare_parameter("yaw_tolerance", 0.08)
        self.declare_parameter("obb_angle_is_long_axis", True)
        self.declare_parameter("grip_short_edge", True)

        # Gripper and verification
        self.declare_parameter("gripper_topic", "/hand_controller/joint_trajectory")
        self.declare_parameter("gripper_joint_names", ["finger1_joint", "finger2_joint"])
        self.declare_parameter("gripper_open_pos", [0.025, -0.025])
        self.declare_parameter("gripper_close_pos", [0.0, 0.0])
        self.declare_parameter("gripper_motion_time", 1.0)
        self.declare_parameter("grasp_duration", 1.5)
        self.declare_parameter("enable_verify", True)
        self.declare_parameter("verify_missing_frames", 5)
        self.declare_parameter("verify_follow_px", 45.0)

        # Debug
        self.declare_parameter("debug_print_detection", True)
        self.declare_parameter("debug_print_servo", True)
        self.declare_parameter("debug_print_rough_target", True)
        self.declare_parameter("debug_print_ik", False)

        # ------------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------------
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / max(1.0, self.control_rate)
        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.max_linear_vel = float(self.get_parameter("max_linear_vel").value)
        self.max_angular_vel = float(self.get_parameter("max_angular_vel").value)
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.det_timeout = float(self.get_parameter("det_timeout").value)

        self.hover_offset_z = float(self.get_parameter("hover_offset_z").value)
        self.descend_offset_z = float(self.get_parameter("descend_offset_z").value)
        self.lift_height = float(self.get_parameter("lift_height").value)
        self.move_group_name = str(self.get_parameter("move_group_name").value)

        j_raw = list(self.get_parameter("j_img_to_base").value)
        if len(j_raw) != 4:
            raise RuntimeError("j_img_to_base must have 4 numbers: [j00,j01,j10,j11]")
        self.j_img_to_base = np.array([[float(j_raw[0]), float(j_raw[1])],
                                       [float(j_raw[2]), float(j_raw[3])]], dtype=float)

        self.gripper_joint_names = list(self.get_parameter("gripper_joint_names").value)
        self.gripper_open_pos = list(self.get_parameter("gripper_open_pos").value)
        self.gripper_close_pos = list(self.get_parameter("gripper_close_pos").value)

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.bridge = CvBridge()
        self.estimator = ErrorEstimator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cb_group = ReentrantCallbackGroup()

        joint_names = list(self.get_parameter("joint_names").value)
        self.planner = MoveItPlanner(
            node=self,
            joint_names=joint_names,
            base_link=self.base_frame,
            end_effector=self.ee_frame,
            group_name=self.move_group_name,
            callback_group=self.cb_group,
        )

        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.cb_group)

        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.cb_info, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self.cb_depth, 10)
        self.create_subscription(Yolov8Inference, "/yolov8/obb_detections", self.cb_det, 10)
        self.create_subscription(JointState, "/joint_states", self.cb_joint_state, 10)
        self.create_subscription(Bool, "/servo_trigger", self.cb_trigger, 10,
                                 callback_group=self.cb_group)

        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter("gripper_topic").value), 10)

        self.create_timer(self.dt, self.control_loop)

        # ------------------------------------------------------------------
        # Runtime caches
        # ------------------------------------------------------------------
        self.depth_img: Optional[np.ndarray] = None
        self.depth_stamp = None
        self.latest_joint_state: Optional[JointState] = None

        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.image_width: Optional[int] = None
        self.image_height: Optional[int] = None

        self.det_seq = 0
        self.latest_det_msg: Optional[Yolov8Inference] = None
        self.latest_det_time: Optional[float] = None
        self.latest_valid_det: Optional[TrackedDetection] = None
        self.last_visible_det: Optional[TrackedDetection] = None
        self.last_visible_det_time: Optional[float] = None

        # Detection stability
        self.detect_seed: Optional[TrackedDetection] = None
        self.detect_stable_count = 0
        self.detect_last_seq = -1

        # Alignment stability counters (used in both lock and fresh modes)
        self.xy_stable_count = 0
        self.xy_last_seq = -1
        self.yaw_stable_count = 0
        self.yaw_last_seq = -1
        self.lock_check_count = 0
        self.lock_check_last_seq = -1
        self.lock_check_retry_count = 0

        # Verification
        self.verify_missing_count = 0
        self.verify_follow_count = 0
        self.verify_last_seq = -1

        # Locked targets for execution
        self.locked_det: Optional[TrackedDetection] = None
        self.locked_target_uv: Optional[np.ndarray] = None    # 锁定的目标像素
        self.rough_target_base: Optional[np.ndarray] = None
        self.pregrasp_goal_base: Optional[np.ndarray] = None
        self.grasp_goal_z: Optional[float] = None
        self.lift_goal_z: Optional[float] = None
        self.target_yaw: float = 0.0

        self.state = ServoState.IDLE
        self.state_entry_time = self._now_sec()
        self.failed_reason = ""
        self.grasp_start_time: Optional[float] = None

        self._planning_timer = None

        # 确保 Servo 在可执行状态
        self.planner.start_servo()


        self.get_logger().info(
            "closed-loop visual_servo started | "
            f"rate={self.control_rate:.1f}Hz motion={self.enable_motion} "
            f"lock_target={self.get_parameter('lock_target_during_alignment').value} "
            f"J={self.j_img_to_base.tolist()}"
        )

    # ----------------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------------
    def cb_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.image_width = int(msg.width)
        self.image_height = int(msg.height)
        self.estimator.set_intrinsics(self.fx, self.fy, self.cx, self.cy)

    def cb_joint_state(self, msg: JointState):
        self.latest_joint_state = msg

    def cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_stamp = msg.header.stamp

    def cb_det(self, msg: Yolov8Inference):
        self.det_seq += 1
        self.latest_det_msg = msg
        self.latest_det_time = self._now_sec()

        det = self._select_detection_from_msg(msg, self.det_seq)
        self.latest_valid_det = det

        if det is not None:
            self.last_visible_det = det
            self.last_visible_det_time = det.stamp_sec
            if bool(self.get_parameter("debug_print_detection").value):
                self.get_logger().info(
                    f"det | class={det.class_name} conf={det.confidence:.3f} "
                    f"uv=({det.u:.1f},{det.v:.1f}) size=({det.width:.1f},{det.height:.1f}) "
                    f"angle={det.angle:+.3f} seq={det.seq}",
                    throttle_duration_sec=1.0,
                )
        else:
            if bool(self.get_parameter("debug_print_detection").value):
                n = len(msg.results) if msg is not None else 0
                self.get_logger().info(
                    f"det invalid/empty | n={n} seq={self.det_seq}",
                    throttle_duration_sec=1.0,
                )

    def cb_trigger(self, msg: Bool):
        if msg.data:
            if self.state not in (ServoState.IDLE, ServoState.DONE, ServoState.FAILED):
                self.get_logger().warning(f"trigger ignored: state={self.state.name}")
                return

            self._reset_runtime_for_new_grasp()
            self._send_gripper(self.gripper_open_pos)
            self.get_logger().info(">>> trigger received: enter DETECTING")
            self._enter_state(ServoState.DETECTING)
        else:
            self.get_logger().info(">>> trigger false: stop and return to IDLE")
            self._enter_state(ServoState.IDLE)

    # ----------------------------------------------------------------------
    # State machine
    # ----------------------------------------------------------------------
    def control_loop(self):
        twist = self._zero_twist()

        if self.state == ServoState.IDLE:
            pass
        elif self.state == ServoState.DETECTING:
            self._state_detecting()
        elif self.state == ServoState.COARSE_PLANNING:
            self._check_state_timeout("coarse_planning_timeout", "coarse planning timeout")
        elif self.state == ServoState.MOVING_TO_PREGRASP:
            self._check_state_timeout("coarse_planning_timeout", "moving to pregrasp timeout")
        elif self.state == ServoState.VISUAL_ALIGN_XY:
            twist = self._state_visual_align_xy()
        elif self.state == ServoState.VISUAL_ALIGN_YAW:
            twist = self._state_visual_align_yaw()
        elif self.state == ServoState.ALIGN_LOCK_CHECK:
            twist = self._state_align_lock_check()
        elif self.state == ServoState.BLIND_DESCEND:
            twist = self._state_blind_descend()
        elif self.state == ServoState.GRASPING:
            self._state_grasping()
        elif self.state == ServoState.LIFTING:
            twist = self._state_lifting()
        elif self.state == ServoState.VERIFY_GRASP:
            self._state_verify_grasp()
        elif self.state == ServoState.CALIBRATION_HOLD:
            twist = self._state_calibration_hold() 
        self.twist_pub.publish(twist)

    def _enter_state(self, new_state: ServoState):
        if new_state == self.state:
            return

        self.get_logger().info(f"state: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_entry_time = self._now_sec()

        if new_state == ServoState.DETECTING:
            self.detect_seed = None
            self.detect_stable_count = 0
            self.detect_last_seq = -1

        if new_state == ServoState.VISUAL_ALIGN_XY:
            self.xy_stable_count = 0
            self.xy_last_seq = -1
            # 进入对齐时锁定 target_uv,后续不依赖 YOLO
            if bool(self.get_parameter("lock_target_during_alignment").value):
                self._lock_target_for_alignment()

        if new_state == ServoState.VISUAL_ALIGN_YAW:
            self.yaw_stable_count = 0
            self.yaw_last_seq = -1

        if new_state == ServoState.ALIGN_LOCK_CHECK:
            self.lock_check_count = 0
            self.lock_check_last_seq = -1

        if new_state == ServoState.GRASPING:
            self.grasp_start_time = self._now_sec()
        else:
            self.grasp_start_time = None

        if new_state == ServoState.VERIFY_GRASP:
            self.verify_missing_count = 0
            self.verify_follow_count = 0
            self.verify_last_seq = -1

        if new_state == ServoState.COARSE_PLANNING:
            self._launch_coarse_planning_once()

        if new_state == ServoState.IDLE:
            self._reset_runtime_for_new_grasp()

    def _fail(self, reason: str):
        self.failed_reason = reason
        self.get_logger().warning(f"FAILED: {reason}")
        self._enter_state(ServoState.FAILED)

    def _check_state_timeout(self, param_name: str, reason: str) -> bool:
        timeout = float(self.get_parameter(param_name).value)
        if self._now_sec() - self.state_entry_time > timeout:
            self._fail(reason)
            return True
        return False

    # ----------------------------------------------------------------------
    # Lock target snapshot
    # ----------------------------------------------------------------------
    def _lock_target_for_alignment(self):
        """进入 VISUAL_ALIGN_XY 时,把 target_uv 拍成快照,
        后续对齐过程不再读 YOLO. 优先用 DETECTING 阶段 EMA 平滑过的值,
        因为它聚合了多帧检测,比单帧 fresh 更稳."""
        if self.locked_det is not None:
            self.locked_target_uv = np.array(
                [self.locked_det.u, self.locked_det.v], dtype=float)
            source = "detect_seed (EMA smoothed)"
        else:
            fresh = self._get_fresh_valid_detection()
            if fresh is not None:
                self.locked_target_uv = np.array([fresh.u, fresh.v], dtype=float)
                source = "fresh fallback"
            else:
                self.locked_target_uv = None
                self.get_logger().error("cannot lock target_uv: no detection available")
                return
        self.get_logger().info(
            f"locked target_uv=({self.locked_target_uv[0]:.1f},"
            f"{self.locked_target_uv[1]:.1f}) source={source}"
        )

    def _compute_ee_reference_uv(self) -> Optional[np.ndarray]:
        """计算 EE 的图像参考像素.

        工程上的关键: 不要直接投影 EE 当前 3D 位置. 那样在 hover 高度做对齐时,
        由于相机不是纯垂直俯视, "像素对齐" 不等于 "XY 对齐", 下降后 EE 会落在
        距 target 几厘米的位置.

        正确做法: 先把 EE 沿垂直方向投影到 target 所在的水平面 (grasp_goal_z),
        再把这个虚拟点投影到像素. 这样 ref_uv == target_uv 的几何含义就是
        "EE 垂直下降会准确落在 target 上", 与盲降运动学一致.
        """
        mode = str(self.get_parameter("visual_reference_mode").value)
        if mode == "image_center":
            return self._desired_center_uv()

        ee_pose = self._lookup_ee_pose_in_base()
        if ee_pose is None:
            return None
        ee_pos, _ = ee_pose

        if bool(self.get_parameter("use_target_plane_projection").value):
            if self.grasp_goal_z is None:
                self.get_logger().warning(
                    "use_target_plane_projection enabled but grasp_goal_z is None; "
                    "fallback to ee_pos projection (may have parallax error)",
                    throttle_duration_sec=2.0,
                )
                ref_point = ee_pos
            else:
                plane_z = self.grasp_goal_z + \
                        float(self.get_parameter("target_plane_z_offset").value)
                # 关键: 用 EE 的 xy + target 平面的 z
                ref_point = np.array([ee_pos[0], ee_pos[1], plane_z], dtype=float)
        else:
            ref_point = ee_pos

        ref_uv = self._project_base_point_to_pixel(ref_point)
        if ref_uv is None:
            return None

        offset = list(self.get_parameter("grasp_projection_offset_px").value)
        if len(offset) == 2:
            ref_uv = ref_uv + np.array([float(offset[0]), float(offset[1])], dtype=float)
        return ref_uv


    # ----------------------------------------------------------------------
    # DETECTING
    # ----------------------------------------------------------------------
    def _state_detecting(self):
        if self._check_state_timeout("detecting_timeout", "detecting timeout"):
            return

        det = self._get_fresh_valid_detection()
        if det is None:
            return

        if det.seq == self.detect_last_seq:
            return
        self.detect_last_seq = det.seq

        if self.detect_seed is None:
            self.detect_seed = det
            self.detect_stable_count = 1
        else:
            same = self._same_detection(self.detect_seed, det,
                                        float(self.get_parameter("same_target_px_gate").value))
            if same:
                self.detect_stable_count += 1
                alpha = 0.25
                self.detect_seed = TrackedDetection(
                    seq=det.seq,
                    stamp_sec=det.stamp_sec,
                    class_name=det.class_name,
                    confidence=det.confidence,
                    u=(1.0 - alpha) * self.detect_seed.u + alpha * det.u,
                    v=(1.0 - alpha) * self.detect_seed.v + alpha * det.v,
                    width=(1.0 - alpha) * self.detect_seed.width + alpha * det.width,
                    height=(1.0 - alpha) * self.detect_seed.height + alpha * det.height,
                    angle=det.angle,
                )
            else:
                self.detect_seed = det
                self.detect_stable_count = 1

        self.get_logger().info(
            f"[DETECTING] stable={self.detect_stable_count}/"
            f"{int(self.get_parameter('stable_detection_frames').value)} "
            f"uv=({self.detect_seed.u:.1f},{self.detect_seed.v:.1f})",
            throttle_duration_sec=0.5,
        )

        if self.detect_stable_count < int(self.get_parameter("stable_detection_frames").value):
            return

        self.locked_det = self.detect_seed
        self.target_yaw = self._target_yaw_from_obb(
            self.locked_det.angle, self.locked_det.width, self.locked_det.height)
        self.rough_target_base = self._observe_target_in_base_rough(self.locked_det)

        if self.rough_target_base is None:
            self._fail("stable detection found, but rough pixel/depth/TF target is unavailable")
            return

        self.pregrasp_goal_base = self._compute_pregrasp_goal(self.rough_target_base)
        self.grasp_goal_z = self._compute_grasp_goal_z(self.rough_target_base)
        self.lift_goal_z = self.grasp_goal_z + self.lift_height

        self.get_logger().info(
            "stable target locked | "
            f"rough=({self.rough_target_base[0]:+.4f},"
            f"{self.rough_target_base[1]:+.4f},"
            f"{self.rough_target_base[2]:+.4f}) "
            f"pregrasp=({self.pregrasp_goal_base[0]:+.4f},"
            f"{self.pregrasp_goal_base[1]:+.4f},"
            f"{self.pregrasp_goal_base[2]:+.4f}) "
            f"target_yaw={self.target_yaw:+.3f} "
            f"obb=(w={self.locked_det.width:.1f},h={self.locked_det.height:.1f},"
            f"a={self.locked_det.angle:+.3f})"
        )
        self._enter_state(ServoState.COARSE_PLANNING)

    # ----------------------------------------------------------------------
    # COARSE_PLANNING / MOVING_TO_PREGRASP
    # ----------------------------------------------------------------------
    def _launch_coarse_planning_once(self):
        if self._planning_timer is not None:
            try:
                self._planning_timer.cancel()
                self._planning_timer.destroy()
            except Exception:
                pass
        self._planning_timer = self.create_timer(0.01, self._do_coarse_planning_once)

    def _do_coarse_planning_once(self):
        if self._planning_timer is not None:
            self._planning_timer.cancel()
            self._planning_timer.destroy()
            self._planning_timer = None

        if self.pregrasp_goal_base is None:
            self._fail("pregrasp_goal_base is None")
            return

        self._enter_state(ServoState.MOVING_TO_PREGRASP)

        ok = self.planner.plan_to_pregrasp(self.pregrasp_goal_base, yaw=self.target_yaw)

        if ok:
            self.get_logger().info("MoveIt coarse pregrasp reached; enter image-space XY alignment")
            self._enter_state(ServoState.VISUAL_ALIGN_XY)
        else:
            self._fail("MoveIt coarse pregrasp failed")

    # ----------------------------------------------------------------------
    # VISUAL_ALIGN_XY (locked target_uv, no YOLO dependency)
    # ----------------------------------------------------------------------
    def _state_visual_align_xy(self) -> TwistStamped:
        twist = self._zero_twist()
        if self._check_state_timeout("visual_align_timeout", "visual XY alignment timeout"):
            return twist

        use_lock = (bool(self.get_parameter("lock_target_during_alignment").value)
                    and self.locked_target_uv is not None)

        if use_lock:
            ref_uv = self._compute_ee_reference_uv()
            if ref_uv is None:
                return twist
            target_uv = self.locked_target_uv
            err_uv = target_uv - ref_uv
        else:
            pack = self._compute_image_error_uv()
            if pack is None:
                return twist
            det, err_uv, ref_uv = pack
            target_uv = np.array([det.u, det.v], dtype=float)

        err_norm = float(np.linalg.norm(err_uv))
        tol = float(self.get_parameter("visual_align_pixel_tolerance").value)

        if use_lock:
            # 50Hz tick 计数
            if err_norm <= tol:
                self.xy_stable_count += 1
            else:
                self.xy_stable_count = 0
            need = int(self.get_parameter("visual_align_stable_ticks").value)
        else:
            # 按 detection seq 计数 (回退路径)
            if pack is not None and pack[0].seq != self.xy_last_seq:
                self.xy_last_seq = pack[0].seq
                if err_norm <= tol:
                    self.xy_stable_count += 1
                else:
                    self.xy_stable_count = 0
            need = int(self.get_parameter("visual_align_stable_frames").value)

        if bool(self.get_parameter("debug_print_servo").value):
            self.get_logger().info(
                f"[VISUAL_ALIGN_XY] {'LOCKED' if use_lock else 'FRESH'} "
                f"e_uv=({err_uv[0]:+.1f},{err_uv[1]:+.1f}) "
                f"|e|={err_norm:.1f}px stable={self.xy_stable_count}/{need} "
                f"target=({target_uv[0]:.1f},{target_uv[1]:.1f}) "
                f"ref=({ref_uv[0]:.1f},{ref_uv[1]:.1f})",
                throttle_duration_sec=0.25,
            )

        if self.xy_stable_count >= need:
            self._enter_state(ServoState.VISUAL_ALIGN_YAW)
            return twist

        vxy = self._image_error_to_base_velocity(
            err_uv,
            gain=float(self.get_parameter("visual_align_gain").value),
            max_step=float(self.get_parameter("visual_align_max_step").value),
            max_vel=float(self.get_parameter("visual_align_max_vel").value),
        )
        return self._fill_twist(twist, vx=vxy[0], vy=vxy[1], vz=0.0, wz=0.0)

    # ----------------------------------------------------------------------
    # VISUAL_ALIGN_YAW (locked target_yaw)
    # ----------------------------------------------------------------------
    def _state_visual_align_yaw(self) -> TwistStamped:
        twist = self._zero_twist()
        if self._check_state_timeout("yaw_align_timeout", "yaw alignment timeout"):
            return twist

        ee_pose = self._lookup_ee_pose_in_base()
        if ee_pose is None:
            return twist
        _, current_yaw = ee_pose

        use_lock = bool(self.get_parameter("lock_target_during_alignment").value)

        if use_lock:
            target_yaw = self.target_yaw  # 在 DETECTING 阶段已锁定
        else:
            det = self._get_fresh_valid_detection()
            if det is None:
                return twist
            target_yaw = self._target_yaw_from_obb(det.angle, det.width, det.height)

        yaw_error = normalize_angle(target_yaw - current_yaw)
        if bool(self.get_parameter("yaw_pi_symmetry").value):
            yaw_error = normalize_angle_pi_symmetry(yaw_error)

        tol = float(self.get_parameter("yaw_tolerance").value)

        if use_lock:
            if abs(yaw_error) <= tol:
                self.yaw_stable_count += 1
            else:
                self.yaw_stable_count = 0
            need = int(self.get_parameter("yaw_align_stable_ticks").value)
        else:
            det = self._get_fresh_valid_detection()
            if det is not None and det.seq != self.yaw_last_seq:
                self.yaw_last_seq = det.seq
                if abs(yaw_error) <= tol:
                    self.yaw_stable_count += 1
                else:
                    self.yaw_stable_count = 0
            need = int(self.get_parameter("yaw_stable_frames").value)

        self.get_logger().info(
            f"[VISUAL_ALIGN_YAW] {'LOCKED' if use_lock else 'FRESH'} "
            f"target={target_yaw:+.3f} current={current_yaw:+.3f} "
            f"err={yaw_error:+.3f} stable={self.yaw_stable_count}/{need}",
            throttle_duration_sec=0.25,
        )

        if self.yaw_stable_count >= need:
            self._enter_state(ServoState.ALIGN_LOCK_CHECK)
            return twist

        wz = clamp(float(self.get_parameter("yaw_gain").value) * yaw_error,
                   -self.max_angular_vel, self.max_angular_vel)
        return self._fill_twist(twist, vx=0.0, vy=0.0, vz=0.0, wz=wz)

    # ----------------------------------------------------------------------
    # ALIGN_LOCK_CHECK (strict re-confirmation)
    # ----------------------------------------------------------------------
    def _state_align_lock_check(self) -> TwistStamped:
        twist = self._zero_twist()

        timeout = float(self.get_parameter("lock_check_timeout").value)
        if self._now_sec() - self.state_entry_time > timeout:
            max_retry = int(self.get_parameter("lock_check_max_retries").value)
            if self.lock_check_retry_count >= max_retry:
                self._fail(f"lock check failed after {self.lock_check_retry_count} retries")
                return twist
            self.lock_check_retry_count += 1
            self.get_logger().warning(
                f"lock check timeout, retry {self.lock_check_retry_count}/{max_retry}, "
                "back to VISUAL_ALIGN_XY"
            )
            self._enter_state(ServoState.VISUAL_ALIGN_XY)
            return twist

        use_lock = (bool(self.get_parameter("lock_target_during_alignment").value)
                    and self.locked_target_uv is not None)

        # 计算 XY 误差
        if use_lock:
            ref_uv = self._compute_ee_reference_uv()
            if ref_uv is None:
                return twist
            err_uv = self.locked_target_uv - ref_uv
        else:
            pack = self._compute_image_error_uv()
            if pack is None:
                return twist
            _, err_uv, ref_uv = pack

        err_norm = float(np.linalg.norm(err_uv))

        # 计算 yaw 误差
        ee_pose = self._lookup_ee_pose_in_base()
        if ee_pose is None:
            return twist
        _, current_yaw = ee_pose

        if use_lock:
            target_yaw = self.target_yaw
        else:
            det = self._get_fresh_valid_detection()
            if det is None:
                return twist
            target_yaw = self._target_yaw_from_obb(det.angle, det.width, det.height)

        yaw_error = normalize_angle(target_yaw - current_yaw)
        if bool(self.get_parameter("yaw_pi_symmetry").value):
            yaw_error = normalize_angle_pi_symmetry(yaw_error)

        px_tol = float(self.get_parameter("lock_check_pixel_tolerance").value)
        yaw_tol = float(self.get_parameter("lock_check_yaw_tolerance").value)

        # 漂移监测: 如果 fresh detection 可见,验证锁定值有没有跑偏
        if use_lock:
            fresh = self._get_fresh_valid_detection()
            if fresh is not None:
                drift = math.hypot(fresh.u - self.locked_target_uv[0],
                                   fresh.v - self.locked_target_uv[1])
                drift_gate = float(self.get_parameter("lock_drift_warning_px").value)
                if drift > drift_gate:
                    self.get_logger().warning(
                        f"locked target may have drifted {drift:.1f}px > {drift_gate:.1f}px "
                        f"(locked=({self.locked_target_uv[0]:.0f},{self.locked_target_uv[1]:.0f}) "
                        f"fresh=({fresh.u:.0f},{fresh.v:.0f}))"
                    )

        # 计数
        if use_lock:
            if err_norm <= px_tol and abs(yaw_error) <= yaw_tol:
                self.lock_check_count += 1
            else:
                self.lock_check_count = 0
            need = int(self.get_parameter("lock_check_stable_ticks").value)
        else:
            seq = pack[0].seq if pack is not None else -1
            if seq != self.lock_check_last_seq:
                self.lock_check_last_seq = seq
                if err_norm <= px_tol and abs(yaw_error) <= yaw_tol:
                    self.lock_check_count += 1
                else:
                    self.lock_check_count = 0
            need = int(self.get_parameter("lock_check_stable_frames").value)

        self.get_logger().info(
            f"[ALIGN_LOCK_CHECK] {'LOCKED' if use_lock else 'FRESH'} "
            f"e_uv={err_norm:.2f}px e_yaw={yaw_error:+.3f}rad "
            f"stable={self.lock_check_count}/{need} retry={self.lock_check_retry_count}",
            throttle_duration_sec=0.25,
        )

        if self.lock_check_count >= need:
            self.get_logger().info(
                f"alignment locked OK | e_uv={err_norm:.2f}px e_yaw={yaw_error:+.3f}rad -> BLIND_DESCEND"
            )
            self._enter_state(ServoState.BLIND_DESCEND)
        return twist

    # ----------------------------------------------------------------------
    # BLIND_DESCEND (pure Z, no YOLO)
    # ----------------------------------------------------------------------
    def _state_blind_descend(self) -> TwistStamped:
        twist = self._zero_twist()

        if self._check_state_timeout("descend_timeout", "blind descend timeout"):
            return twist

        if self.grasp_goal_z is None:
            self._fail("grasp_goal_z is None")
            return twist

        ee_pose = self._lookup_ee_pose_in_base()
        if ee_pose is None:
            return twist
        ee_pos, _ = ee_pose

        if ee_pos[2] <= self.grasp_goal_z:
            self.get_logger().info(
                f"grasp height reached: ee_z={ee_pos[2]:+.4f} goal_z={self.grasp_goal_z:+.4f}"
            )
            if bool(self.get_parameter("calibration_mode").value):
                self._enter_state(ServoState.CALIBRATION_HOLD)
            else:
                self._enter_state(ServoState.GRASPING)
            return twist

        vz = -float(self.get_parameter("descend_speed").value)
        if ee_pos[2] + vz * self.dt < self.grasp_goal_z:
            vz = (self.grasp_goal_z - ee_pos[2]) / self.dt

        self.get_logger().info(
            f"[BLIND_DESCEND] ee_z={ee_pos[2]:+.4f} goal_z={self.grasp_goal_z:+.4f} "
            f"vz={vz:+.3f}",
            throttle_duration_sec=0.5,
        )
        return self._fill_twist(twist, vx=0.0, vy=0.0, vz=vz, wz=0.0)

    
     # ----------------------------------------------------------------------
    
    # ----------------------------------------------------------------------
    # CALIBRATION_HOLD
    # ----------------------------------------------------------------------
    def _state_calibration_hold(self) -> TwistStamped:
        """标定保持: 不闭爪, 不动, 等待外部测量.
        
        通过持续打印 EE 位置 + locked_target_uv 让标定脚本订阅 TF/YOLO 即可读取信息.
        收到 /servo_trigger=false 时退出回 IDLE.
        """
        twist = self._zero_twist()
        
        ee_pose = self._lookup_ee_pose_in_base()
        if ee_pose is not None:
            ee_pos, _ = ee_pose
            target_uv = (self.locked_target_uv.tolist() 
                        if self.locked_target_uv is not None else None)
            self.get_logger().info(
                f"[CALIBRATION_HOLD] ee=({ee_pos[0]:+.4f},{ee_pos[1]:+.4f},{ee_pos[2]:+.4f}) "
                f"grasp_goal_z={self.grasp_goal_z:+.4f} "
                f"locked_target_uv={target_uv}",
                throttle_duration_sec=2.0,
            )
        return twist

    # ----------------------------------------------------------------------
    # GRASPING / LIFTING / VERIFY
    # ----------------------------------------------------------------------
    def _state_grasping(self):
        if self.grasp_start_time is None:
            self.grasp_start_time = self._now_sec()

        elapsed = self._now_sec() - self.grasp_start_time
        if elapsed < 0.05:
            self._send_gripper(self.gripper_close_pos)
            self.get_logger().info("gripper close command sent")

        if elapsed >= float(self.get_parameter("grasp_duration").value):
            ee_pose = self._lookup_ee_pose_in_base()
            if ee_pose is not None:
                ee_pos, _ = ee_pose
                self.lift_goal_z = ee_pos[2] + self.lift_height
            self._enter_state(ServoState.LIFTING)

    def _state_lifting(self) -> TwistStamped:
        twist = self._zero_twist()

        if self._check_state_timeout("lifting_timeout", "lifting timeout"):
            return twist

        if self.lift_goal_z is None:
            self._fail("lift_goal_z is None")
            return twist

        ee_pose = self._lookup_ee_pose_in_base()
        if ee_pose is None:
            return twist
        ee_pos, _ = ee_pose

        z_err = self.lift_goal_z - ee_pos[2]
        if z_err <= 0.004:
            if bool(self.get_parameter("enable_verify").value):
                self._enter_state(ServoState.VERIFY_GRASP)
            else:
                self._enter_state(ServoState.DONE)
            return twist

        vz = clamp(1.2 * z_err, 0.0, min(self.max_linear_vel, 0.05))
        self.get_logger().info(
            f"[LIFTING] z_err={z_err*1000:.1f}mm vz={vz:+.3f}",
            throttle_duration_sec=0.25,
        )
        return self._fill_twist(twist, vx=0.0, vy=0.0, vz=vz, wz=0.0)

    def _state_verify_grasp(self):
        if self._check_state_timeout("verify_timeout", "verify timeout"):
            return

        det = self._get_fresh_valid_detection()
        latest_seq = self.det_seq

        if latest_seq == self.verify_last_seq:
            return
        self.verify_last_seq = latest_seq

        if det is None:
            self.verify_missing_count += 1
            self.verify_follow_count = 0
        else:
            pack = self._compute_image_error_uv()
            if pack is not None:
                _, err_uv, _ = pack
                if float(np.linalg.norm(err_uv)) < float(self.get_parameter("verify_follow_px").value):
                    self.verify_follow_count += 1
                else:
                    self.verify_follow_count = 0
            self.verify_missing_count = 0

        self.get_logger().info(
            f"[VERIFY_GRASP] missing={self.verify_missing_count}/"
            f"{int(self.get_parameter('verify_missing_frames').value)} "
            f"follow={self.verify_follow_count}/"
            f"{int(self.get_parameter('verify_missing_frames').value)}",
            throttle_duration_sec=0.25,
        )

        needed = int(self.get_parameter("verify_missing_frames").value)
        if self.verify_missing_count >= needed:
            self.get_logger().info("verify success: target no longer detected on table")
            self._enter_state(ServoState.DONE)
        elif self.verify_follow_count >= needed:
            self.get_logger().info("verify success: target follows gripper projection")
            self._enter_state(ServoState.DONE)

    # ----------------------------------------------------------------------
    # Detection helpers
    # ----------------------------------------------------------------------
    def _select_detection_from_msg(self, msg: Yolov8Inference, seq: int) -> Optional[TrackedDetection]:
        if msg is None or len(msg.results) == 0:
            return None

        candidates: List[TrackedDetection] = []
        now = self._now_sec()

        for r in msg.results:
            det = TrackedDetection(
                seq=seq,
                stamp_sec=now,
                class_name=str(r.class_name),
                confidence=float(r.confidence),
                u=float(r.center_x),
                v=float(r.center_y),
                width=float(r.width),
                height=float(r.height),
                angle=float(r.angle),
            )
            if self._is_detection_valid(det):
                candidates.append(det)

        if not candidates:
            return None

        if self.locked_det is None and self.last_visible_det is None:
            return max(candidates, key=lambda d: d.confidence)

        ref = self.last_visible_det or self.locked_det
        filtered = candidates
        if ref is not None:
            same_class = [d for d in candidates if d.class_name == ref.class_name]
            if same_class:
                filtered = same_class

            nearest = min(filtered, key=lambda d: math.hypot(d.u - ref.u, d.v - ref.v))
            dist = math.hypot(nearest.u - ref.u, nearest.v - ref.v)
            if dist <= float(self.get_parameter("track_px_gate").value):
                return nearest

            return None

        return max(candidates, key=lambda d: d.confidence)

    def _is_detection_valid(self, det: TrackedDetection) -> bool:
        target_class = str(self.get_parameter("target_class_name").value)
        if target_class and det.class_name != target_class:
            return False
        if det.confidence < self.min_confidence:
            return False
        if det.width < float(self.get_parameter("min_obb_width_px").value):
            return False
        if det.height < float(self.get_parameter("min_obb_height_px").value):
            return False
        if det.width > float(self.get_parameter("max_obb_width_px").value):
            return False
        if det.height > float(self.get_parameter("max_obb_height_px").value):
            return False
        if self.image_width is not None and self.image_height is not None:
            margin = float(self.get_parameter("edge_margin_px").value)
            if det.u < margin or det.u > (self.image_width - margin):
                return False
            if det.v < margin or det.v > (self.image_height - margin):
                return False
        return True

    def _same_detection(self, a: TrackedDetection, b: TrackedDetection, gate_px: float) -> bool:
        if a.class_name != b.class_name:
            return False
        return math.hypot(a.u - b.u, a.v - b.v) <= gate_px

    def _get_fresh_valid_detection(self) -> Optional[TrackedDetection]:
        det = self.latest_valid_det
        if det is None:
            return None
        age = self._now_sec() - det.stamp_sec
        if age > self.det_timeout:
            return None
        return det

    # ----------------------------------------------------------------------
    # Rough target only for pregrasp
    # ----------------------------------------------------------------------
    def _observe_target_in_base_rough(self, det: TrackedDetection) -> Optional[np.ndarray]:
        debug = list(self.get_parameter("debug_target").value)
        if len(debug) == 3 and any(abs(float(v)) > 1e-9 for v in debug):
            target_debug = np.array(debug, dtype=float)
            self.get_logger().info(
                f"use debug_target as rough target: {target_debug.tolist()}"
            )
            return target_debug

        if self.depth_img is None:
            self.get_logger().warning("rough target unavailable: no depth image")
            return None
        if not self.estimator.has_intrinsics():
            self.get_logger().warning("rough target unavailable: camera intrinsics not ready")
            return None

        xyz_cam = self.estimator.pixel_to_camera(det.u, det.v, self.depth_img)
        if xyz_cam is None:
            self.get_logger().warning("rough target unavailable: pixel_to_camera failed")
            return None

        pt = PointStamped()
        pt.header.frame_id = self.camera_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(xyz_cam[0])
        pt.point.y = float(xyz_cam[1])
        pt.point.z = float(xyz_cam[2])

        try:
            pt_base = self.tf_buffer.transform(
                pt, self.base_frame, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warning(f"rough target TF camera->base failed: {e}")
            return None

        target = np.array([pt_base.point.x, pt_base.point.y, pt_base.point.z], dtype=float)

        if bool(self.get_parameter("debug_print_rough_target").value):
            self.get_logger().info(
                "rough target in base, for coarse planning only | "
                f"x={target[0]:+.4f} y={target[1]:+.4f} z={target[2]:+.4f}"
            )

        return target

    def _compute_pregrasp_goal(self, rough_target_base: np.ndarray) -> np.ndarray:
        """Pregrasp 在 target 上方,按"图像像素分离最大化"挑选 XY 方向偏移,
        避免 EE 投影压在 target_uv 上(自遮挡)."""
        z_strategy = str(self.get_parameter("z_strategy").value)
        if z_strategy == "table":
            target_z = float(self.get_parameter("table_z").value) + \
                       float(self.get_parameter("pregrasp_height_above_table").value)
        else:
            target_z = rough_target_base[2] + self.hover_offset_z

        base_xy = np.array([rough_target_base[0], rough_target_base[1]])
        default_goal = np.array([base_xy[0], base_xy[1], target_z], dtype=float)

        offset_dist = float(self.get_parameter("pregrasp_camera_offset").value)
        if offset_dist < 1e-6:
            return default_goal

        target_uv = self._project_base_point_to_pixel(rough_target_base)
        if target_uv is None:
            return self._fallback_pregrasp_along_camera_ray(
                rough_target_base, target_z, offset_dist)

        n_dirs = int(self.get_parameter("pregrasp_candidate_dirs").value)
        candidates = []
        for i in range(n_dirs):
            ang = 2.0 * math.pi * i / n_dirs
            ux, uy = math.cos(ang), math.sin(ang)
            cand = np.array([
                base_xy[0] + offset_dist * ux,
                base_xy[1] + offset_dist * uy,
                target_z,
            ], dtype=float)
            cand_uv = self._project_base_point_to_pixel(cand)
            if cand_uv is None:
                continue
            sep = math.hypot(cand_uv[0] - target_uv[0], cand_uv[1] - target_uv[1])
            candidates.append((sep, ang, cand, cand_uv))

        if not candidates:
            self.get_logger().warning(
                "no valid pregrasp candidate found, fallback to camera-ray offset"
            )
            return self._fallback_pregrasp_along_camera_ray(
                rough_target_base, target_z, offset_dist)

        candidates.sort(key=lambda x: -x[0])
        best_sep, best_ang, best_goal, best_uv = candidates[0]

        min_sep = float(self.get_parameter("pregrasp_min_pixel_separation").value)
        if best_sep < min_sep:
            self.get_logger().warning(
                f"best pixel separation {best_sep:.0f}px < min {min_sep:.0f}px, "
                f"camera angle may be too steep; consider larger pregrasp_camera_offset"
            )

        self.get_logger().info(
            f"pregrasp selected | dir_deg={math.degrees(best_ang):+.0f} "
            f"sep_px={best_sep:.0f} target_uv=({target_uv[0]:.0f},{target_uv[1]:.0f}) "
            f"pregrasp_uv=({best_uv[0]:.0f},{best_uv[1]:.0f}) "
            f"goal=({best_goal[0]:+.3f},{best_goal[1]:+.3f},{best_goal[2]:+.3f})"
        )
        return best_goal

    def _fallback_pregrasp_along_camera_ray(self, rough_target_base: np.ndarray,
                                             target_z: float,
                                             offset_dist: float) -> np.ndarray:
        goal = np.array([rough_target_base[0], rough_target_base[1], target_z],
                        dtype=float)
        cam_xy = self._get_camera_xy_in_base()
        if cam_xy is not None:
            dx = rough_target_base[0] - cam_xy[0]
            dy = rough_target_base[1] - cam_xy[1]
            n = math.hypot(dx, dy)
            if n > 1e-6:
                goal[0] += offset_dist * dx / n
                goal[1] += offset_dist * dy / n
        self.get_logger().info(
            f"fallback pregrasp along camera ray: goal={goal.tolist()}"
        )
        return goal

    def _compute_grasp_goal_z(self, rough_target_base: np.ndarray) -> float:
        z_strategy = str(self.get_parameter("z_strategy").value)
        if z_strategy == "table":
            return float(self.get_parameter("table_z").value) + \
                   float(self.get_parameter("grasp_height_above_table").value)
        return float(rough_target_base[2] + self.descend_offset_z)

    # ----------------------------------------------------------------------
    # Adaptive Jacobian
    # ----------------------------------------------------------------------
    def _compute_j_img_to_base_live(self) -> Optional[np.ndarray]:
        if self.fx is None:
            return None
        try:
            tf_cam = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None
        q = tf_cam.transform.rotation
        R = np.array([
            [1-2*q.y*q.y-2*q.z*q.z, 2*q.x*q.y-2*q.z*q.w, 2*q.x*q.z+2*q.y*q.w],
            [2*q.x*q.y+2*q.z*q.w, 1-2*q.x*q.x-2*q.z*q.z, 2*q.y*q.z-2*q.x*q.w],
            [2*q.x*q.z-2*q.y*q.w, 2*q.y*q.z+2*q.x*q.w, 1-2*q.x*q.x-2*q.y*q.y],
        ], dtype=float)
        t_cam = np.array([tf_cam.transform.translation.x,
                        tf_cam.transform.translation.y,
                        tf_cam.transform.translation.z], dtype=float)
        ee_pose = self._lookup_ee_pose_in_base()
        if ee_pose is None:
            return None
        ee_base, _ = ee_pose

        # 关键: 雅可比也要在 "target 平面" 上算, 与 ref_uv 一致
        if (bool(self.get_parameter("use_target_plane_projection").value)
                and self.grasp_goal_z is not None):
            plane_z = self.grasp_goal_z + \
                    float(self.get_parameter("target_plane_z_offset").value)
            linearize_point = np.array([ee_base[0], ee_base[1], plane_z], dtype=float)
        else:
            linearize_point = ee_base

        p_cam = R.T @ (linearize_point - t_cam)
        x, y, z = p_cam[0], p_cam[1], p_cam[2]
        if z <= 1e-6:
            return None
        du_dcam = np.array([self.fx/z, 0.0, -self.fx*x/(z*z)])
        dv_dcam = np.array([0.0, self.fy/z, -self.fy*y/(z*z)])
        J_uv = np.array([
            [np.dot(du_dcam, R[0, :]), np.dot(du_dcam, R[1, :])],
            [np.dot(dv_dcam, R[0, :]), np.dot(dv_dcam, R[1, :])],
        ])
        try:
            return np.linalg.inv(J_uv)
        except np.linalg.LinAlgError:
            return None


    def _get_j_img_to_base(self) -> np.ndarray:
        if bool(self.get_parameter("use_adaptive_jacobian").value):
            live = self._compute_j_img_to_base_live()
            if live is not None:
                return live
        return self.j_img_to_base

    # ----------------------------------------------------------------------
    # Image error and controller
    # ----------------------------------------------------------------------
    def _compute_image_error_uv(self) -> Optional[Tuple[TrackedDetection, np.ndarray, np.ndarray]]:
        """fresh-detection 路径 (lock_target_during_alignment=False 时使用)."""
        det = self._get_fresh_valid_detection()
        if det is None:
            return None

        target_uv = np.array([det.u, det.v], dtype=float)
        ref_uv = self._compute_ee_reference_uv()
        if ref_uv is None:
            return None

        err_uv = target_uv - ref_uv
        return det, err_uv, ref_uv

    def _desired_center_uv(self) -> Optional[np.ndarray]:
        if self.cx is None or self.cy is None:
            return None
        u = float(self.get_parameter("desired_center_x").value)
        v = float(self.get_parameter("desired_center_y").value)
        if u < 0.0:
            u = self.cx
        if v < 0.0:
            v = self.cy
        return np.array([u, v], dtype=float)

    def _image_error_to_base_velocity(self, err_uv: np.ndarray,
                                      gain: float,
                                      max_step: float,
                                      max_vel: float) -> np.ndarray:
        J = self._get_j_img_to_base()
        delta_xy = gain * (J @ err_uv.reshape(2))
        delta_xy = limit_vector_norm(delta_xy, max_step)
        vxy = delta_xy / self.dt
        vxy = limit_vector_norm(vxy, min(max_vel, self.max_linear_vel))
        return vxy

    # ----------------------------------------------------------------------
    # TF and projection
    # ----------------------------------------------------------------------
    def _lookup_ee_pose_in_base(self) -> Optional[Tuple[np.ndarray, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warning(
                f"TF base->ee failed: {e}", throttle_duration_sec=1.0)
            return None

        p = tf.transform.translation
        q = tf.transform.rotation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        yaw = yaw_from_quat_xyzw(q.x, q.y, q.z, q.w)
        return pos, yaw

    def _get_camera_xy_in_base(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except Exception:
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y], dtype=float)

    def _project_base_point_to_pixel(self, point_base: np.ndarray) -> Optional[np.ndarray]:
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None

        pt = PointStamped()
        pt.header.frame_id = self.base_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(point_base[0])
        pt.point.y = float(point_base[1])
        pt.point.z = float(point_base[2])

        try:
            pt_cam = self.tf_buffer.transform(
                pt, self.camera_frame, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warning(
                f"TF base->camera projection failed: {e}",
                throttle_duration_sec=1.0,
            )
            return None

        x = float(pt_cam.point.x)
        y = float(pt_cam.point.y)
        z = float(pt_cam.point.z)

        if z <= 1e-6:
            self.get_logger().warning(
                f"cannot project point behind camera: cam=({x:+.3f},{y:+.3f},{z:+.3f})",
                throttle_duration_sec=1.0,
            )
            return None

        u = self.fx * x / z + self.cx
        v = self.fy * y / z + self.cy
        return np.array([u, v], dtype=float)

    # ----------------------------------------------------------------------
    # Yaw: short-edge selection for parallel gripper
    # ----------------------------------------------------------------------
    def _target_yaw_from_obb(self, obb_angle: float, w: float, h: float) -> float:
        angle = float(obb_angle)
        angle_is_long = bool(self.get_parameter("obb_angle_is_long_axis").value)
        grip_short = bool(self.get_parameter("grip_short_edge").value)

        if angle_is_long:
            long_axis = angle
        else:
            long_axis = angle if w >= h else (angle + math.pi / 2.0)

        if grip_short:
            gripper_yaw = long_axis
        else:
            gripper_yaw = long_axis + math.pi / 2.0

        sign = float(self.get_parameter("obb_to_gripper_yaw_sign").value)
        offset = float(self.get_parameter("obb_to_gripper_yaw_offset").value)
        return normalize_angle(sign * gripper_yaw + offset)

    # ----------------------------------------------------------------------
    # Twist / gripper / IK
    # ----------------------------------------------------------------------
    def _zero_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        return t

    def _fill_twist(self, twist: TwistStamped,
                    vx: float, vy: float, vz: float, wz: float) -> TwistStamped:
        lin = np.array([vx, vy, vz], dtype=float)
        lin = limit_vector_norm(lin, self.max_linear_vel)
        wz = clamp(wz, -self.max_angular_vel, self.max_angular_vel)

        if self.enable_motion:
            twist.twist.linear.x = float(lin[0])
            twist.twist.linear.y = float(lin[1])
            twist.twist.linear.z = float(lin[2])
            twist.twist.angular.z = float(wz)
        return twist

    def _send_gripper(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        t = float(self.get_parameter("gripper_motion_time").value)
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        msg.points.append(pt)
        self.gripper_pub.publish(msg)

    def _check_reachable(self, position_in_base: np.ndarray) -> bool:
        if not self.ik_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning("/compute_ik unavailable; skip reachability check")
            return True

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.move_group_name
        req.ik_request.pose_stamped.header.frame_id = self.base_frame
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose.position.x = float(position_in_base[0])
        req.ik_request.pose_stamped.pose.position.y = float(position_in_base[1])
        req.ik_request.pose_stamped.pose.position.z = float(position_in_base[2])
        req.ik_request.pose_stamped.pose.orientation.x = 1.0
        req.ik_request.pose_stamped.pose.orientation.y = 0.0
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 0.0
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = int(0.5 * 1e9)
        req.ik_request.avoid_collisions = self.avoid_collisions

        future = self.ik_client.call_async(req)
        if not self._wait_future(future, timeout_sec=1.0):
            self.get_logger().warning("IK service call timed out")
            return False

        result = future.result()
        if result is None:
            return False

        code = int(result.error_code.val)
        if code != 1:
            self.get_logger().warning(f"IK failed: error_code={code}")
            return False
        return True

    def _wait_future(self, future, timeout_sec: float) -> bool:
        import time
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return False

    # ----------------------------------------------------------------------
    # Runtime reset and clock
    # ----------------------------------------------------------------------
    def _reset_runtime_for_new_grasp(self):
        self.failed_reason = ""
        self.detect_seed = None
        self.detect_stable_count = 0
        self.detect_last_seq = -1
        self.xy_stable_count = 0
        self.xy_last_seq = -1
        self.yaw_stable_count = 0
        self.yaw_last_seq = -1
        self.lock_check_count = 0
        self.lock_check_last_seq = -1
        self.lock_check_retry_count = 0
        self.verify_missing_count = 0
        self.verify_follow_count = 0
        self.verify_last_seq = -1
        self.locked_det = None
        self.locked_target_uv = None
        self.rough_target_base = None
        self.pregrasp_goal_base = None
        self.grasp_goal_z = None
        self.lift_goal_z = None
        self.grasp_start_time = None

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    node = VisualServoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
'''
ros2 run visual_servo visual_servo_node --ros-args \
  -p enable_motion:=true \
  -p z_strategy:="table" \
  -p table_z:=0.0 \
  -p grasp_height_above_table:=0.035 \
  -p pregrasp_height_above_table:=0.16 \
  -p lock_check_pixel_tolerance:=12.0 \
  -p descend_speed:=0.04 \
  -p descend_timeout:=30.0 \
  -p visual_align_timeout:=25.0
  
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger & ros2 topic pub /servo_trigger std_msgs/msg/Bool "data: true" --once


ros2 run visual_servo visual_servo_node --ros-args \
  -p enable_motion:=true \
  -p z_strategy:="table" \
  -p table_z:=0.0 \
  -p grasp_height_above_table:=0.030 \
  -p pregrasp_height_above_table:=0.16 \
  -p lock_check_pixel_tolerance:=12.0 \
  -p descend_speed:=0.04 \
  -p descend_timeout:=30.0 \
  -p visual_align_timeout:=25.0 \
  -p pregrasp_camera_offset:=0.03

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger & ros2 topic pub /servo_trigger std_msgs/msg/Bool "data: true" --once

ros2 run visual_servo visual_servo_node --ros-args \
  -p enable_motion:=true \
  -p z_strategy:="table" \
  -p table_z:=0.0 \
  -p grasp_height_above_table:=0.030 \
  -p pregrasp_height_above_table:=0.16 \
  -p lock_check_pixel_tolerance:=12.0 \
  -p descend_speed:=0.04 \
  -p descend_timeout:=30.0 \
  -p visual_align_timeout:=25.0 \
  -p pregrasp_camera_offset:=0.03 \
  -p lifting_timeout:=30.0 \
  -p lift_height:=0.10


'''
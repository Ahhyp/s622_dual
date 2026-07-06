#!/usr/bin/env python3
# -*- coding: utf-8 -*-


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
    DESCEND_WITH_FEEDBACK = 6
    GRASPING = 7
    LIFTING = 8
    VERIFY_GRASP = 9
    DONE = 10
    FAILED = 11

@dataclass
class TrackedDetection:
    seq: int            # 检测消息序号（自增，区分不同帧）
    stamp_sec: float    # 收到检测的时间戳（秒）
    class_name: str     # YOLO 检测类别名，如 "red_block"
    confidence: float
    u: float
    v: float
    width: float
    height: float
    angle: float

def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
    
def yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    # Standard yaw extraction from quaternion.
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def limit_vector_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > max_norm and n > 1e-12:
        return v * (max_norm / n)
    return v

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def normalize_angle_pi_symmetry(a: float) -> float:
    """For a parallel gripper, yaw and yaw+pi are often equivalent."""
    a = normalize_angle(a)
    if a > math.pi / 2.0:
        a -= math.pi
    elif a < -math.pi / 2.0:
        a += math.pi
    return a

class a(Node):
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

        # State timeouts, seconds
        self.declare_parameter("detecting_timeout", 5.0)
        self.declare_parameter("coarse_planning_timeout", 20.0)
        self.declare_parameter("visual_align_timeout", 10.0)
        self.declare_parameter("yaw_align_timeout", 6.0)
        self.declare_parameter("descend_timeout", 10.0)
        self.declare_parameter("lifting_timeout", 8.0)
        self.declare_parameter("verify_timeout", 3.0)

        # ------------------------------------------------------------------
        # Frames
        # ------------------------------------------------------------------
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("ee_frame", "grasp_frame")

        # ------------------------------------------------------------------
        # YOLO detection gating
        # ------------------------------------------------------------------
        self.declare_parameter("target_class_name", "")  # empty means any class
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("det_timeout", 0.50)
        self.declare_parameter("stable_detection_frames", 5)
        self.declare_parameter("same_target_px_gate", 35.0)
        self.declare_parameter("track_px_gate", 80.0)
        self.declare_parameter("edge_margin_px", 25.0)
        self.declare_parameter("min_obb_width_px", 5.0)
        self.declare_parameter("min_obb_height_px", 5.0)
        self.declare_parameter("max_obb_width_px", 10000.0)
        self.declare_parameter("max_obb_height_px", 10000.0)

        # ------------------------------------------------------------------
        # Rough 3D estimate for coarse MoveIt planning ONLY
        # ------------------------------------------------------------------
        self.declare_parameter("debug_target", [0.0, 0.0, 0.0])
        self.declare_parameter("hover_offset_z", 0.12)
        self.declare_parameter("coarse_xy_margin", 0.00)
        self.declare_parameter("move_group_name", "robot_arm")
        self.declare_parameter("joint_names", ["j1", "j2", "j3", "j4", "j5", "j6"])

        # Final z strategy. "rough_target" uses rough_target.z + offsets.
        # "table" uses table_z + heights, often better in Gazebo.
        self.declare_parameter("z_strategy", "rough_target")  # rough_target/table
        self.declare_parameter("table_z", 0.0)
        self.declare_parameter("pregrasp_height_above_table", 0.16)
        self.declare_parameter("grasp_height_above_table", 0.025)
        self.declare_parameter("descend_offset_z", 0.025)
        self.declare_parameter("lift_height", 0.16)

        # ------------------------------------------------------------------
        # Image-space visual servo
        # ------------------------------------------------------------------
        # fixed_external: use target_uv - projected_grasp_frame_uv.
        # image_center: use target_uv - desired image center.
        self.declare_parameter("visual_reference_mode", "ee_projection")
        self.declare_parameter("desired_center_x", -1.0)  # <0 means camera cx
        self.declare_parameter("desired_center_y", -1.0)  # <0 means camera cy
        self.declare_parameter("grasp_projection_offset_px", [0.0, 0.0])

        # Matrix maps image error [du,dv] to small base XY displacement [dx,dy].
        # Unit: meter / pixel. This MUST be calibrated for reliable behavior.
        # Positive convention:
        #   error_uv = target_uv - reference_uv
        #   delta_base_xy = J_img_to_base * error_uv
        self.declare_parameter("j_img_to_base", [0.00035, 0.0,
                                                  0.0, 0.00035])
        self.declare_parameter("visual_align_gain", 0.45)
        self.declare_parameter("visual_align_pixel_tolerance", 8.0)
        self.declare_parameter("visual_align_max_step", 0.0015)
        self.declare_parameter("visual_align_max_vel", 0.045)
        self.declare_parameter("visual_align_stable_frames", 6)
        self.declare_parameter("visual_align_derivative_alpha", 0.0)

        # In descend, only small lateral corrections are allowed.
        self.declare_parameter("descend_xy_gain", 0.20)
        self.declare_parameter("descend_xy_pixel_tolerance", 12.0)
        self.declare_parameter("descend_xy_max_step", 0.0006)
        self.declare_parameter("descend_xy_max_vel", 0.018)
        self.declare_parameter("descend_speed", 0.018)
        self.declare_parameter("descend_lost_policy", "continue_slow")  # continue_slow/hold/fail
        self.declare_parameter("descend_lost_hold_time", 0.30)
        self.declare_parameter("descend_lost_continue_time", 2.00)
        self.declare_parameter("descend_lost_speed", 0.008)

        # ------------------------------------------------------------------
        # Yaw alignment using YOLO OBB angle
        # ------------------------------------------------------------------
        self.declare_parameter("obb_to_gripper_yaw_sign", 1.0)
        self.declare_parameter("obb_to_gripper_yaw_offset", 0.0)
        self.declare_parameter("yaw_pi_symmetry", True)
        self.declare_parameter("yaw_gain", 1.2)
        self.declare_parameter("yaw_tolerance", 0.08)
        self.declare_parameter("yaw_stable_frames", 4)

        # ------------------------------------------------------------------
        # Gripper and verification
        # ------------------------------------------------------------------
        self.declare_parameter("gripper_topic", "/hand_controller/joint_trajectory")
        self.declare_parameter("gripper_joint_names", ["finger1_joint", "finger2_joint"])
        self.declare_parameter("gripper_open_pos", [0.025, -0.025])
        self.declare_parameter("gripper_close_pos", [0.0, 0.0])
        self.declare_parameter("gripper_motion_time", 1.0)
        self.declare_parameter("grasp_duration", 1.5)
        self.declare_parameter("enable_verify", True)
        self.declare_parameter("verify_missing_frames", 5)
        self.declare_parameter("verify_follow_px", 45.0)

        # Debug logging
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
        self.last_visible_error_uv: Optional[np.ndarray] = None
        self.last_visible_error_time: Optional[float] = None

        # Detection stability
        # detect_seed 是稳定检测的"锚点"——在 DETECTING 阶段用来判断"是不是同一个物体"的参考帧。
        self.detect_seed: Optional[TrackedDetection] = None
        self.detect_stable_count = 0
        self.detect_last_seq = -1

        # Alignment stability
        self.xy_stable_count = 0
        self.xy_last_seq = -1
        self.yaw_stable_count = 0
        self.yaw_last_seq = -1

        # Verification
        self.verify_missing_count = 0
        self.verify_follow_count = 0
        self.verify_last_seq = -1

        # Locked execution target and goals
        self.locked_det: Optional[TrackedDetection] = None
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

        self.get_logger().info(
            "closed-loop visual_servo started | "
            f"rate={self.control_rate:.1f}Hz motion={self.enable_motion} "
            f"ref_mode={self.get_parameter('visual_reference_mode').value} "
            f"J={self.j_img_to_base.tolist()}"
        )
    
    def cb_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.image_width = int(msg.width)
        self.image_height = int(msg.height)
        self.estimator.set_intrinsics(self.fx, self.fy, self.cx, self.cy)

    def cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_stamp = msg.header.stamp
        
    def cb_det(self, msg: Yolov8Inference): # 未完成
        self.det_seq += 1
        self.latest_det_msg = msg
        self.latest_det_time = self._now_sec()

        det = self._select_detection_from_msg(msg, self.det_seq)
        self.latest_valid_det = det
        
    
    def cb_joint_state(self, msg: JointState):
        self.latest_joint_state = msg
        
    def cb_trigger(self, msg: Bool):
        '''
        
        '''
        if msg.data:
            # 触发了!!!!
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
    
    def control_loop(self):
        twist = self._zero_twist()

        if self.state == ServoState.IDLE:
            pass
        elif self.state == ServoState.DETECTING:
            self._state_detecting()
        elif self.state == ServoState.COARSE_PLANNING:
            self._check_state_timeout("coarse_planning_timeout", "coarse planning timeout")
        elif self.state == ServoState.MOVING_TO_PREGRASP:
            # MoveItPlanner.plan_to_pregrasp is expected to block until execution finishes.
            # This transient state is still useful for logs and timeout monitoring.
            self._check_state_timeout("coarse_planning_timeout", "moving to pregrasp timeout")
        elif self.state == ServoState.VISUAL_ALIGN_XY:
            twist = self._state_visual_align_xy()
        elif self.state == ServoState.VISUAL_ALIGN_YAW:
            twist = self._state_visual_align_yaw()
        elif self.state == ServoState.DESCEND_WITH_FEEDBACK:
            twist = self._state_descend_with_feedback()
            
    
    def _state_visual_align_yaw(self) -> TwistStamped:
        """
        每一步的作用：
        1. 拿当前末端朝向
            ee_pose = self._lookup_ee_pose_in_base()
            _, current_yaw = ee_pose
            从 TF 查 grasp_frame 在 base_link 下的位姿，提取 yaw。

        2. 算目标朝向
            target_yaw = self._target_yaw_from_obb(det.angle)
            从 YOLO OBB 的 angle 算出目标 yaw。_target_yaw_from_obb 里做了 sign * angle + offset，允许反转和固定偏置。

        3. 算误差 + π对称
            yaw_error = normalize_angle(target_yaw - current_yaw)
            if yaw_pi_symmetry:
                yaw_error = normalize_angle_pi_symmetry(yaw_error)
            normalize_angle：把角度归到 [-π, π]。
            normalize_angle_pi_symmetry：平行夹爪关一个 yaw 抓和转 180° 再抓是一样的，所以把误差折叠到 [-π/2, π/2]：
            yaw_error = 170°  → 刚超 90° → 归化到 -10°（转 10° 就行，不用转 170°）
            yaw_error = -150° → 小于 -90° → 归化到 +30°
        
        4. 稳定判定
            if det.seq != self.yaw_last_seq:
                self.yaw_last_seq = det.seq
                if abs(yaw_error) <= 0.08:    # tol=0.08 rad ≈ 4.6°
                    yaw_stable_count += 1
                else:
                    yaw_stable_count = 0
            每新帧才计数（det.seq 去重），连续 N 帧误差都在 0.08 rad 以内才算对齐完成。到齐后 → DESCEND_WITH_FEEDBACK。

        5. 纯 P 控制
            wz = clamp(1.2 * yaw_error, -max_angular_vel, max_angular_vel)
            gain=1.2，限幅 max_angular_vel=0.45 rad/s。没有 D 项、没有位移限幅——就是比例控制。误差大时转得快，快对齐时自然减速。

        整体流程
            每次 control_loop 调一次（50Hz）：

            TF 查末端 yaw → YOLO 拿 OBB angle → 算误差 → π折叠
                → 新帧？→ 误差 < 0.08rad？→ 稳定计数+1
                → 够 N 帧了？→ 进降速阶段
                → 没够？→ 发 wz = 1.2 × yaw_error 继续转
        """
        twist = self._zero_twist()

        if self._check_state_timeout("yaw_align_timeout", "yaw alignment timeout"):
            return twist

        det = self._get_fresh_valid_detection()
        if det is None:
            # Do not immediately fail. A short miss is common.
            return twist

        ee_pose = self._lookup_ee_pose_in_base()
        if ee_pose is None:
            return twist
        _, current_yaw = ee_pose

        target_yaw = self._target_yaw_from_obb(det.angle)
        yaw_error = normalize_angle(target_yaw - current_yaw)
        if bool(self.get_parameter("yaw_pi_symmetry").value):
            yaw_error = normalize_angle_pi_symmetry(yaw_error)

        tol = float(self.get_parameter("yaw_tolerance").value)
        if det.seq != self.yaw_last_seq:
            self.yaw_last_seq = det.seq
            if abs(yaw_error) <= tol:
                self.yaw_stable_count += 1
            else:
                self.yaw_stable_count = 0

        self.get_logger().info(
            f"[VISUAL_ALIGN_YAW] target={target_yaw:+.3f} current={current_yaw:+.3f} "
            f"err={yaw_error:+.3f} stable={self.yaw_stable_count}/"
            f"{int(self.get_parameter('yaw_stable_frames').value)}",
            throttle_duration_sec=0.25,
        )

        if self.yaw_stable_count >= int(self.get_parameter("yaw_stable_frames").value):
            self._enter_state(ServoState.DESCEND_WITH_FEEDBACK)
            return twist

        wz = clamp(float(self.get_parameter("yaw_gain").value) * yaw_error,
                   -self.max_angular_vel, self.max_angular_vel)
        return self._fill_twist(twist, vx=0.0, vy=0.0, vz=0.0, wz=wz)

    def _state_visual_align_xy(self) -> TwistStamped:
        twist = self._zero_twist()

        if self._check_state_timeout("visual_align_timeout", "visual XY alignment timeout"):
            return twist
        
        err_uv_pack = self._compute_image_error_uv()
        if err_uv_pack is None:
            return twist
        
        det, err_uv, ref_uv = err_uv_pack
        err_norm = float(np.linalg.norm(err_uv))
        tol = float(self.get_parameter("visual_align_pixel_tolerance").value)
        
        if det.seq != self.xy_last_seq:
            self.xy_last_seq = det.seq
            if err_norm <= tol:
                self.xy_stable_count += 1
            else:
                self.xy_stable_count = 0

        if bool(self.get_parameter("debug_print_servo").value):
            self.get_logger().info(
                f"[VISUAL_ALIGN_XY] e_uv=({err_uv[0]:+.1f},{err_uv[1]:+.1f}) "
                f"|e|={err_norm:.1f}px stable={self.xy_stable_count}/"
                f"{int(self.get_parameter('visual_align_stable_frames').value)} "
                f"ref=({ref_uv[0]:.1f},{ref_uv[1]:.1f})",
                throttle_duration_sec=0.25,
            )

        if self.xy_stable_count >= int(self.get_parameter("visual_align_stable_frames").value):
            self._enter_state(ServoState.VISUAL_ALIGN_YAW)
            return twist

        vxy = self._image_error_to_base_velocity(
            err_uv,
            gain=float(self.get_parameter("visual_align_gain").value),
            max_step=float(self.get_parameter("visual_align_max_step").value),
            max_vel=float(self.get_parameter("visual_align_max_vel").value),
        )

        return self._fill_twist(twist, vx=vxy[0], vy=vxy[1], vz=0.0, wz=0.0)
    
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
        
    def _image_error_to_base_velocity(self, err_uv: np.ndarray,
                                      gain: float,
                                      max_step: float,
                                      max_vel: float) -> np.ndarray:
        '''
        整体效果
            这是一个分段限幅的 P 控制器（只有比例项，无微分）：

            阶段	约束	作用
            gain	0.45	降低单帧步长，防止抖
            max_step	1.5mm	限制单帧位移上限
            max_vel	0.045 m/s	限制最终速度上限

        三步把像素误差变成基底座速度：
        err_uv (像素) → delta_xy (米) → vxy (米/秒)
        1. 像素 -> 基底坐标位移
            j_img_to_base 是一个 2×2 矩阵，把图像上的像素误差映射为 base_link 系的 XY 位移（米）。
            默认 [0.00035, 0; 0, 0.00035]，即 1 像素 ≈ 0.35 mm。 
            gain=0.45 是阻尼系数，避免一步跨太大。
            err_uv = (-38, -46) 像素
            delta_xy = 0.45 × [[0.00035, 0], [0, 0.00035]] × [-38, -46]
                     = 0.45 × [-0.0133, -0.0161]
                     = [-0.006, -0.0072] 米 = 往左上挪 6-7mm
        2. 限幅位移
            单步最多移 max_step 米（VISUAL_ALIGN 里默认 1.5mm）。防止单帧跨太大导致震荡。
        3. 位移 → 速度
            dt=0.02s（50Hz），位移 / 时间 = 速度。再限幅到 max_vel（默认 0.045 m/s）。
            vxy = [-0.006, -0.0072] / 0.02 = [-0.3, -0.36] m/s
            → 超限 → 缩到 max_vel=0.045 m/s
            → 实际输出 ≈ [-0.032, -0.032] m/s
        '''        
        delta_xy = gain * (self.j_img_to_base @ err_uv.reshape(2))
        delta_xy = limit_vector_norm(delta_xy, max_step)
        vxy = delta_xy / self.dt
        vxy = limit_vector_norm(vxy, min(max_vel, self.max_linear_vel))
        return vxy
    
    def _compute_image_error_uv(self) -> Optional[Tuple[TrackedDetection, np.ndarray, np.ndarray]]:
        det = self._get_fresh_valid_detection()
        if det is None:
            return None

        target_uv = np.array([det.u, det.v], dtype=float)
        # 决定了 图像误差的"零参考点"是谁——即末端应该追到图像里哪个位置
        mode = str(self.get_parameter("visual_reference_mode").value)

        '''
        ee_projection	参考点:grasp_frame 在图像上的正向投影位置	err_uv = target_uv - projected_ee_uv	固定外置相机（你的场景）
        image_center	参考点:图像中心 (cx, cy)	                err_uv = target_uv - center_uv	 眼在手上（camera 装在 EE 上）
        '''
        if mode == "ee_projection":
            ee_pose = self._lookup_ee_pose_in_base()
            if ee_pose is None:
                return None
            ee_pos, _ = ee_pose
            ref_uv = self._project_base_point_to_pixel(ee_pos)
            if ref_uv is None:
                return None
            offset = list(self.get_parameter("grasp_projection_offset_px").value)
            if len(offset) == 2:
                ref_uv = ref_uv + np.array([float(offset[0]), float(offset[1])], dtype=float)
        elif mode == "image_center":
            ref_uv = self._desired_center_uv()
            if ref_uv is None:
                return None
        else:
            self.get_logger().warning(
                f"unknown visual_reference_mode={mode}; use ee_projection",
                throttle_duration_sec=1.0,
            )
            ee_pose = self._lookup_ee_pose_in_base()
            if ee_pose is None:
                return None
            ref_uv = self._project_base_point_to_pixel(ee_pose[0])
            if ref_uv is None:
                return None

        err_uv = target_uv - ref_uv
        self.last_visible_error_uv = err_uv.copy()
        self.last_visible_error_time = self._now_sec()
        return det, err_uv, ref_uv
    
    def _desired_center_uv(self) -> Optional[np.ndarray]:
        '''
        返回 图像里"你希望物体最终出现在哪里"的像素坐标。默认就是相机光心 (cx, cy) = (320, 240)。
        但你的场景用不上这个函数——只有 visual_reference_mode == "image_center" 时才走它（第 1082 行），你用的是
        "ee_projection" 模式。image_center 模式是给眼在手上的相机用的：移动 EE 让物体保持在画面中央。相机固定、物
        体不动时，追图像中心没意义。
        '''
        if self.cx is None or self.cy is None:
            return None
        u = float(self.get_parameter("desired_center_x").value)
        v = float(self.get_parameter("desired_center_y").value)
        if u < 0.0:
            u = self.cx
        if v < 0.0:
            v = self.cy
        return np.array([u, v], dtype=float)

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
    
    def _check_state_timeout(self, param_name: str, reason: str) -> bool:
        timeout = float(self.get_parameter(param_name).value)
        if self._now_sec() - self.state_entry_time > timeout:
            self._fail(reason)
            return True
        return False
            
    def _launch_coarse_planning_once(self):
        #   self.rough_target_base, self.pregrasp_goal_base, self.target_yaw
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
    
    def _state_detecting(self):
        if self._check_state_timeout("detecting_timeout", "detecting timeout"):
            return

        det = self._get_fresh_valid_detection()
        if det is None:
            return 
        
        if det.seq == self.detect_last_seq:
            return 
        self.detect_last_seq = det.seq

        if self.detect_seed == None:
            self.detect_seed = det
            self.detect_last_seq = 1
        else:
            same = self._same_detection(self.detect_seed, det, float(self.get_parameter("same_target_px_gate").value))
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
        self.target_yaw = self._target_yaw_from_obb(self.locked_det.angle)
        self.rough_target_base = self._observe_target_in_base_rough(self.locked_det)

        if self.rough_target_base is None:
            self._fail("stable detection found, but rough pixel/depth/TF target is unavailable")
            return
        
        self.pregrasp_goal_base = self._compute_pregrasp_goal(self.rough_target_base)
        # 盲降终点 z, 	_state_descend_with_feedback 判断"降够了没有"
        self.grasp_goal_z = self._compute_grasp_goal_z(self.rough_target_base)
        self.lift_goal_z = self.grasp_goal_z + self.lift_height
        
        if not self._check_reachable(self.pregrasp_goal_base):
            self._fail("pregrasp is not reachable")
            return

        self.get_logger().info(
            "stable target locked for coarse planning only | "
            f"rough=({self.rough_target_base[0]:+.4f},"
            f"{self.rough_target_base[1]:+.4f},"
            f"{self.rough_target_base[2]:+.4f}) "
            f"pregrasp=({self.pregrasp_goal_base[0]:+.4f},"
            f"{self.pregrasp_goal_base[1]:+.4f},"
            f"{self.pregrasp_goal_base[2]:+.4f}) "
            f"target_yaw={self.target_yaw:+.3f}"
        )
        self._enter_state(ServoState.COARSE_PLANNING)      
        
    def _compute_pregrasp_goal(self, rough_target_base: np.ndarray) -> Optional[np.ndarray]:
        margin = float(self.get_parameter("coarse_xy_margin").value)
        z_strategy = str(self.get_parameter("z_strategy").value)

        goal = np.array(rough_target_base, dtype=float)
        # optional margin hook, useful if you later want to bias the pregrasp.
        goal[0] += 0.0 * margin
        goal[1] += 0.0 * margin

        if z_strategy == "table":
            goal[2] = float(self.get_parameter("table_z").value) + \
                      float(self.get_parameter("pregrasp_height_above_table").value)
        else:
            goal[2] = rough_target_base[2] + self.hover_offset_z

        return goal
    
    def _compute_grasp_goal_z(self, rough_target_base: np.ndarray) -> float:
        z_strategy = str(self.get_parameter("z_strategy").value)
        if z_strategy == "table":
            return float(self.get_parameter("table_z").value) + \
                   float(self.get_parameter("grasp_height_above_table").value)
        return float(rough_target_base[2] + self.descend_offset_z)
        
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
        
        target = np.ndarray([pt_base.point.x, pt_base.point.y, pt_base.point.z], dtype=float)
        
        
        if bool(self.get_parameter("debug_print_rough_target").value):
            self.get_logger().info(
                "rough target in base, for coarse planning only | "
                f"x={target[0]:+.4f} y={target[1]:+.4f} z={target[2]:+.4f}"
            )
            
        return target
           
    def _target_yaw_from_obb(self, obb_angle:float) -> float:
        sign = float(self.get_parameter("obb_to_gripper_yaw_sign").value)
        offset = float(self.get_parameter("obb_to_gripper_yaw_offset").value)
        return normalize_angle(sign * float(obb_angle) + offset)
                
    def _get_fresh_valid_detection(self) -> Optional[TrackedDetection]: # 未完成
        det = self.latest_valid_det
        if det is None:
            return None
        age = self._now_sec() - det.stamp_sec
        if age > self.det_timeout:
            return None
        return det
    
    def _same_detection(self, a: TrackedDetection, b: TrackedDetection, gate_px: float) -> bool:
        if a.class_name != b.class_name:
            return False
        return math.hypot(a.u - b.u, a.v - b.v) <= gate_px
    
    def _zero_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        return t

    def _enter_state(self, new_state: ServoState):
        if new_state == self.state:
            return
        
        self.get_logger().info(f"state: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_entry_time = self._now_sec()
        
        if new_state == ServoState.IDLE:
            self._reset_runtime_for_new_grasp()
            
        if new_state == ServoState.DETECTING:
            self.detect_seed = None
            self.detect_stable_count = 0
            self.detect_last_seq = -1
            
        if new_state == ServoState.VISUAL_ALIGN_XY:
            self.xy_stable_count = 0
            self.xy_last_seq = -1

        if new_state == ServoState.VISUAL_ALIGN_YAW:
            self.yaw_stable_count = 0
            self.yaw_last_seq = -1

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
            
    def _is_detection_valid(self, det:TrackedDetection) -> bool:
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
    
    def _select_detection_from_msg(self, msg: Yolov8Inference, seq:int) -> Optional[TrackedDetection]:
        if msg is None or len(msg.result) == 0:
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
        
        # After locking / tracking, choose same class and nearest to last/locked target in image.
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

            # If it jumps too far, reject rather than switch to a different object.
            return None

        return max(candidates, key=lambda d: d.confidence)
        
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
    
    def _reset_runtime_for_new_grasp(self):
        self.failed_reason = ""
        self.detect_seed = None
        self.detect_stable_count = 0
        self.detect_last_seq = -1
        self.xy_stable_count = 0
        self.xy_last_seq = -1
        self.yaw_stable_count = 0
        self.yaw_last_seq = -1
        self.verify_missing_count = 0
        self.verify_follow_count = 0
        self.verify_last_seq = -1
        self.locked_det = None
        self.rough_target_base = None
        self.pregrasp_goal_base = None
        self.grasp_goal_z = None
        self.lift_goal_z = None
        self.last_visible_error_uv = None
        self.last_visible_error_time = None
        self.grasp_start_time = None
    
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9
    
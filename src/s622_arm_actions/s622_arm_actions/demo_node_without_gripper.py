#!/usr/bin/env python3
"""
Demo 节点（无夹爪版）
====================

用途：
  - 电源未连接夹爪时，单独验证机械臂 MoveIt 规划与执行链路
  - 通过 TF 获取当前末端位姿
  - 沿 base_frame 的 Z 方向做小距离下降
  - 可选返回原位

安全设计：
  1. start_demo 默认 false
     -> 节点启动后不会自动运动
     -> 使用 ~/start Trigger 服务手动触发

  2. execute_motion 默认 false
     -> 默认 Plan Only，只规划、不执行

  3. move_distance 默认 0.005 m
     -> 默认仅 5 mm

  4. return_to_origin 默认 false
     -> 第一次真机测试只做单次 5 mm，不自动返回

  5. max_execute_distance 默认 0.020 m
     -> 真机执行时，单次位移超过 20 mm 自动拒绝

  6. execute_motion=true 时禁止 demo_cycles=0
     -> 禁止真机无限循环

注意：
  当前 target.position.z -= move_distance
  表示沿 base_frame 的 -Z 方向移动，
  不是沿工具自身局部 Z 轴移动。

Plan Only：
  current pose -> target pose
  只验证这一段轨迹。
  因为机器人没有真正到达 target，所以不会继续规划“返回原位”。

Execute：
  current pose -> target pose
  如果 return_to_origin=true：
      target pose -> original pose
"""

import time as _time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import (
    ReentrantCallbackGroup,
    MutuallyExclusiveCallbackGroup,
)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from pymoveit2 import MoveIt2
import tf2_ros

from manipulation_common.utils.params import param
from manipulation_common.utils.pose_tools import PoseTools
from manipulation_common.task.abort_manager import AbortManager  # [2026-08-28] robotarm 式：真实等待执行结果（query_state + motion_suceeded）
from manipulation_common.planning.motion_executor import (
    MoveItMotion,
    PlanScoreConfig,
    PlannerSwitch,
)
from manipulation_common.planning.trajectory_scoring import select_best_path


class DemoNodeWithoutGripper(Node):
    """无夹爪 Demo：读取当前末端位姿并进行小距离 Z 微动。"""

    def __init__(self):
        super().__init__(
            "demo_node_without_gripper",
            automatically_declare_parameters_from_overrides=True,
        )

        # MoveIt / Action 等允许并行回调
        self.callback_group = ReentrantCallbackGroup()

        # Demo 启停逻辑使用互斥回调组
        self.control_cb_group = MutuallyExclusiveCallbackGroup()

        self._startup_ready_logged = False
        self._waiting_for_manual_start_logged = False

        self._demo_started = False
        self._demo_done = False
        self._cycle_count = 0

        # ─────────────────────────────────────────────
        # 参数
        # ─────────────────────────────────────────────
        self._load_params()

        # start_demo=true：
        #   节点 ready 后自动启动
        #
        # start_demo=false：
        #   等待 ~/start 服务
        self._start_requested = self.start_demo

        # ─────────────────────────────────────────────
        # TF
        # ─────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self,
        )

        # ─────────────────────────────────────────────
        # 位姿工具
        # ─────────────────────────────────────────────
        self.pose_tools = PoseTools(
            self,
            base_frame=self.base_frame,
        )

        # ─────────────────────────────────────────────
        # MoveIt
        # ─────────────────────────────────────────────
        self._setup_moveit()

        # ─────────────────────────────────────────────
        # 手动启动服务
        #
        # ros2 service call \
        #   /demo_node_without_gripper/start \
        #   std_srvs/srv/Trigger "{}"
        # ─────────────────────────────────────────────
        self._start_srv = self.create_service(
            Trigger,
            "~/start",
            self._on_start_request,
            callback_group=self.control_cb_group,
        )

        # ─────────────────────────────────────────────
        # Ready / start 状态轮询
        # ─────────────────────────────────────────────
        self._startup_timer = self.create_timer(
            self.startup_poll_period_sec,
            self._try_start_demo,
            callback_group=self.control_cb_group,
        )

        self.get_logger().info(
            "DemoNodeWithoutGripper initialized"
        )

        self.get_logger().info(
            f"  base_frame         = {self.base_frame}"
        )
        self.get_logger().info(
            f"  ee_frame           = {self.ee_frame}"
        )
        self.get_logger().info(
            f"  move_distance      = {self.move_distance:.4f} m"
        )
        self.get_logger().info(
            f"  execute_motion     = {self.execute_motion}"
        )
        self.get_logger().info(
            f"  return_to_origin   = {self.return_to_origin}"
        )
        self.get_logger().info(
            f"  demo_cycles        = {self.demo_cycles}"
        )
        self.get_logger().info(
            f"  start_demo         = {self.start_demo}"
        )

        if not self.execute_motion:
            self.get_logger().warning(
                "PLAN ONLY MODE: robot trajectory will NOT be executed."
            )

        if self.execute_motion:
            self.get_logger().warning(
                "EXECUTION MODE ENABLED: real robot motion is allowed."
            )

        if not self.start_demo:
            self.get_logger().info(
                "Manual start enabled. "
                "Call /demo_node_without_gripper/start when ready."
            )

    # ═══════════════════════════════════════════════════════════
    # 参数
    # ═══════════════════════════════════════════════════════════

    @staticmethod
    def _as_bool(value) -> bool:
        """
        安全解析 bool。

        避免：
            bool("false") == True
        """
        if isinstance(value, bool):
            return value

        if isinstance(value, str):
            return value.strip().lower() in (
                "1",
                "true",
                "yes",
                "on",
            )

        return bool(value)

    def _load_params(self):
        """加载配置参数。"""

        # ─────────────────────────────────────────────
        # MoveIt / frame
        # ─────────────────────────────────────────────
        self.arm_group_name = str(
            param(
                self,
                "arm_group_name",
                "robot_arm",
            )
        )

        self.base_frame = str(
            param(
                self,
                "base_frame",
                "base_link",
            )
        )

        self.ee_frame = str(
            param(
                self,
                "ee_frame",
                "grasp_frame",
            )
        )

        self.move_group_ns_fairino = str(
            param(
                self,
                "move_group_ns_fairino",
                "/move_group_fairino",
            )
        )

        self.move_group_ready_timeout_sec = float(
            param(
                self,
                "move_group_ready_timeout_sec",
                10.0,
            )
        )

        self.startup_poll_period_sec = float(
            param(
                self,
                "startup_poll_period_sec",
                1.0,
            )
        )

        # ─────────────────────────────────────────────
        # Planner
        # ─────────────────────────────────────────────
        self.planning_pipeline_id = PlannerSwitch.normalize_pipeline(
            str(
                param(
                    self,
                    "planning_pipeline_id",
                    "fairino",
                )
            )
        )

        self.planner_id = PlannerSwitch.normalize_planner(
            self.planning_pipeline_id,
            str(
                param(
                    self,
                    "planner_id",
                    "tube_birrt*",
                )
            ),
        )

        # ─────────────────────────────────────────────
        # MoveIt 运动学参数
        # ─────────────────────────────────────────────
        self.max_step_size = float(
            param(
                self,
                "max_step_size",
                0.05,
            )
        )

        self.arm_max_velocity = float(
            param(
                self,
                "arm_max_velocity",
                0.3,
            )
        )

        self.arm_max_acceleration = float(
            param(
                self,
                "arm_max_acceleration",
                0.3,
            )
        )

        self.allowed_planning_time = float(
            param(
                self,
                "allowed_planning_time",
                15.0,
            )
        )

        self.position_tolerance = float(
            param(
                self,
                "position_tolerance",
                0.005,
            )
        )

        self.orientation_tolerance = float(
            param(
                self,
                "orientation_tolerance",
                0.005,
            )
        )

        self.allowed_start_tolerance = float(
            param(
                self,
                "allowed_start_tolerance",
                0.1,
            )
        )

        self.action_delay = float(
            param(
                self,
                "action_delay",
                0.5,
            )
        )

        # ─────────────────────────────────────────────
        # Demo 专用限速
        # ─────────────────────────────────────────────
        self.demo_max_velocity = float(
            param(
                self,
                "demo_max_velocity",
                0.05,
            )
        )

        self.demo_max_acceleration = float(
            param(
                self,
                "demo_max_acceleration",
                0.05,
            )
        )

        # ─────────────────────────────────────────────
        # Demo 微动
        #
        # 注意：
        # 0.005 m = 5 mm
        # ─────────────────────────────────────────────
        self.move_distance = float(
            param(
                self,
                "move_distance",
                0.005,
            )
        )

        # 真机执行时单次允许的最大位移
        # 默认 20 mm
        self.max_execute_distance = float(
            param(
                self,
                "max_execute_distance",
                0.020,
            )
        )

        # ─────────────────────────────────────────────
        # 启动/执行安全开关
        # ─────────────────────────────────────────────

        # false = Plan Only
        self.execute_motion = self._as_bool(
            param(
                self,
                "execute_motion",
                False,
            )
        )

        # false = 节点启动后不自动跑 Demo
        self.start_demo = self._as_bool(
            param(
                self,
                "start_demo",
                False,
            )
        )

        # 第一次真机建议 false：
        # 只下降，不自动返回
        self.return_to_origin = self._as_bool(
            param(
                self,
                "return_to_origin",
                False,
            )
        )

        # ─────────────────────────────────────────────
        # Demo cycle
        # ─────────────────────────────────────────────
        self.demo_cycles = int(
            param(
                self,
                "demo_cycles",
                1,
            )
        )

        # ─────────────────────────────────────────────
        # 轨迹候选打分
        # ─────────────────────────────────────────────
        self.num_candidate_plans = int(
            param(
                self,
                "num_candidate_plans",
                5,
            )
        )

        self.wrist_weight = float(
            param(
                self,
                "wrist_weight",
                50.0,
            )
        )

        self.wrist_joint_indices = tuple(
            int(v)
            for v in param(
                self,
                "wrist_joint_indices",
                [2, 3, 4],
            )
        )

        # ─────────────────────────────────────────────
        # J2 constraint
        # ─────────────────────────────────────────────
        self.j2_constraint = {
            "joint_positions": [
                float(
                    param(
                        self,
                        "j2_constraint_position",
                        -1.5708,
                    )
                )
            ],
            "joint_names": ["j2"],
            "tolerance": float(
                param(
                    self,
                    "j2_constraint_tolerance",
                    1.5708,
                )
            ),
            "weight": 1.0,
        }

        self.get_logger().info("Params loaded")

    # ═══════════════════════════════════════════════════════════
    # MoveIt
    # ═══════════════════════════════════════════════════════════

    def _setup_moveit(self):
        """初始化 MoveIt2，仅机械臂。"""

        self.moveit2_arm = MoveIt2(
            node=self,
            joint_names=[
                "j1",
                "j2",
                "j3",
                "j4",
                "j5",
                "j6",
            ],
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.arm_group_name,
            ignore_new_calls_while_executing=False,
            callback_group=self.callback_group,
            move_group_namespace=self.move_group_ns_fairino,
        )

        self._configure_arm_planner(
            self.moveit2_arm
        )

        self._configure_arm_limits(
            self.moveit2_arm
        )

        self.arm_execute_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/robot_arm_controller/follow_joint_trajectory",
            callback_group=self.callback_group,
        )

        # controller manager client 只创建一次
        self._list_controllers_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
            callback_group=self.callback_group,
        )

        # [2026-08-28] robotarm 式修复：此前 abort=None → _wait 只 sleep 0.5s 返回 True
        # （假成功：abort/TIMED_OUT 也报 "✓ done"）。配 AbortManager 后 _wait 走
        # wait_idle_or_abort：轮询 query_state，动作进入 EXECUTING 后回到 IDLE 才算完成，
        # 结果由 motion_suceeded 判定——SUCCESS 绑定真实执行结果。
        self._abort_mgr = AbortManager(self, arm=self.moveit2_arm, gripper=None)

        self.motion = MoveItMotion(
            node=self,
            arm_clients={
                "fairino": self.moveit2_arm
            },
            default_client="fairino",
            gripper=None,
            pose_tools=self.pose_tools,
            abort=self._abort_mgr,
            select_best_path=select_best_path,
            score_cfg=PlanScoreConfig(
                num_candidates=self.num_candidate_plans,
                wrist_weight=self.wrist_weight,
                wrist_joint_indices=self.wrist_joint_indices,
            ),
            action_delay=self.action_delay,
            joint_constraint=self.j2_constraint,
        )

        self.get_logger().info(
            "MoveIt2 initialized "
            "(arm only, no gripper)"
        )

    def _configure_arm_planner(self, arm):
        arm.pipeline_id = self.planning_pipeline_id
        arm.planner_id = self.planner_id

    def _configure_arm_limits(self, arm):
        arm.max_step_size = self.max_step_size
        arm.max_velocity = self.arm_max_velocity
        arm.max_acceleration = self.arm_max_acceleration
        arm.allowed_planning_time = self.allowed_planning_time
        arm.position_tolerance = self.position_tolerance
        arm.orientation_tolerance = self.orientation_tolerance
        arm.allowed_start_tolerance = self.allowed_start_tolerance

    # ═══════════════════════════════════════════════════════════
    # 手动启动
    # ═══════════════════════════════════════════════════════════

    def _on_start_request(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        """接收手动 Demo 启动请求。"""

        del request

        if self._demo_started:
            response.success = False
            response.message = (
                "Demo is already running."
            )
            return response

        if self._demo_done:
            response.success = False
            response.message = (
                "Demo has already completed. "
                "Restart the node to run again."
            )
            return response

        self._start_requested = True

        if self.execute_motion:
            self.get_logger().warning(
                "Manual START requested "
                "with execute_motion=true."
            )
        else:
            self.get_logger().info(
                "Manual START requested "
                "in Plan Only mode."
            )

        response.success = True
        response.message = (
            "Start request accepted. "
            "Demo will begin when MoveIt/TF/controller "
            "requirements are ready."
        )

        return response

    # ═══════════════════════════════════════════════════════════
    # Ready 检查
    # ═══════════════════════════════════════════════════════════

    def _try_start_demo(self):
        """
        等待：
          1. 用户 start request
          2. MoveIt ready
          3. TF ready
          4. execute 模式下 controller ready
        """

        if self._demo_started or self._demo_done:
            return

        # 没有收到 start 请求时，绝对不运行
        if not self._start_requested:
            if not self._waiting_for_manual_start_logged:
                self.get_logger().info(
                    "Demo armed but NOT started. "
                    "Waiting for manual ~/start request."
                )
                self._waiting_for_manual_start_logged = True
            return

        if not self._startup_ready():
            return

        if not self._validate_runtime_safety():
            self.get_logger().error(
                "Runtime safety validation failed. "
                "Demo will NOT start."
            )
            self._demo_done = True
            return

        self._demo_started = True

        mode = (
            "EXECUTE"
            if self.execute_motion
            else "PLAN ONLY"
        )

        self.get_logger().warning(
            "========================================"
        )
        self.get_logger().warning(
            f"DEMO STARTING: {mode}"
        )
        self.get_logger().warning(
            f"move_distance={self.move_distance:.4f} m"
        )
        self.get_logger().warning(
            f"return_to_origin={self.return_to_origin}"
        )
        self.get_logger().warning(
            "========================================"
        )

        self._run_demo_sequence()

    def _startup_ready(self) -> bool:
        """检查启动 Demo 所需的系统状态。"""

        arm_ready = self._service_ready(
            self.moveit2_arm,
            self.move_group_ready_timeout_sec,
        )

        tf_ready = self._tf_ready(
            timeout_sec=1.0
        )

        # Plan Only 不需要轨迹 Action Server
        # 和 robot_arm_controller 一定可执行
        if not self.execute_motion:
            ready = arm_ready and tf_ready

            if ready and not self._startup_ready_logged:
                self.get_logger().info(
                    "MoveIt + TF ready "
                    "(Plan Only mode)"
                )
                self._startup_ready_logged = True

            elif not ready:
                self.get_logger().info(
                    "Waiting for MoveIt / TF...",
                    throttle_duration_sec=5.0,
                )

            return ready

        # 真正执行时才严格要求控制器
        arm_exec_ready = self._action_ready(
            self.arm_execute_action,
            self.move_group_ready_timeout_sec,
        )

        controllers_ready = self._controllers_active(
            ("robot_arm_controller",),
            self.move_group_ready_timeout_sec,
        )

        ready = (
            arm_ready
            and tf_ready
            and arm_exec_ready
            and controllers_ready
        )

        if ready and not self._startup_ready_logged:
            self.get_logger().info(
                "MoveIt + TF + robot_arm_controller "
                "ready for EXECUTION"
            )
            self._startup_ready_logged = True

        elif not ready:
            self.get_logger().info(
                "Waiting for MoveIt / TF / "
                "arm controller...",
                throttle_duration_sec=5.0,
            )

        return ready

    def _service_ready(
        self,
        moveit_obj,
        timeout_sec: float,
    ) -> bool:
        cli = getattr(
            moveit_obj,
            "_plan_kinematic_path_service",
            None,
        )

        if cli is None:
            return True

        try:
            return bool(
                cli.wait_for_service(
                    timeout_sec=float(timeout_sec)
                )
            )
        except Exception:
            return False

    def _action_ready(
        self,
        action_client,
        timeout_sec: float,
    ) -> bool:
        try:
            return bool(
                action_client.wait_for_server(
                    timeout_sec=float(timeout_sec)
                )
            )
        except Exception:
            return False

    def _tf_ready(
        self,
        timeout_sec: float = 1.0,
    ) -> bool:
        """确认 base_frame -> ee_frame TF 可用。"""

        try:
            return bool(
                self._tf_buffer.can_transform(
                    self.base_frame,
                    self.ee_frame,
                    rclpy.time.Time(),
                    timeout=Duration(
                        seconds=float(timeout_sec)
                    ),
                )
            )

        except Exception:
            return False

    def _controllers_active(
        self,
        names: tuple[str, ...],
        timeout_sec: float,
    ) -> bool:
        """确认指定 ros2_control controller 为 active。"""

        client = self._list_controllers_client

        try:
            if not client.wait_for_service(
                timeout_sec=float(timeout_sec)
            ):
                return False

            future = client.call_async(
                ListControllers.Request()
            )

            deadline = (
                _time.time()
                + float(timeout_sec)
            )

            while rclpy.ok() and not future.done():

                if _time.time() >= deadline:
                    return False

                _time.sleep(0.05)

            if (
                not future.done()
                or future.result() is None
            ):
                return False

            states = {
                c.name: c.state
                for c in future.result().controller
            }

            return all(
                states.get(name) == "active"
                for name in names
            )

        except Exception as exc:
            self.get_logger().error(
                f"Controller state check failed: {exc}"
            )
            return False

    # ═══════════════════════════════════════════════════════════
    # Safety
    # ═══════════════════════════════════════════════════════════

    def _validate_runtime_safety(self) -> bool:
        """Demo 真正启动之前的参数安全检查。"""

        if self.move_distance <= 0.0:
            self.get_logger().error(
                "move_distance must be > 0."
            )
            return False

        # Plan Only 不会产生真实运动
        if not self.execute_motion:
            return True

        # ─────────────────────────────────────────────
        # 真机执行限制
        # ─────────────────────────────────────────────

        if (
            self.max_execute_distance > 0.0
            and self.move_distance
            > self.max_execute_distance
        ):
            self.get_logger().error(
                "Execution rejected: "
                f"move_distance={self.move_distance:.4f} m "
                f"> max_execute_distance="
                f"{self.max_execute_distance:.4f} m"
            )
            return False

        # 禁止真机无限循环
        if self.demo_cycles <= 0:
            self.get_logger().error(
                "Execution rejected: "
                "demo_cycles must be >= 1 "
                "when execute_motion=true."
            )
            return False

        # 如果不返回原位，多 cycle 会不断向 -Z 累加
        if (
            not self.return_to_origin
            and self.demo_cycles != 1
        ):
            self.get_logger().error(
                "Execution rejected: "
                "return_to_origin=false requires "
                "demo_cycles=1 to prevent "
                "cumulative Z motion."
            )
            return False

        return True

    # ═══════════════════════════════════════════════════════════
    # EE Pose
    # ═══════════════════════════════════════════════════════════

    def _get_current_ee_pose(
        self,
        timeout_sec: float = 2.0,
    ) -> Pose | None:
        """读取当前末端在 base_frame 下的实际 TF 位姿。"""

        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(
                    seconds=float(timeout_sec)
                ),
            )

            pose = Pose()

            pose.position.x = (
                tf.transform.translation.x
            )
            pose.position.y = (
                tf.transform.translation.y
            )
            pose.position.z = (
                tf.transform.translation.z
            )

            pose.orientation.x = (
                tf.transform.rotation.x
            )
            pose.orientation.y = (
                tf.transform.rotation.y
            )
            pose.orientation.z = (
                tf.transform.rotation.z
            )
            pose.orientation.w = (
                tf.transform.rotation.w
            )

            return pose

        except Exception as exc:
            self.get_logger().error(
                f"TF lookup failed "
                f"({self.base_frame} -> "
                f"{self.ee_frame}): {exc}"
            )

            return None

    @staticmethod
    def _make_z_target(
        current: Pose,
        dz: float,
    ) -> Pose:
        """
        构造目标位姿。

        dz：
          正值 -> base_frame +Z
          负值 -> base_frame -Z
        """

        target = Pose()

        target.position.x = current.position.x
        target.position.y = current.position.y
        target.position.z = (
            current.position.z + dz
        )

        target.orientation.x = (
            current.orientation.x
        )
        target.orientation.y = (
            current.orientation.y
        )
        target.orientation.z = (
            current.orientation.z
        )
        target.orientation.w = (
            current.orientation.w
        )

        return target

    # ═══════════════════════════════════════════════════════════
    # Demo
    # ═══════════════════════════════════════════════════════════

    def _run_demo_sequence(self):
        """运行 Demo。"""

        # ─────────────────────────────────────────────
        # PLAN ONLY
        #
        # 机器人不会真的移动，所以只规划：
        #
        # current -> target
        #
        # 不继续规划 return。
        # ─────────────────────────────────────────────
        if not self.execute_motion:
            self._run_plan_only()
            return

        # ─────────────────────────────────────────────
        # EXECUTE
        # ─────────────────────────────────────────────
        self._run_execute()

    def _run_plan_only(self):
        """只规划一次下降，不执行。"""

        current = self._get_current_ee_pose()

        if current is None:
            self.get_logger().error(
                "Cannot read current EE pose. "
                "Plan Only aborted."
            )
            self._demo_done = True
            return

        target = self._make_z_target(
            current,
            -self.move_distance,
        )

        self._log_pose_pair(
            current,
            target,
        )

        limits = self._cartesian_limits(
            plan_only=True
        )

        self.get_logger().info(
            "[PLAN ONLY] Planning Cartesian descent..."
        )

        success = self.motion.move_to_pose(
            target,
            action_name=(
                "demo plan-only: "
                f"descend {self.move_distance:.3f}m"
            ),
            cartesian=True,
            **limits,
        )

        self._demo_done = True

        if not success:
            self.get_logger().error(
                "PLAN ONLY failed."
            )
            return

        self.get_logger().info(
            "========================================"
        )
        self.get_logger().info(
            "PLAN ONLY completed successfully."
        )
        self.get_logger().info(
            "Robot was NOT commanded to move."
        )
        self.get_logger().info(
            "Inspect the planned trajectory before "
            "enabling execute_motion."
        )
        self.get_logger().info(
            "========================================"
        )

    def _run_execute(self):
        """执行真实机械臂微动。"""

        total = self.demo_cycles

        while (
            rclpy.ok()
            and self._cycle_count < total
        ):
            self._cycle_count += 1

            cycle_label = (
                f"[cycle {self._cycle_count}/{total}]"
            )

            # 每个 cycle 都重新读取真实当前位置
            current = self._get_current_ee_pose()

            if current is None:
                self.get_logger().error(
                    f"{cycle_label} "
                    "Cannot read current EE pose. "
                    "Execution aborted."
                )
                self._demo_done = True
                return

            target = self._make_z_target(
                current,
                -self.move_distance,
            )

            self._log_pose_pair(
                current,
                target,
            )

            limits = self._cartesian_limits(
                plan_only=False
            )

            # ─────────────────────────────────────
            # Step 1：下降
            # ─────────────────────────────────────
            self.get_logger().warning(
                f"{cycle_label} "
                f"EXECUTING Cartesian descent "
                f"{self.move_distance * 1000.0:.1f} mm"
            )

            success = self.motion.move_to_pose(
                target,
                action_name=(
                    f"demo {cycle_label}: "
                    f"descend "
                    f"{self.move_distance:.3f}m"
                ),
                cartesian=True,
                **limits,
            )

            if not success:
                self.get_logger().error(
                    f"{cycle_label} "
                    "Descent failed. "
                    "Demo aborted."
                )
                self._demo_done = True
                return

            self.get_logger().info(
                f"{cycle_label} "
                "Descent completed."
            )

            # 第一次真机测试：
            # 默认在这里结束。
            if not self.return_to_origin:
                self.get_logger().warning(
                    f"{cycle_label} "
                    "return_to_origin=false: "
                    "stopping after descent."
                )
                break

            # ─────────────────────────────────────
            # Step 2：返回原位
            # ─────────────────────────────────────
            self.get_logger().warning(
                f"{cycle_label} "
                "EXECUTING Cartesian return "
                "to original pose"
            )

            success = self.motion.move_to_pose(
                current,
                action_name=(
                    f"demo {cycle_label}: "
                    "return to origin"
                ),
                cartesian=True,
                **limits,
            )

            if not success:
                self.get_logger().error(
                    f"{cycle_label} "
                    "Return failed. "
                    "Demo aborted."
                )
                self._demo_done = True
                return

            self.get_logger().info(
                f"{cycle_label} "
                "Return completed."
            )

        self._demo_done = True

        self.get_logger().info(
            "========================================"
        )
        self.get_logger().info(
            f"Demo completed "
            f"({self._cycle_count} cycle(s))."
        )
        self.get_logger().info(
            "========================================"
        )

    # ═══════════════════════════════════════════════════════════
    # Motion helpers
    # ═══════════════════════════════════════════════════════════

    def _cartesian_limits(
        self,
        plan_only: bool,
    ) -> dict:
        return {
            "max_velocity": self.demo_max_velocity,
            "max_acceleration": self.demo_max_acceleration,
            "timeout_sec": 60.0,
            "plan_only": plan_only,
            **self._motion_limits_kwargs(),
        }

    def _motion_limits_kwargs(self) -> dict:
        return {
            "max_step_size": (
                self.max_step_size
            ),
            "allowed_planning_time": (
                self.allowed_planning_time
            ),
            "position_tolerance": (
                self.position_tolerance
            ),
            "orientation_tolerance": (
                self.orientation_tolerance
            ),
            "allowed_start_tolerance": (
                self.allowed_start_tolerance
            ),
        }

    def _log_pose_pair(
        self,
        current: Pose,
        target: Pose,
    ):
        self.get_logger().info(
            "Current EE pose: "
            f"x={current.position.x:.4f}, "
            f"y={current.position.y:.4f}, "
            f"z={current.position.z:.4f}"
        )

        self.get_logger().info(
            "Target EE pose:  "
            f"x={target.position.x:.4f}, "
            f"y={target.position.y:.4f}, "
            f"z={target.position.z:.4f}"
        )

        self.get_logger().info(
            f"Requested ΔZ = "
            f"{-self.move_distance * 1000.0:.1f} mm "
            f"in {self.base_frame}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = DemoNodeWithoutGripper()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info(
            "Keyboard interrupt received."
        )

    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

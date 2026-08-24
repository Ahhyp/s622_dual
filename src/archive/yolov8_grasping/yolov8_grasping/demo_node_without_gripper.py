#!/usr/bin/env python3
"""
Demo 节点（无夹爪版）：电源未连接夹爪时的纯机械臂微动演示。

复用 manipulation_common：
  - PoseTools         构建目标位姿
  - MoveItMotion      运动规划与执行
  - param()           参数加载
  - select_best_path  轨迹优选（多候选打分）

与 demo_node.py 的区别：
  - 去掉了夹爪 MoveIt2 客户端
  - 去掉了 AbortManager（依赖夹爪对象）
  - 不需要预设位姿：通过 TF 读取当前末端位姿，仅做 Z 轴微动

演示序列：
  1. 等待 MoveIt 就绪 + TF 就绪
  2. 读取末端当前位姿（TF：base_link → grasp_frame）
  3. 笛卡尔直线下降 move_distance 米
  4. 笛卡尔直线抬升回原位
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from pymoveit2 import MoveIt2
import tf2_ros

from manipulation_common.utils.params import param
from manipulation_common.utils.pose_tools import PoseTools
from manipulation_common.planning.motion_executor import (
    MoveItMotion,
    PlanScoreConfig,
    PlannerSwitch,
)
from manipulation_common.planning.trajectory_scoring import select_best_path


class DemoNodeWithoutGripper(Node):
    """无夹爪 Demo 节点：读取当前末端位姿，做 Z 轴微动。"""

    def __init__(self):
        super().__init__(
            "demo_node_without_gripper",
            automatically_declare_parameters_from_overrides=True,
        )

        self.callback_group = ReentrantCallbackGroup()
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        self._startup_ready_logged = False

        # ── 加载参数 ──
        self._load_params()

        # ── TF 监听（获取末端当前位姿）──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── 位姿工具（MoveItMotion.move_to_pose 需要 Pose → PoseStamped）──
        self.pose_tools = PoseTools(self, base_frame=self.base_frame)

        # ── 初始化 MoveIt ──
        self._setup_moveit()

        # ── 运动执行器（无夹爪、无中止管理器）──
        self.motion = MoveItMotion(
            node=self,
            arm_clients={"fairino": self.moveit2_arm},
            default_client="fairino",
            gripper=None,
            pose_tools=self.pose_tools,
            abort=None,
            select_best_path=select_best_path,
            score_cfg=PlanScoreConfig(
                num_candidates=self.num_candidate_plans,
                wrist_weight=self.wrist_weight,
                wrist_joint_indices=self.wrist_joint_indices,
            ),
            action_delay=self.action_delay,
            joint_constraint=self.j2_constraint,
        )

        # ── 启动 demo（延迟等待就绪）──
        self._demo_started = False
        self._demo_done = False
        self.create_timer(
            3.0, self._try_start_demo, callback_group=self.control_cb_group
        )
        self.get_logger().info(
            f"DemoNodeWithoutGripper initialized "
            f"(move_distance={self.move_distance:.3f}m, cycles={self.demo_cycles})"
        )

    # ═══════════════════════════════════════════════════════════
    #  参数加载
    # ═══════════════════════════════════════════════════════════

    def _load_params(self):
        """加载所有可配置参数。"""
        # 运动组
        self.arm_group_name = str(param(self, "arm_group_name", "robot_arm"))
        self.base_frame = str(param(self, "base_frame", "base_link"))
        self.ee_frame = str(param(self, "ee_frame", "grasp_frame"))
        self.move_group_ns_fairino = str(
            param(self, "move_group_ns_fairino", "/move_group_fairino")
        )
        self.move_group_ready_timeout_sec = float(
            param(self, "move_group_ready_timeout_sec", 10.0)
        )

        # 规划器
        self.planning_pipeline_id = PlannerSwitch.normalize_pipeline(
            str(param(self, "planning_pipeline_id", "fairino"))
        )
        self.planner_id = PlannerSwitch.normalize_planner(
            self.planning_pipeline_id,
            str(param(self, "planner_id", "tube_birrt*")),
        )

        # 运动学限速（与 visual_grasping_node 保持一致）
        self.max_step_size = float(param(self, "max_step_size", 0.05))
        self.arm_max_velocity = float(param(self, "arm_max_velocity", 0.3))
        self.arm_max_acceleration = float(param(self, "arm_max_acceleration", 0.3))
        self.allowed_planning_time = float(param(self, "allowed_planning_time", 15.0))
        self.position_tolerance = float(param(self, "position_tolerance", 0.005))
        self.orientation_tolerance = float(param(self, "orientation_tolerance", 0.005))
        self.allowed_start_tolerance = float(
            param(self, "allowed_start_tolerance", 0.1)
        )
        self.action_delay = float(param(self, "action_delay", 0.5))

        # 微动距离（米）
        self.move_distance = float(param(self, "move_distance", 0.05))

        # 轨迹打分
        self.num_candidate_plans = int(param(self, "num_candidate_plans", 5))
        self.wrist_weight = float(param(self, "wrist_weight", 50.0))
        self.wrist_joint_indices = tuple(
            int(v) for v in param(self, "wrist_joint_indices", [2, 3, 4])
        )

        # 演示循环次数（1 = 单次，0 = 无限循环）
        self.demo_cycles = int(param(self, "demo_cycles", 1))
        self._cycle_count = 0

        # J2 关节约束（可选）
        self.j2_constraint = {
            "joint_positions": [float(param(self, "j2_constraint_position", -1.5708))],
            "joint_names": ["j2"],
            "tolerance": float(param(self, "j2_constraint_tolerance", 1.5708)),
            "weight": 1.0,
        }

        self.get_logger().info("Params loaded")

    # ═══════════════════════════════════════════════════════════
    #  MoveIt 初始化（仅机械臂，无夹爪）
    # ═══════════════════════════════════════════════════════════

    def _setup_moveit(self):
        """初始化 MoveIt2 机械臂客户端（不含夹爪）。"""
        self.moveit2_arm = MoveIt2(
            node=self,
            joint_names=["j1", "j2", "j3", "j4", "j5", "j6"],
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.arm_group_name,
            ignore_new_calls_while_executing=False,
            callback_group=self.callback_group,
            move_group_namespace=self.move_group_ns_fairino,
        )
        self._configure_arm_planner(self.moveit2_arm)
        self._configure_arm_limits(self.moveit2_arm)

        self.arm_execute_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/robot_arm_controller/follow_joint_trajectory",
            callback_group=self.callback_group,
        )

        self.get_logger().info("MoveIt2 initialized (arm only, no gripper)")

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
    #  末端位姿查询
    # ═══════════════════════════════════════════════════════════

    def _get_current_ee_pose(self, timeout_sec: float = 2.0) -> Pose | None:
        """通过 TF 查询当前末端在 base_frame 下的位姿。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=float(timeout_sec)),
            )
            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y
            pose.position.z = tf.transform.translation.z
            pose.orientation = tf.transform.rotation
            return pose
        except Exception as exc:
            self.get_logger().error(f"TF lookup failed: {exc}")
            return None

    # ═══════════════════════════════════════════════════════════
    #  Demo 序列
    # ═══════════════════════════════════════════════════════════

    def _try_start_demo(self):
        """定时检查就绪状态，就绪后启动 demo 序列。"""
        if self._demo_started:
            return
        if not self._startup_ready():
            return
        self._demo_started = True
        self.get_logger().info(
            f"=== Demo sequence starting "
            f"(move_distance={self.move_distance:.3f}m, cycles={self.demo_cycles}) ==="
        )
        self._run_demo_sequence()

    def _startup_ready(self) -> bool:
        """检查 MoveIt 服务、Action Server、控制器是否就绪。"""
        arm_ready = self._service_ready(
            self.moveit2_arm, self.move_group_ready_timeout_sec
        )
        arm_exec_ready = self._action_ready(
            self.arm_execute_action, self.move_group_ready_timeout_sec
        )
        controllers_ready = self._controllers_active(
            ("robot_arm_controller",),
            self.move_group_ready_timeout_sec,
        )
        ready = arm_ready and arm_exec_ready and controllers_ready
        if ready and not self._startup_ready_logged:
            self.get_logger().info("MoveIt + robot_arm_controller ready")
            self._startup_ready_logged = True
        elif not ready:
            self.get_logger().info(
                "Waiting for MoveIt services/arm controller...",
                throttle_duration_sec=5.0,
            )
        return ready

    def _service_ready(self, moveit_obj, timeout_sec: float) -> bool:
        cli = getattr(moveit_obj, "_plan_kinematic_path_service", None)
        if cli is None:
            return True
        try:
            return bool(cli.wait_for_service(timeout_sec=float(timeout_sec)))
        except Exception:
            return False

    def _action_ready(self, action_client, timeout_sec: float) -> bool:
        try:
            return bool(action_client.wait_for_server(timeout_sec=float(timeout_sec)))
        except Exception:
            return False

    def _controllers_active(
        self, names: tuple[str, ...], timeout_sec: float
    ) -> bool:
        """通过 /controller_manager/list_controllers 检查控制器是否 active。"""
        import time as _time
        from controller_manager_msgs.srv import ListControllers

        client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
            callback_group=self.callback_group,
        )
        try:
            if not client.wait_for_service(timeout_sec=float(timeout_sec)):
                return False
            future = client.call_async(ListControllers.Request())
            deadline = _time.time() + float(timeout_sec)
            while rclpy.ok() and not future.done():
                if _time.time() >= deadline:
                    return False
                _time.sleep(0.05)
            if not future.done() or future.result() is None:
                return False
            states = {c.name: c.state for c in future.result().controller}
            return all(states.get(name) == "active" for name in names)
        except Exception:
            return False

    def _run_demo_sequence(self):
        """执行演示运动序列：下 → 上 微动循环。"""
        total = self.demo_cycles if self.demo_cycles > 0 else float("inf")

        # 读取当前末端位姿
        current = self._get_current_ee_pose()
        if current is None:
            self.get_logger().error("Cannot read current EE pose, demo aborted")
            self._demo_done = True
            return
        self.get_logger().info(
            f"Current EE pose: x={current.position.x:.3f}, "
            f"y={current.position.y:.3f}, z={current.position.z:.3f}"
        )

        # 构建下探目标位姿（Z 轴减去 move_distance）
        target = Pose()
        target.position.x = current.position.x
        target.position.y = current.position.y
        target.position.z = current.position.z - self.move_distance
        target.orientation = current.orientation

        self.get_logger().info(
            f"Target pose:     x={target.position.x:.3f}, "
            f"y={target.position.y:.3f}, z={target.position.z:.3f} "
            f"(Δz = -{self.move_distance:.3f}m)"
        )

        # 限速参数
        cartesian_limits = {
            "max_velocity": 0.1,
            "max_acceleration": 0.1,
            "timeout_sec": 60.0,
            **self._motion_limits_kwargs(),
        }

        while self._cycle_count < total:
            self._cycle_count += 1
            cycle_label = f"[cycle {self._cycle_count}]" if total > 1 else ""

            # Step 1: 笛卡尔下降
            self.get_logger().info(f"{cycle_label} [1/2] Cartesian descent")
            if not self.motion.move_to_pose(
                target,
                action_name=f"demo{cycle_label}: descend {self.move_distance:.3f}m",
                cartesian=True,
                **cartesian_limits,
            ):
                self.get_logger().error("Descent failed, demo aborted")
                self._demo_done = True
                return

            # Step 2: 笛卡尔抬升回原位
            self.get_logger().info(f"{cycle_label} [2/2] Cartesian ascent to origin")
            if not self.motion.move_to_pose(
                current,
                action_name=f"demo{cycle_label}: return to origin",
                cartesian=True,
                **cartesian_limits,
            ):
                self.get_logger().error("Ascent failed, demo aborted")
                self._demo_done = True
                return

        self._demo_done = True
        self.get_logger().info(
            f"=== Demo sequence completed ({self._cycle_count} cycles) ==="
        )

    def _motion_limits_kwargs(self) -> dict:
        return {
            "max_step_size": self.max_step_size,
            "allowed_planning_time": self.allowed_planning_time,
            "position_tolerance": self.position_tolerance,
            "orientation_tolerance": self.orientation_tolerance,
            "allowed_start_tolerance": self.allowed_start_tolerance,
        }


def main(args=None):
    rclpy.init(args=args)
    node = DemoNodeWithoutGripper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

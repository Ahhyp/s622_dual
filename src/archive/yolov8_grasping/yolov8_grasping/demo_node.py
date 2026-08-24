#!/usr/bin/env python3
"""
Demo 节点：无相机场景下的简单机械臂运动演示。

复用 manipulation_common：
  - PoseTools         构建目标位姿
  - MoveItMotion      运动规划与执行
  - AbortManager      安全中止
  - param()           参数加载
  - select_best_path  轨迹优选（多候选打分）

演示序列（预设位姿，无需相机/检测）：
  1. 等待 MoveIt 就绪
  2. 移动到 home 位姿（关节空间）
  3. 张开夹爪
  4. 笛卡尔下降到抓取位姿
  5. 闭合夹爪
  6. 笛卡尔抬升
  7. 完成
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from pymoveit2 import MoveIt2

from manipulation_common.utils.params import param
from manipulation_common.utils.pose_tools import PoseTools
from manipulation_common.planning.motion_executor import (
    MoveItMotion,
    PlanScoreConfig,
    PlannerSwitch,
)
from manipulation_common.planning.trajectory_scoring import select_best_path
from manipulation_common.task.abort_manager import AbortManager
from std_msgs.msg import Bool


class DemoNode(Node):
    """无相机 Demo 节点：使用 manipulation_common 执行预设运动序列。"""

    def __init__(self):
        super().__init__(
            "demo_node",
            automatically_declare_parameters_from_overrides=True,
        )

        self.callback_group = ReentrantCallbackGroup()
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        self._startup_ready_logged = False

        # ── 加载参数 ──
        self._load_params()

        # ── 工具 ──
        self.pose_tools = PoseTools(self, base_frame=self.base_frame)

        # ── 构建预设位姿 ──
        self.home_pose = self._build_pose("home_pose")
        self.grasp_pose = self._build_pose("grasp_pose")

        # ── 初始化 MoveIt ──
        self._setup_moveit()

        # ── 中止管理器 ──
        self.abort = AbortManager(
            self, arm=self.moveit2_arm, gripper=self.moveit2_gripper
        )

        # ── 运动执行器（核心）──
        self.motion = MoveItMotion(
            node=self,
            arm_clients={"fairino": self.moveit2_arm},
            default_client="fairino",
            gripper=self.moveit2_gripper,
            pose_tools=self.pose_tools,
            abort=self.abort,
            select_best_path=select_best_path,
            score_cfg=PlanScoreConfig(
                num_candidates=self.num_candidate_plans,
                wrist_weight=self.wrist_weight,
                wrist_joint_indices=self.wrist_joint_indices,
            ),
            action_delay=self.action_delay,
            joint_constraint=self.j2_constraint,
            open_positions=self.gripper_open_positions,
            close_positions=self.gripper_close_positions,
        )

        # ── 中止话题 ──
        self.create_subscription(
            Bool,
            "/manual_abort",
            self.abort.on_manual_abort,
            10,
            callback_group=self.callback_group,
        )

        # ── 启动 demo（延迟等待 MoveIt 就绪）──
        self._demo_started = False
        self._demo_done = False
        self.create_timer(
            3.0, self._try_start_demo, callback_group=self.control_cb_group
        )
        self.get_logger().info("DemoNode initialized (no-camera mode)")

    # ═══════════════════════════════════════════════════════════
    #  参数加载
    # ═══════════════════════════════════════════════════════════

    def _load_params(self):
        """加载所有可配置参数（使用 manipulation_common 的 param()）。"""
        # 运动组
        self.arm_group_name = str(param(self, "arm_group_name", "robot_arm"))
        self.hand_group_name = str(param(self, "hand_group_name", "hand"))
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

        # 运动学限速
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

        # 预设位姿参数
        for pose_name in ("home_pose", "grasp_pose"):
            for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
                self.declare_parameter(
                    f"{pose_name}.{axis}", self._default_pose(pose_name, axis)
                )

        # 夹爪
        self.gripper_open_positions = tuple(
            float(v) for v in param(self, "gripper_open_positions", [0.0305, -0.0305])
        )
        self.gripper_close_positions = tuple(
            float(v) for v in param(self, "gripper_close_positions", [0.0, 0.0])
        )

        # 轨迹打分
        self.num_candidate_plans = int(param(self, "num_candidate_plans", 5))
        self.wrist_weight = float(param(self, "wrist_weight", 50.0))
        self.wrist_joint_indices = tuple(
            int(v) for v in param(self, "wrist_joint_indices", [2, 3, 4])
        )

        # J2 关节约束（可选）
        self.j2_constraint = {
            "joint_positions": [float(param(self, "j2_constraint_position", -1.5708))],
            "joint_names": ["j2"],
            "tolerance": float(param(self, "j2_constraint_tolerance", 1.5708)),
            "weight": 1.0,
        }

        self.get_logger().info("Params loaded")

    @staticmethod
    def _default_pose(pose_name: str, axis: str) -> float:
        """预设位姿默认值。"""
        defaults = {
            "home_pose": {
                "x": 0.149, "y": 0.327, "z": 0.364,
                "roll": -174.091, "pitch": 1.040, "yaw": -50.0,
            },
            "grasp_pose": {
                "x": 0.149, "y": 0.327, "z": 0.164,
                "roll": -174.091, "pitch": 1.040, "yaw": -50.0,
            },
        }
        return defaults.get(pose_name, {}).get(axis, 0.0)

    def _get_pose_cfg(self, name: str) -> dict:
        """从参数服务器读取指定位姿配置。"""
        return {
            axis: float(self.get_parameter(f"{name}.{axis}").value)
            for axis in ("x", "y", "z", "roll", "pitch", "yaw")
        }

    def _build_pose(self, name: str):
        """用 PoseTools 构建指定位姿。"""
        cfg = self._get_pose_cfg(name)
        return self.pose_tools.make_pose(
            cfg["x"], cfg["y"], cfg["z"],
            cfg["roll"], cfg["pitch"], cfg["yaw"],
        )

    # ═══════════════════════════════════════════════════════════
    #  MoveIt 初始化
    # ═══════════════════════════════════════════════════════════

    def _setup_moveit(self):
        """初始化 MoveIt2 机械臂和夹爪客户端。"""
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
        self.gripper_execute_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/hand_controller/follow_joint_trajectory",
            callback_group=self.callback_group,
        )

        self.moveit2_gripper = MoveIt2(
            node=self,
            joint_names=["finger1_joint", "finger2_joint"],
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.hand_group_name,
            callback_group=self.callback_group,
            move_group_namespace=self.move_group_ns_fairino,
        )
        # 夹爪走 OMPL 管线（仅关节空间规划，不需要自定义管线）
        self.moveit2_gripper.pipeline_id = "ompl"
        self.moveit2_gripper.planner_id = ""

        self.get_logger().info("MoveIt2 initialized")

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
    #  Demo 序列
    # ═══════════════════════════════════════════════════════════

    def _try_start_demo(self):
        """定时检查 MoveIt 是否就绪，就绪后启动 demo 序列（仅一次）。"""
        if self._demo_started:
            return
        if not self._startup_ready():
            self.get_logger().info(
                "Waiting for MoveIt services/controllers...",
                throttle_duration_sec=5.0,
            )
            return
        self._demo_started = True
        self.get_logger().info("=== Demo sequence starting ===")
        self._run_demo_sequence()

    def _startup_ready(self) -> bool:
        """检查所有 MoveIt 服务、Action Server、控制器是否就绪。"""
        arm_ready = self._service_ready(
            self.moveit2_arm, self.move_group_ready_timeout_sec
        )
        gripper_ready = self._service_ready(
            self.moveit2_gripper, self.move_group_ready_timeout_sec
        )
        arm_exec_ready = self._action_ready(
            self.arm_execute_action, self.move_group_ready_timeout_sec
        )
        gripper_exec_ready = self._action_ready(
            self.gripper_execute_action, self.move_group_ready_timeout_sec
        )
        controllers_ready = self._controllers_active(
            ("robot_arm_controller", "hand_controller"),
            self.move_group_ready_timeout_sec,
        )
        ready = (
            arm_ready
            and gripper_ready
            and arm_exec_ready
            and gripper_exec_ready
            and controllers_ready
        )
        if ready and not self._startup_ready_logged:
            self.get_logger().info("All MoveIt services ready")
            self._startup_ready_logged = True
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
        import time
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
            deadline = time.time() + float(timeout_sec)
            while rclpy.ok() and not future.done():
                if time.time() >= deadline:
                    return False
                time.sleep(0.05)
            if not future.done() or future.result() is None:
                return False
            states = {c.name: c.state for c in future.result().controller}
            return all(states.get(name) == "active" for name in names)
        except Exception:
            return False

    def _run_demo_sequence(self):
        """执行演示运动序列。"""
        step = 0

        # Step 1: 移动到 home 位姿（关节空间规划）
        step += 1
        self.get_logger().info(f"[{step}/4] Moving to home pose")
        if not self.motion.move_to_pose(
            self.home_pose,
            action_name="demo: move to home",
            cartesian=False,
            max_velocity=self.arm_max_velocity,
            max_acceleration=self.arm_max_acceleration,
            timeout_sec=180.0,
            **self._motion_limits_kwargs(),
        ):
            self.get_logger().error("Failed to reach home pose, demo aborted")
            self._demo_done = True
            return

        # Step 2: 张开夹爪
        step += 1
        self.get_logger().info(f"[{step}/4] Opening gripper")
        if not self.motion.control_gripper(open_gripper=True, timeout_sec=90.0):
            self.get_logger().error("Failed to open gripper, demo aborted")
            self._demo_done = True
            return

        # Step 3: 笛卡尔直线下降到抓取位姿
        step += 1
        self.get_logger().info(f"[{step}/4] Cartesian descent to grasp pose")
        if not self.motion.move_to_pose(
            self.grasp_pose,
            action_name="demo: descend to grasp",
            cartesian=True,
            max_velocity=0.1,
            max_acceleration=0.1,
            timeout_sec=60.0,
            **self._motion_limits_kwargs(),
        ):
            self.get_logger().error("Descent failed, demo aborted")
            self._demo_done = True
            return

        # Step 4: 闭合夹爪 + 抬升回 home
        step += 1
        self.get_logger().info(f"[{step}/4] Closing gripper + returning to home")
        gripper_ok = self.motion.control_gripper(
            open_gripper=False, timeout_sec=90.0
        )
        if not gripper_ok:
            self.get_logger().warn("Gripper close may have failed, continuing...")

        # 抬升回 home
        if not self.motion.move_to_pose(
            self.home_pose,
            action_name="demo: return to home",
            cartesian=True,
            max_velocity=0.1,
            max_acceleration=0.1,
            timeout_sec=60.0,
            **self._motion_limits_kwargs(),
        ):
            self.get_logger().error("Return to home failed")
            self._demo_done = True
            return

        self._demo_done = True
        self.get_logger().info("=== Demo sequence completed successfully ===")

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
    node = DemoNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# colcon build --packages-select yolov8_grasping --merge-install --symlink-install
#!/usr/bin/env python3
"""
motion_demo —— MoveItMotion 全链路独立验证节点（阶段 C1）
=========================================================

目标（C1 能力证明）：
  1. 验证 manipulation_common.MoveItMotion 全链路：plan + execute
  2. 连 fairino move_group（2026-08-23 架构迁移后服务保持 namespaced
     /move_group_fairino/*，客户端用 move_group_namespace 显式连接）
  3. 配合 motion_control_node 验证 stop / reset 事件链路

用法：
  ros2 run manipulation_common motion_demo
  或直接：python3 src/manipulation_common/manipulation_common/nodes/motion_demo_node.py

  启动后：
    - 等待 TF + move_group 就绪
    - 读当前末端位姿（TF: base_link -> grasp_frame）
    - 沿 base Z 下降 motion_distance 米（默认 0.02，仿真快速验证）
    - 调 MoveItMotion.move_to_pose 完成 plan + execute

  可选参数：
    -p move_distance:=0.02    单次下降距离（米）
    -p demo_cycles:=1         执行次数
    -p enable_motion:=true    false = 只规划不执行（Plan Only）
    -p planner_id:="RRTConnectkConfigDefault"
    -p return_to_origin:=false  执行后回到起点

注意：本 demo 为仿真验证设计，真机使用需按 robotarm demo 的安全
检查（max_execute_distance / execute_motion 门控）裁剪。
"""

import time as _time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

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


class MotionDemoNode(rclpy.node.Node):
    """MoveItMotion 全链路独立验证 demo。"""

    def __init__(self):
        super().__init__(
            "motion_demo",
            automatically_declare_parameters_from_overrides=True,
        )
        self.callback_group = ReentrantCallbackGroup()

        # ── 参数 ──
        self.arm_group_name = str(param(self, "arm_group_name", "robot_arm"))
        self.base_frame = str(param(self, "base_frame", "base_link"))
        self.ee_frame = str(param(self, "ee_frame", "grasp_frame"))
        self.move_distance = float(param(self, "move_distance", 0.02))
        self.demo_cycles = int(param(self, "demo_cycles", 1))
        self.enable_motion = bool(param(self, "enable_motion", True))
        self.return_to_origin = bool(param(self, "return_to_origin", False))
        self.max_velocity = float(param(self, "max_velocity", 0.5))
        self.max_acceleration = float(param(self, "max_acceleration", 0.5))
        self.allowed_planning_time = float(param(self, "allowed_planning_time", 5.0))
        self.timeout_sec = float(param(self, "timeout_sec", 30.0))

        # ── TF ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── MoveIt ──
        self._setup_moveit()

        # ── 触发服务：~/start ──
        self.start_srv = self.create_service(
            Trigger, "~/start", self._on_start, callback_group=self.callback_group
        )
        self.get_logger().info(
            "motion_demo ready. Call service: "
            f"ros2 service call /motion_demo/start std_srvs/srv/Trigger '{{}}'"
        )

    def _setup_moveit(self):
        # 连 fairino move_group（2026-08-23 架构迁移后服务保持 namespaced，客户端显式连接）
        self.moveit2_arm = MoveIt2(
            node=self,
            joint_names=["j1", "j2", "j3", "j4", "j5", "j6"],
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.arm_group_name,
            callback_group=self.callback_group,
            move_group_namespace="/move_group_fairino",
        )
        # OMPL 管线（本架构只启用 ompl）
        self.moveit2_arm.pipeline_id = PlannerSwitch.normalize_pipeline("ompl")
        self.moveit2_arm.planner_id = str(param(self, "planner_id", "RRTConnectkConfigDefault"))
        self.moveit2_arm.max_velocity = self.max_velocity
        self.moveit2_arm.max_acceleration = self.max_acceleration
        self.moveit2_arm.allowed_planning_time = self.allowed_planning_time

        self.pose_tools = PoseTools(self, base_frame=self.base_frame)

        self.motion = MoveItMotion(
            self,
            arm_clients={"fairino": self.moveit2_arm},
            default_client="fairino",
            gripper=None,
            pose_tools=self.pose_tools,
            abort=None,
            select_best_path=select_best_path,
            score_cfg=PlanScoreConfig(num_candidates=5),
        )

    # ═══════════════════════════════════════════════════════════

    def _get_current_ee_pose(self, timeout_sec: float = 2.0) -> Pose | None:
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
            pose.orientation.x = tf.transform.rotation.x
            pose.orientation.y = tf.transform.rotation.y
            pose.orientation.z = tf.transform.rotation.z
            pose.orientation.w = tf.transform.rotation.w
            return pose
        except Exception as exc:
            self.get_logger().error(f"TF lookup failed ({self.base_frame} -> {self.ee_frame}): {exc}")
            return None

    def _on_start(self, request, response):
        self.get_logger().info("=== motion_demo triggered ===")
        ok = self._run_demo()
        response.success = ok
        response.message = "motion_demo completed" if ok else "motion_demo FAILED"
        return response

    def _run_demo(self) -> bool:
        # 等待 move_group 规划服务就绪
        if not self.motion.wait_client_ready("fairino", timeout_sec=10.0):
            self.get_logger().error("move_group planning service not ready")
            return False

        cycles = max(1, self.demo_cycles)
        for i in range(cycles):
            self.get_logger().info(f"[cycle {i + 1}/{cycles}] reading current EE pose...")
            current = self._get_current_ee_pose()
            if current is None:
                return False

            target = Pose()
            target.position.x = current.position.x
            target.position.y = current.position.y
            target.position.z = current.position.z - self.move_distance
            target.orientation = current.orientation

            self.get_logger().info(
                f"current: ({current.position.x:.3f}, {current.position.y:.3f}, {current.position.z:.3f})"
            )
            self.get_logger().info(
                f"target:  ({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})  "
                f"ΔZ = {-self.move_distance * 1000:.1f} mm"
            )

            success = self.motion.move_to_pose(
                target,
                planning_client="fairino",
                cartesian=False,
                action_name=f"motion_demo cycle {i + 1}: descend",
                max_velocity=self.max_velocity,
                max_acceleration=self.max_acceleration,
                allowed_planning_time=self.allowed_planning_time,
                timeout_sec=self.timeout_sec,
                plan_only=not self.enable_motion,
            )
            if not success:
                self.get_logger().error(f"cycle {i + 1} FAILED")
                return False

            # 可选返回起点
            if self.return_to_origin and self.enable_motion:
                self.get_logger().info("returning to origin...")
                success = self.motion.move_to_pose(
                    current,
                    planning_client="fairino",
                    cartesian=False,
                    action_name=f"motion_demo cycle {i + 1}: return",
                    max_velocity=self.max_velocity,
                    max_acceleration=self.max_acceleration,
                    allowed_planning_time=self.allowed_planning_time,
                    timeout_sec=self.timeout_sec,
                    plan_only=False,
                )
                if not success:
                    self.get_logger().error("return to origin FAILED")
                    return False

        self.get_logger().info("=== motion_demo completed successfully ===")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MotionDemoNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""MoveToPose Action server。

支持两种 goal：
  - target_pose: 位姿目标
  - named_pose:  通过 yaml 配置的命名关节配置（如 "home"）
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from s622_bt_manager.action import MoveToPose

from s622_arm_actions.servo_lifecycle import ServoLifecycleManager

# C2（2026-08-24）：运动执行改用 manipulation_common.MoveItMotion（对齐 robotarm），
# 不再用自写 MoveItPlanner。接口（MoveToPose.action）不变，BT 层无感知。
from manipulation_common.utils.pose_tools import PoseTools
from manipulation_common.planning.motion_executor import MoveItMotion, PlanScoreConfig
from manipulation_common.planning.trajectory_scoring import select_best_path
from manipulation_common.task.abort_manager import AbortManager
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose


class MoveToPoseServer(Node):
    def __init__(self):
        super().__init__('move_to_pose_server')

        # ---- 参数 ----
        self.declare_parameter('joint_names',
            ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector', 'grasp_frame')
        self.declare_parameter('group_name', 'robot_arm')
        self.declare_parameter('default_velocity_scale', 1.0)
        self.declare_parameter('default_acceleration_scale', 1.0)
        self.declare_parameter('servo_ns', '')    
        self.declare_parameter('arm_controller_action', '')   
        # S2（2026-08-25）：move_group namespace / pipeline 参数化（双臂引入 MoveItMotion）
        self.declare_parameter('move_group_namespace', '/move_group_fairino')
        self.declare_parameter('pipeline_id', 'fairino')
        
        # named poses (joint positions)
        # self.declare_parameter('named_poses.home',
        #     [0.5, -1.05, 1.05, -1.05, -0.8, 0.0])
        # self.declare_parameter('named_poses.safe',
        #     [0.0, -1.2, 1.5, 0.0, 1.2, 0.0])

        self.declare_parameter('named_pose_names', ['home', 'safe'])
        self.declare_parameter('home_joint_positions',
            [0.5, -1.05, 1.05, -1.05, -0.8, 0.0])
        self.declare_parameter('safe_joint_positions',
            [0.0, -1.2, 1.5, 0.0, 1.2, 0.0])
        

        joint_names = list(self.get_parameter('joint_names').value)
        base_link = self.get_parameter('base_link').value
        end_effector = self.get_parameter('end_effector').value
        group_name = self.get_parameter('group_name').value
        self._default_v = self.get_parameter('default_velocity_scale').value
        self._default_a = self.get_parameter('default_acceleration_scale').value
        servo_ns = self.get_parameter('servo_ns').value
        arm_controller_action = self.get_parameter('arm_controller_action').value     
        move_group_namespace = self.get_parameter('move_group_namespace').value
        pipeline_id = self.get_parameter('pipeline_id').value
        
        # self._named_poses = {
        #     'home': list(self.get_parameter('named_poses.home').value),
        #     'safe': list(self.get_parameter('named_poses.safe').value),
        # }
        pose_names = list(self.get_parameter('named_pose_names').value)
        self._named_poses = {}
        for name in pose_names:
            param_name = f'{name}_joint_positions'
            try:
                self.declare_parameter(param_name, [0.0] * 6)
            except Exception:
                pass
            raw = self.get_parameter(param_name).value
            self._named_poses[name] = [float(x) for x in raw]   # 强制 float
        self.get_logger().info(f'loaded named poses: {list(self._named_poses.keys())}')


        cb = ReentrantCallbackGroup()
        self.servo_lc = ServoLifecycleManager(self, callback_group=cb, servo_ns=servo_ns)

        # C2：MoveItMotion（对齐 robotarm fairino_pose_control_server）
        # S2：move_group_namespace / pipeline_id 参数化（双臂按臂传入）
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name,
            callback_group=cb,
            move_group_namespace=move_group_namespace,
        )
        self.moveit2.pipeline_id = pipeline_id
        # 2026-08-25 回归修复：必须传入 AbortManager，否则 MoveItMotion._wait 只 sleep 0.5s
        # 就返回 SUCCESS（机械臂实际还在执行）→ BT 提前进入下一步 → 未到位就 descend/关爪。
        # 对齐 robotarm fairino_pose_control_server（abort=self.abort）。
        self.abort = AbortManager(self, arm=self.moveit2, gripper=None)
        self.motion = MoveItMotion(
            self,
            arm_clients={"fairino": self.moveit2},
            gripper=None,                      # 夹爪由独立 gripper_service 控制（C2 范围内一并换）
            pose_tools=PoseTools(self, base_frame=base_link),
            abort=self.abort,
            select_best_path=select_best_path,
            score_cfg=PlanScoreConfig(num_candidates=8),
            action_delay=0.0,
        )
        self._vel = self._default_v
        self._acc = self._default_a

        self._action_server = ActionServer(
            self,
            MoveToPose,
            'move_to_pose',
            execute_callback=self._execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cb,
        )
        self.get_logger().info(
            f'move_to_pose ready (MoveItMotion): group={group_name}, ee={end_effector}, '
            f'base={base_link}, joints={joint_names}, mg_ns={move_group_namespace}, '
            f'pipeline={pipeline_id}')

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()

        # 1) ensure servo stopped
        if goal.ensure_servo_stopped:
            if not self.servo_lc.force_stop():
                self.get_logger().warning(
                    'force_stop failed; continuing (servo may not be running)')

        # 2) set speed（存参数，move_to_pose 时透传）
        v = goal.velocity_scale if goal.velocity_scale > 0 else self._default_v
        a = goal.acceleration_scale if goal.acceleration_scale > 0 else self._default_a
        self._vel = v
        self._acc = a

        # 3) plan + execute（MoveItMotion）
        try:
            if goal.named_pose:
                if goal.named_pose not in self._named_poses:
                    msg = f'unknown named_pose: {goal.named_pose}'
                    self.get_logger().error(msg)
                    goal_handle.abort()
                    result.success = False
                    result.error_msg = msg
                    return result
                self.get_logger().info(f'going to named pose: {goal.named_pose}')
                # move_to_joints 无速度参数：前置设置 moveit2 属性
                self.moveit2.max_velocity = v
                self.moveit2.max_acceleration = a
                ok = self.motion.move_to_joints(
                    self._named_poses[goal.named_pose],
                    action_name=f"named:{goal.named_pose}",
                    timeout_sec=60.0,
                )
            else:
                p = goal.target_pose.pose
                pose = Pose()
                pose.position.x = p.position.x
                pose.position.y = p.position.y
                pose.position.z = p.position.z
                pose.orientation = p.orientation
                ok = self.motion.move_to_pose(
                    pose,
                    cartesian=False,
                    action_name="move_to_pose",
                    max_velocity=v,
                    max_acceleration=a,
                    timeout_sec=60.0,
                )
        except Exception as e:
            self.get_logger().error(f'plan/execute exception: {e}')
            goal_handle.abort()
            result.success = False
            result.error_msg = str(e)
            return result

        if ok:
            goal_handle.succeed()
            result.success = True
            result.error_msg = ''
        else:
            goal_handle.abort()
            result.success = False
            result.error_msg = 'planning or execution failed'
        return result


def main():
    rclpy.init()
    node = MoveToPoseServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
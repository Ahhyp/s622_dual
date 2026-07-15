# src/s622_arm_actions/launch/arm_actions_dual.launch.py
import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def action_remaps(action_name):
    """Expand an action name to its 5 underlying topics."""
    return [
        (f'{action_name}/_action/feedback',    f'/{action_name}/_action/feedback'),
        (f'{action_name}/_action/status',      f'/{action_name}/_action/status'),
        (f'{action_name}/_action/cancel_goal', f'/{action_name}/_action/cancel_goal'),
        (f'{action_name}/_action/get_result',  f'/{action_name}/_action/get_result'),
        (f'{action_name}/_action/send_goal',   f'/{action_name}/_action/send_goal'),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('s622_arm_actions')
    left_cfg = os.path.join(pkg, 'config', 'left_arm_config.yaml')
    right_cfg = os.path.join(pkg, 'config', 'right_arm_config.yaml')

    moveit_service_remaps = [
        ('plan_kinematic_path', '/plan_kinematic_path'),
        ('compute_ik', '/compute_ik'),
        ('compute_fk', '/compute_fk'),
        ('compute_cartesian_path', '/compute_cartesian_path'),
        ('get_planning_scene', '/get_planning_scene'),
        ('apply_planning_scene', '/apply_planning_scene'),
        ('query_planner_interface', '/query_planner_interface'),
        ('get_planner_params', '/get_planner_params'),
        ('set_planner_params', '/set_planner_params'),
        ('planning_scene', '/planning_scene'),
        ('monitored_planning_scene', '/monitored_planning_scene'),
        ('planning_scene_world', '/planning_scene_world'),
        ('attached_collision_object', '/attached_collision_object'),
        ('collision_object', '/collision_object'),
        ('trajectory_execution_event', '/trajectory_execution_event'),
        ('display_planned_path', '/display_planned_path'),
        ('display_contacts', '/display_contacts'),
        ('joint_states', '/joint_states'),
        ('robot_description', '/robot_description'),
        ('robot_description_semantic', '/robot_description_semantic'),
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
    ]

    moveit_action_remaps = (
        action_remaps('execute_trajectory') +
        action_remaps('move_action')
    )

    moveit_remaps = moveit_service_remaps + moveit_action_remaps

    def make_arm_group(ns, cfg):
        return GroupAction([
            PushRosNamespace(ns),
            Node(
                package='s622_arm_actions',
                executable='move_to_pose_server',
                name='move_to_pose_server',
                output='screen',
                parameters=[cfg, {'use_sim_time': True}],
                remappings=moveit_remaps,
            ),
            Node(
                package='s622_arm_actions',
                executable='visual_align_server',
                name='visual_align_server',
                output='screen',
                parameters=[cfg, {'use_sim_time': True}],
                remappings=moveit_remaps,
            ),
            Node(
                package='s622_arm_actions',
                executable='gripper_service',
                name='gripper_service',
                output='screen',
                parameters=[cfg, {'use_sim_time': True}],
                remappings=moveit_remaps,
            ),
        ])

    
    dual_cfg = os.path.join(pkg, 'config', 'dual_arm_config.yaml')

    dual_group = GroupAction([
        PushRosNamespace('dual'),
        Node(
            package='s622_arm_actions',
            executable='dual_move_server',
            name='dual_move_server',
            output='screen',
            parameters=[dual_cfg, {'use_sim_time': True}],
            remappings=moveit_remaps,   # 复用: 让 dual node 找到全局 MoveIt 服务
        ),
    ])
    
    return LaunchDescription([
        make_arm_group('left', left_cfg),
        make_arm_group('right', right_cfg),
        dual_group,
    ])

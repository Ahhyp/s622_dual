# src/s622_arm_actions/launch/arm_actions_dual.launch.py
# S3（2026-08-25）：双臂 MoveItMotion 化 —— 客户端用 move_group_namespace 直连
# /move_group_fairino（config 里配置），不再 remap 到根级 move_group 服务。
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('s622_arm_actions')
    left_cfg = os.path.join(pkg, 'config', 'left_arm_config.yaml')
    right_cfg = os.path.join(pkg, 'config', 'right_arm_config.yaml')

    # [M4 §7.21] 仿真侧夹爪"机械限位"（等效真机拧螺丝定的最小闭合间隙）。
    # 默认 0.0 = 原行为（close → close_positions=[0,0]），真机路径不受影响。
    # 仿真跑长条交接时显式传 gripper_close_stop_gap:=0.029（30mm 杆留 1mm 过盈）。
    close_stop_gap = DeclareLaunchArgument(
        'gripper_close_stop_gap', default_value='0.0',
        description='[sim] 0=原行为(压到 close_positions); >0=close 目标改为 ±值/2 的限位间隙(m)')

    # 保留的最小 remap：joint_states（namespaced 节点默认订阅 /<ns>/joint_states）
    def make_arm_group(ns, cfg):
        return GroupAction([
            PushRosNamespace(ns),
            Node(
                package='s622_arm_actions',
                executable='move_to_pose_server',
                name='move_to_pose_server',
                output='screen',
                parameters=[cfg, {'use_sim_time': True}],
                remappings=[('joint_states', '/joint_states')],
            ),
            Node(
                package='s622_arm_actions',
                executable='visual_align_server',
                name='visual_align_server',
                output='screen',
                parameters=[cfg, {'use_sim_time': True}],
                remappings=[('joint_states', '/joint_states')],
            ),
            Node(
                package='s622_arm_actions',
                executable='gripper_service',
                name='gripper_service',
                output='screen',
                parameters=[cfg,
                            {'use_sim_time': True,
                             'close_stop_gap': LaunchConfiguration('gripper_close_stop_gap')}],
                remappings=[('joint_states', '/joint_states')],
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
            remappings=[('joint_states', '/joint_states')],
        ),
    ])

    return LaunchDescription([
        close_stop_gap,
        make_arm_group('left', left_cfg),
        make_arm_group('right', right_cfg),
        dual_group,
    ])

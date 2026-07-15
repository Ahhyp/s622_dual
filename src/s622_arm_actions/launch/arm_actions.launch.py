import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('s622_arm_actions'),
        'config', 'arm_config.yaml')

    return LaunchDescription([
        Node(
            package='s622_arm_actions',
            executable='move_to_pose_server',
            name='move_to_pose_server',
            output='screen',
            parameters=[config, {'use_sim_time': True}],
        ),
        Node(
            package='s622_arm_actions',
            executable='gripper_service',
            name='gripper_service',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'gripper_topic': '/hand_controller/joint_trajectory',
                'finger_joint_names': ['finger1_joint', 'finger2_joint'],
                'open_positions': [0.025, -0.025],
                'close_positions': [0.0, 0.0],
                'feedback_joint': 'finger1_joint',
            }],
        ),
        Node(
            package='s622_arm_actions',
            executable='visual_align_server',
            name='visual_align_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_link',
                'ee_frame': 'grasp_frame',
                'camera_frame': 'camera_color_optical_frame',
                'twist_topic': '/servo_node/delta_twist_cmds',
                # 'j_img_to_base': [-0.001939, 0.000069, 0.0, 0.001957],
                'xy_kp': 1.0,
                'xy_max_speed': 0.05,
                'xy_min_pixel_err_to_move': 2.0,
                'yaw_kp': 1.0,
                'yaw_max_speed': 0.5,
                'descend_max_speed': 0.05,
                'lift_max_speed': 0.05,
            }],
        ),
        Node(
            package='s622_arm_actions',
            executable='planning_scene_service',
            name='planning_scene_service',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_link': 'base_link',
                'default_object_size': [0.04, 0.04, 0.04],
                'default_touch_links': ['finger1', 'finger2', 'grasp_frame'],
                'publish_table': True,
            }],
        ),
    ])
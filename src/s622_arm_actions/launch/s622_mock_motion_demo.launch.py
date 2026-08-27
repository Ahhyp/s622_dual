# s622_mock_motion_demo.launch.py
# Mock 硬件最小运动验证（无真机，2026-08-27）
#
# 目的: 不连真机，用 mock_components/GenericSystem 跑通 demo 全链路：
#       RSP + CM(mock) + controllers + move_group + demo 节点
#       （规划 + 执行"假轨迹"，controller 走完整下发流程，验证链路无 bug）
#
# 与 s622_real_motion_demo.launch.py 的差异: 仅 robot_description 用 mock URDF，
#       controllers yaml 复用 real_controllers.yaml（position 接口与 mock 一致）
#
# 用法:
#   ros2 launch s622_arm_actions s622_mock_motion_demo.launch.py
#   ros2 service call /demo_node_without_gripper/start std_srvs/srv/Trigger "{}"
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from manipulation_common.launch_utils.yaml_loader import load_yaml


def generate_launch_description():
    robot_moveit_pkg = get_package_share_directory("s622_moveit_config")
    pkg = get_package_share_directory("s622_arm_actions")

    declare_start_demo = DeclareLaunchArgument(
        "start_demo", default_value="false",
        description="true=自动跑；false=等 ~/start 服务（推荐）")
    declare_execute = DeclareLaunchArgument(
        "execute_motion", default_value="true",
        description="mock 无真机风险，默认 true 走完整执行链路")
    declare_move_distance = DeclareLaunchArgument(
        "move_distance", default_value="0.005",
        description="Z 轴微动距离（米），默认 5mm")

    # ============ MoveIt 配置（mock URDF + 保守 joint_limits） ============
    moveit_config = MoveItConfigsBuilder("s622_moveit_descriptions", package_name="s622_moveit_config") \
        .robot_description(
            os.path.join(robot_moveit_pkg, "config", "s622_mock_arm.urdf.xacro")) \
        .robot_description_semantic("config/s622_moveit_descriptions.srdf") \
        .robot_description_kinematics(robot_moveit_pkg + "/config/kinematics.yaml") \
        .joint_limits("config/real_joint_limits.yaml") \
        .planning_pipelines(pipelines=["fairino", "ompl"], default_planning_pipeline="fairino") \
        .to_moveit_configs()

    # ============ 1. robot_state_publisher ============
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # ============ 2. controller_manager（mock 硬件）+ controllers ============
    real_controllers_yaml = os.path.join(robot_moveit_pkg, "config", "real_controllers.yaml")
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, real_controllers_yaml],
        output="screen",
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["-p", real_controllers_yaml,
                   "robot_arm_controller",
                   "joint_state_broadcaster",
                   "hand_controller"],
        output="screen",
    )

    # ============ 3. move_group（fairino 管线） ============
    fairino_planning = load_yaml("s622_moveit_config", "config/fairino_planning.yaml")
    planning_core = load_yaml("fairino_planning_core", "config/common_planning_params.yaml")
    aapf_star_core = load_yaml("fairino_planning_core", "config/aapf_birrt__params.yaml")
    tube_star_core = load_yaml("fairino_planning_core", "config/tube_birrt__params.yaml")
    birrt_star_core = load_yaml("fairino_planning_core", "config/birrt__params.yaml")
    rrt_star_core = load_yaml("fairino_planning_core", "config/rrt__params.yaml")
    ik_core = load_yaml("fairino_planning_core", "config/ik_params.yaml")
    cartesian_planner = load_yaml("fairino_planning_core", "config/cartesian_path_planner_params.yaml")

    mg_remappings = [
        ("joint_states", "/joint_states"),
        ("trajectory_execution_event", "/trajectory_execution_event"),
        ("planning_scene", "/planning_scene"),
        ("collision_object", "/collision_object"),
        ("attached_collision_object", "/attached_collision_object"),
        ("robot_arm_controller/follow_joint_trajectory", "/robot_arm_controller/follow_joint_trajectory"),
        ("hand_controller/follow_joint_trajectory", "/hand_controller/follow_joint_trajectory"),
        ("robot_arm_controller/follow_joint_trajectory/_action/feedback", "/robot_arm_controller/follow_joint_trajectory/_action/feedback"),
        ("robot_arm_controller/follow_joint_trajectory/_action/status", "/robot_arm_controller/follow_joint_trajectory/_action/status"),
        ("robot_arm_controller/follow_joint_trajectory/_action/cancel_goal", "/robot_arm_controller/follow_joint_trajectory/_action/cancel_goal"),
        ("robot_arm_controller/follow_joint_trajectory/_action/get_result", "/robot_arm_controller/follow_joint_trajectory/_action/get_result"),
        ("robot_arm_controller/follow_joint_trajectory/_action/send_goal", "/robot_arm_controller/follow_joint_trajectory/_action/send_goal"),
        ("hand_controller/follow_joint_trajectory/_action/feedback", "/hand_controller/follow_joint_trajectory/_action/feedback"),
        ("hand_controller/follow_joint_trajectory/_action/status", "/hand_controller/follow_joint_trajectory/_action/status"),
        ("hand_controller/follow_joint_trajectory/_action/cancel_goal", "/hand_controller/follow_joint_trajectory/_action/cancel_goal"),
        ("hand_controller/follow_joint_trajectory/_action/get_result", "/hand_controller/follow_joint_trajectory/_action/get_result"),
        ("hand_controller/follow_joint_trajectory/_action/send_goal", "/hand_controller/follow_joint_trajectory/_action/send_goal"),
        ("robot_description", "/robot_description"),
        ("robot_description_semantic", "/robot_description_semantic"),
    ]

    move_group_fairino = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="move_group_fairino",
        name="move_group",
        output="screen",
        remappings=mg_remappings,
        parameters=[
            moveit_config.to_dict(),
            fairino_planning,
            planning_core,
            aapf_star_core,
            tube_star_core,
            birrt_star_core,
            rrt_star_core,
            ik_core,
            {"fairino": {"ik": {"task_profile": "grasp"}}},
            {"planner": {"random_seed": 0}},
        ],
    )

    # ============ 4. retime_server ============
    retime_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("trajectory_retime_server"),
                "launch",
                "retime_server.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    # ============ 4b. fairino_cartesian_path_server（demo cartesian 段必需） ============
    # MoveItMotion.move_to_pose(cartesian=True) 调用 /fairino_cartesian_path；
    # 参数对齐 robotarm moveit_stack.py（continuous IK profile + cartesian planner 参数）
    cartesian_server = Node(
        package="fairino_planning_ros",
        executable="fairino_cartesian_path_server",
        name="fairino_cartesian_path_server",
        output="screen",
        parameters=[
            fairino_planning,
            planning_core,
            aapf_star_core,
            tube_star_core,
            birrt_star_core,
            rrt_star_core,
            ik_core,
            cartesian_planner,
            {"fairino": {"ik": {"task_profile": "continuous"}}},
            {"planner": {"random_seed": 0}},
        ],
    )

    # ============ 5. demo 节点（延迟 5s 等栈就绪） ============
    demo_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="s622_arm_actions",
                executable="demo_node_without_gripper",
                name="demo_node_without_gripper",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "demo_node_without_gripper.yaml"),
                    {
                        "start_demo": LaunchConfiguration("start_demo"),
                        "execute_motion": LaunchConfiguration("execute_motion"),
                        "move_distance": LaunchConfiguration("move_distance"),
                    },
                ],
            )
        ],
    )

    return LaunchDescription([
        declare_start_demo, declare_execute, declare_move_distance,
        robot_state_pub,
        controller_manager_node,
        controller_spawner,
        move_group_fairino,
        retime_server_launch,
        cartesian_server,
        demo_node,
    ])

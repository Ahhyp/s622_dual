# s622_real_arm.launch.py
# 真机单臂启动（2026-08-26，Phase B-1）
# 组件: RSP + CM(fairino_hardware 插件, ip/prefix 参数化) + 3 controllers(position 版)
#       + move_group_fairino + RViz + retime_server
# 无 Gazebo：真机插件直连控制器 SDK（XML-RPC + ServoJ）
# 用法:
#   ros2 launch s622_moveit_config s622_real_arm.launch.py ip:=192.168.58.3
#   （ip 默认 192.168.58.3；双臂阶段传 ip:=58.2/58.3 + prefix:=left_/right_）
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from manipulation_common.launch_utils.yaml_loader import load_yaml


def generate_launch_description():
    robot_moveit_pkg = get_package_share_directory("s622_moveit_config")

    declare_ip = DeclareLaunchArgument("ip", default_value="192.168.58.3",
                                       description="机械臂控制器 IP（真机单臂 58.3；双臂 left=58.2/right=58.3）")
    declare_prefix = DeclareLaunchArgument("prefix", default_value="",
                                           description="joint 名前缀（双臂 left_/right_；单臂留空）")
    ip = LaunchConfiguration("ip")
    prefix = LaunchConfiguration("prefix")

    # ============ MoveIt 配置（真机 URDF + 保守 joint_limits） ============
    # 真机 URDF: fairino_hardware 插件 + ip/prefix 参数（s622_real_arm.urdf.xacro）
    # joint_limits: real_joint_limits.yaml（scaling 0.3，Phase B-2 保守速度）
    moveit_config = MoveItConfigsBuilder("s622_moveit_descriptions", package_name="s622_moveit_config") \
        .robot_description(
            os.path.join(robot_moveit_pkg, "config", "s622_real_arm.urdf.xacro"),
            mappings={"ip": ip, "prefix": prefix}) \
        .robot_description_semantic("config/s622_moveit_descriptions.srdf") \
        .robot_description_kinematics(robot_moveit_pkg + "/config/kinematics.yaml") \
        .joint_limits("config/real_joint_limits.yaml") \
        .planning_pipelines(pipelines=["fairino", "ompl"], default_planning_pipeline="fairino") \
        .to_moveit_configs()

    # ============ 1. robot_state_publisher（真机不用 use_sim_time） ============
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # ============ 2. controller_manager + controllers（position 版 yaml） ============
    # 真机没有 gz_ros2_control 插件自动创建 CM，必须显式启动 ros2_control_node
    # （仿真里 CM 由 gz 插件内部创建，所以仿真 launch 不需要这一行）
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

    # ============ 3. move_group（fairino 规划管线，客户端用 move_group_namespace 连接） ============
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

    # ============ 4. RViz（真机调试可视化） ============
    # 用 gz_launch.rviz（Move Group Namespace=/move_group_fairino + Planning Scene Topic
    # /move_group_fairino/monitored_planning_scene，与仿真/robotarm 同一方案）；
    # moveit.rviz 的 namespace 为空，会订阅根级 /monitored_planning_scene 导致模型不跟随
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("gz_launch"), "rviz", "gz_launch.rviz")],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    moveit_config.planning_pipelines,
                    fairino_planning],
    )

    # ============ 5. trajectory_retime_server（TOTG，真机不用 sim time） ============
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

    # ============ 5b. fairino_cartesian_path_server（cartesian 路径规划服务） ============
    # MoveItMotion.move_to_pose(cartesian=True) / BT cartesian 段依赖 /fairino_cartesian_path
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

    return LaunchDescription([
        declare_ip, declare_prefix,
        robot_state_pub,
        controller_manager_node,
        controller_spawner,
        move_group_fairino,
        rviz,
        retime_server_launch,
        cartesian_server,
    ])

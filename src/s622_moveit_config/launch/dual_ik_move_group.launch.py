# s622_moveit_config/launch/dual_ik_move_group.launch.py
# 双 move_group：/move_group_fairino（FairinoIKPlugin 解析 IK）+ /move_group_kdl（KDL fallback）
#
# 设计来源：robotarm gazebo_launch/launch_utils/moveit_stack.py 的 move_group_nodes()
# 差异点：
#   - kinematics 用 MoveIt 标准格式注入（robot_description_kinematics.<group>），
#     确保 FairinoIKPlugin 真正被 move_group 加载
#   - 管线保留用户现有配置（ompl + fairino[OMPL 自定义]），不引入 FairinoPlannerManager
# 用途：独立启动验证双 move_group 与 IK 插件（无需 Gazebo）
#   ros2 launch s622_moveit_config dual_ik_move_group.launch.py
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from manipulation_common.launch_utils.yaml_loader import load_yaml
from moveit_configs_utils import MoveItConfigsBuilder


def _action_remaps(action_name):
    """把 action 名展开成其 5 个底层 topic 的 remap（与 arm_actions_dual.launch.py 一致）。"""
    return [
        (f"{action_name}/_action/feedback",    f"/{action_name}/_action/feedback"),
        (f"{action_name}/_action/status",      f"/{action_name}/_action/status"),
        (f"{action_name}/_action/cancel_goal", f"/{action_name}/_action/cancel_goal"),
        (f"{action_name}/_action/get_result",  f"/{action_name}/_action/get_result"),
        (f"{action_name}/_action/send_goal",   f"/{action_name}/_action/send_goal"),
    ]


def generate_launch_description():
    robot_moveit_pkg = get_package_share_directory("s622_moveit_config")
    planning_ros_pkg = get_package_share_directory("fairino_planning_ros")
    gz_launch_pkg = get_package_share_directory("gz_launch")

    # ── MoveIt 配置（单臂 gazebo xacro：显式实例化 s622_arm，M2.8 修复后）──
    moveit_config = (
        MoveItConfigsBuilder("s622_moveit_descriptions", package_name="s622_moveit_config")
        .robot_description(
            os.path.join(gz_launch_pkg, "config", "robot_gazebo.urdf.xacro"),
            mappings={"include_camera": "false"},
        )
        .robot_description_semantic("config/s622_moveit_descriptions.srdf")
        .robot_description_kinematics("config/kinematics.yaml")  # 默认 KDL，下面按实例覆盖
        .trajectory_execution("config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # ── 管线参数（用户现有：ompl + fairino 段，fairino 段是 OMPL 自定义 planner）──
    fairino_planning_yaml = os.path.join(
        planning_ros_pkg, "config", "fairino_planning.yaml"
    )
    with open(fairino_planning_yaml, "r") as f:
        pipeline_params = yaml.safe_load(f)

    # ── 两套 kinematics（MoveIt 标准格式，覆盖 to_dict() 默认）──
    kinematics_fairino = load_yaml("s622_moveit_config", "config/kinematics_fairino.yaml")
    kinematics_kdl = load_yaml("s622_moveit_config", "config/kinematics_kdl.yaml")

    # ── 公共 remap（复制 robotarm moveit_stack.py:126-149）──
    # namespaced move_group 默认订阅 /move_group_*/joint_states 等，统一 remap 回全局
    remappings = [
        ("joint_states", "/joint_states"),
        # 客户端发布的标准 stop 事件在根话题，不 remap 则 namespaced 实例收不到
        ("trajectory_execution_event", "/trajectory_execution_event"),
        # 障碍物发布者在根话题，保持两个实例都订阅全局 PlanningScene
        ("planning_scene", "/planning_scene"),
        ("collision_object", "/collision_object"),
        ("attached_collision_object", "/attached_collision_object"),
        # ── controller action client remap（关键：源名必须相对，无前导 /）──
        # MoveIt 内部用相对名 robot_arm_controller/follow_joint_trajectory 创建 action
        # client（来自 moveit_controllers.yaml），在 /move_group_fairino namespace 下会
        # 解析成 namespaced 地址。对齐 robotarm moveit_stack.py 的写法：源名相对
        # （lstrip('/')）+ 目标绝对；另加 5-topic 兜底（实测顶层 action 名 remap 对
        # client 不总是生效，5-topic 展开对 move_action 已验证有效）。
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
        # ── monitored_planning_scene 不 remap 到根级（2026-08-23 架构迁移）──
        # 每个 move_group 发布到自己的 /move_group_*/monitored_planning_scene，
        # 由 RViz 端 remap 订阅 fairino 实例（对齐 robotarm moveit_stack.py）。
        # robot_description/semantic 话题保留根级暴露（其他工具可能订阅话题）。
        ("robot_description", "/robot_description"),
        ("robot_description_semantic", "/robot_description_semantic"),
    ]

    # ── move_group #1：Fairino 解析 IK（服务保持 namespaced，客户端用 move_group_namespace 连）──
    # 2026-08-23 架构迁移：不再 remap 到根级。对齐 robotarm 做法——move_group 服务留在
    # /move_group_fairino/*，客户端（pymoveit2 move_group_namespace + RViz remap）显式连接。
    move_group_fairino = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="move_group_fairino",
        name="move_group",
        output="screen",
        remappings=remappings,
        parameters=[
            moveit_config.to_dict(),
            pipeline_params,
            {"robot_description_kinematics": kinematics_fairino},
            {"use_sim_time": True},
        ],
    )

    # ── move_group #2：KDL 数值 IK（fallback / 对比）──
    move_group_kdl = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="move_group_kdl",
        name="move_group",
        output="screen",
        remappings=remappings,
        parameters=[
            moveit_config.to_dict(),
            pipeline_params,
            {"robot_description_kinematics": kinematics_kdl},
            {"use_sim_time": True},
        ],
    )

    # ── TF + joint states（独立运行时自包含：move_group 需要 base_link TF 和 /joint_states）──
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([move_group_fairino, move_group_kdl, robot_state_pub, joint_state_pub])

# s622_dual_arm.launch.py
# M2.4: 双臂完整环境启动
# 组件: Gazebo(world+双臂+桌子+cube) + RSP + CM(gz_ros2_control) + 5 controllers
#       + move_group + RViz + planning_scene_service(双臂 touch_links) + camera_bridge
# 不含: obb_node(M2.7 改造), servo(M2.5)
import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from manipulation_common.launch_utils.yaml_loader import load_yaml


def generate_launch_description():
    this_pkg = get_package_share_directory("gz_launch")
    robot_desc_pkg = get_package_share_directory("s622_moveit_descriptions")
    robot_moveit_pkg = get_package_share_directory("s622_moveit_config")

    # ============ 环境变量: gz sim model search path ============
    set_model_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.path.join(this_pkg, "models")
        + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )

    # ============ 1. Gazebo + world ============
    world_file = os.path.join(this_pkg, "worlds", "dual_arm_world.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", world_file + " -r")],
    )

    # ============ 2. Bridges ============
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/world/dual_arm_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        remappings=[("/world/dual_arm_world/clock", "/clock")],
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/camera/image", "/camera/color/image_raw"),
            ("/camera/depth_image", "/camera/depth/image_raw"),
            ("/camera/camera_info", "/camera/color/camera_info"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ============ 3. MoveIt config (双臂 URDF+SRDF) ============
    # S3（2026-08-25）：规划管线统一 —— fairino（FairinoPlannerManager）+ ompl 备用，
    # 默认 fairino。MoveItConfigsBuilder 自动加载 config/fairino_planning.yaml
    # 与 config/ompl_planning.yaml（文件名规则 <pipeline>_planning.yaml）。
    # 2026-08-25 时序/性能对齐 robotarm：
    #   - 延迟参数化（robot_spawn_delay / controller_spawn_delay），move_group 不再延迟
    #   - include_camera_visual=false 简化相机几何体，RViz 不卡
    # 2026-08-27：S1 已解决 spawner 竞态（超时参数），S2 时序调整已回退（见
    #   docs/2026-08-27_双臂控制器启动竞态），恢复 robotarm 对齐时序。
    robot_spawn_delay = DeclareLaunchArgument('robot_spawn_delay', default_value='5.0')
    controller_spawn_delay = DeclareLaunchArgument('controller_spawn_delay', default_value='8.0')
    rv_spawn_delay = LaunchConfiguration('robot_spawn_delay')
    ctrl_spawn_delay = LaunchConfiguration('controller_spawn_delay')

    dual_arm_gazebo_xacro = os.path.join(
        this_pkg, "config", "s622_dual_arm_gazebo.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder("s622_dual_arm", package_name="s622_moveit_config")
        .robot_description(
            file_path=dual_arm_gazebo_xacro,
            mappings={
                "instantiate": "false",
                # 2026-08-25：相机简化几何体（对齐单臂 include_camera_visual=false）
                "include_camera_visual": "false",
            },
        )
        .robot_description_semantic(file_path="config/s622_dual_arm.srdf")
        .robot_description_kinematics(file_path="config/dual_arm_kinematics.yaml")
        .trajectory_execution(file_path="config/dual_arm_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["fairino", "ompl"], default_planning_pipeline="fairino")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # 加载规划管线参数（S3，对齐单臂 s622_gazebo.launch.py）
    # FairinoPlannerManager 从这些参数读取：
    #   - fairino_planning: 顶层 planning_plugin + request_adapters
    #   - planning_core: planner.* / fairino.optimizer.* 等
    #   - aapf/tube/birrt/rrt star core: fairino.algorithms.<name>.*
    #   - ik_core: fairino.ik.*
    fairino_planning = load_yaml("s622_moveit_config", "config/fairino_planning.yaml")
    planning_core = load_yaml("fairino_planning_core", "config/common_planning_params.yaml")
    aapf_star_core = load_yaml("fairino_planning_core", "config/aapf_birrt__params.yaml")
    tube_star_core = load_yaml("fairino_planning_core", "config/tube_birrt__params.yaml")
    birrt_star_core = load_yaml("fairino_planning_core", "config/birrt__params.yaml")
    rrt_star_core = load_yaml("fairino_planning_core", "config/rrt__params.yaml")
    ik_core = load_yaml("fairino_planning_core", "config/ik_params.yaml")

    dual_arm_kinematics_fairino = load_yaml(
        "s622_moveit_config", "config/dual_arm_kinematics_fairino.yaml")
    dual_arm_kinematics_kdl = load_yaml(
        "s622_moveit_config", "config/dual_arm_kinematics_kdl.yaml")

    # ============ 4. Spawn robot（2026-08-25：延迟参数化，对齐 robotarm） ============
    # 把 package:// URI 替换成绝对路径, 让 gz sim 能找到 mesh
    gz_urdf = moveit_config.robot_description["robot_description"].replace(
        "package://s622_moveit_descriptions", robot_desc_pkg
    )
    spawn_robot = TimerAction(
        period=rv_spawn_delay,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-string", gz_urdf,
                    "-x", "0.0", "-y", "0.0", "-z", "0.0",
                    "-R", "0", "-P", "0", "-Y", "0",
                    "-name", "s622_dual_arm",
                ],
                output="screen",
            )
        ],
    )

    # ============ 5. Spawn target cube（默认 robot_spawn_delay=5 + 2 = 7s，晚于 robot） ============
    # 双臂布置: cube 放桌面中间偏左, 靠近左臂工作区
    spawn_box = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", "dual_arm_world",
                    "-file", os.path.join(this_pkg, "models", "target_box", "model.sdf"),
                    "-name", "target_box",
                    "-x", "0.6", "-y", "0.3", "-z", "0.05",
                    "-R", "0", "-P", "0", "-Y", "0.0",
                ],
            )
        ],
    )

    # ============ 6. robot_state_publisher ============
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # ============ 7. Controllers spawner（2026-08-25：延迟参数化，对齐 robotarm）
    #    JSB 先（controller_spawn_delay=8s），arm/hand 后（+1s）
    #    2026-08-27 S1：spawner 加长 service 超时（--service-call-timeout 60 等），
    #    解决启动风暴下 10s 默认超时误判（S2 时序调整已回退，S1 足够） ============
    dual_arm_controllers_yaml = os.path.join(
        robot_moveit_pkg, "config", "dual_arm_controllers.yaml"
    )

    jsb_spawner = TimerAction(
        period=ctrl_spawn_delay,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "-p", dual_arm_controllers_yaml,
                    "joint_state_broadcaster",
                    # 2026-08-27 S1：启动风暴下 CM service 响应可能 >10s，
                    # 默认 --service-call-timeout=10.0 会导致 spawner 误判失败重试
                    # （already loaded）→ 后续 controller 不启动。拉长防御。
                    "--service-call-timeout", "60.0",
                    "--controller-manager-timeout", "60.0",
                    "--switch-timeout", "60.0",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    arm_hand_spawner = TimerAction(
        period=9.0,   # 默认 ctrl=8 + 1（固定偏移，避免 LaunchConfiguration 运算）
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "-p", dual_arm_controllers_yaml,
                    "left_arm_controller",
                    "left_hand_controller",
                    "right_arm_controller",
                    "right_hand_controller",
                    # 2026-08-27 S1：同上，left_arm_controller 扛启动风暴，
                    # 10s 默认超时曾致误判失败（详见 docs/2026-08-27_双臂控制器启动竞态）
                    "--service-call-timeout", "60.0",
                    "--controller-manager-timeout", "60.0",
                    "--switch-timeout", "60.0",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    # ============ 8. planning_scene_service（2026-08-25：controller 后 +1s，不再等 25s） ============
    planning_scene = TimerAction(
        period=10.0,   # 默认 ctrl=8 + 2
        actions=[
            Node(
                package="s622_arm_actions",
                executable="planning_scene_service",
                name="planning_scene_service",
                parameters=[{
                    "use_sim_time": True,
                    "base_link": "world",
                    "default_object_size": [0.04, 0.04, 0.04],
                    "default_touch_links": [
                        "left_finger1", "left_finger2", "left_grasp_frame",
                        "right_finger1", "right_finger2", "right_grasp_frame",
                    ],
                    "publish_table": True,
                    "table_size": [1.5, 0.8, 0.03],
                    "table_center": [0.0, 0.0, -0.015],
                }],
                output="screen",
            )
        ],
    )

    # ============ 9. move_group × 2（S3 双臂现代化，对齐单臂 namespaced 架构）
    #   /move_group_fairino：left_arm/right_arm 用 FairinoIKPlugin 解析 IK + fairino 规划管线
    #   /move_group_kdl：KDL 兜底（kinematics 用 KDL，管线仍 fairino+ompl）
    #   服务保持 namespaced，客户端（pymoveit2 move_group_namespace + RViz）显式连接。
    #   dual_arm 组（12-DOF 联合规划）不配 IK，两个实例都能做关节空间规划。
    #   2026-08-25：立即启动（对齐 robotarm，move_group 不延迟；controller 稍后就绪）。
    #   2026-08-27：S2 曾延后到 14s，已回退（S1 超时参数足够，见 docs/2026-08-27_双臂控制器启动竞态）。
    mg_remappings = [
        ("joint_states", "/joint_states"),
        ("trajectory_execution_event", "/trajectory_execution_event"),
        ("planning_scene", "/planning_scene"),
        ("collision_object", "/collision_object"),
        ("attached_collision_object", "/attached_collision_object"),
        # controller action client remap（对齐单臂经验：源名相对 + 5-topic 兜底）
        ("left_arm_controller/follow_joint_trajectory", "/left_arm_controller/follow_joint_trajectory"),
        ("left_hand_controller/follow_joint_trajectory", "/left_hand_controller/follow_joint_trajectory"),
        ("right_arm_controller/follow_joint_trajectory", "/right_arm_controller/follow_joint_trajectory"),
        ("right_hand_controller/follow_joint_trajectory", "/right_hand_controller/follow_joint_trajectory"),
        ("left_arm_controller/follow_joint_trajectory/_action/feedback", "/left_arm_controller/follow_joint_trajectory/_action/feedback"),
        ("left_arm_controller/follow_joint_trajectory/_action/status", "/left_arm_controller/follow_joint_trajectory/_action/status"),
        ("left_arm_controller/follow_joint_trajectory/_action/cancel_goal", "/left_arm_controller/follow_joint_trajectory/_action/cancel_goal"),
        ("left_arm_controller/follow_joint_trajectory/_action/get_result", "/left_arm_controller/follow_joint_trajectory/_action/get_result"),
        ("left_arm_controller/follow_joint_trajectory/_action/send_goal", "/left_arm_controller/follow_joint_trajectory/_action/send_goal"),
        ("left_hand_controller/follow_joint_trajectory/_action/feedback", "/left_hand_controller/follow_joint_trajectory/_action/feedback"),
        ("left_hand_controller/follow_joint_trajectory/_action/status", "/left_hand_controller/follow_joint_trajectory/_action/status"),
        ("left_hand_controller/follow_joint_trajectory/_action/cancel_goal", "/left_hand_controller/follow_joint_trajectory/_action/cancel_goal"),
        ("left_hand_controller/follow_joint_trajectory/_action/get_result", "/left_hand_controller/follow_joint_trajectory/_action/get_result"),
        ("left_hand_controller/follow_joint_trajectory/_action/send_goal", "/left_hand_controller/follow_joint_trajectory/_action/send_goal"),
        ("right_arm_controller/follow_joint_trajectory/_action/feedback", "/right_arm_controller/follow_joint_trajectory/_action/feedback"),
        ("right_arm_controller/follow_joint_trajectory/_action/status", "/right_arm_controller/follow_joint_trajectory/_action/status"),
        ("right_arm_controller/follow_joint_trajectory/_action/cancel_goal", "/right_arm_controller/follow_joint_trajectory/_action/cancel_goal"),
        ("right_arm_controller/follow_joint_trajectory/_action/get_result", "/right_arm_controller/follow_joint_trajectory/_action/get_result"),
        ("right_arm_controller/follow_joint_trajectory/_action/send_goal", "/right_arm_controller/follow_joint_trajectory/_action/send_goal"),
        ("right_hand_controller/follow_joint_trajectory/_action/feedback", "/right_hand_controller/follow_joint_trajectory/_action/feedback"),
        ("right_hand_controller/follow_joint_trajectory/_action/status", "/right_hand_controller/follow_joint_trajectory/_action/status"),
        ("right_hand_controller/follow_joint_trajectory/_action/cancel_goal", "/right_hand_controller/follow_joint_trajectory/_action/cancel_goal"),
        ("right_hand_controller/follow_joint_trajectory/_action/get_result", "/right_hand_controller/follow_joint_trajectory/_action/get_result"),
        ("right_hand_controller/follow_joint_trajectory/_action/send_goal", "/right_hand_controller/follow_joint_trajectory/_action/send_goal"),
        ("robot_description", "/robot_description"),
        ("robot_description_semantic", "/robot_description_semantic"),
    ]

    mg_common_params = [
        moveit_config.to_dict(),
        fairino_planning,
        planning_core,
        aapf_star_core,
        tube_star_core,
        birrt_star_core,
        rrt_star_core,
        ik_core,
        {"planner": {"random_seed": 0}},
        {"use_sim_time": True},
    ]

    move_group_fairino = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="move_group_fairino",
        name="move_group",
        output="screen",
        remappings=mg_remappings,
        parameters=[
            *mg_common_params,
            {"fairino": {"ik": {"task_profile": "grasp"}}},
            {"robot_description_kinematics": dual_arm_kinematics_fairino},
        ],
    )

    move_group_kdl = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="move_group_kdl",
        name="move_group",
        output="screen",
        remappings=mg_remappings,
        parameters=[
            *mg_common_params,
            {"robot_description_kinematics": dual_arm_kinematics_kdl},
        ],
    )

    # ============ 10. RViz（2026-08-25：controller 后 +1s=9s，对齐 robotarm；面板可见管线列表） ============
    rviz_config = os.path.join(robot_moveit_pkg, "config", "dual_arm.rviz")
    rviz = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    moveit_config.planning_pipelines,
                    fairino_planning,
                    {"use_sim_time": True},
                ],
            )
        ],
    )
    
    # ============ 11. MoveIt Servo (双实例, 各自 namespace; 2026-08-25: controller 后 +2s=10s) ============
    left_servo_yaml_path = os.path.join(
        robot_moveit_pkg, "config", "left_servo_config.yaml"
    )
    right_servo_yaml_path = os.path.join(
        robot_moveit_pkg, "config", "right_servo_config.yaml"
    )

    with open(left_servo_yaml_path, 'r') as f:
        left_servo_params = {"moveit_servo": yaml.safe_load(f)}
    with open(right_servo_yaml_path, 'r') as f:
        right_servo_params = {"moveit_servo": yaml.safe_load(f)}

    left_servo_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                name="servo_node",
                namespace="left",
                parameters=[
                    left_servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {"use_sim_time": True},
                ],
                remappings=[
                    # namespaced node 默认订阅 /left/tf, 必须 remap 回全局 TF
                    ("/tf", "/tf"),
                    ("/tf_static", "/tf_static"),
                    # 全局 joint_states；planning scene 用 fairino move_group 实例的
                    ("/left/joint_states", "/joint_states"),
                    ("/left/monitored_planning_scene", "/move_group_fairino/monitored_planning_scene"),
                ],
                output="screen",
            )
        ],
    )

    right_servo_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                name="servo_node",
                namespace="right",
                parameters=[
                    right_servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {"use_sim_time": True},
                ],
                remappings=[
                    ("/tf", "/tf"),
                    ("/tf_static", "/tf_static"),
                    ("/right/joint_states", "/joint_states"),
                    ("/right/monitored_planning_scene", "/move_group_fairino/monitored_planning_scene"),
                ],
                output="screen",
            )
        ],
    )

    arm_actions_launch = TimerAction(
        period=11.0,   # 在 servo (10s) 之后 1s
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('s622_arm_actions'),
                        'launch', 'arm_actions_dual.launch.py'
                    )
                ),
            )
        ],
    )
    
    # ============ 12. obb_node：入口脚本 shebang 硬编码了 /usr/bin/python3， ============
    # prefix 指定 conda 环境的 Python3 解释器，覆盖 shebang
    from os.path import expanduser

    model_path = expanduser("~/my_S622/src/yolov8_obb/models/best.pt")
    obb_node = Node(
        package="yolov8_obb",
        executable="yolov8_obb_node",
        name="obb_node",
        prefix='/home/yep/miniconda3/envs/yolov8/bin/python3',
        parameters=[
            {"model_path": model_path,
             "image_topic": "/camera/color/image_raw",
             "detections_topic": "/yolov8/obb_detections",
             "confidence_threshold": 0.05,
             "device": "auto",
             "imgsz": 1024,
             "publish_empty": True},
        ]
    )
    
    # ============ 13. BT Executor (双臂公用一份, arm 通过参数切换) ============
    bt_manager_pkg = get_package_share_directory("s622_bt_manager")
    bt_dual_config = os.path.join(bt_manager_pkg, "config", "bt_dual_config.yaml")
    tree_file_arg = DeclareLaunchArgument(
        'tree_file', default_value='pick_place_dual.xml',
        description='BT XML file: pick_place_dual.xml | pick_handover_place.xml')
    tree_id_arg = DeclareLaunchArgument(
        'tree_id', default_value='PickPlaceDual',
        description='BT root ID matching tree_file')
    
    bt_executor = TimerAction(
        period=13.0,   # 在 arm_actions (11s) 之后 2s, planning_scene (10s) 之后 3s
        actions=[
            Node(
                package="s622_bt_manager",
                executable="bt_executor_node",
                name="bt_executor",
                parameters=[
                    bt_dual_config,
                    {
                        "tree_file": LaunchConfiguration('tree_file'),  # ← 改
                        "tree_id":   LaunchConfiguration('tree_id'),    # ← 改
                        "tick_rate_hz": 10,
                        "auto_start": False,
                        "yolo_topic": "/yolov8/obb_detections",
                        "depth_topic": "/camera/depth/image_raw",
                        "caminfo_topic": "/camera/color/camera_info",
                        "grasp_viz_topic": "/grasp_visualization",
                        "arm": "left",   # 默认左臂, 可 ros2 param set 改
                        "use_sim_time": True,
                    },
                ],
                output="screen",
            )
        ],
    )
    
    # ============ 14. trajectory_retime_server（S3 对齐单臂 D2；2026-08-27：S2 曾延后已回退） ============
    retime_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("trajectory_retime_server"),
                "launch",
                "retime_server.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription([
        set_model_path, tree_file_arg, tree_id_arg,
        robot_spawn_delay, controller_spawn_delay,
        gazebo, clock_bridge, camera_bridge,
        spawn_robot, spawn_box,
        robot_state_pub,
        jsb_spawner, arm_hand_spawner,
        planning_scene,
        move_group_fairino, move_group_kdl, rviz,
        left_servo_node, right_servo_node,
        arm_actions_launch, obb_node, bt_executor,
        retime_server_launch,
    ])
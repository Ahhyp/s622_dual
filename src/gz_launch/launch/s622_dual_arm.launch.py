# s622_dual_arm.launch.py
# M2.4: 双臂完整环境启动
# 组件: Gazebo(world+双臂+桌子+cube) + RSP + CM(gz_ros2_control) + 5 controllers
#       + move_group + RViz + planning_scene_service(双臂 touch_links) + camera_bridge
# 不含: obb_node(M2.7 改造), servo(M2.5)
import os
from pathlib import Path
import yaml
import launch
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import load_xacro  # noqa: E402
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression
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
    # [M4 提速] gui:=false → gz 只跑 server（-s）。GUI 实测吃 ~2.2 核,
    # 被杀时还会带走 server（同一进程组）→ 做批量实验时一律 headless。
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', description='gz GUI（false=只跑 server, 省 ~2 核）')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='rviz2（false=不启动, 省 ~2 核）')
    world_file = os.path.join(this_pkg, "worlds", "dual_arm_world.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", [
            world_file + " -r",
            PythonExpression(
                ["' -s' if '", LaunchConfiguration('gui'), "' == 'false' else ''"]),
        ])],
    )

    # ============ 2. Bridges ============
    # [M2.7] M2_MODE 环境变量（s622_global_handeye_sim 设置）决定右腕相机 bridge 是否启用：
    #   base        → 腕部相机开（默认双臂场景）
    #   right/left  → 腕部相机关（M2.7 全局标定，省渲染提帧率）
    _m2_mode_bridge = os.environ.get("M2_MODE", "base")

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/world/dual_arm_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        remappings=[("/world/dual_arm_world/clock", "/clock")],
    )

    # [M4_2 / Plan B'] 夹爪"夹持力"通道：ROS std_msgs/Float64 → gz Double（apply_joint_force 插件）
    # 语法（见 `ros2 run ros_gz_bridge parameter_bridge --help`）：
    #   <topic@ROS类型@GZ类型>，方向符号紧跟 ROS 类型：@双向 / [ = GZ→ROS / ] = ROS→GZ
    # 本机 bridge 0.244.x 用 gz.msgs.* 命名（不是 ignition.msgs.*）。
    # 仅在仿真启用（GRIPPER_FORCE_PLUGINS=1 时插件才订阅；桥本身无害）
    _force_bridge_topics = [
        f"/model/s622_dual_arm/joint/{arm}_finger{i}_joint/cmd_force"
        for arm in ("left", "right")
        for i in (1, 2)
    ]
    gripper_force_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"{t}@std_msgs/msg/Float64]gz.msgs.Double" for t in _force_bridge_topics],
        parameters=[{"use_sim_time": True}],
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

    # [M2.7] 右腕相机桥接（M2_MODE=right/left 时腕部相机不存在 → 不桥接，避免报错）
    wrist_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/wrist_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/wrist_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/wrist_camera/image", "/wrist_camera/color/image_raw"),
            ("/wrist_camera/depth_image", "/wrist_camera/depth/image_raw"),
            ("/wrist_camera/camera_info", "/wrist_camera/color/camera_info"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=launch.conditions.IfCondition(
            "false" if _m2_mode_bridge in ("right", "left") else "true"),
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
    # [M4 §7.21] 仿真侧夹爪"机械限位"（等效真机拧螺丝定的最小闭合间隙）。
    # 默认 0.0 = 原行为（close 压到 close_positions=[0,0]）；跑长条交接时传
    #   gripper_close_stop_gap:=0.029   （30mm 杆留 1mm 过盈）
    gripper_close_stop_gap = DeclareLaunchArgument(
        'gripper_close_stop_gap', default_value='0.0',
        description='[sim] 夹爪最小闭合间隙(m), 0=不启用(原行为)')
    # [M4_2 / A''] 仿真专用：close 绕过 MoveIt 直发（配合 GRIPPER_STOP_GAP 关节限位产生夹持力）
    gripper_direct_close = DeclareLaunchArgument(
        'gripper_direct_close', default_value='false',
        description='[sim] true=close 绕过 MoveIt 直发轨迹(A\'\'方案)')
    # [M4_2 / Plan B'] 仿真纯力控夹爪（气动模型）；默认关闭
    gripper_force_mode = DeclareLaunchArgument(
        'gripper_force_mode', default_value='false',
        description='[sim] true=夹爪力控(close 施闭合恒力/open 施张开力)')
    gripper_clamp_force = DeclareLaunchArgument('gripper_clamp_force', default_value='5.0')
    gripper_open_force = DeclareLaunchArgument('gripper_open_force', default_value='2.0')
    rv_spawn_delay = LaunchConfiguration('robot_spawn_delay')
    ctrl_spawn_delay = LaunchConfiguration('controller_spawn_delay')

    # [M2.7] M2.7 场景由 s622_global_handeye_sim.launch.py 设置 M2_MODE 环境变量后
    # include 本文件；本文件构造期读取并决定 xacro mappings（全 str）。
    # mappings 保持全 str —— 若塞 LaunchConfiguration，robot_description 变
    # ParameterValue，move_group 的 to_dict() 会在 launch 展开时崩溃
    # （'ParameterValue' object has no attribute 'perform'）。
    _m2_mode = os.environ.get("M2_MODE", "base")
    _m2_global = "true" if _m2_mode in ("right", "left") else "false"
    _m2_wrist = "false" if _m2_mode in ("right", "left") else "true"
    _m2_calib = _m2_mode if _m2_mode in ("right", "left") else "none"

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
                # [M2.7] 由 M2_MODE 环境变量决定（base=默认 / right / left）
                "include_global_camera": _m2_global,
                "include_wrist_camera": _m2_wrist,
                "calibration_arm": _m2_calib,
                # [M4_2 / A''] 夹爪"关节限位"路线（已证会与初始位冲突导致 finger1 锁死，默认关闭）。
                # 注意：与下面 S3 的限位块用【不同】环境变量，避免误开。
                "finger_stop_gap": os.environ.get("GRIPPER_JOINT_LIMIT_GAP", "0.0"),
                # [M4_2 / S3] 夹爪"物理限位块"（推荐路线；默认关闭）
                #   GRIPPER_STOP_BLOCK=1 GRIPPER_STOP_GAP=0.028 → 两爪各加一个挡块
                "gripper_stop_block": os.environ.get("GRIPPER_STOP_BLOCK", "false"),
                "gripper_stop_gap": os.environ.get("GRIPPER_STOP_GAP", "0.028"),
                # [M4_2 / Plan B'] 启用 gz 关节力插件（=仿真夹持力来源）
                "gripper_force_plugins": os.environ.get("GRIPPER_FORCE_PLUGINS", "false"),
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
    # 把 package:// URI 替换成绝对路径, 让 gz sim 能找到 mesh。
    # [M2.7] mappings 全 str → robot_description 是纯 str，直接 replace（不需 OpaqueFunction）
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

    # ============ 5b. Spawn M5 regrasp fixture（§2：正式实验不依赖 tmp 手工 spawn）============
    # 标称位姿 (0.10, -0.20, 0) 已做右臂可达性验证（tmp/m5_round/fixture_solve.py：
    # 四个目标位姿残差 ≤0.11mm/0.00°，限位裕度 45.6~67.1°）。
    # 夹具模型建在局部系（原点=落地中心，杆长轴=局部 Y），换位只改下面三个参数。
    fixture_enable_arg = DeclareLaunchArgument(
        'spawn_regrasp_fixture', default_value='true',
        description='[M5] 是否生成重抓夹具')
    fixture_x_arg = DeclareLaunchArgument('fixture_x', default_value='0.10')
    fixture_y_arg = DeclareLaunchArgument('fixture_y', default_value='-0.20')
    fixture_yaw_arg = DeclareLaunchArgument('fixture_yaw', default_value='0.0')

    spawn_fixture = TimerAction(
        period=7.5,   # 略晚于 target_box，避免同时打 Gazebo 的 create 服务
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", "dual_arm_world",
                    "-file", os.path.join(this_pkg, "models", "regrasp_fixture", "model.sdf"),
                    "-name", "regrasp_fixture",
                    "-x", LaunchConfiguration('fixture_x'),
                    "-y", LaunchConfiguration('fixture_y'),
                    "-z", "0.0",
                    "-R", "0", "-P", "0",
                    "-Y", LaunchConfiguration('fixture_yaw'),
                ],
                condition=IfCondition(LaunchConfiguration('spawn_regrasp_fixture')),
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
                condition=IfCondition(LaunchConfiguration('rviz')),
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
                # [M4 §7.21] 把仿真限位透传给 left/right gripper_service
                launch_arguments={
                    'gripper_close_stop_gap': LaunchConfiguration('gripper_close_stop_gap'),
                    'gripper_direct_close': LaunchConfiguration('gripper_direct_close'),
                    'gripper_force_mode': LaunchConfiguration('gripper_force_mode'),
                    'gripper_clamp_force': LaunchConfiguration('gripper_clamp_force'),
                    'gripper_open_force': LaunchConfiguration('gripper_open_force'),
                }.items(),
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
    # [M4 BT 回填] 长条交接参数（bb.* → blackboard，行为树 XML 用 {var} 引用）
    m4_handover_config = os.path.join(bt_manager_pkg, "config", "m4_handover.yaml")
    # [M5] §15：夹具/Anchor/运动原语参数唯一来源
    m5_task_config = os.path.join(bt_manager_pkg, "config", "m5_task.yaml")
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
                    m4_handover_config,
                    m5_task_config,
                    {
                        "subtree_files": "handover_rod.xml,m5_fixture_place_test.xml",
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
    
    # ============ 14. trajectory_retime_server（S3 对齐单臂 D2） ============
    # 2026-08-27 S4：必须传双臂 URDF/SRDF/kinematics——retime_server.launch.py 默认加载
    # 单臂模型（robot_gazebo.urdf.xacro，group=robot_arm）；双臂轨迹（left_j1.. / dual_arm 组）
    # 的 joint 集合/group 在单臂模型上不存在 → retime（TOTG）失败。双臂 launch 覆盖三参数。
    # [M2.7] robot_description 用基础 xacro（mappings 全 str）重新生成一份纯字符串，
    # 不依赖 moveit_config.robot_description（后者在含相机/标定板运行时参数时是
    # ParameterValue，不能直接放进 launch_arguments）。
    with open(os.path.join(robot_moveit_pkg, "config", "s622_dual_arm.srdf"),
              "r", encoding="utf-8") as f:
        dual_arm_srdf_str = f.read()
    retime_robot_description_xml = load_xacro(
        Path(dual_arm_gazebo_xacro),
        mappings={"instantiate": "false", "include_camera_visual": "false"},
    )
    retime_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("trajectory_retime_server"),
                "launch",
                "retime_server.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "robot_description": retime_robot_description_xml,
            "robot_description_semantic": dual_arm_srdf_str,
            "robot_description_kinematics": yaml.safe_dump(dual_arm_kinematics_fairino),
        }.items(),
    )

    return LaunchDescription([
        set_model_path, tree_file_arg, tree_id_arg, gui_arg, rviz_arg,
        robot_spawn_delay, controller_spawn_delay, gripper_close_stop_gap, gripper_direct_close,
        fixture_enable_arg, fixture_x_arg, fixture_y_arg, fixture_yaw_arg,
        gripper_force_mode, gripper_clamp_force, gripper_open_force,
        gazebo, clock_bridge, camera_bridge, wrist_camera_bridge, gripper_force_bridge,
        spawn_robot, spawn_box, spawn_fixture,
        robot_state_pub,
        jsb_spawner, arm_hand_spawner,
        planning_scene,
        move_group_fairino, move_group_kdl, rviz,
        left_servo_node, right_servo_node,
        arm_actions_launch, obb_node, bt_executor,
        retime_server_launch,
    ])
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


def generate_launch_description():
    this_pkg = get_package_share_directory("gz_launch")
    robot_desc_pkg = get_package_share_directory("s622_moveit_descriptions")
    robot_moveit_pkg = get_package_share_directory("s622_moveit_config")
    fairino_planning_pkg = get_package_share_directory("fairino_planning_ros")

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
    dual_arm_gazebo_xacro = os.path.join(
        this_pkg, "config", "s622_dual_arm_gazebo.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder("s622_dual_arm", package_name="s622_moveit_config")
        .robot_description(file_path=dual_arm_gazebo_xacro, mappings={"instantiate": "false"})
        .robot_description_semantic(file_path="config/s622_dual_arm.srdf")
        .robot_description_kinematics(file_path="config/dual_arm_kinematics.yaml")
        .trajectory_execution(file_path="config/dual_arm_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # 自定义 planning pipeline 参数 (M1.7 有的话保留)
    fairino_planning_yaml = os.path.join(
        fairino_planning_pkg, "config", "fairino_planning.yaml"
    )
    with open(fairino_planning_yaml, 'r') as f:
        pipeline_params = yaml.safe_load(f)

    # ============ 4. Spawn robot ============
    # 把 package:// URI 替换成绝对路径, 让 gz sim 能找到 mesh
    gz_urdf = moveit_config.robot_description["robot_description"].replace(
        "package://s622_moveit_descriptions", robot_desc_pkg
    )
    spawn_robot = Node(
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

    # ============ 5. Spawn target cube ============
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

    # ============ 7. Controllers spawner (分两轮: JSB 先, arm/hand 后) ============
    dual_arm_controllers_yaml = os.path.join(
        robot_moveit_pkg, "config", "dual_arm_controllers.yaml"
    )

    jsb_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "-p", dual_arm_controllers_yaml,
                    "joint_state_broadcaster",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    arm_hand_spawner = TimerAction(
        period=12.0,
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
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    # ============ 8. planning_scene_service (双臂 touch_links, table 中心 0,0) ============
    planning_scene = TimerAction(
        period=25.0,
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

    # ============ 9. move_group (等 controller_manager 起来后) ============
    move_group = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[
                    moveit_config.to_dict(),
                    pipeline_params,
                    {"use_sim_time": True},
                ],
                output="screen",
            )
        ],
    )

    # ============ 10. RViz ============
    rviz_config = os.path.join(robot_moveit_pkg, "config", "dual_arm.rviz")
    rviz = TimerAction(
        period=28.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {"use_sim_time": True},
                ],
            )
        ],
    )
    
    # ============ 11. MoveIt Servo (双实例, 各自 namespace) ============
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
        period=17.0,
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
                    # 全局 joint_states 和 planning_scene (由 move_group 单例发)
                    ("/left/joint_states", "/joint_states"),
                    ("/left/monitored_planning_scene", "/monitored_planning_scene"),
                ],
                output="screen",
            )
        ],
    )

    right_servo_node = TimerAction(
        period=17.0,
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
                    ("/right/monitored_planning_scene", "/monitored_planning_scene"),
                ],
                output="screen",
            )
        ],
    )

    arm_actions_launch = TimerAction(
        period=18.0,   # 在 servo (17s) 之后
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
        period=22.0,   # 在 arm_actions (18s) 之后, planning_scene (25s) 之前
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
    
    # ============ 14. Fairino IK Service (DH 解析法, 提供 /fairino/get_all_ik) ============
    fairino_ik_yaml = os.path.join(
        get_package_share_directory("fairino_planning_core"),
        "config", "fairino_ik_service.yaml"
    )
    fairino_ik_service = Node(
        package="fairino_planning_core",
        executable="fairino_ik_service_node",
        name="fairino_ik_service",
        parameters=[
            fairino_ik_yaml,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return LaunchDescription([
        set_model_path,tree_file_arg, tree_id_arg,         # ← 新增
        gazebo, clock_bridge, camera_bridge,
        spawn_robot, spawn_box,
        robot_state_pub,
        jsb_spawner, arm_hand_spawner,
        planning_scene,
        move_group, rviz,
        left_servo_node, right_servo_node,
        arm_actions_launch, obb_node, bt_executor,
        fairino_ik_service,
    ])
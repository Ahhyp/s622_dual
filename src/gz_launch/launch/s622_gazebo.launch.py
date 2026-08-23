# s622_gazebo.launch.py
# 启动 Gazebo + spawn 机械臂 + ros2_control + MoveIt2 + RViz
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable
from manipulation_common.launch_utils.yaml_loader import load_yaml


def generate_launch_description():
    this_pkg = get_package_share_directory("gz_launch")
    robot_desc_pkg = get_package_share_directory("s622_moveit_descriptions")
    robot_moveit_pkg = get_package_share_directory("s622_moveit_config")
    
    set_model_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.path.join(this_pkg, "models")
        + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )
    # 加载自定义规划管线
    fairino_planning_yaml = os.path.join(
        get_package_share_directory("fairino_planning_ros"),
        "config", "fairino_planning.yaml"
    )

    # 1. Gazebo 空世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ros_gz_sim") + "/launch/gz_sim.launch.py"
        ]),
        # 自定义 world：物理步长 0.005（200Hz，匹配 controller 200Hz），RTF 大幅提升
        launch_arguments=[("gz_args", os.path.join(this_pkg, "worlds", "s622_world.sdf") + " -r")]
    )

    # 2. 时钟桥接
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        remappings=[("/world/empty/clock", "/clock")]
    )
    
    # 相机图像桥接
    # /camera/image@sensor_msgs/msg/Image@gz.msgs.Image
    #  ↑ROS2话题       ↑ROS2消息类型          ↑Gazebo消息类型
    # /camera/image ROS2 话题名
    # sensor_msgs/msg/Image ROS2 消息类型
    # gz.msgs.Image Gazebo 消息类型
    # 把 Gazebo 相机产生的 gz.msgs.Image 转成 sensor_msgs/msg/Image，
    # 发到 /camera/image 话题上。 你的 YOLOv8 节点订阅 sensor_msgs/msg/Image 类型
    # 的 /camera/image，就能收到仿真画面了。
    # 这个是双向桥接 arguments：告诉 parameter_bridge 要桥接哪个话题以及消息类型如何转换。
    # 创建一个 ROS 2 话题 /camera/image，类型为 sensor_msgs/msg/Image。
    # 关联到一个 Gazebo / Ignition 的同名话题 /camera/image，类型为 gz.msgs.Image。
    # 在两个中间件之间互相转发消息：当一端收到消息时，自动转换为另一端的格式并发布出去
    
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ],
        remappings=[  # remap 的好处：以后从 Gazebo 切到真实 RealSense，只要换 launch，节点代码一行不改。
            ("/camera/image", "/camera/color/image_raw"),
            ("/camera/depth_image", "/camera/depth/image_raw"),
            ("/camera/camera_info", "/camera/color/camera_info"),
        ],
        parameters=[{"use_sim_time": True}], # parameters 是一个节点参数列表
        output="screen",
    )
    

    # 3. MoveIt 配置（告诉 move_group 用哪个 URDF/SRDF/kinematics）
    moveit_config = MoveItConfigsBuilder("s622_moveit_descriptions", package_name="s622_moveit_config") \
                    .robot_description(this_pkg + '/config/robot_gazebo.urdf.xacro') \
                    .robot_description_semantic('config/s622_moveit_descriptions.srdf') \
                    .robot_description_kinematics(robot_moveit_pkg + '/config/kinematics.yaml') \
                    .planning_pipelines(pipelines=["ompl"],default_planning_pipeline="ompl") \
                    .to_moveit_configs()
    
    
    
    # 4. Spawn 机械臂到 Gazebo
    gz_urdf = moveit_config.robot_description["robot_description"].replace(
        "package://s622_moveit_descriptions", robot_desc_pkg
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", gz_urdf,
                   "-x", "0.0", "-y", "0.0", "-z", "0.0",
                   "-R", "0", "-P", "0", "-Y", "0",
                   "-name", "robot_arm"]
    )
    
    # 加在 variables 区：让 Gazebo 找到你的模型目录
    set_model_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.path.join(this_pkg, "models"),
    )

    # 加在 return LaunchDescription 之前：spawn 方块
    # [0.45, 0.0, 0.15] [0, 0, 0.5] -> planning to pregrasp (0.444, 0.136, 0.117), yaw=1.13
    # [0.45, 0.1, 0.15] -> planning to pregrasp (0.444, 0.210, 0.122), yaw=1.10
    # [0.45, 0.2, 0.15] -> planning to pregrasp (0.447, 0.284, 0.126), yaw=1.06
    # [0.35, 0.1, 0.15] -> planning to pregrasp (0.350, 0.210, 0.122), yaw=1.13
    # [0.45, 0.0, 0.15] [0, 0, 0.5] static:true ->
    # [0.29, 0.57, 0.15] [0, 0, 0.5] static:true 在图像中间 
    spawn_box = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-world", "empty",
                        "-file", os.path.join(this_pkg, "models", "target_box", "model.sdf"),
                        "-name", "target_box",
                        "-x", "0.36", "-y", "0.02", "-z", "0.10", "-R", "0", "-P", "0", "-Y", "0.00"],
            )
        ]
    )

    # 阶段 0 测试: 在桌面 spawn 一个独立 ArUco
    # 位置选在工作区内、不挡 target_box、相机能清楚看到
    # spawn_aruco_test = TimerAction(
    #     period=8.0,
    #     actions=[
    #         Node(
    #             package="ros_gz_sim",
    #             executable="create",
    #             arguments=["-world", "empty",
    #                     "-file", os.path.join(this_pkg, "models", "aruco_marker_0", "model.sdf"),
    #                     "-name", "aruco_test",
    #                     "-x", "0.30", "-y", "0.15", "-z", "0.02",
    #                     "-R", "0", "-P", "0", "-Y", "0"],
    #         )
    #     ]
    # )


    # 5. robot_state_publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}]
    )

    # 6. 控制器（延迟 5s 等 Gazebo 就绪）
    ros2_controllers_yaml = os.path.join(
        robot_moveit_pkg, "config", "ros2_controllers.yaml"
    )
    controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["-p", ros2_controllers_yaml,
                           "robot_arm_controller",
                           "joint_state_broadcaster",
                           "hand_controller"],
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ]
    )

    # 加载管线参数
    with open(fairino_planning_yaml, 'r') as f:
        pipeline_params = yaml.safe_load(f)

    # 7. move_group × 2（双 IK：/move_group_fairino 用 FairinoIKPlugin 解析 IK，/move_group_kdl 用 KDL）
    #    参照 robotarm moveit_stack.py 的 remap 设计；kinematics 用 MoveIt 标准格式注入
    kinematics_fairino = load_yaml("s622_moveit_config", "config/kinematics_fairino.yaml")
    kinematics_kdl = load_yaml("s622_moveit_config", "config/kinematics_kdl.yaml")

    mg_remappings = [
        ("joint_states", "/joint_states"),
        ("trajectory_execution_event", "/trajectory_execution_event"),
        ("planning_scene", "/planning_scene"),
        ("collision_object", "/collision_object"),
        ("attached_collision_object", "/attached_collision_object"),
        # controller action client remap（关键：源名相对，无前导 /，对齐 robotarm moveit_stack.py；
        # 另加 5-topic 兜底——实测顶层 action 名 remap 对 client 不总是生效）
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
        # monitored_planning_scene 不 remap 到根级：每个 move_group 发布到自己的
        # /move_group_*/monitored_planning_scene，由 RViz 端 remap 订阅 fairino 实例（对齐 robotarm）
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
        remappings=mg_remappings,
        parameters=[
            moveit_config.to_dict(),
            pipeline_params,
            {"robot_description_kinematics": kinematics_fairino},
            {"use_sim_time": True}
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
            moveit_config.to_dict(),
            pipeline_params,
            {"robot_description_kinematics": kinematics_kdl},
            {"use_sim_time": True}
        ],
    )


    # 8. RViz（MotionPlanning 面板默认连根级，remap 到 fairino move_group，
    #    对齐 robotarm moveit_stack.py:95-104——GUI 规划/执行与仿真保持一致）
    #    注意：action 顶层名 remap 对 client 不总是生效，另加 5-topic 兜底（同 controller 处理）
    rviz_config = os.path.join(this_pkg, "rviz", "gz_launch.rviz")
    rviz_remaps = [
        ("get_planning_scene", "/move_group_fairino/get_planning_scene"),
        ("plan_kinematic_path", "/move_group_fairino/plan_kinematic_path"),
        ("query_planner_interface", "/move_group_fairino/query_planner_interface"),
        ("compute_cartesian_path", "/move_group_fairino/compute_cartesian_path"),
        ("execute_trajectory", "/move_group_fairino/execute_trajectory"),
        ("move_action", "/move_group_fairino/move_action"),
        ("monitored_planning_scene", "/move_group_fairino/monitored_planning_scene"),
    ]
    for _act in ("execute_trajectory", "move_action"):
        for _sub in ("feedback", "status", "cancel_goal", "get_result", "send_goal"):
            rviz_remaps.append(
                (f"{_act}/_action/{_sub}", f"/move_group_fairino/{_act}/_action/{_sub}")
            )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description,
                     moveit_config.robot_description_semantic,
                     {"use_sim_time": True}],
        remappings=rviz_remaps,
    )

    
    # 9. MoveIt Servo
    servo_yaml_path = os.path.join(robot_moveit_pkg, "config", "servo.yaml")
    with open(servo_yaml_path, 'r') as f:
        servo_params = {"moveit_servo": yaml.safe_load(f)}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        output="screen",
    )
    
    # obb_node：入口脚本 shebang 硬编码了 /usr/bin/python3，
    # prefix 指定 conda 环境的 Python3 解释器，覆盖 shebang
    from os.path import expanduser

    model_path = expanduser("~/my_S622/src/yolov8_obb/models/yolo-obb-gazebo.pt")
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

    return LaunchDescription([
        set_model_path,
        gazebo, clock_bridge, camera_bridge,
        spawn_robot, spawn_box, 
        # spawn_aruco_test,
        robot_state_pub,
        rviz, controller_spawner, 
        move_group_fairino, move_group_kdl,
        servo_node,
        obb_node,
    ])
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
    #    规划管线统一（2026-08-25）：fairino（FairinoPlannerManager）+ ompl 备用，
    #    默认 fairino。MoveItConfigsBuilder 自动加载 config/fairino_planning.yaml
    #    （planning_plugin: fairino_planning/FairinoPlannerManager）与 config/ompl_planning.yaml。
    moveit_config = MoveItConfigsBuilder("s622_moveit_descriptions", package_name="s622_moveit_config") \
                    .robot_description(this_pkg + '/config/robot_gazebo.urdf.xacro') \
                    .robot_description_semantic('config/s622_moveit_descriptions.srdf') \
                    .robot_description_kinematics(robot_moveit_pkg + '/config/kinematics.yaml') \
                    .planning_pipelines(pipelines=["fairino", "ompl"], default_planning_pipeline="fairino") \
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

    # 加载规划管线参数（2026-08-25 规划管线统一，对齐 robotarm moveit_stack.py）
    # FairinoPlannerManager 从这些参数读取：
    #   - fairino_planning: 顶层 planning_plugin + request_adapters（legacy 算法参数）
    #   - planning_core: planner.* / fairino.optimizer.* 等（common_planning_params.yaml）
    #   - aapf/tube/birrt/rrt star core: fairino.algorithms.<name>.*（各算法参数）
    #   - ik_core: fairino.ik.*（IK 选择参数）
    fairino_planning = load_yaml("s622_moveit_config", "config/fairino_planning.yaml")
    planning_core = load_yaml("fairino_planning_core", "config/common_planning_params.yaml")
    aapf_star_core = load_yaml("fairino_planning_core", "config/aapf_birrt__params.yaml")
    tube_star_core = load_yaml("fairino_planning_core", "config/tube_birrt__params.yaml")
    birrt_star_core = load_yaml("fairino_planning_core", "config/birrt__params.yaml")
    rrt_star_core = load_yaml("fairino_planning_core", "config/rrt__params.yaml")
    ik_core = load_yaml("fairino_planning_core", "config/ik_params.yaml")

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
    # 2026-08-25 规划管线统一：注入 fairino_planning + planning_core + 4 算法 core + ik_core
    # （对齐 robotarm moveit_stack.py 的 fairino_parameters；task_profile=grasp 覆盖 ik_params 默认）
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
            fairino_planning,
            planning_core,
            aapf_star_core,
            tube_star_core,
            birrt_star_core,
            rrt_star_core,
            ik_core,
            {"planner": {"random_seed": 0}},
            {"robot_description_kinematics": kinematics_kdl},
            {"use_sim_time": True}
        ],
    )


    # 8. RViz —— 方案 A（2026-08-23 修复 namespace 振荡）
    # 面板连接 move_group 只靠 .rviz 配置的 "Move Group Namespace: /move_group_fairino"
    #（面板自加前缀直连绝对名）。**不再加 launch remap**：
    # 之前同时用 .rviz namespace + launch remap 双路径，导致面板在
    # "/ -> /move_group_fairino" 之间反复 reload（22 次），引发
    # "Link [X] does not exist"（64 次）、TF jump、最终 RViz 段错误崩溃。
    # robotarm 实际也只用 .rviz namespace（其 remap 中 query_planner_interfaces
    # 复数不匹配故基本无效），本方案与其对齐。
    rviz_config = os.path.join(this_pkg, "rviz", "gz_launch.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description,
                     moveit_config.robot_description_semantic,
                     moveit_config.robot_description_kinematics,
                     moveit_config.joint_limits,
                     moveit_config.planning_pipelines,
                     fairino_planning,
                     {"use_sim_time": True}],
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

    # 10. trajectory_retime_server（TOTG 时间最优重定时，2026-08-23 阶段 D2）
    #     pymoveit2 robotarm 版在 cartesian 路径规划后调用 /retime_trajectory，
    #     让 max_velocity/max_acceleration 缩放真正生效（Humble cartesian 轨迹无时间戳问题）
    #     use_sim_time 从外层传入（不硬编码在 retime_server.launch.py）
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
        set_model_path,
        gazebo, clock_bridge, camera_bridge,
        spawn_robot, spawn_box, 
        # spawn_aruco_test,
        robot_state_pub,
        rviz, controller_spawner, 
        move_group_fairino, move_group_kdl,
        servo_node,
        retime_server_launch,
        obb_node,
    ])
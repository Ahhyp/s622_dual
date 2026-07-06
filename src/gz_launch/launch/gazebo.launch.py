# gazebo.launch.py
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



# def generate_launch_description():
#     # 获取当前包路径
#     this_package_path = get_package_share_directory('gz_launch')  
#     # robot_desc_path=get_package_share_directory('fairino_description')
#     robot_desc_path=get_package_share_directory('fairino_description')
#     robot_moveit_path=get_package_share_directory('fairino3_v6_moveit2_config')
    
    
#     # 启动 Gazebo
#     gazebo_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             get_package_share_directory('ros_gz_sim') + '/launch/gz_sim.launch.py']),
#         launch_arguments=[('gz_args', 'empty.sdf -r')]  # 使用自定义空世界
#     )
    
#     # 作用是将 Gazebo 仿真环境中的时钟信号 转换为 ROS 2 中标准的 /clock 话题，从而让所有 ROS 2 节点都能同步使用仿真时间
#     # arguments: 将 Gazebo 仿真世界中的时钟（gz.msgs.Clock）单向地转换为 ROS 2 的 rosgraph_msgs/msg/Clock 消息，
#     # 并发布到 ROS 话题 /world/empty/clock 上。
#     clock_bridge_node = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=['/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], # [ROS话题]@[ROS消息类型][Gazebo话题]
#         output='both',
#         parameters=[{'use_sim_time': True}],
#         remappings=[('/world/empty/clock', '/clock')]  
#     )

#     moveit_config = MoveItConfigsBuilder("fairino3_v6_robot", package_name="fairino3_v6_moveit2_config") \
#                     .robot_description(this_package_path + '/config/robot_gazebo.friction.urdf.xacro') \
#                     .robot_description_semantic('config/fairino3_v6_robot.srdf') \
#                     .robot_description_kinematics(robot_moveit_path + '/config/kinematics.yaml') \
#                     .planning_pipelines(pipelines=["ompl"],default_planning_pipeline="ompl") \
#                     .to_moveit_configs()
    
#     print(moveit_config)
    
#     fairino_planning_config = {}
    
#     # 添加机械臂
#     gz_urdf= moveit_config.robot_description['robot_description'].replace('package://fairino_description',robot_desc_path)

#     robot_to_gazebo_node = Node(
#         package='ros_gz_sim',
#         executable='create',
#         arguments=['-string', gz_urdf,
#             '-x','0.0','-y','0.0','-z','0.0',
#             '-R','0','-P','0','-Y','0',   # yaw=90°
#             '-name','robot_arm']
#     )
    
#     # 发布机械臂状态
#     robot_desc_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="both",
#         parameters=[moveit_config.robot_description,
#             {'use_sim_time': True},    #必须使用仿真时间
#             { "publish_frequency":100.0,},
#             ],
#     )
    
#     # 启动RViz
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         output="log",
#         arguments=["-d", LaunchConfiguration('rviz_config')],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             # moveit_config.planning_pipelines,
#             fairino_planning_config,
#             moveit_config.joint_limits,
#             {'use_sim_time': True},
#         ],
#     )

#     # 启动关节状态发布器， arm组控制器 （延迟等待Gazebo就绪）
#     ros2_controllers_yaml = os.path.join(
#         robot_moveit_path, 'config', 'ros2_controllers.yaml'
#     )
#     controller_spawner_node = TimerAction(
#         period=5.0,
#         actions=[
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=[
#                     "-p", ros2_controllers_yaml,
#                     "fairino3_controller",
#                     "joint_state_broadcaster",
#                 ],
#                 parameters=[{'use_sim_time': True}],
#                 output="screen",
#             )
#         ],
#     )

#     # 启动move_group
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#             fairino_planning_config,
#             {'use_sim_time': True},
#         ],
#     )

#     # realsense_launch = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([
#     #         PathJoinSubstitution([
#     #             FindPackageShare('realsense2_camera'),
#     #             'launch',
#     #             'rs_launch.py'
#     #         ])
#     #     ]),
#     #     launch_arguments={
#     #         'enable_color': 'true',
#     #         'enable_depth': 'true',
#     #         'depth_module.profile': '640x480x30',
#     #         'rgb_camera.profile': '640x480x30',
#     #         'pointcloud.enable': 'true',
#     #         # 'align_depth.enable': 'true',
#     #         # 'enable_sync': 'true',
#     #         'temporal_filter.enable': 'true',
#     #         'spatial_filter.enable': 'true',
#     #         # 'hole_filling_filter.enable': 'true',
#     #     }.items()
#     # )
    
#     return LaunchDescription([
#         rviz_config_arg,
#         gazebo_node,  # 启动Gazebo仿真环境
#         clock_bridge_node,  # 时钟桥接
#         robot_to_gazebo_node,#启动gazebo环境机械臂
#         robot_desc_node, #启动机械臂状态节点
#         rviz_node,  # 启动RViz
#         controller_spawner_node,#启动关节状态发布器（延迟5s等待Gazebo就绪）
#         move_group_node,  # 启动MoveIt的move_group
#         # servo_node,   # ✅ 新增
#         # realsense_launch,
#         # hand_eye_tf_publisher
#     ])



def generate_launch_description():
    this_pkg = get_package_share_directory("gz_launch")
    robot_desc_pkg = get_package_share_directory("fairino_description")
    robot_moveit_pkg = get_package_share_directory("fairino3_v6_moveit2_config")

    # 1. Gazebo 空世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ros_gz_sim") + "/launch/gz_sim.launch.py"
        ]),
        launch_arguments=[("gz_args", "empty.sdf -r")]
    )

    # 2. 时钟桥接
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        remappings=[("/world/empty/clock", "/clock")]
    )

    # 3. MoveIt 配置（告诉 move_group 用哪个 URDF/SRDF/kinematics）
    # moveit_config = (
    #     MoveItConfigsBuilder("fairino_description", package_name="fairino3_v6_moveit2_config")
    #     .robot_description(this_pkg + "/config/robot_gazebo.urdf.xacro")
    #     .robot_description_semantic("config/fairino_description.srdf")
    #     .robot_description_kinematics(robot_moveit_pkg + "/config/kinematics.yaml")
    #     .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
    #     .to_moveit_configs()
    # )
    moveit_config = MoveItConfigsBuilder("fairino3_v6_robot", package_name="fairino3_v6_moveit2_config") \
                    .robot_description(this_pkg + '/config/robot_gazebo.friction.urdf.xacro') \
                    .robot_description_semantic('config/fairino3_v6_robot.srdf') \
                    .robot_description_kinematics(robot_moveit_pkg + '/config/kinematics.yaml') \
                    .planning_pipelines(pipelines=["ompl"],default_planning_pipeline="ompl") \
                    .to_moveit_configs()
    
    # 4. Spawn 机械臂到 Gazebo
    gz_urdf = moveit_config.robot_description["robot_description"].replace(
        "package://fairino_description", robot_desc_pkg
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", gz_urdf,
                   "-x", "0.0", "-y", "0.0", "-z", "0.0",
                   "-R", "0", "-P", "0", "-Y", "0",
                   "-name", "robot_arm"]
    )

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
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["-p", ros2_controllers_yaml,
                           "fairino3_controller",
                           "joint_state_broadcaster"],
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ]
    )

    # 7. move_group
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        output="screen"
    )

    # 8. RViz
    rviz_config = os.path.join(this_pkg, "rviz", "gz_launch.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description,
                     moveit_config.robot_description_semantic,
                     {"use_sim_time": True}]
    )

    return LaunchDescription([
        gazebo, clock_bridge, spawn_robot, robot_state_pub,
        rviz, controller_spawner, move_group
    ])
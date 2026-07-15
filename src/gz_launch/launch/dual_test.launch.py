# dual.launch.py — V3 半集成验证：左臂 prefix + 新 YAML，不含 MoveIt
import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    this_pkg = get_package_share_directory("gz_launch")
    robot_moveit_pkg = get_package_share_directory("s622_moveit_config")

    set_model_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.path.join(this_pkg, "models")
        + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )

    # 1. Gazebo + 桌子世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ros_gz_sim") + "/launch/gz_sim.launch.py"
        ]),
        launch_arguments=[("gz_args", os.path.join(this_pkg, "worlds", "table_world.sdf") + " -r")]
    )

    # 2. 时钟桥接
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/world/table_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        remappings=[("/world/table_world/clock", "/clock")]
    )

    # 相机图像桥接
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ],
        remappings=[
            ("/camera/image", "/camera/color/image_raw"),
            ("/camera/depth_image", "/camera/depth/image_raw"),
            ("/camera/camera_info", "/camera/color/camera_info"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 3. xacro 展开 URDF（prefix=left_，不含 MoveIt）
    xacro_file = os.path.join(this_pkg, "config", "robot_gazebo.urdf.xacro")
    xacro_result = subprocess.run(
        ["xacro", xacro_file, "prefix:=left_"],
        capture_output=True, text=True,
        env={**os.environ})
    if xacro_result.returncode != 0:
        raise RuntimeError(f"xacro failed:\n{xacro_result.stderr}")
    gz_urdf = xacro_result.stdout
    robot_desc_pkg = get_package_share_directory("s622_moveit_descriptions")
    gz_urdf = gz_urdf.replace("package://s622_moveit_descriptions", robot_desc_pkg)
    robot_desc = {"robot_description": gz_urdf}

    # 4. Spawn 左臂
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", gz_urdf,
                   "-x", "0.0", "-y", "0.0", "-z", "0.0",
                   "-R", "0", "-P", "0", "-Y", "0",
                   "-name", "left_arm"]
    )

    # 5. spawn 方块
    spawn_box = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-world", "table_world",
                           "-file", os.path.join(this_pkg, "models", "target_box", "model.sdf"),
                           "-name", "target_box",
                           "-x", "0.36", "-y", "0.02", "-z", "0.10",
                           "-R", "0", "-P", "0", "-Y", "0.00"],
            )
        ]
    )

    # 6. robot_state_publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_desc, {"use_sim_time": True}]
    )

    # 7. controller spawner（左臂 + new YAML）
    dual_arm_controllers_yaml = os.path.join(
        robot_moveit_pkg, "config", "dual_arm_controllers.yaml"
    )
    controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["-p", dual_arm_controllers_yaml,
                           "left_arm_controller",
                           "joint_state_broadcaster",
                           "left_hand_controller"],
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ]
    )

    # 8. YOLO OBB
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
        robot_state_pub,
        controller_spawner,
        obb_node,
    ])

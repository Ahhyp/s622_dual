# C4.3 S4 W7 -- combined Gazebo launch for the end-to-end transport flow.
#
# This is the "W7-A wiring" launch: ONE launch that brings up
#   * the W5 world (table + support + sample),
#   * the W6 robot description (dual-arm xacro WITH the DetachableJoint plugin),
#   * gz_ros2_control + the same controller set as s622_dual_arm_qp.launch.py,
#   * the three weld topics (attach / detach / weld_state) bridged gz<->ros,
#   * the weld pose stream (Pose_V -> tf2_msgs/TFMessage) the snap measurement
#     needs, and
#   * the transport executor.
#
# It deliberately does NOT start MoveIt and does NOT claim the flow is validated:
# W7-B/W7-C evidence comes from what actually happens at runtime, not from this
# file existing.  Everything here is simulated (Gazebo) and MUST NOT be read as a
# real-hardware or real-time result.
#
#   ros2 launch gz_launch c43_s4_full_flow.launch.py                # headless
#   ros2 launch gz_launch c43_s4_full_flow.launch.py gui:=true      # with GUI
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_param_builder import load_xacro

import os
from pathlib import Path


def generate_launch_description():
    gz_share = get_package_share_directory("gz_launch")
    desc_share = get_package_share_directory("s622_moveit_descriptions")
    qp_share = get_package_share_directory("dual_arm_qp_controller")
    ex_share = get_package_share_directory("dual_arm_transport_executor")

    gui_arg = DeclareLaunchArgument("gui", default_value="false",
                                    description="Gazebo GUI (false = server only)")
    executor_arg = DeclareLaunchArgument(
        "executor", default_value="true",
        description="start the transport executor (needed for the weld guards)")
    world_name = "c43_s4_world"

    set_model_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.path.join(gz_share, "models") + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )

    world_file = os.path.join(gz_share, "worlds", f"{world_name}.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", [
            world_file + " -r -v 4",
            PythonExpression(["' -s' if '", LaunchConfiguration("gui"), "' == 'false' else ''"]),
        ])],
    )

    controllers_yaml = os.environ.get(
        "QP_CONTROLLERS_YAML",
        os.path.join(qp_share, "config", "dual_arm_qp_gazebo_controllers.yaml"))
    # The ONLY difference from the production profile: the description carries the
    # DetachableJoint plugin (W6).  Every other mapping is copied verbatim so the
    # kinematics and the controller set stay identical.
    urdf = load_xacro(
        Path(os.path.join(gz_share, "config", "c43_s4_dual_arm_gazebo.urdf.xacro")),
        mappings={
            "instantiate": "false",
            "include_camera_visual": "false",
            "include_global_camera": "false",
            "include_wrist_camera": "false",
            "finger_stop_gap": "0.0",
            "gripper_stop_block": "false",
            "gripper_force_plugins": "false",
            "arm_command_mode": "position",
            "arm_position_gain": os.environ.get("QP_ARM_POSITION_GAIN", "20.0"),
            "controllers_yaml": controllers_yaml,
        },
    ).replace("package://s622_moveit_descriptions", desc_share)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf}],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(package="ros_gz_sim", executable="create",
                      arguments=["-string", urdf, "-x", "0.0", "-y", "0.0", "-z", "0.0",
                                 "-R", "0", "-P", "0", "-Y", "0", "-name", "s622_dual_arm",
                                 # Spawn at the controller's standing configuration (home_q):
                                 # the QP refuses to start transport when the arms are not there,
                                 # and it does not drive the arms to home by itself.
                                 '-J', 'left_j1', '2.795036909814', '-J', 'left_j2', '-1.706950857914', '-J', 'left_j3', '1.279963524342', '-J', 'left_j4', '-1.143808993223', '-J', 'left_j5', '-1.570796326795', '-J', 'left_j6', '-0.346555743776', '-J', 'right_j1', '0.346555743776', '-J', 'right_j2', '-1.434641795676', '-J', 'right_j3', '-1.279963524342', '-J', 'right_j4', '-1.997783660367', '-J', 'right_j5', '1.570796326795', '-J', 'right_j6', '-2.795036909814'],
                      output="screen")],
    )

    def spawner(names, extra=None):
        return Node(package="controller_manager", executable="spawner",
                    arguments=names + (extra or []) + ["-c", "/controller_manager",
                                       "--controller-manager-timeout", "90"],
                    output="screen")

    spawn_controllers = TimerAction(
        period=12.0,
        actions=[
            spawner(["joint_state_broadcaster"]),
            spawner(["left_hand_controller", "right_hand_controller"]),
            TimerAction(period=3.0, actions=[
                # B-1: gz_ros2_control initialises the arms from
                # s622_moveit_config/config/initial_positions.yaml (0.5/-1.05/1.05/
                # -1.05/-0.8/0).  The controller compares the live joints against its
                # `home_q` and refuses start_transport when they differ, so for this
                # Gazebo profile the standing configuration *is* that spawn pose.
                # S2's default home_q is untouched elsewhere.
                spawner(["dual_arm_qp_controller"], extra=[
                    "--param-file",
                    os.path.join(gz_share, "config", "c43_s4_qp_home_override.yaml")]),
            ]),
        ],
    )

    # gz <-> ros plumbing.  Attach/detach are ROS->GZ because the plugin is a
    # subscriber only (W6-0: it does not advertise the topics).  weld_state is
    # GZ->ROS (StringMsg).  pose/info is bridged as TFMessage because that is the
    # only ROS type ros_gz_bridge maps Pose_V onto; the executor reads
    # child_frame_id == "<model>::<link>" from it.
    bridges = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"/world/{world_name}/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            f"/world/{world_name}/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/c43/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/c43/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/c43/weld_state@std_msgs/msg/String[ignition.msgs.StringMsg",
        ],
        output="screen",
        # The executor subscribes to weld_pose_topic (default /c43/weld_pose); without
        # this remap the bridged pose/info stream stays on the gz topic name and
        # onPoses() never fires, which turns every attach into attach_snap_unmeasured.
        remappings=[(f"/world/{world_name}/pose/info", "/c43/weld_pose")],
    )

    executor = TimerAction(
        period=16.0,
        actions=[Node(
            package="dual_arm_transport_executor",
            executable="c43_s4_executor",
            name="c43_s4_transport_executor",
            parameters=[
                os.path.join(ex_share, "config", "c43_s4_executor.yaml"),
                {
                    # Gazebo Fortress 6.18's pose/info -> TFMessage bridge emits
                    # UNscoped link names (left_wrist3_link, c43_sample); 6.16 was
                    # measured with scoped "model::link" names.  The executor
                    # matches these strings exactly, so the profile has to follow
                    # the bridge of the running version (T2 fix 2026-10-29).
                    "weld_parent_frame": "left_wrist3_link",
                    "weld_child_frame": "c43_sample",
                    "weld_pose_topic": os.environ.get("C43_WELD_POSE_TOPIC", "/c43/weld_pose"),
                    "artifacts_dir": os.environ.get("C43_W7_ARTIFACTS", "/tmp/c43_s4_w7"),
                    "run_id": os.environ.get("C43_W7_RUN_ID", "c43_s4_w7_run"),
                    # T2 fix: the executor needs the five-phase table from cycle 0
                    # (learning it from the live stream took a whole run, so the
                    # 5 s approach timeout always fired first).  These MUST match
                    # the start_transport request; the live phase index/name are
                    # still audited and a mismatch is a HOLD.
                    "phase_names": ["approach", "grasp", "lift", "transport", "lower", "release"],
                    "phase_durations": [3.0, 2.0, 3.0, 3.0, 3.0, 2.0],
                    "trajectory_duration_s": 16.0,
                    # S4-T16 (2026-10-29): close the LEFT fingers, then check the
                    # sample against the frozen grasp chain before attach.  The
                    # 3 mm / 1 deg tolerances are the plan's SUGGESTED values and
                    # remain [待标定]; they are profile defaults, not approved
                    # acceptance thresholds.  require_grasp_check=false restores
                    # the pre-S4-T16 behaviour.
                    "require_grasp_check": True,
                    "gripper_action": "/left_hand_controller/follow_joint_trajectory",
                    "gripper_joint_names": ["left_finger1_joint", "left_finger2_joint"],
                    "gripper_close_positions": [0.0251, -0.0251],
                    "gripper_motion_s": 1.0,
                    "gripper_position_tol": 0.002,
                    "gripper_timeout_s": 2.5,
                    "grasp_pos_tol_m": 0.003,
                    "grasp_rot_tol_rad": 0.017453292519943295,
                    "grasp_snapshot_max_age_s": 5.0,
                    # T2 fix (2026-10-29): the executor's stale/timeout checks must
                    # run on the same clock as the controller trajectory.  This
                    # container runs the 125 Hz controller at ~50 Hz wall, so a
                    # 10 Hz decimated status is 0.26-0.35 s WALL apart and tripped
                    # stale_state; in sim time it is 13 x 8 ms = 0.104 s.
                    "use_sim_time": True,
                },
            ],
            remappings=[("/clock", f"/world/{world_name}/clock")],
            output="screen",
        )],
        condition=IfCondition(LaunchConfiguration("executor")),
    )

    return LaunchDescription([
        gui_arg, executor_arg, set_model_path, gazebo, bridges,
        robot_state_publisher, spawn_robot, spawn_controllers, executor,
    ])

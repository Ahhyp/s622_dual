"""Isolated no-GUI C43 diagnostic; optional full dual-arm controller setup."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    out=Path(os.environ['C43_AB_OUT'])
    share=get_package_share_directory('gz_launch')
    robot=(out/'robot.urdf').read_text()
    def spawn(names,inactive=False):
        return Node(package='controller_manager',executable='spawner',
            arguments=names+['-c','/controller_manager','--controller-manager-timeout','60']+(['--inactive'] if inactive else []),output='screen')
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',share+'/models:'+os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('ros_gz_sim')+'/launch/gz_sim.launch.py'),
            launch_arguments={'gz_args':str(out/'world.sdf')+' -r -v 4'+('' if os.environ.get('C43_LAB_GUI')=='1' else ' -s')}.items()),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':robot}],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=[
            '/world/c43_s4_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/c43_s4_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
            remappings=[('/world/c43_s4_world/pose/info','/c43/weld_pose')],output='screen'),
        # The controller's heartbeat callback uses node ROS time, while update()
        # uses Gazebo time. Supply /clock in the isolated experiment domain.
        *([Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=[
            '/world/c43_s4_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            remappings=[('/world/c43_s4_world/clock','/clock')],output='screen')]
            if os.environ.get('C43_ARM_BACKEND')=='qp-executor' else []),
        TimerAction(period=5.,actions=[Node(package='ros_gz_sim',executable='create',
            arguments=['-file',str(out/'robot.sdf'),'-name','s622_dual_arm'],output='screen')]),
        TimerAction(period=10.,actions=[spawn(['joint_state_broadcaster']),spawn(['left_arm_controller'])]+
            ([] if os.environ.get('C43_LAB_MODE') in ('force','dual_transport') else [spawn(['left_hand_controller'])])),
        TimerAction(period=12.,actions=([spawn(['right_arm_controller'])]+([] if os.environ.get('C43_LAB_MODE')=='dual_transport' else [spawn(['right_hand_controller'])])
            if os.environ.get('C43_LAB_DUAL')=='1' else [])),
        TimerAction(period=14.,actions=([spawn(['dual_arm_qp_controller'],inactive=True)]
            if os.environ.get('C43_ARM_BACKEND','').startswith('qp-') else [])),
        TimerAction(period=16.,actions=([Node(package='dual_arm_transport_executor',executable='c43_s4_executor',
            parameters=[str(out/'executor.yaml')],remappings=[('/clock','/world/c43_s4_world/clock')],output='screen')]
            if os.environ.get('C43_ARM_BACKEND')=='qp-executor' else [])),
    ])

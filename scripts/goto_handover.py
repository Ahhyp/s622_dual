#!/usr/bin/env python3
"""双臂初始位姿：手掌面对面，4cm 间隙"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from s622_bt_manager.action import MoveToPose
import math


def make_pose(x, y, z, rx, ry, rz):
    """rx/ry/rz in radians, RPY order"""
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    # RPY → quaternion
    cr, sr = math.cos(rx/2), math.sin(rx/2)
    cp, sp = math.cos(ry/2), math.sin(ry/2)
    cy, sy = math.cos(rz/2), math.sin(rz/2)
    p.orientation.x = sr*cp*cy - cr*sp*sy
    p.orientation.y = cr*sp*cy + sr*cp*sy
    p.orientation.z = cr*cp*sy - sr*sp*cy
    p.orientation.w = cr*cp*cy + sr*sp*sy
    return p


def send_goal(node, action_name, frame, x, y, z, rx, ry, rz, label):
    client = ActionClient(node, MoveToPose, action_name)
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f'{label}: action server not available')
        return False

    goal = MoveToPose.Goal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.pose = make_pose(x, y, z, rx, ry, rz)
    goal.velocity_scale = 0.5
    goal.acceleration_scale = 0.5
    goal.timeout_sec = 30.0

    node.get_logger().info(f'sending {label}: ({x:.3f},{y:.3f},{z:.3f}) rpy=({rx:.2f},{ry:.2f},{rz:.2f})')
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error(f'{label}: rejected')
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    ok = result is not None and result.result.success
    node.get_logger().info(f'{label}: {"OK" if ok else "FAIL"}')
    return ok


def main():
    rclpy.init()
    node = Node('initial_handover_pose')

    # frame_id 被 move_to_pose_server 忽略，坐标始终是 base_link 系！
    # 左臂 base 在 world (0.35, 0, 0) 且 yaw=π:
    #   world (0.02, 0.2, 0.35) → left_base_link (0.33, -0.20, 0.35)
    #   Z 朝 -x(world) → Z 朝 +x(left_base_link) → R_y(+π/2)
    send_goal(node, '/left/move_to_pose', 'left_base_link',
              0.33, -0.20, 0.35,
              0.0, math.pi/2, 0.0, 'LEFT')

    # 右臂 base 在 world (-0.35, 0, 0) 且 yaw=0:
    #   world (-0.02, 0.2, 0.35) → right_base_link (0.33, 0.20, 0.35)
    #   Z 朝 +x(world) → Z 朝 +x(right_base_link) → R_y(+π/2)
    send_goal(node, '/right/move_to_pose', 'right_base_link',
              0.33, 0.20, 0.35,
              0.0, math.pi/2, 0.0, 'RIGHT')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

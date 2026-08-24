#!/usr/bin/env python3
"""C2 接口契约测试：验证 move_to_pose_server / gripper_service 改造后接口不变。

不启动 ROS 节点，只验证：
1. MoveToPose.action / SetGripper.srv 接口契约（字段、服务名）不变
2. 两个 server 的构造参数（MoveItMotion 依赖项）可解析
"""
import inspect
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def test_move_to_pose_action_interface_unchanged():
    """MoveToPose.action 契约不变（BT 依赖）。"""
    from s622_bt_manager.action import MoveToPose

    # goal 字段（get_fields_and_field_types() 返回 {name: type} dict）
    goal_fields = set(MoveToPose.Goal.get_fields_and_field_types().keys())
    # 期望：named_pose / target_pose / velocity_scale / acceleration_scale /
    #       ensure_servo_stopped / timeout_sec
    for expected in [
        "named_pose", "target_pose", "velocity_scale",
        "acceleration_scale", "ensure_servo_stopped", "timeout_sec",
    ]:
        assert expected in goal_fields, f"MoveToPose.Goal 缺字段: {expected}"
    # result 字段
    result_fields = set(MoveToPose.Result.get_fields_and_field_types().keys())
    assert "success" in result_fields
    assert "error_msg" in result_fields


def test_set_gripper_service_interface_unchanged():
    """SetGripper.srv 契约不变（BT 依赖）。"""
    from s622_bt_manager.srv import SetGripper

    req_fields = set(SetGripper.Request.get_fields_and_field_types().keys())
    assert "command" in req_fields
    res_fields = set(SetGripper.Response.get_fields_and_field_types().keys())
    for expected in ["success", "finger_position", "error_msg"]:
        assert expected in res_fields, f"SetGripper.Response 缺字段: {expected}"


def test_move_to_pose_server_uses_movemotion():
    """move_to_pose_server 构造不再依赖 MoveItPlanner。"""
    import s622_arm_actions.move_to_pose_server as mod

    src = inspect.getsource(mod)
    assert "MoveItPlanner" not in src.replace(
        "不再用自写 MoveItPlanner", ""
    ), "move_to_pose_server 仍引用 MoveItPlanner"
    assert "MoveItMotion" in src, "move_to_pose_server 未使用 MoveItMotion"
    assert "self.motion" in src


def test_gripper_service_uses_control_gripper():
    """gripper_service 改用 control_gripper。"""
    import s622_arm_actions.gripper_service as mod

    src = inspect.getsource(mod)
    assert "control_gripper" in src
    assert "JointTrajectoryPoint" not in src, "gripper_service 仍直接发轨迹"
    assert "moveit2_gripper" in src, "gripper_service 缺夹爪客户端"


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))

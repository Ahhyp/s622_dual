#!/usr/bin/env python3
"""[M2.7] 自动采集 20 组 eye-on-base 标定样本（关节空间版）。

原理：手眼标定只要求各组 grasp 姿态**足够多样**（旋转跨度≥20°、平移跨度≥40mm），
不要求精确等于引导值。本脚本：
  1. 读当前左右臂关节角作为 ROOT（第 1 组直接采集，不移动）。
  2. 第 2~20 组：在 ROOT 右臂关节角上叠加确定性腕部增量（j4/j5/j6 组合，
     覆盖 roll/pitch/yaw + 少量 j2/j3 位移），通过 /dual/move_to_joint_state 移动。
  3. 每组移动到位后调用 /manual_calibration_assistant/validate_latest 采集。

与助手的状态同步（重要）：
  - 助手是有状态工作流：不合格样本会挂起（blocked），必须先调用
    /manual_calibration_assistant/remove_latest 清除，下一次 validate 才放行；
  - 助手进程常驻，内存里可能已有已接受样本（accepted）。脚本每轮先查
    /manual_calibration_assistant/status：已采集的组直接跳过、挂起的失败样本先 Remove；
  - 若校验返回"与第 N 组重复"，视为该组位姿已在记录中：移除挂起样本后当作已采集，
    因此脚本可安全重跑（断点续采，重跑已采组不会重复入账）。

依赖：仿真 + 手动助手（global_eye_on_base_right.yaml）+ dual_move_server 运行中。

用法：
  ros2 run gz_launch auto_collect_eye_on_base.py \
      [--arm right|left] [--start 1] [--end 20] [--pause 2.0]
"""

import argparse
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from s622_bt_manager.action import DualMoveToJointState

# 每组右臂关节增量（度），索引 = [j1,j2,j3,j4,j5,j6]：
# 腕部 j4/j5/j6 主导姿态（roll/pitch/yaw），肩部 j2/j3 少量位移
_JOINT_DELTAS_DEG = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),     # 1 ROOT（不移动）
    (0.0, 0.0, 0.0, 0.0, 0.0, 20.0),    # 2 +Z roll
    (0.0, 0.0, 0.0, 0.0, 0.0, -20.0),   # 3 -Z roll
    (0.0, 0.0, 0.0, 15.0, 0.0, 0.0),    # 4 +X tilt
    (0.0, 0.0, 0.0, -15.0, 0.0, 0.0),   # 5 -X tilt
    (0.0, 0.0, 0.0, 0.0, 15.0, 0.0),    # 6 +Y tilt
    (0.0, 0.0, 0.0, 0.0, -15.0, 0.0),   # 7 -Y tilt
    (0.0, 0.0, 0.0, 28.0, 0.0, 0.0),    # 8 +X tilt big
    (0.0, 0.0, 0.0, -28.0, 0.0, 0.0),   # 9 -X tilt big
    (0.0, 0.0, 0.0, 0.0, 28.0, 0.0),    # 10 +Y tilt big
    (0.0, 0.0, 0.0, 0.0, -28.0, 0.0),   # 11 -Y tilt big
    (0.0, 0.0, 0.0, 20.0, 15.0, 0.0),   # 12 XY +/+
    (0.0, 0.0, 0.0, 20.0, -15.0, 0.0),  # 13 XY +/-
    (0.0, 0.0, 0.0, -20.0, 15.0, 0.0),  # 14 XY -/+
    (0.0, 0.0, 0.0, -20.0, -15.0, 0.0), # 15 XY -/-
    (0.0, 5.0, 0.0, 0.0, 18.0, 0.0),    # 16 位移+姿态
    (0.0, -5.0, 0.0, 0.0, -18.0, 0.0),  # 17
    (0.0, 0.0, 5.0, 18.0, 0.0, 0.0),    # 18
    (0.0, 0.0, -5.0, -18.0, 0.0, 0.0),  # 19
    (0.0, 5.0, -5.0, 15.0, 15.0, 0.0),  # 20 复合
]


class AutoCollect(Node):
    def __init__(self, arm, start, end, pause):
        super().__init__('auto_collect_eye_on_base')
        self.arm = arm
        self.start = start
        self.end = end
        self.pause = float(pause)
        ns = '/manual_calibration_assistant/'
        self.validate_service = ns + 'validate_latest'
        self.status_service = ns + 'status'
        self.remove_service = ns + 'remove_latest'
        self._last_validate_message = ''

        self._joint_lock = threading.Lock()
        self._joints = {}
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._dual = ActionClient(self, DualMoveToJointState, '/dual/move_to_joint_state')
        self._validate_client = self.create_client(Trigger, self.validate_service)
        self._status_client = self.create_client(Trigger, self.status_service)
        self._remove_client = self.create_client(Trigger, self.remove_service)

    def _on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        joints = {}
        for prefix in ('left', 'right'):
            try:
                joints[prefix] = [float(values[f'{prefix}_j{i}']) for i in range(1, 7)]
            except (KeyError, TypeError):
                return
        with self._joint_lock:
            self._joints = joints

    def _wait_ready(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._joint_lock:
                has_joints = len(self._joints) == 2
            if (has_joints
                    and self._dual.server_is_ready()
                    and self._validate_client.service_is_ready()
                    and self._status_client.service_is_ready()
                    and self._remove_client.service_is_ready()):
                return True
            time.sleep(0.1)
        return False

    def _call_trigger(self, client, what, timeout=30.0):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(f"{what} timeout")
            return None
        return future.result()

    def _assistant_status(self):
        resp = self._call_trigger(self._status_client, 'status')
        if resp is None:
            return None
        try:
            return json.loads(resp.message)
        except ValueError:
            self.get_logger().error(f"status parse failed: {resp.message[:120]}")
            return None

    def _remove_latest(self):
        resp = self._call_trigger(self._remove_client, 'remove_latest')
        if resp is None:
            return False
        self.get_logger().info(f"remove_latest: success={resp.success} | {resp.message[:80]}")
        return resp.success

    def _move_arm(self, target_deg):
        """把 self.arm 移动到 target（关节角弧度），另一臂保持当前位姿。

        [M2.7 左臂修复 2026-09-03] 原 _move_right 硬编码把 right_positions 设为
        target、left 保持当前——--arm left 时会把无标定板的右臂驱到左臂目标位姿
        （既移动错臂又遮挡相机视野，左板从未动过 → 全部 no markers detected）。
        """
        with self._joint_lock:
            left_now = list(self._joints.get('left', [0.0] * 6))
            right_now = list(self._joints.get('right', [0.0] * 6))
        goal = DualMoveToJointState.Goal()
        if self.arm == 'left':
            goal.left_positions = [float(v) for v in target_deg]
            goal.right_positions = right_now
        else:
            goal.left_positions = left_now
            goal.right_positions = [float(v) for v in target_deg]
        goal.velocity_scale = 0.3
        goal.acceleration_scale = 0.3
        goal.timeout_sec = 30.0
        goal.ensure_servo_stopped_left = True
        goal.ensure_servo_stopped_right = True
        future = self._dual.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error("dual goal not accepted")
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=40.0)
        if not result_future.done():
            self.get_logger().error("dual move timeout")
            return False
        result = result_future.result()
        if not result.result.success:
            self.get_logger().error(f"dual move failed: {result.result.error_msg}")
            return False
        self.get_logger().info("moved")
        return True

    def _validate(self):
        resp = self._call_trigger(self._validate_client, 'validate')
        if resp is None:
            self._last_validate_message = ''
            return False
        self._last_validate_message = resp.message
        self.get_logger().info(f"validate: success={resp.success} | {resp.message[:90]}")
        return resp.success

    def run(self):
        if not self._wait_ready():
            self.get_logger().error("joints / dual / assistant services not ready")
            return False
        with self._joint_lock:
            root = list(self._joints[self.arm])
        self.get_logger().info(f"ROOT joints ({self.arm}): {[round(v, 3) for v in root]}")

        status = self._assistant_status()
        if status is None:
            return False
        self.get_logger().info(
            f"assistant state: accepted={status.get('accepted', 0)}/"
            f"{status.get('target', len(_JOINT_DELTAS_DEG))} "
            f"blocked={status.get('blocked', False)}"
        )

        for i in range(self.start, self.end + 1):
            if not 1 <= i <= len(_JOINT_DELTAS_DEG):
                self.get_logger().warn(f"group {i} out of range 1..{len(_JOINT_DELTAS_DEG)}; skip")
                continue
            status = self._assistant_status()
            if status is None:
                return False
            accepted = int(status.get('accepted', 0))
            if i <= accepted:
                self.get_logger().info(f"group {i}/{len(_JOINT_DELTAS_DEG)} already collected; skip")
                continue
            if status.get('blocked'):
                if not self._remove_latest():
                    self.get_logger().warn("clear pending sample failed; abort run")
                    return False
                self.get_logger().info("pending rejected sample cleared")

            delta_deg = _JOINT_DELTAS_DEG[i - 1]
            self.get_logger().info(f"--- group {i}/{len(_JOINT_DELTAS_DEG)} delta={delta_deg} ---")
            if i > 1 or accepted >= 1:
                target = [root[k] + math.radians(delta_deg[k]) for k in range(6)]
                if not self._move_arm(target):
                    self.get_logger().warn(f"group {i} move failed; skipping")
                    continue
                time.sleep(1.0)
            ok = self._validate()
            if not ok:
                # 校验失败时助手已挂起该样本，必须 Remove 才能继续下一组
                self._remove_latest()
                if '重复' in self._last_validate_message:
                    self.get_logger().info(f"group {i} pose already in records; treated as collected")
                elif i == 1:
                    self.get_logger().warn(
                        "group 1 (ROOT) validate failed —— 助手无 root 且当前位姿不合格；"
                        "检查标定板是否清晰可见，或重启助手清空状态后重试"
                    )
                else:
                    self.get_logger().warn(f"group {i} validate failed; sample dropped")
            time.sleep(self.pause)

        self.get_logger().info("auto collection finished")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', default='right', choices=['left', 'right'])
    parser.add_argument('--start', type=int, default=1)
    parser.add_argument('--end', type=int, default=20)
    parser.add_argument('--pause', type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = AutoCollect(args.arm, args.start, args.end, args.pause)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

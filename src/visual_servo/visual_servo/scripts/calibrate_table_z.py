#!/usr/bin/env python3
"""
标定 table_z 的辅助脚本：MoveIt 规划夹爪垂直向下 → 纯 Z 慢降直到指尖接触桌面。

用法:
  1. 确保仿真在运行、visual_servo_node 已 pkill
  2. python3 calibrate_table_z.py [--x X] [--y Y]

     不带参数：在当前 EE 位置规划垂直向下姿态
     --x --y：  指定 base_link 系水平位置

  3. MoveIt 自动规划夹爪垂直向下姿态
  4. 执行规划
  5. 规划完成后自动进入慢速 Z 下降（5mm/s）
  6. 观察 Gazebo，指尖碰到桌面立即 Ctrl+C
  7. 脚本打印 grasp_frame.z 和 grasp_height_above_table
"""

import argparse
import signal
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger

from visual_servo.moveit_planner import MoveItPlanner

DESCEND_SPEED = 0.005          # Z 下降速度 m/s
PLAN_Z_ABOVE_CURRENT = 0.15    # 规划目标在 EE 当前位置上方 15cm（留下降空间）


class TableZCalibrator(Node):
    def __init__(self):
        super().__init__("calibrate_table_z",
                         parameter_overrides=[rclpy.Parameter("use_sim_time", value=True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.start_cli = self.create_client(Trigger, "/servo_node/start_servo")
        self.stop_cli = self.create_client(Trigger, "/servo_node/stop_servo")

        self.running = True
        self.start_time = time.time()

        self.planner = MoveItPlanner(
            node=self,
            joint_names=["j1", "j2", "j3", "j4", "j5", "j6"],
            base_link="base_link",
            end_effector="grasp_frame",
            group_name="robot_arm",
        )

    # ------------------------------------------------------------------
    def _send_twist(self, vz: float):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.z = vz
        self.twist_pub.publish(t)

    def _read_grasp_pose(self) -> Optional[tuple]:
        """返回 (x, y, z) 或 None"""
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", "grasp_frame",
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warning(
                f"TF lookup base_link→grasp_frame failed: {e}",
                throttle_duration_sec=2.0,
            )
            return None
        t = tf.transform.translation
        return (float(t.x), float(t.y), float(t.z))

    # ------------------------------------------------------------------
    def run(self, target_xy: Optional[tuple] = None):
        print("=" * 60)
        print("table_z 标定")
        print("=" * 60)
        print("")

        # 1. 读取当前 EE 位置
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
        ee = self._read_grasp_pose()
        if ee is None:
            print("❌ 无法读取 grasp_frame TF")
            print("   请确认:")
            print("   1. 仿真已启动 (ros2 launch gz_launch s622_gazebo.launch.py)")
            print("   2. ros2_control / robot_state_publisher 在运行")
            print("")
            print("   诊断: 列出当前所有 TF frame...")
            try:
                frames = self.tf_buffer.all_frames_as_string()
                print(f"   已缓存的 frame:\n{frames[:800]}")
            except Exception:
                print("   (无法获取 frame 列表)")
            return
        ee_x, ee_y, ee_z = ee
        print(f"当前 EE 位置: x={ee_x:.3f} y={ee_y:.3f} z={ee_z:.3f}")

        # 目标水平位置
        tx = float(target_xy[0]) if target_xy else ee_x
        ty = float(target_xy[1]) if target_xy else ee_y
        # 目标 z: 在当前 EE 位置上方，保证不撞桌子
        tz = ee_z + PLAN_Z_ABOVE_CURRENT

        print(f"MoveIt 规划目标: x={tx:.3f} y={ty:.3f} z={tz:.3f}  姿态: 夹爪垂直向下")
        print("")

        # 2. MoveIt 规划
        import numpy as np
        target_pos = np.array([tx, ty, tz])
        self.get_logger().info("MoveIt 规划中...")
        ok = self.planner.plan_to_pregrasp(target_pos, yaw=0.0)
        if not ok:
            print("❌ MoveIt 规划失败")
            return

        # 等 arm 稳定
        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        ee2 = self._read_grasp_pose()
        if ee2 is not None:
            print(f"规划后 EE: x={ee2[0]:.3f} y={ee2[1]:.3f} z={ee2[2]*1000:.1f}mm")
        print("")

        # 3. 手动慢降
        print(f"下降速度: {DESCEND_SPEED*1000:.0f} mm/s  方向: base Z 负")
        print("⚠️  指尖碰到桌面立即 Ctrl+C！")
        print("")

        if not self.start_cli.wait_for_service(timeout_sec=2.0):
            print("❌ /servo_node/start_servo 不可用")
            return
        self.start_cli.call_async(Trigger.Request())
        time.sleep(0.3)

        def handler(sig, frame):
            self.running = False
        signal.signal(signal.SIGINT, handler)

        try:
            while self.running:
                self._send_twist(-DESCEND_SPEED)
                rclpy.spin_once(self, timeout_sec=0.02)
        except KeyboardInterrupt:
            pass

        # 4. 停止
        self._send_twist(0.0)
        if self.stop_cli.wait_for_service(timeout_sec=1.0):
            future = self.stop_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        rclpy.spin_once(self, timeout_sec=0.3)

        # 5. 输出结果
        z1 = self._read_grasp_pose()
        elapsed = time.time() - self.start_time
        print("")
        print("=" * 60)
        print("结果")
        print("=" * 60)
        if z1 is not None:
            z_val = z1[2]
            print(f"接触时 grasp_frame.z = {z_val*1000:.2f} mm")
            print(f"")
            print(f"👉 启动 visual_servo_node 参数:")
            print(f"   table_z: 0.0")
            print(f"   grasp_height_above_table: {0.025 + z_val:.4f}")
            print(f"   pregrasp_height_above_table: 0.16")
            print(f"")
            print(f"   完整命令:")
            print(f"   -p z_strategy:=\"table\"")
            print(f"   -p table_z:=0.0")
            print(f"   -p grasp_height_above_table:={0.025 + z_val:.4f}")
            print(f"   -p pregrasp_height_above_table:=0.16")
        else:
            print("❌ 无法读取 grasp_frame TF")
        print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="标定 table_z")
    parser.add_argument("--x", type=float, help="目标 base x (默认: 当前 EE x)")
    parser.add_argument("--y", type=float, help="目标 base y (默认: 当前 EE y)")
    args = parser.parse_args()

    target_xy = None
    if args.x is not None and args.y is not None:
        target_xy = (args.x, args.y)

    rclpy.init()
    node = TableZCalibrator()
    try:
        node.run(target_xy=target_xy)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

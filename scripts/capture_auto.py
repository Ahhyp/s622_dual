#!/usr/bin/env python3
"""自动随机放方块 + 采图"""
import os, sys, random, subprocess, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def move_cube(x, y, yaw_deg):
    """用 ign service 把 target_box 挪到指定位置"""
    cmd = [
        'ign', 'service', '-s', '/world/dual_arm_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req',
        f'name: "target_box", '
        f'position: {{x: {x:.3f}, y: {y:.3f}, z: 0.03}}, '
        f'orientation: {{z: {__import__("math").sin(__import__("math").radians(yaw_deg)/2):.4f}, w: {__import__("math").cos(__import__("math").radians(yaw_deg)/2):.4f}}}'
    ]
    subprocess.run(cmd, capture_output=True, timeout=3)


class Capture(Node):
    def __init__(self, out_dir: str, max_count: int):
        super().__init__('capture')
        self.bridge = CvBridge()
        self.out_dir = out_dir
        self.max_count = max_count
        self.latest = None
        os.makedirs(out_dir, exist_ok=True)
        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw', lambda m: setattr(self, 'latest', m), 10)

    def run(self):
        count = 0
        self.get_logger().info(f'开始采集 {self.max_count} 张 → {self.out_dir}')
        while rclpy.ok() and count < self.max_count:
            # 每 5 帧换位置
            if count % 5 == 0:
                x = random.uniform(-0.25, 0.25)
                y = random.uniform(-0.20, 0.20)
                yaw = random.uniform(0, 360)
                self.get_logger().info(f'移动方块 → ({x:.2f}, {y:.2f}, yaw={yaw:.0f}°)')
                move_cube(x, y, yaw)
                time.sleep(2.0)  # 等方块落稳

            rclpy.spin_once(self, timeout_sec=0.5)
            if self.latest is None:
                continue

            path = os.path.join(self.out_dir, f'frame_{count:04d}.jpg')
            cv2.imwrite(path, self.bridge.imgmsg_to_cv2(self.latest, 'bgr8'))
            count += 1
            if count % 20 == 0:
                self.get_logger().info(f'{count}/{self.max_count}')

        self.get_logger().info('完成')


def main():
    rclpy.init()
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/train_data'
    max_count = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    Capture(out_dir, max_count).run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

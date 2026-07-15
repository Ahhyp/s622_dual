#!/usr/bin/env python3
"""从 /camera/color/image_raw 采集训练图片"""
import os
import sys
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageCapture(Node):
    def __init__(self, out_dir: str, max_count: int):
        super().__init__('image_capture')
        self.bridge = CvBridge()
        self.out_dir = out_dir
        self.max_count = max_count
        self.count = 0
        os.makedirs(out_dir, exist_ok=True)

        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.cb, 10)

        self.get_logger().info(
            f'等待图片... 保存到 {out_dir}, 最多 {max_count} 张')

    def cb(self, msg: Image):
        path = os.path.join(self.out_dir, f'frame_{self.count:04d}.jpg')
        cv2.imwrite(path, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        self.count += 1
        print(f'[{self.count}/{self.max_count}] {path}')
        if self.count >= self.max_count:
            self.get_logger().info('采集完成')
            rclpy.shutdown()
            raise SystemExit


def main():
    rclpy.init()
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/train_data'
    max_count = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    node = ImageCapture(out_dir, max_count)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == '__main__':
    main()

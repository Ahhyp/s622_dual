import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/camera/color/image_raw"

rclpy.init()
node = Node("image_probe")
got = []


def cb(msg):
    a = np.frombuffer(msg.data, dtype=np.uint8)
    print(
        "size=%dx%d enc=%s mean=%.1f max=%d nonzero_ratio=%.3f"
        % (msg.width, msg.height, msg.encoding, a.mean(), a.max(), (a > 10).mean()),
        flush=True,
    )
    got.append(1)
    rclpy.shutdown()


node.create_subscription(Image, TOPIC, cb, 10)
try:
    rclpy.spin(node)
except Exception:
    pass
if not got:
    print("NO MESSAGE", flush=True)

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SaveOneImage(Node):
    def __init__(self):
        super().__init__("save_one_image")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.cb,
            10,
        )

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        path = "./tmp/gazebo_camera_frame.jpg"
        cv2.imwrite(path, img)
        self.get_logger().info(f"saved {path}")
        rclpy.shutdown()

rclpy.init()
node = SaveOneImage()
rclpy.spin(node)

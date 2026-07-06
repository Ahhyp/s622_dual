#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class DemoPathPlanningNode(Node):
    def __init__(self):
        super().__init__("demo_pathplanning_node")
        self.get_logger().info("Demo path planning node started")

        # TODO:
        # 1. create MoveIt2 interface
        # 2. set start state
        # 3. set target pose
        # 4. plan
        # 5. execute


def main():
    rclpy.init()
    node = DemoPathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
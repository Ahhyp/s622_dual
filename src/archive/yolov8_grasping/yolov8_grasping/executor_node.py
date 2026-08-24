#!/usr/bin/env python3
# executor_node.py
"""抓取执行节点：订阅 /grasp_pose，调 MoveIt2 完成抓取。"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from yolov8_grasping.arm_executor import ArmExecutor


class GraspExecutorNode(Node):
    def __init__(self):
        super().__init__("grasp_executor_node")

        # ---------- 参数 ----------
        self.declare_parameter(
            "joint_names",
            ["j1", "j2", "j3", "j4", "j5", "j6"],   # 按 SRDF 改
        )
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("end_effector", "grasp_frame")
        self.declare_parameter("group_name", "robot_arm")
        self.declare_parameter("pregrasp_offset_z", 0.10)
        self.declare_parameter("auto_execute", False)

        joint_names = self.get_parameter("joint_names").value
        base_link = self.get_parameter("base_link").value
        end_effector = self.get_parameter("end_effector").value
        group_name = self.get_parameter("group_name").value
        self.pregrasp_offset_z = self.get_parameter("pregrasp_offset_z").value
        self.auto_execute = self.get_parameter("auto_execute").value

        # ---------- 执行器 ----------
        self.arm = ArmExecutor(
            node=self,
            joint_names=joint_names,
            base_link=base_link,
            end_effector=end_effector,
            group_name=group_name,
        )

        # ---------- 订阅 ----------
        self.busy = False
        self.latest_pose: Optional[PoseStamped] = None

        self.create_subscription(
            PoseStamped, "/grasp_pose", self.cb_grasp_pose, 10)
        self.create_subscription(
            Bool, "/grasp_trigger", self.cb_trigger, 10)

        self.get_logger().info(
            f"Grasp executor ready, auto_execute={self.auto_execute}, "
            f"group={group_name}, ee={end_effector}"
        )
        


    # ---------- 回调 ----------
    def cb_grasp_pose(self, msg: PoseStamped):
        self.latest_pose = msg
        if self.auto_execute and not self.busy:
            self.execute_grasp(msg)

    def cb_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self.busy:
            self.get_logger().warning("busy, ignore trigger")
            return
        if self.latest_pose is None:
            self.get_logger().warning("no grasp pose yet, ignore trigger")
            return
        self.execute_grasp(self.latest_pose)

    # ---------- 抓取流程 ----------
    def execute_grasp(self, pose_stamped: PoseStamped):
        self.busy = True
        try:
            target = pose_stamped.pose

            # 提取 yaw。
            # grasping_node 发的姿态是 (roll=π, pitch=0, yaw=yaw_obb)，
            # euler_from_quaternion 默认 'sxyz' 静态轴顺序，能正确还原 yaw。
            _, _, yaw = euler_from_quaternion([
                target.orientation.x, target.orientation.y,
                target.orientation.z, target.orientation.w,
            ])

            # 重建姿态时**必须用同一种约定**：roll=π, pitch=0, yaw=yaw。
            # 注意：(0, -π, yaw) 看起来也是"夹爪朝下"，但和 (π, 0, yaw)
            # 差一个绕 z 的 180°，会让 MoveIt 解出完全不同的关节解，
            # 同时跟 grasping_node 的约定不一致，禁用。
            qx, qy, qz, qw = quaternion_from_euler(math.pi, 0.0, yaw)

            pregrasp = Pose()
            pregrasp.position.x = target.position.x
            pregrasp.position.y = target.position.y
            pregrasp.position.z = target.position.z + self.pregrasp_offset_z
            pregrasp.orientation.x = qx
            pregrasp.orientation.y = qy
            pregrasp.orientation.z = qz
            pregrasp.orientation.w = qw

            grasp = Pose()
            grasp.position.x = target.position.x
            grasp.position.y = target.position.y
            grasp.position.z = target.position.z
            grasp.orientation = pregrasp.orientation

            # 1. 张开夹爪 + 关节空间到 pregrasp
            self.get_logger().info("[1/5] open gripper + move to pregrasp")
            self.arm.open_gripper()
            if not self.arm.move_to_pose(pregrasp, cartesian=False):
                self.get_logger().error("pregrasp planning failed")
                return

            # 2. 笛卡尔直线下降
            self.get_logger().info("[2/5] cartesian descent")
            if not self.arm.move_to_pose(grasp, cartesian=True):
                self.get_logger().error("descent failed")
                return

            # 3. 关闭夹爪
            self.get_logger().info("[3/5] close gripper")
            self.arm.close_gripper()

            # 4. 笛卡尔直线上升
            self.get_logger().info("[4/5] cartesian ascent")
            if not self.arm.move_to_pose(pregrasp, cartesian=True):
                self.get_logger().error("ascent failed")
                return

            # 5. 完成
            self.get_logger().info("[5/5] grasp done")
        finally:
            self.busy = False
            
        '''
        如果 input quat 和 output quat 几乎一样，说明 grasping_node 和 executor_node 用的是同一种约定，
        "取出 yaw 再重建"是恒等操作。如果差很远，说明两边约定不一致，得回去检查 grasping_node.py 的 grasp_quat_top_down。
        '''
        self.get_logger().info(
            f"target yaw={yaw:.3f}, "
            f"input quat=({target.orientation.x:.3f}, {target.orientation.y:.3f}, "
            f"{target.orientation.z:.3f}, {target.orientation.w:.3f}), "
            f"output quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
        )

def main():
    rclpy.init()
    node = GraspExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
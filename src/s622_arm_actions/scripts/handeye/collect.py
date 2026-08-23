#!/usr/bin/env python3
"""手眼标定采集: pymoveit2（IK 由 move_group 的 FairinoIKPlugin 内部完成）"""
import os
import sys
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo, JointState
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
import tf_transformations as tft

from pymoveit2 import MoveIt2

from poses import generate_poses
from aruco_utils import make_aruco_detector, solve_marker_pose

MARKER_ID   = 1
MARKER_SIZE = 0.032  # 经验校准值: PNG 白边 + gz-sim 渲染偏差, 实际可检测黑边 ≈3.2cm
EE_FRAME    = 'grasp_frame'
BASE_FRAME  = 'base_link'
SETTLE_SEC  = 1.2

# ==== 需要按你的 MoveIt 配置填 ====
JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
GROUP_NAME  = 'robot_arm'
# ==================================


def tf_msg_to_matrix(m):
    t = m.transform.translation
    q = m.transform.rotation
    M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    M[:3, 3] = [t.x, t.y, t.z]
    return M


class Collector(Node):
    def __init__(self):
        super().__init__('handeye_collector')
        self.cb = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.last_image = None    # (msg, cv_img)
        self.detect = make_aruco_detector()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._on_info, 10, callback_group=self.cb)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._on_image, 10, callback_group=self.cb)
        self._latest_joint_state = None
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joint_state, 10, callback_group=self.cb)

        # v3: 不再创建 IK service client——IK 由 move_group 的 FairinoIKPlugin 内部完成
        #（解析全解 + IKSelector 四维评分选解），pymoveit2 直接连 move_group

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_FRAME,
            end_effector_name=EE_FRAME,
            group_name=GROUP_NAME,
            callback_group=self.cb,
            use_move_group_action=True,
            # 2026-08-23 架构迁移：move_group 服务保持 namespaced，客户端显式连接
            move_group_namespace="/move_group_fairino",
        )
        self.moveit2.planner_id = 'RRTConnectkConfigDefault'
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        self.get_logger().info('collector ready')

    def _on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d) if len(msg.d) else np.zeros(5)
            self.get_logger().info(f'camera K ready fx={self.K[0,0]:.1f}')

    def _on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.last_image = (msg, img)
        except Exception as e:
            self.get_logger().warn(f'cvbridge: {e}')

    def _on_joint_state(self, msg):
        self._latest_joint_state = msg

    def _get_current_joints(self):
        if self._latest_joint_state is None:
            return None
        name_to_pos = dict(zip(self._latest_joint_state.name,
                               self._latest_joint_state.position))
        try:
            return np.array([name_to_pos[n] for n in JOINT_NAMES])
        except KeyError:
            return None

    def goto(self, xyz, rpy):
        """移动机械臂到 xyz/rpy 位姿。

        v3: IK 由 move_group 的 FairinoIKPlugin 内部完成（解析全解 + IKSelector
            四维评分选解），不再自己调 /fairino/get_all_ik 和评分（service 已退役）。
        """
        q = tft.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        self.moveit2.move_to_pose(
            position=list(xyz), quat_xyzw=list(q),
            cartesian=False, frame_id=BASE_FRAME)
        return self.moveit2.wait_until_executed()

    def capture_one(self):
        if self.K is None or self.last_image is None:
            return None, 'camera not ready'
        img_msg, img = self.last_image
        stamp = img_msg.header.stamp
        try:
            tfm = self.tf_buffer.lookup_transform(
                BASE_FRAME, EE_FRAME, stamp,
                timeout=Duration(seconds=0.5))
        except Exception as e:
            return None, f'tf: {e}'
        T_ee_base = tf_msg_to_matrix(tfm)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)
        if ids is None or MARKER_ID not in ids.flatten():
            return None, 'marker not detected'
        idx = list(ids.flatten()).index(MARKER_ID)
        T_mc = solve_marker_pose(corners[idx], MARKER_SIZE, self.K, self.D)
        if T_mc is None:
            return None, 'pnp failed'

        # 计算 side_px 用作质量指标
        c = corners[idx].reshape(-1, 2)
        side_px = float(np.linalg.norm(c[0] - c[1]))
        return (T_ee_base, T_mc, img, side_px), None


def run(node: Collector):
    # 等 camera_info
    t0 = time.time()
    while node.K is None and time.time() - t0 < 5:
        time.sleep(0.1)
    assert node.K is not None, 'camera_info timeout'

    poses = generate_poses()
    out_dir = os.path.join(os.path.dirname(__file__), 'data')
    os.makedirs(out_dir, exist_ok=True)
    saved = []

    for i, (xyz, rpy) in enumerate(poses):
        node.get_logger().info(
            f'[{i+1}/{len(poses)}] xyz={xyz.round(3)} '
            f'rpy_deg={np.rad2deg(rpy).round(1)}')
        ok = node.goto(xyz, rpy)
        if not ok:
            node.get_logger().warn('  plan/exec failed, skip')
            continue

        time.sleep(SETTLE_SEC)

        res, err = node.capture_one()
        if res is None:
            node.get_logger().warn(f'  capture: {err}, skip')
            continue
        T_ee_base, T_mc, img, side_px = res
        if side_px < 20:
            node.get_logger().warn(f'  side_px={side_px:.1f} too small, skip')
            continue
        saved.append({'T_ee_base': T_ee_base, 'T_marker_cam': T_mc})
        cv2.imwrite(os.path.join(out_dir, f'img_{i:02d}.png'), img)
        node.get_logger().info(
            f'  OK t_mc={T_mc[:3,3].round(3)} side_px={side_px:.1f}')

    if not saved:
        node.get_logger().error('no samples collected!')
        return

    T_ee_base = np.stack([s['T_ee_base'] for s in saved])
    T_marker_cam = np.stack([s['T_marker_cam'] for s in saved])
    out_file = os.path.join(out_dir, 'handeye.npz')
    np.savez(out_file, T_ee_base=T_ee_base, T_marker_cam=T_marker_cam,
             K=node.K, D=node.D)
    node.get_logger().info(f'saved {len(saved)} samples -> {out_file}')


def main():
    rclpy.init()
    node = Collector()
    exec_ = MultiThreadedExecutor(num_threads=4)
    exec_.add_node(node)

    import threading
    spin_thread = threading.Thread(target=exec_.spin, daemon=True)
    spin_thread.start()

    try:
        run(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
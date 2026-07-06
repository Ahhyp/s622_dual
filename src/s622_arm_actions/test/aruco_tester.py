#!/usr/bin/env python3
"""阶段 0 验证: Gazebo 里能不能检测到 ArUco"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoTester(Node):
    def __init__(self):
        super().__init__('aruco_tester')
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.aruco_params = aruco.DetectorParameters_create()

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._on_info, 10)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, 10)

        self._log_counter = 0
        self.get_logger().info('aruco_tester ready, waiting for camera_info...')

    # def _on_info(self, msg):
    #     if self.K is None:
    #         self.K = np.array(msg.k).reshape(3, 3)
    #         # 强制修正为实际渲染用的 fx (从对比中反推得到)
    #         scale = 548.0 / self.K[0, 0]  # ≈ 0.785
    #         self.K[0, 0] *= scale  # fx
    #         self.K[1, 1] *= scale  # fy
    #         self.D = np.array(msg.d) if len(msg.d) else np.zeros(5)
    
    # def _on_info(self, msg):
    #     if self.K is None:
    #         # 从像素反推得到的真实渲染参数
    #         self.K = np.array([
    #             [548.0, 0.0,   488.0],
    #             [0.0,   548.0, 237.0],
    #             [0.0,   0.0,   1.0]
    #         ])
    #         self.D = np.zeros(5)
    #         # self.get_logger().info(
    #         #     f'camera ready: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
    #         #     f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}')

    def _on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d) if len(msg.d) else np.zeros(5)
            self.get_logger().info(
                f'camera ready: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
                f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}')

    def _on_image(self, msg):
        if self.K is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        # 每次都存一张,看看 detector 看到的是什么
        cv2.imwrite('/tmp/aruco_frame.png', img)
        cv2.imwrite('/tmp/aruco_gray.png', gray)
        
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, 0.04, self.K, self.D)
            self._log_counter += 1
            if self._log_counter % 10 == 0:
                for i, mid in enumerate(ids.flatten()):
                    t = tvecs[i].flatten()
                    c = corners[i][0]  # 4x2 pixel coords
                    side_px = np.linalg.norm(c[0] - c[1])
                    self.get_logger().info(
                        f'ID={mid}: pos_cam=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m '
                        f'corners=({c[0][0]:.0f},{c[0][1]:.0f}) ({c[1][0]:.0f},{c[1][1]:.0f}) '
                        f'({c[2][0]:.0f},{c[2][1]:.0f}) ({c[3][0]:.0f},{c[3][1]:.0f}) '
                        f'side_px={side_px:.1f}')
            # for i in range(len(ids)):
            #     cv2.drawFrameAxes(img, self.K, self.D, rvecs[i], tvecs[i], 0.03)
            # aruco.drawDetectedMarkers(img, corners, ids)
            for c in corners:
                # 四边形边框（绿色）
                pts = c.astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(img, [pts], True, (0, 255, 0), 2)
                # for j, pt in enumerate(c):
                #     px, py = int(pt[0][0]), int(pt[0][1])
                #     cv2.circle(img, (px, py), 6, (0, 0, 255), -1)
                #     cv2.circle(img, (px, py), 6, (0, 0, 0), 1)
                #     cv2.putText(img, str(j), (px + 8, py - 8),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            self._log_counter += 1
            if self._log_counter % 30 == 0:
                self.get_logger().warn('no marker detected')
        cv2.imshow('aruco_tester', img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    n = ArucoTester()
    try:
        rclpy.spin(n)
    finally:
        cv2.destroyAllWindows()
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
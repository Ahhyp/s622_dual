# pose_estimator.py
import numpy as np
from tf_transformations import quaternion_from_euler


class PoseEstimator:
    '''
        X = (u - c_x) \cdot Z / f_x , Y = (v - c_y) \cdot Z / f_y
    '''
    def __init__(self):
        self.fx = self.fy = self.cx = self.cy = None
    
    # 内参
    def set_intrinsics(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
    
    def pixel_to_camera(self, u, v, depth_img, window=5):
        """返回 (X, Y, Z) 米，相机光学系下；失败返回 None"""
        if self.fx is None: 
            return None
        h, w = depth_img.shape
        u, v = int(round(u)), int(round(v))
        if not (0 <= u < w and 0 <= v < h):
            return None

        # 取小窗口中位数，抗深度空洞
        k = window // 2
        patch = depth_img[max(0, v-k):v+k+1, max(0, u-k):u+k+1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        z_mm = float(np.median(valid))
        Z = z_mm / 1000.0  # mm → m

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return X, Y, Z
    
    import math


def grasp_quat_top_down(yaw_rad: float):
    """俯视抓取姿态四元数。
    约定：
      - 输入 yaw 是物体在图像平面里的旋转角（弧度），长边水平时为 0。
      - 输出夹爪 z 轴朝下（roll=π），绕 base z 轴旋转 yaw。
    返回 (qx, qy, qz, qw)
    """
    return quaternion_from_euler(math.pi, 0.0, yaw_rad)

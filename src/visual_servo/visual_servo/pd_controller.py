#!/usr/bin/env python3
"""3 维位置 PD 控制器，纯算法，没有任何 ROS 依赖。

状态：上一次的误差和时间，用来算微分项。
输入：当前误差向量 (np.ndarray, shape=(3,))，单位米；当前时间，单位秒。
输出：速度向量 (np.ndarray, shape=(3,))，单位 m/s，已限幅。
"""
import numpy as np

class PDController:
    def __init__(self, kp: float, kd: float, max_output: float):
        """
        Args:
            kp: 比例系数。误差 1m 时输出 kp m/s（限幅前）。
            kd: 微分系数。抑制超调。先调 kp 再加 kd。
            max_output: 速度模长上限 (m/s)，安全限幅。
        """
        self.kp = kp
        self.kd = kd
        self.max_output = max_output
        self.prev_error = None      # 上次误差，None 表示第一次
        self.prev_time = None       # 上次时间戳

    def reset(self):
        """状态机切换或长时间停摆后必须 reset，否则微分项会瞎跳。"""
        self.prev_error = None
        self.prev_time = None

    def compute(self, error: np.ndarray, now_sec: float) -> np.ndarray:
        """
        Args:
            error: shape=(3,) 误差向量，单位米。
            now_sec: 当前时间（秒），用来算 dt。
        Returns:
            速度向量 shape=(3,)，单位 m/s，已经做模长限幅。
        """
        # 第一次调用没有历史，微分项为 0，避免初始大跳变
        if self.prev_error is None or self.prev_time is None:
            d_error = np.zeros_like(error)
        else:
            dt = now_sec - self.prev_time
            if dt <= 1e-6:
                # 时间没走（重复调用），微分项为 0
                d_error = np.zeros_like(error)
            else:
                d_error = (error - self.prev_error) / dt

        # PD 主公式
        u = self.kp * error + self.kd * d_error

        # 模长限幅：保证方向不变，只缩比例
        norm = float(np.linalg.norm(u))
        if norm > self.max_output:
            u = u * (self.max_output / norm)

        # 更新历史，留给下一次
        self.prev_error = error.copy()
        self.prev_time = now_sec

        return u
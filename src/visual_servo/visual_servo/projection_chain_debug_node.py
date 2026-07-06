#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros2 run visual_servo projection_chain_debug_node --ros-args \
  -p true_target_base:="[0.45, 0.0, 0.02]" \
  -p target_plane_z:=0.02
  
  
ros2 run visual_servo projection_chain_debug_node --ros-args \
  -p true_target_base:="[0.29, 0.57, 0.15]" \
  -p target_plane_z:=0.02
  

projection_chain_debug_node.py

目的：
  专门检查这条链路，而不是检查 Planning/Approaching：

    Gazebo 真值 base 坐标
      → TF 到 camera_color_optical_frame
      → camera_info 投影成理论像素 u_gt, v_gt
      → 与 YOLO OBB 像素 u_det, v_det 对比
      → 分别用 depth 反投影回 base
      → 分别用 ray-plane 交点回 base

用它来判断：
  1) YOLO 像素是否偏；
  2) depth/color 是否对齐；
  3) camera_info 是否匹配；
  4) TF / optical frame 是否有问题；
  5) 是否应该临时改用 ray-plane 替代 center depth。

建议放置：
  ~/my_S622/src/visual_servo/visual_servo/projection_chain_debug_node.py
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

from yolov8_obb_msgs.msg import Yolov8Inference


class ProjectionChainDebugNode(Node):
    """YOLO 像素 / depth / camera_info / TF 坐标链路体检节点。"""

    def __init__(self) -> None:
        super().__init__("projection_chain_debug_node")

        # ----------------------------
        # Frames / topics
        # ----------------------------
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")

        self.declare_parameter("color_camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("depth_camera_info_topic", "")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("detection_topic", "/yolov8/obb_detections")

        # 这里填“物体真实中心”的 base_link 坐标。
        # 注意：
        # - static:false 方块落地后，target_box 中心大约是 spawn_x, spawn_y, 0.02
        # - static:true 且 spawn z=0.15 时，中心大约是 spawn_x, spawn_y, 0.17
        self.declare_parameter("true_target_base", [0.45, 0.0, 0.02])

        # ray-plane 用的目标平面 z。
        # 对落地方块，通常用 0.02；抓顶部可用 0.04；悬空 static:true 可用 0.17。
        self.declare_parameter("target_plane_z", 0.02)

        self.declare_parameter("print_rate_hz", 1.0)
        self.declare_parameter("patch_radius", 2)      # 5x5
        self.declare_parameter("large_patch_radius", 5)  # 11x11

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.true_target_base = np.array(
            list(self.get_parameter("true_target_base").value), dtype=float
        )
        self.target_plane_z = float(self.get_parameter("target_plane_z").value)
        self.patch_radius = int(self.get_parameter("patch_radius").value)
        self.large_patch_radius = int(self.get_parameter("large_patch_radius").value)

        print_rate_hz = float(self.get_parameter("print_rate_hz").value)
        if print_rate_hz <= 0.0:
            print_rate_hz = 1.0

        color_info_topic = str(self.get_parameter("color_camera_info_topic").value)
        depth_info_topic = str(self.get_parameter("depth_camera_info_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        detection_topic = str(self.get_parameter("detection_topic").value)

        # ----------------------------
        # State cache
        # ----------------------------
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.color_info: Optional[CameraInfo] = None
        self.depth_info: Optional[CameraInfo] = None
        self.depth_img: Optional[np.ndarray] = None
        self.latest_det: Optional[Yolov8Inference] = None

        # ----------------------------
        # Subscriptions
        # ----------------------------
        self.create_subscription(CameraInfo, color_info_topic, self.cb_color_info, 10)

        if depth_info_topic:
            self.create_subscription(CameraInfo, depth_info_topic, self.cb_depth_info, 10)

        self.create_subscription(Image, depth_topic, self.cb_depth, 10)
        self.create_subscription(Yolov8Inference, detection_topic, self.cb_det, 10)

        self.create_timer(1.0 / print_rate_hz, self.tick)

        self.get_logger().info(
            "projection_chain_debug_node started | "
            f"true_target_base=({self.true_target_base[0]:+.4f}, "
            f"{self.true_target_base[1]:+.4f}, {self.true_target_base[2]:+.4f}) | "
            f"plane_z={self.target_plane_z:+.4f} | "
            f"base_frame={self.base_frame} camera_frame={self.camera_frame}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def cb_color_info(self, msg: CameraInfo) -> None:
        self.color_info = msg

    def cb_depth_info(self, msg: CameraInfo) -> None:
        self.depth_info = msg

    def cb_depth(self, msg: Image) -> None:
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as exc:
            self.get_logger().warning(f"depth cv_bridge failed: {exc}")

    def cb_det(self, msg: Yolov8Inference) -> None:
        self.latest_det = msg

    # ------------------------------------------------------------------
    # Main debug loop
    # ------------------------------------------------------------------
    def tick(self) -> None:
        if self.color_info is None:
            self.get_logger().warning("waiting for /camera/color/camera_info ...")
            return

        if self.depth_img is None:
            self.get_logger().warning("waiting for depth image ...")
            return

        fx, fy, cx, cy = self._intrinsics(self.color_info)

        # 1) 真值 base 点 → camera 点 → 理论像素
        gt_cam = self._transform_np_point(
            self.true_target_base,
            source_frame=self.base_frame,
            target_frame=self.camera_frame,
        )
        if gt_cam is None:
            return

        if gt_cam[2] <= 0.0:
            self.get_logger().warning(
                f"true target is behind camera? gt_cam.z={gt_cam[2]:+.4f}"
            )
            return

        u_gt = fx * gt_cam[0] / gt_cam[2] + cx
        v_gt = fy * gt_cam[1] / gt_cam[2] + cy

        # 2) YOLO 当前检测像素
        det = self._best_detection()
        if det is None:
            self.get_logger().warning(
                "no YOLO detection yet or latest detection is empty",
                throttle_duration_sec=1.0,
            )
            return

        u_det = float(det.center_x)
        v_det = float(det.center_y)

        # 3) 分别用 depth 反投影
        gt_depth_result = self._uv_depth_to_base(u_gt, v_gt)
        det_depth_result = self._uv_depth_to_base(u_det, v_det)

        # 4) 分别用 ray-plane 求交
        gt_ray_base = self._uv_ray_plane_to_base(u_gt, v_gt, self.target_plane_z)
        det_ray_base = self._uv_ray_plane_to_base(u_det, v_det, self.target_plane_z)

        # 5) 打印 camera_info / depth_info 基本一致性
        self._print_sensor_summary(fx, fy, cx, cy)

        # 6) 打印核心对比
        du = u_det - u_gt
        dv = v_det - v_gt

        self.get_logger().info(
            "\n"
            "================ Projection Chain Debug ================\n"
            f"true base      : ({self.true_target_base[0]:+.4f}, "
            f"{self.true_target_base[1]:+.4f}, {self.true_target_base[2]:+.4f})\n"
            f"true -> camera : ({gt_cam[0]:+.4f}, {gt_cam[1]:+.4f}, {gt_cam[2]:+.4f})\n"
            f"projected uv   : u_gt={u_gt:.1f}, v_gt={v_gt:.1f}\n"
            f"YOLO uv        : u_det={u_det:.1f}, v_det={v_det:.1f}, "
            f"class={det.class_name}, conf={det.confidence:.3f}, "
            f"angle={det.angle:.3f}\n"
            f"pixel delta    : du={du:+.1f}px, dv={dv:+.1f}px, "
            f"|d|={math.hypot(du, dv):.1f}px\n"
            "--------------------------------------------------------"
        )

        self._print_reprojection_result("GT uv + depth", gt_depth_result)
        self._print_reprojection_result("YOLO uv + depth", det_depth_result)
        self._print_base_result("GT uv + ray-plane", gt_ray_base)
        self._print_base_result("YOLO uv + ray-plane", det_ray_base)

        self._print_diagnosis(
            pixel_error=math.hypot(du, dv),
            gt_depth_base=None if gt_depth_result is None else gt_depth_result[1],
            det_depth_base=None if det_depth_result is None else det_depth_result[1],
            gt_ray_base=gt_ray_base,
            det_ray_base=det_ray_base,
        )

    # ------------------------------------------------------------------
    # Computation helpers
    # ------------------------------------------------------------------
    def _intrinsics(self, info: CameraInfo) -> Tuple[float, float, float, float]:
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        return fx, fy, cx, cy

    def _best_detection(self):
        if self.latest_det is None or len(self.latest_det.results) == 0:
            return None
        return max(self.latest_det.results, key=lambda r: r.confidence)

    def _transform_np_point(
        self,
        xyz: Sequence[float],
        source_frame: str,
        target_frame: str,
    ) -> Optional[np.ndarray]:
        pt = PointStamped()
        pt.header.frame_id = source_frame
        # 调试节点优先用最新 TF，避免时间同步先把问题搞复杂。
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(xyz[0])
        pt.point.y = float(xyz[1])
        pt.point.z = float(xyz[2])

        try:
            out = self.tf_buffer.transform(
                pt, target_frame, timeout=Duration(seconds=0.2)
            )
        except Exception as exc:
            self.get_logger().warning(
                f"TF failed: {source_frame} -> {target_frame}: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

        return np.array([out.point.x, out.point.y, out.point.z], dtype=float)

    def _depth_raw_to_m(self, raw, dtype) -> Optional[float]:
        if raw is None:
            return None

        if not np.isfinite(raw):
            return None

        value = float(raw)
        if value <= 0.0:
            return None

        # 常见真实相机/ros_gz depth：uint16 是 mm，float32 通常是 m。
        if np.issubdtype(dtype, np.integer):
            value = value / 1000.0
        else:
            # 保险：float 深度如果大于 10，大概率仍是 mm。
            if value > 10.0:
                value = value / 1000.0

        if not np.isfinite(value) or value <= 0.0:
            return None

        return value

    def _sample_depth(self, u: float, v: float, radius: int) -> Optional[Tuple[float, float, float, float, int]]:
        """返回 center/mean/min/max/count，单位 m。"""
        if self.depth_img is None:
            return None

        h, w = self.depth_img.shape[:2]
        x = int(round(u))
        y = int(round(v))

        if x < 0 or x >= w or y < 0 or y >= h:
            return None

        center_m = self._depth_raw_to_m(self.depth_img[y, x], self.depth_img.dtype)
        if center_m is None:
            return None

        x0 = max(0, x - radius)
        x1 = min(w, x + radius + 1)
        y0 = max(0, y - radius)
        y1 = min(h, y + radius + 1)

        patch = self.depth_img[y0:y1, x0:x1]
        vals = []
        for raw in patch.reshape(-1):
            z = self._depth_raw_to_m(raw, self.depth_img.dtype)
            if z is not None:
                vals.append(z)

        if not vals:
            return None

        arr = np.array(vals, dtype=float)
        return (
            center_m,
            float(np.mean(arr)),
            float(np.min(arr)),
            float(np.max(arr)),
            int(arr.size),
        )

    def _uv_to_camera(self, u: float, v: float, z: float) -> np.ndarray:
        fx, fy, cx, cy = self._intrinsics(self.color_info)
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return np.array([x, y, z], dtype=float)

    def _uv_depth_to_base(self, u: float, v: float):
        small = self._sample_depth(u, v, self.patch_radius)
        large = self._sample_depth(u, v, self.large_patch_radius)
        if small is None:
            return None

        center_m, mean_m, min_m, max_m, count = small

        # 与 visual_servo_node 当前逻辑一致：中心像素 depth。
        xyz_cam_center = self._uv_to_camera(u, v, center_m)
        base_center = self._transform_np_point(
            xyz_cam_center,
            source_frame=self.camera_frame,
            target_frame=self.base_frame,
        )
        if base_center is None:
            return None

        # 额外给一个 5x5 mean depth 结果，用于判断单点 depth 是否敏感。
        xyz_cam_mean = self._uv_to_camera(u, v, mean_m)
        base_mean = self._transform_np_point(
            xyz_cam_mean,
            source_frame=self.camera_frame,
            target_frame=self.base_frame,
        )

        return {
            "uv": (float(u), float(v)),
            "small": small,
            "large": large,
            "xyz_cam_center": xyz_cam_center,
            "base_center": base_center,
            "xyz_cam_mean": xyz_cam_mean,
            "base_mean": base_mean,
        }, base_center

    def _uv_ray_plane_to_base(self, u: float, v: float, plane_z: float) -> Optional[np.ndarray]:
        """像素射线与 base_link 下 z=plane_z 平面的交点。"""
        fx, fy, cx, cy = self._intrinsics(self.color_info)

        # optical frame 中的相机原点和 z=1 处的像素射线点。
        origin_cam = np.array([0.0, 0.0, 0.0], dtype=float)
        ray_point_cam = np.array(
            [(float(u) - cx) / fx, (float(v) - cy) / fy, 1.0],
            dtype=float,
        )

        origin_base = self._transform_np_point(
            origin_cam,
            source_frame=self.camera_frame,
            target_frame=self.base_frame,
        )
        ray_point_base = self._transform_np_point(
            ray_point_cam,
            source_frame=self.camera_frame,
            target_frame=self.base_frame,
        )

        if origin_base is None or ray_point_base is None:
            return None

        ray_dir = ray_point_base - origin_base
        norm = np.linalg.norm(ray_dir)
        if norm < 1e-9:
            return None
        ray_dir = ray_dir / norm

        if abs(ray_dir[2]) < 1e-9:
            self.get_logger().warning("ray is parallel to target plane")
            return None

        t = (float(plane_z) - origin_base[2]) / ray_dir[2]
        if t <= 0.0:
            self.get_logger().warning(
                f"ray-plane intersection behind camera: t={t:.4f}"
            )
            return None

        return origin_base + t * ray_dir

    # ------------------------------------------------------------------
    # Printing helpers
    # ------------------------------------------------------------------
    def _print_sensor_summary(self, fx: float, fy: float, cx: float, cy: float) -> None:
        h, w = self.depth_img.shape[:2]
        msg = (
            f"sensor summary | color_info K: fx={fx:.2f}, fy={fy:.2f}, "
            f"cx={cx:.2f}, cy={cy:.2f} | depth image: {w}x{h}, "
            f"dtype={self.depth_img.dtype}"
        )

        if self.depth_info is not None:
            dfx, dfy, dcx, dcy = self._intrinsics(self.depth_info)
            msg += (
                f" | depth_info K: fx={dfx:.2f}, fy={dfy:.2f}, "
                f"cx={dcx:.2f}, cy={dcy:.2f}"
            )

            dk = abs(dfx - fx) + abs(dfy - fy) + abs(dcx - cx) + abs(dcy - cy)
            if dk > 1e-3:
                msg += " | WARNING: color/depth camera_info differ"

        self.get_logger().info(msg, throttle_duration_sec=2.0)

    def _print_reprojection_result(self, name: str, result) -> None:
        if result is None:
            self.get_logger().warning(f"{name}: unavailable")
            return

        data, base_center = result
        small = data["small"]
        large = data["large"]
        base_mean = data["base_mean"]
        xyz_cam_center = data["xyz_cam_center"]

        err_center = base_center - self.true_target_base

        line = (
            f"{name}: "
            f"depth center={small[0]:.4f}m | "
            f"5x5 mean/min/max={small[1]:.4f}/{small[2]:.4f}/{small[3]:.4f}m | "
        )

        if large is not None:
            line += (
                f"11x11 mean/min/max={large[1]:.4f}/{large[2]:.4f}/{large[3]:.4f}m | "
            )

        line += (
            f"cam_center=({xyz_cam_center[0]:+.4f}, {xyz_cam_center[1]:+.4f}, "
            f"{xyz_cam_center[2]:+.4f}) | "
            f"base_center=({base_center[0]:+.4f}, {base_center[1]:+.4f}, "
            f"{base_center[2]:+.4f}) | "
            f"err=({err_center[0]*1000:+.1f}, {err_center[1]*1000:+.1f}, "
            f"{err_center[2]*1000:+.1f})mm"
        )

        if base_mean is not None:
            err_mean = base_mean - self.true_target_base
            line += (
                f" | base_5x5mean=({base_mean[0]:+.4f}, {base_mean[1]:+.4f}, "
                f"{base_mean[2]:+.4f}) "
                f"err_mean=({err_mean[0]*1000:+.1f}, {err_mean[1]*1000:+.1f}, "
                f"{err_mean[2]*1000:+.1f})mm"
            )

        self.get_logger().info(line)

    def _print_base_result(self, name: str, base: Optional[np.ndarray]) -> None:
        if base is None:
            self.get_logger().warning(f"{name}: unavailable")
            return

        err = base - self.true_target_base
        self.get_logger().info(
            f"{name}: base=({base[0]:+.4f}, {base[1]:+.4f}, {base[2]:+.4f}) | "
            f"err=({err[0]*1000:+.1f}, {err[1]*1000:+.1f}, {err[2]*1000:+.1f})mm "
            f"| plane_z={self.target_plane_z:+.4f}"
        )

    def _print_diagnosis(
        self,
        pixel_error: float,
        gt_depth_base: Optional[np.ndarray],
        det_depth_base: Optional[np.ndarray],
        gt_ray_base: Optional[np.ndarray],
        det_ray_base: Optional[np.ndarray],
    ) -> None:
        notes = []

        if pixel_error > 15.0:
            notes.append(
                "YOLO uv 和真值投影 uv 差距较大：优先查 YOLO 坐标还原、debug_image、OBB中心定义。"
            )
        else:
            notes.append(
                "YOLO uv 和真值投影 uv 接近：YOLO 像素坐标大概率不是主因。"
            )

        if gt_depth_base is not None:
            e = gt_depth_base - self.true_target_base
            if abs(e[1]) > 0.03:
                notes.append(
                    "即使用真值投影 uv + depth，base_y 仍偏大：优先查 depth/color 对齐、camera_info、optical frame TF。"
                )
            else:
                notes.append(
                    "真值投影 uv + depth 的 base_y 较准：投影/TF 主链路基本可用。"
                )

        if gt_ray_base is not None:
            e = gt_ray_base - self.true_target_base
            if abs(e[0]) < 0.02 and abs(e[1]) < 0.02:
                notes.append(
                    "真值投影 uv + ray-plane 较准：TF 和 camera_info 基本可用，depth 取点/对齐嫌疑更大。"
                )

        if det_ray_base is not None and det_depth_base is not None:
            diff = det_depth_base - det_ray_base
            if np.linalg.norm(diff[:2]) > 0.03:
                notes.append(
                    "YOLO uv 的 depth 结果和 ray-plane 结果差距较大：center depth 很可能取到了错误表面或未对齐 depth。"
                )

        self.get_logger().info("diagnosis: " + " ".join(notes))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProjectionChainDebugNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



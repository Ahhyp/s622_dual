#!/usr/bin/env python3
"""
YOLOv8 OBB 真检测节点

功能：
1. 订阅 /camera/color/image_raw
2. 使用 ultralytics YOLOv8 OBB 模型推理
3. 将 OBB 结果转换成 Yolov8Inference
4. 发布到 /yolov8/obb_detections

注意：
- 本节点不使用 create_timer
- 来一帧相机图像，推理一次，发布一次
- 没有检测到目标时，发布空 results
- angle 直接使用 ultralytics OBB 的 rotation，不做角度转换
"""
from __future__ import annotations

from typing import Any

import numpy as np
import rclpy
import torch

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from yolov8_obb_msgs.msg import Yolov8Inference, InferenceResult

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class Yolov8ObbNode(Node):
    def __init__(self):
        super().__init__("yolov8_obb_node")
        
        
        
        # ============================================================
        # 1. 参数声明
        # ============================================================
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/yolov8/obb_detections")

        # 初期可以用官方 OBB 权重验证链路；
        # 后续正式抓取建议换成你自训的抓取物体 OBB 权重。
        self.declare_parameter("model_path", "yolov8-obb-gazebo.pt")

        # 置信度阈值，低于这个值的检测不发布。
        self.declare_parameter("confidence_threshold", 0.25)

        # auto：有 CUDA 用 GPU 0，否则用 CPU
        # 也可以手动传 device:=cpu 或 device:=0
        self.declare_parameter("device", "auto")

        # 推理输入尺寸。
        # CPU 慢时用 640；GPU 或精度要求高时可试 1024。
        self.declare_parameter("imgsz", 640)

        # 每帧最多保留多少个检测框。
        self.declare_parameter("max_det", 20)

        # 没检测到时是否发布空 results。
        # 推荐 True，避免下游继续误认为当前帧有目标。
        self.declare_parameter("publish_empty", True)

        # 如果相机消息 header.frame_id 为空，使用这个默认 frame。
        self.declare_parameter("fallback_frame_id", "camera_color_optical_frame")

        # 是否打印 ultralytics 自己的推理日志。
        self.declare_parameter("yolo_verbose", False)
        
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/yolov8/obb_debug_image")
        self.declare_parameter("print_detections", False)
        
        # ============================================================
        # 2. 读取参数
        # ============================================================
        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        
        
        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.device_param = self.get_parameter("device").value
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.publish_empty = bool(self.get_parameter("publish_empty").value)
        self.fallback_frame_id = self.get_parameter("fallback_frame_id").value
        self.yolo_verbose = bool(self.get_parameter("yolo_verbose").value)
        # 检测可视化所需参数
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.print_detections = bool(self.get_parameter("print_detections").value)

        
        # ============================================================
        # 3. 选择推理设备
        # ============================================================
        if self.device_param.lower() == "auto":
            self.device = "0" if torch.cuda.is_available() else "cpu"
        else:
            self.device = self.device_param
        
        # ============================================================
        # 4. 加载 YOLO 模型
        # ============================================================
        # 重点：模型只在 __init__ 加载一次，不能放在 image_callback 里。
        if YOLO is None:
            raise RuntimeError(
                "Cannot import ultralytics. "
                "请确认已经进入 conda env `yolov8`，并且安装了 ultralytics。"
            )

        self.get_logger().info(f"Loading YOLOv8 OBB model: {self.model_path}")
        self.get_logger().info(f"Using device: {self.device}")

        self.model = YOLO(self.model_path)
        
        # ============================================================
        # 5. ROS 通信
        # ============================================================
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(
            Yolov8Inference,
            self.detections_topic,
            10,
        )
        
        # 发布检测可视化的话题， 方便调试
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(
                Image,
                self.debug_image_topic,
                10,
            )
            self.get_logger().info(
                f"debug image enabled: {self.debug_image_topic}"
            )
        else:
            self.debug_pub = None
            
    
        self.frame_count = 0
        self.last_detection_count = 0

        self.get_logger().info(
            "YOLOv8 OBB node started | "
            f"image_topic={self.image_topic} | "
            f"detections_topic={self.detections_topic} | "
            f"conf={self.confidence_threshold} | "
            f"imgsz={self.imgsz} | "
            f"max_det={self.max_det} | "
            f"publish_empty={self.publish_empty}"
        )


    def image_callback(self, msg: Image):
        """
        收到一帧相机图像后，同步执行一次 YOLOv8 OBB 推理。

        流程：
        ROS Image
            -> cv_bridge 转 OpenCV BGR 图像
            -> YOLO predict
            -> 解析 result.obb
            -> 填 Yolov8Inference
            -> publish
        """

        self.frame_count += 1
        out = Yolov8Inference()
        out.header = msg.header
        
        # ------------------------------------------------------------
        # 1. ROS Image 转 OpenCV 图像
        # ------------------------------------------------------------
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        if cv_image is None or cv_image.size == 0:
            self.get_logger().warning("Received empty image, skip inference.")
            return

        # 保证图像是连续的 uint8 numpy 数组。
        # 这样可以减少 OpenCV / torch / ultralytics 的 stride 问题。
        if cv_image.dtype != np.uint8:
            cv_image = cv_image.astype(np.uint8)

        cv_image = np.ascontiguousarray(cv_image)

        # ------------------------------------------------------------
        # 2. YOLOv8 OBB 推理
        # ------------------------------------------------------------
        try:
            results = self.model.predict(
                source=cv_image,
                conf=self.confidence_threshold,
                imgsz=self.imgsz,
                device=self.device,
                max_det=self.max_det,
                verbose=self.yolo_verbose,
            )
            # ------------------------------------------------------------
            # 发布 YOLO 检测可视化图像
            # ------------------------------------------------------------
            if self.publish_debug_image and self.debug_pub is not None and results:
                try:
                    debug_img = results[0].plot()
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                    debug_msg.header = msg.header
                    self.debug_pub.publish(debug_msg)
                except Exception as e:
                    self.get_logger().warning(
                        f"failed to publish debug image: {e}",
                        throttle_duration_sec=2.0,
                    )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return
        
        # ------------------------------------------------------------
        # 3. 解析检测结果
        # ------------------------------------------------------------
        detections: list[InferenceResult] = []

        if results:
            detections = self.parse_yolo_obb_result(results[0])
            if self.print_detections:
                self.get_logger().info(
                    f"YOLO detections: {len(detections)}",
                    throttle_duration_sec=1.0,
                )

                for det in detections:
                    self.get_logger().info(
                        f"det | class={det.class_name} "
                        f"conf={det.confidence:.3f} "
                        f"cx={det.center_x:.1f} cy={det.center_y:.1f} "
                        f"w={det.width:.1f} h={det.height:.1f} "
                        f"angle={det.angle:.3f}",
                        throttle_duration_sec=1.0,
                    )
                    
        self.last_detection_count = len(detections)
        
        # ------------------------------------------------------------
        # 4. 发布 Yolov8Inference
        # ------------------------------------------------------------
        out = Yolov8Inference()

        # 使用原始图像时间戳，保证检测结果和图像同源。
        out.header.stamp = msg.header.stamp

        # frame_id 优先使用相机图像自带的 frame_id。
        # 如果为空，则使用默认 camera_color_optical_frame。
        if msg.header.frame_id:
            out.header.frame_id = msg.header.frame_id
        else:
            out.header.frame_id = self.fallback_frame_id

        for det in detections:
            out.results.append(det)

        # 没检测到目标时，发布空 results。
        # 你的 visual_servo_node 里 len(results)==0 会返回 None，因此这是安全的。
        if len(out.results) > 0 or self.publish_empty:
            self.pub.publish(out)

        # 降低日志频率，避免刷屏。
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"Processed {self.frame_count} frames, "
                f"last detections={self.last_detection_count}"
            )

    # ==================================================================
    # YOLO OBB 结果解析
    # ==================================================================
    def parse_yolo_obb_result(self, result: Any) -> list[InferenceResult]:
        """
        将 ultralytics 单帧 Results 转成 InferenceResult 列表。

        ultralytics OBB 关键字段：
        - result.obb.xywhr: [center_x, center_y, width, height, rotation]
        - result.obb.conf: confidence
        - result.obb.cls: class id

        本项目 InferenceResult 字段：
        - class_name
        - confidence
        - center_x
        - center_y
        - width
        - height
        - angle
        """

        detections: list[InferenceResult] = []

        obb = getattr(result, "obb", None)
        if obb is None:
            return detections

        try:
            if len(obb) == 0:
                return detections
        except TypeError:
            return detections

        try:
            xywhr = self.to_numpy(obb.xywhr)
            confs = self.to_numpy(obb.conf)
            class_ids = self.to_numpy(obb.cls)
        except Exception as e:
            self.get_logger().error(f"Failed to parse OBB result: {e}")
            return detections

        # result.names 通常是 {class_id: class_name}
        names = getattr(result, "names", None)
        if names is None:
            names = getattr(self.model, "names", {})

        for i in range(len(xywhr)):
            confidence = float(confs[i])

            # predict 已经用 conf 过滤过，这里再过滤一次作为保险。
            if confidence < self.confidence_threshold:
                continue

            class_id = int(class_ids[i])

            if isinstance(names, dict):
                class_name = str(names.get(class_id, class_id))
            else:
                try:
                    class_name = str(names[class_id])
                except Exception:
                    class_name = str(class_id)

            det = InferenceResult()
            det.class_name = class_name
            det.confidence = confidence

            # xywhr: center_x, center_y, width, height, rotation
            det.center_x = float(xywhr[i][0])
            det.center_y = float(xywhr[i][1])
            det.width = float(xywhr[i][2])
            det.height = float(xywhr[i][3])

            # 关键：直接使用 ultralytics 的 OBB rotation。
            # 按你项目约定：这里就是 rad，不做转换。
            det.angle = float(xywhr[i][4])

            detections.append(det)

        return detections

    @staticmethod
    def to_numpy(value: Any) -> np.ndarray:
        """
        将 torch.Tensor / numpy.ndarray / ultralytics tensor-like 转成 numpy.ndarray。
        """

        if isinstance(value, np.ndarray):
            return value

        if hasattr(value, "detach"):
            return value.detach().cpu().numpy()

        if hasattr(value, "cpu"):
            return value.cpu().numpy()

        return np.asarray(value)


def main(args=None):
    rclpy.init(args=args)

    node = Yolov8ObbNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# Phase 6.1：visual_servo 骨架

## 一、先把目标拆清楚

完整版（最终态）的伺服节点要做这些事：

```
检测(像素) → 深度 → 相机系 3D → TF → base_link 3D
                                        ↓
                           current_ee_in_base (TF 查)
                                        ↓
                                  error = target - ee
                                        ↓
                                       PD
                                        ↓
                              TwistStamped (base_link)
                                        ↓
                              /servo_node/delta_twist_cmds
```

**骨架版**先把这条管道建起来，但**不开马达**：[PD](./PD比例微分控制器.md) 输出真实算出来，但 publish 出去的 twist 强制为零，先看错误估计准不准、状态机切得对不对。等管道验证完再把"真发 twist"那个开关打开。


PD 简要概述
Proportional - Derivative，比例-微分控制器。PID 控制器的简化版（少了 I 积分项）。

直觉理解： 你在推一个购物车对准地上的标记。

项	干什么	类比
P（比例）	误差越大，纠正越猛	"偏了 10cm，用力推；偏了 1cm，轻轻推"
D（微分）	抑制震荡，刹车	"推太快了，减点速别冲过头"
代码就是两行：


# error = 目标位置 - 当前位置（像素差）
output = kp * error + kd * (error - last_error)
#        ↑ 比例项       ↑ 微分项（刹车）
不用 I（积分）是因为视觉伺服每 50ms 更新一次，积分容易累积过冲，反而抖。




## 二、状态机（先三个状态）

| 状态     | 含义             | 入口条件             | 退出条件                    |
| -------- | ---------------- | -------------------- | --------------------------- |
| IDLE     | 待机，发零 twist | 启动 / 上一次完成    | 收到 `/servo_trigger=true`  |
| SERVOING | 闭环跟踪         | 在 IDLE 收到 trigger | 误差 < tolerance            |
| DONE     | 完成，发零 twist | SERVOING 收敛        | 收到新 trigger 或外部 reset |

[twist](./twist.md)

后续 Phase 6.2/6.3 再加 DESCENDING、GRASPING、RETREATING。

## 三、建包

```bash
cd ~/my_S622/src
ros2 pkg create visual_servo \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs geometry_msgs std_msgs \
                 yolov8_obb_msgs cv_bridge tf2_ros tf2_geometry_msgs
```

## 四、目录结构

```
visual_servo/
├── visual_servo/
│   ├── __init__.py
│   ├── pd_controller.py        # 纯 PD，无 ROS 依赖
│   ├── error_estimator.py      # 像素+深度 → base_link 3D
│   └── visual_servo_node.py    # 状态机 + 控制循环
├── package.xml
└── setup.py
```

## 五、`pd_controller.py`

```python
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
```

## 六、`error_estimator.py`

```python
#!/usr/bin/env python3
"""把"像素 + 深度图 + 相机内参"反投影到相机系 3D 点。
跟 yolov8_grasping/pose_estimator.py 几乎一样，
独立放一份是为了 visual_servo 包不依赖 yolov8_grasping。
"""
import numpy as np


class ErrorEstimator:
    def __init__(self):
        self.fx = self.fy = self.cx = self.cy = None

    def set_intrinsics(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy

    def has_intrinsics(self) -> bool:
        return self.fx is not None

    def pixel_to_camera(self, u, v, depth_img, window=5):
        """像素 (u, v) → 相机光学系 (X, Y, Z) 米。失败返回 None。

        depth_img: numpy 数组，单位 mm（uint16）或 m（float32 自行换算）。
                   这里按 RealSense / Gazebo rgbd_camera 的常见 mm 处理。
        window: 取 (u,v) 周围 window×window 的中位数，抗深度空洞。
        """
        if self.fx is None:
            return None
        h, w = depth_img.shape
        u, v = int(round(u)), int(round(v))
        if not (0 <= u < w and 0 <= v < h):
            return None

        k = window // 2
        patch = depth_img[max(0, v - k):v + k + 1,
                          max(0, u - k):u + k + 1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        z_mm = float(np.median(valid))
        Z = z_mm / 1000.0

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return np.array([X, Y, Z], dtype=np.float64)
```

## 七、`visual_servo_node.py`（核心）

```python
#!/usr/bin/env python3
"""视觉伺服节点骨架（Phase 6.1）

状态机：IDLE → SERVOING → DONE
控制频率：固定 50Hz（独立 timer，不跟检测节奏走）

骨架版的关键安全设计：
  - 默认 enable_motion=False，PD 计算照常做、误差照常打印，
    但发布的 twist 强制清零，机械臂不会动。验证管道用。
  - 设 enable_motion=true 才真正驱动机械臂。

升级路径（后续阶段）：
  - 加 DESCENDING / GRASPING / RETREATING 三个状态
  - 加 yaw 闭环（角速度通道）
  - 加丢失目标超时回 IDLE
"""
import math
from enum import Enum
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped, PointStamped
from std_msgs.msg import Bool
from yolov8_obb_msgs.msg import Yolov8Inference

from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  注册 do_transform 实现

from visual_servo.pd_controller import PDController
from visual_servo.error_estimator import ErrorEstimator


# --------------------------------------------------------------------
# 状态枚举
# --------------------------------------------------------------------
class ServoState(Enum):
    IDLE = 0       # 待机，发零 twist
    SERVOING = 1   # 闭环跟踪
    DONE = 2       # 已收敛


# --------------------------------------------------------------------
# 节点
# --------------------------------------------------------------------
class VisualServoNode(Node):
    def __init__(self):
        super().__init__("visual_servo_node")

        # ---------- 参数 ----------
        # 控制循环频率，太低跟不上目标，太高 servo 节点处理不过来
        self.declare_parameter("control_rate", 50.0)
        # PD 增益，初始保守
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("kd", 0.05)
        # 速度限幅，单位 m/s。0.1 = 10cm/s 比较安全
        self.declare_parameter("max_linear_vel", 0.10)
        # 收敛阈值：误差模长 < 此值视为到达
        self.declare_parameter("position_tolerance", 0.005)   # 5mm
        # 期望停在目标上方多高（pre-grasp）
        self.declare_parameter("hover_offset_z", 0.10)
        # 坐标系
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("ee_frame", "tool0")           # 末端执行器
        # 安全开关：False 时 publish 全零 twist
        self.declare_parameter("enable_motion", False)

        self.control_rate = float(self.get_parameter("control_rate").value)
        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        max_v = float(self.get_parameter("max_linear_vel").value)
        self.tol = float(self.get_parameter("position_tolerance").value)
        self.hover_z = float(self.get_parameter("hover_offset_z").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.enable_motion = bool(self.get_parameter("enable_motion").value)

        # ---------- 模块 ----------
        self.pd = PDController(kp=kp, kd=kd, max_output=max_v)
        self.estimator = ErrorEstimator()
        self.bridge = CvBridge()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- 订阅缓存 ----------
        # 控制循环按固定频率读最新的，不在回调里直接算
        self.depth_img: Optional[np.ndarray] = None
        self.depth_stamp = None
        self.latest_det: Optional[Yolov8Inference] = None

        self.create_subscription(
            CameraInfo, "/camera/color/camera_info",
            self.cb_info, 10)
        self.create_subscription(
            Image, "/camera/depth/image_raw",
            self.cb_depth, 10)
        self.create_subscription(
            Yolov8Inference, "/yolov8/obb_detections",
            self.cb_det, 10)
        self.create_subscription(
            Bool, "/servo_trigger",
            self.cb_trigger, 10)

        # ---------- 发布 ----------
        # MoveIt Servo 默认订阅这个话题
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)

        # ---------- 状态机 ----------
        self.state = ServoState.IDLE

        # ---------- 控制循环 ----------
        # 用 timer 独立于检测频率推进，保证发布稳定
        period = 1.0 / self.control_rate
        self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f"visual_servo started | rate={self.control_rate}Hz | "
            f"kp={kp} kd={kd} max_v={max_v} | "
            f"motion_enabled={self.enable_motion}"
        )

    # ==================================================================
    # 回调：只缓存数据，不做计算
    # ==================================================================
    def cb_info(self, msg: CameraInfo):
        self.estimator.set_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_stamp = msg.header.stamp

    def cb_det(self, msg: Yolov8Inference):
        self.latest_det = msg

    def cb_trigger(self, msg: Bool):
        """外部触发：true 进入 SERVOING，false 强制回 IDLE"""
        if msg.data:
            if self.state != ServoState.SERVOING:
                self.get_logger().info(">>> trigger: enter SERVOING")
                self.pd.reset()
                self.state = ServoState.SERVOING
        else:
            self.get_logger().info(">>> trigger: back to IDLE")
            self.state = ServoState.IDLE

    # ==================================================================
    # 控制循环（每 1/rate 秒一次）
    # ==================================================================
    def control_loop(self):
        # 默认输出零 twist。即便没目标、状态非 SERVOING，
        # 也要持续往下发，让 MoveIt Servo 知道"还活着"，
        # 否则 servo 节点会自动 timeout 停止。
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame

        if self.state == ServoState.SERVOING:
            error = self._compute_error_in_base()

            if error is not None:
                err_norm = float(np.linalg.norm(error))

                # 收敛检查
                if err_norm < self.tol:
                    self.get_logger().info(
                        f"converged, |e|={err_norm*1000:.1f}mm → DONE"
                    )
                    self.state = ServoState.DONE
                else:
                    # PD 计算
                    now = self.get_clock().now().nanoseconds * 1e-9
                    v = self.pd.compute(error, now)

                    # 节流日志：每 10 帧打印一次（5Hz）
                    if int(now * self.control_rate) % 10 == 0:
                        self.get_logger().info(
                            f"|e|={err_norm*1000:.1f}mm  "
                            f"v=({v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.3f}) m/s  "
                            f"motion={'ON' if self.enable_motion else 'OFF'}"
                        )

                    # 安全开关：未启用就不写 twist，发零
                    if self.enable_motion:
                        twist.twist.linear.x = float(v[0])
                        twist.twist.linear.y = float(v[1])
                        twist.twist.linear.z = float(v[2])
            else:
                # 拿不到误差（深度无效 / TF 失败 / 没检测）
                # 暂时不切状态，下一帧重试。后续可加超时机制
                pass

        self.twist_pub.publish(twist)

    # ==================================================================
    # 误差计算：检测 → base_link 3D → 减去 EE 位置
    # ==================================================================
    def _compute_error_in_base(self) -> Optional[np.ndarray]:
        # 0. 数据齐全性检查
        if self.depth_img is None:
            return None
        if self.latest_det is None or len(self.latest_det.results) == 0:
            return None
        if not self.estimator.has_intrinsics():
            return None

        # 1. 选一个目标（先用最高置信度）
        target = max(self.latest_det.results, key=lambda r: r.confidence)

        # 2. 像素 → 相机系
        xyz_cam = self.estimator.pixel_to_camera(
            target.center_x, target.center_y, self.depth_img
        )
        if xyz_cam is None:
            return None

        # 3. 相机系 → base_link
        pt = PointStamped()
        pt.header.frame_id = self.camera_frame
        pt.header.stamp = self.depth_stamp
        pt.point.x, pt.point.y, pt.point.z = (
            float(xyz_cam[0]), float(xyz_cam[1]), float(xyz_cam[2])
        )
        try:
            pt_base = self.tf_buffer.transform(
                pt, self.base_frame,
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warning(f"TF target→base failed: {e}", throttle_duration_sec=2.0)
            return None

        target_in_base = np.array(
            [pt_base.point.x, pt_base.point.y, pt_base.point.z + self.hover_z]
        )

        # 4. 查 EE 当前位置
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warning(f"TF base→ee failed: {e}", throttle_duration_sec=2.0)
            return None

        ee_in_base = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ])

        # 5. 误差 = 目标 - 当前
        return target_in_base - ee_in_base


# --------------------------------------------------------------------
def main():
    rclpy.init()
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## 八、`setup.py` 入口

```python
entry_points={
    "console_scripts": [
        "visual_servo_node = visual_servo.visual_servo_node:main",
    ],
},
```

## 九、构建 + 跑通骨架

```bash
cd ~/my_S622
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
colcon build --merge-install --symlink-install --packages-select visual_servo
source install/setup.bash
```

### 验证步骤（按顺序）


```bash
# 1. 起 launch
ros2 launch gz_launch s622_gazebo.launch.py

# 2. 看 servo_node 起来了吗
ros2 node list | grep servo
ros2 topic list | grep servo
# 期望看到:
#   /servo_node/delta_twist_cmds
#   /servo_node/delta_joint_cmds
#   /servo_node/status

# 先将机械臂的状态弄到非奇异值(pos1)， 默认状态就是奇异值状态。

# 3. 激活
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

# 4. 手发一个 twist 试试机械臂会不会动（不通过 visual_servo_node）
ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.05}}}" -r 30
# 末端应该往 +x 方向慢慢移动 (2cm/s)
# 这一步通过之后

# 终端 1：仿真
ros2 launch gz_launch s622_gazebo.launch.py

# 终端 2：激活 servo
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

# 终端 3：obb 假检测
ros2 run yolov8_obb yolov8_obb_node

# 终端 4：visual_servo（先 enable_motion=false 验证）
ros2 run visual_servo visual_servo_node

# 终端 5：触发
ros2 topic pub --once /servo_trigger std_msgs/msg/Bool "{data: true}"

# 看 visual_servo 日志，误差合理，twist 全零（因为 motion=OFF）
# 直接在线改，不需要重启
ros2 param set /visual_servo_node enable_motion true

```

YOLO假检测 → visual_servo_node(PD+ErrorEstimator) → twist → MoveIt Servo → 机械臂动




## 十、几个埋下的坑提醒

* **MoveIt Servo 还没启动**：`/servo_node/delta_twist_cmds` 现在没人订阅，发布是没问题的，但要真伺服得起 `moveit_servo` 节点，那是 Phase 6.2 的事。
* **`ee_frame` 默认 `tool0`**：跟你 SRDF 末端 link 名要对上，错了 TF 查不到，日志会刷 `TF base→ee failed`。
* **目标会跟着检测频率"跳"**：因为 obb 现在发的是固定假框，跳不起来。等后面接真模型，目标会随检测有抖动，这是 PD 的 D 项要抑制的东西。
* **`hover_offset_z=0.10`**：现在伺服的目标是"物体上方 10cm"，不是物体本身。下一阶段加 DESCENDING 状态后，达到 hover 就切下降，这时把 offset 设 0。

## 十一、下一步

骨架跑通后，按这个顺序往里加：

1. **Phase 6.2**：起 `moveit_servo` 节点，把 `enable_motion=true`，让机械臂真的跟着假目标移动
2. **Phase 6.3**：加 DESCENDING / GRASPING / RETREATING 状态，做完整闭环抓取
3. **Phase 6.4**：加 yaw 通道（angular.z），让夹爪也对准 OBB 角度
4. **Phase 6.5**：加丢失目标超时、卡死保护

要不要我接着写 Phase 6.2 的 MoveIt Servo 配置？写之前你需要确认一下你的 `s622_moveit_config` 里有没有 `servo.yaml`，没有的话要从模板复制一份过来改。

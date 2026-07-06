可以。这个问题建议不要打补丁式修，而是把 `visual_servo_node` 里的“目标来源逻辑”重构清楚。

你现在的 bug 本质是：

```text
执行阶段仍然强依赖实时 YOLO 检测。
机械臂一遮挡目标，YOLO results 变空，servo 就没有 target，于是停住。
```

正确结构应该是：

```text
检测层：负责观察当前画面里的目标
目标管理层：负责决定当前执行时用哪个目标
控制层：只根据目标管理层给出的目标运动
```

对于你当前的静态红盒子，推荐策略是：

```text
trigger 时观察目标并锁定 target_locked
PLANNING / APPROACHING / DESCENDING / RETREATING 都使用 target_locked
YOLO 后续被遮挡不影响执行
```

---

# 我还需要哪些代码？

目前修这个 bug，**你之前贴的 `visual_servo_node.py` 已经够了**。

后面如果要继续把这些也一起整理，我再需要：

```text
1. visual_servo/error_estimator.py
   用于确认 pixel_to_camera 深度单位、内参计算是否完全正确

2. visual_servo/moveit_planner.py
   用于确认 plan_to_pregrasp 里是否也重新读取了目标，是否姿态/yaw 使用正确

3. 当前 launch 文件或启动命令
   用于确认参数默认值、enable_motion、avoid_collisions、enable_grasp_sequence 等配置
```

但这次“机械臂遮挡导致停止”的逻辑 bug，可以直接在 `visual_servo_node.py` 里修。

---

# 推荐修改方案

我建议把目标获取拆成两个函数：

```text
_observe_target_in_base()
    只负责从当前 YOLO + depth + TF 观察目标
    如果当前看不见，就返回 None

_get_execution_target_in_base()
    执行阶段使用
    如果已经锁定 target_locked，就优先返回 target_locked
    不因为 YOLO 当前看不见就返回 None
```

然后让 servo 控制使用：

```python
target = self._get_execution_target_in_base()
```

而不是每次都用：

```python
target = self._compute_target_in_base()
```

---

# 一、增加参数和缓存

在 `__init__` 里参数声明区加入：

```python
# ---------- 目标管理策略 ----------
# locked: trigger 后锁定目标，执行阶段一直用锁定目标。推荐当前静态红盒子使用。
# realtime: 每次都用实时检测。目标一遮挡就停，不推荐当前场景。
# hybrid: 可见时可小范围刷新，遮挡时用 locked。
self.declare_parameter("target_policy", "locked")

# 检测消息超过多久算过期
self.declare_parameter("det_timeout", 2.0)

# APPROACHING 收敛时是否尝试刷新 target_locked
# 当前遮挡问题下，建议默认 False，避免把有效 locked target 覆盖掉。
self.declare_parameter("refresh_target_at_approach_done", False)

# 如果允许刷新，刷新后的目标与旧目标距离不能超过这个阈值，单位 m
self.declare_parameter("target_refresh_max_jump", 0.05)
```

在读参数区域加入：

```python
self.target_policy = str(self.get_parameter("target_policy").value)
self.det_timeout = float(self.get_parameter("det_timeout").value)
self.refresh_target_at_approach_done = bool(
    self.get_parameter("refresh_target_at_approach_done").value)
self.target_refresh_max_jump = float(
    self.get_parameter("target_refresh_max_jump").value)
```

在缓存区域加入：

```python
self.latest_det_time: Optional[float] = None

# 最近一次“真实可见”的目标位置
self.last_visible_target: Optional[np.ndarray] = None
self.last_visible_target_time: Optional[float] = None

# target_locked 已经有了，这里补一个时间戳
self.target_locked_time: Optional[float] = None
```

---

# 二、修改 `cb_det`

把你现在的：

```python
def cb_det(self, msg: Yolov8Inference):
    self.latest_det = msg
```

替换成：

```python
def cb_det(self, msg: Yolov8Inference):
    """缓存 YOLO 检测消息。

    注意：
    - 这里不直接决定机械臂目标
    - 只记录最新检测是否为空
    - 真正执行阶段用什么目标，由 _get_execution_target_in_base() 决定
    """
    self.latest_det = msg
    self.latest_det_time = self._now_sec()

    if len(msg.results) == 0:
        self.get_logger().info(
            "det received: empty results",
            throttle_duration_sec=1.0,
        )
        return

    target = max(msg.results, key=lambda r: r.confidence)

    self.get_logger().info(
        f"det received | n={len(msg.results)} "
        f"best={target.class_name} "
        f"conf={target.confidence:.3f} "
        f"px=({target.center_x:.1f},{target.center_y:.1f}) "
        f"size=({target.width:.1f},{target.height:.1f}) "
        f"angle={target.angle:.3f} "
        f"frame={msg.header.frame_id}",
        throttle_duration_sec=1.0,
    )
```

---

# 三、把“实时观察目标”单独拆出来

你现在的 `_compute_target_in_base()` 本质上是“实时从 YOLO + depth 算目标”。

建议把它改名成 `_observe_target_in_base()`。

也就是新增这个函数。你可以把你当前 `_compute_target_in_base()` 的主体搬进去。

```python
def _observe_target_in_base(self) -> Optional[np.ndarray]:
    """从当前 YOLO 检测、深度图、相机内参和 TF 中实时观察目标。

    这个函数只表示“当前画面里是否看得见目标”。

    看得见：
        返回 base_link 下的目标位置
    看不见：
        返回 None

    注意：
        本函数不负责 target_locked 策略。
    """

    debug = list(self.get_parameter("debug_target").value)
    if any(abs(v) > 1e-6 for v in debug):
        target_debug = np.array(debug, dtype=float)
        self.get_logger().info(
            f"use debug_target in base_link: "
            f"({target_debug[0]:+.4f}, "
            f"{target_debug[1]:+.4f}, "
            f"{target_debug[2]:+.4f})",
            throttle_duration_sec=1.0,
        )
        return target_debug

    if self.depth_img is None:
        self.get_logger().warning(
            "observe target failed: no depth image yet",
            throttle_duration_sec=1.0,
        )
        return None

    if self.latest_det is None:
        self.get_logger().warning(
            "observe target failed: no detection msg yet",
            throttle_duration_sec=1.0,
        )
        return None

    if self.latest_det_time is not None:
        age = self._now_sec() - self.latest_det_time
        if age > self.det_timeout:
            self.get_logger().warning(
                f"observe target failed: detection timeout "
                f"age={age:.2f}s > {self.det_timeout:.2f}s",
                throttle_duration_sec=1.0,
            )
            return None

    if len(self.latest_det.results) == 0:
        self.get_logger().warning(
            "observe target failed: current detection has empty results",
            throttle_duration_sec=1.0,
        )
        return None

    if not self.estimator.has_intrinsics():
        self.get_logger().warning(
            "observe target failed: camera intrinsics not ready",
            throttle_duration_sec=1.0,
        )
        return None

    # 取最高置信度目标
    target = max(self.latest_det.results, key=lambda r: r.confidence)

    if getattr(self, "debug_print_target", False):
        self.get_logger().info(
            f"observe pixel | class={target.class_name} "
            f"conf={target.confidence:.3f} "
            f"u={target.center_x:.1f} v={target.center_y:.1f} "
            f"w={target.width:.1f} h={target.height:.1f} "
            f"angle={target.angle:.3f}",
            throttle_duration_sec=1.0,
        )

        if hasattr(self, "_debug_depth_at_pixel"):
            self._debug_depth_at_pixel(target.center_x, target.center_y)

    # 像素 + 深度 -> 相机坐标
    xyz_cam = self.estimator.pixel_to_camera(
        target.center_x,
        target.center_y,
        self.depth_img,
    )

    if xyz_cam is None:
        self.get_logger().warning(
            "observe target failed: pixel_to_camera returned None",
            throttle_duration_sec=1.0,
        )
        return None

    if getattr(self, "debug_print_target", False):
        self.get_logger().info(
            f"observe target in camera | "
            f"x={xyz_cam[0]:+.4f}, "
            f"y={xyz_cam[1]:+.4f}, "
            f"z={xyz_cam[2]:+.4f} "
            f"frame={self.camera_frame}",
            throttle_duration_sec=1.0,
        )

    pt = PointStamped()
    pt.header.frame_id = self.camera_frame

    # 调试阶段建议用最新 TF，避免时间同步问题。
    # 如果后续要严格同步，再改成 self.depth_stamp。
    pt.header.stamp = rclpy.time.Time().to_msg()

    pt.point.x = float(xyz_cam[0])
    pt.point.y = float(xyz_cam[1])
    pt.point.z = float(xyz_cam[2])

    try:
        pt_base = self.tf_buffer.transform(
            pt,
            self.base_frame,
            timeout=Duration(seconds=0.1),
        )
    except Exception as e:
        self.get_logger().warning(
            f"observe target failed: TF camera→base failed: {e}",
            throttle_duration_sec=1.0,
        )
        return None

    target_base = np.array([
        pt_base.point.x,
        pt_base.point.y,
        pt_base.point.z,
    ], dtype=float)

    self.last_visible_target = target_base.copy()
    self.last_visible_target_time = self._now_sec()

    if getattr(self, "debug_print_target", False):
        self.get_logger().info(
            f"observe target in base | "
            f"x={target_base[0]:+.4f}, "
            f"y={target_base[1]:+.4f}, "
            f"z={target_base[2]:+.4f}",
            throttle_duration_sec=1.0,
        )

    return target_base
```

---

# 四、新增“执行阶段目标选择函数”

这是这次修 bug 的核心。

```python
def _get_execution_target_in_base(self) -> Optional[np.ndarray]:
    """返回当前执行阶段应该使用的目标位置。

    target_policy:
    - locked:
        trigger 后锁定目标，执行阶段永远使用 target_locked。
        适合静态目标，能解决机械臂遮挡导致 YOLO 丢失的问题。

    - realtime:
        每次都用当前 YOLO 实时检测。
        遮挡时会停，适合后续测试动态目标，不适合当前红盒子。

    - hybrid:
        如果当前看得见目标，并且新目标离 locked target 不远，则刷新；
        如果看不见，则继续使用 locked target。
    """

    policy = self.target_policy

    # 非执行阶段，直接实时观察即可。
    # trigger 时也会走这里，此时 state 通常是 IDLE 或 DONE。
    if self.state not in (
        ServoState.PLANNING,
        ServoState.APPROACHING,
        ServoState.DESCENDING,
        ServoState.GRASPING,
        ServoState.RETREATING,
    ):
        return self._observe_target_in_base()

    # locked 策略：执行阶段只用锁定目标
    if policy == "locked":
        if self.target_locked is not None:
            self.get_logger().info(
                f"use locked target | "
                f"x={self.target_locked[0]:+.4f}, "
                f"y={self.target_locked[1]:+.4f}, "
                f"z={self.target_locked[2]:+.4f}",
                throttle_duration_sec=1.0,
            )
            return self.target_locked.copy()

        self.get_logger().warning(
            "execution target unavailable: target_policy=locked but target_locked is None",
            throttle_duration_sec=1.0,
        )
        return None

    # realtime 策略：始终依赖当前检测
    if policy == "realtime":
        return self._observe_target_in_base()

    # hybrid 策略：看得见则小范围刷新，看不见则用 locked
    if policy == "hybrid":
        observed = self._observe_target_in_base()

        if observed is not None:
            if self.target_locked is None:
                self.target_locked = observed.copy()
                self.target_locked_time = self._now_sec()
                return observed

            jump = float(np.linalg.norm(observed - self.target_locked))

            if jump <= self.target_refresh_max_jump:
                self.target_locked = observed.copy()
                self.target_locked_time = self._now_sec()

                self.get_logger().info(
                    f"hybrid target refreshed | jump={jump*1000:.1f}mm "
                    f"x={observed[0]:+.4f}, "
                    f"y={observed[1]:+.4f}, "
                    f"z={observed[2]:+.4f}",
                    throttle_duration_sec=1.0,
                )
                return observed

            self.get_logger().warning(
                f"hybrid observed target jump too large: "
                f"{jump*1000:.1f}mm > {self.target_refresh_max_jump*1000:.1f}mm, "
                f"keep locked target",
                throttle_duration_sec=1.0,
            )

        if self.target_locked is not None:
            self.get_logger().warning(
                "hybrid target not visible, use locked target",
                throttle_duration_sec=1.0,
            )
            return self.target_locked.copy()

        return None

    self.get_logger().warning(
        f"unknown target_policy={policy}, fallback to locked behavior",
        throttle_duration_sec=1.0,
    )

    if self.target_locked is not None:
        return self.target_locked.copy()

    return self._observe_target_in_base()
```

---

# 五、保留 `_compute_target_in_base()` 作为兼容入口

你的代码里很多地方已经调用 `_compute_target_in_base()`，不一定要全部改名。

可以把原来的 `_compute_target_in_base()` 改成一个包装函数：

```python
def _compute_target_in_base(self) -> Optional[np.ndarray]:
    """兼容旧接口。

    现在它不再直接等价于实时 YOLO 检测，
    而是根据状态和 target_policy 返回合适的目标。
    """
    return self._get_execution_target_in_base()
```

这样原有调用不容易漏改。

---

# 六、修改 `cb_trigger`

你现在 `cb_trigger()` 里有：

```python
target = self._compute_target_in_base()
```

建议这里明确用实时观察，而不是执行策略。

因为 trigger 的时候必须真的看见目标，才能锁定。

把：

```python
target = self._compute_target_in_base()
```

改成：

```python
target = self._observe_target_in_base()
```

然后锁定目标这段改得更明确：

```python
# 锁定目标位置。后续执行阶段优先使用这个 target_locked，
# 避免机械臂遮挡目标后 YOLO 丢失导致停止。
self.target_locked = target.copy()
self.target_locked_time = self._now_sec()
```

完整关键片段建议改成：

```python
target = self._observe_target_in_base()
if target is None:
    self.get_logger().warning("trigger refused: no valid visible target")
    return

hover_pt = target + np.array([0.0, 0.0, self.hover_z])
if not self._check_reachable(hover_pt):
    self.get_logger().warning("trigger refused: not reachable")
    return

self.target_locked = target.copy()
self.target_locked_time = self._now_sec()

det = max(self.latest_det.results, key=lambda r: r.confidence)
self.planned_yaw = float(det.angle)

self.get_logger().info(
    f">>> trigger: lock target and enter PLANNING | "
    f"target=({self.target_locked[0]:+.4f}, "
    f"{self.target_locked[1]:+.4f}, "
    f"{self.target_locked[2]:+.4f}) "
    f"yaw={self.planned_yaw:+.3f}"
)

self._enter_state(ServoState.PLANNING)
```

---

# 七、修改 `_compute_error_in_base`

你现在这里会因为 YOLO 空结果停住：

```python
target = self._compute_target_in_base()
if target is None:
    return None
```

现在 `_compute_target_in_base()` 已经会走 target policy，所以这里可以保留。

但建议加更清楚的日志：

```python
def _compute_error_in_base(self,
                           z_offset: float) -> Optional[np.ndarray]:
    """误差 = (执行目标位置 + z_offset) - 末端位置"""

    target = self._compute_target_in_base()

    if target is None:
        self.get_logger().warning(
            f"servo target unavailable in state={self.state.name}, output zero twist",
            throttle_duration_sec=1.0,
        )
        return None

    ee = self._lookup_ee_in_base()
    if ee is None:
        return None

    goal = target + np.array([0.0, 0.0, z_offset])
    error = goal - ee

    self.get_logger().info(
        f"servo goal | state={self.state.name} "
        f"target=({target[0]:+.4f}, {target[1]:+.4f}, {target[2]:+.4f}) "
        f"goal=({goal[0]:+.4f}, {goal[1]:+.4f}, {goal[2]:+.4f}) "
        f"ee=({ee[0]:+.4f}, {ee[1]:+.4f}, {ee[2]:+.4f}) "
        f"err=({error[0]:+.4f}, {error[1]:+.4f}, {error[2]:+.4f})",
        throttle_duration_sec=1.0,
    )

    return error
```

---

# 八、修改 `_on_approach_done`

这是第二个关键 bug 点。

你现在有：

```python
self.target_locked = self._compute_target_in_base()
```

这很危险。

因为 APPROACHING 收敛时，目标可能已经被机械臂遮挡。
如果此时重新计算目标失败，就可能把原来有效的 `target_locked` 弄坏。

建议把 `_on_approach_done()` 改成：

```python
def _on_approach_done(self):
    """APPROACHING 收敛回调。

    关键原则：
    - 不要因为当前 YOLO 遮挡就丢掉 target_locked
    - 默认继续使用 trigger 时锁定的目标
    - 如需刷新，只有在当前能看见目标且新旧目标距离很近时才刷新
    """

    if self.refresh_target_at_approach_done:
        observed = self._observe_target_in_base()

        if observed is not None:
            if self.target_locked is None:
                self.target_locked = observed.copy()
                self.target_locked_time = self._now_sec()
                self.get_logger().info(
                    "target_locked created at approach done"
                )
            else:
                jump = float(np.linalg.norm(observed - self.target_locked))

                if jump <= self.target_refresh_max_jump:
                    self.target_locked = observed.copy()
                    self.target_locked_time = self._now_sec()

                    self.get_logger().info(
                        f"target_locked refreshed at approach done | "
                        f"jump={jump*1000:.1f}mm "
                        f"x={self.target_locked[0]:+.4f}, "
                        f"y={self.target_locked[1]:+.4f}, "
                        f"z={self.target_locked[2]:+.4f}"
                    )
                else:
                    self.get_logger().warning(
                        f"target refresh rejected at approach done: "
                        f"jump={jump*1000:.1f}mm > "
                        f"{self.target_refresh_max_jump*1000:.1f}mm, "
                        f"keep previous target_locked"
                    )
        else:
            self.get_logger().warning(
                "target not visible at approach done, keep previous target_locked"
            )
    else:
        self.get_logger().info(
            "approach done: keep existing target_locked"
        )

    if self.target_locked is None:
        self.get_logger().warning(
            "no target_locked available at approach done, abort to DONE"
        )
        self._enter_state(ServoState.DONE)
        return

    if not self.get_parameter("enable_grasp_sequence").value:
        self._enter_state(ServoState.DONE)
    else:
        self._send_gripper(self.gripper_open_pos)
        self._enter_state(ServoState.DESCENDING)
```

---

# 九、修改 `_retreat_step`

你现在的 `_retreat_step()` 已经用的是：

```python
self.target_locked
```

这很好，不需要大改。

但建议在 `RETREATING` 也不要受 YOLO 影响，这一点现在已经符合。

---

# 十、修改 `_enter_state(IDLE)`

你现在：

```python
if new_state == ServoState.IDLE:
    self.target_locked = None
```

建议补上：

```python
if new_state == ServoState.IDLE:
    self.target_locked = None
    self.target_locked_time = None
```

如果你希望 DONE 后也清空，可以加：

```python
if new_state in (ServoState.IDLE, ServoState.DONE):
    ...
```

但我建议先只在 IDLE 清空。
因为 DONE 后保留 target_locked 方便你查看日志和调试。

---

# 十一、启动参数建议

当前红盒子是静态目标，使用：

```bash
ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=true \
  -p enable_grasp_sequence:=true \
  -p avoid_collisions:=false \
  -p target_policy:=locked \
  -p refresh_target_at_approach_done:=false \
  -p debug_print_target:=true \
  -p debug_print_ik:=true
```

确认逻辑正常后，再打开运动：

```bash
ros2 run visual_servo visual_servo_node \
  --ros-args \
  -p enable_motion:=true \
  -p enable_grasp_sequence:=true \
  -p avoid_collisions:=false \
  -p target_policy:=locked \
  -p refresh_target_at_approach_done:=false
```

---

# 十二、你应该看到的日志变化

修改前，遮挡后会出现：

```text
latest detection has empty results
servo 输出 zero twist
机械臂停住
```

修改后，即使 YOLO 空了，也应该看到：

```text
det received: empty results
use locked target | x=..., y=..., z=...
servo goal | state=APPROACHING ...
```

也就是说：

```text
YOLO 可以丢
servo 不能因为短暂遮挡丢目标
```

---

# 十三、这个版本的逻辑结果

修改后，系统逻辑会变成：

```text
trigger
  ↓
必须当前可见目标
  ↓
_observe_target_in_base()
  ↓
target_locked = target
  ↓
IK 检查 hover_pt
  ↓
MoveIt plan_to_pregrasp
  ↓
APPROACHING 使用 target_locked
  ↓
即使机械臂挡住目标，YOLO results=[]
  ↓
继续靠近 target_locked
  ↓
DESCENDING 继续使用 target_locked
  ↓
GRASPING
  ↓
RETREATING 使用 target_locked + hover_z
  ↓
DONE
```

这就是解决你当前逻辑 bug 的完整方法。

---

# 十四、我建议你现在先改这几个函数

按优先级：

```text
1. cb_det
2. 新增 _observe_target_in_base
3. 新增 _get_execution_target_in_base
4. 修改 _compute_target_in_base
5. 修改 cb_trigger
6. 修改 _compute_error_in_base
7. 修改 _on_approach_done
```

你不需要马上动 YOLO 节点。
YOLO 节点继续真实发布：

```text
看得见：results 非空
看不见：results 空
```

这是正确的。真正要修的是 `visual_servo_node` 对遮挡的处理策略。

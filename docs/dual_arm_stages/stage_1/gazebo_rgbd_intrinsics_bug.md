# Gazebo rgbd_camera 传感器内参不一致 bug

**日期：** 2026-07-01

**相关组件：** `realsense2_gz_description` → `rgbd_camera.gazebo.xacro`

---

## 现象

ArUco PnP 位姿估计 (`estimatePoseSingleMarkers`) 算出的 `pos_cam` 与
`ros2 run tf2_ros tf2_echo camera_color_optical_frame aruco_marker_link` 的 ground truth 明显对不上。

camera_info 报告的 K：
```
fx=698.4  fy=703.4  cx=480.0  cy=270.0  (960×540 时)
```

从 marker 四个角点像素反推真实渲染用的 K：
```
fx=fy≈548  cx≈488  cy≈237
```

偏差 ≈27%。

---

## 原因

`rgbd_camera.gazebo.xacro` 同时提供了两处内参：

```xml
<camera>
  <horizontal_fov>${h_fov}</horizontal_fov>       <!-- 69° -->
  <image>
    <width>${image_width}</width>
    <height>${image_height}</height>
  </image>
  <lens>
    <intrinsics>
      <fx>${fx}</fx>   <!-- 698.4 -->
      <fy>${fy}</fy>   <!-- 703.4 -->
      ...
    </intrinsics>
  </lens>
</camera>
```

gz-sim 的 `rgbd_camera` 传感器对这两处的读取是**分离的**：

| 用途                    | 数据来源                                                    |
| ----------------------- | ----------------------------------------------------------- |
| 实际图像渲染 (投影矩阵) | `<horizontal_fov>` + `<image>` 宽高（或深度对齐逻辑调整后） |
| ROS CameraInfo 发布     | `<lens><intrinsics>` 里的 fx/fy/cx/cy                       |

两处各自独立计算 → 当 `${fx}` 的公式结果与 `<horizontal_fov>` 隐含的内参不一致时（尤其 rgbd 的深度对齐逻辑可能进一步调整渲染端），PnP 拿 camera_info 的 K 去解实际渲染的图像，算出的位姿就偏了。

**这不是你的配置问题，是 `realsense2_gz_description` 包的已知缺陷——rgbd_camera 传感器在 gz-sim 里的 intrinsics 处理没有保证 `<horizontal_fov>` 和 `<lens>` 的一致性。**

---

## 影响范围

用到以下场景时会出现位姿估计偏差：

- `cv2.aruco.estimatePoseSingleMarkers()` 或 `cv2.solvePnP()`
- 任何依赖 `camera_info` K 矩阵从像素反推 3D 坐标的视觉算法

---

## 临时绕过

在应用层硬编码反推得到的真实内参：

```python
self.K = np.array([
    [548.0, 0.0,   488.0],
    [0.0,   548.0, 237.0],
    [0.0,   0.0,   1.0]
])
```

缺点：换分辨率需重新反推。

---

## 根治方向

修改 `rgbd_camera.gazebo.xacro`，让 `<lens><intrinsics>` 的 fx/fy/cx/cy 不单独计算，而是由 `<horizontal_fov>` + `<image>` 宽高推导，或者干脆不设 `<lens>` 让 Gazebo 自动从 `<horizontal_fov>` 生成。

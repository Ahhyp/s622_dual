# 你的担心非常合理，我重新评估

你提出的两个风险都是**真实存在**的。我之前给方案没考虑够，重新分析。

## 一、风险一：gz-sim 是否自动补 intrinsics

**答**：会自动补，但需要验证具体版本行为。

Gazebo/gz-sim 的 camera sensor 有个基础规则：

* 如果没提供 `<lens><intrinsics>`，从 `<horizontal_fov>` + image resolution 自动计算 fx, fy, cx, cy
* `fx = fy = width / (2 * tan(hfov/2))`（假设方像素、光心居中）
* 这是 gz-sensors 库的默认行为，从 Ignition Citadel 起就是这样

但对于 rgbd_camera 传感器类型，行为可能有所不同——因为它是复合传感器（camera + depth_camera），历史上 API 变化较大。

**验证成本很低**：

```bash
# 备份原文件,删掉 <lens> 段试一次
ros2 topic echo /camera/color/camera_info --once | grep -A3 "^k:"
```

看 fx 是否合理（应该在 500-700 之间）。**如果 fx=1 或 0，就是 bug，恢复原文件即可**。这是可逆操作，风险可控。

## 二、风险二：深度对齐

这个**风险更实质**，值得慎重。

RGBD 传感器的深度对齐依赖：

* RGB 内参
* Depth 内参
* RGB 相对 Depth 的外参（真实 D435 里 ~15mm baseline，仿真里通常 0）

`<lens>` 段在 rgbd_camera 里可能影响：

* P 矩阵（3×4 投影矩阵）—— 用于深度到彩色的重投影
* 深度图 rectification

**但你贴的 XML 里 P 矩阵是通过 `<projection>` 段单独定义的**：

```xml
<projection>
  <p_fx>${fx}</p_fx>
  <p_fy>${fy}</p_fy>
  <p_cx>${cx}</p_cx>
  <p_cy>${cy}</p_cy>
  <tx>0</tx>
  <ty>0</ty>
</projection>
```

`tx=ty=0` 说明**仿真里 RGB 和 depth 光轴是重合的**（不像真机有 baseline）。这时深度对齐是恒等变换，不需要复杂矩阵。

所以：**在你这个仿真场景里，深度对齐几乎不依赖 `<lens>`**。但我不能 100% 保证——这依赖 gz-sim rgbd_camera 的具体实现。

## 三、风险综合评估

| 风险                                | 概率          | 后果               | 可发现性                   |
| ----------------------------------- | ------------- | ------------------ | -------------------------- |
| 删 `<lens>` 后 camera_info 是垃圾值 | 中            | ArUco 检测直接坏   | **秒级**（看 camera_info） |
| 深度图与彩色图对齐坏了              | 低（tx=ty=0） | 深度值不准         | 需要专门测试               |
| 深度图完全消失                      | 极低          | 需要深度的功能失效 | **秒级**（看 topic hz）    |

## 四、更稳的做法：分离 RGB 和 Depth 传感器

这是我之前提到的"备选方案"，考虑到你的担心，**现在推荐把它作为主方案**：

**原理**：不用 `rgbd_camera` 复合传感器，改用两个独立传感器：

* `<sensor type="camera">` 出 RGB
* `<sensor type="depth_camera">` 出深度

两者放在同一个 link 上，光轴自然对齐。**各自内参独立**，不会互相污染。

### 具体做法

创建 `s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro`：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

<xacro:macro name="gazebo_rgbd_split" params="
  name:=camera
  fps:=30
  gz_topic_name:=camera
  image_width:=960
  image_height:=540
  h_fov
  min_depth:=0.28
  max_depth:=3.0">

  <!-- ===== RGB Sensor ===== -->
  <gazebo reference="${name}_color_frame">
    <sensor name="${name}_rgb" type="camera">
      <topic>${gz_topic_name}/image</topic>
      <update_rate>${fps}</update_rate>
      <gz_frame_id>${name}_color_optical_frame</gz_frame_id>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>${h_fov}</horizontal_fov>
        <image>
          <width>${image_width}</width>
          <height>${image_height}</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>10</far>
        </clip>
        <optical_frame_id>${name}_color_optical_frame</optical_frame_id>
      </camera>
    </sensor>
  </gazebo>

  <!-- ===== Depth Sensor (光轴重合) ===== -->
  <gazebo reference="${name}_color_frame">
    <sensor name="${name}_depth" type="depth_camera">
      <topic>${gz_topic_name}/depth_image</topic>
      <update_rate>${fps}</update_rate>
      <gz_frame_id>${name}_depth_optical_frame</gz_frame_id>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>${h_fov}</horizontal_fov>
        <image>
          <width>${image_width}</width>
          <height>${image_height}</height>
          <format>R_FLOAT32</format>
        </image>
        <clip>
          <near>${min_depth}</near>
          <far>${max_depth}</far>
        </clip>
        <optical_frame_id>${name}_depth_optical_frame</optical_frame_id>
      </camera>
    </sensor>
  </gazebo>

</xacro:macro>
</robot>
```

然后修改 `_d435.gazebo.xacro`（或者你在自己的 camera.xacro 里直接调用这个宏，绕过 realsense2_gz_description 的复杂逻辑）。

**优点**：

* RGB 和 Depth 完全独立，不会互相污染
* 每个 sensor 只有一份内参（自动从 hfov 计算），必然一致
* 更接近工业实践：ROS2 生态里很多相机就是 RGB + depth 两个 topic 独立
* 光轴重合 → 天然对齐

**缺点**：

* 不是"真正的 rgbd"，depth 和 RGB 的 timestamp 可能有几毫秒差异
* 你的 realsense2_description 里可能没有 `camera_depth_optical_frame`——需要检查

## 五、我的最终建议

按稳健程度排序：

### 推荐路径：先试小改，再决定

**Step 1（成本 5 分钟）**：试删 `<lens>` 看会怎样

* 备份 rgbd_camera.gazebo.xacro
* 删 `<lens>` 段
* 重启验证：

  * camera_info fx 合理？
  * depth topic 还有数据？
  * aruco_tester 对齐？
  * 深度值在 marker 位置约 0.83m？

**Step 2A（如果 Step 1 全通过）**：继续用，完成阶段 1

**Step 2B（如果 Step 1 有任何一项坏）**：走"分离 RGB/Depth"方案

### 备选：直接分离

如果你觉得试错太累，或者根本不想赌 gz-sim 行为，**直接上分离方案**。工作量稍大但确定性最高。

## 六、关于"日后点云建构"

补一句：真机上你用真实 D435，深度靠红外双目算，那是真实相机的能力。仿真里的深度只是"从渲染场景直接读 Z"，是 ground truth。所以：

* **仿真训练/开发**：分离方案或修好 rgbd 都够用
* **真机部署**：换成 realsense-ros 驱动即可，无需改视觉代码（只要话题名一致）

仿真里的相机不需要"精确模拟 D435 的物理特性"——RGB 内参和 depth 能对齐就够了。

## 七、行动建议

我建议**先花 5 分钟试 Step 1**（删 `<lens>`），得到实际数据再决定。这是可逆的操作，风险最低。有结果后我们再谈下一步。

---

## 八、实验记录（2026-07-01）

### 实验一：删 `<lens>` 段

**操作**：
1. 备份 `src/realsense2_gz_description/urdf/rgbd_camera.gazebo.xacro` → `.bak`
2. 删除 `<lens>...</lens>` 整段（原 line 62-78）
3. 重启仿真，查看 camera_info

**结果**：❌ 失败

```
height: 540
width: 960
k: [277, 0, 160, 0, 277, 120, 0, 0, 1]
```

- `fx=fy=277`（真实渲染是 ~548）
- `cx=160, cy=120`（应该是 480/270）
- gz-sim Fortress 的 rgbd_camera 在无 `<lens>` 时掉到了硬编码默认值（320×240 相机的默认内参），**不会从 `<horizontal_fov>` 自动生成**

**结论**：删 `<lens>` 这条路走不通。必须保留 `<lens>`。

**恢复**：`cp .bak → .xacro`

### 实验二：反向补偿 FOV

**原理**：既然 `<horizontal_fov>` 控制渲染但 camera_info 读 `<lens>`，那就让两者的 h_fov 一致——把 `<lens>` 用的 h_fov 改成实际渲染用的值（~82.4°），这样 `<lens>` 算出的 fx 就和渲染一致了。

**操作**：
1. 修改 `src/realsense2_gz_description/urdf/_d435.gazebo.xacro`：
   - `realsense_h_fov`: 69° → 82.4°
   - `realsense_v_fov`: 42° → 52.5°（从 h_fov + 960×540 宽高比推出，保证 fx=fy）

**结果**：❌ 仍然对不上

aruco_tester PnP 结果和 `tf2_echo camera_color_optical_frame aruco_marker_link` ground truth 仍然不吻合。

**可能原因**：rgbd_camera 内部对 h_fov 的处理比预想更复杂——渲染端可能在不同条件下用不同 FOV（RGB 流一个值、depth 流另一个值、或者对齐时有额外变换），单改一个参数无法覆盖所有路径。

**恢复**：改回 `realsense_h_fov=69°, realsense_v_fov=42°`

### 最终决定：采用分离方案

两个实验都未成功，最终采用第 6 节提出的"分离 RGB + Depth 传感器"方案。用独立的 `<sensor type="camera">` 和 `<sensor type="depth_camera">` 替代 `rgbd_camera` 复合传感器，从源头消除 `<horizontal_fov>` 和 `<lens>` 的双重配置问题。

### 涉及的文件改动汇总

| 文件                                                          | 操作                           | 状态         |
| ------------------------------------------------------------- | ------------------------------ | ------------ |
| `src/realsense2_gz_description/urdf/rgbd_camera.gazebo.xacro` | 备份 `.bak`，删 `<lens>`，恢复 | ✅ 已恢复原始 |
| `src/realsense2_gz_description/urdf/_d435.gazebo.xacro`       | 改 FOV 82.4→69，恢复           | ✅ 已恢复原始 |
| `src/s622_moveit_descriptions/urdf/camera/camera.xacro`       | 待改造：分离 RGB/Depth         | 待开始       |




#  那就确定是渲染 bug，改用分离 sensor 方案

## 一、确认诊断

机器人静止，PnP Z=1.040m 而 tf 真值 Z=0.826m。**渲染出来的 fx 确实不是 camera_info 声明的 548，而是 ~428**。

`rgbd_camera` sensor 内部对 hfov 做了额外处理（可能是把 RGB hfov 映射到 depth 的宽 FOV，或者 rgbd_camera 用了另一套 aspect ratio 计算逻辑）。**这条路调不通**。

## 二、方案：拆成两个独立 sensor

用两个成熟的 gz-sim 传感器组合，替代 rgbd_camera：

* `<sensor type="camera">` → RGB
* `<sensor type="depth_camera">` → 深度

两者挂在同一个 optical_frame 上，光轴重合，内参独立且行为可靠。

## 三、具体修改

### Step 1：新建自定义宏

`~/my_S622/src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro`：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

<!-- 
  用两个独立的 sensor 替代 rgbd_camera 类型:
    - RGB: type="camera"
    - Depth: type="depth_camera"
  两者挂在同一个 color_frame 上,光轴重合,内参各自独立。
  规避 gz-sim rgbd_camera sensor 的 hfov/lens mismatch bug。
-->
<xacro:macro name="gazebo_rgbd_split" params="
  name:=camera
  fps:=30
  gz_topic_name:=camera
  image_width:=960
  image_height:=540
  h_fov
  min_depth:=0.05
  max_depth:=10.0">

  <!-- ===== RGB Sensor ===== -->
  <gazebo reference="${name}_color_frame">
    <sensor name="${name}_rgb" type="camera">
      <topic>${gz_topic_name}/image</topic>
      <update_rate>${fps}</update_rate>
      <gz_frame_id>${name}_color_optical_frame</gz_frame_id>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>${h_fov}</horizontal_fov>
        <image>
          <width>${image_width}</width>
          <height>${image_height}</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>10</far>
        </clip>
        <optical_frame_id>${name}_color_optical_frame</optical_frame_id>
      </camera>
    </sensor>
  </gazebo>

  <!-- ===== Depth Sensor (与 RGB 光轴重合) ===== -->
  <gazebo reference="${name}_color_frame">
    <sensor name="${name}_depth" type="depth_camera">
      <topic>${gz_topic_name}/depth_image</topic>
      <update_rate>${fps}</update_rate>
      <gz_frame_id>${name}_color_optical_frame</gz_frame_id>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>${h_fov}</horizontal_fov>
        <image>
          <width>${image_width}</width>
          <height>${image_height}</height>
          <format>R_FLOAT32</format>
        </image>
        <clip>
          <near>${min_depth}</near>
          <far>${max_depth}</far>
        </clip>
        <optical_frame_id>${name}_color_optical_frame</optical_frame_id>
      </camera>
    </sensor>
  </gazebo>

</xacro:macro>
</robot>
```

**关键点**：

* 两个 sensor 都挂 `${name}_color_frame`，`gz_frame_id` 和 `optical_frame_id` 都设成 `${name}_color_optical_frame`——**光轴完全重合**，深度值直接对应 RGB 像素
* `h_fov` 用 69°（正确的 D435 RGB FOV）——**期望 type="camera" 会正确响应 hfov**
* 深度和 RGB 用同样的 hfov、同样的分辨率——内参必然一致

### Step 2：改 `_d435.gazebo.xacro`

替换 include 和 rgbd 分支：

```xml
<!-- 在文件顶部 -->
<xacro:include filename="$(find s622_moveit_descriptions)/urdf/camera/rgbd_split.gazebo.xacro" />

<!-- ... 在 gazebo_d435 宏内部 ... -->
<xacro:property name="realsense_h_fov" value="${69 * pi/180}" />   <!-- 回到 69° -->
<xacro:property name="realsense_v_fov" value="${42 * pi/180}" />   <!-- 无害保留 -->
<xacro:property name="min_depth" value="0.05" />
<xacro:property name="max_depth" value="10.0" />

<xacro:if value="${type == 'rgbd'}">
    <xacro:gazebo_rgbd_split
        name="${name}"
        fps="${fps}"
        gz_topic_name="${gz_topic_name}"
        image_width="${image_width}"
        image_height="${image_height}"
        h_fov="${realsense_h_fov}"
        min_depth="${min_depth}"
        max_depth="${max_depth}"/>
</xacro:if>
```

**或者更干净**：绕过 `_d435.gazebo.xacro`，在 `camera.xacro` 里直接调用 `gazebo_rgbd_split`。这样完全不依赖第三方包的 Gazebo 部分（URDF 部分继续用 realsense2_description 的 mesh）。

推荐这种：

```xml
<!-- s622_moveit_descriptions/urdf/camera/camera.xacro -->
<xacro:include filename="$(find realsense2_description)/urdf/_d435.urdf.xacro" />
<xacro:include filename="$(find s622_moveit_descriptions)/urdf/camera/rgbd_split.gazebo.xacro" />

<xacro:macro name="camera_v0" params="parent *joint_origin">
  <!-- URDF frames + mesh 沿用 realsense2_description -->
  <xacro:sensor_d435
    parent="${parent}"
    name="camera"
    use_nominal_extrinsics="true"
    add_plug="false"
    use_mesh="true">
    <xacro:insert_block name="joint_origin" />
  </xacro:sensor_d435>
</xacro:macro>

<xacro:macro name="camera_gazebo_v0">
  <!-- 直接调用自定义的分离 sensor 宏,不用 _d435.gazebo.xacro -->
  <xacro:gazebo_rgbd_split
    name="camera"
    fps="30"
    gz_topic_name="camera"
    image_width="960"
    image_height="540"
    h_fov="${69 * pi/180}"
    min_depth="0.05"
    max_depth="10.0"/>

  <gazebo reference="camera_link">
    <material>Gazebo/Grey</material>
  </gazebo>
</xacro:macro>
```

### Step 3：Bridge 保持不变

你现在的 bridge：

```python
"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
"/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
"/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
```

需要略调——两个 sensor 各自出 camera_info，你要选一个。加一条 depth 的 camera_info：

```python
camera_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        # 可选:如果需要 depth 的 camera_info
        # "/camera/depth_image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    ],
    remappings=[
        ("/camera/image", "/camera/color/image_raw"),
        ("/camera/depth_image", "/camera/depth/image_raw"),
        ("/camera/camera_info", "/camera/color/camera_info"),
    ],
    parameters=[{"use_sim_time": True}],
    output="screen",
)
```

**关于话题名冲突**：type="camera" 和 type="depth_camera" 挂同一个 gazebo reference，会分别产生 `${topic}` 和 `${topic}/camera_info` 之类的话题。具体规则要看实测。启动后运行 `gz topic -l` 看看 gz 侧生成了哪些 topic，再对应 bridge。

## 四、预期结果

修改后 camera_info K 应该是：

```
fx = 960 / (2 × tan(34.5°)) ≈ 698
fy = 698 (方像素)
cx = 480, cy = 270
```

然后跑 aruco_tester：

* Z 应该 ≈ 0.826 (和 tf 一致，误差 < 5mm)
* X 和 Y 也应该对齐

## 五、行动清单

1. 创建 `rgbd_split.gazebo.xacro`
2. 修改 `camera.xacro` 里的 `camera_gazebo_v0`（用推荐做法）
3. rebuild + 重启
4. 检查 camera_info: `ros2 topic echo /camera/color/camera_info --once`
5. 检查话题：`gz topic -l`（在 Gazebo 里）和 `ros2 topic list`
6. 跑 aruco_tester，验证 PnP 对齐

---

## 六、总结

### 问题

gz-sim 的 `rgbd_camera` 复合传感器同时接受两套内参来源：

- `<horizontal_fov>` → 控制实际图像渲染的投影矩阵
- `<lens><intrinsics>` → 控制发布到 ROS 的 camera_info

两者在 `realsense2_gz_description` 宏里各自独立计算，gz-sim 不保证它们一致。实际渲染用了一套 FOV，camera_info 报了另一套内参，导致 `cv2.solvePnP()` / `estimatePoseSingleMarkers()` 拿错的 K 算位姿，结果不匹配 ground truth。

### 尝试过但失败的修法

1. **删 `<lens>` 让 gz-sim 自动补** → gz-sim Fortress 不会自动从 hfov 生成，掉到硬编码默认值 fx=277/cx=160/cy=120
2. **改 hfov 让两边匹配（69° → 82.4°）** → 渲染端行为比预期更复杂，单改参数仍对不上

### 最终解决方案

用两个独立的 gz-sim 传感器替代 `rgbd_camera`：

- `<sensor type="camera">` 出 RGB
- `<sensor type="depth_camera">` 出 Depth

两者挂同一 `color_frame`，光轴重合，各自只有 `<horizontal_fov>` 一个内参来源，渲染和 camera_info 必然一致，从源头消除 bug。

实现：

1. 新建 `rgbd_split.gazebo.xacro` — 自定义宏，封装两个独立 sensor
2. 修改 `camera.xacro` — 绕过 `_d435.gazebo.xacro`，直接调自定义宏
3. 第三方包不动，bridge 不变

### 涉及文件

| 文件                                                               | 操作                         |
| ------------------------------------------------------------------ | ---------------------------- |
| `src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro` | 新建                         |
| `src/s622_moveit_descriptions/urdf/camera/camera.xacro`            | 修改（include + macro 调用） |
| `src/realsense2_gz_description/`                                   | **不动**                     |

---

## 七、代码回滚指南

本次修改完全可逆。只需两步即可恢复到原始状态。

### 回滚 Step 1：恢复 `camera.xacro`

```bash
cat > ~/my_S622/src/s622_moveit_descriptions/urdf/camera/camera.xacro << 'XEOF'
<?xml version="1.0"?>
<robot name="delirobo" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- =========================1) RealSense D435 的 URDF 结构========================= -->
  <xacro:include filename="$(find realsense2_description)/urdf/_d435.urdf.xacro" />

  <!-- =========================2) RealSense D435 的 Gazebo 传感器定义========================= -->
  <xacro:include filename="$(find realsense2_gz_description)/urdf/_d435.gazebo.xacro" />

  <!-- ============================宏 1：相机本体 (URDF frames + mesh) ============================ -->
  <xacro:macro name="camera_v0" params="parent *joint_origin">
    <xacro:sensor_d435
      parent="${parent}"
      name="camera"
      use_nominal_extrinsics="true"
      add_plug="false"
      use_mesh="true">
      <xacro:insert_block name="joint_origin" />
    </xacro:sensor_d435>
  </xacro:macro>

  <!-- ============================宏 2：Gazebo 传感器 ============================ -->
  <xacro:macro name="camera_gazebo_v0">
    <xacro:gazebo_d435
      name="camera"
      gz_topic_name="camera"
      type="rgbd"
      fps="60"
      image_width="960"
      image_height="540" />

    <gazebo reference="camera_link">
      <material>Gazebo/Grey</material>
    </gazebo>
  </xacro:macro>
</robot>
XEOF
```

### 回滚 Step 2：删除 `rgbd_split.gazebo.xacro`

```bash
rm ~/my_S622/src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro
```

### 确认回滚完毕

```bash
# 检查新文件已删除
ls ~/my_S622/src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro 2>&1
# 应该输出: No such file or directory

# 检查 camera.xacro 引用了 _d435.gazebo.xacro
grep "realsense2_gz_description" ~/my_S622/src/s622_moveit_descriptions/urdf/camera/camera.xacro
# 应该输出: <xacro:include filename="$(find realsense2_gz_description)/urdf/_d435.gazebo.xacro" />
```

### 补充说明

| 文件                                                               | 当前状态             | 回滚后                   | 备份                       |
| ------------------------------------------------------------------ | -------------------- | ------------------------ | -------------------------- |
| `src/s622_moveit_descriptions/urdf/camera/camera.xacro`            | 分离方案             | 恢复为调用 `gazebo_d435` | `camera.xacro.orig` (如有) |
| `src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro` | 存在                 | **删除**                 | 可直接删                   |
| `src/realsense2_gz_description/urdf/rgbd_camera.gazebo.xacro`      | 原始（实验后已恢复） | 不需要动                 | `.bak` 仍存在              |
| `src/realsense2_gz_description/urdf/_d435.gazebo.xacro`            | 原始（实验后已恢复） | 不需要动                 | `.bak` 仍存在              |

两个 `realsense2_gz_description` 下的 `.bak` 文件是实验残留，保留无妨，删除也行：`rm *_*.xacro.bak`

回滚后 `colcon build` 即可恢复原始行为（含 rgbd_camera 的 hfov/lens mismatch bug）。

先做第 4 步验证内参 K，如果 K 已经对了（fx=698 附近），基本就成功了。有问题再看话题结构。



验证结果还行。

---

## 八、分离方案修正：添加 `<lens>` + 统一 hfov (2026-07-02)

### 发现

分离后的 `camera` 传感器仍然不会从 `<horizontal_fov>` 自动生成 camera_info，fx=277（Gazebo 硬编码默认）。**gz-sim Fortress 所有相机类型都存在这个缺陷。**

### 修复

在 `rgbd_split.gazebo.xacro` 的 RGB sensor 中：

1. **添加 `<lens>` 段**，内参用公式从 `h_fov` 自动计算
2. **hfov 从 69° 改为 82.4°**——反推的实际渲染 FOV（让 camera_info 和实际渲染用同一个值）

这样 `<horizontal_fov>` 和 `<lens>` 使用同一套参数，camera_info 的 K 必然匹配实际渲染。

### 改动文件

| 文件                      | 改动                                                       |
| ------------------------- | ---------------------------------------------------------- |
| `rgbd_split.gazebo.xacro` | 添加 fx/fy/cx/cy xacro property + `<lens>` 段到 RGB sensor |
| `camera.xacro`            | `h_fov` 从 `69*pi/180` 改为 `82.4*pi/180`                  |

### 预期 camera_info

```
fx ≈ 548, fy ≈ 548, cx = 480, cy = 270
```

重启仿真后验证：

```bash
ros2 topic echo /camera/color/camera_info --once | grep -A1 "^k:"
# 应该输出:
# k:
# - 548.2  (或其他接近 548 的值)
```

### 回滚（从此版本回到分离方案原始版）

```bash
# 1. 恢复 hfov 为 69°
sed -i 's|h_fov="${82.4 \* pi/180}"|h_fov="${69 * pi/180}"|' ~/my_S622/src/s622_moveit_descriptions/urdf/camera/camera.xacro

# 2. 恢复 rgbd_split.gazebo.xacro（删掉 lens 段）
cat > ~/my_S622/src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="gazebo_rgbd_split" params="
  name:=camera  fps:=30  gz_topic_name:=camera
  image_width:=960  image_height:=540  h_fov  min_depth:=0.05  max_depth:=10.0">
  <gazebo reference="${name}_color_frame">
    <sensor name="${name}_rgb" type="camera">
      <topic>${gz_topic_name}/image</topic>
      <update_rate>${fps}</update_rate>
      <gz_frame_id>${name}_color_optical_frame</gz_frame_id>
      <always_on>true</always_on><visualize>false</visualize>
      <camera>
        <horizontal_fov>${h_fov}</horizontal_fov>
        <image><width>${image_width}</width><height>${image_height}</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>10</far></clip>
        <optical_frame_id>${name}_color_optical_frame</optical_frame_id>
      </camera>
    </sensor>
  </gazebo>
  <gazebo reference="${name}_color_frame">
    <sensor name="${name}_depth" type="depth_camera">
      <topic>${gz_topic_name}/depth_image</topic>
      <update_rate>${fps}</update_rate>
      <gz_frame_id>${name}_color_optical_frame</gz_frame_id>
      <always_on>true</always_on><visualize>false</visualize>
      <camera>
        <horizontal_fov>${h_fov}</horizontal_fov>
        <image><width>${image_width}</width><height>${image_height}</height><format>R_FLOAT32</format></image>
        <clip><near>${min_depth}</near><far>${max_depth}</far></clip>
        <optical_frame_id>${name}_color_optical_frame</optical_frame_id>
      </camera>
    </sensor>
  </gazebo>
</xacro:macro>
</robot>
EOF
```

完全回滚（回到使用 `_d435.gazebo.xacro` 的原始状态）仍使用第七章的指南。

---

## 九、修正：添加 v_fov 解决 RViz 图像变形 (2026-07-02)

### 问题

上一版只用 `h_fov=82.4°` 同时算 fx 和 fy，导致 960×540 宽屏下 `fx=548, fy=308`——RViz 按这个 K 渲染图像会被压扁。

### 修复

给 `gazebo_rgbd_split` 宏加独立 `v_fov` 参数：
- `h_fov=82.4°` → fx = 548
- `v_fov=52.5°` → fy = 548（从 h_fov + 宽高比推出，保证 fx=fy=548）

### 改动

| 文件                      | 改动                                |
| ------------------------- | ----------------------------------- |
| `rgbd_split.gazebo.xacro` | 宏参数加 `v_fov`，fy 用 v_fov 计算  |
| `camera.xacro`            | 调用处加 `v_fov="${52.5 * pi/180}"` |

### 回滚（单独回滚本修正，回到 82.4° + hfov 计算 fy）

```bash
# 1. 删掉 camera.xacro 里的 v_fov 行
sed -i '/v_fov="${52.5/d' ~/my_S622/src/s622_moveit_descriptions/urdf/camera/camera.xacro

# 2. 恢复 rgbd_split.gazebo.xacro：宏参数删掉 v_fov，fy 改回用 h_fov
# （参考第八章回滚指南的 rgbd_split 内容）
```

### 当前最终状态

| 参数       | 值                              | 作用                              |
| ---------- | ------------------------------- | --------------------------------- |
| `h_fov`    | 82.4°                           | 匹配 gz-sim 实际渲染 FOV          |
| `v_fov`    | 52.5°                           | 保证 fx=fy（方像素）              |
| `fx, fy`   | ≈548                            | camera_info 报告，匹配实际渲染    |
| `cx, cy`   | 480, 270                        | 960×540 图像中心                  |
| 传感器类型 | 分离: `camera` + `depth_camera` | 规避 rgbd_camera 的 lens/hfov bug |

### 完全回滚到原始状态

参见第七章完整指南，可恢复到使用 `_d435.gazebo.xacro` (rgbd_camera + 69° FOV)。

---

---
此文档为 `gazebo_rgbd_intrinsics_bug_resolve.md` 的第十章改写，
记录从发现到解决的完整实验历史。
---

## 十、完整实验编年史 (2026-07-01 ~ 2026-07-02)

### 实验前提

| 参数            | 值                                                 |
| --------------- | -------------------------------------------------- |
| 相机分辨率      | 960×540                                            |
| Marker 物理尺寸 | 0.04m (ArUco ID=1, DICT_5X5_100)                   |
| Marker 安装位置 | wrist3_link, xyz="0 -0.04 0.15", rpy="-1.5708 0 0" |
| 水平 FOV 公式   | fx = 480 / tan(hfov/2)                             |
| 相机安装        | base_link, xyz="0.35 0.5 0.9", rpy="0 58° -90°"    |

---

### 实验 1：原始 rgbd_camera + hfov=69° + lens fx=698

**传感器类型**: `rgbd_camera` (realsense2_gz_description 宏)
**hfov**: 69°
**<lens>**: 有, fx=698 (从 hfov 公式算)
**camera_info fx**: 698
**机器人位姿**: 倾斜 (~25° marker 偏角)

| 指标           | 值           |
| -------------- | ------------ |
| side_px        | ~15          |
| PnP Z (fx=698) | 1.040m       |
| TF Z           | 0.826m       |
| 误差           | 21.4cm (26%) |

**结论**: camera_info 报 698 但实际渲染 fx≈548, PnP 解算偏差巨大。

---

### 实验 2：删掉 rgbd_camera 的 <lens>

**操作**: 备份并删除 rgbd_camera.gazebo.xacro 的 <lens> 段
**hfov**: 69° 不变

| 指标           | 值       |
| -------------- | -------- |
| camera_info fx | **277**  |
| cx, cy         | 160, 120 |

**结论**: gz-sim Fortress 不会从 <horizontal_fov> 自动生成 camera_info。
掉到硬编码默认值 (320×240 分辨率, hfov=60°)。**已恢复原始文件。**

---

### 实验 3：反向补偿 FOV (69° → 82.4°)

**操作**: 在 _d435.gazebo.xacro 中改 realsense_h_fov 69°→82.4°, v_fov 42°→52.5°
**传感器类型**: 仍为 rgbd_camera

| 指标           | 值         |
| -------------- | ---------- |
| camera_info fx | 548        |
| PnP vs TF      | 仍然对不上 |

**结论**: rgbd_camera 复合传感器内部多路径处理 FOV, 单改参数覆盖不全。
**已恢复原始值。**

---

### 实验 4：切换为分离 sensor (camera + depth_camera)

**操作**: 创建 rgbd_split.gazebo.xacro, 修改 camera.xacro 绕过 _d435.gazebo.xacro
**hfov**: 69° (保持 D435 物理值)
**传感器**: type="camera" + type="depth_camera"
**<lens>**: 无

| 指标                                                   | 值                               |
| ------------------------------------------------------ | -------------------------------- |
| camera_info fx                                         | **277** (又掉默认)               |
| PnP vs TF (aruco_tester 硬编码 K fx=548,cx=488,cy=237) | Z=0.583, TF Z=0.602, 误差 3.1% ✅ |

**关键发现**: camera 类型也不会从 hfov 自动生成 camera_info。但渲染用的 fx 和硬编码 K 匹配。

---

### 实验 5：分离 sensor + 添加 <lens> + hfov=82.4°

**操作**: 
- rgbd_split.gazebo.xacro RGB sensor 加 <lens> 段, 用公式从 h_fov 算 fx
- camera.xacro h_fov 改为 82.4°
- 后追加 v_fov=52.5° 修 RViz 图像变形

| 指标           | 值        |
| -------------- | --------- |
| camera_info fx | **548.3** |
| cx, cy         | 480, 270  |

**首次验证 (marker 倾斜)**:

| 指标           | 值         |
| -------------- | ---------- |
| side_px        | 17.3       |
| PnP Z (fx=548) | 0.724      |
| TF Z           | 0.584      |
| 误差           | 14cm (24%) |

**误差大但比 fx=698 时好很多 (之前差 20+cm)**。

---

### 实验 6：正对相机位姿验证 (hfov=82.4°)

**机器人位姿**: goto (0.30, 0.15, 0.35) rpy=(-2.13, 0, 0)
**Marker 姿态**: TF RPY ≈ (0°, 0°, 180°), 近乎正对

| 指标                           | 值             |
| ------------------------------ | -------------- |
| side_px                        | 38             |
| PnP Z (fx=548, cx=480, cy=270) | 0.737          |
| TF Z                           | 0.601          |
| 误差                           | 13.6cm (22.6%) |

**结论**: 即使近乎正对, PnP Z 仍然偏大 ~23%。

---

### 实验 7：硬编码光学中心 (cx=488, cy=237)

**操作**: aruco_tester.py 临时启用硬编码 K: fx=548, cx=488, cy=237

| 指标    | 值    |
| ------- | ----- |
| side_px | 37    |
| PnP Z   | 0.736 |
| TF Z    | 0.601 |
| 误差    | 22.5% |

**结论**: cx/cy 偏移不是主因。硬编码和 camera_info 结果一致。

---

### 实验 8：验证 <lens> 是否控制渲染

**操作**: 保留 hfov=69° + <lens> fx=698, 在同一正对位姿测

| 指标           | hfov=69°, fx=698 | hfov=82.4°, fx=548 | 比例  |
| -------------- | ---------------- | ------------------ | ----- |
| camera_info fx | 698              | 548                | 1.27  |
| side_px        | 37               | 29                 | 1.28  |
| PnP Z          | 0.741            | 0.737              | ~1.01 |
| TF Z           | 0.601            | 0.601              | -     |

**关键结论**:
1. **`<horizontal_fov>` 确实控制渲染**: side_px 等比缩放 (1.27 ≈ 698/548)
2. **`<lens>` 不控制渲染**: 同样 hfov 下, 有无 lens / lens 填什么值, side_px 不变
3. **PnP Z 在所有配置下都偏大 ~23%**: gz-sim Fortress Ogre2 渲染引擎 residual projection error

---


结论:
<horizontal_fov> 确实控制渲染——69°→82.4°，side_px 从 37→29，完美等比
<lens> 不控制渲染——实验 2 vs 3 对比，有无 lens 对 side_px 无影响
所有组合 PnP Z 始终 ~23% 偏差——可能是 Ogre2 的非理想 pinhole 投影残余
最终选 hfov=82.4° + lens fx=548，让 camera_info 说实话（和渲染一致）


### 最终配置

| 参数           | 值                           | 原因                        |
| -------------- | ---------------------------- | --------------------------- |
| 传感器类型     | camera + depth_camera (分离) | 规避 rgbd_camera 双配置路径 |
| hfov           | 82.4°                        | 匹配 gz-sim 实际渲染 FOV    |
| vfov           | 52.5°                        | 保证 fx=fy (方像素)         |
| <lens>         | 保留, fx=548                 | camera_info 说实话          |
| camera_info fx | 548.3                        | 和渲染一致                  |
| 残余误差       | ~23% Z offset                | 多帧手眼标定联立求解消除    |

---

## 十一、最终状态 (2026-07-02 — 锁定)

### 当前配置

| 参数           | 值                                     |
| -------------- | -------------------------------------- |
| 传感器         | camera + depth_camera (分离)           |
| hfov           | **82.4°**                              |
| vfov           | **52.5°**                              |
| camera_info fx | **548.3** (自洽)                       |
| `<lens>`       | 保留，fx=548                           |
| 体系文件       | rgbd_split.gazebo.xacro + camera.xacro |

### 残余已知问题

- PnP Z 系统性偏大约 23%（gz-sim Ogre2 残余投影误差）
- 手眼标定多帧联立求解可消除
- visual_servo 不受影响（图像空间闭环）
- 真机部署换 realsense-ros 驱动自动恢复

### 不再折腾


| 参数           | D435 真机        | 当前仿真（修复后）    |
| -------------- | ---------------- | --------------------- |
| fx             | ~698             | 548.3                 |
| fy             | ~698             | 547.5                 |
| cx             | ~480             | 480.0                 |
| cy             | ~270             | 270.0                 |
| horizontal_fov | 69°              | 82.4°                 |
| vertical_fov   | 42.5°            | 52.5°                 |
| 传感器类型     | rgbd_camera	拆分 | camera + depth_camera |


# 基于你的具体配置的完整流程

信息够了。先做几个关键计算，因为它们决定了 marker 尺寸：

## 关键的几何分析

| 量                   | 值                                             |
| -------------------- | ---------------------------------------------- |
| 相机位置（base 系）  | (0.35, 0.5, 0.9)                               |
| 相机姿态             | pitch=58°, yaw=-90° → 朝向 +Y 看向 -Y、向下    |
| target_box 位置      | (0.43, 0.0, 0.10)                              |
| **相机到工作区距离** | $\sqrt{0.08^2 + 0.5^2 + 0.8^2} \approx 0.95$ m |
| **相机视野中心朝向** | 大致看向 (0.35, 0, 0)，距相机约 0.92m          |

D435 默认 RGB 是 1280×720 或 640×480，水平 FOV ≈ 69°。

* 在 1280×720 下，$f_x \approx 928$ 像素
* **marker 在图像里的像素大小** = `marker物理边长 × fx / 距离`

  * 5cm marker @ 0.95m → ~49 像素 ✓ 够用
  * 3cm marker @ 0.95m → ~29 像素 ⚠️ 边缘
  * 2cm marker @ 0.95m → ~20 像素 ✗ 不够

**结论**：

* 世界中的独立测试 marker：**5cm**
* 夹爪上的 marker：**至少 3cm，推荐 4cm**
* 配 640×480 时所有 marker 都要 ×1.4 才稳

## 关键的朝向分析

相机 yaw=-90° + pitch=58°，意味着：

* 相机光轴大致从 (0.35, 0.5, 0.9) 指向 (0.35, 0, 0)
* **光轴在 base 系的方向向量** ≈ (0, -sin58°, -cos58°) = (0, -0.85, -0.53)
* 即从 +Y 方向、向下 58° 俯视

夹爪在抓取时通常 approach 沿 -Z（往下），手指张开方向受目标 yaw 影响。**夹爪上能稳定面向相机的面是 +Y 方向那一面**（朝着相机来的方向）。这是贴 marker 的最佳位置。

---

# 完整流程（重写版）

```
┌─────────────────────────────────────────────────────────────┐
│ 阶段 0 — Gazebo Sim 里引入 ArUco                              │
│   0.1 生成 ArUco PNG (ID 0、1，5cm 测试 + 4cm 夹爪)            │
│   0.2 建 aruco_marker_0 独立模型 (世界中,测试用)              │
│   0.3 修 launch: 资源路径累加 + spawn 测试 marker             │
│   0.4 写 aruco_tester.py 验证检测                            │
│   ✓ 通过标准: 终端持续打出 ID=0 的位姿                        │
├─────────────────────────────────────────────────────────────┤
│ 阶段 1 — 把 marker 焊到夹爪上                                 │
│   1.1 在 robot_gazebo.urdf.xacro 加 aruco_marker_link        │
│       - parent: wrist3_link                                  │
│       - 朝向: 法向沿 +Y_base (面对相机)                       │
│   1.2 用 <gazebo> 标签给 PBR 材质                            │
│   1.3 验证: TF 树里有 aruco_marker_link,Gazebo 里能看到       │
│   ✓ 通过标准: 关节运动时 marker 跟着动且检测稳定              │
├─────────────────────────────────────────────────────────────┤
│ 阶段 2 — 手眼标定 (T_cam^base)                                │
│   2.1 仿真里 T_cam^base 其实从 URDF 已知 (sanity check)       │
│   2.2 写采集脚本: 摆 20-30 个姿态,记录 FK + PnP               │
│   2.3 cv2.calibrateHandEye → 出标定值                        │
│   2.4 对比 URDF 真值 → 量化标定流程的误差                     │
│   ✓ 通过标准: 与真值偏差 < 2mm / 1°                          │
├─────────────────────────────────────────────────────────────┤
│ 阶段 3 — 接入 visual_align_server                            │
│   3.1 marker_pose_node: 订阅 image,发布 T_ee^base_from_aruco │
│   3.2 health_check: 抓取前对比 PnP 与 FK,差异>5mm 报警        │
│   3.3 升级 xy 模式: 真闭环 IBVS (可选,可后做)                 │
│   ✓ 通过标准: 4cm 方块抓取对齐误差 < 5mm                      │
├─────────────────────────────────────────────────────────────┤
│ 阶段 4 — 调优 + 真机迁移准备                                  │
│   4.1 在线外参修正                                            │
│   4.2 多 marker 冗余                                          │
│   4.3 文档化标定步骤,真机上重做即可                           │
└─────────────────────────────────────────────────────────────┘
```

**仿真里特别说明**：T_cam^base 从 URDF 直接可知。所以阶段 2 在仿真里是"流程演练"，不是真正必需。但走一遍流程**非常重要**——真机时这一步是关键。同时它能验证你的 PnP + ArUco 检测链路是否健康。

---

# 阶段 0：详细执行方案

下面是阶段 0 全部可直接执行的内容。

## 0.1 生成 ArUco 图

```bash
cd ~/my_S622/src/gz_launch/models
```

新建生成脚本（一次性，运行完可删）：

```python
# gen_aruco.py
import cv2
import cv2.aruco as aruco
import os

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)

# ID 0: 5cm 测试 marker (放世界里)
# ID 1: 4cm 夹爪 marker
for marker_id in [0, 1]:
    img = aruco.generateImageMarker(aruco_dict, marker_id, 1000)
    bordered = cv2.copyMakeBorder(
        img, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    out_dir = f'aruco_marker_{marker_id}/materials/textures'
    os.makedirs(out_dir, exist_ok=True)
    cv2.imwrite(f'{out_dir}/aruco_id{marker_id}.png', bordered)

print('generated aruco_marker_0/, aruco_marker_1/')
```

```bash
python3 gen_aruco.py
ls aruco_marker_0/materials/textures/    # 应有 aruco_id0.png
```

> OpenCV 4.7 以下用 `aruco.drawMarker(aruco_dict, marker_id, 1000)` 替代。

## 0.2 写 model 文件

`aruco_marker_0/model.config`：

```xml
<?xml version="1.0"?>
<model>
  <name>aruco_marker_0</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>ArUco DICT_5X5_100 ID 0, edge 50mm</description>
</model>
```

`aruco_marker_0/model.sdf`（5cm，仿真不动）：

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="aruco_marker_0">
    <static>true</static>
    <link name="link">
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.05 0.05 0.002</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr>
            <metal>
              <albedo_map>materials/textures/aruco_id0.png</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.05 0.05 0.002</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

阶段 0 暂时只用 ID 0，ID 1 留给阶段 1。但你也可以现在就把 `aruco_marker_1/` 写好（把 5cm 改 4cm、所有 0 改 1），省得回头建。

## 0.3 修改 launch

两处改动：

**改动 A：资源路径累加**

```python
set_model_path = SetEnvironmentVariable(
    "IGN_GAZEBO_RESOURCE_PATH",
    os.path.join(this_pkg, "models")
    + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
)
```

**改动 B：spawn 测试 marker**

在 `spawn_box` 后面加：

```python
# 阶段 0 测试: 在桌面 spawn 一个独立 ArUco
# 位置选在工作区内、不挡 target_box、相机能清楚看到
spawn_aruco_test = TimerAction(
    period=8.0,
    actions=[
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-world", "empty",
                       "-file", os.path.join(this_pkg, "models", "aruco_marker_0", "model.sdf"),
                       "-name", "aruco_test",
                       "-x", "0.30", "-y", "0.15", "-z", "0.02",
                       "-R", "0", "-P", "0", "-Y", "0"],
        )
    ]
)
```

把 `spawn_aruco_test` 加到 `return LaunchDescription([...])` 列表里。

> 位置 (0.30, 0.15, 0.02) 选在 target_box 旁边、桌面上、相机视野中心附近。z=0.02 是为了 marker 厚度 2mm 的中心略高于桌面。如果你的桌面不是 z=0 平面，相应调整。

## 0.4 验证检测节点

放在任意位置，比如 `~/my_S622/src/visual_servo_dev/scripts/aruco_tester.py`：

```python
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
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, params)

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._on_info, 10)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, 10)

        self._log_counter = 0
        self.get_logger().info('aruco_tester ready, waiting for camera_info...')

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
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, 0.05, self.K, self.D)  # 5cm marker
            self._log_counter += 1
            if self._log_counter % 10 == 0:  # 限速 ~3Hz
                for i, mid in enumerate(ids.flatten()):
                    t = tvecs[i].flatten()
                    self.get_logger().info(
                        f'ID={mid}: pos_cam=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m')
            for i in range(len(ids)):
                cv2.drawFrameAxes(img, self.K, self.D, rvecs[i], tvecs[i], 0.03)
            aruco.drawDetectedMarkers(img, corners, ids)

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
```

加可执行权限：

```bash
chmod +x ~/my_S622/src/visual_servo_dev/scripts/aruco_tester.py
```

## 0.5 运行 & 验证

```bash
# 终端 1
cd ~/my_S622 && colcon build --packages-select gz_launch
source install/setup.bash
ros2 launch gz_launch s622_gazebo.launch.py

# 终端 2 (等 Gazebo 起来 + spawn 完成 ~15s)
source install/setup.bash
python3 ~/my_S622/src/visual_servo_dev/scripts/aruco_tester.py
```

**通过标准**：

* Gazebo 窗口里桌面上 (0.30, 0.15) 处出现一个白色小方片，上面有 ArUco 图案
* `aruco_tester` 窗口里 marker 被绿框圈住，有 RGB 三轴
* 终端持续打出 `ID=0: pos_cam=(...)`
* 大致预期：`pos_cam = (+0.35, -0.05, +0.95)` 量级（相机系下，符号取决于你 D435 模型的 optical_frame 约定）

**如果检测不到**：

* Gazebo 看不到图案 → 资源路径问题，查 launch 终端有没有 `Could not find` 警告
* 看到图案但 tester 检测不到 → 多半是 ArUco 字典不一致，或 marker 边界没渲染好
* 看到也检测到但姿态不稳 → 边缘可能模糊，检查 PNG 的清晰度

---

##  等阶段 0 通过后

把以下信息发我：

1. Gazebo 里 marker 渲染截图（确认贴图正确）
2. `aruco_tester` 输出的 `fx`、`pos_cam` 实际数值
3. 你打算把 marker 贴夹爪哪个 link 上？wrist3_link 本体？还是新建一个辅助 link？

然后进入**阶段 1**，把 marker 嵌入夹爪。届时我会基于你的 wrist3_link 几何给出 marker 的 `<origin>` 数值。

---


# 阶段1: 基于你的实际 URDF 的贴 marker 方案

看完 URDF 有几个关键发现，先说结论再解释。

---

## 一、几个关键发现

### 发现 1：没有独立的"夹爪壳"link

夹爪的物理外形是通过 `wrist3_link` 自己的 visual mesh 表现的。fingers 直接挂在 wrist3_link 上，中间没有额外 link。所以 marker 就贴到 **wrist3_link 上**。

### 发现 2：你已经有一个 `calibration_marker`！

```xml
<link name="calibration_marker"/>
<joint name="wrist3_to_calibration_marker" type="fixed">
  <parent link="wrist3_link"/>
  <child link="calibration_marker"/>
  <origin xyz="4.8765E-05 -0.011601 0.1636" rpy="0 0 0"/>
</joint>
```

这个 link 已经在 wrist3_link 的 **-Y 侧**，高度 0.1636（正好在 fingers 挂载高度 0.1658 附近）。这告诉我：

1. 真实机器人上这个位置**大概率有物理特征**（贴纸位置、螺孔、丝印），或者你之前就打算在这做标定
2. **这就是贴 ArUco 的最佳位置**——已经过前人考虑过

### 发现 3：几何关系

```
wrist3_link (原点)
  │
  │ +Z 方向 (工具轴)
  │
  ├─ z=0.1636 → calibration_marker (在 -Y 侧, 0.0116m 偏移)
  ├─ z=0.1658 → finger1、finger2 挂载点
  ├─ z=0.2168 → grasp_frame (TCP)
  ▼
```

wrist3_link 的 +Z 是工具轴（指向 TCP）；X/Y 是横向。fingers 沿 X 方向开合。所以 -Y 侧是**垂直于手指开合方向的一个"侧面"**，不会被手指遮挡。

### 发现 4：相机视角与夹爪姿态匹配

* 相机从 (0.35, 0.5, 0.9) 俯视，主要从 +Y_base 方向
* 抓取时夹爪 +Z_ee 朝下（-Z_base 方向）
* 若抓取 yaw=0，那么 wrist3_link 的 -Y_ee **正好朝向相机方向**（+Y_base）

也就是说，**-Y_ee 侧面刚好面对相机**。而这也正是 `calibration_marker` 所在的位置。设计非常合理。

---

## 二、Marker 应该长这样

在 `calibration_marker` 附近，加一个 `aruco_marker_link`，作为可视化载体（一个 4cm×4cm×2mm 的薄板）：

```
wrist3_link
     │
     │ +Z (指向 TCP,即向下)
     │
     │        ┌──────────┐
     │────────┤ ArUco    │ ← 法向指向 -Y_ee (面朝相机)
     │        │ 4cm×4cm  │
     │        └──────────┘
     │
     ▼
   fingers (再往下 5mm)
```

关键设计：

* **父 link**：wrist3_link（fingers 不动它就不动）
* **位置**：xyz=(0, -0.035, 0.08)——在 -Y 侧偏出 3.5cm（避免和 wrist3_link 本体重叠），高度 0.08m（在 fingers 挂载点下方 8cm，靠近 wrist 一侧，避免被 fingers 遮挡）
* **朝向**：法向沿 -Y_ee → 需要绕 X 轴转 -90°，即 rpy=(-1.5708, 0, 0)

---

## 三、URDF 修改（可直接复制）

在你的 xacro 里，`calibration_marker` 后面插入：

### 澄清一个前提

先说清楚：**wrist3_link 本身就是"夹爪的固定部分"**，不是"机械臂上"。

在这个 URDF 里：

* wrist3_link 是末端法兰 + 夹爪本体的合并 mesh（没有单独的夹爪 link）
* fingers 是它的子 link（会动）
* calibration_marker、grasp_frame 都是 wrist3_link 的 fixed 子 link

从"跟着 TCP 一起动、不被关节污染"的角度看，两个位置都是等价的。

**但是**——`calibration_marker` 有一个特殊含义：

### calibration_marker 是"设计意图"

它出现在 URDF 里，位置精确到 5 位小数（4.8765E-05, -0.011601, 0.1636），这明显不是随手加的。它多半代表：

1. **真实机器人上物理 marker 的粘贴位置**（可能已经贴了，或者留了位置）
2. **CAD 模型里预留的标定基准点**
3. **手眼标定的默认参考点**

在真实机器人上，物理 ArUco 就应该贴在这里。仿真里应该匹配这个位置——**这样从仿真移植到真机时几何完全一致，不用改任何标定**。

我上一版把 marker 挪到 z=0.08、y=-0.035，虽然避开了 fingers 但破坏了这种一致性。**你的思路是对的**。

### 修正方案

**用 calibration_marker 作为父 link**，把 ArUco 直接挂在它下面：

```xml
<!-- ArUco marker visual, mounted at calibration_marker position -->
<link name="aruco_marker_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.04 0.04 0.002"/>
    </geometry>
    <material name="aruco_placeholder">
      <color rgba="1 1 1 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.04 0.04 0.002"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.005"/>
    <inertia ixx="1e-7" iyy="1e-7" izz="1e-7"
             ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<joint name="calibration_to_aruco_marker" type="fixed">
  <parent link="calibration_marker"/>
  <child link="aruco_marker_link"/>
  <!-- 
    calibration_marker 与 wrist3_link 同姿态 (rpy=0)
    要让 marker 法向 (+Z_local) 指向 wrist3_link 的 -Y 方向
    绕 X 转 -90° 即可
    平移 z=-0.001 让 marker 外表面刚好在 calibration_marker 位置上
    (marker 厚度 2mm,中心在 z=-0.001,外表面在 z=0)
  -->
  <origin xyz="0 0 -0.001" rpy="-1.5708 0 0"/>
</joint>

<!-- Gazebo Sim PBR 材质 -->
<gazebo reference="aruco_marker_link">
  <visual>
    <material>
      <ambient>1 1 1 1</ambient>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <pbr>
        <metal>
          <albedo_map>model://aruco_marker_1/materials/textures/aruco_id1.png</albedo_map>
          <metalness>0.0</metalness>
          <roughness>1.0</roughness>
        </metal>
      </pbr>
    </material>
  </visual>
</gazebo>
```

### 关键变化

|            | 上一版                          | 修正版                                       |
| ---------- | ------------------------------- | -------------------------------------------- |
| 父 link    | wrist3_link                     | **calibration_marker**                       |
| 意图       | 我自己算了个避开 fingers 的位置 | 沿用 URDF 设计者预留的位置                   |
| xyz        | (0, -0.035, 0.08)               | (0, 0, -0.001) 相对 calibration_marker       |
| 真机一致性 | 差（需要改真机贴纸位置）        | **完美**（真机上就贴 calibration_marker 处） |

### 关于遮挡

calibration_marker 在 z=0.1636，刚好在 fingers 挂载点 z=0.1658 附近。担心 fingers 遮挡是合理的，但看几何：

* fingers 挂载点在 (±0.01, 0, 0.1658)
* fingers 沿 X 张开，Y 方向厚度小
* marker 法向朝 -Y，相机也从 +Y 侧俯视

**只要 fingers 不在 -Y 方向延伸，就不会遮 marker**。从 URDF 描述看 fingers 是沿 X 张开，不会遮挡 -Y 侧的 marker。

如果 mesh 加载后发现 fingers 真的挡了，再考虑把 origin z 稍微上移，比如：

```xml
<origin xyz="0 0 -0.02" rpy="-1.5708 0 0"/>
```

这样 marker 中心相对 calibration_marker 上移 2cm，即在 wrist3_link 系下 z=0.1436，远离 fingers。**这个微调不影响真机一致性**，因为 calibration_marker 位置是标定基准点，marker 板可以从这里稍偏。

## 更好的做法（可选）

如果你想让 URDF 更规整，也可以**直接给 calibration_marker 加 visual**，不用新建 link：

```xml
<link name="calibration_marker">
  <visual>
    <origin xyz="0 0 -0.001" rpy="-1.5708 0 0"/>
    <geometry>
      <box size="0.04 0.04 0.002"/>
    </geometry>
  </visual>
</link>

<gazebo reference="calibration_marker">
  <visual>
    ...PBR 材质...
  </visual>
</gazebo>
```

这样连新 link 都省了。**推荐这个做法**。缺点是"calibration_marker"这个名字有点误导（既是标定参考点又是 ArUco 载体），但完全可以接受。




---

## 四、rpy 计算的说明

box 的默认法向沿它自己的 **+Z_local** 方向（2mm 那个薄方向）。我们希望法向沿 wrist3_link 的 **-Y** 方向。

绕 X 轴旋转的效果：

* roll = +90°：+Z_local → +Y_parent
* roll = **-90°**：+Z_local → **-Y_parent** ✓

所以 `rpy="-1.5708 0 0"`。

装好后你可以在 RViz 里勾选 aruco_marker_link 的 axes 显示，确认蓝色（Z）箭头是不是指向 wrist3_link 的 -Y 方向。

---

## 五、遮挡分析

Fingers 在 wrist3_link 的 (±0.01, 0, 0.1658) 附近向 +Z 延伸，最长约 5.5cm 到 z=0.22 左右。它们主要沿 X 方向张开，Y 方向厚度小。

marker 在 (0, -0.035, 0.08)：

* **X 方向**：Y 平面上，fingers 的 X 位置 ±0.01，marker 中心 x=0，重合但 marker 在 fingers **下方**（z=0.08 < fingers 起始 0.1658）
* **Y 方向**：marker 在 y=-0.035，fingers 在 y≈0，**marker 探出去 3.5cm**
* 从相机（+Y_base 上方）看下来：fingers 不会遮 marker

**唯一潜在遮挡**：抓取时被 target_box 遮住。但 marker 在 wrist3_link 上，box 在 grasp_frame 附近（z 更远），相机高角度俯视，box 遮不到 marker。除非 box 特别高。

---

## 六、执行步骤

1. 在 `gz_launch/models/aruco_marker_1/materials/textures/` 下放好 `aruco_id1.png`（用阶段 0 的生成脚本，marker_id=1）
2. 修改夹爪 xacro，加上面的 link + joint + gazebo 标签
3. `colcon build && source install/setup.bash`
4. **先只启 RViz 验证**：`ros2 launch gz_launch s622_gazebo.launch.py`，在 RViz 里

   * TF 树里能看到 `aruco_marker_link`
   * Fixed Frame 切到 `wrist3_link`，看到 marker 在 -Y 方向 3.5cm 外
   * 检查 axes：marker 的 Z 轴（蓝）指向 wrist3_link 的 -Y
5. **Gazebo 里验证渲染**：找角度看到 marker 表面，应能看到 ArUco 图案
6. **让机器人走个姿态**：go 到 pre-grasp 附近，跑 `aruco_tester.py`，改成检测 ID 1：

   ```python
   # aruco_tester.py 里 marker 边长改成 0.04
   rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
       corners, 0.04, self.K, self.D)
   ```

   看能不能持续检测到

---

## 七、通过标准

* RViz 里 marker 位置符合预期
* Gazebo 里能看到 ArUco 图案清晰
* pre-grasp 姿态下，`aruco_tester` 持续输出 `ID=1: pos_cam=...`
* 检测到的位置在 Y 方向大致在 -0.3 附近（因为夹爪在 y=0.1，marker 在 -y 侧偏 3.5cm，相机在 y=0.5，Y_cam 是相机右向）——具体数值需要看你相机 optical_frame 约定

跑通了这一步就可以进阶段 2 手眼标定演练。有问题随时说。







# 07/01 22:41 机械臂视觉伺服

> # 阶段 2 上下文交接文档
>
> 复制下面全部内容，作为新对话的第一条消息发给我（或者直接作为 system prompt / context）。
>
> ---
>
> # 项目上下文：机械臂视觉伺服抓取（阶段 2 起点）
>
> ## 项目概况
>
> * **平台**：ROS2 Humble + Gazebo Sim (Ignition Fortress) + MoveIt2
> * **机器人**：法奥（fairino）6轴机械臂 + 二指夹爪
> * **工作空间**：`~/my_S622/`
> * **相机**：Intel RealSense D435（仿真中）
> * **相机安装**：**Eye-to-hand**，固定在 base_link 上方 (0.35, 0.5, 0.9)，pitch=58°、yaw=-90° 俯视工作台
> * **目标物**：4×4×4cm 纯绿色方块，spawn 在 (0.43, 0, 0.10)、yaw=0.52 rad
>
> ## 当前阶段进度
>
> ### 阶段 0（已通过）：Gazebo 里引入 ArUco
>
> * 用 PBR emissive_map 材质解决了灰色渲染问题
> * 使用 DICT_5X5_100 字典
>
> ### 阶段 1（已通过）：夹爪 marker 标定几何 + 相机内参一致性
>
> * ArUco marker（ID=1，4cm）通过 URDF 挂在 wrist3_link 上
> * **位置**：`<origin xyz="0 -0.05 0.15" rpy="-1.5708 0 0"/>`（-Y 方向偏出 5cm 避免被机械臂本体遮挡）
> * **父 link**：wrist3_link（不用 calibration_marker 是为避开圆柱本体遮挡）
> * 无 collision + 无 inertial（避免 MoveIt 规划碰撞误报）
> * **重大 bug 修复**：realsense2_gz_description 的 `rgbd_camera` sensor 存在 hfov/lens intrinsics mismatch bug（渲染 fx ≠ camera_info 声明 fx）。**已弃用 rgbd_camera，改用分离的 `type="camera"` + `type="depth_camera"` 双 sensor**，两者挂同一个 optical_frame 光轴重合
> * **相机内参已正确对齐**：fx=fy=698.4，cx=480，cy=270（960×540，hfov=69°）
> * **PnP 位姿 vs TF 真值**：X/Y 差 < 5mm，Z 差 ~13mm（可接受，主要是 marker 边缘定位误差）
>
> ### 关键文件
>
> * `~/my_S622/src/gz_launch/launch/s622_gazebo.launch.py` — 主 launch
> * `~/my_S622/src/gz_launch/config/robot_gazebo.urdf.xacro` — 集成 URDF
> * `~/my_S622/src/s622_moveit_descriptions/urdf/camera/camera.xacro` — 相机 macro
> * `~/my_S622/src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro` — 自定义分离 sensor 宏（新建）
> * `~/my_S622/src/gz_launch/models/aruco_marker_1/materials/textures/aruco_id1.png` — ArUco 贴图
> * `~/my_S622/src/s622_arm_actions/test/aruco_tester.py` — 验证脚本
> * `~/my_S622/src/s622_arm_actions/` — VisualAlign action server 所在包
>
> ## 夹爪 URDF 链结构
>
> ```
> base_link → ... → j6 → wrist3_link
>                          ├── [fixed, z+0.2168] → grasp_frame  (TCP)
>                          ├── [fixed, xyz=0,-0.0116,0.1636] → calibration_marker (未使用)
>                          ├── [fixed, xyz=0,-0.05,0.15, rpy=-π/2,0,0] → aruco_marker_link (ID=1, 4cm)
>                          ├── finger1_joint → finger1
>                          └── finger2_joint → finger2
> ```
>
> ## 已知问题（暂缓）
>
> 1. **Gazebo 帧率 4.5Hz**（RTF 6-7%）：性能瓶颈，不影响标定，但影响未来 IBVS 闭环。后续要考虑降分辨率
> 2. **降分辨率涉及的改动位置**：
>
>    * `s622_moveit_descriptions/urdf/camera/camera.xacro` 里 `xacro:camera_gazebo_v0` 调用处的 `image_width/image_height`
>    * 会自动传递到 `rgbd_split.gazebo.xacro`
>    * 相机内参 fx/fy 会同步变化（因为从 hfov 自算），代码不用改
> 3. **视觉伺服 xy 模式**（`visual_align_server.py` 里的 `_run_align_xy`）：**实际上不是真 IBVS**，是"接收 base 系 xy 目标 + 笛卡尔 P 控制器"。命名有误导。未来阶段 3 会升级
>
> ## 相机配置速查
>
> * 分辨率：960×540
> * hfov：69°（xacro 里 `${69 * pi/180}`）
> * fx = fy = 698.40 px
> * cx=480, cy=270
> * 光心居中，无畸变（`d = [0,0,0,0,0]`）
> * 深度图 encoding：32FC1（米）
> * 话题：
>
>   * `/camera/color/image_raw`
>   * `/camera/color/camera_info`
>   * `/camera/depth/image_raw`
>
> ## VisualAlign Action Server 现状
>
> `~/my_S622/src/s622_arm_actions/scripts/visual_align_server.py`（或类似路径）
>
> * 5 个模式：descend / lift / retreat / xy / yaw
> * 发布 TwistStamped 到 `/servo_node/delta_twist_cmds`，30Hz
> * 已知代码可改进项（**先记录不动，阶段 3 再改**）：
>
>   * stuck detector 只在 descend 里，xy/yaw 应补
>   * `import tf_transformations` 放循环里
>   * xy 模式的 `target_xy < 0.1` 防护过严，应改成 NaN 判断
>   * `_publish_zero()` 应连发多次防吞
>   * xy 模式的名字有误导，实际不是 IBVS
>
> ## 阶段 2 目标：手眼标定演练
>
> ### 为什么在仿真里做标定
>
> * **真值已知**（URDF 里 T_cam^base 是精确的），可以量化标定流程的误差
> * 走通完整流程，真# 上下文总结（用于新对话的启动提示词）
>
> ---
>
> ## 项目背景
>
> **项目名**：s622 机械臂视觉伺服抓取系统（ROS2 Humble + Gazebo Sim / Ignition Fortress + MoveIt2）
> **目标**：机械臂精确抓取 4×4×4cm 绿色方块。当前问题——夹爪对不齐目标，需引入 ArUco 做视觉伺服/精定位改造。
> **当前阶段**：仿真开发。真机迁移是后续目标。
>
> ## 系统配置
>
> ### 硬件与仿真环境
>
> * **仿真**：Gazebo Sim (Ignition Fortress，Humble 默认版本)，非 Gazebo Classic
> * **性能**：RTF 约 6-7%（Sim time 55s ≈ Real time 10min），电脑性能瓶颈
> * **相机帧率**：4.5Hz（预期 30Hz）——已知性能问题，暂不处理
>
> ### 相机配置
>
> * **型号**：Intel D435（realsense2_description 提供 URDF/mesh）
> * **安装**：eye-to-hand，固定在 base_link 上
>
>   * 位置：`(0.35, 0.5, 0.9)`（base 系）
>   * 姿态：`rpy=(0, 58°, -90°)`（俯视桌面）
> * **图像分辨率**：960×540
> * **相机内参（camera_info）**：
>
>   * `fx = fy = 698.40`
>   * `cx = 480, cy = 270`
>   * 无畸变，hfov = 69°
> * **话题**：
>
>   * `/camera/color/image_raw`
>   * `/camera/color/camera_info`
>   * `/camera/depth/image_raw` (encoding: 32FC1，单位米)
>
> ### 机械臂配置
>
> * **末端链结构**：
>
>   ```
>   base_link → ... → j6 → wrist3_link
>                            ├── [fixed, xyz="0 0 0.2168"] → grasp_frame  (TCP)
>                            ├── [fixed, xyz="4.87e-5 -0.0116 0.1636"] → calibration_marker
>                            ├── finger1_joint → finger1
>                            └── finger2_joint → finger2
>   ```
>
> * **grasp_frame**：TCP，通过 fixed joint 挂 wrist3_link
>
> * **IK/规划器**：fairino_planning 管线 + OMPL 备份
>
> ### 目标物体
>
> * **target_box**：4×4×4cm 纯绿色方块
> * **默认 spawn 位置**：`(0.43, 0.0, 0.10)`，yaw=0.52 rad
>
> ## ArUco 集成状态（已完成，阶段 1 通过）
>
> ### Marker 定义
>
> * **字典**：`DICT_5X5_100`
> * **ID 1**：夹爪 marker，4cm×4cm
> * **ID 0**：世界中独立测试 marker（可选）
>
> ### 夹爪 Marker 挂载
>
> 在 URDF 里定义：
>
> ```xml
> <link name="aruco_marker_link">
>   <visual>
>     <origin xyz="0 0 0" rpy="0 0 0"/>
>     <geometry>
>       <box size="0.04 0.04 0.002"/>
>     </geometry>
>     <material name="aruco_placeholder">
>       <color rgba="1 1 1 1"/>
>     </material>
>   </visual>
>   <!-- 无 collision, 无 inertial (纯可视化 link) -->
> </link>
> 
> <joint name="wrist3_to_aruco_marker" type="fixed">
>   <parent link="wrist3_link"/>
>   <child link="aruco_marker_link"/>
>   <origin xyz="0 -0.05 0.15" rpy="-1.5708 0 0"/>
> </joint>
> ```
>
> **位置理由**：偏离 wrist3_link 圆柱轴 5cm，避免被机械臂本体和 fingers 遮挡。法向朝 -Y_ee（抓取姿态时朝相机）。
>
> ### Gazebo 材质（关键！）
>
> 使用 emissive_map 让 marker 自发光，规避 gz-sim 光照过暗问题：
>
> ```xml
> <gazebo reference="aruco_marker_link">
>   <visual>
>     <material>
>       <ambient>1 1 1 1</ambient>
>       <diffuse>1 1 1 1</diffuse>
>       <specular>0.05 0.05 0.05 1</specular>
>       <emissive>1 1 1 1</emissive>
>       <pbr>
>         <metal>
>           <albedo_map>model://aruco_marker_1/materials/textures/aruco_id1.png</albedo_map>
>           <emissive_map>model://aruco_marker_1/materials/textures/aruco_id1.png</emissive_map>
>           <metalness>0.0</metalness>
>           <roughness>1.0</roughness>
>         </metal>
>       </pbr>
>     </material>
>   </visual>
> </gazebo>
> ```
>
> ### 相机 SDF（关键修复）
>
> **重要背景**：realsense2_gz_description 原生的 `rgbd_camera` 传感器类型存在 hfov/lens 内参 mismatch bug（渲染用一套参数，camera_info 发布另一套，导致 PnP 深度 Z 有 1.27 倍偏差）。
>
> **采用的修复**：拆分成两个独立 sensor（`type="camera"` + `type="depth_camera"`），光轴重合。
>
> 自定义文件：`s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro`
>
> ```xml
> <xacro:macro name="gazebo_rgbd_split" params="name fps gz_topic_name image_width image_height h_fov min_depth max_depth">
>   <!-- RGB sensor -->
>   <gazebo reference="${name}_color_frame">
>     <sensor name="${name}_rgb" type="camera">
>       <topic>${gz_topic_name}/image</topic>
>       <update_rate>${fps}</update_rate>
>       <gz_frame_id>${name}_color_optical_frame</gz_frame_id>
>       <always_on>true</always_on>
>       <camera>
>         <horizontal_fov>${h_fov}</horizontal_fov>
>         <image><width>${image_width}</width><height>${image_height}</height><format>R8G8B8</format></image>
>         <clip><near>0.05</near><far>10</far></clip>
>         <optical_frame_id>${name}_color_optical_frame</optical_frame_id>
>       </camera>
>     </sensor>
>   </gazebo>
>   <!-- Depth sensor (光轴与 RGB 重合) -->
>   <gazebo reference="${name}_color_frame">
>     <sensor name="${name}_depth" type="depth_camera">
>       <topic>${gz_topic_name}/depth_image</topic>
>       <!-- 其他同 RGB,format 改为 R_FLOAT32 -->
>     </sensor>
>   </gazebo>
> </xacro:macro>
> ```
>
> 在 `camera.xacro` 里调用此宏，绕过 realsense2_gz_description 的 rgbd_camera。
>
> ## 阶段 1 通过验证结果
>
> **PnP 与 TF 真值对比**（机器人静止时）：
>
> |      | X      | Y      | Z      |
> | ---- | ------ | ------ | ------ |
> | PnP  | +0.065 | -0.179 | +0.813 |
> | tf   | +0.061 | -0.181 | +0.826 |
> | 偏差 | 4mm ✓  | 2mm ✓  | 13mm ✓ |
>
> * ArUco 检测稳定，marker 6D 位姿准确到 cm 级
> * camera_info 内参和实际渲染一致
> * 深度图正常输出
>
> ## 已知未解决问题（记账）
>
> 1. **相机帧率 4.5Hz**（预期 30Hz），电脑性能瓶颈
>
>    * 后续可考虑降分辨率到 640×360 或 480×270
>    * 需要改动的地方：
>
>      * `camera.xacro` 里 `gazebo_rgbd_split` 的 `image_width` / `image_height` 参数
>      * 其他所有基于图像分辨率的处理（比如 YOLOv8 的 `imgsz` 参数）不用改，camera_info 会自动跟随
>    * **不影响阶段 2 手眼标定**，暂不处理
>
> 2. **PnP Z 轴还有 13mm 偏差**（cm 级别可接受，不阻塞标定演练）
>
> ## 现有关键代码
>
> ### visual_align_server（`s622_arm_actions/visual_align_server.py`）
>
> * Action 服务器，支持 5 模式：descend / lift / retreat / xy / yaw
> * 目前 `xy` 模式是"笛卡尔 goto"而非真闭环 IBVS（用 TF 拿 base 系位置做 P 控制）
> * 通过 `/servo_node/delta_twist_cmds` 发速度指令给 MoveIt Servo
> * 30Hz 控制频率
>
> ### aruco_tester（`s622_arm_actions/test/aruco_tester.py`）
>
> * 订阅 `/camera/color/image_raw` 和 `/camera/color/camera_info`
> * 用 `DICT_5X5_100`、`cv2.aruco.detectMarkers`（老 API）+ `estimatePoseSingleMarkers`
> * 用于验证 ArUco 检测和 PnP 准确性
>
> ## 下一步任务：阶段 2 - 手眼标定演练
>
> ### 目标
>
> 在仿真里跑通完整的手眼标定流程（`cv2.calibrateHandEye`），与 URDF 真值对比，量化标定精度。
>
> ### 需要产出
>
> 1. **数据采集脚本**：自动/半自动摆机械臂到不同姿态（20-30 个），每个姿态同步记录：
>
>    * `T_ee^base`（从 FK / TF：base_link → grasp_frame 或 wrist3_link）
>    * `T_marker^cam`（从 ArUco PnP）
> 2. **标定求解脚本**：调用 `cv2.calibrateHandEye`，试多种算法（Tsai、Park、Daniilidis 等）
> 3. **验证脚本**：
>
>    * 与 URDF 真值对比 T_cam^base
>    * 重投影误差验证
>    * 触点验证（可选）
>
> ### 阶段 2 关键点
>
> * 姿态采样必须**覆盖三个旋转轴充分变化**（每轴 > 30°），否则手眼方程退化
> * 采集时机器人必须**完全静止**，避免 TF 和图像不同步
> * 用 `msg.header.stamp` 做时间同步查询 TF（不用 `rclpy.time.Time()`）
> * 由于是仿真，`T_cam^base` 真值可从 URDF 提取（camera 挂载在 `base_link`，`xyz="0.35 0.5 0.9" rpy="0 58° -90°"`），可用来验证标定精度
>
> ## 用户偏好
>
> * 中文交流
> * 喜欢分阶段推进、每步有验证标准
> * 遇到问题时会主动质疑方案（比如担心风险、深度信息保留等），期望技术方案能经得起推敲
> * 会自己动手改代码、跑验证，不需要手把手
> * 遇到 bug 时深入到根因（比如查了半天 gz-sim rgbd_camera 的内参 mismatch），不满足于"能用就行"
>
> ## 下一对话的建议开场
>
> > "我要进入阶段 2 手眼标定演练。上一轮完成了阶段 1（ArUco 集成到仿真、PnP 与 TF 对齐验证通过）。请给我阶段 2 的详细执行方案，包括数据采集脚本、求解脚本和验证方法。"
>
> ---
>
> 等下我将整个计划发给您。

上下文已接收，关键信息我梳理一下确认理解一致：

**阶段 1 成果**（我理解为已固定的前提）

- ArUco DICT_5X5_100，ID=1（4cm）挂在 wrist3_link，位置 `xyz="0 -0.05 0.15" rpy="-1.5708 0 0"`
- 相机走的是分离 sensor 方案（`camera` + `depth_camera`），已绕开 rgbd_camera 内参 mismatch
- 内参 fx=fy=698.4，cx=480，cy=270，无畸变
- PnP vs TF：X/Y < 5mm，Z ~13mm

**阶段 2 定位**（我理解的核心目标）

- **eye-to-hand** 标定，未知量是 `T_cam^base`（相机在 base 系下的位姿）
- URDF 真值：`T_cam^base` = `(0.35, 0.5, 0.9)`, `rpy=(0, 58°, -90°)`，用于精度评估
- 走通 `cv2.calibrateHandEye` 全流程

**先提前提醒一个坑位**（避免方案里踩到）：
`cv2.calibrateHandEye` 的原生签名是给 **eye-in-hand** 设计的，输出 `T_cam^gripper`。要用它解 eye-to-hand 的 `T_cam^base`，标准做法是把输入里的 `R/t_gripper2base` **换成它们的逆**（即传入 `T_base^gripper`），其它输入不变，输出就直接是 `T_cam^base`。这个细节等看到您的计划时会重点关注是否已经处理。

**另外几个我想在方案里确认的点**（不用现在答，可以体现在您的计划里）：

1. 姿态采样策略：是纯手动 teach、脚本预设 joint 列表、还是围绕一个中心姿态做半径扰动
2. 时间同步：图像 `header.stamp` 查 TF vs 静止后独立采样
3. 采集时 marker 是否需要始终在视野内（可能需要写个 pre-check）
4. 多算法对比是横向列表还是选一个主算法 + 其它做 sanity check

我已就绪，请发计划。

-----

> # 基于你的具体配置的完整流程
>
> 信息够了。先做几个关键计算，因为它们决定了 marker 尺寸：
>
> ## 关键的几何分析
>
> | 量                   | 值                                             |
> | -------------------- | ---------------------------------------------- |
> | 相机位置（base 系）  | (0.35, 0.5, 0.9)                               |
> | 相机姿态             | pitch=58°, yaw=-90° → 朝向 +Y 看向 -Y、向下    |
> | target_box 位置      | (0.43, 0.0, 0.10)                              |
> | **相机到工作区距离** | $\sqrt{0.08^2 + 0.5^2 + 0.8^2} \approx 0.95$ m |
> | **相机视野中心朝向** | 大致看向 (0.35, 0, 0)，距相机约 0.92m          |
>
> D435 默认 RGB 是 1280×720 或 640×480，水平 FOV ≈ 69°。
>
> * 在 1280×720 下，$f_x \approx 928$ 像素
> * **marker 在图像里的像素大小** = `marker物理边长 × fx / 距离`
>
>   * 5cm marker @ 0.95m → ~49 像素 ✓ 够用
>   * 3cm marker @ 0.95m → ~29 像素 ⚠️ 边缘
>   * 2cm marker @ 0.95m → ~20 像素 ✗ 不够
>
> **结论**：
>
> * 世界中的独立测试 marker：**5cm**
> * 夹爪上的 marker：**至少 3cm，推荐 4cm**
> * 配 640×480 时所有 marker 都要 ×1.4 才稳
>
> ## 关键的朝向分析
>
> 相机 yaw=-90° + pitch=58°，意味着：
>
> * 相机光轴大致从 (0.35, 0.5, 0.9) 指向 (0.35, 0, 0)
> * **光轴在 base 系的方向向量** ≈ (0, -sin58°, -cos58°) = (0, -0.85, -0.53)
> * 即从 +Y 方向、向下 58° 俯视
>
> 夹爪在抓取时通常 approach 沿 -Z（往下），手指张开方向受目标 yaw 影响。**夹爪上能稳定面向相机的面是 +Y 方向那一面**（朝着相机来的方向）。这是贴 marker 的最佳位置。
>
> ---
>
> # 完整流程（重写版）
>
> ```
> ┌─────────────────────────────────────────────────────────────┐
> │ 阶段 0 — Gazebo Sim 里引入 ArUco                              │
> │   0.1 生成 ArUco PNG (ID 0、1，5cm 测试 + 4cm 夹爪)            │
> │   0.2 建 aruco_marker_0 独立模型 (世界中,测试用)              │
> │   0.3 修 launch: 资源路径累加 + spawn 测试 marker             │
> │   0.4 写 aruco_tester.py 验证检测                            │
> │   ✓ 通过标准: 终端持续打出 ID=0 的位姿                        │
> ├─────────────────────────────────────────────────────────────┤
> │ 阶段 1 — 把 marker 焊到夹爪上                                 │
> │   1.1 在 robot_gazebo.urdf.xacro 加 aruco_marker_link        │
> │       - parent: wrist3_link                                  │
> │       - 朝向: 法向沿 +Y_base (面对相机)                       │
> │   1.2 用 <gazebo> 标签给 PBR 材质                            │
> │   1.3 验证: TF 树里有 aruco_marker_link,Gazebo 里能看到       │
> │   ✓ 通过标准: 关节运动时 marker 跟着动且检测稳定              │
> ├─────────────────────────────────────────────────────────────┤
> │ 阶段 2 — 手眼标定 (T_cam^base)                                │
> │   2.1 仿真里 T_cam^base 其实从 URDF 已知 (sanity check)       │
> │   2.2 写采集脚本: 摆 20-30 个姿态,记录 FK + PnP               │
> │   2.3 cv2.calibrateHandEye → 出标定值                        │
> │   2.4 对比 URDF 真值 → 量化标定流程的误差                     │
> │   ✓ 通过标准: 与真值偏差 < 2mm / 1°                          │
> ├─────────────────────────────────────────────────────────────┤
> │ 阶段 3 — 接入 visual_align_server                            │
> │   3.1 marker_pose_node: 订阅 image,发布 T_ee^base_from_aruco │
> │   3.2 health_check: 抓取前对比 PnP 与 FK,差异>5mm 报警        │
> │   3.3 升级 xy 模式: 真闭环 IBVS (可选,可后做)                 │
> │   ✓ 通过标准: 4cm 方块抓取对齐误差 < 5mm                      │
> ├─────────────────────────────────────────────────────────────┤
> │ 阶段 4 — 调优 + 真机迁移准备                                  │
> │   4.1 在线外参修正                                            │
> │   4.2 多 marker 冗余                                          │
> │   4.3 文档化标定步骤,真机上重做即可                           │
> └─────────────────────────────────────────────────────────────┘
> ```
>
> **仿真里特别说明**：T_cam^base 从 URDF 直接可知。所以阶段 2 在仿真里是"流程演练"，不是真正必需。但走一遍流程**非常重要**——真机时这一步是关键。同时它能验证你的 PnP + ArUco 检测链路是否健康。
>
> ---
>
> # 阶段 0：详细执行方案
>
> 下面是阶段 0 全部可直接执行的内容。
>
> ## 0.1 生成 ArUco 图
>
> ```bash
> cd ~/my_S622/src/gz_launch/models
> ```
>
> 新建生成脚本（一次性，运行完可删）：
>
> ```python
> # gen_aruco.py
> import cv2
> import cv2.aruco as aruco
> import os
> 
> aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
> 
> # ID 0: 5cm 测试 marker (放世界里)
> # ID 1: 4cm 夹爪 marker
> for marker_id in [0, 1]:
>     img = aruco.generateImageMarker(aruco_dict, marker_id, 1000)
>     bordered = cv2.copyMakeBorder(
>         img, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
>     out_dir = f'aruco_marker_{marker_id}/materials/textures'
>     os.makedirs(out_dir, exist_ok=True)
>     cv2.imwrite(f'{out_dir}/aruco_id{marker_id}.png', bordered)
> 
> print('generated aruco_marker_0/, aruco_marker_1/')
> ```
>
> ```bash
> python3 gen_aruco.py
> ls aruco_marker_0/materials/textures/    # 应有 aruco_id0.png
> ```
>
> > OpenCV 4.7 以下用 `aruco.drawMarker(aruco_dict, marker_id, 1000)` 替代。
>
> ## 0.2 写 model 文件
>
> `aruco_marker_0/model.config`：
>
> ```xml
> <?xml version="1.0"?>
> <model>
>   <name>aruco_marker_0</name>
>   <version>1.0</version>
>   <sdf version="1.9">model.sdf</sdf>
>   <description>ArUco DICT_5X5_100 ID 0, edge 50mm</description>
> </model>
> ```
>
> `aruco_marker_0/model.sdf`（5cm，仿真不动）：
>
> ```xml
> <?xml version="1.0"?>
> <sdf version="1.9">
>   <model name="aruco_marker_0">
>     <static>true</static>
>     <link name="link">
>       <inertial>
>         <mass>0.01</mass>
>         <inertia>
>           <ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
>           <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
>         </inertia>
>       </inertial>
>       <visual name="visual">
>         <geometry>
>           <box>
>             <size>0.05 0.05 0.002</size>
>           </box>
>         </geometry>
>         <material>
>           <ambient>1 1 1 1</ambient>
>           <diffuse>1 1 1 1</diffuse>
>           <specular>0.05 0.05 0.05 1</specular>
>           <pbr>
>             <metal>
>               <albedo_map>materials/textures/aruco_id0.png</albedo_map>
>               <metalness>0.0</metalness>
>               <roughness>1.0</roughness>
>             </metal>
>           </pbr>
>         </material>
>       </visual>
>       <collision name="collision">
>         <geometry>
>           <box>
>             <size>0.05 0.05 0.002</size>
>           </box>
>         </geometry>
>       </collision>
>     </link>
>   </model>
> </sdf>
> ```
>
> 阶段 0 暂时只用 ID 0，ID 1 留给阶段 1。但你也可以现在就把 `aruco_marker_1/` 写好（把 5cm 改 4cm、所有 0 改 1），省得回头建。
>
> ## 0.3 修改 launch
>
> 两处改动：
>
> **改动 A：资源路径累加**
>
> ```python
> set_model_path = SetEnvironmentVariable(
>     "IGN_GAZEBO_RESOURCE_PATH",
>     os.path.join(this_pkg, "models")
>     + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
> )
> ```
>
> **改动 B：spawn 测试 marker**
>
> 在 `spawn_box` 后面加：
>
> ```python
> # 阶段 0 测试: 在桌面 spawn 一个独立 ArUco
> # 位置选在工作区内、不挡 target_box、相机能清楚看到
> spawn_aruco_test = TimerAction(
>     period=8.0,
>     actions=[
>         Node(
>             package="ros_gz_sim",
>             executable="create",
>             arguments=["-world", "empty",
>                        "-file", os.path.join(this_pkg, "models", "aruco_marker_0", "model.sdf"),
>                        "-name", "aruco_test",
>                        "-x", "0.30", "-y", "0.15", "-z", "0.02",
>                        "-R", "0", "-P", "0", "-Y", "0"],
>         )
>     ]
> )
> ```
>
> 把 `spawn_aruco_test` 加到 `return LaunchDescription([...])` 列表里。
>
> > 位置 (0.30, 0.15, 0.02) 选在 target_box 旁边、桌面上、相机视野中心附近。z=0.02 是为了 marker 厚度 2mm 的中心略高于桌面。如果你的桌面不是 z=0 平面，相应调整。
>
> ## 0.4 验证检测节点
>
> 放在任意位置，比如 `~/my_S622/src/visual_servo_dev/scripts/aruco_tester.py`：
>
> ```python
> #!/usr/bin/env python3
> """阶段 0 验证: Gazebo 里能不能检测到 ArUco"""
> import rclpy
> from rclpy.node import Node
> from sensor_msgs.msg import Image, CameraInfo
> from cv_bridge import CvBridge
> import cv2
> import cv2.aruco as aruco
> import numpy as np
> 
> class ArucoTester(Node):
>     def __init__(self):
>         super().__init__('aruco_tester')
>         self.bridge = CvBridge()
>         self.K = None
>         self.D = None
>         aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
>         params = aruco.DetectorParameters()
>         self.detector = aruco.ArucoDetector(aruco_dict, params)
> 
>         self.create_subscription(
>             CameraInfo, '/camera/color/camera_info', self._on_info, 10)
>         self.create_subscription(
>             Image, '/camera/color/image_raw', self._on_image, 10)
> 
>         self._log_counter = 0
>         self.get_logger().info('aruco_tester ready, waiting for camera_info...')
> 
>     def _on_info(self, msg):
>         if self.K is None:
>             self.K = np.array(msg.k).reshape(3, 3)
>             self.D = np.array(msg.d) if len(msg.d) else np.zeros(5)
>             self.get_logger().info(
>                 f'camera ready: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
>                 f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}')
> 
>     def _on_image(self, msg):
>         if self.K is None:
>             return
>         img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
>         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
>         corners, ids, _ = self.detector.detectMarkers(gray)
> 
>         if ids is not None:
>             rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
>                 corners, 0.05, self.K, self.D)  # 5cm marker
>             self._log_counter += 1
>             if self._log_counter % 10 == 0:  # 限速 ~3Hz
>                 for i, mid in enumerate(ids.flatten()):
>                     t = tvecs[i].flatten()
>                     self.get_logger().info(
>                         f'ID={mid}: pos_cam=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m')
>             for i in range(len(ids)):
>                 cv2.drawFrameAxes(img, self.K, self.D, rvecs[i], tvecs[i], 0.03)
>             aruco.drawDetectedMarkers(img, corners, ids)
> 
>         cv2.imshow('aruco_tester', img)
>         cv2.waitKey(1)
> 
> def main():
>     rclpy.init()
>     n = ArucoTester()
>     try:
>         rclpy.spin(n)
>     finally:
>         cv2.destroyAllWindows()
>         n.destroy_node()
>         rclpy.shutdown()
> 
> if __name__ == '__main__':
>     main()
> ```
>
> 加可执行权限：
>
> ```bash
> chmod +x ~/my_S622/src/visual_servo_dev/scripts/aruco_tester.py
> ```
>
> ## 0.5 运行 & 验证
>
> ```bash
> # 终端 1
> cd ~/my_S622 && colcon build --packages-select gz_launch
> source install/setup.bash
> ros2 launch gz_launch s622_gazebo.launch.py
> 
> # 终端 2 (等 Gazebo 起来 + spawn 完成 ~15s)
> source install/setup.bash
> python3 ~/my_S622/src/visual_servo_dev/scripts/aruco_tester.py
> ```
>
> **通过标准**：
>
> * Gazebo 窗口里桌面上 (0.30, 0.15) 处出现一个白色小方片，上面有 ArUco 图案
> * `aruco_tester` 窗口里 marker 被绿框圈住，有 RGB 三轴
> * 终端持续打出 `ID=0: pos_cam=(...)`
> * 大致预期：`pos_cam = (+0.35, -0.05, +0.95)` 量级（相机系下，符号取决于你 D435 模型的 optical_frame 约定）
>
> **如果检测不到**：
>
> * Gazebo 看不到图案 → 资源路径问题，查 launch 终端有没有 `Could not find` 警告
> * 看到图案但 tester 检测不到 → 多半是 ArUco 字典不一致，或 marker 边界没渲染好
> * 看到也检测到但姿态不稳 → 边缘可能模糊，检查 PNG 的清晰度
>
> ---
>
> ##  等阶段 0 通过后
>
> 把以下信息发我：
>
> 1. Gazebo 里 marker 渲染截图（确认贴图正确）
> 2. `aruco_tester` 输出的 `fx`、`pos_cam` 实际数值
> 3. 你打算把 marker 贴夹爪哪个 link 上？wrist3_link 本体？还是新建一个辅助 link？
>
> 然后进入**阶段 1**，把 marker 嵌入夹爪。届时我会基于你的 wrist3_link 几何给出 marker 的 `<origin>` 数值。
>
> ---
>
>
> # 阶段1: 基于你的实际 URDF 的贴 marker 方案
>
> 看完 URDF 有几个关键发现，先说结论再解释。
>
> ---
>
> ## 一、几个关键发现
>
> ### 发现 1：没有独立的"夹爪壳"link
>
> 夹爪的物理外形是通过 `wrist3_link` 自己的 visual mesh 表现的。fingers 直接挂在 wrist3_link 上，中间没有额外 link。所以 marker 就贴到 **wrist3_link 上**。
>
> ### 发现 2：你已经有一个 `calibration_marker`！
>
> ```xml
> <link name="calibration_marker"/>
> <joint name="wrist3_to_calibration_marker" type="fixed">
>   <parent link="wrist3_link"/>
>   <child link="calibration_marker"/>
>   <origin xyz="4.8765E-05 -0.011601 0.1636" rpy="0 0 0"/>
> </joint>
> ```
>
> 这个 link 已经在 wrist3_link 的 **-Y 侧**，高度 0.1636（正好在 fingers 挂载高度 0.1658 附近）。这告诉我：
>
> 1. 真实机器人上这个位置**大概率有物理特征**（贴纸位置、螺孔、丝印），或者你之前就打算在这做标定
> 2. **这就是贴 ArUco 的最佳位置**——已经过前人考虑过
>
> ### 发现 3：几何关系
>
> ```
> wrist3_link (原点)
>   │
>   │ +Z 方向 (工具轴)
>   │
>   ├─ z=0.1636 → calibration_marker (在 -Y 侧, 0.0116m 偏移)
>   ├─ z=0.1658 → finger1、finger2 挂载点
>   ├─ z=0.2168 → grasp_frame (TCP)
>   ▼
> ```
>
> wrist3_link 的 +Z 是工具轴（指向 TCP）；X/Y 是横向。fingers 沿 X 方向开合。所以 -Y 侧是**垂直于手指开合方向的一个"侧面"**，不会被手指遮挡。
>
> ### 发现 4：相机视角与夹爪姿态匹配
>
> * 相机从 (0.35, 0.5, 0.9) 俯视，主要从 +Y_base 方向
> * 抓取时夹爪 +Z_ee 朝下（-Z_base 方向）
> * 若抓取 yaw=0，那么 wrist3_link 的 -Y_ee **正好朝向相机方向**（+Y_base）
>
> 也就是说，**-Y_ee 侧面刚好面对相机**。而这也正是 `calibration_marker` 所在的位置。设计非常合理。
>
> ---
>
> ## 二、Marker 应该长这样
>
> 在 `calibration_marker` 附近，加一个 `aruco_marker_link`，作为可视化载体（一个 4cm×4cm×2mm 的薄板）：
>
> ```
> wrist3_link
>      │
>      │ +Z (指向 TCP,即向下)
>      │
>      │        ┌──────────┐
>      │────────┤ ArUco    │ ← 法向指向 -Y_ee (面朝相机)
>      │        │ 4cm×4cm  │
>      │        └──────────┘
>      │
>      ▼
>    fingers (再往下 5mm)
> ```
>
> 关键设计：
>
> * **父 link**：wrist3_link（fingers 不动它就不动）
> * **位置**：xyz=(0, -0.035, 0.08)——在 -Y 侧偏出 3.5cm（避免和 wrist3_link 本体重叠），高度 0.08m（在 fingers 挂载点下方 8cm，靠近 wrist 一侧，避免被 fingers 遮挡）
> * **朝向**：法向沿 -Y_ee → 需要绕 X 轴转 -90°，即 rpy=(-1.5708, 0, 0)
>
> ---
>
> ## 三、URDF 修改（可直接复制）
>
> 在你的 xacro 里，`calibration_marker` 后面插入：
>
> ### 澄清一个前提
>
> 先说清楚：**wrist3_link 本身就是"夹爪的固定部分"**，不是"机械臂上"。
>
> 在这个 URDF 里：
>
> * wrist3_link 是末端法兰 + 夹爪本体的合并 mesh（没有单独的夹爪 link）
> * fingers 是它的子 link（会动）
> * calibration_marker、grasp_frame 都是 wrist3_link 的 fixed 子 link
>
> 从"跟着 TCP 一起动、不被关节污染"的角度看，两个位置都是等价的。
>
> **但是**——`calibration_marker` 有一个特殊含义：
>
> ### calibration_marker 是"设计意图"
>
> 它出现在 URDF 里，位置精确到 5 位小数（4.8765E-05, -0.011601, 0.1636），这明显不是随手加的。它多半代表：
>
> 1. **真实机器人上物理 marker 的粘贴位置**（可能已经贴了，或者留了位置）
> 2. **CAD 模型里预留的标定基准点**
> 3. **手眼标定的默认参考点**
>
> 在真实机器人上，物理 ArUco 就应该贴在这里。仿真里应该匹配这个位置——**这样从仿真移植到真机时几何完全一致，不用改任何标定**。
>
> 我上一版把 marker 挪到 z=0.08、y=-0.035，虽然避开了 fingers 但破坏了这种一致性。**你的思路是对的**。
>
> ### 修正方案
>
> **用 calibration_marker 作为父 link**，把 ArUco 直接挂在它下面：
>
> ```xml
> <!-- ArUco marker visual, mounted at calibration_marker position -->
> <link name="aruco_marker_link">
>   <visual>
>     <origin xyz="0 0 0" rpy="0 0 0"/>
>     <geometry>
>       <box size="0.04 0.04 0.002"/>
>     </geometry>
>     <material name="aruco_placeholder">
>       <color rgba="1 1 1 1"/>
>     </material>
>   </visual>
>   <collision>
>     <origin xyz="0 0 0" rpy="0 0 0"/>
>     <geometry>
>       <box size="0.04 0.04 0.002"/>
>     </geometry>
>   </collision>
>   <inertial>
>     <mass value="0.005"/>
>     <inertia ixx="1e-7" iyy="1e-7" izz="1e-7"
>              ixy="0" ixz="0" iyz="0"/>
>   </inertial>
> </link>
> 
> <joint name="calibration_to_aruco_marker" type="fixed">
>   <parent link="calibration_marker"/>
>   <child link="aruco_marker_link"/>
>   <!--
>     calibration_marker 与 wrist3_link 同姿态 (rpy=0)
>     要让 marker 法向 (+Z_local) 指向 wrist3_link 的 -Y 方向
>     绕 X 转 -90° 即可
>     平移 z=-0.001 让 marker 外表面刚好在 calibration_marker 位置上
>     (marker 厚度 2mm,中心在 z=-0.001,外表面在 z=0)
>   -->
>   <origin xyz="0 0 -0.001" rpy="-1.5708 0 0"/>
> </joint>
> 
> <!-- Gazebo Sim PBR 材质 -->
> <gazebo reference="aruco_marker_link">
>   <visual>
>     <material>
>       <ambient>1 1 1 1</ambient>
>       <diffuse>1 1 1 1</diffuse>
>       <specular>0.05 0.05 0.05 1</specular>
>       <pbr>
>         <metal>
>           <albedo_map>model://aruco_marker_1/materials/textures/aruco_id1.png</albedo_map>
>           <metalness>0.0</metalness>
>           <roughness>1.0</roughness>
>         </metal>
>       </pbr>
>     </material>
>   </visual>
> </gazebo>
> ```
>
> ### 关键变化
>
> |            | 上一版                          | 修正版                                       |
> | ---------- | ------------------------------- | -------------------------------------------- |
> | 父 link    | wrist3_link                     | **calibration_marker**                       |
> | 意图       | 我自己算了个避开 fingers 的位置 | 沿用 URDF 设计者预留的位置                   |
> | xyz        | (0, -0.035, 0.08)               | (0, 0, -0.001) 相对 calibration_marker       |
> | 真机一致性 | 差（需要改真机贴纸位置）        | **完美**（真机上就贴 calibration_marker 处） |
>
> ### 关于遮挡
>
> calibration_marker 在 z=0.1636，刚好在 fingers 挂载点 z=0.1658 附近。担心 fingers 遮挡是合理的，但看几何：
>
> * fingers 挂载点在 (±0.01, 0, 0.1658)
> * fingers 沿 X 张开，Y 方向厚度小
> * marker 法向朝 -Y，相机也从 +Y 侧俯视
>
> **只要 fingers 不在 -Y 方向延伸，就不会遮 marker**。从 URDF 描述看 fingers 是沿 X 张开，不会遮挡 -Y 侧的 marker。
>
> 如果 mesh 加载后发现 fingers 真的挡了，再考虑把 origin z 稍微上移，比如：
>
> ```xml
> <origin xyz="0 0 -0.02" rpy="-1.5708 0 0"/>
> ```
>
> 这样 marker 中心相对 calibration_marker 上移 2cm，即在 wrist3_link 系下 z=0.1436，远离 fingers。**这个微调不影响真机一致性**，因为 calibration_marker 位置是标定基准点，marker 板可以从这里稍偏。
>
> ## 更好的做法（可选）
>
> 如果你想让 URDF 更规整，也可以**直接给 calibration_marker 加 visual**，不用新建 link：
>
> ```xml
> <link name="calibration_marker">
>   <visual>
>     <origin xyz="0 0 -0.001" rpy="-1.5708 0 0"/>
>     <geometry>
>       <box size="0.04 0.04 0.002"/>
>     </geometry>
>   </visual>
> </link>
> 
> <gazebo reference="calibration_marker">
>   <visual>
>     ...PBR 材质...
>   </visual>
> </gazebo>
> ```
>
> 这样连新 link 都省了。**推荐这个做法**。缺点是"calibration_marker"这个名字有点误导（既是标定参考点又是 ArUco 载体），但完全可以接受。
>
> 
>
>
> ---
>
> ## 四、rpy 计算的说明
>
> box 的默认法向沿它自己的 **+Z_local** 方向（2mm 那个薄方向）。我们希望法向沿 wrist3_link 的 **-Y** 方向。
>
> 绕 X 轴旋转的效果：
>
> * roll = +90°：+Z_local → +Y_parent
> * roll = **-90°**：+Z_local → **-Y_parent** ✓
>
> 所以 `rpy="-1.5708 0 0"`。
>
> 装好后你可以在 RViz 里勾选 aruco_marker_link 的 axes 显示，确认蓝色（Z）箭头是不是指向 wrist3_link 的 -Y 方向。
>
> ---
>
> ## 五、遮挡分析
>
> Fingers 在 wrist3_link 的 (±0.01, 0, 0.1658) 附近向 +Z 延伸，最长约 5.5cm 到 z=0.22 左右。它们主要沿 X 方向张开，Y 方向厚度小。
>
> marker 在 (0, -0.035, 0.08)：
>
> * **X 方向**：Y 平面上，fingers 的 X 位置 ±0.01，marker 中心 x=0，重合但 marker 在 fingers **下方**（z=0.08 < fingers 起始 0.1658）
> * **Y 方向**：marker 在 y=-0.035，fingers 在 y≈0，**marker 探出去 3.5cm**
> * 从相机（+Y_base 上方）看下来：fingers 不会遮 marker
>
> **唯一潜在遮挡**：抓取时被 target_box 遮住。但 marker 在 wrist3_link 上，box 在 grasp_frame 附近（z 更远），相机高角度俯视，box 遮不到 marker。除非 box 特别高。
>
> ---
>
> ## 六、执行步骤
>
> 1. 在 `gz_launch/models/aruco_marker_1/materials/textures/` 下放好 `aruco_id1.png`（用阶段 0 的生成脚本，marker_id=1）
>
> 2. 修改夹爪 xacro，加上面的 link + joint + gazebo 标签
>
> 3. `colcon build && source install/setup.bash`
>
> 4. **先只启 RViz 验证**：`ros2 launch gz_launch s622_gazebo.launch.py`，在 RViz 里
>
>    * TF 树里能看到 `aruco_marker_link`
>    * Fixed Frame 切到 `wrist3_link`，看到 marker 在 -Y 方向 3.5cm 外
>    * 检查 axes：marker 的 Z 轴（蓝）指向 wrist3_link 的 -Y
>
> 5. **Gazebo 里验证渲染**：找角度看到 marker 表面，应能看到 ArUco 图案
>
> 6. **让机器人走个姿态**：go 到 pre-grasp 附近，跑 `aruco_tester.py`，改成检测 ID 1：
>
>    ```python
>    # aruco_tester.py 里 marker 边长改成 0.04
>    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
>        corners, 0.04, self.K, self.D)
>    ```
>
>    看能不能持续检测到
>
> ---
>
> ## 七、通过标准
>
> * RViz 里 marker 位置符合预期
> * Gazebo 里能看到 ArUco 图案清晰
> * pre-grasp 姿态下，`aruco_tester` 持续输出 `ID=1: pos_cam=...`
> * 检测到的位置在 Y 方向大致在 -0.3 附近（因为夹爪在 y=0.1，marker 在 -y 侧偏 3.5cm，相机在 y=0.5，Y_cam 是相机右向）——具体数值需要看你相机 optical_frame 约定
>
> 跑通了这一步就可以进阶段 2 手眼标定演练。有问题随时说。
>
> 虽说后面几个阶段并没有详细流程。

# 阶段 2 详细执行方案：手眼标定演练

## 一、开工前必须先掰清楚的四件事

### 1) `cv2.calibrateHandEye` 是给 eye-in-hand 设计的，eye-to-hand 要"传逆"

OpenCV 的签名假设：相机在 gripper 上、target 在世界。它期望输入 `R/t_gripper2base` + `R/t_target2cam`，输出 `R/t_cam2gripper`。

而我们的场景是反过来：相机在 world、marker 在 gripper 上，未知是 `T_cam^base`。

**做法**：把 API 里的 "gripper" 和 "base" 角色对换。

- 传给 `R_gripper2base` 参数的实际是 `T_base^ee`（即 `T_ee^base` 的逆）
- 传给 `R_target2cam` 参数的是 `T_marker^cam`（照常）
- 输出的 `R/t_cam2gripper` 就直接是我们要的 `T_cam^base`

一行推导：eye-in-hand 里 "gripper"↔"base" 对调后，`T_cam^gripper` 就是 `T_cam^base`。

### 2) `T_cam^base` 里的 cam 是 optical_frame，不是 camera_link

PnP 输出的位姿是在 **optical_frame**（Z 前、X 右、Y 下）下的。而 URDF 里 `rpy=(0, 58°, -90°)` 描述的是 camera_link（REP-103：X 前、Y 左、Z 上）。两者差一个固定旋转 `rpy=(-π/2, 0, -π/2)`。

**别自己乘矩阵，让 TF 帮你算**：直接查 `base_link → camera_color_optical_frame`，得到的就是可比的真值。

### 3) 姿态多样性是标定质量的第一决定因素

`AX=XB` 只有在 **A 的旋转轴至少覆盖两个独立方向** 时才可解。三轴每个都 ±20°~30° 变化最保险。只做纯平移的样本，旋转部分退化，标定必炸。

### 4) 时间同步

图像有 `header.stamp`。查 TF 时必须用**图像的 stamp**去 lookup，别用 `rclpy.time.Time()`（那是取"最新"）。仿真 RTF 6-7%，图像和 TF 时序偏差被放大，用最新会引入几厘米的假误差。

采样时机械臂必须**静止至少 1 秒**再拍。

---

## 二、目录结构

在 `s622_arm_actions/` 下建一个子包，或直接放脚本目录：

```
scripts/handeye/
  ├── collect.py       # 采集
  ├── solve.py         # 求解 + 验证
  ├── poses.py         # 姿态列表
  └── data/            # 保存 .npz
```

---

## 三、姿态列表（`poses.py`）

用中心姿态 + rpy 扰动生成 25 个末端目标。中心姿态选 marker 面向相机、机械臂舒展的构型。

```python
# poses.py
import numpy as np

# 中心姿态: TCP 在 (0.3, 0.15, 0.35), z 轴朝下, yaw 让 -Y_ee 大致朝 +Y_base
# 这样 marker 法向对着相机 (相机在 y=0.5)
CENTER_XYZ = np.array([0.30, 0.15, 0.35])
CENTER_RPY = np.array([np.pi, 0.0, 0.0])  # z 轴朝下 (绕 x 转 180°)

def generate_poses():
    """返回 [(xyz, rpy), ...]，25 个"""
    poses = []
    # 5 个位置 x 5 组姿态
    xy_offsets = [
        (0.0, 0.0), (0.06, 0.0), (-0.06, 0.0),
        (0.0, 0.06), (0.0, -0.06),
    ]
    rpy_offsets = [
        (0, 0, 0),
        (np.deg2rad(25), 0, 0),
        (np.deg2rad(-25), 0, 0),
        (0, np.deg2rad(25), 0),
        (0, np.deg2rad(-25), 0),
        # 兼顾 yaw 变化
        (np.deg2rad(15), np.deg2rad(15), np.deg2rad(20)),
        (np.deg2rad(-15), np.deg2rad(-15), np.deg2rad(-20)),
        (np.deg2rad(20), 0, np.deg2rad(30)),
        (np.deg2rad(-20), 0, np.deg2rad(-30)),
    ]
    for dx, dy in xy_offsets:
        for drx, dry, drz in rpy_offsets[:5]:
            xyz = CENTER_XYZ + np.array([dx, dy, 0.0])
            rpy = CENTER_RPY + np.array([drx, dry, drz])
            poses.append((xyz, rpy))
    # 再加几个大幅 yaw 的
    for drx, dry, drz in rpy_offsets[5:]:
        rpy = CENTER_RPY + np.array([drx, dry, drz])
        poses.append((CENTER_XYZ.copy(), rpy))
    return poses
```

**先跑一次 dry-run**：只调 MoveIt 规划，不采集，把不可达 / marker 出视野的位姿剔掉。目标是留下 20+ 个有效样本。

---

## 四、数据采集脚本（`collect.py`）

我建议**半自动模式**：脚本按列表 goto，每到位后打印状态，人眼确认 marker 可见后按 Enter 触发采集。这样能过滤明显失败的样本，比全自动稳。

```python
#!/usr/bin/env python3
"""手眼标定数据采集: 每个姿态记录 T_ee^base + T_marker^cam"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf_transformations as tft
from poses import generate_poses

MARKER_SIZE = 0.04      # meters
EE_FRAME    = 'grasp_frame'
BASE_FRAME  = 'base_link'


def tf_msg_to_matrix(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    M[:3, 3] = [t.x, t.y, t.z]
    return M


class Collector(Node):
    def __init__(self):
        super().__init__('handeye_collector')
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.last_image = None    # (img, stamp)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._on_info, 10)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._on_image, 10)
        self.samples = []
        self.get_logger().info('collector ready')

    def _on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d) if len(msg.d) else np.zeros(5)

    def _on_image(self, msg):
        self.last_image = (msg, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def detect_marker_pose(self, img, marker_id=1):
        """PnP: 返回 T_marker^cam (4x4), or None"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or marker_id not in ids.flatten():
            return None
        idx = list(ids.flatten()).index(marker_id)
        # 使用 solvePnP + IPPE_SQUARE (比 estimatePoseSingleMarkers 更稳)
        half = MARKER_SIZE / 2.0
        obj = np.array([[-half,  half, 0],
                        [ half,  half, 0],
                        [ half, -half, 0],
                        [-half, -half, 0]], dtype=np.float32)
        img_pts = corners[idx].reshape(-1, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj, img_pts, self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T

    def capture_one(self):
        """采一帧: 返回 (T_ee_base, T_marker_cam) 或 None"""
        if self.K is None or self.last_image is None:
            self.get_logger().warn('camera not ready')
            return None
        img_msg, img = self.last_image
        stamp = img_msg.header.stamp

        # 用图像的 stamp 查 TF
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                BASE_FRAME, EE_FRAME, stamp,
                timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None
        T_ee_base = tf_msg_to_matrix(tf_msg)

        T_marker_cam = self.detect_marker_pose(img)
        if T_marker_cam is None:
            self.get_logger().warn('marker not detected')
            return None

        return T_ee_base, T_marker_cam, img


def move_to_pose(xyz, rpy):
    """占位: 调你的 MoveIt 客户端。返回是否成功。"""
    # 用现成的 moveit_commander 或者你自己包过一层的 client
    # 建议在这里等一个 IsMoving flag 变 False + sleep(1.0) 保证静止
    raise NotImplementedError('接入你的 MoveIt goto API')


def main():
    rclpy.init()
    node = Collector()
    # 等 camera_info + tf 就绪
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.K is not None:
            break

    poses = generate_poses()
    saved = []
    for i, (xyz, rpy) in enumerate(poses):
        print(f'\n[{i+1}/{len(poses)}] moving to xyz={xyz}, rpy={np.rad2deg(rpy)}')
        try:
            ok = move_to_pose(xyz, rpy)
        except NotImplementedError:
            input('  handmove 到目标姿态后按 Enter (or "s" 跳过): ')
            ok = True
        if not ok:
            print('  unreachable, skip')
            continue

        # 静止等待
        import time; time.sleep(1.2)
        rclpy.spin_once(node, timeout_sec=0.3)

        res = node.capture_one()
        if res is None:
            print('  capture failed, skip')
            continue
        T_ee_base, T_marker_cam, img = res
        saved.append({'T_ee_base': T_ee_base, 'T_marker_cam': T_marker_cam})
        print(f'  OK  t_marker_cam = {T_marker_cam[:3,3]}')
        cv2.imwrite(f'data/img_{i:02d}.png', img)

    # 保存
    T_ee_base_arr    = np.stack([s['T_ee_base'] for s in saved])
    T_marker_cam_arr = np.stack([s['T_marker_cam'] for s in saved])
    np.savez('data/handeye.npz',
             T_ee_base=T_ee_base_arr,
             T_marker_cam=T_marker_cam_arr,
             K=node.K, D=node.D)
    print(f'\nsaved {len(saved)} samples to data/handeye.npz')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**几个提醒**：

- `move_to_pose` 留成占位。你可以：
  - 用 `moveit_commander` / `MoveGroupInterface` 接进来
  - 或者一开始就手动模式：脚本只打印目标姿态，你 rviz 里拖 marker + Plan&Execute，回终端按 Enter 采集
- `solvePnP + SOLVEPNP_IPPE_SQUARE` 比 `estimatePoseSingleMarkers` 稳（后者在 OpenCV 4.7+ 已 deprecated）。你阶段 1 的 tester 也建议顺手换掉
- 图像保存下来，标定失败时可以回看是哪个样本坏了

---

## 五、求解 + 验证（`solve.py`）

一份脚本跑完 5 种算法、对比 URDF 真值、算内部一致性。

```python
#!/usr/bin/env python3
"""手眼标定求解 + 验证"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft


def rot_angle_deg(R):
    """从旋转矩阵取角度大小 (度)"""
    c = (np.trace(R) - 1) / 2
    c = np.clip(c, -1.0, 1.0)
    return np.rad2deg(np.arccos(c))


def pose_diff(T1, T2):
    """返回 (位置误差 m, 姿态误差 度)"""
    dT = np.linalg.inv(T1) @ T2
    return np.linalg.norm(dT[:3, 3]), rot_angle_deg(dT[:3, :3])


def calibrate(T_ee_base_list, T_marker_cam_list):
    """eye-to-hand: 输入取逆，输出直接是 T_cam^base"""
    R_b2e, t_b2e = [], []
    for T in T_ee_base_list:
        Ti = np.linalg.inv(T)
        R_b2e.append(Ti[:3, :3])
        t_b2e.append(Ti[:3, 3])
    R_m2c = [T[:3, :3] for T in T_marker_cam_list]
    t_m2c = [T[:3, 3]  for T in T_marker_cam_list]

    methods = {
        'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
        'PARK':       cv2.CALIB_HAND_EYE_PARK,
        'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
        'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
        'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    out = {}
    for name, m in methods.items():
        R, t = cv2.calibrateHandEye(R_b2e, t_b2e, R_m2c, t_m2c, method=m)
        T = np.eye(4); T[:3,:3] = R; T[:3,3] = t.flatten()
        out[name] = T
    return out


def internal_consistency(T_cam_base, T_ee_base_list, T_marker_cam_list):
    """T_marker^ee 应为常数, 算所有样本的方差"""
    T_marker_ee_list = []
    for T_ee_b, T_m_c in zip(T_ee_base_list, T_marker_cam_list):
        T_m_ee = np.linalg.inv(T_ee_b) @ T_cam_base @ T_m_c
        T_marker_ee_list.append(T_m_ee)
    pos = np.array([T[:3, 3] for T in T_marker_ee_list])
    pos_std = pos.std(axis=0) * 1000  # mm
    # 姿态一致性: 每个 T 与均值的 rot 差
    mean_R = T_marker_ee_list[0][:3,:3]  # 取第一个当参考
    rot_diffs = [rot_angle_deg(np.linalg.inv(mean_R) @ T[:3,:3]) for T in T_marker_ee_list]
    return pos_std, np.mean(rot_diffs), np.array([T[:3,3] for T in T_marker_ee_list]).mean(axis=0)


def get_gt_from_tf():
    """从 TF 拿真值: base_link -> camera_color_optical_frame"""
    rclpy.init()
    node = Node('gt_lookup')
    buf = Buffer(); TransformListener(buf, node)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    tf_msg = buf.lookup_transform(
        'base_link', 'camera_color_optical_frame',
        rclpy.time.Time(), timeout=Duration(seconds=2.0))
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    M[:3, 3] = [t.x, t.y, t.z]
    node.destroy_node(); rclpy.shutdown()
    return M


def main():
    d = np.load('data/handeye.npz')
    T_ee_base = d['T_ee_base']
    T_marker_cam = d['T_marker_cam']
    print(f'loaded {len(T_ee_base)} samples')

    # 求解
    results = calibrate(T_ee_base, T_marker_cam)

    # 真值 (仿真专属优势)
    T_gt = get_gt_from_tf()
    print('\n=== T_cam^base 真值 (from TF) ===')
    print(T_gt)

    print(f'\n{"method":<12} {"pos_err[mm]":>12} {"rot_err[°]":>11} '
          f'{"marker_ee std[mm]":>18} {"marker_ee rot std[°]":>21}')
    print('-' * 78)
    for name, T_est in results.items():
        pos_err, rot_err = pose_diff(T_gt, T_est)
        pos_std, rot_std, _ = internal_consistency(T_est, T_ee_base, T_marker_cam)
        print(f'{name:<12} {pos_err*1000:>12.2f} {rot_err:>11.3f} '
              f'{str(np.round(pos_std,2)):>18} {rot_std:>21.3f}')

    # 采用中位/平均
    best_T = results['DANIILIDIS']  # 通常最稳
    np.save('data/T_cam_base_estimated.npy', best_T)
    print(f'\nsaved estimate to T_cam_base_estimated.npy')
    print(best_T)


if __name__ == '__main__':
    main()
```

**验证输出解读**：

- **pos_err / rot_err**：与真值直接比。仿真里这是"金标"。目标 < 5mm / < 1°
- **marker_ee std**：即使没真值，`T_marker^ee` 是常数，它的方差反映样本间的一致性。目标 < 3mm / < 0.5°
- 五种算法结果应该接近。如果某一种明显跑偏，通常是它对噪声更敏感，忽略即可

---

## 六、通过标准

| 指标                                      | 阈值   |
| ----------------------------------------- | ------ |
| `T_cam^base` 位置误差 vs 真值             | < 5 mm |
| `T_cam^base` 姿态误差 vs 真值             | < 1°   |
| `T_marker^ee` 位置方差                    | < 3 mm |
| `T_marker^ee` 姿态方差                    | < 0.5° |
| 至少 3 种算法结果一致 (相互差 < 3mm/0.5°) | ✓      |

达标就可以进阶段 3，把标定结果替换成 `T_cam^base` 的固定 TF broadcaster，接入 visual_align_server。

---

## 七、常见踩坑与调试路径

1. **五个算法全部误差 > 2cm**：几乎必然是姿态多样性不够。看看你的 25 个样本，rpy 是不是主要在一个平面上变
2. **DANIILIDIS 结果好、TSAI 差**：正常，TSAI 对小旋转敏感
3. **误差集中在 Z**：跟阶段 1 的 PnP Z 偏差 13mm 一脉相承，多半是 marker 边缘 subpixel 定位问题
4. **`T_marker^ee` 均值 vs URDF 真值**：URDF 里 marker 在 wrist3_link 系下是 `(0,-0.05,0.15)` + rpy 变换。你可以额外算一步 `T_marker^grasp = T_grasp^ee_URDF · T_marker^wrist3_URDF`，与算出来的均值对比，能诊断出问题在标定还是在 URDF 挂载点

---

准备好就开跑。数据采集脚本里的 `move_to_pose` 那块你想接 MoveIt 客户端还是走手动 teach + Enter，跟我说一声，可以给具体接法。

-----

## 我：
> 决策1： 我有 pymoveit。
> 决策2，
> 位姿改成如下
> CENTER_XYZ = np.array([0.30, 0.15, 0.35])   # 位置不变
> CENTER_RPY = np.array([-2.13, 0.0, 0.0])    # 让 marker 正对相机 (不再是"末端z朝下")|
> 这一位姿之下：
> PnP	TF	误差
> x	0.093	0.082	1.1cm
> y	-0.039	-0.059	2.0cm
> z	0.583	0.602	1.9cm
> 60cm 距离，相对误差 ~3%，手眼标定够用。side_px=37 说明 marker 占图像 ~6%，检测很稳。手眼标定靠的是多个姿态联立求解，单帧在厘米级偏差会被平均掉，不影响最终 T_cam_to_base 的精度。
> 决策3：脚本放现有的 s622_arm_actions/ 下建 scripts/handeye/ 子目录。
> 决策4：环境	OpenCV 版本
> 系统 Python (python3 裸跑)	4.5.4（ROS 2 Humble 自带，无 ArucoDetector）
> conda yolov8	4.13.0（支持新 API）
> 我做的几个决策。

决策收到，四个都合理。下面把方案按你的决策具体化。

## 关于决策 2 的旁注

`rpy=(-2.13, 0, 0)` 是绕 X 转 -122°，末端从"朝上"翻到"朝下 + 前倾 32°"，正好补偿相机 58° pitch，让 marker 平面法向大致对准相机光轴。物理上合理。

**顺便提醒**：扰动 rpy 时，绕 X 的扰动会直接改 marker 与相机的夹角，超过 ±30° 可能让 ArUco 检测掉星。建议在 dry-run 时留意每个姿态的 side_px，掉到 20 以下就丢弃。

## 关于决策 4 的处理

collect.py 跑在系统 Python（要用 rclpy 和 tf2），必须兼容 4.5.4。方案：全脚本走"老 API 优先 + 新 API 回退"路径。**避开** `ArucoDetector`、`DetectorParameters()`（无参构造）等 4.7+ 特性；`solvePnP + IPPE_SQUARE` 在 4.5.4 上是 OK 的（IPPE 从 4.0 就有）。

solve.py 不需要检测 marker，用哪个环境都行。

---

## 代码 1：`aruco_utils.py`（跨版本兼容层）

```python
# scripts/handeye/aruco_utils.py
import cv2
import numpy as np


def make_aruco_detector(dict_id=cv2.aruco.DICT_5X5_100):
    """兼容 OpenCV 4.5.4 (老 API) 与 4.7+ (新 API)"""
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
    try:
        params = cv2.aruco.DetectorParameters()
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()

    # 提高亚像素精度 (对 60cm 距离 side_px=37 场景有用)
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 30
    params.cornerRefinementMinAccuracy = 0.01

    def detect(gray):
        return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    return detect


def solve_marker_pose(corners, marker_size, K, D):
    """solvePnP + IPPE_SQUARE, 返回 4x4 T_marker^cam 或 None"""
    half = marker_size / 2.0
    obj = np.array([[-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0]], dtype=np.float32)
    img_pts = corners.reshape(-1, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        obj, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T
```

## 代码 2：`poses.py`（新中心姿态）

```python
# scripts/handeye/poses.py
import numpy as np

CENTER_XYZ = np.array([0.30, 0.15, 0.35])
CENTER_RPY = np.array([-2.13, 0.0, 0.0])  # marker 正对相机

def generate_poses():
    poses = []
    xy_offsets = [
        (0.0, 0.0),
        (0.06, 0.0), (-0.06, 0.0),
        (0.0, 0.05), (0.0, -0.05),
    ]
    # 扰动范围收敛在 marker 仍能被稳定检测的锥体内
    rpy_offsets = [
        (0.0, 0.0, 0.0),
        (np.deg2rad( 15), 0.0, 0.0),   # 绕 X: 前后仰
        (np.deg2rad(-15), 0.0, 0.0),
        (0.0, np.deg2rad( 20), 0.0),   # 绕 Y: 左右倾
        (0.0, np.deg2rad(-20), 0.0),
    ]
    yaw_offsets = [
        (np.deg2rad( 10), np.deg2rad( 10), np.deg2rad( 25)),
        (np.deg2rad(-10), np.deg2rad(-10), np.deg2rad(-25)),
        (np.deg2rad( 15), 0.0,              np.deg2rad( 30)),
        (np.deg2rad(-15), 0.0,              np.deg2rad(-30)),
    ]
    for dx, dy in xy_offsets:
        for drx, dry, drz in rpy_offsets:
            poses.append((
                CENTER_XYZ + np.array([dx, dy, 0.0]),
                CENTER_RPY + np.array([drx, dry, drz]),
            ))
    for drx, dry, drz in yaw_offsets:
        poses.append((
            CENTER_XYZ.copy(),
            CENTER_RPY + np.array([drx, dry, drz]),
        ))
    return poses  # 5*5 + 4 = 29 个
```

## 代码 3：`collect.py`（pymoveit2 全自动）

```python
#!/usr/bin/env python3
"""手眼标定采集: pymoveit2 全自动版"""
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
import tf_transformations as tft

from pymoveit2 import MoveIt2

from poses import generate_poses
from aruco_utils import make_aruco_detector, solve_marker_pose

MARKER_ID   = 1
MARKER_SIZE = 0.04
EE_FRAME    = 'grasp_frame'
BASE_FRAME  = 'base_link'
SETTLE_SEC  = 1.2

# ==== 需要按你的 MoveIt 配置填 ====
JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']   # 改成你的实际名字
GROUP_NAME  = 'fairino_arm'                          # 改成你的 planning group
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

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_FRAME,
            end_effector_name=EE_FRAME,
            group_name=GROUP_NAME,
            callback_group=self.cb,
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

    def goto(self, xyz, rpy):
        q = tft.quaternion_from_euler(rpy[0], rpy[1], rpy[2])  # xyzw
        self.moveit2.move_to_pose(
            position=list(xyz),
            quat_xyzw=list(q),
            cartesian=False,
            frame_id=BASE_FRAME,
        )
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
```

**你需要改的两处**：

1. `JOINT_NAMES` — 用 `ros2 topic echo /joint_states -n1` 拿到确切名字
2. `GROUP_NAME` — 看你的 SRDF 或者 MoveIt config，通常在 `<group name="...">`

## 代码 4：`solve.py`

跟上一版一样，兼容性没问题（不用 aruco）。这里贴精简版方便你 copy：

```python
#!/usr/bin/env python3
"""手眼标定求解 + 验证"""
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft


def rot_angle_deg(R):
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    return np.rad2deg(np.arccos(c))


def pose_diff(T1, T2):
    dT = np.linalg.inv(T1) @ T2
    return np.linalg.norm(dT[:3, 3]), rot_angle_deg(dT[:3, :3])


def calibrate(T_ee_base_list, T_marker_cam_list):
    R_b2e, t_b2e = [], []
    for T in T_ee_base_list:
        Ti = np.linalg.inv(T)
        R_b2e.append(Ti[:3, :3])
        t_b2e.append(Ti[:3, 3])
    R_m2c = [T[:3, :3] for T in T_marker_cam_list]
    t_m2c = [T[:3, 3]  for T in T_marker_cam_list]

    methods = {
        'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
        'PARK':       cv2.CALIB_HAND_EYE_PARK,
        'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
        'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
        'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    out = {}
    for name, m in methods.items():
        R, t = cv2.calibrateHandEye(R_b2e, t_b2e, R_m2c, t_m2c, method=m)
        T = np.eye(4); T[:3,:3] = R; T[:3,3] = t.flatten()
        out[name] = T
    return out


def internal_consistency(T_cam_base, T_ee_base_list, T_marker_cam_list):
    T_marker_ee_list = []
    for T_eb, T_mc in zip(T_ee_base_list, T_marker_cam_list):
        T_me = np.linalg.inv(T_eb) @ T_cam_base @ T_mc
        T_marker_ee_list.append(T_me)
    pos = np.array([T[:3, 3] for T in T_marker_ee_list])
    pos_std = pos.std(axis=0) * 1000
    ref_R = T_marker_ee_list[0][:3, :3]
    rot_diffs = [rot_angle_deg(np.linalg.inv(ref_R) @ T[:3, :3])
                 for T in T_marker_ee_list]
    return pos_std, float(np.std(rot_diffs)), pos.mean(axis=0)


def get_gt():
    rclpy.init()
    node = Node('gt_lookup')
    buf = Buffer(); TransformListener(buf, node)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    m = buf.lookup_transform('base_link', 'camera_color_optical_frame',
                             rclpy.time.Time(), timeout=Duration(seconds=2.0))
    t = m.transform.translation; q = m.transform.rotation
    M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    M[:3, 3] = [t.x, t.y, t.z]
    node.destroy_node(); rclpy.shutdown()
    return M


def main():
    data = np.load(os.path.join(os.path.dirname(__file__), 'data/handeye.npz'))
    T_eb = data['T_ee_base']; T_mc = data['T_marker_cam']
    print(f'loaded {len(T_eb)} samples')

    results = calibrate(T_eb, T_mc)
    T_gt = get_gt()
    print('\n=== T_cam^base ground truth (from TF) ===')
    print(T_gt.round(4))

    print(f'\n{"method":<12}{"pos[mm]":>10}{"rot[°]":>9}'
          f'{"me_std[mm]":>26}{"me_rot_std[°]":>18}')
    print('-' * 75)
    for name, T_est in results.items():
        pe, re = pose_diff(T_gt, T_est)
        ps, rs, _ = internal_consistency(T_est, T_eb, T_mc)
        print(f'{name:<12}{pe*1000:>10.2f}{re:>9.3f}'
              f'  {np.round(ps,2)!s:>22}{rs:>18.3f}')

    # 保存 DANIILIDIS 结果
    out = os.path.join(os.path.dirname(__file__), 'data/T_cam_base_estimated.npy')
    np.save(out, results['DANIILIDIS'])
    print(f'\nsaved -> {out}')


if __name__ == '__main__':
    main()
```

---

## 执行顺序
2026/7/1 到这里了。  很累啊!
```bash
mkdir -p ~/my_S622/src/s622_arm_actions/scripts/handeye/data
cd ~/my_S622/src/s622_arm_actions/scripts/handeye
# 把 4 个 .py 放进来

# 装 pymoveit2 (如果还没有)
# pip install pymoveit2 或 apt-get install ros-humble-pymoveit2

# 1. 起仿真 + MoveIt (你熟悉)
# 2. 采集
python3 collect.py
# 3. 求解
python3 solve.py
```

---

## 提前告诉我这几件事，避免第一跑就卡

1. `JOINT_NAMES` 和 `GROUP_NAME` 你能直接告诉我吗？我可以在代码里直接填好
2. 你的 pymoveit 具体是 `pymoveit2`（AndrejOrsula 那个）还是别的封装？API 可能有差
3. camera 的 optical_frame 在 TF 里叫什么？我猜是 `camera_color_optical_frame`，如果是别的名字请告诉我，`solve.py` 里查真值要用

-----

> JOINT_NAMES：['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
> GROUP_NAME：'robot_arm'
> pymoveit2：是 AndrejOrsula 的 pymoveit2，你的 visual_servo 和 s622_arm_actions 都在用它
> camera optical_frame：camera_color_optical_frame，tf2_echo 和 aruco_tester 里一直用的就是这个，没错另外， 决策2， 在这一个位姿测试， 会不会导致手眼标定结果不好？

## 先回答你的担心

**不会**。你贴的那张 1-2cm 误差表是"单姿态单帧"的即时误差。手眼标定关心的是完全不同的东西。

### 单帧误差 vs 标定误差的本质区别

手眼方程 `AX = XB` 用的是**多个 (T_ee^base, T_marker^cam) 的联合约束**。它对两种误差反应不一样：

**随机误差（会被平均掉）**：

- marker 边缘亚像素抖动（每帧 ±2-3px，投影到 60cm 距离就是 ±1cm 级）
- 图像噪声、深度歧义

这种误差在 20+ 姿态下会以 √N 速度衰减，1.9cm 单帧误差 → 标定结果里 <3mm。

**系统误差（会全部保留）**：

- marker 物理尺寸填错（你写 4cm 但实际是 3.98cm → 直接 0.5% 深度偏差）
- 相机内参错（你的 fx=698.4 就是从 hfov 反算的，如果 hfov 声明和实际渲染不一致，就全错）
- 姿态多样性不够（rpy 扰动全在一个平面 → AX=XB 退化，标定沿某轴无约束）

### 你的具体情况诊断

看你给的数据：`60cm 距离，误差 (1.1, 2.0, 1.9)cm`。Z 误差比 X/Y 大，这是 IPPE_SQUARE 的典型特征——从平面 marker 求深度天然有歧义，depth 方向不确定性大。这是**随机误差为主**，标定会显著平均掉它。

内参这块你阶段 1 已经修过 rgbd_camera bug 了，是真对齐的，所以系统误差项已经排除了。

**结论**：只要 poses.py 里 rpy 扰动覆盖三个轴（我给的列表就是这样设计的），最终 `T_cam^base` 位置误差应该 < 5mm，姿态误差 < 1°。如果跑出来偏离得远，是姿态多样性问题，不是单帧精度问题。

### 一个 sanity check 方法

跑完 solve.py 后看 5 种算法的一致性。如果 TSAI/PARK/DANIILIDIS 结果互相差 < 3mm，说明数据健康。如果它们互相差 1cm+，就是姿态多样性不够，不是单帧精度问题。

---

## 参数已填 & 代码微调

`collect.py` 里两处已确定：

```python
JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
GROUP_NAME  = 'robot_arm'
```

`solve.py` 里 `get_gt()` 查的就是 `base_link → camera_color_optical_frame`，也对得上。

## 一个 pymoveit2 潜在坑先说好

AndrejOrsula 的 pymoveit2 在不同版本里，`move_to_pose` 的调用方式有两种风格：

**风格 A（老版本，非阻塞 + wait）**：

```python
self.moveit2.move_to_pose(position=..., quat_xyzw=...)
success = self.moveit2.wait_until_executed()
```

**风格 B（较新版本，同步 API）**：

```python
success = self.moveit2.move_to_pose(position=..., quat_xyzw=...,
                                     tolerance_position=0.001,
                                     tolerance_orientation=0.001)
```

你的 visual_servo_dev / s622_arm_actions 里已经在用了，扒一下它是怎么调的。如果和我 collect.py 里 `goto()` 写的不一样，改成一致的就行。

跑第一个姿态前，先加一个 sanity print，确认能到位：

```python
def goto(self, xyz, rpy):
    q = tft.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    self.moveit2.move_to_pose(position=list(xyz), quat_xyzw=list(q))
    ok = self.moveit2.wait_until_executed()
    self.get_logger().info(f'  goto returned: {ok}')
    return ok
```

第一个姿态跑通了再往下 loop。

---

准备好就开跑。第一姿态出结果告诉我 `t_mc` 和 `side_px` 是多少，方便判断链路健康。
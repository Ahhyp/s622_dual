# 阶段五：遇到的问题与解决方案

---

## 1. `yolov8_obb_msgs` 建包后 `ros2 interface show` 找不到

**现象**：默认 C++ 模板的 CMakeLists.txt 不含 `rosidl_generate_interfaces()`，消息类型未生成。

**修复**：
- CMakeLists.txt 加 `rosidl_generate_interfaces()` 块，指定 `.msg` 文件和 DEPENDENCIES
- package.xml 加 `<member_of_group>rosidl_interface_packages</member_of_group>` 和 `<depend>rosidl_default_runtime</depend>`

**教训**：ROS 2 消息包不是标准模板能搞定的，必须手动加这两处。

---

## 2. `ros2 run yolov8_obb` → `PackageNotFoundError`

**现象**：Python 能找到包元数据，但 entry point 脚本报 `PackageNotFoundError: No package metadata was found for yolov8-obb`。

**原因**：没 `source install/setup.bash`，PYTHONPATH 不含 install 路径。

**修复**：`source install/setup.bash` 后正常运行。

**教训**：每次打开新终端都要 source，忘了就白 debug。

---

## 3. 相机桥接话题不全

**现象**：仿真启动后只有 `/camera/image`，缺深度图和 camera_info。

**原因**：
- `ros_gz_bridge` 只桥接了 RGB 图像，没加 depth_image 和 camera_info
- 没有 topic 重映射到 RealSense 风格名称
- `camera_bridge` 定义了但没放进 `LaunchDescription` 返回列表

**修复**：
- arguments 加 `/camera/depth_image` 和 `/camera/camera_info`
- remapping 统一到 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info`
- 把 `camera_bridge` 加入 LaunchDescription

---

## 4. Camera xacro 报 `PI` 未定义 / `M_PI` 未定义

**现象**：`camera_v0` 宏里用了 `${PI}`，然后报 `PI` 未定义。

**原因**：系统 `_d435.urdf.xacro` 没定义全局 `M_PI`。

**修复**：在 `robot_gazebo.urdf.xacro` 顶部加 `<xacro:property name="M_PI" value="3.1415926535897931" />`，然后 camera 的 rpy 里用 `M_PI` 替代 `PI`。

**教训**：xacro property 不是自动装好的，不同宏文件之间 property 不共享。

---

## 5. `ignition-gazebo-sensors-system` 插件缺失

**现象**：相机在 URDF 里定义了但 Gazebo 不渲染。

**原因**：Gazebo 需要 `ignition-gazebo-sensors-system` 系统插件来驱动传感器（含相机）。

**修复**：在 `<gazebo>` 块加：
```xml
<plugin filename="ignition-gazebo-sensors-system"
    name="ignition::gazebo::systems::Sensors">
    <render_engine>ogre2</render_engine>
</plugin>
```

`rgbd_camera.gazebo.xacro` 头部注释明确写了需要这个插件。

---

## 6. `rclpy.time.Time().to_msg()` 是不是 bug？

**结论**：不是。零时间戳在 tf2 里含义是"给我最新可用的变换"，是合法且常用的写法。之前误判为 bug。

---

## 7. 夹爪控制链路不通

**现象**：发送 `Float64` 到 `/gripper_command` 夹爪没反应。

**原因**：
- 夹爪是 `joint_trajectory_controller/JointTrajectoryController`，不是简单的 topic
- `ros2_control` xacro 里缺 `finger1_joint` / `finger2_joint`
- launch 文件没 spawn `hand_controller`
- 仿真 xacro 应统一到 `gz_launch/config/`

**修复**：
1. `s622_gazebo_ros2_control.xacro` 加 finger 关节（position 接口）
2. launch 里 controller_spawner 加 `hand_controller`
3. `robot_gazebo.urdf.xacro` include 指向新 xacro 路径
4. 用正确接口调用：action `/hand_controller/follow_joint_trajectory` 或 topic `/hand_controller/joint_trajectory`
5. 打开 `[0.025, -0.025]`，闭合 `[0.0, 0.0]`

**教训**：position 接口是 feed-forward，不需要 PID 增益；加了 PID 反而会导致不工作。

---

## 8. 夹爪只有一个手指动

**现象**：发 `[0.025, -0.025]` 只有一个手指响应。

**原因**：Gazebo 物理初始化未就绪，仿真刚启动时部分关节还没稳定。

**修复**：等仿真跑一会儿再测，或重发命令，两个都正常响应。

**教训**：`joint_trajectory_controller` 依赖 Gazebo 内部状态，启动初期不稳定。

---

## 9. `tf_transformations` / `transforms3d` 缺失

**现象**：`ros2 run` 报 `ModuleNotFoundError: No module named 'tf_transformations'`，然后 `transforms3d`。

**原因**：这两个包不在 conda 环境里，清华镜像也没有。

**修复**：
```bash
sudo apt install ros-humble-tf-transformations
pip install transforms3d -i https://pypi.org/simple/
```

---

## 10. `trajectory_retime_server` 缺失

**现象**：pymoveit2 import 链报 `ModuleNotFoundError: No module named 'trajectory_retime_server'`。

**原因**：pymoveit2 的 `moveit2.py` 依赖 `trajectory_retime_server.srv.RetimeTrajectory`，这是源项目的 C++ 包。

**修复**：从源项目复制并 colcon 编译：
```bash
cp -r /home/yep/S622_robotarm/src/trajectory_retime_server src/
```

---

## 11. `package.xml` 冗余依赖告警

**现象**：`colcon build` 报 `The generic dependency on 'std_msgs' is redundant with: exec_depend`。

**原因**：`std_msgs` 同时写了 `<depend>` 和 `<exec_depend>`，`<depend>` 已经覆盖了 exec。

**修复**：删掉 `<exec_depend>std_msgs</exec_depend>`。

---

## 12. Camera mesh 没加载

**状态**：未修复，不影响功能。

**原因**：Gazebo 里 D435 3D 模型没渲染出来。话题和 TF 都正常，纯视觉效果。

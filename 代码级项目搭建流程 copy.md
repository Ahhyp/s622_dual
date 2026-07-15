# S622_robotarm 代码级项目搭建流程

> 用途：主动式学习。  
> 目标：按照正常工程开发流程，从零搭建一个类似原 `S622_robotarm` 的 ROS 2 机械臂智能抓取项目。  
> 原则：每一步都必须能 **建包、写代码、编译、运行、验收、记录、提交 Git**。

---

# 0. 这个项目到底是什么

原项目不是一个单一 ROS 2 包，而是一个完整机械臂系统：

```text
厂商模型 / SDK / 通信协议
  ↓
S622 专用 URDF / SRDF / 相机 / 夹爪
  ↓
ros2_control 硬件接口
  ↓
MoveIt2 规划执行
  ↓
Gazebo 仿真
  ↓
自研 planning_core：FK / IK / RRT* / BiRRT*
  ↓
MoveIt2 插件：PlannerManager / IKPlugin / CollisionChecker
  ↓
YOLOv8 / OBB / 手眼标定
  ↓
视觉伺服抓取状态机
  ↓
OctoMap / GraspNet / Web 控制 / 数据监控 / 工作流编排
```

主动学习时，不要只问“这个包是什么”，而要问：

```text
这个包在哪条能力链路上？
它的输入是什么？
输出是什么？
怎么单独运行？
怎么证明它成功？
失败时先查哪里？
```

---

# 1. 推荐总目录结构

按照原项目 26 个包，建议用分层目录管理：

```text
S622_robotarm/
├── README.md
├── docs/
│   ├── daily_learning_log.md
│   ├── package_study_template.md
│   ├── architecture_notes.md
│   └── debug_notes.md
│
└── src/
    ├── vendor/
    │   ├── fairino_description/
    │   ├── fairino_msgs/
    │   ├── fairino_hardware/
    │   ├── pymoveit2/
    │   ├── easy_handeye2/
    │   ├── ros2_aruco/
    │   └── realsense2_gz_description/
    │
    ├── robot_description/
    │   └── s622_moveit_descriptions/
    │
    ├── moveit_config/
    │   ├── fairino3_v6_moveit2_config/
    │   └── s622_moveit_config/
    │
    ├── planning/
    │   ├── fairino_planning_core/
    │   └── fairino_planning_ros/
    │
    ├── simulation/
    │   └── gz_launch/
    │
    ├── perception/
    │   ├── yolov8_obb/
    │   ├── yolov8_obb_msgs/
    │   └── yolov8_grasping/
    │
    ├── calibration/
    │   └── hand_eye_calibration/
    │
    ├── grasping/
    │   ├── visual_servo/
    │   ├── octomap_yolo_grasping/
    │   └── graspnet_grasping/
    │
    ├── tools/
    │   ├── trajectory_retime_server/
    │   ├── control_servers/
    │   ├── data_monitor/
    │   └── panda_arm_msg/
    │
    ├── GraphExecuter/
    │   └── COLCON_IGNORE
    │
    ├── .vscode/
    └── API说明.md
```

说明：

```text
vendor/、planning/、simulation/ 等只是普通目录，不是 ROS 2 包。
真正的 ROS 2 包是里面有 package.xml 的目录。
GraphExecuter 是独立 PySide6 项目，必须放 COLCON_IGNORE。
```

---

# 2. 初始化工程

```bash
mkdir -p S622_robotarm/src
cd S622_robotarm
git init

mkdir -p docs
mkdir -p src/vendor
mkdir -p src/robot_description
mkdir -p src/moveit_config
mkdir -p src/planning
mkdir -p src/simulation
mkdir -p src/perception
mkdir -p src/calibration
mkdir -p src/grasping
mkdir -p src/tools
```

创建 `.gitignore`：

```bash
cat > .gitignore << 'EOF'
build/
install/
log/
*.pyc
__pycache__/
.vscode/ipch/
*.bag
*.db3
*.o
*.a
*.swp
.DS_Store
EOF
```

注意：如果你要把 `libfairino.so` 放进仓库，不要忽略 `*.so`。

创建 README：

```bash
cat > README.md << 'EOF'
# S622 Robotarm

ROS 2 robotic arm system including hardware interface, robot description,
MoveIt2 configuration, planning algorithms, simulation, perception,
calibration, visual servo grasping, Web control, and data monitoring.
EOF
```

空工作区测试：

```bash
colcon build
source install/setup.bash
git add .
git commit -m "init S622 robotarm workspace"
```

验收：

```text
colcon build 成功
source install/setup.bash 成功
Git 初始提交完成
```

---

# 3. 第一阶段：厂商与硬件层 vendor

对应包：

```text
fairino_description
fairino_msgs
fairino_hardware
```

---

## 3.1 fairino_description：多型号 URDF + STL

### 目标

先让 Fairino 多型号机器人能在 RViz 显示。

原结构：

```text
fairino_description/
├── CMakeLists.txt / package.xml
├── urdf/
│   ├── fairino3_v6.urdf
│   ├── fairino5_v6.urdf
│   ├── fairino10_v6.urdf
│   ├── fairino16_v6.urdf
│   ├── fairino20_v6.urdf
│   ├── fairino30_v6.urdf
│   └── fairino3_mt_v6.urdf
├── meshes/
├── launch/display.launch.py
└── rviz/urdf.rviz
```

### 建包

```bash
cd src/vendor

ros2 pkg create fairino_description --build-type ament_cmake
cd fairino_description
mkdir -p urdf meshes launch rviz
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(fairino_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### package.xml 核心依赖

```xml
<package format="3">
  <name>fairino_description</name>
  <version>0.0.1</version>
  <description>Fairino robot description package</description>
  <maintainer email="you@example.com">your_name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### display.launch.py

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="fairino3_v6.urdf"
    )

    urdf_path = PathJoinSubstitution([
        FindPackageShare("fairino_description"),
        "urdf",
        LaunchConfiguration("model")
    ])

    robot_description = ParameterValue(
        Command(["cat ", urdf_path]),
        value_type=str
    )

    return LaunchDescription([
        model_arg,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        )
    ])
```

### 编译运行

```bash
colcon build --packages-select fairino_description
source install/setup.bash
ros2 launch fairino_description display.launch.py model:=fairino3_v6.urdf
```

验收：

```text
RViz 能显示模型
joint_state_publisher_gui 能拖动关节
TF 树完整
STL 路径无报错
```

主动学习任务：

```text
1. 画出 fairino3_v6 的 link/joint 树。
2. 统计 7 个型号各自的 mesh 目录。
3. 修改一个 mesh 路径，观察 RViz 报错。
4. 记录 URDF 中 visual、collision、inertial 的区别。
```

提交：

```bash
git add src/vendor/fairino_description
git commit -m "add fairino description package"
```

---

## 3.2 fairino_msgs：自定义消息与服务

原结构：

```text
fairino_msgs/
├── msg/RobotNonrtState.msg
├── srv/RemoteCmdInterface.srv
└── srv/RemoteScriptContent.srv
```

建包：

```bash
cd src/vendor
ros2 pkg create fairino_msgs --build-type ament_cmake
cd fairino_msgs
mkdir -p msg srv
```

### msg/RobotNonrtState.msg

```text
builtin_interfaces/Time stamp

float64[6] joint_position
float64[6] joint_velocity
float64[6] joint_current
float64[6] tcp_pose

bool connected
bool enabled
bool emergency_stop
bool in_error

int32 robot_mode
int32 gripper_state
bool do0_state
```

### srv/RemoteCmdInterface.srv

```text
string command
string[] args
---
bool success
int32 error_code
string message
```

### srv/RemoteScriptContent.srv

```text
string script_content
---
bool success
int32 error_code
string message
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(fairino_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotNonrtState.msg"
  "srv/RemoteCmdInterface.srv"
  "srv/RemoteScriptContent.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### package.xml 关键点

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<depend>builtin_interfaces</depend>
```

测试：

```bash
colcon build --packages-select fairino_msgs
source install/setup.bash

ros2 interface show fairino_msgs/msg/RobotNonrtState
ros2 interface show fairino_msgs/srv/RemoteCmdInterface
```

验收：

```text
接口生成成功
其他 C++ / Python 包能引用 fairino_msgs
```

主动学习任务：

```text
1. 解释 msg 与 srv 的区别。
2. 给夹爪补一个 SetGripper.srv。
3. 写一个 Python 节点发布 RobotNonrtState 假数据。
```

提交：

```bash
git add src/vendor/fairino_msgs
git commit -m "add fairino custom interfaces"
```

---

## 3.3 fairino_hardware：ros2_control 硬件接口

原结构：

```text
fairino_hardware/
├── include/fairino_hardware/
│   ├── fairino_hardware_interface.hpp
│   ├── command_server.hpp
│   ├── data_type_def.h
│   ├── visibility_control.h
│   └── version_control.h
├── src/
│   ├── fairino_hardware_interface.cpp
│   ├── command_server.cpp
│   └── command_server_node.cpp
├── libfairino/
│   ├── include/robot.h robot_error.h robot_types.h
│   └── lib/libfairino.so.2.2.2 libfairino.so.2.2.5
├── examples/
├── fairino_hardware.xml
└── fairino_remotecmdinterface_para.yaml
```

### 建包

```bash
cd src/vendor

ros2 pkg create fairino_hardware   --build-type ament_cmake   --dependencies rclcpp hardware_interface pluginlib controller_manager fairino_msgs

cd fairino_hardware
mkdir -p include/fairino_hardware src libfairino/include libfairino/lib examples/include examples/src
```

### include/fairino_hardware/fairino_hardware_interface.hpp

```cpp
#pragma once

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace fairino_hardware
{

enum class GripperState
{
  UNKNOWN = 0,
  OPEN = 1,
  CLOSE = 2,
  MOVING = 3,
  ERROR = 4
};

class FairinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  bool connectRobot();
  bool readRobotState();
  bool writeJointCommand();
  void updateGripperState();

private:
  std::string robot_ip_;
  bool connected_{false};

  std::array<double, 6> hw_positions_{};
  std::array<double, 6> hw_velocities_{};
  std::array<double, 6> hw_commands_{};

  double finger1_position_{0.0};
  double finger2_position_{0.0};
  double finger1_command_{0.0};
  double finger2_command_{0.0};

  bool do0_command_{false};
  bool do0_feedback_{false};
  GripperState gripper_state_{GripperState::UNKNOWN};
};

}  // namespace fairino_hardware
```

### src/fairino_hardware_interface.cpp 核心骨架

```cpp
#include "fairino_hardware/fairino_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace fairino_hardware
{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_ip_ = info_.hardware_parameters.count("robot_ip")
    ? info_.hardware_parameters.at("robot_ip")
    : "192.168.58.2";

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;

  if (!connectRobot()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  readRobotState();
  hw_commands_ = hw_positions_;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  connected_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < 6; ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  state_interfaces.emplace_back(
    "finger1",
    hardware_interface::HW_IF_POSITION,
    &finger1_position_);

  state_interfaces.emplace_back(
    "finger2",
    hardware_interface::HW_IF_POSITION,
    &finger2_position_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < 6; ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  command_interfaces.emplace_back(
    "finger1",
    hardware_interface::HW_IF_POSITION,
    &finger1_command_);

  command_interfaces.emplace_back(
    "finger2",
    hardware_interface::HW_IF_POSITION,
    &finger2_command_);

  return command_interfaces;
}

hardware_interface::return_type FairinoHardwareInterface::read(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  (void)time;
  (void)period;

  if (!readRobotState()) {
    return hardware_interface::return_type::ERROR;
  }

  updateGripperState();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  (void)time;
  (void)period;

  if (!writeJointCommand()) {
    return hardware_interface::return_type::ERROR;
  }

  do0_command_ = finger1_command_ > 0.5 || finger2_command_ > 0.5;

  // TODO: 调用 SDK 设置 DO0
  // Robot_SetDO(0, do0_command_);

  return hardware_interface::return_type::OK;
}

bool FairinoHardwareInterface::connectRobot()
{
  // TODO: 调用 libfairino.so 连接机器人
  connected_ = true;
  return true;
}

bool FairinoHardwareInterface::readRobotState()
{
  if (!connected_) {
    return false;
  }

  // TODO:
  // 读取真实关节角、速度、DO0、错误状态。
  // 学习阶段先 mock 数据。

  return true;
}

bool FairinoHardwareInterface::writeJointCommand()
{
  if (!connected_) {
    return false;
  }

  // TODO: 把 hw_commands_ 发送给 SDK。
  return true;
}

void FairinoHardwareInterface::updateGripperState()
{
  if (do0_feedback_) {
    gripper_state_ = GripperState::CLOSE;
    finger1_position_ = 1.0;
    finger2_position_ = 1.0;
  } else {
    gripper_state_ = GripperState::OPEN;
    finger1_position_ = 0.0;
    finger2_position_ = 0.0;
  }
}

}  // namespace fairino_hardware

PLUGINLIB_EXPORT_CLASS(
  fairino_hardware::FairinoHardwareInterface,
  hardware_interface::SystemInterface)
```

### fairino_hardware.xml

```xml
<library path="fairino_hardware">
  <class
    name="fairino_hardware/FairinoHardwareInterface"
    type="fairino_hardware::FairinoHardwareInterface"
    base_class_type="hardware_interface::SystemInterface">
    <description>Fairino ros2_control hardware interface</description>
  </class>
</library>
```

### command_server 作用

`command_server` 用来把厂商远程命令接口包装成 ROS 2 service：

```text
RemoteCmdInterface.srv
RemoteScriptContent.srv
```

最小骨架：

```cpp
#include <rclcpp/rclcpp.hpp>
#include "fairino_msgs/srv/remote_cmd_interface.hpp"

class CommandServer : public rclcpp::Node
{
public:
  CommandServer() : Node("fairino_command_server")
  {
    srv_ = create_service<fairino_msgs::srv::RemoteCmdInterface>(
      "fairino_remote_cmd",
      [this](auto request, auto response) {
        RCLCPP_INFO(get_logger(), "command: %s", request->command.c_str());
        response->success = true;
        response->error_code = 0;
        response->message = "accepted";
      });
  }

private:
  rclcpp::Service<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr srv_;
};
```

### CMakeLists.txt 核心

```cmake
cmake_minimum_required(VERSION 3.8)
project(fairino_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(fairino_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/fairino_hardware_interface.cpp
  src/command_server.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  libfairino/include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  controller_manager
  fairino_msgs
)

pluginlib_export_plugin_description_file(
  hardware_interface
  fairino_hardware.xml
)

add_executable(command_server_node
  src/command_server_node.cpp
)

target_link_libraries(command_server_node ${PROJECT_NAME})

ament_target_dependencies(command_server_node
  rclcpp
  fairino_msgs
)

install(TARGETS ${PROJECT_NAME} command_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY libfairino examples DESTINATION share/${PROJECT_NAME})
install(FILES fairino_hardware.xml fairino_remotecmdinterface_para.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

测试：

```bash
colcon build --packages-select fairino_hardware
source install/setup.bash

ros2 run fairino_hardware command_server_node
ros2 service list | grep fairino
```

验收：

```text
fairino_hardware 编译成功
pluginlib 能加载硬件接口
command_server_node 能启动
finger1/finger2 作为硬件接口导出
DO0 逻辑能控制夹爪/气缸
```

主动学习任务：

```text
1. 画出 ros2_control read/write 调用周期。
2. 给 SDK 封装写 mock 模式。
3. 给 GripperState 做单元测试。
4. 记录 libfairino.so.2.2.2 和 2.2.5 差异。
5. 解释为什么 finger1/finger2 要补进 hardware interface。
```

提交：

```bash
git add src/vendor/fairino_hardware
git commit -m "add fairino hardware interface"
```

---

# 4. 第二阶段：S622 专用机器人模型

对应包：

```text
s622_moveit_descriptions
```

原结构：

```text
s622_moveit_descriptions/
├── urdf/
│   ├── s622_moveit_descriptions.urdf
│   └── camera/_d435.urdf.xacro camera.xacro
├── meshes/
├── visual/
├── config/joint_names_s622_moveit_descriptions.yaml
├── launch/display.launch.py gazebo.launch
└── rviz/urdf.rviz
```

建包：

```bash
cd src/robot_description

ros2 pkg create s622_moveit_descriptions --build-type ament_cmake
cd s622_moveit_descriptions

mkdir -p urdf/camera meshes visual config launch rviz
```

### config/joint_names_s622_moveit_descriptions.yaml

```yaml
controller_joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - finger1
  - finger2
```

### camera.xacro 最小示例

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="s622_camera" params="parent">
    <link name="camera_link"/>

    <joint name="camera_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="camera_link"/>
      <origin xyz="0.05 0.0 0.08" rpy="0 0 0"/>
    </joint>

    <link name="camera_color_optical_frame"/>

    <joint name="camera_color_optical_joint" type="fixed">
      <parent link="camera_link"/>
      <child link="camera_color_optical_frame"/>
      <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
    </joint>
  </xacro:macro>

</robot>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(s622_moveit_descriptions)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes visual config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 测试

```bash
colcon build --packages-select s622_moveit_descriptions
source install/setup.bash
ros2 launch s622_moveit_descriptions display.launch.py
```

验收：

```text
S622 模型显示正常
finger1 / finger2 存在
camera_link 存在
camera_color_optical_frame 存在
TF 树连通
```

主动学习任务：

```text
1. 对比 fairino_description 和 s622_moveit_descriptions。
2. 找出 finger1/finger2 的 parent link。
3. 找出 camera_link 挂在哪个 link 上。
4. 画出 base_link 到 camera_color_optical_frame 的 TF。
```

提交：

```bash
git add src/robot_description/s622_moveit_descriptions
git commit -m "add S622 robot description"
```

---

# 5. 第三阶段：MoveIt2 配置

对应包：

```text
fairino3_v6_moveit2_config
s622_moveit_config
```

MoveIt2 配置包负责：

```text
URDF xacro
SRDF
kinematics.yaml
joint_limits.yaml
ros2_controllers.yaml
moveit_controllers.yaml
ompl_planning.yaml
pilz_cartesian_limits.yaml
sensors_3d.yaml
servo_parameters.yaml
RViz 配置
launch 文件
```

---

## 5.1 s622_moveit_config 推荐结构

```text
s622_moveit_config/
├── config/
│   ├── s622_moveit_descriptions.srdf
│   ├── s622_moveit_descriptions.urdf.xacro
│   ├── s622_moveit_descriptions.ros2_control.xacro
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   ├── initial_positions.yaml
│   ├── ros2_controllers.yaml
│   ├── moveit_controllers.yaml
│   ├── ompl_planning.yaml
│   ├── pilz_cartesian_limits.yaml
│   ├── sensors_3d.yaml
│   ├── servo_parameters.yaml
│   └── moveit.rviz
└── launch/
    ├── demo.launch.py
    ├── move_group.launch.py
    ├── moveit_rviz.launch.py
    ├── rsp.launch.py
    ├── setup_assistant.launch.py
    ├── spawn_controllers.launch.py
    ├── static_virtual_joint_tfs.launch.py
    └── warehouse_db.launch.py
```

建包：

```bash
cd src/moveit_config
ros2 pkg create s622_moveit_config --build-type ament_cmake
cd s622_moveit_config
mkdir -p config launch rviz
```

---

## 5.2 SRDF 核心

```xml
<?xml version="1.0" ?>
<robot name="s622_moveit_descriptions">

  <group name="arm">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>

  <group name="gripper">
    <joint name="finger1"/>
    <joint name="finger2"/>
  </group>

  <end_effector
    name="s622_gripper"
    parent_link="tool0"
    group="gripper"
    parent_group="arm"/>

  <group_state name="home" group="arm">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0"/>
    <joint name="joint3" value="0"/>
    <joint name="joint4" value="0"/>
    <joint name="joint5" value="0"/>
    <joint name="joint6" value="0"/>
  </group_state>

  <virtual_joint
    name="fixed_base"
    type="fixed"
    parent_frame="world"
    child_link="base_link"/>

</robot>
```

---

## 5.3 kinematics.yaml

先用 KDL：

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

后续替换自研 IK：

```yaml
arm:
  kinematics_solver: fairino_planning_ros/FairinoIKPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

---

## 5.4 ros2_controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

gripper_controller:
  ros__parameters:
    joints:
      - finger1
      - finger2
    command_interfaces:
      - position
    state_interfaces:
      - position
```

---

## 5.5 moveit_controllers.yaml

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6

  gripper_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints:
      - finger1
      - finger2
```

---

## 5.6 sensors_3d.yaml

```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/color/points
    max_range: 2.0
    point_subsample: 1
    padding_offset: 0.02
    padding_scale: 1.0
    max_update_rate: 5.0
    filtered_cloud_topic: filtered_cloud
```

---

## 5.7 servo_parameters.yaml

```yaml
moveit_servo:
  ros__parameters:
    move_group_name: arm
    planning_frame: base_link
    ee_frame_name: tool0
    command_in_type: speed_units
    command_out_type: trajectory_msgs/JointTrajectory
    publish_period: 0.01

    scale:
      linear: 0.2
      rotational: 0.4
      joint: 0.5

    incoming_command_timeout: 0.1
    joint_limit_margin: 0.1
```

---

## 5.8 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(s622_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

测试：

```bash
colcon build --packages-select s622_moveit_config
source install/setup.bash
ros2 launch s622_moveit_config demo.launch.py
```

验收：

```text
move_group 能启动
RViz Planning 面板能加载 arm group
Plan 成功
Execute 成功
controller 能 active
```

调试命令：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
ros2 action list | grep follow_joint_trajectory
```

主动学习任务：

```text
1. 解释 URDF 与 SRDF 的区别。
2. 解释 ros2_controllers.yaml 与 moveit_controllers.yaml 的区别。
3. 修改 kinematics.yaml，切换 KDL 与自研 IK。
4. 添加一个 collision object，观察规划变化。
```

提交：

```bash
git add src/moveit_config
git commit -m "add MoveIt2 configuration packages"
```

---

# 6. 第四阶段：自研规划核心 fairino_planning_core

对应包：

```text
fairino_planning_core
```

原结构：

```text
fairino_planning_core/
├── include/fairino_planning_core/
│   ├── types.h
│   ├── dh_kinematics.h
│   ├── ik/fairino_ik.h
│   ├── ik/ik_selector.h
│   ├── algorithms/planning_algorithm.h
│   ├── algorithms/rrt_star.h
│   ├── algorithms/bi_rrt_star.h
│   ├── samplers/mixed_sampler.h
│   ├── collision/collision_interface.h
│   ├── constraints/orientation_checker.h
│   ├── tree/rrt_tree.h
│   └── trajectory/path_shortcut.h
│       trajectory_smoother.h
└── src/
```

建包：

```bash
cd src/planning
ros2 pkg create fairino_planning_core --build-type ament_cmake
cd fairino_planning_core

mkdir -p include/fairino_planning_core/ik
mkdir -p include/fairino_planning_core/algorithms
mkdir -p include/fairino_planning_core/samplers
mkdir -p include/fairino_planning_core/collision
mkdir -p include/fairino_planning_core/constraints
mkdir -p include/fairino_planning_core/tree
mkdir -p include/fairino_planning_core/trajectory

mkdir -p src/ik src/algorithms src/samplers src/constraints src/tree src/trajectory
mkdir -p test examples
```

---

## 6.1 types.h

```cpp
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace fairino_planning_core
{

constexpr int DOF = 6;

using JointVector = Eigen::Matrix<double, DOF, 1>;
using Transform = Eigen::Isometry3d;

struct JointLimit
{
  double lower{0.0};
  double upper{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

struct PlanningRequest
{
  JointVector start;
  JointVector goal;
  std::vector<JointLimit> limits;
  double planning_time_limit{5.0};
};

struct PlanningResult
{
  bool success{false};
  std::vector<JointVector> path;
  double planning_time{0.0};
  std::string message;
};

}  // namespace fairino_planning_core
```

---

## 6.2 dh_kinematics.h

```cpp
#pragma once

#include <array>
#include "fairino_planning_core/types.h"

namespace fairino_planning_core
{

struct DHParam
{
  double a;
  double alpha;
  double d;
  double theta_offset;
};

class DHKinematics
{
public:
  explicit DHKinematics(const std::array<DHParam, DOF>& dh_params);

  Transform forward(const JointVector& q) const;

private:
  Transform dhTransform(double a, double alpha, double d, double theta) const;

private:
  std::array<DHParam, DOF> dh_params_;
};

}  // namespace fairino_planning_core
```

---

## 6.3 collision_interface.h

```cpp
#pragma once

#include "fairino_planning_core/types.h"

namespace fairino_planning_core
{

class CollisionInterface
{
public:
  virtual ~CollisionInterface() = default;

  virtual bool isStateValid(const JointVector& q) const = 0;

  virtual bool isSegmentValid(
    const JointVector& q1,
    const JointVector& q2,
    double resolution) const = 0;
};

}  // namespace fairino_planning_core
```

设计思想：

```text
planning_core 不依赖 MoveIt2。
它只定义抽象碰撞接口。
MoveIt2 具体碰撞检测放到 fairino_planning_ros。
```

---

## 6.4 planning_algorithm.h

```cpp
#pragma once

#include "fairino_planning_core/types.h"
#include "fairino_planning_core/collision/collision_interface.h"

namespace fairino_planning_core
{

class PlanningAlgorithm
{
public:
  virtual ~PlanningAlgorithm() = default;

  virtual PlanningResult plan(
    const PlanningRequest& request,
    const CollisionInterface& collision_checker) = 0;
};

}  // namespace fairino_planning_core
```

---

## 6.5 rrt_tree.h

```cpp
#pragma once

#include <vector>
#include "fairino_planning_core/types.h"

namespace fairino_planning_core
{

struct RRTNode
{
  JointVector q;
  int parent{-1};
  double cost{0.0};
};

class RRTTree
{
public:
  int addNode(const JointVector& q, int parent, double cost);
  int nearest(const JointVector& q) const;
  std::vector<int> near(const JointVector& q, double radius) const;
  std::vector<JointVector> extractPath(int node_index) const;

private:
  std::vector<RRTNode> nodes_;
};

}  // namespace fairino_planning_core
```

---

## 6.6 rrt_star.h

```cpp
#pragma once

#include <random>
#include "fairino_planning_core/algorithms/planning_algorithm.h"
#include "fairino_planning_core/tree/rrt_tree.h"

namespace fairino_planning_core
{

struct RRTStarConfig
{
  int max_iterations{5000};
  double step_size{0.1};
  double goal_threshold{0.05};
  double goal_sample_rate{0.1};
  double rewire_radius{0.3};
};

class RRTStar : public PlanningAlgorithm
{
public:
  explicit RRTStar(const RRTStarConfig& config);

  PlanningResult plan(
    const PlanningRequest& request,
    const CollisionInterface& collision_checker) override;

private:
  JointVector sample(const PlanningRequest& request);
  JointVector steer(const JointVector& from, const JointVector& to) const;
  double distance(const JointVector& a, const JointVector& b) const;

private:
  RRTStarConfig config_;
  std::mt19937 rng_;
};

}  // namespace fairino_planning_core
```

---

## 6.7 bi_rrt_star.h

```cpp
#pragma once

#include "fairino_planning_core/algorithms/planning_algorithm.h"

namespace fairino_planning_core
{

struct BiRRTStarConfig
{
  int max_iterations{5000};
  double step_size{0.1};
  double connect_threshold{0.05};
};

class BiRRTStar : public PlanningAlgorithm
{
public:
  explicit BiRRTStar(const BiRRTStarConfig& config);

  PlanningResult plan(
    const PlanningRequest& request,
    const CollisionInterface& collision_checker) override;

private:
  BiRRTStarConfig config_;
};

}  // namespace fairino_planning_core
```

---

## 6.8 IK 相关文件

`ik/fairino_ik.h`：

```cpp
#pragma once

#include <vector>
#include "fairino_planning_core/types.h"
#include "fairino_planning_core/dh_kinematics.h"

namespace fairino_planning_core
{

class FairinoIK
{
public:
  explicit FairinoIK(const DHKinematics& fk);

  std::vector<JointVector> solve(const Transform& target) const;

private:
  const DHKinematics& fk_;
};

}  // namespace fairino_planning_core
```

`ik/ik_selector.h`：

```cpp
#pragma once

#include <vector>
#include "fairino_planning_core/types.h"

namespace fairino_planning_core
{

class IKSelector
{
public:
  bool selectNearest(
    const std::vector<JointVector>& candidates,
    const JointVector& seed,
    JointVector& selected) const;
};

}  // namespace fairino_planning_core
```

主动学习建议：

```text
第一版：数值 IK。
第二版：解析 IK。
第三版：多解筛选。
第四版：奇异点和 joint limit 处理。
```

---

## 6.9 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(fairino_planning_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/dh_kinematics.cpp
  src/ik/fairino_ik.cpp
  src/ik/ik_selector.cpp
  src/algorithms/rrt_star.cpp
  src/algorithms/bi_rrt_star.cpp
  src/samplers/mixed_sampler.cpp
  src/constraints/orientation_checker.cpp
  src/tree/rrt_tree.cpp
  src/trajectory/path_shortcut.cpp
  src/trajectory/trajectory_smoother.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

ament_package()
```

测试顺序：

```text
1. FK 测试：q -> T
2. IK 测试：q -> FK -> IK -> FK
3. RRT* 测试：无障碍 start -> goal
4. BiRRT* 测试：比较规划速度
5. Shortcut 测试：路径点减少但仍无碰撞
```

验收：

```text
不依赖 ROS 运行时
C++17 编译通过
FK / IK / RRT* / BiRRT* 有独立测试
path_shortcut.h 无错误语法
```

主动学习任务：

```text
1. 手写 DH 正运动学推导。
2. 写 test_fk.cpp。
3. 写 test_ik.cpp。
4. 画出 RRT 树增长过程。
5. 比较 RRT* 与 BiRRT* 的耗时和路径长度。
```

提交：

```bash
git add src/planning/fairino_planning_core
git commit -m "add fairino planning core"
```

---

# 7. 第五阶段：fairino_planning_ros 插件封装

对应包：

```text
fairino_planning_ros
```

原结构：

```text
fairino_planning_ros/
├── include/fairino_planning_ros/
│   ├── fairino_planner_manager.h
│   ├── fairino_ik_plugin.h
│   └── moveit_collision_checker.h
├── src/
│   ├── fairino_planner_manager.cpp
│   ├── fairino_ik_plugin.cpp
│   ├── moveit_collision_checker.cpp
│   └── standalone_planner_node.cpp
├── config/ik_params.yaml planning_params.yaml
├── launch/demo.launch.py planning_pipeline.launch.py
└── plugins/fairino_planning_plugins.xml
```

建包：

```bash
cd src/planning

ros2 pkg create fairino_planning_ros   --build-type ament_cmake   --dependencies rclcpp pluginlib moveit_core moveit_ros_planning fairino_planning_core

cd fairino_planning_ros
mkdir -p include/fairino_planning_ros src config launch plugins
```

---

## 7.1 moveit_collision_checker.h

```cpp
#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include "fairino_planning_core/collision/collision_interface.h"

namespace fairino_planning_ros
{

class MoveItCollisionChecker
  : public fairino_planning_core::CollisionInterface
{
public:
  MoveItCollisionChecker(
    planning_scene::PlanningSceneConstPtr planning_scene,
    const moveit::core::JointModelGroup* joint_model_group);

  bool isStateValid(
    const fairino_planning_core::JointVector& q) const override;

  bool isSegmentValid(
    const fairino_planning_core::JointVector& q1,
    const fairino_planning_core::JointVector& q2,
    double resolution) const override;

private:
  planning_scene::PlanningSceneConstPtr planning_scene_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

}
```

---

## 7.2 fairino_planner_manager.h

```cpp
#pragma once

#include <moveit/planning_interface/planning_interface.h>

namespace fairino_planning_ros
{

class FairinoPlannerManager
  : public planning_interface::PlannerManager
{
public:
  bool initialize(
    const moveit::core::RobotModelConstPtr& model,
    const rclcpp::Node::SharedPtr& node,
    const std::string& parameter_namespace) override;

  bool canServiceRequest(
    const moveit_msgs::msg::MotionPlanRequest& req) const override;

  std::string getDescription() const override;

  void getPlanningAlgorithms(
    std::vector<std::string>& algs) const override;

  planning_interface::PlanningContextPtr getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
};

}
```

---

## 7.3 fairino_ik_plugin.h

```cpp
#pragma once

#include <moveit/kinematics_base/kinematics_base.h>

namespace fairino_planning_ros
{

class FairinoIKPlugin : public kinematics::KinematicsBase
{
public:
  bool initialize(
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name,
    const std::string& base_frame,
    const std::vector<std::string>& tip_frames,
    double search_discretization) override;

  bool getPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options =
      kinematics::KinematicsQueryOptions()) const override;
};

}
```

---

## 7.4 fairino_planning_plugins.xml

```xml
<library path="fairino_planning_ros">
  <class
    name="fairino_planning_ros/FairinoPlannerManager"
    type="fairino_planning_ros::FairinoPlannerManager"
    base_class_type="planning_interface::PlannerManager">
    <description>Fairino custom MoveIt2 planner manager</description>
  </class>

  <class
    name="fairino_planning_ros/FairinoIKPlugin"
    type="fairino_planning_ros::FairinoIKPlugin"
    base_class_type="kinematics::KinematicsBase">
    <description>Fairino custom IK plugin</description>
  </class>
</library>
```

---

## 7.5 planning_params.yaml

```yaml
fairino_planning:
  planner_id: BiRRTstar
  max_iterations: 5000
  step_size: 0.1
  goal_threshold: 0.05
  shortcut_iterations: 100
  collision_check_resolution: 0.02
```

## 7.6 ik_params.yaml

```yaml
fairino_ik:
  position_tolerance: 0.001
  orientation_tolerance: 0.01
  max_solutions: 8
  select_nearest_solution: true
```

测试：

```bash
colcon build --packages-select fairino_planning_core fairino_planning_ros
source install/setup.bash
ros2 run fairino_planning_ros standalone_planner_node
```

验收：

```text
pluginlib 能加载 FairinoPlannerManager
pluginlib 能加载 FairinoIKPlugin
standalone_planner_node 能运行
MoveIt2 点击 Plan 时能调用自研规划器
```

主动学习任务：

```text
1. 在 IK 插件中打印目标 pose。
2. 在 PlannerManager 中打印 start/goal joint。
3. 比较 OMPL 和自研 BiRRTstar。
4. 故意让 IK 失败，观察 MoveIt2 报错链。
```

提交：

```bash
git add src/planning/fairino_planning_ros
git commit -m "add fairino MoveIt2 planning plugins"
```

---

# 8. 第六阶段：Gazebo 仿真 gz_launch

原结构：

```text
gz_launch/
├── config/
│   ├── robot_gazebo.urdf.xacro
│   ├── robot_gazebo.friction.urdf.xacro
│   ├── controller_setting.yaml
│   ├── fairino_planning.yaml / movelt_cpp.yaml
│   ├── box.urdf case.urdf obstacle.urdf cube.sdf
├── launch/
│   ├── gazebo.launch.py
│   ├── gazebo_yolo.launch.py
│   ├── pick_block.launch.py
│   ├── servo_yolo_grasping_gz.launch.py
│   ├── pen_box_system.launch.py
│   ├── yolo_pick.launch.py
│   ├── yolo_detector.launch.py
│   └── stopmotion.launch.py
├── scripts/
│   ├── demo_pathplanning_node.py
│   ├── cube_controller_node.py
│   ├── yolo_Kalman_detector_obb_node.py
│   ├── yolo_detector_obb_node.py
│   ├── pen_box_grasping_node.py
│   ├── pick_drop_node.py / pick_drop_ik_node.py
│   ├── robot_control_from_UI_node.py
│   └── stopmotion_node.py
├── worlds/
└── rviz/
```

建包：

```bash
cd src/simulation

ros2 pkg create gz_launch   --build-type ament_python   --dependencies rclpy geometry_msgs sensor_msgs trajectory_msgs std_msgs

cd gz_launch
mkdir -p config launch scripts worlds rviz
```

---

## 8.1 controller_setting.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

gripper_controller:
  ros__parameters:
    joints:
      - finger1
      - finger2
    command_interfaces:
      - position
    state_interfaces:
      - position
```

---

## 8.2 demo_pathplanning_node.py

```python
import rclpy
from rclpy.node import Node


class DemoPathPlanningNode(Node):
    def __init__(self):
        super().__init__("demo_pathplanning_node")
        self.get_logger().info("Demo path planning node started")

    def run(self):
        # TODO:
        # 1. 设置目标位姿
        # 2. 调用 pymoveit2 或 MoveIt action
        # 3. 执行轨迹
        self.get_logger().info("TODO: call MoveIt2 planning")


def main():
    rclpy.init()
    node = DemoPathPlanningNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## 8.3 pick_drop_node.py

```python
import rclpy
from rclpy.node import Node


class PickDropNode(Node):
    def __init__(self):
        super().__init__("pick_drop_node")

    def move_to_pregrasp(self):
        pass

    def descend(self):
        pass

    def close_gripper(self):
        pass

    def lift(self):
        pass

    def move_to_place(self):
        pass

    def open_gripper(self):
        pass

    def run(self):
        self.move_to_pregrasp()
        self.descend()
        self.close_gripper()
        self.lift()
        self.move_to_place()
        self.open_gripper()


def main():
    rclpy.init()
    node = PickDropNode()
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

开发顺序：

```text
1. gazebo.launch.py 打开空 world
2. 加载 arm_on_the_table.sdf
3. spawn 机器人
4. joint_state_broadcaster active
5. arm_controller active
6. MoveIt2 控制 Gazebo 机器人
7. 加 cube.sdf
8. pick_block.launch.py 完成固定抓取
9. gazebo_yolo.launch.py 加入视觉检测
10. servo_yolo_grasping_gz.launch.py 完成视觉伺服仿真
```

测试：

```bash
colcon build --packages-select gz_launch
source install/setup.bash

ros2 launch gz_launch gazebo.launch.py
ros2 launch gz_launch pick_block.launch.py
ros2 launch gz_launch servo_yolo_grasping_gz.launch.py
```

验收：

```text
Gazebo 能打开 world
机器人能 spawn
controller active
MoveIt2 能控制仿真机器人
cube / box / case / obstacle 能显示
仿真抓取 demo 能运行
```

主动学习任务：

```text
1. 解释 Gazebo、RViz、MoveIt2 的关系。
2. 在 world 里添加一个障碍物。
3. 把障碍物同步到 PlanningScene。
4. 检查 movelt_cpp.yaml 是否为 moveit_cpp.yaml 拼写问题。
```

提交：

```bash
git add src/simulation/gz_launch
git commit -m "add Gazebo simulation package"
```

---

# 9. 第七阶段：感知与标定

对应包：

```text
yolov8_obb_msgs
yolov8_obb
yolov8_grasping
hand_eye_calibration
```

---

## 9.1 yolov8_obb_msgs

建包：

```bash
cd src/perception
ros2 pkg create yolov8_obb_msgs --build-type ament_cmake
cd yolov8_obb_msgs
mkdir -p msg
```

### msg/InferenceResult.msg

```text
string class_name
float32 confidence
float32 center_x
float32 center_y
float32 width
float32 height
float32 angle
```

### msg/Yolov8Inference.msg

```text
std_msgs/Header header
InferenceResult[] results
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(yolov8_obb_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

测试：

```bash
colcon build --packages-select yolov8_obb_msgs
source install/setup.bash
ros2 interface show yolov8_obb_msgs/msg/Yolov8Inference
```

注意：

```text
原项目说明 package.xml 中名为 yolov8_msgs。
学习时必须检查 package.xml 的 <name> 与文件夹名是否一致。
ROS 2 真实包名以 package.xml 为准。
```

---

## 9.2 yolov8_obb

原结构：

```text
yolov8_obb/
├── scripts/yolov8_obb_publisher.py yolov8_obb_subscriber.py best.pt
└── launch/yolov8_obb.launch.py
```

建包：

```bash
cd src/perception

ros2 pkg create yolov8_obb   --build-type ament_python   --dependencies rclpy sensor_msgs std_msgs yolov8_obb_msgs
```

### yolov8_obb_publisher.py

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov8_obb_msgs.msg import Yolov8Inference


class YoloV8OBBPublisher(Node):
    def __init__(self):
        super().__init__("yolov8_obb_publisher")

        self.sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Yolov8Inference,
            "/yolov8_obb/inference",
            10
        )

    def image_callback(self, msg):
        output = Yolov8Inference()
        output.header = msg.header

        # TODO:
        # ROS Image -> OpenCV
        # YOLOv8 OBB inference
        # Fill results

        self.pub.publish(output)


def main():
    rclpy.init()
    node = YoloV8OBBPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

验收：

```text
能订阅图像
能发布 /yolov8_obb/inference
OBB 包含 center_x center_y width height angle
```

主动学习任务：

```text
1. 比较 bbox 和 OBB。
2. 用 best.pt 离线推理一张图。
3. 记录 FPS。
4. 把 OBB angle 转成抓取 yaw。
```

---

## 9.3 yolov8_grasping

原结构重点：

```text
yolov8_grasping/
├── launch/
├── yolov8_grasping/
│   ├── yolo_detector_node.py
│   ├── yolo_detector_obb_node.py
│   ├── pen_box_grasping_node.py
│   ├── pick_drop_node.py
│   ├── pick_drop_ik_node.py
│   ├── stopmotion_node.py
│   └── scripts/
│       ├── abort_manager.py
│       ├── keepout_manager.py
│       ├── pose_tools.py
│       ├── tf_tools.py
│       └── trajectory_scoring.py
```

建包：

```bash
cd src/perception

ros2 pkg create yolov8_grasping   --build-type ament_python   --dependencies rclpy sensor_msgs geometry_msgs tf2_ros std_msgs
```

关键学习模块：

```text
yolo_detector_node.py：普通 YOLO 检测
yolo_detector_obb_node.py：OBB 检测
pick_drop_node.py：基础抓取流程
pick_drop_ik_node.py：IK 目标抓取
pose_tools.py：位姿构造
tf_tools.py：坐标变换
trajectory_scoring.py：轨迹评分
abort_manager.py：中止逻辑
keepout_manager.py：禁区管理
```

验收：

```text
相机图像能订阅
检测结果能发布
像素 + 深度能转 3D 点
目标 pose 能转到 base_link
基础 pick/drop 能跑通
```

主动学习任务：

```text
1. 整理 yolov8_grasping 与 visual_servo 的重复脚本。
2. 解释 abort_manager 的作用。
3. 解释 keepout_manager 的作用。
4. 给每个脚本画输入输出表。
```

---

## 9.4 hand_eye_calibration

原结构：

```text
hand_eye_calibration/
├── hand_eye_calibration.repos
├── launch/calibrate.launch.py validate.launch.py
├── scripts/
│   ├── calibration_aruco_publisher.py
│   ├── handeye_publisher.py
│   ├── follow_aruco_marker.py
│   └── visualize_aruco_marker.py
├── config/aruco_parameters.yaml
└── rviz/
```

建包：

```bash
cd src/calibration

ros2 pkg create hand_eye_calibration   --build-type ament_python   --dependencies rclpy geometry_msgs tf2_ros tf2_geometry_msgs
```

### config/aruco_parameters.yaml

```yaml
aruco_node:
  ros__parameters:
    marker_size: 0.05
    aruco_dictionary_id: DICT_4X4_50
    image_topic: /camera/color/image_raw
    camera_info_topic: /camera/color/camera_info
    camera_frame: camera_color_optical_frame
```

### handeye_publisher.py 骨架

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class HandeyePublisher(Node):
    def __init__(self):
        super().__init__("handeye_publisher")
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_transform()

    def publish_transform(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "tool0"
        tf.child_frame_id = "camera_link"

        # TODO: 从标定 yaml 读取
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = HandeyePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

标定流程：

```text
1. 启动相机
2. 启动 ros2_aruco
3. 确认 /aruco_markers 有输出
4. 启动 easy_handeye2
5. 移动机械臂采集 10~20 组样本
6. 求解 hand-eye transform
7. 保存结果
8. 发布 static TF
9. validate.launch.py 验证误差
```

验收：

```text
ArUco 能检测
camera_link 能发布
camera 坐标能转 base_link
目标实际位置与机器人到达位置误差可接受
```

主动学习任务：

```text
1. 区分 Eye-in-hand 和 Eye-to-hand。
2. 画 base_link、tool0、camera_link、aruco_marker 的 TF 图。
3. 记录每次标定样本数和误差。
4. 写 validate 脚本验证标定。
```

提交：

```bash
git add src/perception src/calibration
git commit -m "add perception and hand-eye calibration packages"
```

---

# 10. 第八阶段：视觉伺服抓取 visual_servo

原结构：

```text
visual_servo/
├── visual_servo/
│   ├── servo_yolo_grasping_node.py
│   ├── servo_gazebo_grasping_node.py
│   ├── yolo_detector_obb_node.py
│   ├── stopmotion_node.py
│   ├── controllers/
│   │   ├── servo_controller.py
│   │   ├── servo_controller_gazebo.py
│   │   ├── pd_controller.py
│   │   ├── ladrc_controller.py
│   │   ├── mpc_controller.py
│   │   └── ema_filter.py
│   └── scripts/
│       ├── moveit_motion.py
│       ├── detection_cache.py
│       ├── target_selector.py
│       ├── trajectory_scoring.py
│       ├── abort_manager.py
│       ├── keepout_manager.py
│       ├── grasp_latch.py
│       ├── pose_factory.py
│       ├── pose_tools.py
│       ├── tf_tools.py
│       ├── param_tools.py
│       └── publishers.py
```

建包：

```bash
cd src/grasping

ros2 pkg create visual_servo   --build-type ament_python   --dependencies rclpy geometry_msgs sensor_msgs std_msgs tf2_ros trajectory_msgs

cd visual_servo
mkdir -p visual_servo/controllers visual_servo/scripts launch rviz
```

---

## 10.1 状态机

建议 13+1 状态：

```text
IDLE
INIT
SEARCHING
TARGET_FOUND
POSE_ESTIMATING
PRE_GRASP_PLANNING
MOVING_TO_PRE_GRASP
VISUAL_SERVOING
DESCENDING
GRASPING
LIFTING
PLACING
COMPLETED
ERROR
```

核心流程：

```text
SEARCHING
  ↓
TARGET_FOUND
  ↓
POSE_ESTIMATING
  ↓
PRE_GRASP_PLANNING
  ↓
MOVING_TO_PRE_GRASP
  ↓
VISUAL_SERVOING
  ↓
DESCENDING
  ↓
GRASPING
  ↓
LIFTING
  ↓
PLACING
  ↓
COMPLETED
```

---

## 10.2 controllers/pd_controller.py

```python
class PDController:
    def __init__(self, kp: float, kd: float, output_limit: float):
        self.kp = kp
        self.kd = kd
        self.output_limit = output_limit
        self.last_error = 0.0

    def reset(self):
        self.last_error = 0.0

    def update(self, error: float, dt: float) -> float:
        dt = max(dt, 1e-6)
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.kd * derivative
        self.last_error = error
        return max(-self.output_limit, min(self.output_limit, output))
```

---

## 10.3 controllers/ema_filter.py

```python
class EMAFilter:
    def __init__(self, alpha: float):
        self.alpha = alpha
        self.value = None

    def reset(self):
        self.value = None

    def update(self, new_value: float) -> float:
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value
        return self.value
```

---

## 10.4 controllers/servo_controller.py

```python
from .pd_controller import PDController


class ServoController:
    def __init__(self):
        self.x_controller = PDController(0.002, 0.0001, 0.05)
        self.y_controller = PDController(0.002, 0.0001, 0.05)

    def reset(self):
        self.x_controller.reset()
        self.y_controller.reset()

    def compute_command(self, pixel_error_x: float, pixel_error_y: float, dt: float):
        vx = self.x_controller.update(pixel_error_x, dt)
        vy = self.y_controller.update(pixel_error_y, dt)
        return vx, vy
```

---

## 10.5 scripts/detection_cache.py

```python
import time


class DetectionCache:
    def __init__(self, timeout: float):
        self.timeout = timeout
        self.latest_detection = None
        self.latest_time = 0.0

    def update(self, detection):
        self.latest_detection = detection
        self.latest_time = time.time()

    def get(self):
        if self.latest_detection is None:
            return None
        if time.time() - self.latest_time > self.timeout:
            return None
        return self.latest_detection
```

---

## 10.6 scripts/target_selector.py

```python
class TargetSelector:
    def __init__(self, target_class: str = ""):
        self.target_class = target_class

    def select(self, detections):
        if not detections:
            return None

        candidates = detections
        if self.target_class:
            candidates = [
                d for d in detections
                if getattr(d, "class_name", "") == self.target_class
            ]

        if not candidates:
            return None

        return max(candidates, key=lambda d: getattr(d, "confidence", 0.0))
```

---

## 10.7 scripts/abort_manager.py

```python
import time


class AbortManager:
    def __init__(self):
        self.abort_requested = False
        self.state_start_time = time.time()

    def request_abort(self):
        self.abort_requested = True

    def clear_abort(self):
        self.abort_requested = False

    def reset_state_timer(self):
        self.state_start_time = time.time()

    def is_timeout(self, timeout: float) -> bool:
        return time.time() - self.state_start_time > timeout

    def should_abort(self) -> bool:
        return self.abort_requested
```

---

## 10.8 scripts/keepout_manager.py

```python
class KeepoutManager:
    def __init__(self):
        self.z_min = 0.02
        self.x_min = -1.0
        self.x_max = 1.0
        self.y_min = -1.0
        self.y_max = 1.0

    def is_pose_safe(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z

        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and z >= self.z_min
        )
```

---

## 10.9 scripts/moveit_motion.py

```python
class MoveItMotion:
    def __init__(self, node):
        self.node = node

    def move_to_joint_goal(self, joint_goal):
        self.node.get_logger().info(f"Move to joint goal: {joint_goal}")
        # TODO: 调用 pymoveit2 或 MoveIt action
        return True

    def move_to_pose_goal(self, pose):
        self.node.get_logger().info("Move to pose goal")
        # TODO: 调用 MoveIt2 pose planning
        return True

    def execute_cartesian_offset(self, dx: float, dy: float, dz: float):
        self.node.get_logger().info(
            f"Cartesian offset dx={dx}, dy={dy}, dz={dz}"
        )
        return True
```

---

## 10.10 servo_yolo_grasping_node.py 最小状态机

```python
from enum import Enum, auto

import rclpy
from rclpy.node import Node

from visual_servo.controllers.servo_controller import ServoController
from visual_servo.scripts.detection_cache import DetectionCache
from visual_servo.scripts.target_selector import TargetSelector
from visual_servo.scripts.abort_manager import AbortManager
from visual_servo.scripts.keepout_manager import KeepoutManager
from visual_servo.scripts.moveit_motion import MoveItMotion


class ServoState(Enum):
    IDLE = auto()
    INIT = auto()
    SEARCHING = auto()
    TARGET_FOUND = auto()
    POSE_ESTIMATING = auto()
    PRE_GRASP_PLANNING = auto()
    MOVING_TO_PRE_GRASP = auto()
    VISUAL_SERVOING = auto()
    DESCENDING = auto()
    GRASPING = auto()
    LIFTING = auto()
    PLACING = auto()
    COMPLETED = auto()
    ERROR = auto()


class ServoYoloGraspingNode(Node):
    def __init__(self):
        super().__init__("servo_yolo_grasping_node")

        self.state = ServoState.IDLE
        self.servo_controller = ServoController()
        self.detection_cache = DetectionCache(timeout=0.5)
        self.target_selector = TargetSelector()
        self.abort_manager = AbortManager()
        self.keepout_manager = KeepoutManager()
        self.motion = MoveItMotion(self)

        self.timer = self.create_timer(0.02, self.loop)

    def set_state(self, new_state):
        self.get_logger().info(f"{self.state.name} -> {new_state.name}")
        self.state = new_state
        self.abort_manager.reset_state_timer()

    def loop(self):
        if self.abort_manager.should_abort():
            self.set_state(ServoState.ERROR)
            return

        if self.state == ServoState.IDLE:
            self.set_state(ServoState.INIT)

        elif self.state == ServoState.INIT:
            self.set_state(ServoState.SEARCHING)

        elif self.state == ServoState.SEARCHING:
            if self.detection_cache.get() is not None:
                self.set_state(ServoState.TARGET_FOUND)

        elif self.state == ServoState.TARGET_FOUND:
            self.set_state(ServoState.POSE_ESTIMATING)

        elif self.state == ServoState.POSE_ESTIMATING:
            # TODO: bbox/obb + depth -> 3D pose -> base_link
            self.set_state(ServoState.PRE_GRASP_PLANNING)

        elif self.state == ServoState.PRE_GRASP_PLANNING:
            # TODO: MoveIt2 规划到预抓取位姿
            self.set_state(ServoState.MOVING_TO_PRE_GRASP)

        elif self.state == ServoState.MOVING_TO_PRE_GRASP:
            self.set_state(ServoState.VISUAL_SERVOING)

        elif self.state == ServoState.VISUAL_SERVOING:
            # TODO: PD/LADRC/MPC 视觉闭环
            converged = True
            if converged:
                self.set_state(ServoState.DESCENDING)

        elif self.state == ServoState.DESCENDING:
            self.set_state(ServoState.GRASPING)

        elif self.state == ServoState.GRASPING:
            # TODO: 夹爪或 DO0 气缸闭合
            self.set_state(ServoState.LIFTING)

        elif self.state == ServoState.LIFTING:
            self.set_state(ServoState.PLACING)

        elif self.state == ServoState.PLACING:
            self.set_state(ServoState.COMPLETED)

        elif self.state == ServoState.ERROR:
            self.get_logger().error("ERROR state")


def main():
    rclpy.init()
    node = ServoYoloGraspingNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

测试：

```bash
colcon build --packages-select visual_servo
source install/setup.bash

ros2 run visual_servo servo_yolo_grasping_node
ros2 run visual_servo servo_gazebo_grasping_node
```

验收：

```text
状态机能启动
每个状态有日志
目标丢失不会崩溃
检测缓存有效
目标选择有效
PD 输出有限幅
禁区检查有效
中止管理有效
仿真版先跑通
真机版低速测试
```

主动学习任务：

```text
1. 给 13 个状态画状态转移图。
2. 给每个状态写进入条件和退出条件。
3. 比较 PD、LADRC、MPC。
4. 记录 20 次抓取成功率。
5. 分析失败原因：检测、标定、规划、控制、夹爪。
```

提交：

```bash
git add src/grasping/visual_servo
git commit -m "add visual servo grasping package"
```

---

# 11. 第九阶段：OctoMap + YOLO 融合抓取

对应包：

```text
octomap_yolo_grasping
```

原结构：

```text
octomap_yolo_grasping/
├── octomap_yolo_grasping/
│   ├── octomap_yolo_grasping_node.py
│   ├── dynamic_collision_objects_node.py
│   ├── semantic_octomap_cloud_filter_node.py
│   ├── yolo_detector_obb_node.py
│   ├── stopmotion_node.py
│   └── scripts/
├── launch/
└── rviz/
```

建包：

```bash
cd src/grasping

ros2 pkg create octomap_yolo_grasping   --build-type ament_python   --dependencies rclpy sensor_msgs geometry_msgs std_msgs tf2_ros
```

核心流程：

```text
YOLO OBB 检测
  ↓
深度点云
  ↓
语义点云过滤
  ↓
OctoMap 更新
  ↓
PlanningScene 更新
  ↓
MoveIt2 规划到 pre-grasp
  ↓
视觉伺服修正
  ↓
抓取
```

### dynamic_collision_objects_node.py 骨架

```python
import rclpy
from rclpy.node import Node


class DynamicCollisionObjectsNode(Node):
    def __init__(self):
        super().__init__("dynamic_collision_objects_node")
        self.get_logger().info("Dynamic collision objects node started")

    def add_box(self, name, pose, size):
        # TODO:
        # 构造 moveit_msgs/CollisionObject 并发布到 PlanningScene
        pass


def main():
    rclpy.init()
    node = DynamicCollisionObjectsNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### semantic_octomap_cloud_filter_node.py 骨架

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class SemanticOctomapCloudFilterNode(Node):
    def __init__(self):
        super().__init__("semantic_octomap_cloud_filter_node")
        self.sub = self.create_subscription(
            PointCloud2,
            "/camera/depth/color/points",
            self.cloud_callback,
            10
        )
        self.pub = self.create_publisher(
            PointCloud2,
            "/semantic_filtered_cloud",
            10
        )

    def cloud_callback(self, msg):
        # TODO: 根据 YOLO 语义区域过滤点云
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SemanticOctomapCloudFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

验收：

```text
PlanningScene 能看到动态障碍物
OctoMap 不包含机械臂自身点云
目标物体和障碍物能区分
规划能绕开障碍物
```

主动学习任务：

```text
1. 发布一个静态 box collision object。
2. 发布一个动态 box collision object。
3. 用点云生成 OctoMap。
4. 比较 sensors_3d.yaml 参数变化对地图的影响。
```

提交：

```bash
git add src/grasping/octomap_yolo_grasping
git commit -m "add OctoMap YOLO grasping package"
```

---

# 12. 第十阶段：GraspNet 抓取姿态规划

对应包：

```text
graspnet_grasping
```

原结构：

```text
graspnet_grasping/
├── graspnet_grasping/
│   ├── grasp_planner_node.py
│   └── realsense_capture_node.py
└── launch/graspnet_system.launch.py
```

建包：

```bash
cd src/grasping

ros2 pkg create graspnet_grasping   --build-type ament_python   --dependencies rclpy sensor_msgs geometry_msgs std_msgs
```

### realsense_capture_node.py

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class RealSenseCaptureNode(Node):
    def __init__(self):
        super().__init__("realsense_capture_node")
        self.sub = self.create_subscription(
            PointCloud2,
            "/camera/depth/color/points",
            self.cloud_callback,
            10
        )
        self.latest_cloud = None

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def save_cloud(self, path):
        # TODO: PointCloud2 -> PCD
        self.get_logger().info(f"Save point cloud to {path}")


def main():
    rclpy.init()
    node = RealSenseCaptureNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### grasp_planner_node.py

```python
import rclpy
from rclpy.node import Node


class GraspPlannerNode(Node):
    def __init__(self):
        super().__init__("grasp_planner_node")

    def run_graspnet(self, point_cloud_path):
        # TODO:
        # 1. 调用 GraspNet 推理
        # 2. 输出 grasp candidates
        # 3. 转 PoseStamped
        # 4. 按 score 排序
        return []

    def filter_candidates(self, candidates):
        # TODO:
        # IK 可达性检查 + 碰撞检查
        return candidates


def main():
    rclpy.init()
    node = GraspPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

流程：

```text
采集点云
  ↓
保存 PCD
  ↓
GraspNet 推理
  ↓
输出 grasp candidates
  ↓
分数排序
  ↓
IK 可达性检查
  ↓
碰撞检查
  ↓
MoveIt2 规划到 pre-grasp
```

验收：

```text
能采集点云
能离线跑 GraspNet
能输出多个抓取姿态
每个姿态有 score
能筛选不可达姿态
```

主动学习任务：

```text
1. 保存一帧 PCD 并可视化。
2. 记录 GraspNet 输出前 10 个候选。
3. 把候选姿态转换到 base_link。
4. 分析最高分候选是否适合机械臂。
```

提交：

```bash
git add src/grasping/graspnet_grasping
git commit -m "add GraspNet grasping package"
```

---

# 13. 第十一阶段：工具包

对应包：

```text
trajectory_retime_server
control_servers
data_monitor
panda_arm_msg
```

---

## 13.1 trajectory_retime_server

原结构：

```text
trajectory_retime_server/
├── srv/RetimeTrajectory.srv
├── src/retime_server.cpp
└── launch/retime_server.launch.py
```

建包：

```bash
cd src/tools

ros2 pkg create trajectory_retime_server   --build-type ament_cmake   --dependencies rclcpp trajectory_msgs
```

### srv/RetimeTrajectory.srv

```text
trajectory_msgs/JointTrajectory input_trajectory
float64 velocity_scale
float64 acceleration_scale
---
bool success
string message
trajectory_msgs/JointTrajectory output_trajectory
```

### retime_server.cpp 骨架

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>

class RetimeServer : public rclcpp::Node
{
public:
  RetimeServer()
  : Node("trajectory_retime_server")
  {
    RCLCPP_INFO(get_logger(), "Trajectory retime server started");
    // TODO: create_service<RetimeTrajectory>
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RetimeServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

验收：

```text
服务能启动
能接收 JointTrajectory
velocity_scale 生效
非法参数返回失败
```

---

## 13.2 control_servers

原结构：

```text
control_servers/
├── control_servers/joint_control_app.py
└── app/
    ├── main.py
    ├── requirements.txt
    ├── templates/index.html
    └── static/css/style.css static/js/script.js static/favicon.svg
```

建包：

```bash
cd src/tools

ros2 pkg create control_servers   --build-type ament_python   --dependencies rclpy sensor_msgs geometry_msgs trajectory_msgs
```

### app/main.py

```python
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI(title="S622 Robot Control Server")


class JointCommand(BaseModel):
    joints: list[float]


@app.get("/api/robot_state")
def get_robot_state():
    return {
        "connected": True,
        "enabled": True,
        "joint_position": [0, 0, 0, 0, 0, 0],
        "gripper_state": "unknown"
    }


@app.post("/api/move_joint")
def move_joint(command: JointCommand):
    if len(command.joints) != 6:
        return {"success": False, "message": "Expected 6 joints"}

    # TODO: 通过 ROS bridge 调用 MoveIt2 或 FollowJointTrajectory action
    return {"success": True, "message": "Joint command accepted"}


@app.post("/api/gripper/open")
def open_gripper():
    return {"success": True}


@app.post("/api/gripper/close")
def close_gripper():
    return {"success": True}


@app.post("/api/stop")
def stop_robot():
    return {"success": True}
```

运行：

```bash
cd src/tools/control_servers/app
uvicorn main:app --host 0.0.0.0 --port 8000
```

验收：

```text
浏览器能打开页面
/api/robot_state 返回状态
/api/move_joint 能接收命令
能控制夹爪
有 stop 接口
```

---

## 13.3 data_monitor

原结构：

```text
data_monitor/
├── data_monitor/
│   ├── joint_trajectory_monitor_server.py
│   └── get_end_position.py
└── launch/data_monitor.launch.py
```

建包：

```bash
cd src/tools

ros2 pkg create data_monitor   --build-type ament_python   --dependencies rclpy sensor_msgs trajectory_msgs geometry_msgs tf2_ros
```

### joint_trajectory_monitor_server.py

```python
import csv
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointTrajectoryMonitorServer(Node):
    def __init__(self):
        super().__init__("joint_trajectory_monitor_server")

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        filename = f"joint_states_{int(time.time())}.csv"
        self.file = open(filename, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "time",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ])

    def joint_state_callback(self, msg):
        joint_map = dict(zip(msg.name, msg.position))
        row = [self.get_clock().now().nanoseconds * 1e-9]
        for name in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
            row.append(joint_map.get(name, 0.0))

        self.writer.writerow(row)
        self.file.flush()


def main():
    rclpy.init()
    node = JointTrajectoryMonitorServer()
    rclpy.spin(node)
    rclpy.shutdown()
```

验收：

```text
能记录 joint_states CSV
能查询 tool0 在 base_link 下的位置
能用于分析轨迹误差
```

主动学习任务：

```text
1. 记录 10 次执行轨迹。
2. 画 target vs actual 曲线。
3. 统计最大误差。
4. 分析误差出现在哪个动作阶段。
```

提交：

```bash
git add src/tools
git commit -m "add tool packages"
```

---

# 14. 第十二阶段：外部开源包

对应：

```text
pymoveit2
easy_handeye2
ros2_aruco
realsense2_gz_description
```

放置：

```text
src/vendor/
├── pymoveit2/
├── easy_handeye2/
├── ros2_aruco/
└── realsense2_gz_description/
```

---

## 14.1 pymoveit2

用途：

```text
Python 调用 MoveIt2。
用于 pick_drop_node、visual_servo、Web 控制等。
```

主动学习任务：

```text
1. 运行 ex_joint_goal.py。
2. 运行 ex_pose_goal.py。
3. 运行 ex_ik.py。
4. 阅读 moveit2.py 的 action/service 调用方式。
```

---

## 14.2 easy_handeye2

用途：

```text
硬件无关手眼标定。
```

主动学习任务：

```text
1. 阅读 handeye_sampler。
2. 阅读 handeye_calibration。
3. 理解样本如何采集。
4. 理解 transform 如何发布。
```

---

## 14.3 ros2_aruco

用途：

```text
ArUco 标记检测。
```

主动学习任务：

```text
1. 查看 ArucoMarkers.msg。
2. 启动 aruco_node。
3. 确认 marker pose 的 frame_id。
4. 配合 easy_handeye2 使用。
```

---

## 14.4 realsense2_gz_description

用途：

```text
RealSense D435 Gazebo 仿真模型。
```

主动学习任务：

```text
1. 查看 D435 xacro。
2. 确认 color/depth/camera_info topic。
3. 把 D435 挂到 S622 末端。
4. 在 Gazebo 中显示相机视野。
```

---

# 15. 第十三阶段：GraphExecuter 独立项目

原结构：

```text
GraphExecuter/
├── COLCON_IGNORE
└── graph_executer/
    ├── main.py
    ├── nodes/
    ├── src/mainwindow.py messageconsole.py updatelog.py
    ├── ui/
    ├── utils/general.py
    └── bin/
```

关键点：

```text
GraphExecuter 不是 ROS 2 包。
必须放 COLCON_IGNORE。
它是 PySide6 + NodeGraphQt 可视化工作流系统。
```

创建：

```bash
cd src
mkdir -p GraphExecuter/graph_executer
touch GraphExecuter/COLCON_IGNORE
```

推荐节点：

```text
StartNode
DetectObjectNode
EstimatePoseNode
PlanMotionNode
ExecuteMotionNode
VisualServoNode
GripperCloseNode
LiftNode
PlaceNode
EndNode
ErrorHandlerNode
```

基础节点接口：

```python
class BaseWorkflowNode:
    def __init__(self, name: str):
        self.name = name

    def execute(self, context: dict) -> dict:
        raise NotImplementedError
```

示例：

```python
class DetectObjectNode(BaseWorkflowNode):
    def execute(self, context: dict) -> dict:
        context["object_detected"] = True
        context["object_pose"] = [0.4, 0.1, 0.2]
        return context
```

验收：

```text
colcon build 时跳过 GraphExecuter
PySide6 GUI 能单独运行
能拖拽节点
能保存工作流
能执行简单任务链
```

主动学习任务：

```text
1. 把视觉抓取拆成 8 个节点。
2. 定义每个节点输入输出。
3. 设计失败回滚逻辑。
4. 用 GraphExecuter 调用 ROS 2 service。
```

提交：

```bash
git add src/GraphExecuter
git commit -m "add GraphExecuter workflow project"
```

---

# 16. 分批编译顺序

不要一开始全量编译。按依赖顺序分批：

## 第一批：接口和模型

```bash
colcon build --packages-select   fairino_msgs   yolov8_obb_msgs   fairino_description   s622_moveit_descriptions
```

## 第二批：硬件接口

```bash
colcon build --packages-select fairino_hardware
```

## 第三批：MoveIt2 配置

```bash
colcon build --packages-select   fairino3_v6_moveit2_config   s622_moveit_config
```

## 第四批：规划核心

```bash
colcon build --packages-select   fairino_planning_core   fairino_planning_ros
```

## 第五批：仿真与视觉

```bash
colcon build --packages-select   gz_launch   yolov8_obb   yolov8_grasping   hand_eye_calibration
```

## 第六批：抓取

```bash
colcon build --packages-select   visual_servo   octomap_yolo_grasping   graspnet_grasping
```

## 第七批：工具

```bash
colcon build --packages-select   trajectory_retime_server   control_servers   data_monitor   panda_arm_msg
```

## 全量编译

```bash
colcon build --symlink-install
source install/setup.bash
```

---

# 17. 主线 Demo 路线

## Demo 1：Fairino 模型显示

```bash
ros2 launch fairino_description display.launch.py model:=fairino3_v6.urdf
```

验收：

```text
RViz 显示 Fairino3 v6
```

## Demo 2：S622 模型显示

```bash
ros2 launch s622_moveit_descriptions display.launch.py
```

验收：

```text
RViz 显示 S622、夹爪、相机
```

## Demo 3：MoveIt2 规划

```bash
ros2 launch s622_moveit_config demo.launch.py
```

验收：

```text
RViz Planning 面板能 Plan
```

## Demo 4：controller 检查

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

验收：

```text
joint_state_broadcaster active
arm_controller active
gripper_controller active
```

## Demo 5：Gazebo 仿真

```bash
ros2 launch gz_launch gazebo.launch.py
```

验收：

```text
Gazebo 打开 world，机器人加载
```

## Demo 6：仿真抓取

```bash
ros2 launch gz_launch pick_block.launch.py
```

验收：

```text
机械臂完成固定物体 pick/drop
```

## Demo 7：自研规划器

```bash
ros2 run fairino_planning_ros standalone_planner_node
```

验收：

```text
输出规划路径
```

## Demo 8：YOLO OBB 检测

```bash
ros2 launch yolov8_obb yolov8_obb.launch.py
```

验收：

```text
/yolov8_obb/inference 有检测结果
```

## Demo 9：手眼标定

```bash
ros2 launch hand_eye_calibration calibrate.launch.py
```

验收：

```text
能采集样本并发布 camera_link TF
```

## Demo 10：视觉伺服仿真

```bash
ros2 launch visual_servo servo_yolo_grasping.launch.py
```

验收：

```text
状态机走到 COMPLETED
```

## Demo 11：OctoMap 融合抓取

```bash
ros2 launch octomap_yolo_grasping octomap_yolo_grasping.launch.py
```

验收：

```text
PlanningScene 中有障碍物，抓取路径避障
```

## Demo 12：Web 控制

```bash
cd src/tools/control_servers/app
uvicorn main:app --host 0.0.0.0 --port 8000
```

验收：

```text
浏览器能访问控制页面
```

---

# 18. 调试检查清单

## 18.1 基础

```bash
source install/setup.bash
ros2 pkg list | grep fairino
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

## 18.2 TF

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link tool0
ros2 run tf2_ros tf2_echo base_link camera_link
```

检查：

```text
base_link 是否存在
tool0 是否存在
camera_link 是否存在
camera_color_optical_frame 是否存在
是否有断开的 TF 树
```

## 18.3 ros2_control

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

检查：

```text
joint name 是否和 URDF 一致
controller joints 是否和 MoveIt 一致
command_interface 是否存在
state_interface 是否存在
controller 是否 active
```

## 18.4 MoveIt2

```text
robot_description 是否加载
robot_description_semantic 是否加载
planning group 是否叫 arm
end effector 是否绑定 gripper
kinematics.yaml 是否正确
moveit_controllers.yaml 是否正确
collision matrix 是否过严
```

## 18.5 视觉

```bash
ros2 topic echo /camera/color/image_raw
ros2 topic echo /camera/depth/image_raw
ros2 topic echo /camera/color/camera_info
ros2 topic echo /yolov8_obb/inference
```

检查：

```text
图像帧率
深度是否对齐
camera_info 是否存在
检测结果 frame_id 是否正确
```

## 18.6 标定

```bash
ros2 run tf2_ros tf2_echo tool0 camera_link
ros2 run tf2_ros tf2_echo base_link camera_link
```

检查：

```text
手眼结果是否发布
frame_id 是否反了
米/毫米单位是否混乱
光学坐标系方向是否正确
```

## 18.7 抓取

检查：

```text
状态机卡在哪个状态
检测缓存是否过期
目标选择是否为空
禁区是否误触发
PD 输出是否太大
MoveIt 规划是否失败
夹爪命令是否下发
DO0 是否真正动作
```

---

# 19. 每个包的学习模板

文件：`docs/package_study_template.md`

```markdown
# Package Study Note

## Package Name

## Layer

vendor / description / moveit / planning / simulation / perception / calibration / grasping / tools / independent

## Purpose

这个包解决什么问题？

## Inputs

输入 topic/service/action/config/file 是什么？

## Outputs

输出 topic/service/action/config/file 是什么？

## Main Files

最重要的文件有哪些？

## Dependencies

依赖哪些包？

## Build

```bash
colcon build --packages-select package_name
```

## Run

```bash
ros2 launch ...
```

## Validation

怎么证明它跑通？

## Common Bugs

常见错误有哪些？

## My Understanding

我现在如何理解这个包？

## Next Step

下一步做什么？

```
---

# 20. 每日学习日志模板

文件：`docs/daily_learning_log.md`

```markdown
# Daily Learning Log

## Date

YYYY-MM-DD

## Today Goal

今天要打通哪条链路？

## Packages

今天涉及哪些包？

## Files Modified

修改了哪些文件？

## Commands

运行过哪些命令？

## Result

结果是什么？

## Bugs

遇到什么 bug？

## Debug Process

如何定位？

## Fix

如何修复？

## What I Learned

今天真正理解了什么？

## Next Action

下一步做什么？
```

---

# 21. 推荐学习节奏

| 周期     | 目标                  | 包                                                      |
| -------- | --------------------- | ------------------------------------------------------- |
| 第 1 周  | ROS 2、URDF、TF       | fairino_description、s622_moveit_descriptions           |
| 第 2 周  | msg/srv、ros2_control | fairino_msgs、fairino_hardware                          |
| 第 3 周  | MoveIt2 基础          | s622_moveit_config、fairino3_v6_moveit2_config          |
| 第 4 周  | Gazebo 仿真           | gz_launch、realsense2_gz_description                    |
| 第 5 周  | C++ FK/IK             | fairino_planning_core                                   |
| 第 6 周  | RRT*/BiRRT*           | fairino_planning_core                                   |
| 第 7 周  | MoveIt2 插件          | fairino_planning_ros                                    |
| 第 8 周  | YOLO 检测             | yolov8_obb、yolov8_obb_msgs、yolov8_grasping            |
| 第 9 周  | 手眼标定              | hand_eye_calibration、easy_handeye2、ros2_aruco         |
| 第 10 周 | 视觉伺服              | visual_servo                                            |
| 第 11 周 | OctoMap/GraspNet      | octomap_yolo_grasping、graspnet_grasping                |
| 第 12 周 | 工具化                | trajectory_retime_server、control_servers、data_monitor |
| 第 13 周 | 工作流                | GraphExecuter                                           |
| 第 14 周 | 系统整合              | 全部                                                    |

---

# 22. 最终验收总表

| 层          | 包                         | 验收                          |
| ----------- | -------------------------- | ----------------------------- |
| vendor      | fairino_description        | 多型号模型能显示              |
| vendor      | fairino_msgs               | msg/srv 可生成                |
| vendor      | fairino_hardware           | ros2_control 能读写机器人     |
| description | s622_moveit_descriptions   | S622 + 夹爪 + 相机 TF 正确    |
| MoveIt2     | fairino3_v6_moveit2_config | Fairino3 MoveIt2 可规划       |
| MoveIt2     | s622_moveit_config         | S622 MoveIt2 可规划执行       |
| planning    | fairino_planning_core      | FK/IK/RRT*/BiRRT* 单测通过    |
| planning    | fairino_planning_ros       | MoveIt2 能加载自研 planner/IK |
| simulation  | gz_launch                  | Gazebo 能完成抓取 demo        |
| perception  | yolov8_obb_msgs            | OBB 消息可用                  |
| perception  | yolov8_obb                 | OBB 检测 topic 有输出         |
| perception  | yolov8_grasping            | 检测+基础抓取流程可运行       |
| calibration | hand_eye_calibration       | camera 与 base 坐标转换正确   |
| grasping    | visual_servo               | 视觉伺服抓取状态机可运行      |
| grasping    | octomap_yolo_grasping      | 动态障碍物可避障              |
| grasping    | graspnet_grasping          | 点云生成抓取候选              |
| tools       | trajectory_retime_server   | 轨迹能重新定时                |
| tools       | control_servers            | Web API 能控制机器人          |
| tools       | data_monitor               | 能记录轨迹和末端位置          |
| tools       | panda_arm_msg              | 遗留消息隔离                  |
| external    | pymoveit2                  | Python 可调用 MoveIt2         |
| external    | easy_handeye2              | 可用于手眼标定                |
| external    | ros2_aruco                 | 可检测 ArUco                  |
| external    | realsense2_gz_description  | D435 仿真模型可用             |
| independent | GraphExecuter              | 可视化工作流独立运行          |

---

# 23. 一句话总结

从零搭建原项目时，正确路线是：

```text
模型显示
  ↓
消息接口
  ↓
硬件控制
  ↓
MoveIt2 规划
  ↓
Gazebo 仿真
  ↓
自研 planning_core
  ↓
MoveIt2 插件
  ↓
YOLO 检测
  ↓
手眼标定
  ↓
视觉伺服抓取
  ↓
OctoMap / GraspNet
  ↓
Web / 数据监控 / GraphExecuter
```

主动学习的核心标准是：

```text
我今天是否让系统多了一条可运行、可验证、可解释、可复盘的能力链路？
```
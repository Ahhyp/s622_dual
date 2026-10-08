# Phase 4：执行抓取

## 一、功能

订阅 `/grasp_pose`，把"看到一个抓取点"变成"机械臂走过去抓起来"。

## 二、思路

### 1. 沿用 Phase 3 的分层风格

```
yolov8_grasping/
├── pose_estimator.py        # Phase 3：算法层
├── grasping_node.py         # Phase 1-3：感知节点
├── arm_executor.py          # Phase 4：执行模块（封装 MoveIt2 + 夹爪）
└── executor_node.py         # Phase 4：执行节点（状态机）
```

**为什么再开一个节点而不是塞进 grasping_node**：感知和执行是两种关注点，崩了也不应该互相牵连；将来可能要"感知一直跑、执行手动触发"，分开后用参数就能切。

### 2. 五步状态机

| 步骤 | 动作                                      | 规划方式                     |
| ---- | ----------------------------------------- | ---------------------------- |
| 1    | 张开夹爪 + 飞到 pregrasp（物体上方 10cm） | 关节空间规划（避障）         |
| 2    | 直线下降到 grasp                          | **笛卡尔**（不能绕弯撞物体） |
| 3    | 关闭夹爪                                  | 直接发话题                   |
| 4    | 直线上升回 pregrasp                       | **笛卡尔**                   |
| 5    | 标记完成                                  | —                            |

### 3. 关键安全设计

* **默认不自动执行**：`auto_execute=false`，因为 `/grasp_pose` 是跟着深度图发的，频率高；自动执行会让机械臂在每帧都重新开始抓，乱套。
* **触发方式两种**：

  * 设 `auto_execute=true`：第一帧来了就抓（适合调试单次任务）
  * 默认情况：发 `/grasp_trigger`（Bool）才抓最近一次的位姿
* **`self.busy` 标志位**：抓取过程中收到的新位姿全部忽略，防重入。

### 4. 姿态约定

按你的要求，**进入执行节点后重写姿态**：从 `/grasp_pose` 里抽出 yaw，按 `roll=0, pitch=-π, yaw=yaw` 重新构造四元数。

### 5. 手动验证先行

写代码之前**先在 RViz 里**：

1. 加 PoseStamped 显示 `/grasp_pose`，确认箭头位置合理
2. 用 MoveIt RViz 插件（MotionPlanning panel）的 Interactive Marker 把末端拖到那个位置，点 Plan & Execute，确认**可达 + 不奇异 + 不撞自己**
3. 这一步通过了，再启动 executor_node，否则代码层面再怎么写都白搭

## 三、实现

### 1. 安装依赖

pymoveit2 不在 PyPI 上。Python 调 MoveIt2 其实不需要它——直接通过 rclpy action client 调 move_group 的 /move_action 就行，零额外依赖。

让我看看源项目怎么做的。

源项目的 pymoveit2 是一个 ROS 2 包（放在 src/ 下用 colcon 编译），不是 pip 包。看看你有没有拷贝过来。



直接拷过来就行：

```bash
cp -r /home/yep/S622_robotarm/src/pymoveit2 /home/yep/my_S622/src/
```
然后你的 yolov8_grasping/package.xml 加一个 <depend>pymoveit2</depend>，就能按源项目的 API 写了：

```python
from pymoveit2 import MoveIt2

self.moveit2_arm = MoveIt2("arm_group", ...)
self.moveit2_arm.plan(pose, cartesian=False)
self.moveit2_arm.execute(trajectory)
```
这样 Phase 4 的代码和源项目一致，你之前发给 AI 的提示词里 pymoveit2 的部分也就不用改了。




### 2. `arm_executor.py`（执行模块）

```python
#!/usr/bin/env python3
"""MoveIt2 + 夹爪封装"""
import time
from typing import List

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

from pymoveit2 import MoveIt2


class ArmExecutor:
    def __init__(
        self,
        node: Node,
        joint_names: List[str],
        base_link: str,
        end_effector: str,
        group_name: str,
        gripper_topic: str = "/gripper_command",
        gripper_open: float = 0.0,
        gripper_close: float = 0.02,
        max_vel: float = 0.3,
        max_acc: float = 0.3,
    ):
        self.node = node
        self.gripper_open_val = gripper_open
        self.gripper_close_val = gripper_close

        cb_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=node,
            joint_names=joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name,
            callback_group=cb_group,
        )
        self.moveit2.max_velocity = max_vel
        self.moveit2.max_acceleration = max_acc

        self.gripper_pub = node.create_publisher(
            Float64, gripper_topic, 10
        )

    # ---------- 运动 ----------
    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w]

        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat,
            cartesian=cartesian,
        )
        return self.moveit2.wait_until_executed()

    # ---------- 夹爪 ----------
    def open_gripper(self, wait: float = 1.0):
        msg = Float64()
        msg.data = self.gripper_open_val
        self.gripper_pub.publish(msg)
        time.sleep(wait)

    def close_gripper(self, wait: float = 1.0):
        msg = Float64()
        msg.data = self.gripper_close_val
        self.gripper_pub.publish(msg)
        time.sleep(wait)
```

### 3. `executor_node.py`（状态机节点）

```python
#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from yolov8_grasping.arm_executor import ArmExecutor


class GraspExecutorNode(Node):
    def __init__(self):
        super().__init__("grasp_executor_node")

        # ---------- 参数 ----------
        self.declare_parameter(
            "joint_names",
            ["j1", "j2", "j3", "j4", "j5", "j6"],   # 按 SRDF 改
        )
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("end_effector", "tool0")        # 按 SRDF 改
        self.declare_parameter("group_name", "robot_arm")      # 按 SRDF 改
        self.declare_parameter("pregrasp_offset_z", 0.10)
        self.declare_parameter("auto_execute", False)

        joint_names = self.get_parameter("joint_names").value
        base_link = self.get_parameter("base_link").value
        end_effector = self.get_parameter("end_effector").value
        group_name = self.get_parameter("group_name").value
        self.pregrasp_offset_z = self.get_parameter("pregrasp_offset_z").value
        self.auto_execute = self.get_parameter("auto_execute").value

        # ---------- 执行器 ----------
        self.arm = ArmExecutor(
            node=self,
            joint_names=joint_names,
            base_link=base_link,
            end_effector=end_effector,
            group_name=group_name,
        )

        # ---------- 订阅 ----------
        self.busy = False
        self.latest_pose: Optional[PoseStamped] = None

        self.create_subscription(
            PoseStamped, "/grasp_pose", self.cb_grasp_pose, 10)
        self.create_subscription(
            Bool, "/grasp_trigger", self.cb_trigger, 10)

        self.get_logger().info(
            f"Grasp executor ready, auto_execute={self.auto_execute}, "
            f"group={group_name}, ee={end_effector}"
        )

    # ---------- 回调 ----------
    def cb_grasp_pose(self, msg: PoseStamped):
        self.latest_pose = msg
        if self.auto_execute and not self.busy:
            self.execute_grasp(msg)

    def cb_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self.busy:
            self.get_logger().warning("busy, ignore trigger")
            return
        if self.latest_pose is None:
            self.get_logger().warning("no grasp pose yet, ignore trigger")
            return
        self.execute_grasp(self.latest_pose)

    # ---------- 抓取流程 ----------
    def execute_grasp(self, pose_stamped: PoseStamped):
        self.busy = True
        try:
            target = pose_stamped.pose

            # 重写姿态：roll=0, pitch=-π, yaw 来自 /grasp_pose
            _, _, yaw = euler_from_quaternion([
                target.orientation.x, target.orientation.y,
                target.orientation.z, target.orientation.w,
            ])
            qx, qy, qz, qw = quaternion_from_euler(0.0, -math.pi, yaw)

            pregrasp = Pose()
            pregrasp.position.x = target.position.x
            pregrasp.position.y = target.position.y
            pregrasp.position.z = target.position.z + self.pregrasp_offset_z
            pregrasp.orientation.x = qx
            pregrasp.orientation.y = qy
            pregrasp.orientation.z = qz
            pregrasp.orientation.w = qw

            grasp = Pose()
            grasp.position.x = target.position.x
            grasp.position.y = target.position.y
            grasp.position.z = target.position.z
            grasp.orientation = pregrasp.orientation

            # 1. 张开夹爪 + 关节空间到 pregrasp
            self.get_logger().info("[1/5] open gripper + move to pregrasp")
            self.arm.open_gripper()
            if not self.arm.move_to_pose(pregrasp, cartesian=False):
                self.get_logger().error("pregrasp planning failed")
                return

            # 2. 笛卡尔直线下降
            self.get_logger().info("[2/5] cartesian descent")
            if not self.arm.move_to_pose(grasp, cartesian=True):
                self.get_logger().error("descent failed")
                return

            # 3. 关闭夹爪
            self.get_logger().info("[3/5] close gripper")
            self.arm.close_gripper()

            # 4. 笛卡尔直线上升
            self.get_logger().info("[4/5] cartesian ascent")
            if not self.arm.move_to_pose(pregrasp, cartesian=True):
                self.get_logger().error("ascent failed")
                return

            # 5. 完成
            self.get_logger().info("[5/5] grasp done")
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = GraspExecutorNode()
    # pymoveit2 需要多线程 spin（action callback 和主循环并发）
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 4. 更新 `setup.py`

加入口点：

```python
entry_points={
    "console_scripts": [
        "grasping_node = yolov8_grasping.grasping_node:main",
        "grasp_executor_node = yolov8_grasping.executor_node:main",   # ← 新增
    ],
},
```

### 5. 更新 `package.xml`

添加依赖：

```xml
<exec_depend>std_msgs</exec_depend>
<exec_depend>moveit_msgs</exec_depend>
<exec_depend>action_msgs</exec_depend>
<exec_depend>tf_transformations</exec_depend>
```

## 四、运行步骤

### 第一步：手动验证可达性（**别跳过**）

```bash
ros2 launch gz_launch s622_gazebo.launch.py
ros2 run yolov8_obb yolov8_obb_node
ros2 run yolov8_grasping grasping_node
```

在 RViz 里：

* 加 `PoseStamped`，topic = `/grasp_pose`，看箭头位置
* 用 MoveIt MotionPlanning panel 把交互 marker 拖到箭头位置 → Plan → Execute
* 如果 Plan 失败：物体太远 / 太近 / 姿态不可达，**改 obb 节点里的假坐标**重试

### 第二步：构建

```bash
cd ~/my_S622
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
colcon build --merge-install --symlink-install --packages-select yolov8_grasping
source install/setup.bash
```

### 第三步：先用手动触发模式跑

```bash
# 终端 1：执行节点（默认 auto_execute=false）
ros2 run yolov8_grasping grasp_executor_node

# 终端 2：手动触发一次
ros2 topic pub --once /grasp_trigger std_msgs/msg/Bool "{data: true}"
```

观察机械臂是不是按 5 步走完了。

### 第四步：联调通了再开自动模式

```bash
ros2 run yolov8_grasping grasp_executor_node --ros-args -p auto_execute:=true
```

## 五、需要你确认/调整的几个值

下面这些我**只能猜**，启动报错或者动作不对就来这里改：

| 参数               | 我给的默认  | 怎么确认真实值                                      |
| ------------------ | ----------- | --------------------------------------------------- |
| `joint_names`      | `j1..j6`    | `cat src/s622_moveit_config/config/*.srdf` 看 group |
| `group_name`       | `robot_arm` | 同上，找 `<group name="...">`                       |
| `end_effector`     | `tool0`     | SRDF 里 `<end_effector>` 或 URDF 末端 link          |
| `/gripper_command` | 你说的      | 已确认                                              |

把你的 SRDF 关键片段贴出来，我可以把这三个默认值直接改对。

## 六、已知风险

* **pymoveit2 与 MoveIt2 Humble 版本兼容**：少数情况下 action 名字对不上，启动后会卡住等 action server。卡住超过 5 秒就 Ctrl+C，加 `--ros-args --log-level debug` 看具体卡在哪。
* **笛卡尔规划可能只规划出部分路径**（fraction < 1.0）：pymoveit2 的 `cartesian=True` 内部如果完成度不够会失败。如果遇到，下一步要换成 `compute_cartesian_path` + 检查 fraction，再单独 execute，逻辑稍复杂，但暂时不动。
* **夹爪用 `time.sleep(1.0)` 等到位**是简化做法。真硬件如果有反馈话题（关节状态、电流），后面可以换成等反馈。

需不需要我接着帮你写一个最简的 `phase4.launch.py`，把 obb + grasping + executor 三个节点串起来？


在告知 ai 一些必要信息之后：


# 先回答约定问题

`(roll=π, pitch=0, yaw=θ)` 和 `(roll=0, pitch=-π, yaw=θ)` **不等效**，差一个绕 z 的 180°。

简单推导（XYZ 外旋）：

* 方案 A `(π, 0, θ)`：先把工具系绕 X 翻转 → Z 朝下、Y 反向、**X 不变**，再绕新 Z 转 θ。
* 方案 B `(0, -π, θ)`：先把工具系绕 Y 翻转 → Z 朝下、**X 反向**、Y 不变，再绕新 Z 转 θ。

两者都是"夹爪朝下"，但 X/Y 朝向相反。把同一个 `yaw=θ` 喂进去，两条路径出来的四元数不一样，相当于夹爪手指方向差 180°。对二指对称夹爪可能"看起来一样"，但实际是两个不同的位姿，会让 MoveIt 解出截然不同的关节解。

**统一方案**：跟 `grasping_node` 对齐，用 `(π, 0, yaw)`。

下面给你改好的两个文件。

## arm_executor.py

```python
#!/usr/bin/env python3
"""MoveIt2 机械臂运动 + 夹爪控制封装

夹爪是 joint_trajectory_controller，吃 trajectory_msgs/JointTrajectory，
不是 std_msgs/Float64。两根手指反向运动：
    打开：finger1=+0.025, finger2=-0.025
    关闭：finger1=0.0,    finger2=0.0
"""
import time
from typing import List, Sequence

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2 import MoveIt2


class ArmExecutor:
    """把 pymoveit2 的运动接口和夹爪 trajectory 接口封一层，
    让上层 executor_node 只关心"动到哪 / 抓不抓"，不关心底层消息。
    """

    def __init__(
        self,
        node: Node,
        # ---- 机械臂相关 ----
        joint_names: List[str],
        base_link: str,
        end_effector: str,
        group_name: str,
        # ---- 夹爪相关 ----
        gripper_topic: str = "/hand_controller/joint_trajectory",
        gripper_joint_names: Sequence[str] = ("finger1_joint", "finger2_joint"),
        gripper_open: Sequence[float] = (0.025, -0.025),
        gripper_close: Sequence[float] = (0.0, 0.0),
        gripper_time_sec: float = 1.0,
        # ---- 运动学限速 ----
        max_vel: float = 0.3,
        max_acc: float = 0.3,
    ):
        self.node = node

        # 夹爪参数（list/tuple 都接受，统一存成 list 方便后续赋值）
        self.gripper_joint_names = list(gripper_joint_names)
        self.gripper_open_val = list(gripper_open)
        self.gripper_close_val = list(gripper_close)
        self.gripper_time_sec = float(gripper_time_sec)

        # ---- MoveIt2 接口 ----
        # ReentrantCallbackGroup 让 action 回调和主循环可以并发，
        # 否则 wait_until_executed() 会和回调互锁。
        cb_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=node,
            joint_names=joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name,
            callback_group=cb_group,
        )
        self.moveit2.max_velocity = max_vel
        self.moveit2.max_acceleration = max_acc

        # ---- 夹爪 publisher ----
        # JointTrajectory 是控制器接口，不要换成 Float64。
        self.gripper_pub = node.create_publisher(
            JointTrajectory, gripper_topic, 10
        )

    # ------------------------------------------------------------------
    # 机械臂运动
    # ------------------------------------------------------------------
    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        """把末端运动到指定 Pose。

        Args:
            pose: 目标位姿，frame 由 MoveIt2 内部按 base_link 处理。
            cartesian: True 走笛卡尔直线（用于下降/抬升），
                       False 走关节空间规划（用于飞向 pregrasp）。

        Returns:
            执行是否成功。失败原因可能是规划失败或执行被打断。
        """
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat = [
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ]

        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat,
            cartesian=cartesian,
        )
        return self.moveit2.wait_until_executed()

    # ------------------------------------------------------------------
    # 夹爪控制
    # ------------------------------------------------------------------
    def _send_gripper(self, positions: Sequence[float], wait: float):
        """构造一个单点 JointTrajectory 发出去，然后阻塞等到位。

        硬件没有反馈话题时只能用 sleep 简化处理；后面想做精确控制，
        把这里换成订阅 /joint_states 等到位即可。
        """
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        # time_from_start 不能为 0，否则部分控制器会拒收
        point.time_from_start.sec = int(self.gripper_time_sec)
        point.time_from_start.nanosec = int(
            (self.gripper_time_sec - int(self.gripper_time_sec)) * 1e9
        )
        msg.points.append(point)

        self.gripper_pub.publish(msg)
        time.sleep(wait)

    def open_gripper(self, wait: float = None):
        """张开夹爪。wait 默认等夹爪走完轨迹的时间。"""
        wait = self.gripper_time_sec if wait is None else wait
        self._send_gripper(self.gripper_open_val, wait)

    def close_gripper(self, wait: float = None):
        """闭合夹爪。wait 默认等夹爪走完轨迹的时间。"""
        wait = self.gripper_time_sec if wait is None else wait
        self._send_gripper(self.gripper_close_val, wait)
```

## executor_node.py

只改一处：`quaternion_from_euler(0.0, -math.pi, yaw)` → `quaternion_from_euler(math.pi, 0.0, yaw)`，并加注释说明。其他不动。

```python
#!/usr/bin/env python3
"""抓取执行节点：订阅 /grasp_pose，调 MoveIt2 完成抓取。"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from yolov8_grasping.arm_executor import ArmExecutor


class GraspExecutorNode(Node):
    def __init__(self):
        super().__init__("grasp_executor_node")

        # ---------- 参数 ----------
        self.declare_parameter(
            "joint_names",
            ["j1", "j2", "j3", "j4", "j5", "j6"],   # 按 SRDF 改
        )
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("end_effector", "tool0")
        self.declare_parameter("group_name", "robot_arm")
        self.declare_parameter("pregrasp_offset_z", 0.10)
        self.declare_parameter("auto_execute", False)

        joint_names = self.get_parameter("joint_names").value
        base_link = self.get_parameter("base_link").value
        end_effector = self.get_parameter("end_effector").value
        group_name = self.get_parameter("group_name").value
        self.pregrasp_offset_z = self.get_parameter("pregrasp_offset_z").value
        self.auto_execute = self.get_parameter("auto_execute").value

        # ---------- 执行器 ----------
        self.arm = ArmExecutor(
            node=self,
            joint_names=joint_names,
            base_link=base_link,
            end_effector=end_effector,
            group_name=group_name,
        )

        # ---------- 订阅 ----------
        self.busy = False
        self.latest_pose: Optional[PoseStamped] = None

        self.create_subscription(
            PoseStamped, "/grasp_pose", self.cb_grasp_pose, 10)
        self.create_subscription(
            Bool, "/grasp_trigger", self.cb_trigger, 10)

        self.get_logger().info(
            f"Grasp executor ready, auto_execute={self.auto_execute}, "
            f"group={group_name}, ee={end_effector}"
        )

    # ---------- 回调 ----------
    def cb_grasp_pose(self, msg: PoseStamped):
        self.latest_pose = msg
        if self.auto_execute and not self.busy:
            self.execute_grasp(msg)

    def cb_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self.busy:
            self.get_logger().warning("busy, ignore trigger")
            return
        if self.latest_pose is None:
            self.get_logger().warning("no grasp pose yet, ignore trigger")
            return
        self.execute_grasp(self.latest_pose)

    # ---------- 抓取流程 ----------
    def execute_grasp(self, pose_stamped: PoseStamped):
        self.busy = True
        try:
            target = pose_stamped.pose

            # 提取 yaw。
            # grasping_node 发的姿态是 (roll=π, pitch=0, yaw=yaw_obb)，
            # euler_from_quaternion 默认 'sxyz' 静态轴顺序，能正确还原 yaw。
            _, _, yaw = euler_from_quaternion([
                target.orientation.x, target.orientation.y,
                target.orientation.z, target.orientation.w,
            ])

            # 重建姿态时**必须用同一种约定**：roll=π, pitch=0, yaw=yaw。
            # 注意：(0, -π, yaw) 看起来也是"夹爪朝下"，但和 (π, 0, yaw)
            # 差一个绕 z 的 180°，会让 MoveIt 解出完全不同的关节解，
            # 同时跟 grasping_node 的约定不一致，禁用。
            qx, qy, qz, qw = quaternion_from_euler(math.pi, 0.0, yaw)

            pregrasp = Pose()
            pregrasp.position.x = target.position.x
            pregrasp.position.y = target.position.y
            pregrasp.position.z = target.position.z + self.pregrasp_offset_z
            pregrasp.orientation.x = qx
            pregrasp.orientation.y = qy
            pregrasp.orientation.z = qz
            pregrasp.orientation.w = qw

            grasp = Pose()
            grasp.position.x = target.position.x
            grasp.position.y = target.position.y
            grasp.position.z = target.position.z
            grasp.orientation = pregrasp.orientation

            # 1. 张开夹爪 + 关节空间到 pregrasp
            self.get_logger().info("[1/5] open gripper + move to pregrasp")
            self.arm.open_gripper()
            if not self.arm.move_to_pose(pregrasp, cartesian=False):
                self.get_logger().error("pregrasp planning failed")
                return

            # 2. 笛卡尔直线下降
            self.get_logger().info("[2/5] cartesian descent")
            if not self.arm.move_to_pose(grasp, cartesian=True):
                self.get_logger().error("descent failed")
                return

            # 3. 关闭夹爪
            self.get_logger().info("[3/5] close gripper")
            self.arm.close_gripper()

            # 4. 笛卡尔直线上升
            self.get_logger().info("[4/5] cartesian ascent")
            if not self.arm.move_to_pose(pregrasp, cartesian=True):
                self.get_logger().error("ascent failed")
                return

            # 5. 完成
            self.get_logger().info("[5/5] grasp done")
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = GraspExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## 别忘了 package.xml

```xml
<exec_depend>trajectory_msgs</exec_depend>
```

## 改动总结

| 文件                 | 改了什么                                                                                                                                                                                    |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `arm_executor.py`  | 夹爪从 `Float64@/gripper_command` 改为 `JointTrajectory@/hand_controller/joint_trajectory`；`gripper_open/close` 改为列表；新增 `gripper_joint_names`、`gripper_time_sec` 参数；类名确认是 `ArmExecutor`（带 e） |
| `executor_node.py` | `quaternion_from_euler(0, -π, yaw)` → `quaternion_from_euler(π, 0, yaw)`，与 grasping_node 约定统一                                                                                           |

## 验证一下约定真的对齐了

构建跑起来后，可以加一行调试日志确认：

```python
self.get_logger().info(
    f"target yaw={yaw:.3f}, "
    f"input quat=({target.orientation.x:.3f}, {target.orientation.y:.3f}, "
    f"{target.orientation.z:.3f}, {target.orientation.w:.3f}), "
    f"output quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
)
```

如果 input quat 和 output quat 几乎一样，说明 grasping_node 和 executor_node 用的是同一种约定，"取出 yaw 再重建"是恒等操作。如果差很远，说明两边约定不一致，得回去检查 `grasping_node.py` 的 `grasp_quat_top_down`。

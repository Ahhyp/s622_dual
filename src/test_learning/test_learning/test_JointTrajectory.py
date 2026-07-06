import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


"""
1. 推荐方式：通过 FollowJointTrajectory 动作接口
这是工业级和 MoveIt 中最标准、最健壮的方式。动作接口是异步的，能实时反馈执行状态和最终结果。
"""


class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')
        # 1. 创建动作客户端，指向控制器的动作服务器
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'  # 控制器动作名
        )
        
    def send_goal(self):
        # 2. 构建 JointTrajectory 消息
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']  # 名称需与控制器配置一致[reference:13]

        # 3. 创建轨迹点
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0]  # 起始位置
        point1.time_from_start = Duration(sec=0)  # 起始时间为0

        point2 = JointTrajectoryPoint()
        point2.positions = [1.57, -1.57, 0.0]  # 目标位置（弧度）
        point2.time_from_start = Duration(sec=2)  # 2秒后到达

        trajectory_msg.points = [point1, point2]

        # 4. 将轨迹封装到动作的 Goal 中并发送
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def gol_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        # 可以添加获取结果和反馈的回调


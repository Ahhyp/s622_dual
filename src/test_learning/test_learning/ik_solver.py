import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState

class IKSolver(Node):
    def __init__(self):
        super().__init__("ik_solver_client")
        self.client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 IK 服务...')

    def solve_ik(self, target_pose, group_name="arm_group"):
        # 构建请求
        req = GetPositionIK.Request()
        
        # 设置 IK 请求参数
        req.ik_request.group_name = group_name
        req.ik_request.timeout = 5.0          # 5 秒超时
        req.ik_request.avoid_collisions = True
        
        # 封装目标位姿（PoseStamped）
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"   # 参考坐标系
        pose_stamped.pose = target_pose
        req.ik_request.pose_stamped = pose_stamped
        
        # 可选：提供当前机器人状态（若不提供，MoveIt 会使用默认状态）
        # req.ik_request.robot_state = current_robot_state

        # 发送异步请求
        future = self.client.call_async(req)
        # 注册回调（不阻塞）
        future.add_done_callback(self.ik_response_callback)
        return future
    
    def ik_response_callback(self, future):
        try:
            response = future.result()
            if response.ik_valid:
                # 获取关节角度（存储在 response.solution.joint_state.position）
                joints = response.solution.joint_state
                self.get_logger().info(f"IK 成功：关节角度 = {joints.position}")
            else:
                self.get_logger().warn("IK 无解")
        except Exception as e:
            self.get_logger().error(f"IK 调用失败: {e}")
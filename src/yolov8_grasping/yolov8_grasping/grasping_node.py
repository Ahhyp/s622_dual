import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from yolov8_obb_msgs.msg import Yolov8Inference

from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  注册 PoseStamped 的 do_transform

from yolov8_grasping.pose_estimator import (
    PoseEstimator,
    grasp_quat_top_down,
)

class GraspingNode(Node):
    '''
    功能就是： 
        将 yolov8 那边发布在 obb_detections 的信息， 也就是目标所在图中的像素坐标， 变成机械臂的坐标
        同时还要取得相机的参数
        当然， 模块归模块， 节点归节点
    
    grasping_node.py：负责所有 ROS 相关的东西（订阅、TF、发布），算法部分调 PoseEstimator。
    '''
    def __init__(self):
        super().__init__("grasping_node")
        
        # 参数：angle 单位约定 （与 obb 节点保持一致）
        self.declare_parameter("angle_unit", "rad")   # rad / deg
        self.angle_unit = self.get_parameter("angle_unit").value
        
        self.bridge = CvBridge()
        self.estimator = PoseEstimator()
        self.depth_img = None
        self.depth_stamp = None   # 记下深度图的时间戳，做 TF 时用

        
        # tf2_ros::Buffer：一个坐标系变换的缓存库。它会在内存中保存所有已知的坐标系之间的变换关系
        # （比如从 camera_color_optical_frame 到 base_link 的平移 + 旋转）。你可以向它查询任意两个坐标系
        # 之间的变换，或者直接将一个坐标系下的位姿 / 点变换到另一个坐标系。
        self.tf_buffer = Buffer()
        # tf2_ros::TransformListener：一个订阅者，它会自动订阅 ROS 话题 /tf 和 /tf_static，
        # 持续接收广播出来的 TF 变换数据，并实时更新到 Buffer 中。这样 Buffer 里总是有最新的变换关系。
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self.cb_info, 10)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self.cb_depth, 10)
        self.create_subscription(Yolov8Inference, "/yolov8/obb_detections",
                                 self.cb_det, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/grasp_pose", 10)

        self.get_logger().info(
            f"Grasping node started (top-down grasp, angle_unit={self.angle_unit})"
        )
                
    def cb_info(self, msg):
        # callback , 就是说当上面的订阅收到信息就执行这玩意
        self.estimator.set_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def cb_depth(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_stamp = msg.header.stamp


    def cb_det(self, msg):
        if self.depth_img is None:
            return
        target = max(msg.results, key=lambda r: r.confidence, default=None)
        if target is None:
            return
        
        # 1. 像素 → 相机系（调算法层）
        xyz = self.estimator.pixel_to_camera(
            target.center_x, target.center_y, self.depth_img)
        if xyz is None:
            self.get_logger().warning("invalid depth at target pixel")
            return

        # 2. 相机系 → base_link（ROS 层）
        # 2.1 设置变量
        pose_cam = PoseStamped()
        pose_cam.header.frame_id = "camera_color_optical_frame"
        pose_cam.header.stamp = self.depth_stamp
        pose_cam.pose.position.x, pose_cam.pose.position.y, pose_cam.pose.position.z = xyz
        pose_cam.pose.orientation.w = 1.0

        try:
            # 通过调用TF变换 完成 相机系 → base_link
            pose_base = self.tf_buffer.transform(pose_cam, "base_link",timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        
        # 3. 俯视抓取姿态  就是加上了 obb 旋转角姿态
        yaw = float(target.angle)
        if self.angle_unit == "deg":
            yaw = math.radians(yaw)
        qx, qy, qz, qw = grasp_quat_top_down(yaw)
        pose_base.pose.orientation.x = qx
        pose_base.pose.orientation.y = qy
        pose_base.pose.orientation.z = qz
        pose_base.pose.orientation.w = qw
        
        self.pose_pub.publish(pose_base)




            

def main():
    rclpy.init()
    node = GraspingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
    
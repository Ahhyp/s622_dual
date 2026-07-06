#!/usr/bin/env python3
"""VisualAlign Action server。

支持 5 mode：
  - descend / lift / retreat: 开环笛卡尔
  - xy: IBVS 像素对齐（M1.5.4）
  - yaw: EE yaw 角度对齐（M1.5.5）

发布 TwistStamped 到 /servo_node/delta_twist_cmds，控制频率 30Hz。
"""
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image, CameraInfo, JointState
from yolov8_obb_msgs.msg import Yolov8Inference

from tf2_ros import Buffer, TransformListener

from s622_bt_manager.action import VisualAlign
from s622_arm_actions.servo_lifecycle import ServoLifecycleManager
from std_msgs.msg import Int8

CONTROL_RATE_HZ = 30.0


class VisualAlignServer(Node):
    def __init__(self):
        super().__init__('visual_align_server')

        # ---- 参数 ----
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'grasp_frame')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('twist_topic', '/servo_node/delta_twist_cmds')

        # IBVS Jacobian fallback (m/px, base frame XY <- image uv)
        self.declare_parameter('j_img_to_base',
            [-0.001939, 0.000069, 0.0, 0.001957])  # 2x2 row-major
        self.declare_parameter('xy_kp', 1.0)
        self.declare_parameter('xy_max_speed', 0.05)  # m/s
        self.declare_parameter('xy_min_pixel_err_to_move', 2.0)

        self.declare_parameter('yaw_kp', 1.0)
        self.declare_parameter('yaw_max_speed', 0.5)  # rad/s

        self.declare_parameter('descend_max_speed', 0.05)
        self.declare_parameter('lift_max_speed', 0.05)

        self._base = self.get_parameter('base_frame').value
        self._ee = self.get_parameter('ee_frame').value
        self._cam = self.get_parameter('camera_frame').value
        self._twist_topic = self.get_parameter('twist_topic').value
        j = self.get_parameter('j_img_to_base').value
        self._j = [[float(j[0]), float(j[1])], [float(j[2]), float(j[3])]]

        cb = ReentrantCallbackGroup()
        self.servo_lc = ServoLifecycleManager(self, callback_group=cb)

        self.twist_pub = self.create_publisher(
            TwistStamped, self._twist_topic, 10)

        # subscriptions for IBVS
        self._yolo_lock = threading.Lock()
        self._latest_yolo: Optional[Yolov8Inference] = None
        self.create_subscription(
            Yolov8Inference, '/yolov8/obb_detections',
            self._on_yolo, 10, callback_group=cb)

        self._js_lock = threading.Lock()
        self._latest_js: Optional[JointState] = None
        self.create_subscription(
            JointState, '/joint_states',
            self._on_js, 10, callback_group=cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._cancel_request = threading.Event()

        self._action_server = ActionServer(
            self, VisualAlign, 'visual_align',
            execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=self._on_cancel,
            callback_group=cb,
        )
    
        self._servo_status: int = 0
        self._servo_status_lock = threading.Lock()
        self.create_subscription(
            Int8, '/servo_node/status',
            self._on_servo_status, 10, callback_group=cb)
        
        self.get_logger().info(
            f'visual_align ready: twist_topic={self._twist_topic}')

        # 启动时自动 start + unpause servo（延迟等 servo_node 就绪）
        self._auto_start_timer = self.create_timer(3.0, self._auto_start_servo,
                                                    callback_group=cb)

    def _auto_start_servo(self):
        self._auto_start_timer.cancel()
        if self.servo_lc.start_servo():
            self.get_logger().info('servo auto-started on init')
        else:
            self.get_logger().warning('servo auto-start failed, will retry on first action')

    # ---- callbacks ----
    def _on_yolo(self, msg):
        with self._yolo_lock:
            self._latest_yolo = msg

    def _on_js(self, msg):
        with self._js_lock:
            self._latest_js = msg

    def _on_cancel(self, goal_handle):
        self._cancel_request.set()
        return CancelResponse.ACCEPT

    
    def _on_servo_status(self, msg):
        with self._servo_status_lock:
            self._servo_status = msg.data
        
    def _servo_status_name(self):
        names = {
            0: 'NO_WARNING',
            1: 'DECELERATE_FOR_APPROACHING_SINGULARITY',
            2: 'HALT_FOR_SINGULARITY',
            3: 'DECELERATE_FOR_COLLISION',
            4: 'HALT_FOR_COLLISION',
            5: 'DECELERATE_FOR_LEAVING_SINGULARITY',
            # MoveIt Humble: JOINT_BOUND 在不同版本枚举不同,可能是 6/7
        }
        with self._servo_status_lock:
            return f'{self._servo_status}({names.get(self._servo_status, "UNKNOWN")})'
    
    
    # ---- helpers ----
    def _publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base
        self.twist_pub.publish(msg)

    def _publish_twist(self, vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = float(wx)
        msg.twist.angular.y = float(wy)
        msg.twist.angular.z = float(wz)
        self.twist_pub.publish(msg)

    def _get_ee_position(self):
        """返回 grasp_frame 在 base 系的 (x,y,z)。失败返回 None。"""
        try:
            t = self.tf_buffer.lookup_transform(
                self._base, self._ee, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2))
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return None

    # ---- main execute ----
    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = VisualAlign.Result()
        self._cancel_request.clear()

        if goal.ensure_servo_started:
            if not self.servo_lc.start_servo():
                result.success = False
                result.error_msg = 'failed to start servo'
                goal_handle.abort()
                return result

        # 如果 servo 卡在保护态(非 0), 强制重置
        with self._servo_status_lock:
            s = self._servo_status
        if s != 0:
            self.get_logger().warning(
                f'servo status={self._servo_status_name()}, force restarting...')
            self.servo_lc.force_stop()
            self.servo_lc.start_servo()
            self.get_logger().info('servo restarted')

        try:
            if goal.mode == 'descend':
                ok, dist, msg = self._run_descend(goal, goal_handle,
                                                   direction=-1.0)
                result.actual_distance = dist
            elif goal.mode == 'lift':
                ok, dist, msg = self._run_descend(goal, goal_handle,
                                                   direction=+1.0)
                result.actual_distance = dist
            elif goal.mode == 'retreat':
                ok, dist, msg = self._run_descend(goal, goal_handle,
                                                   direction=+1.0)
                result.actual_distance = dist
            elif goal.mode == 'xy':
                ok, err, msg = self._run_align_xy(goal, goal_handle)
                result.final_error = err
            elif goal.mode == 'yaw':
                ok, err, msg = self._run_align_yaw(goal, goal_handle)
                result.final_error = err
            else:
                ok, msg = False, f'unknown mode: {goal.mode}'
        finally:
            self._publish_zero()  # 总是停下来

        result.success = ok
        result.error_msg = msg
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ---- mode: descend/lift/retreat ----
    def _run_descend(self, goal, goal_handle, direction: float):
        distance = float(goal.distance)
        speed = float(goal.speed) if goal.speed > 0 else 0.04
        timeout = float(goal.timeout_sec) if goal.timeout_sec > 0 else 30.0

        start_xyz = self._get_ee_position()
        if start_xyz is None:
            return False, 0.0, 'failed to get start EE position'
        z0 = start_xyz[2]
        z_target = z0 + direction * distance

        self.get_logger().info(
            f'{goal.mode}: z0={z0:.4f} target={z_target:.4f} '
            f'dist={distance:.3f} speed={speed:.3f} timeout={timeout:.1f}')

        rate = self.create_rate(CONTROL_RATE_HZ)
        t_start = self.get_clock().now()
        last_log_t = t_start
        last_log_z = z0
        stuck_ref_z = z0
        last_stuck_check = t_start
        stuck_count = 0

        while rclpy.ok():
            if self._cancel_request.is_set():
                self._publish_zero()
                return False, 0.0, 'cancelled'

            now = self.get_clock().now()
            elapsed = (now - t_start).nanoseconds / 1e9
            if elapsed > timeout:
                cur = self._get_ee_position()
                cur_z = cur[2] if cur else z0
                self._publish_zero()
                self.get_logger().error(
                    f'{goal.mode} TIMEOUT after {elapsed:.1f}s: '
                    f'z={cur_z:.4f} remain={z_target-cur_z:+.4f} '
                    f'servo_status={self._servo_status_name()}')
                return False, abs(cur_z - z0), 'timeout'

            cur = self._get_ee_position()
            if cur is None:
                rate.sleep()
                continue
            cur_z = cur[2]
            remain = z_target - cur_z

            if abs(remain) < 0.002:
                self._publish_zero()
                self.get_logger().info(
                    f'{goal.mode} DONE: t={elapsed:.2f}s '
                    f'z={cur_z:.4f} (target={z_target:.4f}) '
                    f'avg_speed={abs(cur_z-z0)/elapsed*1000:.1f}mm/s')
                return True, abs(cur_z - z0), ''

            # ===== 1 Hz progress logger (替换原 publish_feedback 闲置) =====
            if (now - last_log_t).nanoseconds / 1e9 >= 1.0:
                dt = (now - last_log_t).nanoseconds / 1e9
                dz = cur_z - last_log_z
                inst_speed_mmps = abs(dz) / dt * 1000.0
                cmd_speed_mmps = speed * 1000.0
                ratio = inst_speed_mmps / cmd_speed_mmps if cmd_speed_mmps > 0 else 0
                self.get_logger().info(
                    f'{goal.mode} t={elapsed:4.1f}s z={cur_z:.4f} '
                    f'remain={remain:+.4f} '
                    f'v={inst_speed_mmps:5.1f}/{cmd_speed_mmps:.1f}mm/s '
                    f'({ratio*100:.0f}%) '
                    f'servo={self._servo_status_name()}')
                last_log_t = now
                last_log_z = cur_z

                # feedback for action client
                fb = VisualAlign.Feedback()
                fb.current_error = abs(remain)
                fb.elapsed_sec = elapsed
                goal_handle.publish_feedback(fb)

            # ===== stuck 检测 (1.5s 移动 < 1.5mm,比原来严) =====
            if (now - last_stuck_check).nanoseconds / 1e9 >= 1.5:
                moved = abs(cur_z - stuck_ref_z)
                if moved < 0.0015:
                    stuck_count += 1
                    if stuck_count >= 2:
                        self._publish_zero()
                        self.get_logger().error(
                            f'{goal.mode} STUCK: moved {moved*1000:.2f}mm in 3s '
                            f'at z={cur_z:.4f} '
                            f'servo_status={self._servo_status_name()}')
                        return False, abs(cur_z - z0), (
                            f'EE stuck (moved {moved*1000:.1f}mm in 3s, '
                            f'servo={self._servo_status_name()})')
                else:
                    stuck_count = 0
                stuck_ref_z = cur_z
                last_stuck_check = now

            vz = math.copysign(min(speed, abs(remain) * 2.0), remain)
            self._publish_twist(vz=vz)
            rate.sleep()

        self._publish_zero()
        return False, 0.0, 'aborted'



    # ---- mode: xy  ----
    def _run_align_xy(self, goal, goal_handle):
        target_x = float(goal.target_x_base)
        target_y = float(goal.target_y_base)
        # 防护
        if abs(target_x) < 0.1 and abs(target_y) < 0.1:
            return False, 0.0, (
                f'target_xy=({target_x:.3f},{target_y:.3f}) too close to base origin. '
                'Likely missing blackboard input.')
    
        tol = float(goal.tolerance_m) if goal.tolerance_m > 0 else 0.005  # 5mm
        timeout = float(goal.timeout_sec) if goal.timeout_sec > 0 else 25.0
        kp = float(self.get_parameter('xy_kp').value)
        max_speed = float(self.get_parameter('xy_max_speed').value)

        self.get_logger().info(
            f'align_xy: target_base=({target_x:.3f},{target_y:.3f}) tol={tol*1000:.1f}mm')

        rate = self.create_rate(CONTROL_RATE_HZ)
        t_start = self.get_clock().now()
        last_log = t_start
        stable = 0
        last_err = float('inf')

        while rclpy.ok():
            if self._cancel_request.is_set():
                return False, last_err, 'cancelled'
            elapsed = (self.get_clock().now() - t_start).nanoseconds / 1e9
            if elapsed > timeout:
                return False, last_err, 'timeout'

            cur = self._get_ee_position()
            if cur is None:
                rate.sleep()
                continue
            dx = target_x - cur[0]
            dy = target_y - cur[1]
            err = math.hypot(dx, dy)
            last_err = err

            if err < tol:
                stable += 1
                if stable >= 5:
                    self._publish_zero()
                    self.get_logger().info(f'align_xy converged: err={err*1000:.1f}mm')
                    return True, err, ''
            else:
                stable = 0

            vx = kp * dx
            vy = kp * dy
            v_norm = math.hypot(vx, vy)
            if v_norm > max_speed:
                vx = vx / v_norm * max_speed
                vy = vy / v_norm * max_speed

            self._publish_twist(vx=vx, vy=vy)

            if (self.get_clock().now() - last_log).nanoseconds / 1e9 > 0.5:
                self.get_logger().info(
                    f'align_xy: err={err*1000:.1f}mm dxy=({dx*1000:.1f},{dy*1000:.1f})')
                fb = VisualAlign.Feedback()
                fb.current_error = err
                fb.elapsed_sec = elapsed
                goal_handle.publish_feedback(fb)
                last_log = self.get_clock().now()
            rate.sleep()
        return False, last_err, 'aborted'


    # ---- mode: yaw ----
    def _run_align_yaw(self, goal, goal_handle):
        target_yaw = float(goal.target_yaw)
        tol = float(goal.tolerance_rad) if goal.tolerance_rad > 0 else 0.05
        timeout = float(goal.timeout_sec) if goal.timeout_sec > 0 else 10.0
        kp = float(self.get_parameter('yaw_kp').value)
        max_speed = float(self.get_parameter('yaw_max_speed').value)

        rate = self.create_rate(CONTROL_RATE_HZ)
        t_start = self.get_clock().now()
        last_log = t_start
        stable = 0
        last_err = float('inf')

        self.get_logger().info(
            f'align_yaw: target={target_yaw:.3f} tol={tol:.3f} '
            f'kp={kp:.1f} max_wz={max_speed:.2f} timeout={timeout:.1f}')

        while rclpy.ok():
            if self._cancel_request.is_set():
                return False, last_err, 'cancelled'
            elapsed = (self.get_clock().now() - t_start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().error(
                    f'align_yaw TIMEOUT after {elapsed:.1f}s: '
                    f'last_err={last_err:.3f}')
                return False, last_err, 'timeout'

            # 读 EE yaw
            try:
                t = self.tf_buffer.lookup_transform(
                    self._base, self._ee, rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.2))
                q = t.transform.rotation
                # RPY from quat
                import tf_transformations
                _, _, cur_yaw = tf_transformations.euler_from_quaternion(
                    [q.x, q.y, q.z, q.w])
            except Exception as e:
                self.get_logger().error(
                    f'yaw TF lookup failed: {e}', throttle_duration_sec=2.0)
                rate.sleep()
                continue

            err = (target_yaw - cur_yaw + math.pi) % (2 * math.pi) - math.pi
            last_err = err

            if abs(err) <= tol:
                stable += 1
                if stable >= 5:
                    self._publish_zero()
                    self.get_logger().info(
                        f'align_yaw converged: err={abs(err):.3f}')
                    return True, abs(err), ''
            else:
                stable = 0

            wz = max(min(kp * err, max_speed), -max_speed)
            # 防止 wz 太小被 servo 衰减到 0
            if abs(wz) < 0.02 and abs(err) > tol:
                wz = 0.02 if wz >= 0 else -0.02
            self._publish_twist(wz=wz)

            # 0.5 Hz 状态日志
            now = self.get_clock().now()
            if (now - last_log).nanoseconds / 1e9 >= 0.5:
                self.get_logger().info(
                    f'align_yaw: t={elapsed:.1f}s cur={cur_yaw:.3f} '
                    f'err={err:+.3f} wz={wz:+.3f} '
                    f'stable={stable}/5')
                fb = VisualAlign.Feedback()
                fb.current_error = float(abs(err))
                fb.elapsed_sec = float(elapsed)
                goal_handle.publish_feedback(fb)
                last_log = now

            rate.sleep()
        return False, last_err, 'aborted'


def main():
    rclpy.init()
    node = VisualAlignServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
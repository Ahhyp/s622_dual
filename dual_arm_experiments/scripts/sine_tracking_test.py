#!/usr/bin/env python3
"""
sine_tracking_test.py — ServoJ 动态跟踪测试（单臂 / 双臂同步）

单臂：-p joints/controller/controller_joints/...（一个 controller）
双臂同步：-p controllers_yaml:=<yaml>（多个 controller 定义，同一时刻同步发送 goal）

记录 cmd（发送轨迹点）与 actual（/joint_states），写 CSV：
  - 单 controller：直接写 csv_path
  - 多 controller：写 <csv_base>__<controller>.<ext>（每个 controller 一个文件）

设计要点（docs/2026-08-25_ServoJ动态跟踪与同步性能测试）：
  - 轨迹 q(t)=q0+A*sin(2πft)，waypoint 密度 = trajectory_sample_rate_hz（不是发送频率，
    整条轨迹一次交给 JTC，JTC 在 update loop 插值执行）
  - 双臂同步：所有 controller 的 trajectory.header.stamp 设为同一未来 T0（now+start_delay_sec），
    JTC 到 T0 同时起跑；cmd 时间基准 = t_start（T0），消除 action 传输/接收时差污染
  - 安全门槛：v_max=2πfA / a_max=(2πf)²A 超限拒绝发送（真机硬性）
  - use_sim_time=true 时等待 /clock 同步后再取基准时间

用法（source 后）：
  # 单臂
  python3 scripts/sine_tracking_test.py --ros-args \
    -p joints:='[j1]' -p controller_joints:='[j1,j2,j3,j4,j5,j6]' \
    -p frequency_hz:=0.5 -p duration_sec:=20.0 -p controller:=/robot_arm_controller \
    -p trajectory_sample_rate_hz:=200.0 -p csv_path:=results/x.csv -p use_sim_time:=true
  # 双臂同步
  python3 scripts/sine_tracking_test.py --ros-args \
    -p controllers_yaml:=config/dual_sync_test.yaml \
    -p frequency_hz:=0.5 -p duration_sec:=20.0 -p trajectory_sample_rate_hz:=100.0 \
    -p csv_path:=results/gazebo/dual_j1_0.5hz.csv -p use_sim_time:=true
"""
import csv
import math
import os
import sys
import threading
import time

import yaml as _yaml

import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


class ControllerRun:
    """一个 controller 的测试：运动关节(joints)正弦，其余(ctrl_joints)保持当前值。"""

    def __init__(self, name, joints, ctrl_joints, node):
        self.name = name
        self.joints = list(joints)
        self.ctrl_joints = list(ctrl_joints)
        self.client = ActionClient(
            node, FollowJointTrajectory, f"{name}/follow_joint_trajectory")
        self.traj = None
        self.cmd_records = []       # (t_abs, {joint: pos})
        self.actual_records = []    # (t_abs, {joint: pos})
        self.result_future = None   # get_result_async 的 future
        self.result_status = 'NONE'


class SineTrackingTest(Node):
    def __init__(self):
        super().__init__('sine_tracking_test')

        self.declare_parameter('joints', ['left_j1'])
        # controller 全部关节（轨迹必须覆盖全部，JTC allow_partial_joints_goal=false）
        self.declare_parameter('controller_joints', [''])   # [''] 占位防 BYTE_ARRAY 推断
        self.declare_parameter('controllers_yaml', '')      # 多 controller（双臂同步）配置
        self.declare_parameter('amplitude_rad', 0.05)
        self.declare_parameter('frequency_hz', 0.5)
        self.declare_parameter('duration_sec', 20.0)
        self.declare_parameter('center_rad', float('nan'))  # NaN=读当前，数值=固定中心
        self.declare_parameter('controller', '/robot_arm_controller')
        # trajectory_sample_rate_hz：轨迹 waypoint 密度。注意这不是"发送频率"——
        # 整条 JointTrajectory 一次交给 JTC，JTC 在 update loop 插值执行。
        self.declare_parameter('trajectory_sample_rate_hz', 200.0)
        self.declare_parameter('csv_path', 'results.csv')
        self.declare_parameter('joint_states_topic', '/joint_states')
        # 统一开始时刻 T0 = now + start_delay_sec（双臂同步：所有 controller 同刻起跑）
        self.declare_parameter('start_delay_sec', 1.5)
        # 安全门槛（真机硬性）：v_max=2πfA、a_max=(2πf)²A 超限拒绝发送
        self.declare_parameter('velocity_limit_rad_s', 3.15)  # URDF j1~j3 限速
        self.declare_parameter('accel_limit_rad_s2', 0.0)     # 0=不检查（真机按实际伺服能力设）
        # CSV 只保留 T0 前后窗口（pre_roll ~ duration+post_roll）
        self.declare_parameter('pre_roll_sec', 1.0)
        self.declare_parameter('post_roll_sec', 1.0)
        # use_sim_time 由 rclpy 自动声明，不在此 declare（运行时 -p use_sim_time:=true/false）
        # warmup 周期由分析脚本 --warmup 处理（脚本不统计，仅生成轨迹）

        self._amp = self.get_parameter('amplitude_rad').value
        self._freq = self.get_parameter('frequency_hz').value
        self._dur = self.get_parameter('duration_sec').value
        self._center = self.get_parameter('center_rad').value
        self._cmd_rate = self.get_parameter('trajectory_sample_rate_hz').value
        self._csv = self.get_parameter('csv_path').value
        self._js_topic = self.get_parameter('joint_states_topic').value

        # ---- 构造 ControllerRun 列表 ----
        cy = self.get_parameter('controllers_yaml').value
        if cy:
            with open(cy) as f:
                cfg = _yaml.safe_load(f)
            self._runs = [ControllerRun(c['name'], c['joints'], c['ctrl_joints'], self)
                          for c in cfg['controllers']]
        else:
            joints = self.get_parameter('joints').value
            ctrl_joints = self.get_parameter('controller_joints').value
            if not ctrl_joints or ctrl_joints == ['']:
                ctrl_joints = list(joints)
            self._runs = [ControllerRun(self.get_parameter('controller').value,
                                        joints, ctrl_joints, self)]

        # ---- 订阅 joint_states（记录所有关节，写 CSV 时按 controller 过滤）----
        self._actual_all = []       # (t_abs, {joint: pos})
        self._last_js = None
        self._js_event = threading.Event()
        self._sub = self.create_subscription(
            JointState, self._js_topic, self._js_cb, 100)

        # 后台 executor：负责所有 ROS 回调
        self._exec = MultiThreadedExecutor(2)
        self._exec.add_node(self)
        threading.Thread(target=self._exec.spin, daemon=True).start()

    def _js_cb(self, msg):
        d = {n: p for n, p in zip(msg.name, msg.position)}
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._actual_all.append((t, d))
        self._last_js = d
        self._js_event.set()

    # ------------------------------------------------------------------
    def _build_trajectory(self, ctrl_joints, move_joints, centers):
        traj = JointTrajectory()
        traj.joint_names = list(ctrl_joints)
        n = int(self._dur * self._cmd_rate) + 1
        for i in range(n):
            t = i / self._cmd_rate
            pt = JointTrajectoryPoint()
            pt.positions = [
                centers[j] + self._amp * math.sin(2.0 * math.pi * self._freq * t)
                if j in move_joints else centers[j]
                for j in ctrl_joints
            ]
            # 整数纳秒，避免浮点边界
            t_ns = round(i * 1e9 / self._cmd_rate)
            pt.time_from_start = Duration(sec=t_ns // 1_000_000_000,
                                          nanosec=t_ns % 1_000_000_000)
            traj.points.append(pt)
        return traj

    def _wait_clock(self, wall_timeout=5.0):
        """use_sim_time 下 /clock 首条消息可能延迟 2-3s，等待 sim 时钟同步。"""
        t0 = time.time()
        while self.get_clock().now().nanoseconds == 0 and time.time() - t0 < wall_timeout:
            time.sleep(0.1)

    def _collect_centers(self):
        """所有 controller 关节中心值：运动关节固定 center（若给定）或当前值，其余当前值。"""
        if not self._js_event.wait(timeout=10.0):
            self.get_logger().error(
                f"no joint_states on '{self._js_topic}' — is the system running?")
            sys.exit(2)
        centers = {}
        for r in self._runs:
            for j in r.ctrl_joints:
                if j not in self._last_js:
                    self.get_logger().error(
                        f"joint '{j}' not in joint_states (have {sorted(self._last_js)})")
                    sys.exit(2)
                centers[j] = self._last_js[j]
        if isinstance(self._center, float) and not math.isnan(self._center):
            for r in self._runs:
                for j in r.joints:
                    centers[j] = self._center
        return centers

    # ------------------------------------------------------------------
    def run(self):
        centers = self._collect_centers()
        self.get_logger().info(
            f"controllers={[r.name for r in self._runs]} "
            f"move_joints={[r.joints for r in self._runs]} "
            f"A={self._amp} rad f={self._freq} Hz dur={self._dur} s "
            f"cmd_rate={self._cmd_rate} Hz")
        self.get_logger().info(
            f"peak velocity={2*math.pi*self._freq*self._amp:.3f} rad/s "
            f"peak accel={(2*math.pi*self._freq)**2*self._amp:.3f} rad/s^2")

        # 安全门槛：超限拒绝发送（真机硬门槛；v_max 对比 URDF 限速，a_max 由用户按伺服能力设）
        v_max = 2 * math.pi * self._freq * self._amp
        a_max = (2 * math.pi * self._freq) ** 2 * self._amp
        v_lim = self.get_parameter('velocity_limit_rad_s').value
        a_lim = self.get_parameter('accel_limit_rad_s2').value
        if v_lim > 0 and v_max > v_lim:
            self.get_logger().error(
                f"peak velocity {v_max:.3f} > limit {v_lim} rad/s — REFUSED")
            sys.exit(4)
        if a_lim > 0 and a_max > a_lim:
            self.get_logger().error(
                f"peak accel {a_max:.3f} > limit {a_lim} rad/s^2 — REFUSED")
            sys.exit(4)

        # 等所有 action server
        for r in self._runs:
            if not r.client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(
                    f"action server {r.name}/follow_joint_trajectory not available")
                sys.exit(2)

        # 构建轨迹
        for r in self._runs:
            r.traj = self._build_trajectory(r.ctrl_joints, r.joints, centers)

        self._wait_clock()
        # 统一未来开始时刻 T0 = now + start_delay：所有 controller 的轨迹 header.stamp 相同，
        # JTC 会等到 T0 同时起跑（消除 action 传输/接收时差对双臂同步测量的污染）
        start_delay = self.get_parameter('start_delay_sec').value
        start_time = self.get_clock().now() + RclpyDuration(seconds=start_delay)
        t_start = start_time.nanoseconds * 1e-9      # cmd 时间基准（不是发送时刻）
        for r in self._runs:
            r.traj.header.stamp = start_time.to_msg()
        self.get_logger().info(
            f"unified start time T0 = {t_start:.3f} s (now + 0.5s)")

        # ---- 发送：同一循环内对所有 controller send_goal_async ----
        goals = {}
        for r in self._runs:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = r.traj
            goals[r.name] = (r, r.client.send_goal_async(goal))
            # cmd 记录（以 t_start 为基准）
            for i, pt in enumerate(r.traj.points):
                tt = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                r.cmd_records.append(
                    (t_start + tt, {j: pt.positions[k] for k, j in enumerate(r.ctrl_joints)}))
        self.get_logger().info(
            f"sending {len(self._runs)} goals (execution starts at T0={t_start:.3f}s) ...")

        # 等待全部 accepted，然后取 result future
        self._sync_ok = True
        for name, (r, fut) in goals.items():
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 10.0:
                time.sleep(0.05)
            if not fut.done() or not fut.result() or not fut.result().accepted:
                self.get_logger().error(f"goal rejected by {name}")
                sys.exit(3)
            r.result_future = fut.result().get_result_async()
        # 所有 goal accepted 后，确认尚未跨过 T0（否则双臂同步"同时起跑"不成立）
        if self.get_clock().now().nanoseconds >= start_time.nanoseconds:
            self.get_logger().error(
                "goal acceptance crossed T0 — synchronization test INVALID")
            self._sync_ok = False
        self.get_logger().info("all goals accepted, executing ...")

        # 等待完成：全部 result 返回 或 actual 覆盖 duration 或 wall 上限
        t0 = time.time()
        deadline_wall = t0 + max(180.0, 5.0 * self._dur)
        done = False
        while time.time() < deadline_wall:
            if all(r.result_future.done() for r in self._runs):
                done = True
                break
            if len(self._actual_all) > 100:
                if self._actual_all[-1][0] - t_start >= self._dur - 0.1:
                    done = True
                    break
            time.sleep(0.1)
        if done:
            self.get_logger().info("execution completed / fully covered by feedback")
        else:
            self.get_logger().warn("timed out waiting for execution (partial data)")
            # 超时：cancel 未完成的 goal（真机必须，防止机械臂继续跑）。
            # 用 goal handle 的公开 API，不用 client 私有方法
            for name, (r, fut) in goals.items():
                if not r.result_future.done():
                    gh = fut.result()
                    if gh is not None and gh.accepted:
                        gh.cancel_goal_async()

        # 数据覆盖后 JTC 的 action result 可能稍晚返回：补等最多 5s 再判结果
        t_wait = time.time()
        while (not all(r.result_future.done() for r in self._runs)
               and time.time() - t_wait < 5.0):
            time.sleep(0.1)

        # 检查各 controller 的 result 状态：未完成/非 SUCCESS 都要标 INVALID
        all_success = True
        for r in self._runs:
            if r.result_future is None or not r.result_future.done():
                all_success = False
                r.result_status = 'NO_RESULT'
                self.get_logger().error(
                    f"{r.name}: no terminal action result — data INVALID")
                continue
            wrapped = r.result_future.result()
            if wrapped is None or wrapped.result is None:
                all_success = False
                r.result_status = 'EMPTY_RESULT'
                self.get_logger().error(f"{r.name}: empty result — data INVALID")
                continue
            code = wrapped.result.error_code
            ok = (code == FollowJointTrajectory.Result.SUCCESSFUL)
            r.result_status = f"status={wrapped.status} error_code={code}"
            if not ok:
                all_success = False
                self.get_logger().error(
                    f"{r.name}: NOT successful — {r.result_status} "
                    f"({wrapped.result.error_string})")
        if not self._sync_ok:
            all_success = False
        if not all_success:
            self.get_logger().error(
                "some controller(s) did NOT finish SUCCESS — data marked INVALID")

        # 分发 actual（snapshot 固定实验数据；只保留 T0 前后窗口）
        pre_roll = self.get_parameter('pre_roll_sec').value
        post_roll = self.get_parameter('post_roll_sec').value
        actual_snapshot = list(self._actual_all)
        for t, d in actual_snapshot:
            if t < t_start - pre_roll or t > t_start + self._dur + post_roll:
                continue
            for r in self._runs:
                r.actual_records.append((t, {j: d[j] for j in r.ctrl_joints if j in d}))

        # 写 CSV（多 controller 时每 controller 一个文件）
        multi = len(self._runs) > 1
        for r in self._runs:
            path = self._csv
            if multi:
                base, ext = self._csv.rsplit('.', 1)
                path = f"{base}__{r.name.strip('/').replace('/', '_')}.{ext}"
            if not all_success:
                path = path.replace('.csv', '_INVALID.csv')
            self._write_csv(r, path)
            self.get_logger().info(
                f"CSV written: {path} (cmd={len(r.cmd_records)}, actual={len(r.actual_records)}, "
                f"{r.result_status})")

    def _write_csv(self, run, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['type', 't_rel', 't_abs'] + run.ctrl_joints)
            for t, d in run.cmd_records:
                w.writerow(['cmd', f"{t - self._t0_base():.6f}", f"{t:.6f}"] +
                           [f"{d.get(j, float('nan')):.6f}" for j in run.ctrl_joints])
            for t, d in run.actual_records:
                w.writerow(['actual', f"{t - self._t0_base():.6f}", f"{t:.6f}"] +
                           [f"{d.get(j, float('nan')):.6f}" for j in run.ctrl_joints])

    def _t0_base(self):
        """CSV t_rel 基准：记录第一条 cmd 的时间（各 controller 文件统一）。"""
        if not hasattr(self, '_base_ts'):
            ts = [r.cmd_records[0][0] for r in self._runs if r.cmd_records]
            self._base_ts = min(ts) if ts else 0.0
        return self._base_ts


def main(args=None):
    rclpy.init(args=args)
    node = SineTrackingTest()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        try:
            node._exec.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

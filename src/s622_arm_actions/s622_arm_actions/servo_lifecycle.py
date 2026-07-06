"""单点管理 MoveIt Servo 的 start/stop 状态。

避免多个 Action server 重复调用 /servo_node/start_servo 或 /stop_servo。
所有 servo 状态变更必须通过这个类。
"""
import threading
import time
from typing import Optional

from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from std_srvs.srv import Trigger


class ServoLifecycleManager:
    STOPPED = 'STOPPED'
    RUNNING = 'RUNNING'

    def __init__(self, node: Node, callback_group: Optional[CallbackGroup] = None):
        self.node = node
        cb = callback_group or ReentrantCallbackGroup()
        self.state = self.STOPPED
        self._lock = threading.Lock()
        self.start_cli = node.create_client(
            Trigger, '/servo_node/start_servo', callback_group=cb)
        self.stop_cli = node.create_client(
            Trigger, '/servo_node/stop_servo', callback_group=cb)

    def _call(self, client, name: str, timeout: float = 3.0) -> bool:
        if not client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warning(f'{name} service unavailable')
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.02)
        if not future.done():
            self.node.get_logger().warning(f'{name}: timeout')
            return False
        res = future.result()
        ok = res is not None and res.success
        msg = res.message if res else 'no result'
        self.node.get_logger().info(f'{name}: {"ok" if ok else "failed"} ({msg})')
        return ok

    def start_servo(self) -> bool:
        with self._lock:
            if self.state == self.RUNNING:
                return True
            ok = self._call(self.start_cli, 'start_servo')
            if ok:
                self.state = self.RUNNING
                # Humble 有些版本 start 后可能处于 paused 态，显式 unpause
                unpause_cli = self.node.create_client(
                    Trigger, '/servo_node/unpause_servo')
                self._call(unpause_cli, 'unpause_servo', timeout=1.0)
            return ok

    def stop_servo(self) -> bool:
        with self._lock:
            if self.state == self.STOPPED:
                return True
            ok = self._call(self.stop_cli, 'stop_servo')
            if ok:
                self.state = self.STOPPED
            return ok

    def force_stop(self) -> bool:
        """无条件 stop，用于 Recovery 路径。"""
        with self._lock:
            ok = self._call(self.stop_cli, 'force_stop_servo')
            self.state = self.STOPPED
            return ok
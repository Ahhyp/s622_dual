#!/usr/bin/env python3
"""PlanningScene attach/detach Service via /planning_scene topic (is_diff=true)."""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, PlanningScene)

from s622_bt_manager.srv import AttachObject, DetachObject


class PlanningSceneService(Node):
    def __init__(self):
        super().__init__('planning_scene_service')

        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('default_object_size', [0.04, 0.04, 0.04])
        self.declare_parameter('default_touch_links',
                               ['finger1', 'finger2', 'grasp_frame'])
        self.declare_parameter('publish_table', True)
        self.declare_parameter('table_size', [1.5, 0.8, 0.03])
        self.declare_parameter('table_center', [0.4, 0.0, -0.015])  # 桌顶精确在 z=0

        self._base = self.get_parameter('base_link').value
        self._default_size = [float(x) for x in
                              self.get_parameter('default_object_size').value]
        self._default_touch = [str(x) for x in
                               self.get_parameter('default_touch_links').value]

        # qos: quality of service, ROS 2 里控制消息传输可靠性的策略配置
        # /planning_scene 用 TRANSIENT_LOCAL 让晚连接的订阅者也能拿到
        # depth	10	    缓存最近 10 条消息
        # durability	TRANSIENT_LOCAL	晚订阅者也能拿到之前发的最后一条。不设的话，MoveIt 启动晚了就收不到 table
        # reliability	RELIABLE	保证送达，丢包会重传
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', qos)

        cb = ReentrantCallbackGroup()
        self.attach_srv = self.create_service(
            AttachObject, 'attach_object', self._on_attach, callback_group=cb)
        self.detach_srv = self.create_service(
            DetachObject, 'detach_object', self._on_detach, callback_group=cb)

        # 启动幂等清理 + 可选 table
        self._attached = {}    # name -> (link, size)
        self.create_timer(0.5, self._initial_publish, callback_group=cb)
        self._initialized = False

        self.get_logger().info('planning_scene_service ready')

    def _initial_publish(self):
        if self._initialized:
            return
        self._initialized = True
        # 发布 table
        if self.get_parameter('publish_table').value:
            self._publish_table()

    def _publish_table(self):
        size = [float(x) for x in self.get_parameter('table_size').value]
        center = [float(x) for x in self.get_parameter('table_center').value]
        co = CollisionObject()
        co.header.frame_id = self._base
        co.id = 'table'
        co.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = size
        co.primitives = [prim]
        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = center[2]
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self.scene_pub.publish(scene)
        self.get_logger().info(
            f'published table collision: size={size}, center={center}')

    def _on_attach(self, req, resp):
        try:
            size = [req.size.x, req.size.y, req.size.z]
            if size == [0.0, 0.0, 0.0]:
                size = self._default_size
            touch = list(req.touch_links) if req.touch_links else self._default_touch

            aco = AttachedCollisionObject()
            aco.link_name = req.link_name
            aco.touch_links = touch
            aco.object.header.frame_id = req.link_name
            aco.object.id = req.object_name
            aco.object.operation = CollisionObject.ADD

            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = size
            aco.object.primitives = [prim]
            aco.object.primitive_poses = [req.pose_in_link]

            # 同时清掉 world 里同名 object（如果之前 detach 留下了）
            world_remove = CollisionObject()
            world_remove.header.frame_id = self._base
            world_remove.id = req.object_name
            world_remove.operation = CollisionObject.REMOVE

            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [aco]
            scene.world.collision_objects = [world_remove]
            self.scene_pub.publish(scene)

            self._attached[req.object_name] = (req.link_name, size)
            resp.success = True
            resp.error_msg = ''
            self.get_logger().info(
                f'attach {req.object_name} to {req.link_name}, '
                f'size={size}, touch={touch}')
        except Exception as e:
            resp.success = False
            resp.error_msg = str(e)
            self.get_logger().error(f'attach exception: {e}')
        return resp

    def _on_detach(self, req, resp):
        try:
            info = self._attached.get(req.object_name)
            if info is None:
                resp.success = True
                resp.error_msg = 'not attached, no-op'
                return resp
            link, size = info

            aco = AttachedCollisionObject()
            aco.link_name = link
            aco.object.id = req.object_name
            aco.object.operation = CollisionObject.REMOVE

            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [aco]

            if req.put_back_in_world:
                co = CollisionObject()
                co.header.frame_id = self._base
                co.id = req.object_name
                co.operation = CollisionObject.ADD
                prim = SolidPrimitive()
                prim.type = SolidPrimitive.BOX
                prim.dimensions = size
                co.primitives = [prim]
                co.primitive_poses = [req.drop_pose]
                scene.world.collision_objects = [co]

            self.scene_pub.publish(scene)
            del self._attached[req.object_name]
            resp.success = True
            resp.error_msg = ''
            self.get_logger().info(
                f'detach {req.object_name} (put_back={req.put_back_in_world})')
        except Exception as e:
            resp.success = False
            resp.error_msg = str(e)
            self.get_logger().error(f'detach exception: {e}')
        return resp


def main():
    rclpy.init()
    node = PlanningSceneService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
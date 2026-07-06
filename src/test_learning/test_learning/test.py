import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import AddTwoInts
import time

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__("minimal_client_async")
        self.client = self.create_client(AddTwoInts, "add_two_ints")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("服务器未就绪")
            
        self.timer1 = self.create_timer(1.0, self.timer_callback_1)
        self.timer2 = self.create_timer(1.5, self.timer_callback_2)

    def send_add_request(self, a, b, request_id):
        """发送异步请求，注册回调"""
        req = AddTwoInts.Request(a=a, b=b)
        """
        当你调用 Service Client 的 call_async() 或 Action Client 的 send_goal_async() 时，ROS2 会立即返回一个 Future 对象，而不是等服务器处理完。
        future = client.call_async(request)   # 立刻返回，不会阻塞
        
        此时：
            服务器可能还没收到请求
            服务器可能正在计算
            结果还不知道
        但这个 future 对象已经存在了，它会在未来某个时刻（收到服务器的响应时）被“完成”（set done）。

        Future 的常用方法
        方法	作用
        done()	返回 True 如果结果已就绪（成功或失败），否则 False
        result()	获取结果。如果还没完成，会阻塞直到完成；如果调用失败，会抛出异常
        add_done_callback(fn)	注册一个回调函数，当 future 完成时自动调用（推荐做法，不阻塞）
        exception()	如果调用失败，返回异常对象
        """
        future = self.client.call_async(req)
        # 注意：为了在回调里能够区分是哪次请求，我们使用 lambda 或绑定参数
        # add_done_callback 是 Future 对象的一个方法，作用就是：为这个 Future 注册一个回调函数，当 Future 完成（成功或失败）时，自动调用这个回调。
        future.add_done_callback(lambda fut, rid=request_id: self.response_callback(fut, rid))
        self.get_logger().info(f'[请求 {request_id}] 已发送: {a} + {b}')
    
    def response_callback(self, future, request_id):
        """处理服务端返回的结果"""
        try:
            response = future.result()
            self.get_logger().info(f'[响应 {request_id}] 结果: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'[响应 {request_id}] 失败: {e}')

    def timer_callback_1(self):
        self.send_add_request(1, 2, request_id='T1')

    def timer_callback_2(self):
        self.send_add_request(10, 20, request_id='T2')
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MinimalClientAsync()

    # 使用多线程执行器，线程数设为 2（可以根据需要调整）
    executor = MultiThreadedExecutor(num_threads=2)
    # add_node 的作用很简单：告诉这个执行器（Executor）去管理和调度这个节点（Node）的所有回调。
    executor.add_node(node)
    '''
    执行器（Executor） 是一个“调度中心”，它负责监听各种事件（定时器到点、新消息到来、服务请求等），然后调用对应的回调函数。

    节点（Node） 拥有各种回调：定时器回调、订阅回调、服务回调、动作回调等。但这些回调不是自动运行的，需要被“某个执行器”驱动。
    
    当你创建了一个 MultiThreadedExecutor（比如线程数为 2 的线程池），它里面有一组工作线程。
    如果不调用 add_node，这个执行器不知道它应该处理哪些节点的回调，所以什么都不会做。

    调用 executor.add_node(node) 后：
        执行器将 node 加入内部管理的节点列表。
        执行器会获取该节点下所有的回调组（callback groups），并根据回调组的类型（互斥/可重入）和线程池策略来调度这些回调。
        工作线程开始从节点的事件队列中取任务并执行。
    '''

    try:
        executor.spin()  # 多线程并发处理所有回调（定时器 + 服务响应）
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# 终端1：运行服务端
# ros2 run examples_rclpy_minimal_service service
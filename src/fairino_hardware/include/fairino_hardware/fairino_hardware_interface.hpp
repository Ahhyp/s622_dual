#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"  //引入 ROS2 C++ 客户端库，提供 Node、logger、Time、Duration 等核心能力。
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp> //ros2_control 的硬件描述信息结构（从 URDF/xacro 解析出来的 joints/interfaces 参数会在这里）
#include <hardware_interface/system_interface.hpp> //ros2_control 的 SystemInterface 基类：你这个硬件插件就是继承它来实现 on_init/read/write/...
#include <hardware_interface/types/hardware_interface_return_values.hpp> //hardware_interface::return_type、CallbackReturn 等返回值类型定义
#include "hardware_interface/types/hardware_interface_type_values.hpp" //HW_IF_POSITION / VELOCITY / EFFORT 等接口名常量
#include "visibility_control.h" //通常用于导出/隐藏符号（Windows/Linux 下的 dll/so 可见性控制），给 pluginlib 用
#include <vector> //使用 std::vector
#include <array>  //[2026-08-28 分层] io_loop 与 read/write 之间的共享缓存
#include <thread> //[2026-08-28 分层] 独立 ServoJ I/O 线程
#include <mutex>  //[2026-08-28 分层] 共享缓存互斥锁
#include <atomic> //[2026-08-28 分层] 跨线程状态标志
#include <chrono> //[2026-08-28 分层] 周期调度 / stall watchdog 计时
#include "libfairino/include/robot.h" //引入厂家 SDK 的头文件（FRRobot 类就在这里）


#define CONTROLLER_IP_ADDRESS "192.168.58.2" //定义控制器默认 IP 地址字符串常量。.cpp 里会用它做 RPC 连接

#define GRIPPER_DO_SINGLE_ID  0    // [MOD] DO0 控制电磁阀线圈
#define GRIPPER_OPEN_LEVEL    0    // [MOD] DO=0 表示张开
#define GRIPPER_CLOSE_LEVEL   1    // [MOD] DO=1 表示关闭
#define GRIPPER_OPEN_THRESHOLD   0.010  // opening > 0.01 -> OPEN
#define GRIPPER_CLOSE_THRESHOLD  0.005  // opening < 0.005 -> CLOSE（滞回，防抖）

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  // [2026-08-28 v2.2] 析构兜底：任何退出路径（含 Humble shutdown 不走 on_deactivate 的情况）
  // 都保证 _io_thread 被 join，避免 std::thread 析构 terminate。只 join 回收线程，不调 SDK。
  FAIRINO_HARDWARE_PUBLIC
  ~FairinoHardwareInterface() override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  double _jnt_position_command[6]; //6 个关节的“位置指令缓冲区”。controller（如 joint_trajectory_controller）写入这里
  double _jnt_velocity_command[6]; //预留：速度指令缓冲区
  double _jnt_torque_command[6]; //预留：力矩指令缓冲区
  double _jnt_position_state[6]; //6 个关节的“反馈位置状态缓冲区”。read() 会写入这里，供 controller/MoveIt 读取
  double _jnt_velocity_state[6]; //预留：反馈速度状态缓冲区
  double _jnt_torque_state[6]; //预留：反馈力矩状态缓冲区

  double _finger_position_command[2]{0.0, 0.0}; // [MOD]
  double _finger_position_state[2]{0.0, 0.0};   // [MOD]

  // [MOD] 标记是否在URDF里存在 finger joint
  bool _has_finger1{false}; // [MOD]
  bool _has_finger2{false}; // [MOD]

  // [MOD] 夹爪状态机（用于回填state、做滞回）
  enum class GripperState { UNKNOWN, OPEN, CLOSE }; // [MOD]
  GripperState _gripper_state{GripperState::UNKNOWN}; // [MOD]

  int _control_mode; //控制模式： 0-位置控制，1-扭矩控制 2-速度控制
  std::string _controller_ip = CONTROLLER_IP_ADDRESS; //控制器 IP，默认用宏；[真机双臂] on_init() 可从 <hardware><param name="ip"> 覆盖
  std::string _prefix; // [真机双臂] joint 名前缀（如 "left_" / "right_"），on_init() 从 <hardware><param name="prefix"> 读取；单臂为空（兼容）
  std::unique_ptr<FRRobot> _ptr_robot; //厂家 SDK 对象指针：on_activate() 创建，on_deactivate() 释放，read/write 里调用 SDK 方法

  double _servoj_cmd_t{0.008}; // [2026-08-28 分层] ServoJ cmdT + io_loop 发送周期（on_init 从 <param name="servoj_cmd_t"> 读取；Phase1 默认 0.008=125Hz，Phase2 测 0.004=250Hz）

  // [2026-08-28 分层] watchdog / 速度保护阈值（on_init 参数化，见设计文档 v2.1 §8）
  double _servo_stall_warn_ms{10.0};     // ServoJ 慢调用警告阈值
  double _servo_stall_fault_ms{20.0};    // ServoJ stall → stream_broken → fault
  double _feedback_stale_fault_ms{100.0};// 反馈过期 → fault
  std::array<double, 6> _v_limit{};      // per-joint 等效速度上限（rad/s，建议 0.8×真机限速）

  // [2026-08-28 分层] I/O 线程与共享缓存
  std::thread _io_thread;                          // 独立 ServoJ I/O 线程（唯一 FRRobot 运动类调用者）
  std::mutex _io_mutex;                            // 保护 _latest_command/_latest_state/_gripper_state
  std::atomic<bool> _io_running{false};            // io_loop 运行标志
  std::atomic<bool> _faulted{false};               // latch fault（只置位，不自动恢复）
  std::atomic<bool> _shutdown_requested{false};    // on_deactivate 请求正常关闭
  std::string _fault_reason;                       // 诊断：最近一次 fault 原因（仅日志）
  std::array<double, 6> _latest_command{};         // write() 写入、io_loop 读取的最新目标（弧度）
  std::array<double, 6> _latest_state{};           // io_loop 写入、read() 读取的反馈状态（弧度）
  std::array<double, 6> _latest_velocity{};        // io_loop 写入的反馈速度（弧度/s，预留）
  std::array<double, 6> _last_sent{};              // 上次真正发送给机器人的位置（防跳变基准）
  std::atomic<int64_t> _last_send_ns{0};           // 上次 ServoJ 发送时刻（send-interval 健康检查）
  std::atomic<int64_t> _last_feedback_ns{0};       // 上次反馈成功时刻（stale 检查）
  std::atomic<uint64_t> _servo_cycles{0};          // ServoJ 成功次数（诊断）
  std::atomic<uint64_t> _servo_failures{0};        // ServoJ rc!=0 次数（诊断）
  std::atomic<uint64_t> _stall_count{0};           // stall 次数（诊断）

  // [2026-08-28 分层] 内部方法
  void io_loop();                      // I/O 线程主循环（周期=cmdT，start-to-start）
  void latch_fault(const char *reason); // 只置位 fault 原子标志 + reason；SDK 收尾由 io_loop 执行
  static int64_t steady_now_ns();      // 单调时钟（ns）

  // 给finger回填用的“名义位置”（与URDF group_state一致）
  double _f1_open{0.0305};   // [MOD]
  double _f2_open{-0.0305};   // [MOD]
  double _f1_close{0.0};      // [MOD]
  double _f2_close{0.0};      // [MOD]
};

} //end namespace


#endif
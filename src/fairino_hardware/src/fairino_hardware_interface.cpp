#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware
{
    // [2026-08-28 v2.2] 析构兜底：Humble shutdown 时 controller_manager 直接析构
    // hardware（不走 on_deactivate），若 _io_thread 仍 joinable → std::terminate（首测实证）。
    // 这里只做 join 回收线程资源，不在析构里调 FAIRINO SDK（SDK 收尾在 io_loop 内完成）。
    FairinoHardwareInterface::~FairinoHardwareInterface()
    {
        if (_io_thread.joinable())
        {
            _shutdown_requested = true; // io_loop 会在当前周期末尾 StopMotion + ServoMoveEnd 后退出
            _io_thread.join();
        }
    }

    // 上接 MoveIt/Controller 的标准指令，下接机器人 SDK 的实际通信
    hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo &sysinfo)
    {
        // ros2_control 生命周期：初始化阶段,这里主要做硬件描述解析、接口数量检查、参数读取（这份主要是检查 joint interface）
        if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) // 先调用父类 on_init，父类会把 sysinfo 存到基类成员并做基础检查
        {
            return hardware_interface::CallbackReturn::ERROR; // 父类失败则直接失败
        }
        info_ = sysinfo; // info_是父类中定义的变量,保存硬件信息（info_ 是 SystemInterface 基类里常见的成员）。后面导出接口、遍历 joints 都靠它。

        // =========================
        // [真机双臂] 从 ros2_control <hardware><param> 读取 ip / prefix
        //   <param name="ip">192.168.58.3</param>      → 覆盖默认控制器 IP
        //   <param name="prefix">left_</param>         → joint 名前缀（双臂），单臂不写则保持空
        // 缺省回退：ip → CONTROLLER_IP_ADDRESS 宏（58.2）；prefix → 空（单臂兼容，行为与改动前一致）
        // =========================
        {
            auto hw_param = [this](const std::string &key, const std::string &def)
            {
                auto it = info_.hardware_parameters.find(key);
                return (it != info_.hardware_parameters.end()) ? it->second : def;
            };
            _controller_ip = hw_param("ip", CONTROLLER_IP_ADDRESS);
            _prefix = hw_param("prefix", "");
            // [2026-08-28 分层] 参数化（设计文档 v2.1 §8）：
            //   servoj_cmd_t           默认 0.008（125Hz，Phase1）；Phase2 测 0.004（250Hz）
            //   servo_v_limit_0..5     per-joint 等效速度上限（rad/s），建议 0.8×真机限速（J3 重点保护）
            //   servo_stall_warn_ms    10ms 慢调用警告
            //   servo_stall_fault_ms   20ms stall → fault
            //   feedback_stale_fault_ms 100ms 反馈过期 → fault
            _servoj_cmd_t = std::stod(hw_param("servoj_cmd_t", "0.008"));
            for (int i = 0; i < 6; ++i)
            {
                _v_limit[i] = std::stod(hw_param("servo_v_limit_" + std::to_string(i), "2.5"));
            }
            _servo_stall_warn_ms = std::stod(hw_param("servo_stall_warn_ms", "10.0"));
            _servo_stall_fault_ms = std::stod(hw_param("servo_stall_fault_ms", "20.0"));
            // [2026-08-28 v2.3] 默认 2000ms：覆盖 1s stall 期间的 feedback 冻结（见 hpp 注释）
            _feedback_stale_fault_ms = std::stod(hw_param("feedback_stale_fault_ms", "2000.0"));
        }

        // =========================
        // [MOD] 先扫描 joints，识别 finger1/finger2 是否存在（带前缀，双臂 left_finger1_joint 等）
        // =========================
        _has_finger1 = false;
        _has_finger2 = false;
        for (const auto &joint : info_.joints)
        {
            if (joint.name == _prefix + "finger1_joint")
                _has_finger1 = true;
            if (joint.name == _prefix + "finger2_joint")
                _has_finger2 = true;
        }

        for (const hardware_interface::ComponentInfo &joint : info_.joints) // 遍历 URDF/ros2_control 中声明的每个 joint
        {

            // 指令部分,命令接口检查
            if (joint.command_interfaces.size() != 1)
            { // 开放servoJ,要求每个关节只有 1 个 command interface
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                             joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR; // 如果不是 1 个，直接报错退出
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) // 强制要求 command interface 名称必须是 "position"
            {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                             joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR; // 不符合就失败
            }
            // 预留未来做“扭矩直接控制”
            //  if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
            //      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
            //             "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
            //             joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
            //      return hardware_interface::CallbackReturn::ERROR;
            //  }

            // 关节状态部分,状态接口检查
            if (joint.state_interfaces.size() != 1) // 要求每关节只有 1 个 state interface
            {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                             joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) // 强制要求 state interface 名称为 "position"
            {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                             joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR; // 不匹配就失败
            }
            // 未来可能扩展更多状态接口
            //  if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            //      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
            //                  "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
            //                  joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            //      return hardware_interface::CallbackReturn::ERROR;
            //  }

            // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
            //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
            //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
            //     return hardware_interface::CallbackReturn::ERROR;
            // }
        }
        // =========================
        // [MOD]强制要求 finger1/finger2 必须成对出现，可以加检查
        // =========================
        if (_has_finger1 != _has_finger2)
        {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                         "URDF has only one finger joint. finger1=%d finger2=%d. They should be both present for scheme A.",
                         (int)_has_finger1, (int)_has_finger2);
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS; // 所有 joint 检查通过，初始化成功
    } // end on_init

    /*
    export_state_interfaces() 和 export_command_interfaces()
    这两个函数将内存地址暴露给 ros2_control 框架：

    对于关节 j1~j6，分别绑定 _jnt_position_state[i] 和 _jnt_position_command[i] 数组。

    对于 finger1_joint、finger2_joint，绑定 _finger_position_state[0/1] 和 _finger_position_command[0/1]。

    上层控制器读写这些数组，就像在读写硬件寄存器一样。
    */

    // 把“状态缓冲区地址”暴露给 ros2_control
    std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces() // ros2_control 会调用它拿到“状态接口列表”
    {
        std::vector<hardware_interface::StateInterface> state_interfaces; // 创建要返回的容器

        // 导出关节相关的状态接口(位置，速度，扭矩)
        //   for (size_t i = 0; i < info_.joints.size(); ++i)//对每个 joint 导出一个 state interface
        //   {
        //     //joint 名称 = info_.joints[i].name,接口名 = "position",数据地址 = _jnt_position_state[i],controller/MoveIt 读取状态时，本质就是读这块内存
        //     state_interfaces.emplace_back(hardware_interface::StateInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

        //     // state_interfaces.emplace_back(hardware_interface::StateInterface(
        //     //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state.at(i)));

        //     // state_interfaces.emplace_back(hardware_interface::StateInterface(
        //     //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
        //   }
        for (const auto &joint : info_.joints)
        {
            if (joint.name == _prefix + "j1")
            {
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[0]);
            }
            else if (joint.name == _prefix + "j2")
            {
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[1]);
            }
            else if (joint.name == _prefix + "j3")
            {
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[2]);
            }
            else if (joint.name == _prefix + "j4")
            {
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[3]);
            }
            else if (joint.name == _prefix + "j5")
            {
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[4]);
            }
            else if (joint.name == _prefix + "j6")
            {
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[5]);
            }
            else if (joint.name == _prefix + "finger1_joint")
            { // [MOD]
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_finger_position_state[0]);
            }
            else if (joint.name == _prefix + "finger2_joint")
            { // [MOD]
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_finger_position_state[1]);
            }
            else
            {
                // [MOD] 未识别的joint直接报错
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Unknown joint name '%s' in ros2_control. Please check URDF joints list. (prefix='%s')",
                             joint.name.c_str(), _prefix.c_str());
            }
        }

        // 导出
        return state_interfaces; // 返回接口列表
    }

    // 把“指令缓冲区地址”暴露给 ros2_control
    std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces() // ros2_control 会拿到“命令接口列表”
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces; // 创建容器
        //   for (size_t i = 0; i < info_.joints.size(); ++i) //遍历 joints
        //   {
        //     //controller 写指令时写的就是 _jnt_position_command[i]
        //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));
        //     //预留扭矩控制接口
        // //     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
        // //         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
        //   }
        for (const auto &joint : info_.joints)
        {
            if (joint.name == _prefix + "j1")
            {
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[0]);
            }
            else if (joint.name == _prefix + "j2")
            {
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[1]);
            }
            else if (joint.name == _prefix + "j3")
            {
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[2]);
            }
            else if (joint.name == _prefix + "j4")
            {
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[3]);
            }
            else if (joint.name == _prefix + "j5")
            {
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[4]);
            }
            else if (joint.name == _prefix + "j6")
            {
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[5]);
            }
            else if (joint.name == _prefix + "finger1_joint")
            { // [MOD]
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_finger_position_command[0]);
            }
            else if (joint.name == _prefix + "finger2_joint")
            { // [MOD]
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &_finger_position_command[1]);
            }
            else
            {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Unknown joint name '%s' in ros2_control. Please check URDF joints list. (prefix='%s')",
                             joint.name.c_str(), _prefix.c_str());
            }
        }

        return command_interfaces; // 返回接口列表
    }

    // 启动硬件（连接 SDK，读取初始状态，避免“上电跳变”）
    hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state) // 生命周期：激活硬件,一般在 controller 启动前调用
    {
        using namespace std::chrono_literals;                                                      // 允许写 200ms 这种字面量
        const std::string tag = _prefix.empty() ? "" : "[" + _prefix + "]";                       // [真机双臂] 日志前缀区分左右
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting %s...please wait...", tag.c_str()); // 提示启动中
        // 做变量的初始化工作
        _ptr_robot = std::make_unique<FRRobot>(); // 创建机器人实例,创建厂家 SDK 的机器人对象
        for (int i = 0; i < 6; i++)
        {                                 // 初始化变量
            _jnt_position_command[i] = 0; // 初始化 command/state 缓冲区为 0,这只是初始化值，真正安全关键在后面“读取反馈并同步到 command”
            _jnt_velocity_command[i] = 0;
            _jnt_torque_command[i] = 0;
            _jnt_position_state[i] = 0;
            _jnt_velocity_state[i] = 0;
            _jnt_torque_state[i] = 0;
        }
        // =========================
        // [MOD] 初始化 finger 缓冲区
        // =========================
        _finger_position_command[0] = _f1_close;
        _finger_position_command[1] = _f2_close;
        _finger_position_state[0] = _f1_close;
        _finger_position_state[1] = _f2_close;
        _gripper_state = GripperState::CLOSE;

        _control_mode = 0;                                            // 默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
        errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str()); // 建立xmlrpc连接,用 SDK 的 RPC 接口连接控制器（XML-RPC）
        rclcpp::sleep_for(200ms);                                     // 等待一段时间让控制器的rpc连接建立完毕,等待连接建立
        if (returncode != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂%sSDK连接失败！ip=%s 请检查端口时候被占用",
                        tag.c_str(), _controller_ip.c_str());
            return hardware_interface::CallbackReturn::ERROR; // 连接失败则报错并返回 ERROR
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂%sSDK连接成功！ip=%s", tag.c_str(), _controller_ip.c_str()); // 提示 SDK 连接成功
        }
        // 做第一步的工作，读取当前状态数据
        JointPos jntpos; // 厂家 SDK 的关节位置结构体
        returncode = _ptr_robot->GetActualJointPosDegree(0, &jntpos);
        /*
        获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
        因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
        */
        if (returncode == 0) // 成功读取反馈
        {
            for (int j = 0; j < 6; j++)
            {
                _jnt_position_command[j] = jntpos.jPos[j] / 180.0 * M_PI; // 把“度”转“弧度”，并把command 初始化为当前实际角度
                // [2026-08-28 分层] 同步共享缓存：最新目标 = 上次发送 = 反馈状态 = 当前实际
                _latest_command[j] = _jnt_position_command[j];
                _last_sent[j] = _jnt_position_command[j];
                _latest_state[j] = _jnt_position_command[j];
            }

            // =========================
            // [MOD] 上电时把夹爪DO也置到一个“已知状态”
            // 避免你上电后 valve 状态不确定
            // =========================
            if (_has_finger1 && _has_finger2)
            {
                _ptr_robot->SetDO(GRIPPER_DO_SINGLE_ID, GRIPPER_CLOSE_LEVEL, 0, 1);
                _gripper_state = GripperState::CLOSE;
            }

            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "初始指令位置: %f,%f,%f,%f,%f,%f", _jnt_position_command[0],
                        _jnt_position_command[1], _jnt_position_command[2], _jnt_position_command[3], _jnt_position_command[4], _jnt_position_command[5]);

            // =========================
            // [2026-08-28 分层] 会话级 ServoMoveStart（设计文档 v2.1 §5）
            // 依据：FAIRINO frcobot_ros2 Issue #32（open，用户实测）——on_activate 不先
            // ServoMoveStart 就 write() 调 ServoJ 会严重阻塞；补上后恢复。
            // 注意：裸测 mode 3（会话级 Start→ServoJ×3000→End）证明它**只降低 stall 发生率**、
            // 不消灭 1s stall（8/3000 次 >500ms，max 1034ms）——所以本插件另有 stall 检测/
            // 速度保护（io_loop），不依赖 ServoMoveStart 根治阻塞。
            // 失败 → 不允许 active（保持旧行为一致性：激活失败即 ERROR）。
            // =========================
            if (_ptr_robot->ServoMoveStart() != 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                             "ServoMoveStart 失败，不允许激活（会话级 ServoJ 前置条件未满足）");
                _ptr_robot->CloseRPC();
                _ptr_robot.release();
                return hardware_interface::CallbackReturn::ERROR;
            }

            // [2026-08-28 分层] 初始化 fault/运行标志，启动独立 I/O 线程
            _faulted = false;
            _shutdown_requested = false;
            _servo_cycles = 0;
            _servo_failures = 0;
            _stall_count = 0;
            _last_feedback_ns = steady_now_ns();
            _last_send_ns = steady_now_ns();
            _io_running = true;
            _io_thread = std::thread(&FairinoHardwareInterface::io_loop, this);
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "ServoJ I/O started: cmdT=%.3f ms (%d Hz), stall_fault=%.1f ms, v_limit=%f/%f/%f/%f/%f/%f",
                        _servoj_cmd_t * 1000.0, (int)(1.0 / _servoj_cmd_t),
                        _servo_stall_fault_ms,
                        _v_limit[0], _v_limit[1], _v_limit[2], _v_limit[3], _v_limit[4], _v_limit[5]);

            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂%s硬件启动成功!", tag.c_str()); // 激活成功
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容"); // 读取初始角度失败就不允许激活（安全设计正确）
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // 停止运动并断开 SDK
    hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait..."); // 提示停止
        // [2026-08-28 分层] 清理顺序（设计文档 v2.1 §5）：
        //   1) 置 shutdown_requested（不动 _io_running）
        //   2) io_loop 在下个周期看到请求 → 自己 StopMotion + ServoMoveEnd + 退出
        //   3) join 等 io_loop 彻底退出（无论正常关闭还是 fault 退出，joinable 就必须 join，
        //      否则 std::thread 析构 terminate；fault 已退出时 shutdown_requested 无害）
        //   4) CloseRPC（保证在 I/O 线程退出后；CloseRPC 自身 stack-smashing 为 KNOWN SDK ISSUE，见设计文档 §11）
        // 运行期所有 FRRobot 运动类调用集中在 io_loop 单线程，无并发。
        _shutdown_requested = true;
        if (_io_thread.joinable())
        {
            _io_thread.join();
        }
        _ptr_robot->CloseRPC(); // 销毁实例，连接断开（KNOWN SDK ISSUE：stack corruption 风险，单独跟踪）
        _ptr_robot.release();
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
        return hardware_interface::CallbackReturn::SUCCESS; // 停止完成
    }

    // 从硬件读取状态，写入 _jnt_position_state[]
    // [2026-08-28 分层] 不再直接调 SDK——反馈由 io_loop 线程读取并写入 _latest_state，这里只锁内拷贝缓存
    hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) // 控制循环中被 ros2_control 周期调用（与 controller 更新频率一致）
    {
        if (_faulted || !_io_running)
        {
            return hardware_interface::return_type::ERROR; // fault/未运行：告诉 ros2_control 硬件已失败
        }
        if (steady_now_ns() - _last_feedback_ns.load() >
            static_cast<int64_t>(_feedback_stale_fault_ms * 1e6))
        {
            return hardware_interface::return_type::ERROR; // 反馈过期（>100ms）→ 视为硬件失败
        }
        {
            std::lock_guard<std::mutex> lock(_io_mutex);
            std::copy(_latest_state.begin(), _latest_state.end(), _jnt_position_state);
            std::copy(_latest_velocity.begin(), _latest_velocity.end(), _jnt_velocity_state);

            // =========================
            // [MOD] finger 状态回填（用“最近一次状态”虚拟反馈）
            // _gripper_state 由 io_loop 线程在锁内更新（SetDO 切换）
            // MoveIt/控制器会看这个 state 判断是否到位
            // =========================
            if (_has_finger1 && _has_finger2)
            {
                if (_gripper_state == GripperState::OPEN)
                {
                    _finger_position_state[0] = _f1_open;
                    _finger_position_state[1] = _f2_open;
                }
                else if (_gripper_state == GripperState::CLOSE)
                {
                    _finger_position_state[0] = _f1_close;
                    _finger_position_state[1] = _f2_close;
                }
                else
                {
                    // UNKNOWN：先回填命令值，避免突变
                    _finger_position_state[0] = _finger_position_command[0];
                    _finger_position_state[1] = _finger_position_command[1];
                }
            }
        }
        return hardware_interface::return_type::OK; // 向 ros2_control 表示读取成功
    }

    // 把 _jnt_position_command[] 下发给硬件（ServoJ）
    // [2026-08-28 分层] 不再直接调 SDK——只把最新目标写入 _latest_command，由 io_loop 线程按 cmdT 周期发送
    hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) // 控制循环中被周期调用,controller 每周期更新 command 缓冲区，这里把它发送给机械臂
    {
        if (_faulted || !_io_running)
        {
            return hardware_interface::return_type::ERROR; // fault/未运行：告诉 ros2_control 硬件已失败，不继续接受指令
        }
        if (_control_mode == 0)
        { // 位置控制模式
            if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],
                            [](double c)
                            { return not std::isfinite(c); }))
            {
                return hardware_interface::return_type::ERROR; // 如果发现 NaN/inf，返回错误，不下发指令
            }
            {
                std::lock_guard<std::mutex> lock(_io_mutex);
                std::copy(std::begin(_jnt_position_command), std::end(_jnt_position_command), _latest_command.begin());
                ++_write_calls; // [2026-08-28 诊断] 统计 write 拷贝次数
                // finger 命令由 io_loop 在锁内读取（SetDO 切换判定），这里不需要拷贝——框架直接写 _finger_position_command
            }
            // 注：SetDO（夹爪）切换已移到 io_loop（仅状态变化时调用一次，单独计时），
            //     避免低频 IO 调用打乱 ServoJ 关键周期（设计文档 v2.1 §7/§8）。
        }
        else if (_control_mode == 1)
        { // 扭矩控制模式,预留扭矩模式
            if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],
                            [](double c)
                            { return not std::isfinite(c); }))
            {
                return hardware_interface::return_type::ERROR;
            }
            //_ptr_robot->write(_jnt_torque_command);//注意单位转换
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    // =========================
    // [2026-08-28 分层] 独立 ServoJ I/O 线程主循环（设计文档 v2.1 §3/§6/§7）
    // 唯一 FRRobot 运动类调用者：ServoJ / GetActualJointPosDegree / SetDO / StopMotion / ServoMoveEnd
    // 周期 = cmdT（start-to-start，sleep_until 而非 "调用耗时 + sleep"）
    // =========================
    void FairinoHardwareInterface::io_loop()
    {
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(_servoj_cmd_t));
        auto next_tick = std::chrono::steady_clock::now();
        bool was_clamped = false; // clamp 状态变化时才打日志（避免刷屏）

        while (_io_running)
        {
            next_tick += period;

            // ── 1. 取最新目标（锁内）+ finger 滞回判定 ──
            std::array<double, 6> command;
            GripperState desired_gripper = _gripper_state;
            {
                std::lock_guard<std::mutex> lock(_io_mutex);
                command = _latest_command;
                if (_has_finger1 && _has_finger2)
                {
                    const double opening = 0.5 * (std::fabs(_finger_position_command[0]) +
                                                  std::fabs(_finger_position_command[1]));
                    if ((_gripper_state == GripperState::CLOSE || _gripper_state == GripperState::UNKNOWN) &&
                        opening > GRIPPER_OPEN_THRESHOLD)
                    {
                        desired_gripper = GripperState::OPEN;
                    }
                    else if (_gripper_state == GripperState::OPEN && opening < GRIPPER_CLOSE_THRESHOLD)
                    {
                        desired_gripper = GripperState::CLOSE;
                    }
                }
            }

            // ── 2. per-joint command rate limiter（最后一道防线，设计文档 v2.2）──
            //    candidate = last_sent + clamp(latest - last_sent, ±v_safe·cmdT)
            //    → 单步位移永远 ≤ v_safe·cmdT，任何跳变（含 stall 恢复、外部大指令）
            //      都不会以超过安全速度的步长交给 ServoJ。
            //    注意：这只是"位置步长限速"，不承诺机器人加速度/伺服滤波安全，
            //    v_safe = 0.8×v_limit 起步，按实机跟踪误差调。
            std::array<double, 6> candidate;
            bool clamped = false;
            for (int i = 0; i < 6; ++i)
            {
                const double dq = command[i] - _last_sent[i];
                const double dq_max = _v_limit[i] * _servoj_cmd_t;
                if (dq > dq_max)
                {
                    candidate[i] = _last_sent[i] + dq_max;
                    clamped = true;
                }
                else if (dq < -dq_max)
                {
                    candidate[i] = _last_sent[i] - dq_max;
                    clamped = true;
                }
                else
                {
                    candidate[i] = command[i];
                }
            }
            if (clamped && !was_clamped)
            {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                            "[rate limiter] command step clamped to v_safe×cmdT "
                            "(dq1=%.4f rad, limit=%.4f rad) — 跟踪将滞后，等待恢复",
                            command[0] - _last_sent[0], _v_limit[0] * _servoj_cmd_t);
            }
            was_clamped = clamped;

            // ── 3. send-interval health check（stream-health 指标，非超速判据）──
            //    异常间隔是 stall 的另一种表现：只计数/警告，不 fault（设计文档 v2.2：
            //    communication timing anomaly ≠ robot command rejection）
            const int64_t now_ns = steady_now_ns();
            const double dt_send_ms = (now_ns - _last_send_ns.load()) / 1e6;
            if (dt_send_ms > _servo_stall_fault_ms && _last_send_ns.load() != 0)
            {
                ++_stall_count;
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                             "send interval discontinuity: %.1f ms (stall_count=%lu)",
                             dt_send_ms, (unsigned long)_stall_count.load());
            }
            else if (dt_send_ms > _servo_stall_warn_ms && _last_send_ns.load() != 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                            "send interval drift: %.1f ms (cmdT=%.3f ms)", dt_send_ms, _servoj_cmd_t * 1000.0);
            }

            // ── 4. ServoJ 下发（计时，watchdog）──
            // [2026-08-28 v2.3 诊断] 每 100 周期打印目标链路，定位"机械臂不动"：
            //   latest = JTC 写入的最新目标（j1 在垂直下降中不动，重点看 j2/j3）
            //   candidate = clamp 后实际发送；last_sent = 上次成功发送
            //   write_calls = write() 拷贝次数（区分 write 没被调 / JTC 没写 / SDK 不动）
            if ((_servo_cycles.load() % 100) == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                            "[diag] cycle=%lu write=%lu | latest j1/j2/j3=%.4f/%.4f/%.4f | cand=%.4f/%.4f/%.4f | sent=%.4f/%.4f/%.4f",
                            (unsigned long)_servo_cycles.load(), (unsigned long)_write_calls.load(),
                            command[0], command[1], command[2],
                            candidate[0], candidate[1], candidate[2],
                            _last_sent[0], _last_sent[1], _last_sent[2]);
            }
            JointPos sdk_cmd{};
            for (int i = 0; i < 6; ++i)
            {
                sdk_cmd.jPos[i] = candidate[i] * 180.0 / M_PI; // 弧度 → 度
            }
            ExaxisPos extcmd{0, 0, 0, 0};
            const int64_t t0 = steady_now_ns();
            const int rc = _ptr_robot->ServoJ(&sdk_cmd, &extcmd, 0, 0, _servoj_cmd_t, 0, 0);
            const double call_ms = (steady_now_ns() - t0) / 1e6;

            if (rc != 0)
            { // rc!=0 → HARD FAULT（命令被机器人拒绝，如 14），不再持续下发
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                             "ServoJ failed: rc=%d, cmdT=%.3f ms, call=%.1f ms", rc, _servoj_cmd_t * 1000.0, call_ms);
                ++_servo_failures;
                latch_fault("ServoJ failed");
                break;
            }
            ++_servo_cycles;
            _last_send_ns = steady_now_ns();
            _last_sent = candidate; // 已发送的就是新基准（clamp 保证单步位移 ≤ v_safe·cmdT）

            // ── 5. stall watchdog（设计文档 v2.2）──
            //    同步阻塞无法打断，只能检测已发生的 stall；>20ms 视为 stream discontinuity，
            //    不直接 fault（保持位 stall 无害，实测 Δq=0 也每 ~100 次卡 1s），
            //    下一条指令已由 clamp 保证不跳跃。rc!=0 才是 hard fault。
            if (call_ms > _servo_stall_fault_ms)
            {
                ++_stall_count;
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                             "ServoJ stream discontinuity: call=%.1f ms (stall_count=%lu)",
                             call_ms, (unsigned long)_stall_count.load());
            }
            else if (call_ms > _servo_stall_warn_ms)
            {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ slow call: %.1f ms", call_ms);
            }

            // ── 6. 反馈读取（io_loop 内，写 _latest_state）──
            JointPos feedback{};
            if (_ptr_robot->GetActualJointPosDegree(1, &feedback) == 0)
            {
                bool finite = true;
                std::array<double, 6> state;
                for (int i = 0; i < 6; ++i)
                {
                    state[i] = feedback.jPos[i] * M_PI / 180.0;
                    finite = finite && std::isfinite(state[i]);
                }
                if (!finite)
                {
                    latch_fault("non-finite feedback");
                    break;
                }
                {
                    std::lock_guard<std::mutex> lock(_io_mutex);
                    _latest_state = state;
                    // finger SetDO：仅状态变化时调用一次（不进 ServoJ 关键路径），单独计时
                    if (_has_finger1 && desired_gripper != _gripper_state)
                    {
                        const uint8_t level = (desired_gripper == GripperState::OPEN) ? GRIPPER_OPEN_LEVEL : GRIPPER_CLOSE_LEVEL;
                        const int64_t tg0 = steady_now_ns();
                        const int io_rc = _ptr_robot->SetDO(GRIPPER_DO_SINGLE_ID, level, 0, 1);
                        const double io_ms = (steady_now_ns() - tg0) / 1e6;
                        if (io_rc == 0)
                        {
                            _gripper_state = desired_gripper;
                        }
                        else
                        {
                            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                                        "SetDO failed: rc=%d (%.1f ms)", io_rc, io_ms);
                        }
                        if (io_ms > 10.0)
                        {
                            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "SetDO slow: %.1f ms", io_ms);
                        }
                    }
                }
                _last_feedback_ns = steady_now_ns();
            }
            else if (steady_now_ns() - _last_feedback_ns.load() >
                     static_cast<int64_t>(_feedback_stale_fault_ms * 1e6))
            {
                latch_fault("joint feedback stale");
                break;
            }

            // ── 7. 周期调度 / 关闭检查 ──
            const auto now2 = std::chrono::steady_clock::now();
            if (now2 >= next_tick)
            {
                next_tick = now2; // 周期超时（overrun），重新锚定，避免连锁漂移
            }
            if (_shutdown_requested)
            {
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                            "io_loop: shutdown requested, StopMotion + ServoMoveEnd");
                _ptr_robot->StopMotion();
                _ptr_robot->ServoMoveEnd();
                break;
            }
            std::this_thread::sleep_until(next_tick);
        }

        // ── 线程退出收尾：fault 时安全停止（SDK 调用仍在 io_loop 线程内）──
        if (_faulted)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "io_loop exited with fault: %s — StopMotion", _fault_reason.c_str());
            _ptr_robot->StopMotion();
        }
    }

    // [2026-08-28 分层] fault latch：只置位原子标志 + reason，不直接调 SDK
    // （SDK 收尾由 io_loop 线程自己执行，保证 FRRobot 单线程访问）
    void FairinoHardwareInterface::latch_fault(const char *reason)
    {
        bool expected = false;
        if (!_faulted.compare_exchange_strong(expected, true))
        {
            return; // 已 fault，忽略后续
        }
        _fault_reason = reason;
        _io_running = false;
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Hardware fault latched: %s", reason);
    }

    int64_t FairinoHardwareInterface::steady_now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

} // end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
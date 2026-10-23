// include/dual_arm_qp_controller/dual_arm_qp_controller.hpp
// C2.2 layer 3: ros2_control ControllerInterface wrapping dual_arm_qp_core.
//
// Interface contract (decision D2):
//   - math: velocity-level QP (dq), execution: position command
//   - claims 12 position command interfaces (arms only) and 12 position
//     state interfaces, in the joint order given by the `joints` parameter
//   - does NOT claim the finger joints (gripper handled elsewhere)
//
// Two operating modes (C4.3 S4 W1):
//   IDLE  -- relative-pose hold: the QP target is ^L T_R captured at activation
//            (freezes to the activation-time chain, C2.3).  A test-only
//            sinusoidal sweep can be enabled to make the D7 interlock
//            observable in simulation.  Bit-identical to the pre-W1 behaviour.
//   RUNNING (a transport trajectory is latched) -- the object task is supplied
//            as an external dual_arm_qp::ObjectTaskRow assembled by
//            TransportTask, and the relative target is the *frozen grasp chain*
//            G_L^-1 G_R.  The legacy symmetric-midpoint object frame is never
//            constructed in this mode.
//
// Control-path discipline: update() performs no heap allocation and takes no
// lock.  A ~/start_transport request is parsed and validated on the executor
// thread; update() only swaps a shared_ptr and builds one trajectory once, at
// that state transition.
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "dual_arm_collision/dual_arm_collision.hpp"
#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_qp_controller/msg/transport_status.hpp"
#include "dual_arm_qp_controller/period_monitor.hpp"
#include "dual_arm_qp_controller/srv/abort_transport.hpp"
#include "dual_arm_qp_controller/srv/start_transport.hpp"
#include "dual_arm_qp_controller/stream_health_monitor.hpp"
#include "dual_arm_qp_controller/transport_task.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace dual_arm_qp_controller {

class DualArmQpController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time,
                                             const rclcpp::Duration& period) override;

private:
    /// Parsed, validated start request handed from the executor thread to the
    /// control loop.  Held behind a shared_ptr so the swap is allocation-free.
    struct StartRequest {
        dual_arm_object_task::TrajectorySpec spec;
    };

    bool readState();
    bool writeCommand(const Vec12& q_cmd);
    void clampToLimits(Vec12* q) const;

    // ---- W1 transport plumbing ----
    void createInterfaces();
    void onStartTransport(const std::shared_ptr<dual_arm_qp_controller::srv::StartTransport::Request> req,
                          std::shared_ptr<dual_arm_qp_controller::srv::StartTransport::Response> res);
    void onAbortTransport(const std::shared_ptr<dual_arm_qp_controller::srv::AbortTransport::Request> req,
                          std::shared_ptr<dual_arm_qp_controller::srv::AbortTransport::Response> res);
    void onResume(std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void onHeartbeat(std_msgs::msg::Empty::ConstSharedPtr msg);
    void onReset(std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    /// Apply a queued start request.  One-off allocation at a state transition.
    void applyPendingStart();
    /// §4.7.1 E1: cheapest possible independent check, before any math.
    bool jointStateInBounds(double* min_margin) const;
    void publishStatus();

    std::vector<std::string> joint_names_;
    std::vector<std::string> command_iface_names_;
    std::vector<std::string> state_iface_names_;

    std::unique_ptr<dual_arm::DualArmKinematics> kinematics_;
    std::unique_ptr<dual_arm_collision::DualArmCollisionModel> collision_;
    std::unique_ptr<dual_arm_qp::DualArmQp> qp_;
    std::unique_ptr<TransportTask> transport_;

    Vec12 q_state_ = Vec12::Zero();
    Vec12 q_cmd_ = Vec12::Zero();
    Vec12 q_min_ = Vec12::Zero();
    Vec12 q_max_ = Vec12::Zero();
    Vec12 v_max_ = Vec12::Zero();
    dual_arm::Iso3 target_ = dual_arm::Iso3::Identity();

    bool activated_ = false;
    rclcpp::Time activate_time_;
    int solve_fail_streak_ = 0;
    bool hold_active_ = false;
    StreamHealthMonitor health_;
    PeriodMonitor period_monitor_;
    long long stats_counter_ = 0;

    // ---- W1 transport ----
    using StatusPublisher =
        realtime_tools::RealtimePublisher<dual_arm_qp_controller::msg::TransportStatus>;
    using StartBuffer = realtime_tools::RealtimeBuffer<std::shared_ptr<StartRequest>>;
    std::unique_ptr<StatusPublisher> status_pub_;
    std::unique_ptr<StartBuffer> start_req_;
    rclcpp::Service<dual_arm_qp_controller::srv::StartTransport>::SharedPtr start_srv_;
    rclcpp::Service<dual_arm_qp_controller::srv::AbortTransport>::SharedPtr abort_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    // ---- S4 W1 review (A): executor heartbeat + honest protection reporting ----
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
    std::atomic<double> last_heartbeat_s_{-1.0};
    std::atomic<bool> heartbeat_seen_{false};
    ProtectionReport protection_;
    Vec12 home_q_ = defaultHomeQ();

    Vec12 last_dq_ = Vec12::Zero();
    bool last_committed_ = false;
    const char* last_failure_code_ = "";
    /// 0 = none, 1 = resume, 2 = reset, 3 = abort (applied by update()).
    std::atomic<int> pending_cmd_{0};
    int status_tick_ = 0;
    int status_period_ticks_ = 13;  // 125 Hz / 10 Hz (overwritten by the parameter)

    // parameters
    double dt_ = 0.008;
    double kp_rel_ = 10.0;
    double w_rel_ = 1.0;
    double w_slack_ = 1e3;
    double w_center_ = 1e-4;  // W1: was 1e-2, see C4.3_S4_执行规划.md §5.1(1)
    double w_reg_ = 1e-6;
    double v_max_default_ = 2.5;
    double test_sweep_amp_m_ = 0.0;
    double test_sweep_period_s_ = 4.0;
    double period_fault_factor_ = 2.0;
    // C2.7c: independent safety layer (NOT the QP/CBF path)
    double collision_d_stop_ = 0.0;      // 0 = disabled
    double collision_link_radius_ = 0.06;

    // object task (C4.3 S4 W1)
    double kp_obj_ = 20.0;
    double w_obj_ = 1.0;
    double w_slack_obj_ = 1e3;
    double obj_rot_error_refuse_rad_ = 3.0915926535897933;  // pi - 0.05, frozen
    int solve_fail_streak_max_ = 10;
    double status_rate_hz_ = 10.0;
    bool require_executor_heartbeat_ = false;
    double executor_heartbeat_timeout_s_ = 0.1;
    double start_pose_tolerance_rad_ = 0.05;
    Vec12 dq_center_ = Vec12::Zero();
    dual_arm::Iso3 object_T_left_tcp_ = dual_arm::Iso3::Identity();
    dual_arm::Iso3 object_T_right_tcp_ = dual_arm::Iso3::Identity();

    template <typename T>
    void declareIfNeeded(const std::string& name, const T& default_value) {
        if (!get_node()->has_parameter(name)) {
            get_node()->declare_parameter(name, default_value);
        }
    }
};

}  // namespace dual_arm_qp_controller

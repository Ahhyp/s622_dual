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
#include <cstdint>
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
#include "dual_arm_qp_controller/srv/hold_transport.hpp"
#include "dual_arm_qp_controller/srv/start_transport.hpp"
#include "dual_arm_qp_controller/stream_health_monitor.hpp"
#include "dual_arm_qp_controller/timing_recorder.hpp"
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
    void onHoldTransport(const std::shared_ptr<dual_arm_qp_controller::srv::HoldTransport::Request> req,
                         std::shared_ptr<dual_arm_qp_controller::srv::HoldTransport::Response> res);
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
    /// §7 S4-T2 / S4-T17: dump the timing ring buffer (non-RT; service thread).
    void onTimingDump(std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    /// C4.3 S4 W3-R1a / T-H13: close phase `phase` (0-based P1..P6) at the
    /// current steady-clock instant.  Reads no clock unless the phase is enabled
    /// by `timing_phases`, so `timing_phases=0` costs one predictable branch.
    void markTimingPhase(int phase);

    /// RAII: records exactly one sample per update(), on every exit path.
    struct TimingScope {
        explicit TimingScope(TimingRecorder& r) : rec(r) { rec.begin(nowNs()); }
        ~TimingScope() { rec.end(solve_us); }
        void setSolveUs(double us) { solve_us = us; }
        static uint64_t nowNs() {
            return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                             std::chrono::steady_clock::now().time_since_epoch())
                                             .count());
        }
        TimingRecorder& rec;
        double solve_us = 0.0;
    };

    std::vector<std::string> joint_names_;
    std::vector<std::string> command_iface_names_;
    std::vector<std::string> state_iface_names_;

    // ---- §7 S4-T2 / S4-T17 diagnostics (off by default) ----
    TimingRecorder timing_;
    bool timing_enable_ = false;
    int timing_capacity_ = 0;
    std::string timing_csv_path_;
    /// C4.3 S4 W3-R1a / T-H13: bit i (0-based) enables phase P(i+1); 0 keeps the
    /// W1 behaviour (P0/P7 only), 0x3F records all six phases.
    int timing_phases_ = 0;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr timing_srv_;

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
  /// Plan §4.7.1 E4: consecutive period-overrun count; see update().
  int deadline_fault_streak_ = 0;
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
    rclcpp::Service<dual_arm_qp_controller::srv::HoldTransport>::SharedPtr hold_srv_;
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
    // ---- C7-A Rev.2: executor HOLD request -----------------------------------
    // Single slot: the service handler CAS-es 0 -> seq and refuses with "busy"
    // while it is occupied, so a request can never be silently overwritten.
    std::atomic<std::uint32_t> hold_req_seq_{0};
    // Packed ack: (seq << 32) | result.  ONE atomic so a reader can never pair a
    // seq with the result of a different request.
    std::atomic<std::uint64_t> hold_ack_{0};
    // C4.3 S4 D3-a patch (C7): replay memory.  The last CONSUMED request is
    // remembered so a retransmission of the same seq (the idempotency key) is
    // acked again WITHOUT freezing a second time.  Deliberately NOT cleared by
    // start / resume / reset: clearing it would let an old seq freeze a fresh
    // run, which is exactly the replay hole this member closes.
    std::atomic<std::uint32_t> hold_last_seq_{0};
    std::atomic<std::uint8_t> hold_last_result_{0};
#ifdef C43_ENABLE_TEST_HOOKS
    /// TEST-ONLY injection (see on_configure): forces the next N consecutive
    /// cycles to report a deadline overrun, so the E4 policy (5 consecutive)
    /// can be exercised deterministically.  Only present in C43_ENABLE_TEST_HOOKS
    /// builds; production builds do not carry the parameter or the members.
    std::atomic<int> test_fault_budget_{0};
    int test_force_deadline_fault_cycles_ = 0;
    /// TEST-ONLY (T-H17): forces N cycles through the readState() error exit.
    std::atomic<int> readstate_fail_budget_{0};
    std::atomic<int> readstate_after_budget_{0};
    int test_fail_readstate_after_cycles_ = 0;
    int test_fail_readstate_cycles_ = 0;
#endif
    int status_tick_ = 0;
    int status_period_ticks_ = 13;  // 125 Hz / 10 Hz (overwritten by the parameter)
    // C7-A Rev.2.1 (P-1): force a status publish when a NEW ack appears, so the
    // ack latency does not inherit the 10 Hz decimation.  Updated only after a
    // successful trylock, so a dropped publish is retried on the next tick.
    std::uint64_t status_published_ack_ = 0;

    // C4.3 S4 W3-R1a / T-H13: cycles in which publishStatus() reached the
    // publisher but lost its trylock (a real P6 that published nothing).
    // Counted only while timing is enabled, so the production path is untouched.
    // Written on the RT thread, read by the timing_dump service: relaxed.
    std::atomic<std::uint64_t> publish_trylock_miss_{0};
  /// One-shot log guard for a refused command write (RT path, log once not per cycle).
  bool write_refused_logged_ = false;

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

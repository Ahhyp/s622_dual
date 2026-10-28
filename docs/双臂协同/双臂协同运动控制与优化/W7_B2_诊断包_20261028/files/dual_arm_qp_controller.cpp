// src/dual_arm_qp_controller.cpp

#include "dual_arm_qp_controller/hold_ack_policy.hpp"
#include "dual_arm_qp_controller/dual_arm_qp_controller.hpp"
#include "dual_arm_qp_controller/frozen_baseline.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/SVD>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace {

/// C7-A Rev.2: pack the ack into ONE atomic (seq high 32, result low 8).
using dual_arm_qp_controller::kAckApplied;
using dual_arm_qp_controller::kAckRefused;
using dual_arm_qp_controller::packHoldAck;

/// S4 W1 review (A): map the interlock's free-text reason onto a stable code.
/// The old code emitted "stale_state" for EVERY non-deadline health failure,
/// including tracking errors and solve streaks -- a code named after a fault it
/// never measured.  Genuine state staleness is not detectable at this layer
/// (see protection: e2 is "degraded"), so the name is now reserved for it.
const char* healthHoldCode(const dual_arm_qp_controller::StreamHealth& h) {
    if (h.reason == "control period overrun") return "deadline_missed";
    if (h.reason == "non-finite state/command") return "non_finite_state";
    if (h.reason == "injected stream stall") return "stall_injected";
    if (h.reason == "QP solve failures") return "qp_solver_failed";
    if (h.reason.compare(0, 14, "tracking error") == 0) return "tracking_error";
    return "stream_unhealthy";
}

}  // namespace

namespace dual_arm_qp_controller {

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

namespace {
constexpr int kNumJoints = 12;
constexpr double kTwoPi = 6.283185307179586;
constexpr double kLimitTol = 1e-9;
/// Plan §4.7.1 E4: consecutive deadline overruns before the D7 hold latches.
constexpr int kDeadlineFaultStreakMax = 5;

/// Row-major 4x4 -> Iso3, rejecting anything that is not a rigid transform.
bool parseIso3(const std::vector<double>& v, dual_arm::Iso3* out) {
    if (v.size() != 16) return false;
    Eigen::Matrix4d m;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) m(r, c) = v[static_cast<size_t>(r * 4 + c)];
    }
    if (std::abs(m(3, 0)) > 1e-9 || std::abs(m(3, 1)) > 1e-9 || std::abs(m(3, 2)) > 1e-9 ||
        std::abs(m(3, 3) - 1.0) > 1e-9) {
        return false;
    }
    const Eigen::Matrix3d R = m.topLeftCorner<3, 3>();
    if ((R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-6) return false;
    if (std::abs(R.determinant() - 1.0) > 1e-6) return false;
    dual_arm::Iso3 iso = dual_arm::Iso3::Identity();
    iso.linear() = R;
    iso.translation() = m.topRightCorner<3, 1>();
    *out = iso;
    return true;
}
}  // namespace

InterfaceConfiguration DualArmQpController::command_interface_configuration() const {
    InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names = command_iface_names_;
    return cfg;
}

InterfaceConfiguration DualArmQpController::state_interface_configuration() const {
    InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names = state_iface_names_;
    return cfg;
}

CallbackReturn DualArmQpController::on_init() {
    declareIfNeeded<std::vector<std::string>>("joints", {});
#ifdef C43_ENABLE_TEST_HOOKS
    // TEST-ONLY (C43_ENABLE_TEST_HOOKS=ON builds only).  When the option is OFF
    // this parameter does not exist at all, so injection cannot be enabled at
    // run time either -- see docs/.../C4.3_S4_W2_测试钩子隔离报告.md.
    declareIfNeeded<int>("test_force_deadline_fault_cycles", 0);
    // TEST-ONLY (T-H17): forces N cycles through the same error exit a
    // real readState() failure takes (return ERROR, before the hold drain
    // and before publishStatus).
    declareIfNeeded<int>("test_fail_readstate_cycles", 0);
    // TEST-ONLY: control-cycle delay before the first injected failure
    // (0 = fail on the first armed cycle).  Counted from on_activate.
    declareIfNeeded<int>("test_fail_readstate_after_cycles", 0);
#endif
    declareIfNeeded<double>("dt", 0.008);
    declareIfNeeded<double>("kp_rel", 10.0);
    declareIfNeeded<double>("w_rel", 1.0);
    declareIfNeeded<double>("w_slack", 1e3);
    // C4.3 S4 W1: 1e-4 is the measured knee -- 1e-2 costs 1.7e-3 of steady-state
    // tracking error (and fails S3-T6), 1e-4 costs 1.7e-5 at the same step size.
    declareIfNeeded<double>("w_center", 1e-4);
    declareIfNeeded<double>("w_reg", 1e-6);
    declareIfNeeded<double>("v_max", 2.5);
    // D7 health / fault injection
    declareIfNeeded<double>("tracking_error_limit", 0.0);
    declareIfNeeded<int>("solve_failure_limit", 5);
    declareIfNeeded<int>("stall_inject_arm", -1);
    declareIfNeeded<double>("stall_inject_start_s", -1.0);
    declareIfNeeded<double>("stall_inject_duration_s", 0.0);
    // test-only stimulus (0 = off)
    declareIfNeeded<double>("test_sweep_amp_m", 0.0);
    declareIfNeeded<double>("test_sweep_period_s", 4.0);
    declareIfNeeded<double>("period_fault_factor", 2.0);
    declareIfNeeded<double>("collision_d_stop", 0.0);
    declareIfNeeded<double>("collision_link_radius", 0.06);

    // ---- C4.3 S4 W1: object / coordination task -------------------------
    declareIfNeeded<double>("kp_obj", 20.0);
    declareIfNeeded<double>("w_obj", 1.0);
    // S4 W1 review (A/B): honest protection + standing-configuration check.
    // W7 fix: defaultHomeQ() returns an Eigen vector BY VALUE, so calling it twice
    // inside the range constructor produced two distinct temporaries: `first`
    // pointed into the first, `last` into the second, and the resulting distance
    // was whatever the stack layout happened to give (measured live: 140 elements,
    // 12 valid + 128 of stack garbage).  Keep ONE copy and take the range from it.
    const dual_arm::Q12 home_q_default = defaultHomeQ();
    declareIfNeeded<std::vector<double>>("home_q", std::vector<double>(
        home_q_default.data(), home_q_default.data() + home_q_default.size()));
    declareIfNeeded<double>("start_pose_tolerance_rad", 0.05);
    declareIfNeeded<bool>("require_executor_heartbeat", false);
    declareIfNeeded<double>("executor_heartbeat_timeout_s", 0.1);
    declareIfNeeded<double>("w_slack_obj", 1e3);
    declareIfNeeded<double>("w_null_margin", 0.0);
    declareIfNeeded<double>("w_null_manip", 0.0);
    declareIfNeeded<double>("w_null_clear", 0.0);
    declareIfNeeded<double>("null_step", 0.005);
    declareIfNeeded<bool>("enable_singularity", false);
    declareIfNeeded<double>("sigma_critical", 0.06);
    declareIfNeeded<double>("sigma_safe", 0.20);
    declareIfNeeded<double>("damp_max", 50.0);
    declareIfNeeded<double>("slack_relax", 0.9);
    declareIfNeeded<double>("vel_scale_min", 0.5);
    declareIfNeeded<double>("e_safe", 1.2);
    declareIfNeeded<double>("e_critical", 2.4);
    declareIfNeeded<std::vector<double>>("dq_center", {});
    // Frozen S2 baseline grasps (object_T_left_tcp / object_T_right_tcp),
    // row-major 4x4.  Never re-derived from measurements (S4 plan §4.4).
    declareIfNeeded<std::vector<double>>("object_T_left_tcp", frozenObjectTLeftTcpVector());
    declareIfNeeded<std::vector<double>>("object_T_right_tcp", frozenObjectTRightTcpVector());
    declareIfNeeded<double>("obj_rot_error_refuse_rad", 3.0915926535897933);
    declareIfNeeded<int>("solve_fail_streak_max", 10);
    declareIfNeeded<double>("transport_status_rate_hz", 10.0);
    // §7 S4-T2 / S4-T17: realtime instrumentation.  Off by default so a
    // production controller pays nothing and W1 behaviour is unchanged.
    declareIfNeeded<bool>("timing_enable", false);
    declareIfNeeded<int>("timing_capacity", 8000);
    declareIfNeeded<std::string>("timing_csv_path", "");
    // C4.3 S4 W3-R1a / T-H13: phase mask for the six update() phases.  0 keeps
    // only P0/P7 (the W1 measurement), 0x3F (=63) records P1..P6.
    declareIfNeeded<int>("timing_phases", 0);

    const auto joints = get_node()->get_parameter("joints").as_string_array();
    if (joints.size() != kNumJoints) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "parameter 'joints' must list %d joint names (got %zu)", kNumJoints,
                     joints.size());
        return CallbackReturn::ERROR;
    }
    joint_names_ = joints;
    command_iface_names_.clear();
    state_iface_names_.clear();
    for (const auto& j : joint_names_) {
        command_iface_names_.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
        state_iface_names_.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmQpController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    dt_ = get_node()->get_parameter("dt").as_double();
    kp_rel_ = get_node()->get_parameter("kp_rel").as_double();
    w_rel_ = get_node()->get_parameter("w_rel").as_double();
    w_slack_ = get_node()->get_parameter("w_slack").as_double();
    w_center_ = get_node()->get_parameter("w_center").as_double();
#ifdef C43_ENABLE_TEST_HOOKS
    test_force_deadline_fault_cycles_ =
        static_cast<int>(get_node()->get_parameter("test_force_deadline_fault_cycles").as_int());
    test_fail_readstate_cycles_ =
        static_cast<int>(get_node()->get_parameter("test_fail_readstate_cycles").as_int());
    test_fail_readstate_after_cycles_ =
        static_cast<int>(get_node()->get_parameter("test_fail_readstate_after_cycles").as_int());
#endif
    w_reg_ = get_node()->get_parameter("w_reg").as_double();
    v_max_default_ = get_node()->get_parameter("v_max").as_double();
    test_sweep_amp_m_ = get_node()->get_parameter("test_sweep_amp_m").as_double();
    test_sweep_period_s_ = get_node()->get_parameter("test_sweep_period_s").as_double();
    period_fault_factor_ = get_node()->get_parameter("period_fault_factor").as_double();
    collision_d_stop_ = get_node()->get_parameter("collision_d_stop").as_double();
    collision_link_radius_ = get_node()->get_parameter("collision_link_radius").as_double();

    kp_obj_ = get_node()->get_parameter("kp_obj").as_double();
    w_obj_ = get_node()->get_parameter("w_obj").as_double();
    w_slack_obj_ = get_node()->get_parameter("w_slack_obj").as_double();
    obj_rot_error_refuse_rad_ = get_node()->get_parameter("obj_rot_error_refuse_rad").as_double();
    solve_fail_streak_max_ = std::max(1, static_cast<int>(
                                           get_node()->get_parameter("solve_fail_streak_max").as_int()));
    status_rate_hz_ = get_node()->get_parameter("transport_status_rate_hz").as_double();
    start_pose_tolerance_rad_ =
        get_node()->get_parameter("start_pose_tolerance_rad").as_double();
    require_executor_heartbeat_ =
        get_node()->get_parameter("require_executor_heartbeat").as_bool();
    executor_heartbeat_timeout_s_ =
        get_node()->get_parameter("executor_heartbeat_timeout_s").as_double();
    {
        const auto hv = get_node()->get_parameter("home_q").as_double_array();
        if (hv.size() == 12) {
            for (int i = 0; i < 12; ++i) home_q_[i] = hv[static_cast<size_t>(i)];
        } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "home_q must have 12 values (got %zu); using the S2 q_grasp default",
                         hv.size());
        }
    }

    if (w_obj_ > 0.0 && w_slack_obj_ < 1e2) {
        // A toothless object task: slack would absorb the whole residual and the
        // QP would "converge" without ever moving the object (S3-T5h).
        RCLCPP_ERROR(get_node()->get_logger(),
                     "w_obj=%.3g > 0 requires w_slack_obj >= 1e2 (got %.3g); the object task "
                     "would be toothless", w_obj_, w_slack_obj_);
        return CallbackReturn::ERROR;
    }

    const auto dq_center_param = get_node()->get_parameter("dq_center").as_double_array();
    dq_center_.setZero();
    if (!dq_center_param.empty()) {
        if (dq_center_param.size() != static_cast<size_t>(kNumJoints)) {
            RCLCPP_ERROR(get_node()->get_logger(), "dq_center must have %d entries", kNumJoints);
            return CallbackReturn::ERROR;
        }
        for (int i = 0; i < kNumJoints; ++i) dq_center_[i] = dq_center_param[static_cast<size_t>(i)];
    }
    if (!parseIso3(get_node()->get_parameter("object_T_left_tcp").as_double_array(),
                   &object_T_left_tcp_)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "object_T_left_tcp must be 16 row-major values of a rigid transform");
        return CallbackReturn::ERROR;
    }
    if (!parseIso3(get_node()->get_parameter("object_T_right_tcp").as_double_array(),
                   &object_T_right_tcp_)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "object_T_right_tcp must be 16 row-major values of a rigid transform");
        return CallbackReturn::ERROR;
    }

    StreamHealthParams hp;
    hp.tracking_error_limit = get_node()->get_parameter("tracking_error_limit").as_double();
    hp.solve_failure_limit = get_node()->get_parameter("solve_failure_limit").as_int();
    hp.stall_inject_arm = get_node()->get_parameter("stall_inject_arm").as_int();
    hp.stall_start_s = get_node()->get_parameter("stall_inject_start_s").as_double();
    hp.stall_duration_s = get_node()->get_parameter("stall_inject_duration_s").as_double();
    health_.configure(hp);
    period_monitor_.configure(dt_, period_fault_factor_);

    dual_arm_qp::defaultS622Limits(&q_min_, &q_max_, &v_max_);
    if (v_max_default_ > 0.0) v_max_.setConstant(v_max_default_);

    // W1: EVERY DualArmQpParams field is now sourced from a parameter.  Before
    // W1 only 9 of 25 were written, so kp_obj/w_obj/w_slack_obj in the Gazebo
    // yaml were dead configuration (S4 plan §2, gap G-3).
    dual_arm_qp::DualArmQpParams qp_params;
    qp_params.dt = dt_;
    qp_params.kp_rel = kp_rel_;
    qp_params.w_rel = w_rel_;
    qp_params.w_slack = w_slack_;
    qp_params.w_center = w_center_;
    qp_params.w_reg = w_reg_;
    qp_params.kp_obj = kp_obj_;
    // The object task stays DISABLED while no trajectory is latched: enabling it
    // would activate the legacy symmetric-midpoint object frame (C4.3 S3 R24).
    qp_params.w_obj = 0.0;
    qp_params.w_slack_obj = w_slack_obj_;
    qp_params.w_null_margin = get_node()->get_parameter("w_null_margin").as_double();
    qp_params.w_null_manip = get_node()->get_parameter("w_null_manip").as_double();
    qp_params.w_null_clear = get_node()->get_parameter("w_null_clear").as_double();
    qp_params.null_step = get_node()->get_parameter("null_step").as_double();
    qp_params.enable_singularity = get_node()->get_parameter("enable_singularity").as_bool();
    qp_params.sigma_critical = get_node()->get_parameter("sigma_critical").as_double();
    qp_params.sigma_safe = get_node()->get_parameter("sigma_safe").as_double();
    qp_params.damp_max = get_node()->get_parameter("damp_max").as_double();
    qp_params.slack_relax = get_node()->get_parameter("slack_relax").as_double();
    qp_params.vel_scale_min = get_node()->get_parameter("vel_scale_min").as_double();
    qp_params.e_safe = get_node()->get_parameter("e_safe").as_double();
    qp_params.e_critical = get_node()->get_parameter("e_critical").as_double();
    qp_params.q_min = q_min_;
    qp_params.q_max = q_max_;
    qp_params.v_max = v_max_;
    qp_params.dq_center = dq_center_;
    // CBF rows stay empty: the CBF is not part of S4.1.
    qp_params.C_ineq.resize(0, 24);
    qp_params.d_ineq.resize(0);

    kinematics_ = std::make_unique<dual_arm::DualArmKinematics>();
    if (collision_d_stop_ > 0.0) {
        collision_ = std::make_unique<dual_arm_collision::DualArmCollisionModel>(
            *kinematics_, collision_link_radius_);
    }
    qp_ = std::make_unique<dual_arm_qp::DualArmQp>(*kinematics_, qp_params);
    qp_->setWarmStart(true);

    TransportTaskConfig tcfg;
    tcfg.object_T_left_tcp = object_T_left_tcp_;
    tcfg.object_T_right_tcp = object_T_right_tcp_;
    tcfg.obj_rot_error_refuse_rad = obj_rot_error_refuse_rad_;
    tcfg.dt = dt_;
    tcfg.solve_fail_streak_max = solve_fail_streak_max_;
    tcfg.home_q = home_q_;
    tcfg.start_pose_tolerance_rad = start_pose_tolerance_rad_;
    transport_ = std::make_unique<TransportTask>(*kinematics_, tcfg);

    // S4 W1 review (A): publish what is actually protected, not what the plan
    // intended.  A controller that says "protected" while E2/E8 are absent is
    // worse than one that admits the gap.
    protection_ = buildProtectionReport(collision_d_stop_ > 0.0, require_executor_heartbeat_);
    if (protection_.complete) {
        RCLCPP_INFO(get_node()->get_logger(), "protection: %s", protection_.summary.c_str());
    } else {
        RCLCPP_WARN(get_node()->get_logger(),
                    "protection is NOT complete: %s (see ~/status.protection)",
                    protection_.summary.c_str());
    }

    status_period_ticks_ = std::max(
        1, static_cast<int>(std::lround(1.0 / (dt_ * std::max(1e-3, status_rate_hz_)))));

    // §7 S4-T2/T17: one allocation here, never in the control loop.
    timing_enable_ = get_node()->get_parameter("timing_enable").as_bool();
    timing_capacity_ = static_cast<int>(get_node()->get_parameter("timing_capacity").as_int());
    timing_csv_path_ = get_node()->get_parameter("timing_csv_path").as_string();
    // C4.3 S4 W3-R1a: phases are recorded only when timing is enabled AND the
    // mask asks for them; the mask survives a timing_enable=false configure so
    // the three A/B/C profiles differ in exactly one parameter each.
    timing_phases_ = static_cast<int>(get_node()->get_parameter("timing_phases").as_int());
    timing_.configure(timing_enable_ ? std::max(1, timing_capacity_) : 0, 1.0e6 * dt_,
                      timing_enable_ ? timing_phases_ : 0);
    createInterfaces();

    RCLCPP_INFO(get_node()->get_logger(),
                "configured: 12-DOF QP, dt=%.4fs, kp_rel=%.2f, kp_obj=%.2f, w_obj=%.2f, "
                "w_slack_obj=%.3g, w_center=%.3g, solver=%s",
                dt_, kp_rel_, kp_obj_, w_obj_, w_slack_obj_, w_center_,
                qp_->solverName().c_str());
    return CallbackReturn::SUCCESS;
}

void DualArmQpController::createInterfaces() {
    start_req_ = std::make_unique<StartBuffer>();

    auto status_topic = get_node()->create_publisher<dual_arm_qp_controller::msg::TransportStatus>(
        "~/transport_status", rclcpp::QoS(10));
    status_pub_ = std::make_unique<StatusPublisher>(status_topic);

    start_srv_ = get_node()->create_service<dual_arm_qp_controller::srv::StartTransport>(
        "~/start_transport",
        [this](const std::shared_ptr<dual_arm_qp_controller::srv::StartTransport::Request> req,
               std::shared_ptr<dual_arm_qp_controller::srv::StartTransport::Response> res) {
            onStartTransport(req, res);
        });
    abort_srv_ = get_node()->create_service<dual_arm_qp_controller::srv::AbortTransport>(
        "~/abort_transport",
        [this](const std::shared_ptr<dual_arm_qp_controller::srv::AbortTransport::Request> req,
               std::shared_ptr<dual_arm_qp_controller::srv::AbortTransport::Response> res) {
            onAbortTransport(req, res);
        });
    hold_srv_ = get_node()->create_service<dual_arm_qp_controller::srv::HoldTransport>(
        "~/hold_transport",
        [this](const std::shared_ptr<dual_arm_qp_controller::srv::HoldTransport::Request> req,
               std::shared_ptr<dual_arm_qp_controller::srv::HoldTransport::Response> res) {
            onHoldTransport(req, res);
        });
    resume_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/resume_transport",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) { onResume(res); });
    // S4 W1 review (A): E8.  The executor (W2) publishes ~/heartbeat; if this
    // interlock is required and the heartbeat goes stale, both arms hold.
    // Default OFF because S4.1 has no executor yet -- and ~/status says so.
    heartbeat_sub_ = get_node()->create_subscription<std_msgs::msg::Empty>(
        "~/heartbeat", rclcpp::QoS(10),
        [this](std_msgs::msg::Empty::ConstSharedPtr msg) { onHeartbeat(msg); });
    timing_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/timing_dump",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) { onTimingDump(res); });
    reset_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/reset_transport",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) { onReset(res); });
}

CallbackReturn DualArmQpController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!readState()) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_activate: failed to read 12 joint states");
        return CallbackReturn::ERROR;
    }
    q_cmd_ = q_state_;
    target_ = kinematics_->leftTright(q_state_);
    activate_time_ = get_node()->now();
    solve_fail_streak_ = 0;
    deadline_fault_streak_ = 0;
    hold_active_ = false;
    health_.reset();
    period_monitor_.reset();
    stats_counter_ = 0;
    status_tick_ = 0;
    activated_ = true;
    // A freshly activated controller never resumes a stale transport.
    if (transport_ != nullptr) transport_->reset();
    pending_cmd_.store(0, std::memory_order_relaxed);
    if (start_req_ != nullptr) start_req_->writeFromNonRT(std::shared_ptr<StartRequest>());
    writeCommand(q_cmd_);

    RCLCPP_INFO(get_node()->get_logger(),
                "activated: relative target captured, q0=[%.3f %.3f %.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f %.3f %.3f]",
                q_state_[0], q_state_[1], q_state_[2], q_state_[3], q_state_[4], q_state_[5],
                q_state_[6], q_state_[7], q_state_[8], q_state_[9], q_state_[10], q_state_[11]);
    return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmQpController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    activated_ = false;
    if (transport_ != nullptr) transport_->reset();
    pending_cmd_.store(0, std::memory_order_relaxed);
    if (start_req_ != nullptr) start_req_->writeFromNonRT(std::shared_ptr<StartRequest>());
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// transport service callbacks (executor thread -- never the control path)
// ---------------------------------------------------------------------------
void DualArmQpController::onStartTransport(
    const std::shared_ptr<dual_arm_qp_controller::srv::StartTransport::Request> req,
    std::shared_ptr<dual_arm_qp_controller::srv::StartTransport::Response> res) {
    if (!activated_) {
        res->accepted = false;
        res->message = "controller is not active";
        return;
    }
    // B (S4 W1 review): refuse to start from an arbitrary pose.  The anchored
    // start pose is derived from the CURRENT joint state, so starting somewhere
    // else silently redefines "the object is here".  TransportTask::start()
    // enforces the same rule on the control thread; this is the friendly path.
    if (start_pose_tolerance_rad_ > 0.0) {
        double worst = 0.0;
        int worst_i = 0;
        for (int i = 0; i < kNumJoints; ++i) {
            const double d = std::abs(q_state_[i] - home_q_[i]);
            if (d > worst) {
                worst = d;
                worst_i = i;
            }
        }
        if (!(worst <= start_pose_tolerance_rad_)) {
            res->accepted = false;
            res->message = "start pose deviation " + std::to_string(worst) +
                           " rad at joint " + std::to_string(worst_i % 6 + 1) + " (" +
                           (worst_i < 6 ? "left" : "right") + " arm) exceeds " +
                           std::to_string(start_pose_tolerance_rad_) +
                           " rad; move both arms to the standing configuration first";
            return;
        }
    }
#ifdef C43_ENABLE_TEST_HOOKS
    // TEST-ONLY: arm the deterministic E4 injection for this run.
    test_fault_budget_.store(test_force_deadline_fault_cycles_, std::memory_order_relaxed);
    readstate_fail_budget_.store(test_fail_readstate_cycles_, std::memory_order_relaxed);
    readstate_after_budget_.store(test_fail_readstate_after_cycles_, std::memory_order_relaxed);
#endif
    const size_t n = req->phase_duration.size();
    if (n == 0) {
        res->accepted = false;
        res->message = "phase_duration is empty";
        return;
    }
    if (req->phase_dx.size() != n || req->phase_dy.size() != n || req->phase_dz.size() != n) {
        res->accepted = false;
        res->message = "phase_dx/phase_dy/phase_dz must match phase_duration";
        return;
    }
    auto request = std::make_shared<StartRequest>();
    auto& spec = request->spec;
    spec.dt = req->dt > 0.0 ? req->dt : dt_;
    spec.time_scaling = req->time_scaling.empty() ? "smoothstep" : req->time_scaling;
    for (size_t i = 0; i < n; ++i) {
        dual_arm_object_task::TrajectoryPhase ph;
        ph.name = (req->phase_name.size() == n) ? req->phase_name[i] : std::string("phase") +
                                                                         std::to_string(i);
        ph.duration = req->phase_duration[i];
        ph.dx = req->phase_dx[i];
        ph.dy = req->phase_dy[i];
        ph.dz = req->phase_dz[i];
        if (!(ph.duration > 0.0) || !std::isfinite(ph.dx) || !std::isfinite(ph.dy) ||
            !std::isfinite(ph.dz)) {
            res->accepted = false;
            res->message = "phase " + std::to_string(i) + " is not finite / has duration <= 0";
            return;
        }
        spec.phases.push_back(ph);
    }
    // Start pose: 12 values = row-major 3x4, otherwise the anchored estimate at
    // the current joint state (resolved on the control thread).
    if (!req->start_world_t_object.empty()) {
        if (req->start_world_t_object.size() != 12) {
            res->accepted = false;
            res->message = "start_world_t_object must have 12 values (row-major 3x4)";
            return;
        }
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                m(r, c) = req->start_world_t_object[static_cast<size_t>(r * 4 + c)];
            }
        }
        dual_arm::Iso3 iso = dual_arm::Iso3::Identity();
        iso.linear() = m.topLeftCorner<3, 3>();
        iso.translation() = m.topRightCorner<3, 1>();
        spec.start_world_T_object = iso;
        spec.start_from_baseline = false;
    } else {
        spec.start_from_baseline = true;
    }
    spec.yaw_sweep_deg = req->yaw_sweep_deg.empty()
                             ? std::vector<double>{0.0}
                             : std::vector<double>(req->yaw_sweep_deg.begin(),
                                                   req->yaw_sweep_deg.end());
    if (!(spec.yaw_sweep_deg.size() == 1 || spec.yaw_sweep_deg.size() == n + 1)) {
        res->accepted = false;
        res->message = "yaw_sweep_deg must have 1 or phases+1 entries";
        return;
    }
    // Validate shape eagerly so a malformed spec is rejected here, not mid-run.
    try {
        const dual_arm_object_task::FrozenObjectTrajectory probe(spec);
        (void)probe;
    } catch (const std::exception& e) {
        res->accepted = false;
        res->message = std::string("trajectory rejected: ") + e.what();
        return;
    }
    start_req_->writeFromNonRT(request);
    // §7 S4-T17: rebase the resource baselines now, so the reported growth is
    // the drift of the run and not the ROS graph allocation of the first
    // second.  Executor thread: no RT syscall.
    timing_.rebaseResources();
    res->accepted = true;
    res->message = "queued (applied on the next control cycle)";
}

void DualArmQpController::onAbortTransport(
    const std::shared_ptr<dual_arm_qp_controller::srv::AbortTransport::Request> req,
    std::shared_ptr<dual_arm_qp_controller::srv::AbortTransport::Response> res) {
    // The reason is advisory: the control thread latches a fixed, loggable code.
    RCLCPP_WARN(get_node()->get_logger(), "abort requested: %s", req->reason.c_str());
    pending_cmd_.store(3, std::memory_order_relaxed);
    res->accepted = true;
    res->message = "abort queued";
}

void DualArmQpController::onHoldTransport(
    const std::shared_ptr<dual_arm_qp_controller::srv::HoldTransport::Request> req,
    std::shared_ptr<dual_arm_qp_controller::srv::HoldTransport::Response> res) {
    // C7-A Rev.2 §4.6: validation and queueing happen on this (non-RT) thread;
    // the control thread only ever sees a seq and must not block or allocate.
    if (req == nullptr || res == nullptr) return;
    res->accepted = false;
    if (req->seq == 0u) {
        res->message = "invalid seq: 0 is reserved for \"no request\"";
        return;
    }
    // Single slot.  A second request while one is queued is REFUSED, never
    // overwritten -- otherwise the first seq would be lost without an ack and
    // the executor would have to choose between waiting forever and aborting.
    std::uint32_t expected = 0;
    if (!hold_req_seq_.compare_exchange_strong(expected, req->seq,
                                               std::memory_order_acq_rel,
                                               std::memory_order_acquire)) {
        res->message = "busy: hold request seq=" + std::to_string(expected) +
                       " is still queued (one request per control cycle)";
        return;
    }
    RCLCPP_WARN(get_node()->get_logger(), "hold requested: seq=%u reason='%s'",
                static_cast<unsigned>(req->seq), req->reason.c_str());
    res->accepted = true;
    res->message = "hold queued";
}

void DualArmQpController::onResume(
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    // C (S4 W1 review): ~/resume only acts on a HOLD.  FAULT needs ~/reset and
    // ABORT needs a new ~/start_transport; TransportTask enforces this on the
    // control thread and reports which of the three applies.
    pending_cmd_.store(1, std::memory_order_relaxed);
    res->success = true;
    res->message = "resume queued (HOLD only; FAULT needs ~/reset_transport, ABORT needs "
                   "a new ~/start_transport)";
}

void DualArmQpController::onHeartbeat(std_msgs::msg::Empty::ConstSharedPtr /*msg*/) {
    // Executor thread.  Two relaxed atomics: update() only needs a recent value,
    // never a consistent pair.
    last_heartbeat_s_.store(get_node()->now().seconds(), std::memory_order_relaxed);
    heartbeat_seen_.store(true, std::memory_order_relaxed);
}

void DualArmQpController::onReset(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    pending_cmd_.store(2, std::memory_order_relaxed);
    res->success = true;
    res->message = "reset queued";
}

void DualArmQpController::onTimingDump(
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    // §7 S4-T2 / S4-T17.  Non-RT: this runs on the executor thread, sorts
    // copies of the samples and writes the CSV.  The control loop keeps
    // running and keeps appending; a sample is only ever seen whole because
    // the counter is published with a release store.
    if (!timing_.enabled()) {
        res->success = false;
        res->message =
            "timing_enable is false; set it in the params file and restart the controller";
        return;
    }
    const TimingRecorder::Summary summary = timing_.summarize();
    std::string err;
    const bool csv_ok = timing_csv_path_.empty() || timing_.dumpCsv(timing_csv_path_, &err);
    res->success = csv_ok;
    res->message = TimingRecorder::format(summary);
    // C4.3 S4 W3-R1a: a P6 that produced a duration but published nothing.
    res->message += " | publish_trylock_miss=" +
                    std::to_string(publish_trylock_miss_.load(std::memory_order_relaxed));
    if (timing_csv_path_.empty()) {
        res->message += " | timing_csv_path empty: stats only";
    } else if (csv_ok) {
        res->message += " | csv=" + timing_csv_path_;
    } else {
        res->message += " | csv write failed: " + err;
    }
}

void DualArmQpController::markTimingPhase(int phase) {
    // C4.3 S4 W3-R1a / T-H13.  The guard is first on purpose: with the phase
    // masked off no clock is read, so `timing_phases=0` leaves the control
    // output bit-identical and the only cost is one predictable branch.
    if (!timing_.phaseEnabled(phase)) return;
    timing_.mark(phase, TimingScope::nowNs());
}

void DualArmQpController::applyPendingStart() {
    if (start_req_ == nullptr) return;
    std::shared_ptr<StartRequest>* slot = start_req_->readFromRT();
    if (slot == nullptr || !(*slot)) return;
    const std::shared_ptr<StartRequest> request = *slot;
    // Clear the mailbox first: a failed latch must not be retried forever.
    start_req_->writeFromNonRT(std::shared_ptr<StartRequest>());

    std::string err;
    if (!transport_->start(request->spec, q_state_, &err)) {
        RCLCPP_ERROR(get_node()->get_logger(), "start_transport rejected: %s", err.c_str());
        transport_->fault("trajectory_invalid");
        return;
    }
    // Enable the object task only now: in IDLE it must stay off, otherwise the
    // legacy symmetric-midpoint object frame becomes active (R24).
    dual_arm_qp::DualArmQpParams params = qp_->params();
    params.w_obj = w_obj_;
    qp_->setParams(params);
    last_failure_code_ = "";
    last_committed_ = false;
    RCLCPP_INFO(get_node()->get_logger(),
                "transport latched: duration=%.3fs, phases=%zu, anchored start p=[%.6f %.6f %.6f]",
                transport_->duration(), request->spec.phases.size(),
                request->spec.start_world_T_object.translation().x(),
                request->spec.start_world_T_object.translation().y(),
                request->spec.start_world_T_object.translation().z());
}

bool DualArmQpController::jointStateInBounds(double* min_margin) const {
    double margin = std::numeric_limits<double>::infinity();
    bool ok = true;
    for (int i = 0; i < kNumJoints; ++i) {
        const double lo = q_state_[i] - q_min_[i];
        const double hi = q_max_[i] - q_state_[i];
        margin = std::min(margin, std::min(lo, hi));
        if (q_state_[i] < q_min_[i] - kLimitTol || q_state_[i] > q_max_[i] + kLimitTol) ok = false;
    }
    if (min_margin != nullptr) *min_margin = margin;
    return ok;
}

return_type DualArmQpController::update(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) {
    if (!activated_) return return_type::OK;
    TimingScope timing_scope(timing_);
    // C4.3 S4 W3-R1a / T-H13.  Default label for a cycle that never reaches a
    // branch (early_exit/0); every exit path below overwrites it with its own
    // (branch, exit_code) pair.  Two stores, no clock read.
    timing_.setOutcome(2, 0);
    if (!readState()) {
        // D3-a: a request queued for this tick must not be lost because the
        // state read failed.  The slot is drained and REFUSED explicitly, and
        // the ack is published before ERROR propagates, so the executor learns
        // "refused: nothing was frozen" instead of timing out and inventing an
        // ABORT.  transport_->hold() is deliberately NOT attempted: the joints
        // we would freeze are exactly the ones we cannot read.
        const std::uint32_t lost_seq = hold_req_seq_.exchange(0, std::memory_order_acq_rel);
        const auto fail_ack = ackForReadStateFailure(lost_seq);
        if (fail_ack.publish) {
            hold_last_result_.store(fail_ack.result, std::memory_order_release);
            hold_last_seq_.store(fail_ack.seq, std::memory_order_release);
            hold_ack_.store(packHoldAck(fail_ack.seq, fail_ack.result), std::memory_order_release);
            publishStatus();
        }
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "QP controller: non-finite joint state");
        timing_.setOutcome(2, 1);  // 1 = readState() rejected the state
        return return_type::ERROR;
    }
    markTimingPhase(0);  // P1: readState()
#ifdef C43_ENABLE_TEST_HOOKS
    // TEST-ONLY (T-H17).  Injected *after* readState() returned true: this
    // exercises the error return path and the ordering guarantee that it runs
    // before the hold drain and before publishStatus(), so a queued request can
    // be neither consumed nor acked in this cycle.  It does not reproduce the
    // side effects inside readState() itself; see the R1a-2 record for that
    // coverage boundary.
    if (readstate_fail_budget_.load(std::memory_order_relaxed) > 0) {
        const int after = readstate_after_budget_.load(std::memory_order_relaxed);
        if (after > 0) {
            readstate_after_budget_.store(after - 1, std::memory_order_relaxed);
        } else {
            readstate_fail_budget_.fetch_sub(1, std::memory_order_relaxed);
            timing_.setOutcome(2, 2);  // 2 = injected readState()-failure hook
            return return_type::ERROR;
        }
    }
#endif

    // ---- apply queued transport commands (allocation-free except a latch) ----
    const int cmd = pending_cmd_.exchange(0, std::memory_order_relaxed);
    if (cmd == 2) {
        transport_->reset();
        dual_arm_qp::DualArmQpParams params = qp_->params();
        params.w_obj = 0.0;
        qp_->setParams(params);
        target_ = kinematics_->leftTright(q_state_);
    } else if (cmd == 1) {
        if (!transport_->resume(q_state_)) {
            RCLCPP_WARN(get_node()->get_logger(), "resume refused: %s",
                        transport_->failureCode());
        }
    } else if (cmd == 3) {
        transport_->abort("aborted_by_request");
    }
    applyPendingStart();

    const bool transport_active = transport_ != nullptr && transport_->active();

    // ---- C7-A Rev.2: executor HOLD request -----------------------------------
    // Placed AFTER the command dispatch and AFTER transport_active is known, but
    // BEFORE the safety layers.  That ordering is load-bearing:
    //   * before the safety layers  -> every later path publishes the ack in the
    //     SAME tick (all post-:661 exits call publishStatus());
    //   * after  applyPendingStart  -> a start() latched this tick is already
    //     visible, so "hold the transport I just started" works;
    //   * unconditional (outside `if (transport_active)`) -> the slot always
    //     drains, so an Idle-state request is acked instead of wedging the slot
    //     into permanent "busy".
    // No early return: the ack is written here and published further down.
    const std::uint32_t hold_seq = hold_req_seq_.exchange(0, std::memory_order_acq_rel);
    if (hold_seq != 0) {
        if (hold_seq == hold_last_seq_.load(std::memory_order_acquire)) {
            // D3-a: retransmission of an ALREADY CONSUMED seq.  Two rules, both
            // load-bearing:
            //   * never call transport_->hold() again -- re-freezing a resumed
            //     trajectory from a stale request is the replay hole;
            //   * never echo the ORIGINAL verdict -- an ack must describe the
            //     CURRENT state.  A replay after a resume used to answer
            //     "applied" while the plant was moving: a false success ack that
            //     the executor is entitled to read as "frozen".  Re-evaluate:
            //     applied only if the trajectory is stopped right now.
            // An "applied" ack claims the trajectory is FROZEN, so the replay
            // verdict must test the latch itself.  `!running()` is too weak:
            // COMPLETE satisfies it (transport_task.hpp:196-198) while step()
            // keeps solving and committing, i.e. q_cmd still advances -- an
            // "applied" ack there is a false success on a moving robot.
            const bool still_held =
                transport_active && transport_->state() == TransportState::Hold;
            const std::uint8_t replay_result = replayAckResult(still_held);
            hold_last_result_.store(replay_result, std::memory_order_release);
            hold_ack_.store(packHoldAck(hold_seq, replay_result),
                            std::memory_order_release);
        } else {
            const bool hold_ok = transport_active && transport_->running();
            if (hold_ok) transport_->hold("hold_by_executor", HoldOrigin::Executor);
            const std::uint8_t result = hold_ok ? kAckApplied : kAckRefused;
            hold_last_result_.store(result, std::memory_order_release);
            hold_last_seq_.store(hold_seq, std::memory_order_release);
            hold_ack_.store(packHoldAck(hold_seq, result), std::memory_order_release);
        }
    }
    markTimingPhase(1);  // P2: command dispatch + applyPendingStart() + hold drain
    const double elapsed = (time - activate_time_).seconds();

    // C2.R2: measure the REAL control period.  NOTE: some backends hand the
    // controller a synthetic `period` (gz_ros2_control always passes
    // 1/update_rate), so we differentiate the timestamp instead and only fall
    // back to `period` when that is not possible.  A deadline overrun is a
    // global (both-arm) D7 hold, so scheduler stalls become part of health.
    const double period_arg_ms = period.seconds() * 1e3;
    bool deadline_fault_raw =
        period_monitor_.updateFromTime(time.seconds(), period.seconds());
#ifdef C43_ENABLE_TEST_HOOKS
    // TEST-ONLY: deterministic E4 injection (default 0 => no effect).  Compiled
    // out entirely in production builds (C43_ENABLE_TEST_HOOKS undefined).
    if (test_fault_budget_.load(std::memory_order_relaxed) > 0) {
        test_fault_budget_.fetch_sub(1, std::memory_order_relaxed);
        deadline_fault_raw = true;
    }
#endif
    // Plan §4.7.1 E4: a deadline overrun only holds when it repeats for five
    // consecutive cycles.  The single-sample rule was measured to be wrong: the
    // monitor differentiates the rclcpp clock, which is the SYSTEM clock here,
    // so one NTP step of a few ms trips it while the control loop never missed
    // a deadline (S4-T17: steady_clock max interval 10.27 ms, monitor fault
    // threshold 16.0 ms).  Consecutive-counting is what the approved plan says
    // and it also rejects a one-off clock discontinuity.
    deadline_fault_streak_ = deadline_fault_raw ? deadline_fault_streak_ + 1 : 0;
    const bool deadline_fault = deadline_fault_streak_ >= kDeadlineFaultStreakMax;
    if (deadline_fault_raw) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                             "deadline overrun %d/%d (measured period %.3f ms, threshold %.3f ms)",
                             deadline_fault_streak_, kDeadlineFaultStreakMax,
                             period_monitor_.lastMs(), period_monitor_.faultMs());
    }

    // C2.7c: INDEPENDENT hard safety layer.  This does not rely on the QP or on
    // the CBF row being feasible -- it stops both arms as soon as the measured
    // arm-arm distance drops below the hard threshold.
    if (collision_ != nullptr) {
        const auto info = collision_->distance(q_state_);
        if (info.distance < collision_d_stop_) {
            markTimingPhase(2);  // P3: safety decision (distance + threshold compare)
            hold_active_ = true;
            if (transport_active) transport_->hold("collision");
            writeCommand(q_cmd_);
            markTimingPhase(4);  // P5: writeCommand() on the stop path
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                                  "SAFETY: arm-arm distance %.4f m < %.4f m -> HOLD",
                                  info.distance, collision_d_stop_);
            publishStatus();
            markTimingPhase(5);      // P6
            timing_.setOutcome(2, 11);  // 11 = collision stop
            return return_type::OK;
        }
    }

    // D7 health interlock: any single-arm problem -> hold BOTH arms.
    const StreamHealth health = health_.evaluate(elapsed, q_state_, q_cmd_,
                                                 solve_fail_streak_, deadline_fault);
    if (!health.healthy) {
        markTimingPhase(2);  // P3: safety decision (D7 interlock)
        if (!hold_active_) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "D7 HOLD enter @ t=%.3fs: %s (arm=%d)", elapsed,
                        health.reason.c_str(), health.unhealthy_arm);
        }
        hold_active_ = true;
        if (transport_active) transport_->hold(healthHoldCode(health));
        writeCommand(q_cmd_);  // freeze BOTH arms
        markTimingPhase(4);    // P5
        publishStatus();
        markTimingPhase(5);        // P6
        timing_.setOutcome(2, 12);  // 12 = D7 health interlock
        return return_type::OK;
    }
    if (hold_active_) {
        RCLCPP_INFO(get_node()->get_logger(), "D7 HOLD exit @ t=%.3fs", elapsed);
    }
    hold_active_ = false;

    // ---- C4.3 S4 W1 review (A): E8 executor heartbeat ------------------------
    // Only enforced when explicitly required (default off in S4.1: there is no
    // executor yet).  When off, ~/status reports e8 as MISSING -- the controller
    // must never imply it is watching something it is not.
    if (require_executor_heartbeat_) {
        const double hb_s = last_heartbeat_s_.load(std::memory_order_relaxed);
        const bool seen = heartbeat_seen_.load(std::memory_order_relaxed);
        const bool stale = executorHeartbeatStale(seen, hb_s, time.seconds(),
                                                  executor_heartbeat_timeout_s_);
        if (stale) {
            markTimingPhase(2);  // P3: safety decision (E8 heartbeat)
            hold_active_ = true;
            if (transport_active) transport_->hold("executor_lost");
            writeCommand(q_cmd_);
            markTimingPhase(4);  // P5
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                                  "E8: executor heartbeat %s (age %.3fs > %.3fs) -> HOLD",
                                  seen ? "stale" : "never received", seen ? time.seconds() - hb_s : -1.0,
                                  executor_heartbeat_timeout_s_);
            publishStatus();
            markTimingPhase(5);        // P6
            timing_.setOutcome(2, 13);  // 13 = E8 executor heartbeat
            return return_type::OK;
        }
    }
    // P3 closes here on the normal path: the deadline monitor / D7 / E8 blocks
    // are the whole safety layer.  E1 below is deliberately left in the gap
    // between P3 and P4 (design §2), i.e. it is charged to P4.
    markTimingPhase(2);

    // ---- C4.3 S4 W1: cooperative-transport mode -----------------------------
    if (transport_active) {
        double min_margin = 0.0;
        if (!jointStateInBounds(&min_margin)) {
            // §4.7.1 E1: cheapest independent check, before any math.
            transport_->hold("joint_state_out_of_bounds");
            writeCommand(q_cmd_);
            markTimingPhase(4);  // P5
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                                  "transport HOLD: joint state out of bounds (margin %.3e rad)",
                                  min_margin);
            publishStatus();
            markTimingPhase(5);        // P6
            timing_.setOutcome(2, 14);  // 14 = E1 joint state out of bounds
            return return_type::OK;
        }
        const CycleOutcome out = transport_->step(q_state_, *qp_);
        markTimingPhase(3);  // P4: TransportTask::step()
        last_committed_ = out.committed;
        last_failure_code_ = out.failure_code;
        last_dq_ = out.dq;
        if (out.committed) {
            q_cmd_ += out.dq;
            clampToLimits(&q_cmd_);
        }
        // Never send an increment that did not commit: HOLD/ABORT/FAULT freeze.
        writeCommand(q_cmd_);
        markTimingPhase(4);  // P5: clampToLimits() + writeCommand()
        if (out.failure_code[0] != '\0') {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                                 "transport %s: %s (t_traj=%.3f)",
                                 transportStateName(transport_->state()), out.failure_code,
                                 transport_->tTraj());
        }
        if (++stats_counter_ % 1250 == 0) {
            RCLCPP_INFO(get_node()->get_logger(),
                        "transport: state=%s t=%.3f/%.3fs committed=%d e_obj=%.3e m "
                        "solve=%.3f ms",
                        transportStateName(transport_->state()), transport_->tTraj(),
                        transport_->duration(), static_cast<int>(out.committed),
                        out.e_obj_pos_m, out.solve_time_ms);
        }
        publishStatus();
        markTimingPhase(5);        // P6: publishStatus()
        timing_.setOutcome(1, 0);  // branch 1 = cooperative transport
        return return_type::OK;
    }

    // ---- IDLE: relative-pose hold (bit-identical to the pre-W1 behaviour) ----
    dual_arm::Iso3 target = target_;
    if (test_sweep_amp_m_ > 0.0 && test_sweep_period_s_ > 0.0) {
        // Test-only relative-pose stimulus along the left grasp X axis.
        target.translation().x() +=
            test_sweep_amp_m_ * std::sin(kTwoPi * elapsed / test_sweep_period_s_);
    }

    const dual_arm_qp::DualArmQpResult result = qp_->solve(q_state_, target);
    markTimingPhase(3);  // P4: qp_->solve() on the IDLE branch
    timing_scope.setSolveUs(result.solve_time_ms * 1.0e3);
    last_committed_ = result.converged;
    last_dq_ = result.dq;
    if (!result.converged) {
        ++solve_fail_streak_;
        writeCommand(q_cmd_);  // never send an unconverged increment
        markTimingPhase(4);    // P5
        publishStatus();
        markTimingPhase(5);        // P6
        timing_.setOutcome(2, 22);  // 22 = IDLE solver did not converge
        return return_type::OK;
    }
    solve_fail_streak_ = 0;

    q_cmd_ += result.dq;
    clampToLimits(&q_cmd_);
    writeCommand(q_cmd_);
    markTimingPhase(4);  // P5: clampToLimits() + writeCommand()

    // C2.R2 statistics (low rate: every 10 s at 125 Hz)
    if (++stats_counter_ % 1250 == 0) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "period ms: arg=%.3f measured(mean=%.3f P95=%.3f P99=%.3f max=%.3f, "
                    "measured_flag=%d) | overruns=%lld/%lld | solver P99=%.3f ms",
                    period_arg_ms, period_monitor_.meanMs(), period_monitor_.percentileMs(0.95),
                    period_monitor_.percentileMs(0.99), period_monitor_.maxMs(),
                    static_cast<int>(period_monitor_.lastWasMeasured()), period_monitor_.faults(),
                    period_monitor_.count(), result.solve_time_ms);
    }
    publishStatus();
    markTimingPhase(5);       // P6: publishStatus()
    timing_.setOutcome(0, 0);  // branch 0 = IDLE relative-pose hold
    return return_type::OK;
}

void DualArmQpController::publishStatus() {
    if (status_pub_ == nullptr) return;
    // C7-A Rev.2.1 (P-1): a NEW hold ack must not wait for the 10 Hz decimation.
    // Without this the ack latency inherits "104 ms x (1 + trylock misses)".
    const std::uint64_t ack_now = hold_ack_.load(std::memory_order_acquire);
    const bool ack_changed = (ack_now != status_published_ack_);
    if (!ack_changed && ++status_tick_ < status_period_ticks_) return;
    status_tick_ = 0;
    if (!status_pub_->trylock()) {
        // C4.3 S4 W3-R1a: P6 ran but published nothing.  Counted only while the
        // recorder is enabled, so a production controller is unchanged.
        if (timing_.enabled()) publish_trylock_miss_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (ack_changed) status_published_ack_ = ack_now;

    auto& m = status_pub_->msg_;
    const bool transport_active = transport_ != nullptr && transport_->active();
    m.state = transport_ ? transportStateName(transport_->state()) : "IDLE";
    m.failure_code = (transport_ != nullptr) ? transport_->failureCode() : "";
    m.active = transport_active;
    m.committed = last_committed_;
    m.t_traj = transport_ ? transport_->tTraj() : 0.0;
    m.t_traj_duration = transport_ ? transport_->duration() : 0.0;
    for (int i = 0; i < kNumJoints; ++i) {
        m.q_state[static_cast<size_t>(i)] = q_state_[i];
        m.q_cmd[static_cast<size_t>(i)] = q_cmd_[i];
        m.dq[static_cast<size_t>(i)] = last_dq_[i];
    }
    m.solve_fail_streak = transport_ ? transport_->solveFailStreak() : solve_fail_streak_;

    // Box activity / margin are cheap and exact, so they are measured here
    // rather than taken from the solver (which does not report them).
    int active_limits = 0;
    double min_margin = std::numeric_limits<double>::infinity();
    for (int i = 0; i < kNumJoints; ++i) {
        const double vlim = v_max_[i] * dt_;
        if (vlim > 0.0 && std::abs(last_dq_[i]) >= vlim - 1e-12) ++active_limits;
        min_margin = std::min(min_margin, std::min(q_state_[i] - q_min_[i], q_max_[i] - q_state_[i]));
    }
    m.active_limit_count = active_limits;
    m.min_limit_margin_rad = std::isfinite(min_margin) ? min_margin : 0.0;

    const dual_arm::Iso3 rel_target =
        transport_active ? transport_->leftTrightTarget() : target_;
    const dual_arm::Vec6 e_rel = kinematics_->relativeError(q_state_, rel_target);
    m.e_rel_pos_m = e_rel.head<3>().norm();
    m.e_rel_rot_rad = e_rel.tail<3>().norm();

    m.e_obj_pos_m = 0.0;
    m.e_obj_rot_rad = 0.0;
    m.anchor_pos_gap_m = 0.0;
    m.anchor_rot_gap_rad = 0.0;
    m.sigma_min_stack = 0.0;
    m.qp_solve_time_ms = 0.0;
    if (transport_active) {
        const TransportSample sample = transport_->sample(q_state_);
        m.e_obj_pos_m = sample.e_obj_pos_m;
        m.e_obj_rot_rad = sample.e_obj_rot_rad;
        m.anchor_pos_gap_m = sample.anchor_pos_gap_m;
        m.anchor_rot_gap_rad = sample.anchor_rot_gap_rad;
        // Stacked conditioning (P0 precondition, S4-T18): [J_rel; J_O].
        Eigen::Matrix<double, 12, 12> stack;
        stack.topRows<6>() =
            kinematics_->relativeJacobian(q_state_, dual_arm::RelativeFrame::BodyR);
        stack.bottomRows<6>() = sample.row.J;
        Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(stack);
        m.sigma_min_stack = svd.singularValues()(11);
    }
    // S4 W1 review (A/B): never publish "fine" by omission.
    m.protection = protection_.summary;
    m.protection_complete = protection_.complete;
    m.executor_heartbeat_required = require_executor_heartbeat_;
    m.executor_heartbeat_seen = heartbeat_seen_.load(std::memory_order_relaxed);
    if (m.executor_heartbeat_seen) {
        const double hb_s = last_heartbeat_s_.load(std::memory_order_relaxed);
        const double now_s = get_node()->now().seconds();
        m.executor_heartbeat_age_s = std::isfinite(hb_s) ? std::max(0.0, now_s - hb_s) : -1.0;
    } else {
        m.executor_heartbeat_age_s = -1.0;
    }
    // B: report the live deviation from the standing configuration even while
    // idle, so an operator can see whether ~/start_transport would be accepted.
    double start_dev = 0.0;
    for (int i = 0; i < kNumJoints; ++i) {
        start_dev = std::max(start_dev, std::abs(q_state_[i] - home_q_[i]));
    }
    m.start_pose_deviation_rad = start_dev;
    m.start_pose_ok = !(start_pose_tolerance_rad_ > 0.0) ||
                      start_dev <= start_pose_tolerance_rad_;

    // ---- C7-A Rev.2: hold-ack echo -------------------------------------------
    // Read ONCE from the packed atomic; seq and result can never mismatch.
    m.hold_request_seq = static_cast<std::uint32_t>(ack_now >> 32);
    m.hold_request_result = static_cast<std::uint8_t>(ack_now & 0xFFu);
    m.hold_origin = transport_ != nullptr ? holdOriginName(transport_->holdOrigin()) : "";

    // ---- W3-R2 (approved D5-1): task-phase observation ----------------------
    // Pure read of the frozen spec + the trajectory clock: no allocation, no
    // extra clock read.  phase_index == -1 while no trajectory is latched.
    if (transport_ != nullptr && transport_->active()) {
        const int idx = transport_->phaseIndexAt(transport_->tTraj());
        m.phase_index = idx;
        m.phase_count = transport_->phaseCount();
        m.phase_name = transport_->phaseName(idx);
        m.phase_duration_s = transport_->phaseDuration(idx);
        m.phase_elapsed_s = transport_->tTraj() - transport_->phaseStartTime(idx);
    } else {
        m.phase_index = -1;
        m.phase_count = 0;
        m.phase_name.clear();
        m.phase_elapsed_s = 0.0;
        m.phase_duration_s = 0.0;
    }

    status_pub_->unlockAndPublish();
}

bool DualArmQpController::readState() {
    if (state_interfaces_.size() != static_cast<size_t>(kNumJoints)) return false;
    for (int i = 0; i < kNumJoints; ++i) {
        const double v = state_interfaces_[i].get_value();
        if (!std::isfinite(v)) return false;
        q_state_[i] = v;
    }
    return true;
}

bool DualArmQpController::writeCommand(const Vec12& q_cmd) {
    if (command_interfaces_.size() != static_cast<size_t>(kNumJoints)) return false;
    for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(q_cmd[i]);
    }
    return true;
}

void DualArmQpController::clampToLimits(Vec12* q) const {
    for (int i = 0; i < kNumJoints; ++i) {
        (*q)[i] = std::min(q_max_[i], std::max(q_min_[i], (*q)[i]));
    }
}

}  // namespace dual_arm_qp_controller

PLUGINLIB_EXPORT_CLASS(dual_arm_qp_controller::DualArmQpController,
                       controller_interface::ControllerInterface)

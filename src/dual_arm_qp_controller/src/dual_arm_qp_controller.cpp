// src/dual_arm_qp_controller.cpp

#include "dual_arm_qp_controller/dual_arm_qp_controller.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dual_arm_qp_controller {

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

namespace {
constexpr int kNumJoints = 12;
constexpr double kTwoPi = 6.283185307179586;
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
    declareIfNeeded<double>("dt", 0.008);
    declareIfNeeded<double>("kp_rel", 10.0);
    declareIfNeeded<double>("w_rel", 1.0);
    declareIfNeeded<double>("w_slack", 1e3);
    declareIfNeeded<double>("w_center", 1e-2);
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
    w_reg_ = get_node()->get_parameter("w_reg").as_double();
    v_max_default_ = get_node()->get_parameter("v_max").as_double();
    test_sweep_amp_m_ = get_node()->get_parameter("test_sweep_amp_m").as_double();
    test_sweep_period_s_ = get_node()->get_parameter("test_sweep_period_s").as_double();
    period_fault_factor_ = get_node()->get_parameter("period_fault_factor").as_double();

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

    dual_arm_qp::DualArmQpParams qp_params;
    qp_params.dt = dt_;
    qp_params.kp_rel = kp_rel_;
    qp_params.w_rel = w_rel_;
    qp_params.w_slack = w_slack_;
    qp_params.w_center = w_center_;
    qp_params.w_reg = w_reg_;
    qp_params.q_min = q_min_;
    qp_params.q_max = q_max_;
    qp_params.v_max = v_max_;

    kinematics_ = std::make_unique<dual_arm::DualArmKinematics>();
    qp_ = std::make_unique<dual_arm_qp::DualArmQp>(*kinematics_, qp_params);
    qp_->setWarmStart(true);

    RCLCPP_INFO(get_node()->get_logger(),
                "configured: 12-DOF QP, dt=%.4fs, kp_rel=%.2f, solver=%s", dt_, kp_rel_,
                qp_->solverName().c_str());
    return CallbackReturn::SUCCESS;
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
    hold_active_ = false;
    health_.reset();
    period_monitor_.reset();
    stats_counter_ = 0;
    activated_ = true;
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
    return CallbackReturn::SUCCESS;
}

return_type DualArmQpController::update(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) {
    if (!activated_) return return_type::OK;
    if (!readState()) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "QP controller: non-finite joint state");
        return return_type::ERROR;
    }

    const double elapsed = (time - activate_time_).seconds();

    // C2.R2: measure the REAL control period; a deadline overrun is a global
    // (both-arm) D7 hold, so scheduler stalls become part of health monitoring.
    const bool deadline_fault = period_monitor_.update(period.seconds());

    // D7 health interlock: any single-arm problem -> hold BOTH arms.
    const StreamHealth health = health_.evaluate(elapsed, q_state_, q_cmd_,
                                                 solve_fail_streak_, deadline_fault);
    if (!health.healthy) {
        if (!hold_active_) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "D7 HOLD enter @ t=%.3fs: %s (arm=%d)", elapsed,
                        health.reason.c_str(), health.unhealthy_arm);
        }
        hold_active_ = true;
        writeCommand(q_cmd_);  // freeze BOTH arms
        return return_type::OK;
    }
    if (hold_active_) {
        RCLCPP_INFO(get_node()->get_logger(), "D7 HOLD exit @ t=%.3fs", elapsed);
    }
    hold_active_ = false;

    dual_arm::Iso3 target = target_;
    if (test_sweep_amp_m_ > 0.0 && test_sweep_period_s_ > 0.0) {
        // Test-only relative-pose stimulus along the left grasp X axis.
        target.translation().x() +=
            test_sweep_amp_m_ * std::sin(kTwoPi * elapsed / test_sweep_period_s_);
    }

    const dual_arm_qp::DualArmQpResult result = qp_->solve(q_state_, target);
    if (!result.converged) {
        ++solve_fail_streak_;
        writeCommand(q_cmd_);  // never send an unconverged increment
        return return_type::OK;
    }
    solve_fail_streak_ = 0;

    q_cmd_ += result.dq;
    clampToLimits(&q_cmd_);
    writeCommand(q_cmd_);

    // C2.R2 statistics (low rate: every 10 s at 125 Hz)
    if (++stats_counter_ % 1250 == 0) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "period ms: mean=%.3f P95=%.3f P99=%.3f max=%.3f | overruns=%lld/%lld "
                    "| solver P99=%.3f ms",
                    period_monitor_.meanMs(), period_monitor_.percentileMs(0.95),
                    period_monitor_.percentileMs(0.99), period_monitor_.maxMs(),
                    period_monitor_.faults(), period_monitor_.count(), result.solve_time_ms);
    }
    return return_type::OK;
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

// include/dual_arm_qp_controller/dual_arm_qp_controller.hpp
// C2.2 layer 3: ros2_control ControllerInterface wrapping dual_arm_qp_core.
//
// Interface contract (decision D2):
//   - math: velocity-level QP (dq), execution: position command
//   - claims 12 position command interfaces (arms only) and 12 position
//     state interfaces, in the joint order given by the `joints` parameter
//   - does NOT claim the finger joints (gripper handled elsewhere)
//
// The QP target is the relative pose ^L T_R captured at activation (C2.3
// "relative pose hold").  A test-only sinusoidal sweep can be enabled to make
// the D7 interlock observable in simulation.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "dual_arm_collision/dual_arm_collision.hpp"
#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"
#include "dual_arm_qp_controller/period_monitor.hpp"
#include "dual_arm_qp_controller/stream_health_monitor.hpp"

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
    bool readState();
    bool writeCommand(const Vec12& q_cmd);
    void clampToLimits(Vec12* q) const;

    std::vector<std::string> joint_names_;
    std::vector<std::string> command_iface_names_;
    std::vector<std::string> state_iface_names_;

    std::unique_ptr<dual_arm::DualArmKinematics> kinematics_;
    std::unique_ptr<dual_arm_collision::DualArmCollisionModel> collision_;
    std::unique_ptr<dual_arm_qp::DualArmQp> qp_;

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

    // parameters
    double dt_ = 0.008;
    double kp_rel_ = 10.0;
    double w_rel_ = 1.0;
    double w_slack_ = 1e3;
    double w_center_ = 1e-2;
    double w_reg_ = 1e-6;
    double v_max_default_ = 2.5;
    double test_sweep_amp_m_ = 0.0;
    double test_sweep_period_s_ = 4.0;
    double period_fault_factor_ = 2.0;
    // C2.7c: independent safety layer (NOT the QP/CBF path)
    double collision_d_stop_ = 0.0;      // 0 = disabled
    double collision_link_radius_ = 0.06;

    template <typename T>
    void declareIfNeeded(const std::string& name, const T& default_value) {
        if (!get_node()->has_parameter(name)) {
            get_node()->declare_parameter(name, default_value);
        }
    }
};

}  // namespace dual_arm_qp_controller

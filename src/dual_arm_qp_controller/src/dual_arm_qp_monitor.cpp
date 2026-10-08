// src/dual_arm_qp_monitor.cpp
// C2.9: independent monitor for the dual-arm QP profile.
//
// Subscribes to /joint_states, records the relative pose ^L T_R of the FIRST
// sample as the target (same semantics as the controller's activation), then
// reports the relative-pose error statistics (RMS / max, position and rotation)
// every `report_period_s`.  This is how the Gazebo relative-pose accuracy is
// quantified without trusting the controller's own bookkeeping.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"

namespace {

class QpMonitor : public rclcpp::Node {
public:
    QpMonitor() : Node("dual_arm_qp_monitor") {
        joints_ = declare_parameter<std::vector<std::string>>(
            "joints", {"left_j1", "left_j2", "left_j3", "left_j4", "left_j5", "left_j6",
                       "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"});
        report_period_s_ = declare_parameter<double>("report_period_s", 5.0);
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 50,
            std::bind(&QpMonitor::onJointState, this, std::placeholders::_1));
        last_report_ = now();
        RCLCPP_INFO(get_logger(), "monitor up: will capture ^L T_R from the first /joint_states");
    }

private:
    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
        dual_arm::JointConfig ql, qr;
        bool ok = true;
        for (int i = 0; i < 12; ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joints_[i]);
            if (it == msg->name.end()) return;
            const size_t k = static_cast<size_t>(std::distance(msg->name.begin(), it));
            const double v = msg->position[k];
            if (!std::isfinite(v)) return;
            (i < 6 ? ql : qr)[i % 6] = v;
        }
        const dual_arm::Q12 q = dual_arm::DualArmKinematics::stack(ql, qr);

        if (!have_target_) {
            target_ = kin_.leftTright(q);
            have_target_ = true;
            RCLCPP_INFO(get_logger(), "target captured from first sample");
            return;
        }
        const dual_arm::Vec6 e = kin_.relativeError(q, target_);
        const double pos_mm = e.head<3>().norm() * 1e3;
        const double rot_deg = e.tail<3>().norm() * 180.0 / M_PI;
        sum_pos2_ += pos_mm * pos_mm;
        sum_rot2_ += rot_deg * rot_deg;
        max_pos_ = std::max(max_pos_, pos_mm);
        max_rot_ = std::max(max_rot_, rot_deg);
        ++n_;

        const double t = (now() - last_report_).seconds();
        if (t >= report_period_s_ && n_ > 0) {
            report();
            last_report_ = now();
        }
    }

    void report() {
        const double rms_pos = std::sqrt(sum_pos2_ / n_);
        const double rms_rot = std::sqrt(sum_rot2_ / n_);
        RCLCPP_INFO(get_logger(),
                    "[relative pose] n=%d RMS %.4f mm / %.5f deg | max %.4f mm / %.5f deg",
                    n_, rms_pos, rms_rot, max_pos_, max_rot_);
    }

    dual_arm::DualArmKinematics kin_;
    std::vector<std::string> joints_;
    double report_period_s_ = 5.0;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Time last_report_;
    dual_arm::Iso3 target_ = dual_arm::Iso3::Identity();
    bool have_target_ = false;
    double sum_pos2_ = 0.0, sum_rot2_ = 0.0, max_pos_ = 0.0, max_rot_ = 0.0;
    int n_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QpMonitor>());
    rclcpp::shutdown();
    return 0;
}

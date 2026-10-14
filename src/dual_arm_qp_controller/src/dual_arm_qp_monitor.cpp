// src/dual_arm_qp_monitor.cpp
// C2.9: independent monitor for the dual-arm QP profile.
//
// Subscribes to /joint_states, records the relative pose ^L T_R of the FIRST
// sample as the target (same semantics as the controller's activation), then
// reports the relative-pose error statistics (RMS / max, position and rotation)
// every `report_period_s`.  This is how the Gazebo relative-pose accuracy is
// quantified without trusting the controller's own bookkeeping.
//
// IMPORTANT when the test sweep is enabled: the sweep MOVES the target
// (target.x += amp*sin(2*pi*t/T)), so the "error vs the first sample" reported
// here is the COMMANDED MOTION plus any tracking error -- it is NOT the tracking
// error on its own.  Set `csv_path` to log the absolute ^L T_R time series and
// separate the two offline (fit the commanded sinusoid, the residual is the
// tracking error).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/se3.hpp"

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
        csv_path_ = declare_parameter<std::string>("csv_path", "");
        if (!csv_path_.empty()) {
            csv_.open(csv_path_);
            // Two time bases on purpose: `t_sim_s` comes from the message header
            // (Gazebo sim time) and is what the controller's `elapsed` uses;
            // `t_wall_s` is the node clock and is always available.  Logging both
            // avoids the trap hit in the first C2.9b run, where use_sim_time did
            // not take effect and every sample was stamped 0.
            csv_ << "t_sim_s,t_wall_s,px,py,pz,rx,ry,rz,q0..q11\n";
            csv_ << std::setprecision(12);   // default 6 sig figs would collapse the timestamps
            RCLCPP_INFO(get_logger(), "logging ^L T_R time series to %s", csv_path_.c_str());
        }
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
        if (csv_.is_open()) {
            // Raw int64 nanoseconds on purpose: rclcpp::Time refuses to subtract
            // across clock sources (header stamp is RCL_ROS_TIME, the node clock
            // without sim time is RCL_SYSTEM_TIME), and "1 != 2" is a runtime
            // throw, not a warning.
            const int64_t sim_ns =
                static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
                static_cast<int64_t>(msg->header.stamp.nanosec);
            if (sim_ns != 0 && !have_sim0_) { sim0_ns_ = sim_ns; have_sim0_ = true; }
            const double t_sim =
                (sim_ns != 0 && have_sim0_) ? (sim_ns - sim0_ns_) * 1e-9 : 0.0;
            const int64_t wall_ns = now().nanoseconds();
            if (!have_t0_) { t0_ns_ = wall_ns; have_t0_ = true; }
            const double t_wall = (wall_ns - t0_ns_) * 1e-9;

            const dual_arm::Iso3 rel = kin_.leftTright(q);
            const Eigen::Vector3d t = rel.translation();
            const Eigen::Vector3d r = dual_arm::logSO3(rel.rotation());
            csv_ << t_sim << "," << t_wall
                 << "," << t.x() << "," << t.y() << "," << t.z()
                 << "," << r.x() << "," << r.y() << "," << r.z();
            for (int i = 0; i < 12; ++i) csv_ << "," << q[i];
            csv_ << "\n";
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
            if (csv_.is_open()) csv_.flush();
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
    std::string csv_path_;
    std::ofstream csv_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Time last_report_;
    dual_arm::Iso3 target_ = dual_arm::Iso3::Identity();
    bool have_target_ = false;
    int64_t t0_ns_ = 0;
    int64_t sim0_ns_ = 0;
    bool have_t0_ = false;
    bool have_sim0_ = false;
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

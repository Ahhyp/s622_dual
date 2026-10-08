// include/dual_arm_qp_core/dual_arm_qp.hpp
// Task-level QP for dual-arm coordinated control (C2.2 skeleton).
//
// Decision variables: z = [dq (12); s (6)]
//   dq : joint increment over one control period
//   s  : slack of the relative-pose task (SE(3) twist, right-grasp frame)
//
// Cost:
//   w_rel   * || J_rel dq + alpha * e - s ||^2      (relative pose task)
//   w_slack * || s ||^2                             (slack penalty)
//   w_center* || dq - dq_center ||^2                (secondary: joint centering)
//   w_reg   * || dq ||^2                            (regularization)
//   (alpha = kp_rel * dt)
//
// Hard constraints (l <= A z <= u):
//   dq_lower <= dq <= dq_upper      with
//       dq_lower = max(-v_max*dt, q_min - q)
//       dq_upper = min( v_max*dt, q_max - q)
//   plus optional user inequality rows  C z <= d  (e.g. CBF).

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/qp_types.hpp"

namespace dual_arm_qp {

using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::Q12;

using Vec12 = Eigen::Matrix<double, 12, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;

struct DualArmQpParams {
    double dt = 0.008;          // 125 Hz
    double kp_rel = 10.0;       // relative-pose feedback gain [1/s]
    double w_rel = 1.0;
    double w_slack = 1e3;
    double w_center = 1e-2;
    double w_reg = 1e-6;

    Vec12 q_min;
    Vec12 q_max;
    Vec12 v_max;

    /// Preferred joint increment for the secondary objective (usually 0, or a
    /// nullspace direction from a higher-level planner).
    Vec12 dq_center = Vec12::Zero();

    /// Extra linear inequality rows  C z <= d  (columns = [dq; s]).
    /// Rows are appended after the box constraints.
    Eigen::MatrixXd C_ineq;   // (k x 18)
    Eigen::VectorXd d_ineq;   // (k)

    DualArmQpParams();
};

struct DualArmQpResult {
    Vec12 dq = Vec12::Zero();
    Vec6 slack = Vec6::Zero();
    bool converged = false;
    int iterations = 0;
    double solve_time_ms = 0.0;
    double relative_error_norm = 0.0;  // ||e|| at the linearization point
    std::string solver_name;
};

class DualArmQp {
public:
    DualArmQp(const DualArmKinematics& kinematics, const DualArmQpParams& params,
              QpSolverPtr solver = nullptr);

    /// One QP step: returns the joint increment that drives the relative pose
    /// towards `left_T_right_target` while respecting the hard constraints.
    DualArmQpResult solve(const Q12& q, const Iso3& left_T_right_target);

    const DualArmQpParams& params() const { return params_; }
    std::string solverName() const { return solver_->name(); }
    void setParams(const DualArmQpParams& params) { params_ = params; }

    /// Last assembled problem (for inspection / debugging / tests).
    const QpProblem& lastProblem() const { return last_problem_; }

    /// Warm start across successive control cycles (default on).
    void setWarmStart(bool enabled) { warm_start_enabled_ = enabled; }
    bool warmStartEnabled() const { return warm_start_enabled_; }

private:
    const DualArmKinematics& kin_;
    DualArmQpParams params_;
    QpSolverPtr solver_;
    QpProblem last_problem_;

    // ---- pre-allocated scratch (C2.R4: no heap allocation in the control path) ----
    // Sizes are constant except when the number of general inequality rows (k)
    // changes; the dynamic ones only resize when k changes.
    Eigen::Matrix<double, 6, 18> A_task_;
    Eigen::Matrix<double, 18, 18> P_;
    Eigen::Matrix<double, 18, 1> q_cost_;
    Eigen::Matrix<double, 12, 1> lower_;
    Eigen::Matrix<double, 12, 1> upper_;
    Eigen::MatrixXd A_scratch_;
    Eigen::VectorXd l_scratch_;
    Eigen::VectorXd u_scratch_;

    Eigen::VectorXd warm_start_;
    bool warm_start_enabled_ = true;
};

/// Default S622 joint limits / velocity limit (rad, rad/s).
void defaultS622Limits(Vec12* q_min, Vec12* q_max, Vec12* v_max);

}  // namespace dual_arm_qp

// include/dual_arm_qp_core/dual_arm_qp.hpp
// Task-level QP for dual-arm coordinated control (C2.2 skeleton).
//
// Decision variables (C2.4b): z = [dq (12); s_rel (6); s_obj (6)]
//   dq    : joint increment over one control period
//   s_rel : slack of the relative-pose task (twist, right-grasp frame)
//   s_obj : slack of the object/coordination-frame task (twist, object frame)
//
// Cost:
//   w_rel    * || J_rel dq + alpha_rel e_rel - s_rel ||^2   (relative task)
//   w_obj    * || J_obj dq + alpha_obj e_obj - s_obj ||^2   (object task)
//   w_slack  * || s_rel ||^2
//   w_slack_obj * || s_obj ||^2
//   w_center * || dq - dq_center ||^2
//   w_reg    * || dq ||^2
//   (alpha_* = kp_* * dt; the object task is disabled when w_obj == 0)
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

    // ---- object / coordination-frame task (C2.4b) ----
    double kp_obj = 10.0;       // object feedback gain [1/s]
    double w_obj = 0.0;         // 0 = object task disabled
    double w_slack_obj = 1e-6;  // keeps P PD when w_obj == 0

    // ---- nullspace secondary objectives (C2.5) ----------------------------
    // dq_center is built from a weighted sum of improving directions and
    // clamped to at most null_step (rad per step) in norm.
    double w_null_margin = 0.0;  // joint-limit margin
    double w_null_manip = 0.0;   // sigma_min(J_rel)
    double w_null_clear = 0.0;   // arm-arm clearance proxy (elbow/wrist)
    double null_step = 0.005;    // max |dq_center| per step [rad]

    Vec12 q_min;
    Vec12 q_max;
    Vec12 v_max;

    /// Preferred joint increment for the secondary objective (usually 0, or a
    /// nullspace direction from a higher-level planner).
    Vec12 dq_center = Vec12::Zero();

    /// Extra linear inequality rows  C z <= d  (columns = [dq; s]).
    /// Rows are appended after the box constraints.
    Eigen::MatrixXd C_ineq;   // (k x 24)
    Eigen::VectorXd d_ineq;   // (k)

    DualArmQpParams();
};

struct DualArmQpResult {
    Vec12 dq = Vec12::Zero();
    Vec6 slack = Vec6::Zero();        // relative-task slack
    Vec6 slack_object = Vec6::Zero(); // object-task slack
    bool converged = false;
    int iterations = 0;
    double solve_time_ms = 0.0;
    double relative_error_norm = 0.0;  // ||e_rel|| at the linearization point
    double object_error_norm = 0.0;    // ||e_obj|| at the linearization point
    std::string solver_name;
};

class DualArmQp {
public:
    DualArmQp(const DualArmKinematics& kinematics, const DualArmQpParams& params,
              QpSolverPtr solver = nullptr);

    /// One QP step (relative task only): the object target defaults to the
    /// current object pose.
    DualArmQpResult solve(const Q12& q, const Iso3& left_T_right_target);

    /// One QP step with both tasks (C2.4b).
    DualArmQpResult solve(const Q12& q, const Iso3& left_T_right_target,
                          const Iso3& world_T_object_target);

    /// Same, with a feed-forward object body twist (object frame, [v; w]).
    /// Feed-forward is what removes the v/kp lag of pure proportional tracking.
    DualArmQpResult solve(const Q12& q, const Iso3& left_T_right_target,
                          const Iso3& world_T_object_target,
                          const Vec6& object_body_twist_ff);

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
    Eigen::Matrix<double, 6, 24> A_task_rel_;
    Eigen::Matrix<double, 6, 24> A_task_obj_;
    Eigen::Matrix<double, 24, 24> P_;
    Eigen::Matrix<double, 24, 1> q_cost_;
    Eigen::Matrix<double, 12, 1> lower_;
    Eigen::Matrix<double, 12, 1> upper_;
    Eigen::MatrixXd A_scratch_;
    Eigen::VectorXd l_scratch_;
    Eigen::VectorXd u_scratch_;

    Vec12 dq_center_scratch_ = Vec12::Zero();
    Eigen::VectorXd warm_start_;
    bool warm_start_enabled_ = true;
};

/// Default S622 joint limits / velocity limit (rad, rad/s).
void defaultS622Limits(Vec12* q_min, Vec12* q_max, Vec12* v_max);

}  // namespace dual_arm_qp

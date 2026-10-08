// src/dual_arm_qp.cpp

#include "dual_arm_qp_core/dual_arm_qp.hpp"

#include <algorithm>
#include <limits>

namespace dual_arm_qp {

void defaultS622Limits(Vec12* q_min, Vec12* q_max, Vec12* v_max) {
    const double lo[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
    const double hi[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};
    const double vv[6] = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5};
    for (int a = 0; a < 2; ++a) {
        for (int i = 0; i < 6; ++i) {
            (*q_min)[a * 6 + i] = lo[i];
            (*q_max)[a * 6 + i] = hi[i];
            (*v_max)[a * 6 + i] = vv[i];
        }
    }
}

DualArmQpParams::DualArmQpParams() { defaultS622Limits(&q_min, &q_max, &v_max); }

DualArmQp::DualArmQp(const DualArmKinematics& kinematics, const DualArmQpParams& params,
                     QpSolverPtr solver)
    : kin_(kinematics), params_(params), solver_(solver ? std::move(solver) : makeDefaultSolver()) {}

DualArmQpResult DualArmQp::solve(const Q12& q, const Iso3& left_T_right_target) {
    DualArmQpResult result;
    result.solver_name = solver_->name();

    const Vec6 e = kin_.relativeError(q, left_T_right_target);
    result.relative_error_norm = e.norm();

    const dual_arm::Mat6x12 J =
        kin_.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);

    constexpr int kNumVars = 18;  // [dq(12); slack(6)]
    (void)kNumVars;
    const double dt = params_.dt;
    const double alpha = params_.kp_rel * dt;

    // ---- cost (pre-allocated scratch, no heap allocation) ------------------
    A_task_.setZero();
    A_task_.leftCols<12>() = J;
    A_task_.rightCols<6>() = -Eigen::Matrix<double, 6, 6>::Identity();

    // Task residual r = J_rel dq - alpha*e - s, with alpha = kp_rel*dt.
    // (edot ~= -J_rel dq, so J_rel dq = +alpha*e drives e -> 0.)
    const Vec6 task_bias = -alpha * e;
    P_.noalias() = params_.w_rel * (A_task_.transpose() * A_task_);
    q_cost_.noalias() = params_.w_rel * (A_task_.transpose() * task_bias);

    P_.topLeftCorner<12, 12>() +=
        (params_.w_center + params_.w_reg) * Eigen::Matrix<double, 12, 12>::Identity();
    P_.bottomRightCorner<6, 6>() += params_.w_slack * Eigen::Matrix<double, 6, 6>::Identity();
    q_cost_.head<12>() -= params_.w_center * params_.dq_center;

    // ---- box constraints on dq -------------------------------------------
    for (int i = 0; i < 12; ++i) {
        double lo = std::max(-params_.v_max[i] * dt, params_.q_min[i] - q[i]);
        double hi = std::min(params_.v_max[i] * dt, params_.q_max[i] - q[i]);
        if (lo > hi) {
            // Joint is far outside its range: move back as fast as allowed.
            const double v = (q[i] > params_.q_max[i]) ? -params_.v_max[i] * dt
                                                       : params_.v_max[i] * dt;
            lo = hi = v;
        }
        lower_[i] = lo;
        upper_[i] = hi;
    }

    const int k = static_cast<int>(params_.C_ineq.rows());
    const int m = 12 + k;
    if (A_scratch_.rows() != m || A_scratch_.cols() != kNumVars) {
        A_scratch_.resize(m, kNumVars);
        l_scratch_.resize(m);
        u_scratch_.resize(m);
    }
    A_scratch_.topRows(12).setZero();
    A_scratch_.topRows(12).leftCols<12>() = Eigen::Matrix<double, 12, 12>::Identity();
    l_scratch_.head(12) = lower_;
    u_scratch_.head(12) = upper_;
    if (k > 0) {
        A_scratch_.bottomRows(k) = params_.C_ineq;
        l_scratch_.tail(k).setConstant(-std::numeric_limits<double>::infinity());
        u_scratch_.tail(k) = params_.d_ineq;
    }

    last_problem_.P = P_;
    last_problem_.q = q_cost_;
    last_problem_.A = A_scratch_;
    last_problem_.l = l_scratch_;
    last_problem_.u = u_scratch_;
    if (warm_start_enabled_ && warm_start_.size() == kNumVars) {
        last_problem_.x0 = warm_start_;
    }

    const QpSolution solution = solver_->solve(last_problem_);
    result.converged = solution.converged;
    result.iterations = solution.iterations;
    result.solve_time_ms = solution.solve_time_ms;
    if (solution.x.size() == kNumVars) {
        result.dq = solution.x.head<12>();
        result.slack = solution.x.tail<6>();
    } else if (solution.x.size() == 12) {
        result.dq = solution.x;
    }

    // Safety clamp: the solver tolerances are ~1e-6, but the joint box is a
    // hard constraint, so project dq onto it before it ever reaches hardware.
    // NOTE (C2.R3): this element-wise projection is exact ONLY for the box.
    // It is skipped when general inequality rows (e.g. CBF) are present,
    // because it does not preserve C*dq <= d in general.
    if (k == 0) {
        for (int i = 0; i < 12; ++i) {
            result.dq[i] = std::min(upper_[i], std::max(lower_[i], result.dq[i]));
        }
    }

    if (warm_start_enabled_) {
        if (warm_start_.size() != kNumVars) warm_start_.setZero(kNumVars);
        warm_start_.head<12>() = result.dq;
        warm_start_.tail<6>() = result.slack;
    }
    return result;
}

}  // namespace dual_arm_qp

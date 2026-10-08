// src/dual_arm_qp.cpp
// C2.4b: two main tasks (relative pose + object/coordination frame), each with
// its own slack.  Decision vector is always 24 = [dq(12); s_rel(6); s_obj(6)],
// so the QP structure never changes at runtime (OSQP workspace stays warm);
// the object task is switched off by w_obj == 0.

#include "dual_arm_qp_core/dual_arm_qp.hpp"

#include <algorithm>
#include <limits>

namespace dual_arm_qp {

namespace {

/// sigma_min of the relative Jacobian (dual-arm task capability metric).
double sigmaMinRel(const DualArmKinematics& kin, const Q12& q) {
    const dual_arm::Mat6x12 J = kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);
    Eigen::JacobiSVD<dual_arm::Mat6x12> svd(J);
    return svd.singularValues().minCoeff();
}

/// Arm-arm clearance proxy: min distance between key link origins of the two
/// arms (shoulder / elbow / wrist / flange).  A comfort objective, NOT safety.
double clearanceProxy(const DualArmKinematics& kin, const Q12& q) {
    dual_arm::JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    const int idx[4] = {2, 3, 4, 5};
    double d_min = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 4; ++i) {
        const Eigen::Vector3d pl = kin.worldTframe(dual_arm::Arm::Left, ql, idx[i]).translation();
        for (int j = 0; j < 4; ++j) {
            const Eigen::Vector3d pr =
                kin.worldTframe(dual_arm::Arm::Right, qr, idx[j]).translation();
            d_min = std::min(d_min, (pl - pr).norm());
        }
    }
    return d_min;
}

/// Improving (ascent) direction of a scalar objective, by central differences.
template <typename Fn>
Vec12 ascentDirection(const Q12& q, double h, Fn f) {
    Vec12 g;
    for (int i = 0; i < 12; ++i) {
        Q12 qp = q, qm = q;
        qp[i] += h;
        qm[i] -= h;
        g[i] = (f(qp) - f(qm)) / (2.0 * h);
    }
    return g;
}

/// Joint-limit margin: push joints back towards the middle of their range.
Vec12 marginDirection(const DualArmQpParams& p, const Q12& q) {
    Vec12 g;
    for (int i = 0; i < 12; ++i) {
        const double range = std::max(1e-6, p.q_max[i] - p.q_min[i]);
        const double mid = 0.5 * (p.q_max[i] + p.q_min[i]);
        g[i] = -2.0 * (q[i] - mid) / (range * range);  // descent of sum(r_i^2)
    }
    return g;
}

}  // namespace

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
    return solve(q, left_T_right_target, kin_.worldTobject(q));
}

DualArmQpResult DualArmQp::solve(const Q12& q, const Iso3& left_T_right_target,
                                 const Iso3& world_T_object_target) {
    return solve(q, left_T_right_target, world_T_object_target, Vec6::Zero());
}

DualArmQpResult DualArmQp::solve(const Q12& q, const Iso3& left_T_right_target,
                                 const Iso3& world_T_object_target,
                                 const Vec6& object_body_twist_ff) {
    DualArmQpResult result;
    result.solver_name = solver_->name();

    constexpr int kNumVars = 24;
    constexpr int kSRel0 = 12;
    constexpr int kSObj0 = 18;
    const Eigen::Matrix<double, 12, 12> I12 = Eigen::Matrix<double, 12, 12>::Identity();
    const Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();

    const Vec6 e_rel = kin_.relativeError(q, left_T_right_target);
    result.relative_error_norm = e_rel.norm();
    const dual_arm::Mat6x12 J_rel =
        kin_.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);

    const Vec6 e_obj = kin_.objectError(q, world_T_object_target);
    result.object_error_norm = e_obj.norm();

    const double dt = params_.dt;
    const double alpha_rel = params_.kp_rel * dt;
    const double alpha_obj = params_.kp_obj * dt;
    const bool object_on = params_.w_obj > 0.0;

    // ---- relative task row block: [J_rel, -I6, 0] -------------------------
    A_task_rel_.setZero();
    A_task_rel_.block<6, 12>(0, 0) = J_rel;
    A_task_rel_.block<6, 6>(0, kSRel0) = -I6;

    // ---- object task row block: [J_obj, 0, -I6] ---------------------------
    const dual_arm::Mat6x12 J_obj = kin_.objectJacobianBody(q);
    A_task_obj_.setZero();
    if (object_on) {
        A_task_obj_.block<6, 12>(0, 0) = J_obj;
        A_task_obj_.block<6, 6>(0, kSObj0) = -I6;
    }

    // residual = A z - alpha*e  (so that edot ~= -alpha*e drives e -> 0)
    const Vec6 bias_rel = -alpha_rel * e_rel;
    // bias = -(alpha*e + dt*v_ff): the proportional term removes error, the
    // feed-forward term removes the velocity-dependent lag (C2.4c).
    const Vec6 bias_obj = -(alpha_obj * e_obj + dt * object_body_twist_ff);

    P_ = params_.w_rel * (A_task_rel_.transpose() * A_task_rel_);
    q_cost_ = params_.w_rel * (A_task_rel_.transpose() * bias_rel);
    if (object_on) {
        P_ += params_.w_obj * (A_task_obj_.transpose() * A_task_obj_);
        q_cost_ += params_.w_obj * (A_task_obj_.transpose() * bias_obj);
    }
    // ---- nullspace secondary objectives (C2.5) ----------------------------
    // Each direction is normalised before weighting; the sum is capped at
    // null_step so the secondary objective can never dominate the main tasks.
    const bool nullspace_on = params_.w_null_margin > 0.0 || params_.w_null_manip > 0.0 ||
                              params_.w_null_clear > 0.0;
    if (nullspace_on) {
        Vec12 dir = Vec12::Zero();
        auto add = [&](const Vec12& d, double w) {
            const double n = d.norm();
            if (n > 1e-12) dir += w * (d / n);
        };
        if (params_.w_null_margin > 0.0) add(marginDirection(params_, q), params_.w_null_margin);
        if (params_.w_null_manip > 0.0) {
            add(ascentDirection(q, 1e-3, [&](const Q12& qq) { return sigmaMinRel(kin_, qq); }),
                params_.w_null_manip);
        }
        if (params_.w_null_clear > 0.0) {
            add(ascentDirection(q, 1e-3, [&](const Q12& qq) { return clearanceProxy(kin_, qq); }),
                params_.w_null_clear);
        }
        // Project the preference into the NULLSPACE of the active main tasks
        // (damped pseudo-inverse):  N = I - J_task^+ J_task.  This is what makes
        // it a true redundancy allocation instead of a competing soft cost.
        // With both tasks active J_task is 12x12 -> N ~= 0 (no redundancy).
        const int m_task = object_on ? 12 : 6;
        Eigen::MatrixXd J_task(m_task, 12);
        J_task.topRows<6>() = J_rel;
        if (object_on) J_task.bottomRows<6>() = J_obj;
        const double lambda2 = 1e-8;
        const Eigen::MatrixXd JJt =
            J_task * J_task.transpose() +
            lambda2 * Eigen::MatrixXd::Identity(m_task, m_task);
        const Eigen::MatrixXd N =
            Eigen::Matrix<double, 12, 12>::Identity() - J_task.transpose() * JJt.inverse() * J_task;
        Vec12 dir_null = N * dir;
        const double n = dir_null.norm();
        if (n > 1e-12) {
            dq_center_scratch_ = (params_.null_step / std::max(1.0, n)) * dir_null;
        } else {
            dq_center_scratch_.setZero();
        }
    } else {
        dq_center_scratch_ = params_.dq_center;
    }

    P_.block<12, 12>(0, 0) += (params_.w_center + params_.w_reg) * I12;
    P_.block<6, 6>(kSRel0, kSRel0) += params_.w_slack * I6;
    P_.block<6, 6>(kSObj0, kSObj0) += params_.w_slack_obj * I6;
    q_cost_.head<12>() -= params_.w_center * dq_center_scratch_;

    // ---- box constraints on dq -------------------------------------------
    for (int i = 0; i < 12; ++i) {
        double lo = std::max(-params_.v_max[i] * dt, params_.q_min[i] - q[i]);
        double hi = std::min(params_.v_max[i] * dt, params_.q_max[i] - q[i]);
        if (lo > hi) {
            const double v = (q[i] > params_.q_max[i]) ? -params_.v_max[i] * dt
                                                       : params_.v_max[i] * dt;
            lo = hi = v;
        }
        lower_[i] = lo;
        upper_[i] = hi;
    }

    int k = 0;
    if (params_.C_ineq.rows() > 0 && params_.C_ineq.cols() == kNumVars) {
        k = static_cast<int>(params_.C_ineq.rows());
    }
    const int m = 12 + k;
    if (A_scratch_.rows() != m || A_scratch_.cols() != kNumVars) {
        A_scratch_.resize(m, kNumVars);
        l_scratch_.resize(m);
        u_scratch_.resize(m);
    }
    A_scratch_.topRows(12).setZero();
    A_scratch_.topRows(12).leftCols<12>() = I12;
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
        result.dq = solution.x.segment<12>(0);
        result.slack = solution.x.segment<6>(kSRel0);
        result.slack_object = solution.x.segment<6>(kSObj0);
    } else if (solution.x.size() == 12) {
        result.dq = solution.x;
    }

    // Safety clamp: exact for the box only.  Skipped when general inequality
    // rows are present (C2.R3), because it can break C*dq <= d.
    if (k == 0) {
        for (int i = 0; i < 12; ++i) {
            result.dq[i] = std::min(upper_[i], std::max(lower_[i], result.dq[i]));
        }
    }

    if (warm_start_enabled_) {
        if (warm_start_.size() != kNumVars) warm_start_.setZero(kNumVars);
        warm_start_.head<12>() = result.dq;
        warm_start_.segment<6>(kSRel0) = result.slack;
        warm_start_.segment<6>(kSObj0) = result.slack_object;
    }
    return result;
}

}  // namespace dual_arm_qp

// test/test_nullspace_c25.cpp
// C2.5 ablation: nullspace secondary objectives
//   A: OFF   B: +joint margin   C: +manipulability   D: +clearance   E: ALL
// Metrics: min joint margin, min sigma_min(J_rel), min arm clearance,
//          main-task RMS (must not degrade), solve time.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;

const double kLower[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kUpper[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

JointConfig randomQ(std::mt19937& rng, double shrink) {
    JointConfig q;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 6; ++i) {
        const double mid = 0.5 * (kLower[i] + kUpper[i]);
        const double half = 0.5 * (kUpper[i] - kLower[i]) * shrink;
        q[i] = mid + (2.0 * unit(rng) - 1.0) * half;
    }
    return q;
}

double sigmaMinRel(const DualArmKinematics& kin, const Q12& q) {
    const dual_arm::Mat6x12 J = kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);
    Eigen::JacobiSVD<dual_arm::Mat6x12> svd(J);
    return svd.singularValues().minCoeff();
}

double clearanceProxy(const DualArmKinematics& kin, const Q12& q) {
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    const int idx[4] = {2, 3, 4, 5};
    double d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 4; ++i) {
        const Eigen::Vector3d pl = kin.worldTframe(Arm::Left, ql, idx[i]).translation();
        for (int j = 0; j < 4; ++j) {
            const Eigen::Vector3d pr = kin.worldTframe(Arm::Right, qr, idx[j]).translation();
            d = std::min(d, (pl - pr).norm());
        }
    }
    return d;
}

Q12 findWellConditionedQ0(const DualArmKinematics& kin, std::mt19937& rng) {
    Q12 best = DualArmKinematics::stack(JointConfig::Zero(), JointConfig::Zero());
    double best_cond = 1e18;
    for (int t = 0; t < 2000; ++t) {
        const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.5), randomQ(rng, 0.5));
        JointConfig ql, qr;
        DualArmKinematics::split(q, &ql, &qr);
        const Eigen::Vector3d xi = dual_arm::logSO3(
            kin.worldTgrasp(Arm::Left, ql).linear().transpose() *
            kin.worldTgrasp(Arm::Right, qr).linear());
        if (xi.norm() > 1.0) continue;
        Eigen::Matrix<double, 12, 12> J;
        J.topRows<6>() = kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);
        J.bottomRows<6>() = kin.objectJacobianBody(q);
        Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(J);
        const double cond = svd.singularValues().maxCoeff() / svd.singularValues().minCoeff();
        if (cond < best_cond) {
            best_cond = cond;
            best = q;
        }
    }
    return best;
}

struct Ablation {
    std::string name;
    double min_margin = 1e9;
    double min_sigma = 1e9;
    double min_clear = 1e9;
    double obj_rms_mm = 0, rel_rms_mm = 0;
    double mean_solve_ms = 0;
    int failures = 0;
};

Ablation run(const DualArmKinematics& kin, const DualArmQpParams& params, const Iso3& rel_target,
             Q12 q0, const std::function<Iso3(double)>& objTarget, const std::string& name,
             const dual_arm::Q12& dist_bias = dual_arm::Q12::Zero(), double T = 4.0,
             double tau = 0.05) {
    DualArmQp qp(kin, params);
    const double dt = params.dt;
    const int steps = static_cast<int>(T / dt);
    Q12 q = q0, q_cmd = q0;
    Ablation a;
    a.name = name;
    double so = 0, sr = 0, tsum = 0;
    int n = 0;
    for (int k = 0; k < steps; ++k) {
        const double t = k * dt;
        const Iso3 target = objTarget(t);
        const Iso3 tm = objTarget(t - 0.5 * dt), tp = objTarget(t + 0.5 * dt);
        const dual_arm::Vec6 v_ff = dual_arm::logSE3(tm.inverse() * tp) / dt;
        const auto r = qp.solve(q, rel_target, target, v_ff);
        if (!r.converged) ++a.failures; else q_cmd += r.dq;
        q += (dt / tau) * (q_cmd - q);
        q += dist_bias * dt;  // external joint-space disturbance
        tsum += r.solve_time_ms;

        for (int i = 0; i < 12; ++i) {
            a.min_margin = std::min(a.min_margin,
                                    std::min(q[i] - params.q_min[i], params.q_max[i] - q[i]));
        }
        a.min_sigma = std::min(a.min_sigma, sigmaMinRel(kin, q));
        a.min_clear = std::min(a.min_clear, clearanceProxy(kin, q));
        if (t >= 0.5) {
            so += std::pow(kin.objectError(q, target).head<3>().norm() * 1e3, 2);
            sr += std::pow(kin.relativeError(q, rel_target).head<3>().norm() * 1e3, 2);
            ++n;
        }
    }
    if (n) {
        a.obj_rms_mm = std::sqrt(so / n);
        a.rel_rms_mm = std::sqrt(sr / n);
    }
    a.mean_solve_ms = tsum / steps;
    return a;
}

}  // namespace

TEST(NullspaceC25, AblationOFFvsON) {
    DualArmKinematics kin;
    std::mt19937 rng(20261016u);
    Q12 q0 = findWellConditionedQ0(kin, rng);
    // C2.5 scenario: ONLY the relative task is active, so the 12-DOF system has
    // a genuine 6-dimensional task nullspace (the dual-arm-specific redundancy).
    // A constant joint-space disturbance pushes both j2 joints towards their
    // upper limit; the secondary objectives must shape the nullspace motion.
    const DualArmQpParams lim;
    q0[1] = lim.q_max[1] - 0.45;
    q0[7] = lim.q_max[7] - 0.45;
    dual_arm::Q12 dist = dual_arm::Q12::Zero();
    dist[1] = 1.2;    // rad/s, pushing j2 towards the upper limit
    dist[7] = 1.2;

    const Iso3 rel_target = kin.leftTright(q0);
    const Iso3 obj0 = kin.worldTobject(q0);

    const double w = 2.0 * M_PI / 4.0;
    const auto traj = [&](double t) {
        Iso3 T_ = obj0;
        T_.translation() += Eigen::Vector3d(0.04 * std::sin(w * t), 0.03 * std::sin(2 * w * t),
                                            0.02 * std::sin(w * t));
        T_.linear() = obj0.linear() * dual_arm::expSO3(Eigen::Vector3d(0.15 * std::sin(w * t),
                                                                       0.1 * std::sin(w * t),
                                                                       0.0));
        return T_;
    };

    auto base = [&]() {
        DualArmQpParams p;
        p.dt = 0.008;
        p.kp_rel = 20.0;
        p.kp_obj = 20.0;
        p.w_rel = 1.0;
        p.w_obj = 0.0;   // relative-only -> 6-dim nullspace
        p.w_slack = 1e3;
        p.w_slack_obj = 1e3;
        p.w_center = 0.5;      // enough authority for the secondary preference
        p.w_reg = 1e-6;
        p.null_step = 0.006;   // max |dq_center| per step [rad]
        return p;
    };

    const auto traj_const = [&](double) { return obj0; };
    DualArmQpParams pA = base();
    DualArmQpParams pB = base(); pB.w_null_margin = 1.0;
    DualArmQpParams pC = base(); pC.w_null_manip = 1.0;
    DualArmQpParams pD = base(); pD.w_null_clear = 1.0;
    DualArmQpParams pE = base(); pE.w_null_margin = 1.0; pE.w_null_manip = 1.0;
    pE.w_null_clear = 1.0;

    const std::vector<Ablation> res = {
        run(kin, pA, rel_target, q0, traj_const, "A OFF", dist),
        run(kin, pB, rel_target, q0, traj_const, "B +margin", dist),
        run(kin, pC, rel_target, q0, traj_const, "C +manip", dist),
        run(kin, pD, rel_target, q0, traj_const, "D +clear", dist),
        run(kin, pE, rel_target, q0, traj_const, "E ALL", dist),
    };

    std::printf("[C2.5] %-12s | minMargin(rad) minSigma  minClear(m) | objRMS(mm) relRMS(mm) | ms/solve | fail\n",
                "config");
    for (const auto& a : res) {
        std::printf("[C2.5] %-12s | %13.4f %8.4f %11.4f | %10.3f %10.3f | %8.3f | %d\n",
                    a.name.c_str(), a.min_margin, a.min_sigma, a.min_clear, a.obj_rms_mm,
                    a.rel_rms_mm, a.mean_solve_ms, a.failures);
    }

    for (const auto& a : res) EXPECT_EQ(a.failures, 0);
    // main tasks must not be degraded by the secondary objectives
    for (const auto& a : res) {
        EXPECT_LT(a.rel_rms_mm, 8.0);   // the only main task must not degrade
    }
    // at least ONE secondary objective must show a measurable improvement
    const Ablation& A = res[0];
    const Ablation& E = res[4];
    EXPECT_GT(E.min_margin, A.min_margin);
    EXPECT_GT(E.min_sigma, A.min_sigma);
    EXPECT_GT(E.min_clear, A.min_clear);
}

// test/test_dual_task_c24.cpp
// C2.4b: object/coordination-frame trajectory tracking WHILE holding the
// relative pose, closed loop with a first-order actuator.
//
//   min w_rel || J_rel dq - a_rel e_rel - s_r ||^2
//     + w_obj || J_obj dq - a_obj e_obj - s_o ||^2
//     + w_reg || dq ||^2
//
// Four trajectory types, reporting RMS and peak for BOTH tasks.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

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

/// Pick a well-conditioned dual-arm configuration for the combined
/// (relative + object) task, away from the |xi| ~ pi branch of logSO3.
Q12 findWellConditionedQ0(const DualArmKinematics& kin, std::mt19937& rng) {
    Q12 best = DualArmKinematics::stack(
        JointConfig::Zero(), JointConfig::Zero());
    double best_cond = 1e18, best_xi = 0.0;
    for (int trial = 0; trial < 2000; ++trial) {
        const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.5), randomQ(rng, 0.5));
        JointConfig ql, qr;
        DualArmKinematics::split(q, &ql, &qr);
        const Eigen::Vector3d xi = dual_arm::logSO3(
            kin.worldTgrasp(dual_arm::Arm::Left, ql).linear().transpose() *
            kin.worldTgrasp(dual_arm::Arm::Right, qr).linear());
        if (xi.norm() > 1.0) continue;
        Eigen::Matrix<double, 12, 12> J;
        J.topRows<6>() = kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);
        J.bottomRows<6>() = kin.objectJacobianBody(q);
        Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(J);
        const double cond = svd.singularValues().maxCoeff() / svd.singularValues().minCoeff();
        if (cond < best_cond) {
            best_cond = cond;
            best_xi = xi.norm();
            best = q;
        }
    }
    std::printf("[C2.4b] chosen q0: cond(J_stack)=%.1f, |xi|=%.3f rad\n", best_cond, best_xi);
    return best;
}

struct Metrics {
    double obj_pos_rms_mm = 0, obj_pos_peak_mm = 0;
    double obj_rot_rms_deg = 0, obj_rot_peak_deg = 0;
    double rel_pos_rms_mm = 0, rel_pos_peak_mm = 0;
    double rel_rot_rms_deg = 0, rel_rot_peak_deg = 0;
    int failures = 0;
    int saturated_steps = 0;
    double max_dq = 0.0;
};

Metrics runTrajectory(const DualArmKinematics& kin, const DualArmQpParams& params,
                      const Iso3& rel_target, Q12 q0,
                      const std::function<Iso3(double)>& objTarget, double T, double tau) {
    DualArmQp qp(kin, params);
    const double dt = params.dt;
    const int steps = static_cast<int>(T / dt);
    Q12 q = q0, q_cmd = q0;

    Metrics m;
    double so_p = 0, so_r = 0, sr_p = 0, sr_r = 0;
    int n = 0;
    for (int k = 0; k < steps; ++k) {
        const double t = k * dt;
        const Iso3 target = objTarget(t);
        // feed-forward: central-difference body twist of the target trajectory
        const Iso3 tm = objTarget(t - 0.5 * dt);
        const Iso3 tp = objTarget(t + 0.5 * dt);
        const dual_arm::Vec6 v_ff = dual_arm::logSE3(tm.inverse() * tp) / dt;
        const auto r = qp.solve(q, rel_target, target, v_ff);
        if (r.converged) {
            q_cmd += r.dq;
        } else {
            ++m.failures;
        }
        const double dqm = r.dq.cwiseAbs().maxCoeff();
        m.max_dq = std::max(m.max_dq, dqm);
        if (dqm > 0.99 * params.v_max[0] * dt) ++m.saturated_steps;
        q += (dt / tau) * (q_cmd - q);

        if (t < 0.5) continue;  // skip the initial transient
        const dual_arm::Vec6 eo = kin.objectError(q, target);
        const dual_arm::Vec6 er = kin.relativeError(q, rel_target);
        const double op = eo.head<3>().norm() * 1e3, orr = eo.tail<3>().norm() * 180.0 / M_PI;
        const double rp = er.head<3>().norm() * 1e3, rr = er.tail<3>().norm() * 180.0 / M_PI;
        so_p += op * op; so_r += orr * orr;
        sr_p += rp * rp; sr_r += rr * rr;
        m.obj_pos_peak_mm = std::max(m.obj_pos_peak_mm, op);
        m.obj_rot_peak_deg = std::max(m.obj_rot_peak_deg, orr);
        m.rel_pos_peak_mm = std::max(m.rel_pos_peak_mm, rp);
        m.rel_rot_peak_deg = std::max(m.rel_rot_peak_deg, rr);
        ++n;
    }
    if (n > 0) {
        m.obj_pos_rms_mm = std::sqrt(so_p / n);
        m.obj_rot_rms_deg = std::sqrt(so_r / n);
        m.rel_pos_rms_mm = std::sqrt(sr_p / n);
        m.rel_rot_rms_deg = std::sqrt(sr_r / n);
    }
    return m;
}

DualArmQpParams makeParams() {
    DualArmQpParams p;
    p.dt = 0.008;
    p.kp_rel = 20.0;
    p.kp_obj = 20.0;
    p.w_rel = 1.0;
    p.w_obj = 1.0;
    p.w_slack = 1e3;
    p.w_slack_obj = 1e3;
    p.w_center = 1e-2;
    p.w_reg = 1e-6;
    return p;
}

void report(const char* name, const Metrics& m) {
    std::printf(
        "[C2.4b] %-22s | obj: RMS %6.3f mm / %6.3f deg  peak %7.3f mm / %6.3f deg"
        " | rel: RMS %6.3f mm / %6.3f deg  peak %7.3f mm / %6.3f deg | fail %d\n",
        name, m.obj_pos_rms_mm, m.obj_rot_rms_deg, m.obj_pos_peak_mm, m.obj_rot_peak_deg,
        m.rel_pos_rms_mm, m.rel_rot_rms_deg, m.rel_pos_peak_mm, m.rel_rot_peak_deg, m.failures);
    std::printf("        |dq|max=%.4f (limit %.4f)  saturated steps=%d\n", m.max_dq,
                0.02, m.saturated_steps);
}

}  // namespace

TEST(C24DualTask, FourTrajectoriesWithRelativeHold) {
    DualArmKinematics kin;
    const DualArmQpParams params = makeParams();

    // The mock home pose has |xi| ~ pi (relative rotation of ~180 deg), which
    // sits on the logSO3 branch and is numerically hostile.  Pick a
    // well-conditioned configuration instead (singularity handling = C2.6).
    std::mt19937 rng0(20261015u);
    const Q12 q0 = findWellConditionedQ0(kin, rng0);
    const Iso3 rel_target = kin.leftTright(q0);
    const Iso3 obj0 = kin.worldTobject(q0);

    const double T = 4.0;      // 4 s per trajectory
    const double tau = 0.05;   // 50 ms actuator
    const double w = 2.0 * M_PI / 4.0;

    // 1) pure translation (3-axis sinusoid)
    const auto traj_trans = [&](double t) {
        Iso3 T_ = obj0;
        T_.translation() += Eigen::Vector3d(0.05 * std::sin(w * t),
                                            0.03 * std::sin(2.0 * w * t),
                                            0.02 * std::sin(0.5 * w * t));
        return T_;
    };
    // 2) single-axis rotation about the object z
    const auto traj_rot = [&](double t) {
        Iso3 T_ = obj0;
        T_.linear() = obj0.linear() * dual_arm::expSO3(Eigen::Vector3d(0, 0, 0.4 * std::sin(w * t)));
        return T_;
    };
    // 3) translation + rotation
    const auto traj_mix = [&](double t) {
        Iso3 T_ = obj0;
        T_.translation() += Eigen::Vector3d(0.04 * std::sin(w * t), 0.0, 0.02 * std::sin(2 * w * t));
        T_.linear() = obj0.linear() * dual_arm::expSO3(
                                          Eigen::Vector3d(0.2 * std::sin(w * t),
                                                          0.15 * std::sin(1.5 * w * t), 0.0));
        return T_;
    };
    // 4) 3-D curve (Lissajous + combined rotation).  Amplitudes are chosen so
    // the required joint rate stays inside v_max; a faster curve saturates the
    // hard velocity box (that is a physical limit, not a controller fault).
    const auto traj_curve = [&](double t) {
        Iso3 T_ = obj0;
        T_.translation() += Eigen::Vector3d(0.025 * std::sin(w * t), 0.025 * std::sin(2 * w * t),
                                            0.02 * std::sin(3 * w * t));
        T_.linear() = obj0.linear() * dual_arm::expSO3(
                                          Eigen::Vector3d(0.10 * std::sin(1.5 * w * t),
                                                          0.08 * std::sin(w * t),
                                                          0.12 * std::sin(2 * w * t)));
        return T_;
    };

    const Metrics m1 = runTrajectory(kin, params, rel_target, q0, traj_trans, T, tau);
    const Metrics m2 = runTrajectory(kin, params, rel_target, q0, traj_rot, T, tau);
    const Metrics m3 = runTrajectory(kin, params, rel_target, q0, traj_mix, T, tau);
    const Metrics m4 = runTrajectory(kin, params, rel_target, q0, traj_curve, T, tau);

    report("translation", m1);
    report("single-axis rotation", m2);
    report("translation+rotation", m3);
    report("3D curve", m4);

    for (const Metrics* m : {&m1, &m2, &m3, &m4}) {
        EXPECT_EQ(m->failures, 0);
        EXPECT_EQ(m->saturated_steps, 0) << "trajectory exceeds the velocity budget";
        EXPECT_LT(m->obj_pos_rms_mm, 15.0);   // object tracking (mm)
        EXPECT_LT(m->obj_rot_rms_deg, 1.0);   // object tracking (deg)
        EXPECT_LT(m->rel_pos_rms_mm, 6.0);    // relative hold (mm)
        EXPECT_LT(m->rel_rot_rms_deg, 0.5);   // relative hold (deg)
    }
}

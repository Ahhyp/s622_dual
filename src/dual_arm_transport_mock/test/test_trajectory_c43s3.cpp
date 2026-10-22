// C4.3 S3 -- W4 tests: frozen trajectory + body-twist feed-forward (S3-T4).
//
// S3-T4 asserts three things that make the feed-forward trustworthy without tuning:
//   * time calibration           duration == sum(phases), steps == ceil(duration/dt)
//   * boundary continuity        v_ff == 0 at every phase boundary
//   * analytic == numeric        v_ff matches a central difference of T_d (<= 1e-6),
//                                and its integral reproduces the displacement (<= 1e-6)
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>

#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_object_task/trajectory.hpp"

namespace {

using dual_arm::Iso3;
using dual_arm::Vec6;
using dual_arm_object_task::FrozenObjectTrajectory;
using dual_arm_object_task::TrajectoryError;
using dual_arm_object_task::TrajectoryPhase;
using dual_arm_object_task::TrajectorySpec;
using dual_arm_object_task::loadTrajectorySpec;

TrajectoryPhase phase(const char* name, double dur, double dx, double dy, double dz) {
    TrajectoryPhase p;
    p.name = name;
    p.duration = dur;
    p.dx = dx;
    p.dy = dy;
    p.dz = dz;
    return p;
}

/// The shipped spec, rebuilt in code so the numeric assertions never depend on
/// YAML parsing (that is covered separately).
TrajectorySpec defaultSpec() {
    TrajectorySpec s;
    s.dt = 0.008;
    s.time_scaling = "smoothstep";
    s.start_from_baseline = true;
    s.start_world_T_object = Iso3::Identity();
    s.start_world_T_object.translation() = Eigen::Vector3d(0.0, 0.0, 0.275);
    s.phases = {phase("settle", 0.30, 0.0, 0.0, 0.0),
                phase("lift", 1.50, 0.0, 0.0, 0.05),
                phase("translate", 1.50, -0.06, 0.0, 0.0),
                phase("lower", 1.50, 0.0, 0.0, -0.05),
                phase("settle2", 0.30, 0.0, 0.0, 0.0)};
    s.yaw_sweep_deg = {0.0};
    return s;
}

bool isFarFromBoundaries(const FrozenObjectTrajectory& tr, double t, double h) {
    for (double tb : tr.boundaryTimes()) {
        if (std::abs(t - tb) < 3.0 * h) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// time calibration + shipped config
// ---------------------------------------------------------------------------
TEST(TrajectoryC43S3, T4a_TimeCalibrationMatchesThePhases) {
    const FrozenObjectTrajectory tr(defaultSpec());
    EXPECT_NEAR(tr.duration(), 5.1, 1e-12);
    // ceil(5.1 / 0.008) = 638
    EXPECT_EQ(tr.steps(), 638u);
    ASSERT_EQ(tr.boundaryTimes().size(), 6u);
    EXPECT_DOUBLE_EQ(tr.boundaryTimes().front(), 0.0);
    EXPECT_NEAR(tr.boundaryTimes().back(), 5.1, 1e-12);
    for (std::size_t i = 1; i < tr.boundaryTimes().size(); ++i) {
        EXPECT_GT(tr.boundaryTimes()[i], tr.boundaryTimes()[i - 1]);
    }

    const Iso3 start = tr.at(0.0, nullptr);
    EXPECT_LT((start.translation() - Eigen::Vector3d(0.0, 0.0, 0.275)).norm(), 1e-15);

    const Iso3 end = tr.at(tr.duration(), nullptr);
    EXPECT_LT((end.translation() - Eigen::Vector3d(-0.06, 0.0, 0.275)).norm(), 1e-15)
        << "net displacement must be exactly the sum of the phase deltas";
}

TEST(TrajectoryC43S3, T4b_ShippedYamlLoadsAndIsValid) {
    const std::string path =
        std::string(C43_CONFIG_DIR) + "/s3_transport_trajectory.yaml";
    const TrajectorySpec spec = loadTrajectorySpec(path);
    EXPECT_DOUBLE_EQ(spec.dt, 0.008);
    EXPECT_EQ(spec.time_scaling, "smoothstep");
    EXPECT_TRUE(spec.start_from_baseline);
    ASSERT_EQ(spec.phases.size(), 5u);
    EXPECT_EQ(spec.phases[1].name, "lift");
    EXPECT_DOUBLE_EQ(spec.phases[2].dx, -0.06);
    const FrozenObjectTrajectory tr(spec);
    EXPECT_NEAR(tr.duration(), 5.1, 1e-12);
    EXPECT_EQ(tr.steps(), 638u);
}

TEST(TrajectoryC43S3, T4c_YawNodeCountIsValidated) {
    TrajectorySpec bad = defaultSpec();
    bad.yaw_sweep_deg = {0.0, 8.0};  // neither 1 nor phases.size()+1
    EXPECT_THROW(FrozenObjectTrajectory{bad}, TrajectoryError);

    TrajectorySpec good = defaultSpec();
    good.yaw_sweep_deg = {0.0, 0.0, 8.0, 8.0, 0.0, 0.0};
    EXPECT_NO_THROW(FrozenObjectTrajectory{good});
}

// ---------------------------------------------------------------------------
// boundary continuity + analytic/numeric agreement
// ---------------------------------------------------------------------------
TEST(TrajectoryC43S3, T4d_FeedforwardVanishesAtEveryPhaseBoundary) {
    const FrozenObjectTrajectory tr(defaultSpec());
    for (double tb : tr.boundaryTimes()) {
        Vec6 v;
        (void)tr.at(tb, &v);
        EXPECT_LT(v.cwiseAbs().maxCoeff(), 1e-15)
            << "v_ff must be exactly 0 at boundary t=" << tb << ", got " << v.transpose();
        // ... and it must arrive there continuously from both sides.
        Vec6 v_lo, v_hi;
        (void)tr.at(tb - 1e-9, &v_lo);
        (void)tr.at(tb + 1e-9, &v_hi);
        EXPECT_LT(v_lo.cwiseAbs().maxCoeff(), 1e-8);
        EXPECT_LT(v_hi.cwiseAbs().maxCoeff(), 1e-8);
        EXPECT_LT((v_lo - v_hi).cwiseAbs().maxCoeff(), 1e-8);
    }
}

TEST(TrajectoryC43S3, T4e_FeedforwardMatchesCentralDifference) {
    const FrozenObjectTrajectory tr(defaultSpec());
    const double h = 8e-4;  // dt/10

    double worst_lin = 0.0;
    double worst_ang = 0.0;
    int checked = 0;

    const int n = 500;
    for (int i = 1; i < n; ++i) {
        const double t = tr.duration() * static_cast<double>(i) / n;
        if (!isFarFromBoundaries(tr, t, h)) continue;

        Vec6 v_an;
        const Iso3 T = tr.at(t, &v_an);

        const Iso3 T_lo = tr.at(t - h, nullptr);
        const Iso3 T_hi = tr.at(t + h, nullptr);

        const Eigen::Vector3d p_dot = (T_hi.translation() - T_lo.translation()) / (2.0 * h);
        const Eigen::Vector3d v_cd = T.linear().transpose() * p_dot;
        const Eigen::Vector3d w_cd =
            dual_arm::logSO3(T_lo.linear().transpose() * T_hi.linear()) / (2.0 * h);

        worst_lin = std::max(worst_lin, (v_an.head<3>() - v_cd).cwiseAbs().maxCoeff());
        worst_ang = std::max(worst_ang, (v_an.tail<3>() - w_cd).cwiseAbs().maxCoeff());
        ++checked;
    }

    EXPECT_GT(checked, 400) << "too few samples away from the boundaries";
    EXPECT_LE(worst_lin, 1e-6) << "linear feed-forward off by " << worst_lin << " m/s";
    EXPECT_LE(worst_ang, 1e-6) << "angular feed-forward off by " << worst_ang << " rad/s";
    std::printf("[T4e] d/dt vs analytic: lin worst=%.3e, ang worst=%.3e, samples=%d\n",
                worst_lin, worst_ang, checked);
}

TEST(TrajectoryC43S3, T4f_IntegratedFeedforwardEqualsTheDisplacement) {
    const FrozenObjectTrajectory tr(defaultSpec());
    const int n = 20000;
    const double h = tr.duration() / n;
    Eigen::Vector3d integral = Eigen::Vector3d::Zero();
    Vec6 v_lo, v_hi;
    (void)tr.at(0.0, &v_lo);
    for (int i = 1; i <= n; ++i) {
        const double t = tr.duration() * static_cast<double>(i) / n;
        (void)tr.at(t, &v_hi);
        integral += 0.5 * (v_lo.head<3>() + v_hi.head<3>()) * h;  // trapezoid
        v_lo = v_hi;
    }
    const Iso3 end = tr.at(tr.duration(), nullptr);
    const Eigen::Vector3d disp = end.translation() - tr.at(0.0, nullptr).translation();
    const double err = (integral - disp).norm();
    EXPECT_LE(err, 1e-6) << "integral of v_ff = " << integral.transpose()
                         << " vs displacement " << disp.transpose();
    std::printf("[T4f] int v_ff dt = [%.9f %.9f %.9f], disp = [%.9f %.9f %.9f], err=%.3e\n",
                integral.x(), integral.y(), integral.z(), disp.x(), disp.y(), disp.z(), err);
}

// ---------------------------------------------------------------------------
// the rotational channel (yaw sweep) must be equally exact
// ---------------------------------------------------------------------------
TEST(TrajectoryC43S3, T4g_YawSweepDrivesTheAngularChannelExactly) {
    TrajectorySpec spec = defaultSpec();
    spec.yaw_sweep_deg = {0.0, 0.0, 8.0, 8.0, 0.0, 0.0};
    const FrozenObjectTrajectory tr(spec);
    const double h = 8e-4;

    double worst_ang = 0.0;
    double max_yaw_rate = 0.0;
    int checked = 0;
    for (int i = 1; i < 500; ++i) {
        const double t = tr.duration() * static_cast<double>(i) / 500.0;
        if (!isFarFromBoundaries(tr, t, h)) continue;
        Vec6 v_an;
        (void)tr.at(t, &v_an);
        const Iso3 T_lo = tr.at(t - h, nullptr);
        const Iso3 T_hi = tr.at(t + h, nullptr);
        const Eigen::Vector3d w_cd =
            dual_arm::logSO3(T_lo.linear().transpose() * T_hi.linear()) / (2.0 * h);
        worst_ang = std::max(worst_ang, (v_an.tail<3>() - w_cd).cwiseAbs().maxCoeff());
        // yaw about world z => body angular velocity is parallel to +z.
        EXPECT_LT(std::abs(v_an[3]), 1e-12);
        EXPECT_LT(std::abs(v_an[4]), 1e-12);
        max_yaw_rate = std::max(max_yaw_rate, std::abs(v_an[5]));
        ++checked;
    }
    EXPECT_GT(checked, 400);
    EXPECT_GT(max_yaw_rate, 1e-3) << "the yaw sweep did not actually rotate anything";
    EXPECT_LE(worst_ang, 1e-6) << "angular feed-forward off by " << worst_ang << " rad/s";
}

}  // namespace

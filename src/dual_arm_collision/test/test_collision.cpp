// test/test_collision.cpp  (C2.7a)

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

#include "dual_arm_collision/dual_arm_collision.hpp"

namespace {

using dual_arm::DualArmKinematics;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm_collision::DualArmCollisionModel;
using dual_arm_collision::Vec12;

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

}  // namespace

TEST(DualArmCollision, DistanceFiniteAndNearestPointsConsistent) {
    DualArmKinematics kin;
    DualArmCollisionModel model(kin, 0.06);
    std::mt19937 rng(7u);
    double max_err = 0.0;
    int n = 0;
    int penetrating = 0;
    for (int t = 0; t < 300; ++t) {
        const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.6), randomQ(rng, 0.6));
        const auto info = model.distance(q);
        ASSERT_TRUE(info.valid);
        // FCL returns a NEGATIVE distance for penetration; that is meaningful
        // (it is the penetration depth), so only require finiteness here.
        EXPECT_TRUE(std::isfinite(info.distance));
        if (info.distance < -1e-9) ++penetrating;
        if (info.distance > 1e-6) {
            // |p2 - p1| must equal the reported distance
            max_err = std::max(max_err, std::abs((info.p2 - info.p1).norm() - info.distance));
            ++n;
        }
    }
    std::printf("[C2.7a] nearest-point consistency: max err = %.3e (n=%d, penetrating=%d/300)\n",
                max_err, n, penetrating);
    EXPECT_LT(max_err, 1e-9);
}

TEST(DualArmCollision, GradientMatchesFullFiniteDifference) {
    DualArmKinematics kin;
    DualArmCollisionModel model(kin, 0.06);
    std::mt19937 rng(11u);
    double max_diff = 0.0;
    int checked = 0;
    for (int t = 0; t < 60; ++t) {
        const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.5), randomQ(rng, 0.5));
        const double d0 = model.distance(q).distance;
        if (d0 < 0.10) continue;  // skip near-contact cases (min pair may switch)
        const double h = 1e-4;
        Vec12 g_full;
        for (int k = 0; k < 12; ++k) {
            Q12 qp = q, qm = q;
            qp[k] += h;
            qm[k] -= h;
            g_full[k] = (model.distance(qp).distance - model.distance(qm).distance) / (2 * h);
        }
        const Vec12 g_model = model.distanceGradient(q, h);
        max_diff = std::max(max_diff, (g_full - g_model).cwiseAbs().maxCoeff());
        ++checked;
    }
    std::printf("[C2.7a] gradient vs full FD: max diff = %.3e (checked %d configs)\n", max_diff,
                checked);
    EXPECT_LT(max_diff, 1e-4);
}

TEST(DualArmCollision, GradientAscentIncreasesDistance) {
    DualArmKinematics kin;
    DualArmCollisionModel model(kin, 0.06);
    std::mt19937 rng(13u);
    int improved = 0, tested = 0;
    for (int t = 0; t < 100; ++t) {
        const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.5), randomQ(rng, 0.5));
        const double d0 = model.distance(q).distance;
        if (d0 < 0.08) continue;
        const Vec12 g = model.distanceGradient(q);
        if (g.norm() < 1e-6) continue;
        const Q12 q1 = q + (1e-3 / g.norm()) * g;   // small step along +gradient
        const double d1 = model.distance(q1).distance;
        if (d1 > d0) ++improved;
        ++tested;
    }
    std::printf("[C2.7a] ascent step increased distance in %d/%d cases\n", improved, tested);
    EXPECT_GT(tested, 50);
    EXPECT_GT(improved, static_cast<int>(0.9 * tested));
}

TEST(DualArmCollision, ClosestConfigurationIsDistinguishable) {
    DualArmKinematics kin;
    DualArmCollisionModel model(kin, 0.06);

    JointConfig stretched;
    stretched << 0.0, -1.5708, 0.0, 0.0, 0.0, 0.0;
    const Q12 q_stretched = DualArmKinematics::stack(stretched, stretched);
    const double d_stretched = model.distance(q_stretched).distance;

    // search for a close configuration
    std::mt19937 rng(23u);
    double d_min = std::numeric_limits<double>::infinity();
    Q12 q_min = q_stretched;
    for (int t = 0; t < 500; ++t) {
        const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.7), randomQ(rng, 0.7));
        const double d = model.distance(q).distance;
        if (d < d_min) {
            d_min = d;
            q_min = q;
        }
    }
    (void)q_min;
    std::printf("[C2.7a] distance: stretched=%.4f m, closest found=%.4f m\n", d_stretched, d_min);
    EXPECT_LT(d_min, d_stretched);
    EXPECT_TRUE(std::isfinite(d_min));
}

TEST(DualArmCollision, GradientTiming) {
    DualArmKinematics kin;
    DualArmCollisionModel model(kin, 0.06);
    std::mt19937 rng(17u);
    const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.5), randomQ(rng, 0.5));
    constexpr int kN = 2000;
    auto t0 = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (int k = 0; k < kN; ++k) sink += model.distanceGradient(q).norm();
    auto t1 = std::chrono::steady_clock::now();
    const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / kN;
    std::printf("[C2.7a] distanceGradient = %.1f us/call (sink=%.3f)\n", us, sink);
    EXPECT_LT(us, 2000.0);
}

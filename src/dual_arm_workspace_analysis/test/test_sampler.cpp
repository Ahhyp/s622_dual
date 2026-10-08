// test/test_sampler.cpp
// C4.1 unit tests: Halton coverage, SVD metrics, filtering thresholds, and the
// two consistency checks that guard against a wrong relative-Jacobian
// convention (the C2.4a failure mode):
//   * rank(J_rel) is frame-invariant
//   * rank(J_rel) matches the rank of the finite-difference error Jacobian

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <random>
#include <vector>

#include "dual_arm_workspace_analysis/halton.hpp"
#include "dual_arm_workspace_analysis/workspace_sampler.hpp"

namespace {

using dual_arm::DualArmKinematics;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm::RelativeFrame;
using dual_arm_workspace::Halton;
using dual_arm_workspace::SampleRecord;
using dual_arm_workspace::SamplerParams;
using dual_arm_workspace::SvdMetrics;
using dual_arm_workspace::WorkspaceSampler;
using dual_arm_workspace::encodeRecord;
using dual_arm_workspace::kRecordFloats;
using dual_arm_workspace::svdMetrics;

Q12 randomQ12(std::mt19937& rng) {
    SamplerParams p;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    JointConfig ql, qr;
    for (int i = 0; i < 6; ++i) {
        const double lo = p.limits.lower[static_cast<std::size_t>(i)];
        const double hi = p.limits.upper[static_cast<std::size_t>(i)];
        ql[i] = lo + unit(rng) * (hi - lo);
        qr[i] = lo + unit(rng) * (hi - lo);
    }
    return DualArmKinematics::stack(ql, qr);
}

}  // namespace

// ---------------------------------------------------------------------------
// Halton
// ---------------------------------------------------------------------------

TEST(C41Halton, ComponentsStayInUnitInterval) {
    Halton h(12);
    double u[12];
    for (int i = 0; i < 5000; ++i) {
        h.next(u);
        for (int d = 0; d < 12; ++d) {
            ASSERT_GE(u[d], 0.0);
            ASSERT_LT(u[d], 1.0);
        }
    }
}

TEST(C41Halton, OneDimensionalMeanConvergesFast) {
    // Low discrepancy: the 1-D mean of the first 100k points is much closer to
    // 0.5 than Monte Carlo's ~1/sqrt(N) = 3e-3 would give.
    Halton h(1);
    double u[1];
    double sum = 0.0;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
        h.next(u);
        sum += u[0];
    }
    EXPECT_NEAR(sum / n, 0.5, 1e-3);
}

TEST(C41Halton, SegmentsDoNotOverlap) {
    // The parallel workers rely on disjoint segments.
    Halton a(2, 0);
    Halton b(2, 100);
    double ua[2];
    double ub[2];
    for (int i = 0; i < 100; ++i) {
        a.next(ua);
        b.next(ub);
        EXPECT_GT(std::abs(ua[0] - ub[0]) + std::abs(ua[1] - ub[1]), 1e-12);
    }
}

// ---------------------------------------------------------------------------
// SVD metrics
// ---------------------------------------------------------------------------

TEST(C41Svd, IdentityIsFullRankUnitManipulability) {
    const SvdMetrics m = svdMetrics(Eigen::MatrixXd::Identity(6, 6), 1e-6);
    EXPECT_NEAR(m.w, 1.0, 1e-12);
    EXPECT_NEAR(m.smin, 1.0, 1e-12);
    EXPECT_EQ(m.rank, 6);
}

TEST(C41Svd, TinySingularValueIsCountedAsDeficiency) {
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
    J(5, 5) = 1e-9;
    const SvdMetrics m = svdMetrics(J, 1e-6);
    EXPECT_EQ(m.rank, 5);           // sigma_6 / sigma_1 = 1e-9 < 1e-6
    EXPECT_NEAR(m.smin, 1e-9, 1e-15);
    EXPECT_NEAR(m.w, 1e-9, 1e-15);  // prod(sigma_i)
}

// ---------------------------------------------------------------------------
// Filtering
// ---------------------------------------------------------------------------

TEST(C41Sampler, AcceptsAStretchedConfiguration) {
    SamplerParams p;
    WorkspaceSampler sampler(p);

    JointConfig stretched;
    stretched << 0.0, -1.5708, 0.0, 0.0, 0.0, 0.0;
    const Q12 q = DualArmKinematics::stack(stretched, stretched);

    SampleRecord rec;
    ASSERT_TRUE(sampler.evaluate(q, &rec));
    EXPECT_GT(rec.d_arms, 0.05);
    EXPECT_GT(rec.w_left, 0.0);
    EXPECT_GT(rec.w_right, 0.0);
    EXPECT_GE(rec.rank_rel, 0.0);
    EXPECT_LE(rec.rank_rel, 6.0);
}

TEST(C41Sampler, NegativeThresholdAcceptsEverything) {
    SamplerParams p;
    p.d_min = -1.0;        // distance is always >= 0
    p.use_z_floor = false;  // isolate the distance test
    WorkspaceSampler sampler(p);

    std::mt19937 rng(7);
    SampleRecord rec;
    for (int i = 0; i < 50; ++i) {
        EXPECT_TRUE(sampler.evaluate(randomQ12(rng), &rec));
    }
}

TEST(C41Sampler, FloorFilterRejectsBelowGroundConfigurations) {
    // dual_arm_world.sdf has a ground plane at z = 0, so configurations whose DH
    // frame origins or TCP dip below it are physically unreachable.  With the
    // distance test disabled, evaluate() must reduce exactly to aboveFloor().
    SamplerParams p;
    p.d_min = -1.0;  // isolate the floor test
    WorkspaceSampler sampler(p);

    std::mt19937 rng(13);
    SampleRecord rec;
    int rejected = 0;
    for (int i = 0; i < 200; ++i) {
        const Q12 q = randomQ12(rng);
        const bool ok = sampler.evaluate(q, &rec);
        EXPECT_EQ(ok, sampler.aboveFloor(q)) << "sample " << i;
        if (!ok) ++rejected;
    }
    // Not a precise ratio -- just that the filter actually bites and does not
    // swallow everything.
    EXPECT_GT(rejected, 0);
    EXPECT_LT(rejected, 200);
}

TEST(C41Sampler, HugeThresholdRejectsEverything) {
    SamplerParams p;
    p.d_min = 1e9;
    WorkspaceSampler sampler(p);

    std::mt19937 rng(11);
    SampleRecord rec;
    for (int i = 0; i < 50; ++i) {
        EXPECT_FALSE(sampler.evaluate(randomQ12(rng), &rec));
    }
}

// ---------------------------------------------------------------------------
// Consistency: rank is frame-invariant, and matches finite differences
// ---------------------------------------------------------------------------

TEST(C41Consistency, RelativeRankIsFrameInvariant) {
    DualArmKinematics kin;
    std::mt19937 rng(23);

    int checked = 0;
    for (int i = 0; i < 200 && checked < 100; ++i) {
        const Q12 q = randomQ12(rng);
        const int r_body =
            svdMetrics(kin.relativeJacobian(q, RelativeFrame::BodyR), 1e-6).rank;
        const int r_left =
            svdMetrics(kin.relativeJacobian(q, RelativeFrame::LeftL), 1e-6).rank;
        const int r_world =
            svdMetrics(kin.relativeJacobian(q, RelativeFrame::WorldSpatial), 1e-6).rank;
        // Ad(T) is invertible => rank(J_rel) does not depend on the frame.
        EXPECT_EQ(r_body, r_left);
        EXPECT_EQ(r_body, r_world);
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

TEST(C41Consistency, RelativeRankMatchesFiniteDifference) {
    // log((^L T_R)^-1 * T_target) differentiated numerically must have the same
    // numerical rank as the analytic J_rel.  A frame/convention mistake (the
    // C2.4a failure mode) shows up here immediately.
    DualArmKinematics kin;
    std::mt19937 rng(31);
    const double h = 1e-6;

    for (int trial = 0; trial < 20; ++trial) {
        const Q12 q0 = randomQ12(rng);
        const dual_arm::Iso3 target = kin.leftTright(q0);

        Eigen::MatrixXd J_fd(6, 12);
        for (int j = 0; j < 12; ++j) {
            Q12 qp = q0;
            Q12 qm = q0;
            qp[j] += h;
            qm[j] -= h;
            const dual_arm::Vec6 ep = kin.relativeError(qp, target);
            const dual_arm::Vec6 em = kin.relativeError(qm, target);
            J_fd.col(j) = (ep - em) / (2.0 * h);
        }

        const int rank_fd = svdMetrics(J_fd, 1e-6).rank;
        const int rank_an =
            svdMetrics(kin.relativeJacobian(q0, RelativeFrame::BodyR), 1e-6).rank;
        EXPECT_EQ(rank_fd, rank_an) << "trial " << trial;
    }
}

TEST(C41Consistency, RelativeManipulabilityIsFrameInvariant) {
    // w_rel = sqrt(det(J J^T)) = prod(sigma_i).  Changing the expression frame
    // left-multiplies J by Ad(T), and det(Ad(T)) = det(R)^2 = 1, so the product
    // of singular values is EXACTLY frame-invariant even though Ad is not
    // orthogonal.  (Measured: frame-to-frame difference ~1e-14.)
    DualArmKinematics kin;
    std::mt19937 rng(41);

    for (int trial = 0; trial < 50; ++trial) {
        const Q12 q = randomQ12(rng);
        const double w_body =
            svdMetrics(kin.relativeJacobian(q, RelativeFrame::BodyR), 1e-6).w;
        const double w_left =
            svdMetrics(kin.relativeJacobian(q, RelativeFrame::LeftL), 1e-6).w;
        const double w_world =
            svdMetrics(kin.relativeJacobian(q, RelativeFrame::WorldSpatial), 1e-6).w;
        ASSERT_GT(w_body, 0.0);
        EXPECT_NEAR(w_left / w_body, 1.0, 1e-9) << "trial " << trial;
        EXPECT_NEAR(w_world / w_body, 1.0, 1e-9) << "trial " << trial;
    }
}

TEST(C41Consistency, RelativeSminIsFrameDependent) {
    // The COMPLEMENT of the test above: individual singular values (and hence
    // the conditioning sigma_min) DO depend on the expression frame, because Ad
    // is not orthogonal when the relative transform has a translation.  This is
    // asserted on purpose so that the field is never reported without its frame.
    DualArmKinematics kin;
    std::mt19937 rng(43);

    int differs = 0;
    for (int trial = 0; trial < 50; ++trial) {
        const Q12 q = randomQ12(rng);
        const double s_body =
            svdMetrics(kin.relativeJacobian(q, RelativeFrame::BodyR), 1e-6).smin;
        const double s_world =
            svdMetrics(kin.relativeJacobian(q, RelativeFrame::WorldSpatial), 1e-6).smin;
        if (std::abs(s_body - s_world) > 1e-12 * s_body) ++differs;
    }
    EXPECT_GT(differs, 0);
}

// ---------------------------------------------------------------------------
// Record encoding
// ---------------------------------------------------------------------------

TEST(C41Record, EncodingOrderMatchesSchema) {
    SampleRecord r;
    r.q = Q12::LinSpaced(12, 0.0, 11.0);
    r.p_left = Eigen::Vector3d(100.0, 101.0, 102.0);
    r.p_right = Eigen::Vector3d(200.0, 201.0, 202.0);
    r.p_object = Eigen::Vector3d(300.0, 301.0, 302.0);
    r.p_rel = Eigen::Vector3d(400.0, 401.0, 402.0);
    r.rel_rotvec = Eigen::Vector3d(500.0, 501.0, 502.0);
    r.d_arms = 1.0;
    r.w_left = 2.0;
    r.w_right = 3.0;
    r.smin_left = 4.0;
    r.smin_right = 5.0;
    r.w_rel_body = 6.0;
    r.smin_rel_body = 7.0;
    r.w_rel_left = 8.0;
    r.w_rel_world = 9.0;
    r.rank_rel = 6.0;

    float out[kRecordFloats];
    encodeRecord(r, out);

    EXPECT_EQ(kRecordFloats, 37);
    for (int i = 0; i < 12; ++i) EXPECT_FLOAT_EQ(out[i], static_cast<float>(i));
    EXPECT_FLOAT_EQ(out[12], 100.0f);
    EXPECT_FLOAT_EQ(out[26], 502.0f);
    EXPECT_FLOAT_EQ(out[27], 1.0f);
    EXPECT_FLOAT_EQ(out[35], 9.0f);
    EXPECT_FLOAT_EQ(out[36], 6.0f);
}

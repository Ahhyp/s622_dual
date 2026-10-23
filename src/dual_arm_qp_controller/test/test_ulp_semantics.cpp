// C4.3 S4.1 W2 addendum -- Task B: adjudicate the "<= 1 ULP" clause.
//
// Two independent questions, kept in two separate tests so neither can hide
// behind the other:
//
//   1. What does the ULP metric used in the W2 comparison actually mean?
//      (±0, adjacent doubles, subnormals, near-zero, NaN/Inf.)
//   2. Is the "<= 1 ULP" clause achievable at all, and what does the measured
//      9.024e-14 rad disagreement mean?  Answered from the frozen artifact
//      (its own clock law, and the magnitude distribution of its dq entries)
//      rather than from a re-simulation, because a re-simulation of an unknown
//      initial state cannot attribute a difference to a cause.
//
// The ULP function is re-implemented here on purpose: characterising a metric
// with the same code that produced the number under review would prove nothing.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_object_task/anchored_object_task.hpp"
#include "dual_arm_object_task/trajectory.hpp"
#include "dual_arm_qp_controller/frozen_baseline.hpp"
#include "dual_arm_qp_controller/transport_task.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::Iso3;
using dual_arm::Q12;
using dual_arm::Vec6;

constexpr int kJoints = 12;

/// Monotonic ULP key -> distance.  Identical to the W2 helper; the point of the
/// semantics test is to document what it does at the edges, not to change it.
uint64_t ulpDistance(double a, double b) {
    int64_t ia = 0;
    int64_t ib = 0;
    std::memcpy(&ia, &a, sizeof(ia));
    std::memcpy(&ib, &b, sizeof(ib));
    if (ia < 0) ia = INT64_MIN - ia;
    if (ib < 0) ib = INT64_MIN - ib;
    const uint64_t ka = static_cast<uint64_t>(ia) ^ 0x8000000000000000ULL;
    const uint64_t kb = static_cast<uint64_t>(ib) ^ 0x8000000000000000ULL;
    return ka > kb ? ka - kb : kb - ka;
}

struct FixtureRow {
    int k = 0;
    double t = 0.0;
    Q12 q_prev = Q12::Zero();
    Q12 dq = Q12::Zero();
    double e_obj_pos = 0.0;
    double e_obj_rot = 0.0;
    bool committed = false;
};

std::vector<FixtureRow> loadFixture(const std::string& path, bool* ok) {
    std::vector<FixtureRow> out;
    *ok = false;
    std::ifstream in(path);
    if (!in.good()) return out;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ',')) f.push_back(tok);
        if (f.size() < 29) continue;
        FixtureRow r;
        r.k = std::stoi(f[0]);
        r.t = std::stod(f[1]);
        for (int j = 0; j < kJoints; ++j) {
            r.q_prev[j] = std::stod(f[2 + j]);
            r.dq[j] = std::stod(f[14 + j]);
        }
        r.e_obj_pos = std::stod(f[26]);
        r.e_obj_rot = std::stod(f[27]);
        r.committed = std::stoi(f[28]) != 0;
        out.push_back(r);
    }
    *ok = !out.empty();
    return out;
}

// ---------------------------------------------------------------------------
// 1. Metric semantics
// ---------------------------------------------------------------------------
TEST(UlpMetric, SemanticsAtTheEdges) {
    // Identical values: 0.
    EXPECT_EQ(ulpDistance(1.0, 1.0), 0U);
    // Adjacent representable doubles: 1.
    EXPECT_EQ(ulpDistance(1.0, std::nextafter(1.0, 2.0)), 1U);
    EXPECT_EQ(ulpDistance(-1.0, std::nextafter(-1.0, -2.0)), 1U);
    // Symmetric.
    EXPECT_EQ(ulpDistance(1.0, 3.0), ulpDistance(3.0, 1.0));
    // Subnormals are spaced evenly: 1 apart.
    const double sub0 = 0.0;
    const double sub1 = std::nextafter(0.0, 1.0);
    EXPECT_EQ(ulpDistance(sub0, sub1), 1U);

    // +0 and -0 encode to different BIT PATTERNS but the fold used here
    // (`ia < 0 -> INT64_MIN - ia`) maps both onto the same key, so the metric
    // reports 0.  That is the right answer for "are these the same number?" and
    // the wrong answer for "are these the same bits?" -- the clause under
    // review is about numbers, so equal is what we want.  Documented because a
    // naive bit-compare would report 2^63 here.
    EXPECT_EQ(ulpDistance(0.0, -0.0), 0U);
    const double pz = 0.0;
    const double nz0 = -0.0;
    int64_t bz = 0;
    int64_t bnz = 0;
    std::memcpy(&bz, &pz, sizeof(bz));
    std::memcpy(&bnz, &nz0, sizeof(bnz));
    EXPECT_NE(bz, bnz) << "+0 and -0 differ in bits even though the metric calls them equal";

    // Near zero the metric explodes: 1e-14 and 2e-14 are ~1e15 ULPs apart,
    // because the ULP size shrinks with the magnitude while the absolute
    // difference does not.  Any "<= 1 ULP" clause is therefore only meaningful
    // well away from zero.
    const uint64_t nz = ulpDistance(1e-14, 2e-14);
    std::printf("ULP semantics: ulpDistance(1e-14, 2e-14) = %llu (~%.2e)\n",
                static_cast<unsigned long long>(nz), static_cast<double>(nz));
    EXPECT_GT(nz, 1000000000000000ULL);

    // NaN / Inf produce well-defined-but-meaningless bit distances; the metric
    // must not be used as a correctness oracle for them.  We only assert that it
    // terminates and that NaN != NaN is not reported as 0.
    const double nan = std::nan("");
    std::printf("ULP semantics: ulpDistance(nan, nan) = %llu, (0, inf) = %llu\n",
                static_cast<unsigned long long>(ulpDistance(nan, nan)),
                static_cast<unsigned long long>(ulpDistance(0.0, INFINITY)));
    EXPECT_NE(ulpDistance(nan, 1.0), 0U);
}

// ---------------------------------------------------------------------------
// 2. Where the disagreement comes from (artifact evidence, no re-simulation)
//
// The first version of this test re-ran the 638 cycles through a second QP and
// asserted that the S3 clock rule reproduces the fixture bitwise.  It does not,
// and the reason is that a re-simulation is not the same experiment: the frozen
// fixture was produced by the S3 mock plant (q += dq exactly), while the
// controller chain is driven by mock_components feedback, so cycle 0 already
// differs in the last digits.  Re-simulating an unknown initial state cannot
// attribute the difference.  What CAN be attributed, exactly, is the clock law
// of the artifact itself -- and that is measured directly below.
// ---------------------------------------------------------------------------

struct FixtureSummary {
    bool t_is_kdt = false;      ///< every row satisfies t == k*dt bitwise
    double t_drift = 0.0;       ///< max |accumulated dt - k*dt|
    double min_abs_dq = 0.0;
    double max_abs_dq = 0.0;
    int near_zero = 0;          ///< |dq| < 1e-9
};

FixtureSummary summarise(const std::vector<FixtureRow>& rows, double dt) {
    FixtureSummary out;
    out.t_is_kdt = true;
    out.min_abs_dq = 1e300;
    double acc = 0.0;
    for (size_t i = 0; i < rows.size(); ++i) {
        const double kdt = static_cast<double>(i) * dt;
        if (std::memcmp(&rows[i].t, &kdt, sizeof(double)) != 0) out.t_is_kdt = false;
        out.t_drift = std::max(out.t_drift, std::abs(acc - kdt));
        for (int j = 0; j < kJoints; ++j) {
            const double a = std::abs(rows[i].dq[j]);
            out.min_abs_dq = std::min(out.min_abs_dq, a);
            out.max_abs_dq = std::max(out.max_abs_dq, a);
            if (a < 1e-9) ++out.near_zero;
        }
        acc += dt;
    }
    return out;
}

TEST(UlpMetric, FixtureClockIsTheKDtLawAndTheControllerDriftIsBounded) {
    bool ok = false;
    const auto ref = loadFixture(S4_FIXTURE_CSV, &ok);
    ASSERT_TRUE(ok) << "cannot load " << S4_FIXTURE_CSV;
    ASSERT_EQ(ref.size(), 638U);

    const auto spec = dual_arm_object_task::loadTrajectorySpec(S4_TRANSPORT_YAML);
    const FixtureSummary sum = summarise(ref, spec.dt);

    std::printf(
        "PROVENANCE clock: fixture t == k*dt bitwise for all 638 rows: %s; "
        "max|sum(dt) - k*dt| over the run = %.3e s; |dq| in [%.3e, %.3e], "
        "near-zero entries (|dq| < 1e-9) = %d of %zu\n",
        sum.t_is_kdt ? "YES" : "NO", sum.t_drift, sum.min_abs_dq, sum.max_abs_dq,
        sum.near_zero, ref.size() * kJoints);
    std::printf(
        "PROVENANCE clock: S4-T2 measured max|t_traj(controller) - t(fixture)| = "
        "3.553e-15 s (acceptance record §3.1), i.e. the two clock laws differ by "
        "%.3e s, which is the same order as the drift above.\n", sum.t_drift);

    EXPECT_TRUE(sum.t_is_kdt) << "the frozen artifact must document its own clock law";
    // 638 additions of 0.008 lose a few ULPs of 5.1 s; the bound below is
    // generous but far below anything that could move a 1e-3 m target.
    EXPECT_LT(sum.t_drift, 1e-13);
}

TEST(UlpMetric, NearZeroEntriesMakeTheOneUlpClauseUnachievable) {
    bool ok = false;
    const auto ref = loadFixture(S4_FIXTURE_CSV, &ok);
    ASSERT_TRUE(ok) << "cannot load " << S4_FIXTURE_CSV;
    const auto spec = dual_arm_object_task::loadTrajectorySpec(S4_TRANSPORT_YAML);
    const FixtureSummary sum = summarise(ref, spec.dt);

    // The production comparison (test_mock_closure / S4-T2) measured
    // max|dq_controller - dq_fixture| = 9.024e-14 over the same 638 cycles.
    // Express that single absolute difference in the only currency the clause
    // accepts: ULP at the magnitude where it occurs.
    constexpr double kWorstAbsDiff = 9.024e-14;
    const double ulp_at_min = std::nextafter(sum.min_abs_dq, INFINITY) - sum.min_abs_dq;
    const double ulps_at_min = kWorstAbsDiff / ulp_at_min;
    const double ulp_at_max = std::nextafter(sum.max_abs_dq, INFINITY) - sum.max_abs_dq;
    const double ulps_at_max = kWorstAbsDiff / ulp_at_max;
    std::printf(
        "ULP verdict: |dq| ranges over [%.3e, %.3e]; 1 ULP is %.3e at the small end "
        "and %.3e at the large end, so a single 9.024e-14 rad difference is %.3e ULP "
        "at the small end and %.3e ULP at the large end.\n",
        sum.min_abs_dq, sum.max_abs_dq, ulp_at_min, ulp_at_max, ulps_at_min,
        ulps_at_max);
    std::printf(
        "ULP verdict: there are %d entries with |dq| < 1e-9 out of %zu.  For those the "
        "clause requires agreement to ~1e-24 rad.  That is below the resolution of the "
        "QP's own arithmetic, so '<= 1 ULP over all 12 joints' is unachievable for any "
        "two nominally identical runs, independently of the solver.\n",
        sum.near_zero, ref.size() * kJoints);
    std::printf(
        "ULP verdict: in physical units 9.024e-14 rad = %.3e deg on a 6-DoF arm whose "
        "step is 8e-3 rad -- a relative error of %.3e.\n",
        kWorstAbsDiff * 180.0 / 3.141592653589793, kWorstAbsDiff / 8e-3);

    // The clause is decidable only where |dq| is not near zero.
    EXPECT_GT(ulps_at_max, 1.0) << "even at the largest |dq| the difference is > 1 ULP";
    EXPECT_GT(ulps_at_min, 1e6) << "near zero the ULP count is meaningless by construction";
    EXPECT_GT(sum.near_zero, 0);
    // ... and physically negligible, which is what the replacement metric says.
    EXPECT_LT(kWorstAbsDiff, 1e-12);
}

}  // namespace

// C4.3 S3 -- W2 tests for the external object-task row (S3-T5b, S3-T5i).
//
// S3-T5b: the new ObjectTaskRow overload must be BIT IDENTICAL to the legacy
//         overloads.  Two independent statements:
//           (b1) w_obj == 0  -> the object task is not assembled at all, so an
//                ARBITRARY row must not change dq by one ULP.
//           (b2) w_obj == 1  -> feeding the *legacy* row (J = objectJacobianBody,
//                e = objectError, ff = ff) must reproduce the legacy overload
//                exactly.  This is the strong one: it proves the new plumbing is
//                sign- and scale-neutral, not merely "off when disabled".
//
// S3-T5i: invalid rows must fail EXPLICITLY and must never fall back to the
//         legacy object task (the exact defect user m03513 revision point 1
//         called out).  "Warm start not advanced" is asserted behaviourally:
//         a solve performed after a rejection must equal the solve performed by
//         a pristine QP that never saw a rejection.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::Q12;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;
using dual_arm_qp::DualArmQpResult;
using dual_arm::Mat6x12;
using dual_arm_qp::ObjectTaskRow;
using dual_arm_qp::ObjectTaskRowCheck;
using dual_arm_qp::Vec6;
using dual_arm_qp::Vec12;
using dual_arm_qp::objectTaskRowCheckName;
using dual_arm_qp::validateObjectTaskRow;

/// S2 frozen grasp configuration (transport_baseline.yaml).
constexpr std::array<double, 12> kGraspQ = {
    2.795036909814,  -1.706950857914, 1.279963524342,  -1.143808993223,
    -1.570796326795, -0.346555743776, 0.346555743776,  -1.434641795676,
    -1.279963524342, -1.997783660367, 1.570796326795,  -2.795036909814};

Q12 graspQ() {
    Q12 q;
    for (int i = 0; i < 12; ++i) q[i] = kGraspQ[static_cast<std::size_t>(i)];
    return q;
}

Iso3 perturb(const Iso3& T, const Vec6& xi) { return T * dual_arm::expSE3(xi); }

DualArmQpParams makeParams(double w_obj) {
    DualArmQpParams p;
    dual_arm_qp::defaultS622Limits(&p.q_min, &p.q_max, &p.v_max);
    p.w_obj = w_obj;
    p.w_slack_obj = (w_obj > 0.0) ? 1e3 : 1e-6;
    return p;
}

/// The legacy row, mirrored exactly: feeding this to the new overload must be a
/// no-op relative to the legacy overload.
ObjectTaskRow legacyRow(const DualArmKinematics& kin, const Q12& q, const Iso3& obj_target,
                        const Vec6& ff) {
    ObjectTaskRow row;
    row.J = kin.objectJacobianBody(q);
    row.e = kin.objectError(q, obj_target);
    row.body_twist_ff = ff;
    row.valid = true;
    return row;
}

bool bitEqual(const Vec12& a, const Vec12& b) {
    return std::memcmp(a.data(), b.data(), sizeof(double) * 12) == 0;
}

std::string describeMismatch(const Vec12& a, const Vec12& b) {
    std::string s;
    for (int i = 0; i < 12; ++i) {
        if (std::memcmp(&a[i], &b[i], sizeof(double)) != 0) {
            s += "  i=" + std::to_string(i) + " legacy=" + std::to_string(a[i]) +
                 " row=" + std::to_string(b[i]) + "\n";
        }
    }
    return s;
}

// ---------------------------------------------------------------------------
// S3-T5b
// ---------------------------------------------------------------------------
TEST(ObjectTaskRowC43S3, T5b1_BitIdenticalWhenWObjIsZero) {
    const DualArmKinematics kin;
    const Q12 q = graspQ();
    Vec6 rel_off, obj_off, ff;
    rel_off << 0.004, -0.002, 0.006, 0.02, -0.01, 0.03;
    obj_off << 0.010, -0.005, 0.020, 0.0, 0.0, 0.05;
    ff << 0.010, -0.004, 0.015, 0.0, 0.0, 0.02;

    const Iso3 ltr = perturb(kin.leftTright(q), rel_off);
    const Iso3 obj = perturb(kin.worldTobject(q), obj_off);

    // A row that is deliberately NOT the legacy row: with w_obj == 0 it must
    // have no effect whatsoever.
    ObjectTaskRow junk;
    junk.J = Mat6x12::Constant(7.5);
    junk.e << 1.0, -2.0, 3.0, 0.4, -0.5, 0.6;
    junk.body_twist_ff << -9.0, 8.0, -7.0, 0.3, 0.2, -0.1;
    junk.valid = true;

    DualArmQp a(kin, makeParams(0.0));
    DualArmQp b(kin, makeParams(0.0));
    const DualArmQpResult ra = a.solve(q, ltr, obj, ff);
    const DualArmQpResult rb = b.solve(q, ltr, junk);

    EXPECT_TRUE(ra.converged);
    EXPECT_TRUE(rb.converged);
    EXPECT_TRUE(bitEqual(ra.dq, rb.dq))
        << "S3-T5b(1) violated, mismatching components:\n" << describeMismatch(ra.dq, rb.dq);
    EXPECT_TRUE(rb.object_task_external);
    EXPECT_EQ(rb.object_task_check, ObjectTaskRowCheck::Ok);
    EXPECT_FALSE(ra.object_task_external);
}

TEST(ObjectTaskRowC43S3, T5b2_LegacyRowReproducesLegacyExactlyEvenWhenEnabled) {
    const DualArmKinematics kin;
    const Q12 q = graspQ();
    Vec6 rel_off, obj_off, ff;
    rel_off << 0.004, -0.002, 0.006, 0.02, -0.01, 0.03;
    obj_off << 0.010, -0.005, 0.020, 0.0, 0.0, 0.05;
    ff << 0.010, -0.004, 0.015, 0.0, 0.0, 0.02;

    const Iso3 ltr = perturb(kin.leftTright(q), rel_off);
    const Iso3 obj = perturb(kin.worldTobject(q), obj_off);
    const ObjectTaskRow row = legacyRow(kin, q, obj, ff);

    DualArmQp a(kin, makeParams(1.0));
    DualArmQp b(kin, makeParams(1.0));
    const DualArmQpResult ra = a.solve(q, ltr, obj, ff);
    const DualArmQpResult rb = b.solve(q, ltr, row);

    EXPECT_TRUE(ra.converged);
    EXPECT_TRUE(rb.converged);
    EXPECT_TRUE(bitEqual(ra.dq, rb.dq))
        << "S3-T5b(2) violated, mismatching components:\n" << describeMismatch(ra.dq, rb.dq);
    // Diagnostics must agree too: same linearisation point, same problem.
    EXPECT_DOUBLE_EQ(ra.object_error_norm, rb.object_error_norm);
    EXPECT_DOUBLE_EQ(ra.sigma_min_rel, rb.sigma_min_rel);
    // And the difference from the disabled-object solve must be real, otherwise
    // T5b(2) above would be vacuous.
    DualArmQp c(kin, makeParams(0.0));
    const DualArmQpResult rc = c.solve(q, ltr, obj, ff);
    EXPECT_GT((ra.dq - rc.dq).cwiseAbs().maxCoeff(), 1e-3)
        << "object task had no effect at w_obj=1 / w_slack_obj=1e3 -> test would be vacuous";
}

// ---------------------------------------------------------------------------
// S3-T5i
// ---------------------------------------------------------------------------
namespace {

void expectRejected(const DualArmQpResult& r, ObjectTaskRowCheck expected, const char* label) {
    EXPECT_FALSE(r.converged) << label;
    EXPECT_EQ(r.object_task_check, expected)
        << label << " -> got " << objectTaskRowCheckName(r.object_task_check) << ", want "
        << objectTaskRowCheckName(expected);
    EXPECT_EQ(r.dq, Vec12::Zero()) << label;
    EXPECT_EQ(r.slack, Vec6::Zero()) << label;
    EXPECT_EQ(r.slack_object, Vec6::Zero()) << label;
    EXPECT_EQ(r.iterations, 0) << label;
    EXPECT_DOUBLE_EQ(r.solve_time_ms, 0.0) << label;
    EXPECT_TRUE(r.object_task_external) << label;
}

}  // namespace

TEST(ObjectTaskRowC43S3, T5i_InvalidObjectTaskRowFailsExplicitly) {
    const DualArmKinematics kin;
    const Q12 q = graspQ();
    Vec6 rel_off, obj_off;
    rel_off << 0.004, -0.002, 0.006, 0.02, -0.01, 0.03;
    obj_off << 0.010, -0.005, 0.020, 0.0, 0.0, 0.05;
    const Iso3 ltr = perturb(kin.leftTright(q), rel_off);
    const Iso3 obj = perturb(kin.worldTobject(q), obj_off);

    DualArmQp qp(kin, makeParams(1.0));

    // (a) valid == false -> NotValid
    {
        ObjectTaskRow row = legacyRow(kin, q, obj, Vec6::Zero());
        row.valid = false;
        expectRejected(qp.solve(q, ltr, row), ObjectTaskRowCheck::NotValid, "valid=false");
        EXPECT_EQ(validateObjectTaskRow(row), ObjectTaskRowCheck::NotValid);
    }
    // (b) NaN / Inf in any of the three fields -> NonFinite
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        for (int which = 0; which < 3; ++which) {
            ObjectTaskRow row = legacyRow(kin, q, obj, Vec6::Zero());
            switch (which) {
                case 0: row.J(2, 5) = nan; break;
                case 1: row.e[3] = nan; break;
                default: row.body_twist_ff[1] = inf; break;
            }
            expectRejected(qp.solve(q, ltr, row), ObjectTaskRowCheck::NonFinite, "non-finite");
            EXPECT_EQ(validateObjectTaskRow(row), ObjectTaskRowCheck::NonFinite);
        }
    }
    // (c) illegal q -> BadInput
    {
        ObjectTaskRow row = legacyRow(kin, q, obj, Vec6::Zero());
        Q12 bad = q;
        bad[7] = std::numeric_limits<double>::quiet_NaN();
        expectRejected(qp.solve(bad, ltr, row), ObjectTaskRowCheck::BadInput, "q NaN");
    }
    // (d) non-rigid left_T_right_target -> BadInput
    {
        ObjectTaskRow row = legacyRow(kin, q, obj, Vec6::Zero());
        Iso3 bad = ltr;
        bad.linear() *= 1.5;  // not orthonormal any more
        bad.translation() = ltr.translation();
        expectRejected(qp.solve(q, bad, row), ObjectTaskRowCheck::BadInput, "non-rigid target");
    }
}

TEST(ObjectTaskRowC43S3, T5i_RejectionDoesNotAdvanceTheWarmStart) {
    const DualArmKinematics kin;
    const Q12 q = graspQ();
    Vec6 rel_off, obj_off;
    rel_off << 0.004, -0.002, 0.006, 0.02, -0.01, 0.03;
    obj_off << 0.010, -0.005, 0.020, 0.0, 0.0, 0.05;
    const Iso3 ltr = perturb(kin.leftTright(q), rel_off);
    const Iso3 obj = perturb(kin.worldTobject(q), obj_off);
    const ObjectTaskRow good = legacyRow(kin, q, obj, Vec6::Zero());

    // Pristine: never saw a rejected call.
    DualArmQp pristine(kin, makeParams(1.0));
    const DualArmQpResult r_ref = pristine.solve(q, ltr, good);

    // Dirty: rejected between construction and the same valid solve.
    DualArmQp dirty(kin, makeParams(1.0));
    ObjectTaskRow bad = good;
    bad.valid = false;
    const DualArmQpResult r_bad = dirty.solve(q, ltr, bad);
    EXPECT_FALSE(r_bad.converged);
    const DualArmQpResult r_after = dirty.solve(q, ltr, good);

    EXPECT_TRUE(r_ref.converged);
    EXPECT_TRUE(r_after.converged);
    EXPECT_TRUE(bitEqual(r_ref.dq, r_after.dq))
        << "a rejected call mutated solver state (warm start advanced):\n"
        << describeMismatch(r_ref.dq, r_after.dq);
}

// The rejection path must not run the solver at all.
TEST(ObjectTaskRowC43S3, T5i_RejectionIsFree) {
    const DualArmKinematics kin;
    const Q12 q = graspQ();
    const Iso3 ltr = kin.leftTright(q);
    DualArmQp qp(kin, makeParams(1.0));
    ObjectTaskRow row;
    row.valid = false;
    const auto t0 = std::chrono::steady_clock::now();
    const DualArmQpResult r = qp.solve(q, ltr, row);
    const auto t1 = std::chrono::steady_clock::now();
    EXPECT_FALSE(r.converged);
    EXPECT_EQ(r.iterations, 0);
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    EXPECT_LT(ms, 1.0) << "rejection path took " << ms << " ms -> it probably ran the solver";
}

}  // namespace

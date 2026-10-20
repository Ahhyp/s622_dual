// C4.3 S3 -- W2 test S3-T5c: the LEGACY object path is behaviourally unchanged.
//
// Compares the current build against dq vectors captured once from
// libdual_arm_qp_core at git HEAD 6e5cb26 (pre-C4.3-S3) by
// tools/gen_legacy_golden.cpp.  The comparison is BITWISE: not one ULP of drift
// is allowed on the legacy overloads, because C4.2 and every C2.x result was
// produced with them.
//
// Regenerating the golden values is only legitimate when the legacy behaviour is
// *intended* to change.  See tools/gen_legacy_golden.cpp for the exact recipe.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cstring>
#include <string>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

#include "legacy_cases.hpp"        // tools/ (shared with the generator)
#include "golden_legacy_dq.hpp"    // test/ (generated)

namespace {

using legacy_golden::CaseSpec;
using legacy_golden::kCases;
using legacy_golden::kGolden;
using legacy_golden::Kind;

dual_arm_qp::DualArmQpParams makeParams(const CaseSpec& c) {
    dual_arm_qp::DualArmQpParams p;
    dual_arm_qp::defaultS622Limits(&p.q_min, &p.q_max, &p.v_max);
    p.w_obj = c.w_obj;
    p.w_slack_obj = (c.w_obj > 0.0) ? 1e3 : 1e-6;
    return p;
}

}  // namespace

TEST(LegacyGoldenC43S3, T5c_LegacyPathIsBitwiseUnchanged) {
    static_assert(sizeof(kGolden) / sizeof(kGolden[0]) == sizeof(kCases) / sizeof(kCases[0]),
                  "golden file and case table have drifted apart -- regenerate the golden");

    const dual_arm::DualArmKinematics kin;
    const dual_arm::Q12 q = legacy_golden::graspQ();

    for (std::size_t ci = 0; ci < kCases.size(); ++ci) {
        const CaseSpec& c = kCases[ci];
        const legacy_golden::GoldenDq& g = kGolden[ci];
        ASSERT_STREQ(c.name, g.name) << "case order mismatch at index " << ci;

        dual_arm_qp::DualArmQpParams p = makeParams(c);
        dual_arm_qp::DualArmQp qp(kin, p);
        qp.setWarmStart(c.warm_start);

        const dual_arm::Iso3 ltr_target =
            kin.leftTright(q) * legacy_golden::expTwist(c.relative_offset);
        const dual_arm::Iso3 obj_target =
            kin.worldTobject(q) * legacy_golden::expTwist(c.object_offset);
        dual_arm::Vec6 ff;
        ff << c.ff[0], c.ff[1], c.ff[2], c.ff[3], c.ff[4], c.ff[5];

        dual_arm_qp::DualArmQpResult r;
        switch (c.kind) {
            case Kind::kRelativeOnly:
                r = qp.solve(q, ltr_target);
                break;
            case Kind::kRelativeAndObject:
                r = qp.solve(q, ltr_target, obj_target);
                break;
            case Kind::kWithFeedforward:
                r = qp.solve(q, ltr_target, obj_target, ff);
                break;
        }

        ASSERT_EQ(r.converged, g.converged) << c.name << ": converged flag changed";
        for (int i = 0; i < 12; ++i) {
            const bool same = std::memcmp(&r.dq[i], &g.dq[i], sizeof(double)) == 0;
            ASSERT_TRUE(same) << c.name << ": component " << i << " changed: HEAD=" << g.dq[i]
                              << " now=" << r.dq[i] << "  (delta=" << (r.dq[i] - g.dq[i])
                              << ")";
        }
    }
}

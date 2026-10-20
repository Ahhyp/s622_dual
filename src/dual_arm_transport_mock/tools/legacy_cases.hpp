// C4.3 S3 -- shared input table for the LEGACY-equivalence golden test (S3-T5c).
//
// One source of truth: the generator (tools/gen_legacy_golden.cpp) and the test
// (test/test_legacy_golden_c43s3.cpp) both include this header, so the inputs
// can never drift apart.  The generator only adds the OUTPUTS (the dq vectors
// produced by the HEAD build of libdual_arm_qp_core).
//
// Why a golden file instead of linking /tmp/s3_w0/libdual_arm_qp_core.HEAD.so at
// test time: DualArmQpResult gained two members in C4.3 S3, so a binary built
// against the new header CANNOT link against the old library (different struct
// layout -> memory corruption, not a clean error).  Capturing the old outputs
// once, with the old header, and embedding them is the only sound way to prove
// "legacy path unchanged" afterwards.
#ifndef DUAL_ARM_TRANSPORT_MOCK__TOOLS__LEGACY_CASES_HPP_
#define DUAL_ARM_TRANSPORT_MOCK__TOOLS__LEGACY_CASES_HPP_

#include <array>
#include <cstddef>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace legacy_golden {

using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::Q12;

/// S2 frozen grasp configuration (transport_baseline.yaml, 12 decimals).
inline constexpr std::array<double, 12> kGraspQ = {
    2.795036909814,  -1.706950857914, 1.279963524342,  -1.143808993223,
    -1.570796326795, -0.346555743776, 0.346555743776,  -1.434641795676,
    -1.279963524342, -1.997783660367, 1.570796326795,  -2.795036909814};

/// Which legacy overload to call.
enum class Kind {
  kRelativeOnly = 0,      // solve(q, ltr)
  kRelativeAndObject = 1, // solve(q, ltr, T_object)
  kWithFeedforward = 2,   // solve(q, ltr, T_object, object_body_twist_ff)
};

struct CaseSpec {
  const char* name;
  Kind kind;
  double w_obj;
  bool warm_start;
  /// Object target = worldTobject(q) * exp(this twist)  [body twist, 6]
  std::array<double, 6> object_offset;
  /// Relative target = leftTright(q) * exp(this twist)  [body twist, 6]
  std::array<double, 6> relative_offset;
  /// Feed-forward passed verbatim (object body twist).
  std::array<double, 6> ff;
};

inline constexpr std::array<CaseSpec, 6> kCases = {{
    {"rel_only_warm_wobj0", Kind::kRelativeOnly, 0.0, true,
     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.004, -0.002, 0.006, 0.02, -0.01, 0.03},
     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    {"rel_obj_warm_wobj0", Kind::kRelativeAndObject, 0.0, true,
     {0.010, -0.005, 0.020, 0.0, 0.0, 0.05}, {0.002, 0.001, -0.003, 0.01, 0.0, 0.0},
     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    {"rel_obj_warm_wobj1", Kind::kRelativeAndObject, 1.0, true,
     {0.010, -0.005, 0.020, 0.0, 0.0, 0.05}, {0.002, 0.001, -0.003, 0.01, 0.0, 0.0},
     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    {"rel_obj_ff_wobj1", Kind::kWithFeedforward, 1.0, true,
     {0.010, -0.005, 0.020, 0.0, 0.0, 0.05}, {0.002, 0.001, -0.003, 0.01, 0.0, 0.0},
     {0.010, -0.004, 0.015, 0.0, 0.0, 0.02}},
    {"rel_obj_ff_wobj0", Kind::kWithFeedforward, 0.0, true,
     {0.010, -0.005, 0.020, 0.0, 0.0, 0.05}, {0.002, 0.001, -0.003, 0.01, 0.0, 0.0},
     {0.010, -0.004, 0.015, 0.0, 0.0, 0.02}},
    {"rel_only_cold_wobj0", Kind::kRelativeOnly, 0.0, false,
     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.004, -0.002, 0.006, 0.02, -0.01, 0.03},
     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
}};

/// exp() of a 6-vector body twist (translation [m] then rotation [rad]).
inline Iso3 expTwist(const std::array<double, 6>& xi) {
  dual_arm::Vec6 v;
  v << xi[0], xi[1], xi[2], xi[3], xi[4], xi[5];
  return dual_arm::expSE3(v);
}

inline Q12 graspQ() {
  Q12 q;
  for (int i = 0; i < 12; ++i) q[i] = kGraspQ[static_cast<std::size_t>(i)];
  return q;
}

}  // namespace legacy_golden

#endif  // DUAL_ARM_TRANSPORT_MOCK__TOOLS__LEGACY_CASES_HPP_

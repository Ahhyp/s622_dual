// C4.3 S2 / W0 -- S2-T17 (user m02331 F-S1-1).  S1 decided not to change
// se3.hpp.  That decision is only sound if a hard filter that thresholds the
// logSO3 NORM is stable near pi -- and "we only use the norm" is NOT by itself a
// proof, so this suite MEASURES it instead of asserting it.
//
// MEASURED against src/dual_arm_kinematics/include/dual_arm_kinematics/se3.hpp:51-68.
// Let delta = pi - theta be the true angular distance from pi.
//
//   delta      worst | ||logSO3(R)|| - theta |   regime
//   ---------  --------------------------------  ------------------------------
//   >= 1e-4    <= 8e-8                            general branch, usable
//   1e-5       up to 3.6e-6                       general branch, degrading
//   2e-6       up to 2.1e-4                       general branch, unusable
//   1e-6       up to 4.9e-4                       boundary, decided by noise
//   <= 1e-7    == delta                           near-pi branch, angle FORCED to pi
//
// Two distinct defects, both real:
//   (a) se3.hpp:58 tests (M_PI - th) < 1e-6 on an acos-derived `th` that itself
//       carries ~1e-8..1e-10 of noise, so a true delta of 1e-6 lands on EITHER
//       side of the branch.  On the general side the code divides by
//       2*sin(th) ~ 2e-6, amplifying the (R - R^T) cancellation; the error reaches
//       4.9e-4 rad -- 500x worse than the window the branch exists to avoid.
//   (b) on the near-pi side the function returns M_PI * axis, discarding delta.
//
// Consequence for S2, and the reason S1's no-change decision survives:
//   * kMidpointSafeMinMarginRad = 1.5e-3 rad is decidable: for delta >= 1e-4 the
//     error is <= 8e-8, over four orders of magnitude below the threshold.
//   * the 1e-9 band in closureResidual's rotation_direction_ambiguous flag is NOT
//     decidable -- its real resolution is ~5e-4 rad.  The flag OVER-flags inside
//     the window, i.e. it errs on the safe side, and S2 never evaluates it near pi
//     (a valid closure drives that residual to ~0).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "dual_arm_kinematics/closure_residual.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace {
using dual_arm::Iso3;
using dual_arm::Mat3;
using dual_arm::Vec3;
using dual_arm::Vec6;

Iso3 rotAbout(const Vec3& axis, double angle) {
  Iso3 T = Iso3::Identity();
  T.rotate(Eigen::AngleAxisd(angle, axis.normalized()));
  return T;
}

Vec3 axisA() { return Vec3(1.0, 2.0, 3.0).normalized(); }
Vec3 axisB() { return Vec3(0.3, -0.5, 0.81).normalized(); }

/// | ||logSO3(expSO3(axis*(pi-d)))|| - (pi-d) |
double errAt(const Vec3& axis, double d) {
  const double theta = M_PI - d;
  return std::abs(dual_arm::logSO3(dual_arm::expSO3(axis * theta)).norm() - theta);
}
}  // namespace

// The defensible half of the claim: the S2 guard's threshold IS resolvable.
TEST(LogSo3NormStability, AwayFromPiTheNormIsFarMoreAccurateThanTheS2GuardNeeds) {
  double worst = 0.0;
  for (const Vec3& axis : {axisA(), axisB()}) {
    for (double d : {1e-1, 1e-2, 1e-3, 1e-4}) worst = std::max(worst, errAt(axis, d));
  }
  EXPECT_LT(worst, 1e-6);
  // The midpoint guard thresholds at 1.5e-3; the measurement above is >3 orders
  // (and in fact >4) below it, so the threshold decides by value, not by noise.
  EXPECT_GT(dual_arm::kMidpointSafeMinMarginRad, 1e3 * worst);
}

// The indefensible half: just inside the branch window the code discards delta.
TEST(LogSo3NormStability, InsideTheWindowTheAngleIsForcedToPiAndDeltaIsLost) {
  for (const Vec3& axis : {axisA(), axisB()}) {
    for (double d : {1e-7, 1e-8, 1e-9, 1e-10}) {
      const Vec3 w = dual_arm::logSO3(dual_arm::expSO3(axis * (M_PI - d)));
      EXPECT_NEAR(w.norm(), M_PI, 1e-9) << "delta = " << d;
      // The error is exactly delta: the returned angle is pi regardless.
      EXPECT_NEAR(w.norm() - (M_PI - d), d, 1e-12) << "delta = " << d;
    }
  }
}

// And the boundary itself is decided by noise, not by the value.
TEST(LogSo3NormStability, TheNearPiBranchBoundaryIsDecidedByNoiseNotByDelta) {
  // At delta = 1e-6 the acos-derived theta lands on either side of the
  // `M_PI - th < 1e-6` test depending on the axis.  On the general side the error
  // blows up to 4.9e-4 rad; on the forced side it is exactly 1e-6.  Either way it
  // is far outside any nanoradian-scale band.
  double worst = 0.0;
  for (const Vec3& axis : {axisA(), axisB()}) {
    for (double d : {2e-6, 1e-6}) worst = std::max(worst, errAt(axis, d));
  }
  EXPECT_GT(worst, 1e-5);
  // Record the measurement so a future regression that fixes se3.hpp is caught.
  EXPECT_LT(worst, 1e-3);
}

// The consequence, spelled out: the 1e-9 ambiguity band cannot be resolved.
TEST(LogSo3NormStability, ClosureAmbiguityBandIsNotResolvableAtOneNanoRadian) {
  const Vec3 axis = Vec3(1.0, -0.4, 0.2).normalized();
  const Iso3 object_L = Iso3::Identity();

  // Comfortably outside the window the flag is honest ...
  for (double d : {1e-2, 1e-4, 1e-5}) {
    const dual_arm::ClosureResidual r = dual_arm::closureResidual(object_L, rotAbout(axis, M_PI - d));
    EXPECT_GT(M_PI - r.rotation_rad, 1e-9) << "delta = " << d;
    EXPECT_FALSE(r.rotation_direction_ambiguous) << "delta = " << d;
  }
  // ... but inside it the angle is pinned to pi and the flag fires, even though
  // the true delta is 1e-8 -- nine orders above the declared 1e-9 band.
  for (double d : {1e-8, 1e-9, 1e-10}) {
    const dual_arm::ClosureResidual r = dual_arm::closureResidual(object_L, rotAbout(axis, M_PI - d));
    EXPECT_TRUE(r.rotation_direction_ambiguous) << "delta = " << d;
  }
  // Exactly pi must of course be flagged.
  const dual_arm::ClosureResidual exact =
      dual_arm::closureResidual(object_L, rotAbout(axis, M_PI));
  EXPECT_TRUE(exact.rotation_direction_ambiguous);
}

// A hard threshold on the midpoint margin decides by the value, provided the
// margin is kept outside the unresolvable window.
TEST(LogSo3NormStability, TheMidpointGuardDecidesByTheValueAcrossItsThreshold) {
  const Vec3 axis = Vec3(0.2, 0.9, -0.35).normalized();
  const double threshold = dual_arm::kMidpointSafeMinMarginRad;  // 1.5e-3 rad

  for (double margin : {1e-4, 1e-3, threshold - 1e-6, threshold, threshold + 1e-6, 1e-2}) {
    const Iso3 L = Iso3::Identity();
    const Iso3 R = rotAbout(axis, M_PI - margin);
    const double measured = M_PI - dual_arm::relativeRotationAngle(L, R);
    EXPECT_NEAR(measured, margin, 1e-6) << "margin = " << margin;
    // MEASURED: isMidpointSafe / midpointUnsafeReason / midpointMargin all use
    // an INCLUSIVE '>=' -- a margin exactly equal to the threshold is SAFE.
    const bool expected_safe = margin >= threshold;
    EXPECT_EQ(dual_arm::isMidpointSafe(L, R), expected_safe) << "margin = " << margin;
    EXPECT_EQ(dual_arm::midpointMargin(L, R) >= threshold, expected_safe) << "margin = " << margin;
    EXPECT_EQ(dual_arm::midpointUnsafeReason(L, R).empty(), expected_safe) << "margin = " << margin;
  }
}

TEST(LogSo3NormStability, ClosureJacobianIsFiniteRightNextToPi) {
  const Vec3 axis = Vec3(0.5, 0.5, 0.5).normalized();
  for (double d : {1e-3, 1e-5, 1e-7}) {
    const Iso3 object_L = Iso3::Identity();
    const Iso3 object_R = rotAbout(axis, M_PI - d);
    const dual_arm::Mat6 J = dual_arm::Mat6::Identity();
    const dual_arm::Mat6x12 Jc = dual_arm::closureErrorJacobian(object_L, object_R, J, J);
    EXPECT_TRUE(Jc.allFinite()) << "delta = " << d;
    EXPECT_GT(Jc.norm(), 0.0);
  }
}

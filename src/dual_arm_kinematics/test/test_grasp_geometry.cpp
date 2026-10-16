// test/test_grasp_geometry.cpp
// C4.3 S1 / T1 — parametric grasp geometry: legal inputs are accepted, illegal
// inputs are REJECTED (never clamped, never silently repaired).
//
// Requirement source: 正式执行规划 §4.1 (parametric box, no hard-coded M4/M5
// dimension) and C4.3 S0 report §A7 T1.  User adjustment (m01472) #1: the new
// task-given grasp interface MUST support an arm-relative rotation of exactly π.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "dual_arm_kinematics/grasp_geometry.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace {

using dual_arm::GraspGeometry;
using dual_arm::Iso3;
using dual_arm::Mat3;
using dual_arm::Vec3;

/// A legal, generic grasp: 4 cm box, left TCP on +x of the object, right TCP on
/// -x and rotated by π about z (the two jaws face each other).
GraspGeometry legalGrasp() {
    GraspGeometry g;
    const double a = 0.04;
    g.size_xyz = Vec3(a, a, a);
    g.mass = 0.0612;
    const double I = g.mass / 12.0 * (a * a + a * a);
    g.inertia = I * Mat3::Identity();
    g.world_T_object_star = Iso3::Identity();
    g.object_T_left_tcp = Iso3(Eigen::Translation3d(0.03, 0.0, 0.0));
    g.object_T_right_tcp = Iso3(Eigen::Translation3d(-0.03, 0.0, 0.0)) *
                           Iso3(Eigen::AngleAxisd(M_PI, Vec3::UnitZ()));
    return g;
}

TEST(GraspGeometry, LegalGraspIsAccepted) {
    const GraspGeometry g = legalGrasp();
    EXPECT_EQ(g.validate(), "") << g.validate();
}

TEST(GraspGeometry, LegalGraspHasRelativeRotationExactlyPi) {
    // Both jaws grip the same box from opposite sides: the arm-relative rotation
    // is exactly π, and that MUST be a legal input.
    const GraspGeometry g = legalGrasp();
    const Vec3 xi = dual_arm::logSO3(g.object_T_left_tcp.linear().transpose() *
                                     g.object_T_right_tcp.linear());
    std::printf("[C4.3 S1 T1] legal grasp relative rotation ||xi|| = %.17g (pi = %.17g)\n",
                xi.norm(), M_PI);
    EXPECT_NEAR(xi.norm(), M_PI, 1e-12);
    EXPECT_EQ(g.validate(), "");
}

TEST(GraspGeometry, RejectsNonPositiveSize) {
    GraspGeometry g = legalGrasp();
    g.size_xyz = Vec3(0.04, 0.0, 0.04);
    EXPECT_NE(g.validate(), "");
    g.size_xyz = Vec3(0.04, -0.01, 0.04);
    EXPECT_NE(g.validate(), "");
    g.size_xyz = Vec3(0.04, 0.04, 0.04);
    EXPECT_EQ(g.validate(), "");
}

TEST(GraspGeometry, RejectsEmptyAndCollidingFrameIds) {
    GraspGeometry g = legalGrasp();
    g.world_frame = "";
    EXPECT_NE(g.validate(), "");
    g = legalGrasp();
    g.object_frame = "";
    EXPECT_NE(g.validate(), "");
    g = legalGrasp();
    g.left_tcp_frame = "";
    EXPECT_NE(g.validate(), "");
    g = legalGrasp();
    g.right_tcp_frame = "";
    EXPECT_NE(g.validate(), "");
    // Same id on both TCPs is ambiguous, not merely unusual.
    g = legalGrasp();
    g.right_tcp_frame = g.left_tcp_frame;
    EXPECT_NE(g.validate(), "");
    // Object frame may not alias a TCP frame.
    g = legalGrasp();
    g.object_frame = g.left_tcp_frame;
    EXPECT_NE(g.validate(), "");
}

TEST(GraspGeometry, RejectsNonRotationMatrix) {
    GraspGeometry g = legalGrasp();
    g.world_T_object_star.linear() = Mat3::Identity() * 2.0;  // not orthonormal
    EXPECT_NE(g.validate(), "");

    g = legalGrasp();
    g.object_T_left_tcp.linear() = Mat3::Identity();
    g.object_T_left_tcp.linear()(0, 0) = 0.99;  // slightly non-orthonormal
    EXPECT_NE(g.validate(), "");

    // A reflection: orthonormal but det = -1, i.e. not in SO(3).
    g = legalGrasp();
    g.object_T_right_tcp.linear() = Mat3::Identity();
    g.object_T_right_tcp.linear()(2, 2) = -1.0;
    EXPECT_NE(g.validate(), "");
    EXPECT_FALSE(dual_arm::isRotationMatrix(g.object_T_right_tcp.linear()));
}

TEST(GraspGeometry, RejectsNonInvertibleTransform) {
    GraspGeometry g = legalGrasp();
    g.world_T_object_star.linear() = Mat3::Zero();  // singular
    EXPECT_NE(g.validate(), "");
    EXPECT_FALSE(dual_arm::isRigidTransform(g.world_T_object_star));
}

TEST(GraspGeometry, RejectsNonFiniteNumbers) {
    GraspGeometry g = legalGrasp();
    g.world_T_object_star.translation() = Vec3(0.0, std::nan(""), 0.0);
    EXPECT_NE(g.validate(), "");
    g = legalGrasp();
    g.mass = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NE(g.validate(), "");
    g = legalGrasp();
    g.object_T_left_tcp.translation() = Vec3(std::numeric_limits<double>::infinity(), 0.0, 0.0);
    EXPECT_NE(g.validate(), "");
}

TEST(GraspGeometry, RejectsNonPositiveDefiniteInertia) {
    GraspGeometry g = legalGrasp();
    g.inertia(1, 1) = -1.0;
    EXPECT_NE(g.validate(), "");
    g = legalGrasp();
    g.inertia(0, 1) = g.inertia(1, 0) = 0.0;
    g.inertia = Mat3::Zero();
    EXPECT_NE(g.validate(), "");
    g = legalGrasp();
    g.inertia(0, 1) = 1e-3;  // asymmetric
    EXPECT_NE(g.validate(), "");
}

TEST(GraspGeometry, RejectsIdenticalLeftAndRightGraspTransforms) {
    GraspGeometry g = legalGrasp();
    g.object_T_right_tcp = g.object_T_left_tcp;
    EXPECT_NE(g.validate(), "");
    // The frozen relative grasp would then be the identity: a vacuous C4.3 query.
    EXPECT_TRUE(g.leftTrightStar().matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-12));
}

TEST(GraspGeometry, RejectsNonPositiveMass) {
    GraspGeometry g = legalGrasp();
    g.mass = 0.0;
    EXPECT_NE(g.validate(), "");
    g.mass = -1.0;
    EXPECT_NE(g.validate(), "");
}

TEST(GraspGeometry, FrozenRelativeGraspMatchesTheDefinition) {
    const GraspGeometry g = legalGrasp();
    const Iso3 expected = g.object_T_left_tcp.inverse() * g.object_T_right_tcp;
    EXPECT_TRUE(g.leftTrightStar().matrix().isApprox(expected.matrix(), 1e-15));
}

}  // namespace

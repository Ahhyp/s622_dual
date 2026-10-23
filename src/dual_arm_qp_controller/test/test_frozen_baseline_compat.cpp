// C4.3 S4.1 W2 addendum -- Task C: frozen grasp-transform compatibility.
//
// Why this file exists: during W2 the `TransportTaskConfig` default for
// `object_T_left_tcp` / `object_T_right_tcp` was `Iso3::Identity()` while the
// controller's ROS parameter defaults held the frozen S2 baseline.  Every
// caller that used `TransportTask` directly with default config therefore got a
// wrong object frame -- a silent, plausible-looking error (it made the object
// task demand a pi rotation).  The defaults now come from a single header.
//
// The six checks the review asked for, one test each:
//   C1  config defaults == the S2 frozen baseline values (yaml cross-check)
//   C2  an explicit override is not overwritten by the default
//   C3  left and right are not swapped
//   C4  frame semantics: the matrices are object_T_tcp (rigid, det == +1)
//   C5  the closed chain G_L^-1 * G_R is the value S3 measured
//   C6  no production path silently uses Identity as a grasp default

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_object_task/anchored_object_task.hpp"
#include "dual_arm_qp_controller/frozen_baseline.hpp"
#include "dual_arm_qp_controller/transport_task.hpp"

namespace {

using dual_arm::Iso3;

/// Parse `object_T_left_tcp: "r,c,r,c;..."` (4 rows of 4) out of the S2 yaml.
std::array<double, 16> parseGraspFromYaml(const std::string& path, const std::string& key,
                                          bool* ok) {
    std::array<double, 16> out{};
    *ok = false;
    std::ifstream in(path);
    if (!in.good()) return out;
    std::string line;
    while (std::getline(in, line)) {
        const auto p = line.find(key + ":");
        if (p == std::string::npos) continue;
        auto q0 = line.find('"', p);
        auto q1 = line.find('"', q0 + 1);
        if (q0 == std::string::npos || q1 == std::string::npos) return out;
        const std::string body = line.substr(q0 + 1, q1 - q0 - 1);
        std::vector<double> vals;
        std::stringstream rows(body);
        std::string row;
        while (std::getline(rows, row, ';')) {
            std::stringstream cells(row);
            std::string cell;
            while (std::getline(cells, cell, ',')) {
                if (!cell.empty()) vals.push_back(std::stod(cell));
            }
        }
        if (vals.size() != 16) return out;
        for (size_t i = 0; i < 16; ++i) out[i] = vals[i];
        *ok = true;
        return out;
    }
    return out;
}

}  // namespace

// C1 ----------------------------------------------------------------------
TEST(FrozenBaseline, C1_HeaderConstantsMatchTheS2FrozenYaml) {
    bool ok = false;
    const auto yaml_left = parseGraspFromYaml(S4_BASELINE_YAML, "object_T_left_tcp", &ok);
    ASSERT_TRUE(ok) << "cannot parse object_T_left_tcp from " << S4_BASELINE_YAML;
    const auto yaml_right = parseGraspFromYaml(S4_BASELINE_YAML, "object_T_right_tcp", &ok);
    ASSERT_TRUE(ok) << "cannot parse object_T_right_tcp from " << S4_BASELINE_YAML;

    const auto vec_left = dual_arm_qp_controller::frozenObjectTLeftTcpVector();
    const auto vec_right = dual_arm_qp_controller::frozenObjectTRightTcpVector();
    ASSERT_EQ(vec_left.size(), 16U);
    ASSERT_EQ(vec_right.size(), 16U);
    for (size_t i = 0; i < 16; ++i) {
        // Exact: both sides must carry the same double, not merely a close one.
        EXPECT_EQ(vec_left[i], yaml_left[i]) << "left[" << i << "]";
        EXPECT_EQ(vec_right[i], yaml_right[i]) << "right[" << i << "]";
    }
}

// C6 ----------------------------------------------------------------------
TEST(FrozenBaseline, C6_ConfigDefaultIsNeverIdentity) {
    const dual_arm_qp_controller::TransportTaskConfig cfg;
    // The regression this pins: an Identity default silently puts the object
    // frame on the TCP frame, which asks the QP for a pi rotation.
    EXPECT_FALSE(cfg.object_T_left_tcp.isApprox(Iso3::Identity()))
        << "the default grasp must not be Identity (W2 bug)";
    EXPECT_FALSE(cfg.object_T_right_tcp.isApprox(Iso3::Identity()));
    EXPECT_TRUE(cfg.object_T_left_tcp.isApprox(
        dual_arm_qp_controller::frozenObjectTLeftTcp()));
    EXPECT_TRUE(cfg.object_T_right_tcp.isApprox(
        dual_arm_qp_controller::frozenObjectTRightTcp()));
    // And the ROS parameter default (a std::vector<double>) is the same 16
    // numbers as the yaml, so the controller and the library cannot drift.
    bool ok = false;
    const auto yaml_left = parseGraspFromYaml(S4_BASELINE_YAML, "object_T_left_tcp", &ok);
    ASSERT_TRUE(ok);
    const auto vec = dual_arm_qp_controller::frozenObjectTLeftTcpVector();
    ASSERT_EQ(vec.size(), 16U);
    for (size_t i = 0; i < 16; ++i) EXPECT_EQ(vec[i], yaml_left[i]) << "param[" << i << "]";
}

// C2 ----------------------------------------------------------------------
TEST(FrozenBaseline, C2_ExplicitOverrideIsNotOverwritten) {
    const dual_arm::DualArmKinematics kin;
    Iso3 custom = dual_arm_qp_controller::frozenObjectTLeftTcp();
    custom.translation() += Eigen::Vector3d(0.001, -0.002, 0.003);

    const dual_arm_object_task::AnchoredObjectTaskBuilder builder(
        kin, custom, dual_arm_qp_controller::frozenObjectTRightTcp());
    const double exact =
        (builder.leftGrasp().matrix() - custom.matrix()).cwiseAbs().maxCoeff();
    EXPECT_EQ(exact, 0.0) << "an explicit G_L must be used verbatim";
    EXPECT_GT((builder.leftGrasp().matrix() -
               dual_arm_qp_controller::frozenObjectTLeftTcp().matrix())
                  .cwiseAbs()
                  .maxCoeff(),
              0.0)
        << "the override must differ from the default, otherwise this test is vacuous";
}

// C3 ----------------------------------------------------------------------
TEST(FrozenBaseline, C3_LeftAndRightAreNotSwapped) {
    const auto gl = dual_arm_qp_controller::frozenObjectTLeftTcp();
    const auto gr = dual_arm_qp_controller::frozenObjectTRightTcp();
    // The object frame sits between the two TCPs: +x for the left, -x for the
    // right (the object's long axis is +x, S2 config header).
    EXPECT_NEAR(gl.translation().x(), 0.0497, 1e-15);
    EXPECT_NEAR(gr.translation().x(), -0.0497, 1e-15);
    EXPECT_NEAR(gl.translation().y(), 0.0, 1e-15);
    EXPECT_NEAR(gr.translation().y(), 0.0, 1e-15);
    EXPECT_NEAR(gl.translation().z(), 0.025, 1e-15);
    EXPECT_NEAR(gr.translation().z(), 0.025, 1e-15);
}

// C4 ----------------------------------------------------------------------
TEST(FrozenBaseline, C4_GraspTransformsAreRigidObjectToTcp) {
    for (const auto* g : {&dual_arm_qp_controller::kFrozenObjectTLeftTcp,
                          &dual_arm_qp_controller::kFrozenObjectTRightTcp}) {
        const Iso3 G = dual_arm_qp_controller::iso3FromRowMajor16(*g);
        EXPECT_NEAR(G.linear().determinant(), 1.0, 1e-15) << "must be a rotation";
        EXPECT_TRUE(G.linear().transpose().isApprox(G.linear().inverse(), 1e-15))
            << "must be orthonormal";
        EXPECT_TRUE(G.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 0.0))
            << "row-major parse must preserve the homogeneous row";
    }
}

// C5 ----------------------------------------------------------------------
TEST(FrozenBaseline, C5_ClosedChainIsTheMeasuredS3Value) {
    const dual_arm::DualArmKinematics kin;
    const dual_arm_object_task::AnchoredObjectTaskBuilder builder(
        kin, dual_arm_qp_controller::frozenObjectTLeftTcp(),
        dual_arm_qp_controller::frozenObjectTRightTcp());
    const Iso3 ltr = builder.leftTrightGrasp();
    // S3 measured: R = I3, p = (0, -0.0994, 0).
    EXPECT_TRUE(ltr.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-15));
    EXPECT_NEAR(ltr.translation().x(), 0.0, 1e-15);
    EXPECT_NEAR(ltr.translation().y(), -0.0994, 1e-15);
    EXPECT_NEAR(ltr.translation().z(), 0.0, 1e-15);
    std::printf("frozen baseline: |L T R* - (I3, (0,-0.0994,0))| = %.3e\n",
                (ltr.translation() - Eigen::Vector3d(0.0, -0.0994, 0.0)).norm());
}

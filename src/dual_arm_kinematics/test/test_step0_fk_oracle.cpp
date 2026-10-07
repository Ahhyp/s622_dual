// test/test_step0_fk_oracle.cpp
// Step 0 / S0.1, S0.2, S0.5:
//   - DH FK vs KDL oracle (per-arm base frame and world frame)
//   - DH analytical Jacobian vs KDL Jacobian and vs finite differences
//   - tool-chain consistency (d6 = 0.1 m, gripper tool = 0.1168 m)
//   - rough timing report

#include <gtest/gtest.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>
#include <tinyxml2.h>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"

namespace {

constexpr int kNumSamples = 500;
constexpr double kH = 1e-6;
constexpr double kDhVsUrdfTol = 2e-5;  // see NOTE above
// NOTE (S0.1 acceptance): the URDF fixture and the DH table are *not* bit
// identical by construction -- the SolidWorks-exported URDF rounds
//   - j4 link length to -0.24001 m  (DH a3 = -0.240,  delta = 1e-5 m)
//   - j2/j5/j6 rpy     to  1.5708 rad (exact pi/2,   delta = 3.7e-6 rad each)
// which produces a measured FK/Jacobian agreement of ~1.2e-5 m / ~1.1e-5 rad.
// That is the expected floor; the threshold below is 2e-5. A genuine frame or
// convention error would be orders of magnitude larger.


// Joint limits (from docs/AI_CONTEXT.md / URDF).
const double kLower[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kUpper[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

dual_arm::JointConfig randomQ(std::mt19937& rng) {
    dual_arm::JointConfig q;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 6; ++i) {
        q[i] = kLower[i] + unit(rng) * (kUpper[i] - kLower[i]);
    }
    return q;
}

Eigen::Isometry3d kdlFrameToIso(const KDL::Frame& F) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond quat;
    F.M.GetQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
    T.linear() = quat.toRotationMatrix();
    T.translation() = Eigen::Vector3d(F.p.x(), F.p.y(), F.p.z());
    return T;
}

double rotationAngleDeg(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
    const double c = std::min(1.0, std::max(-1.0, (0.5 * ((R1.transpose() * R2).trace() - 1.0))));
    return std::acos(c) * 180.0 / M_PI;
}

/// Independent oracle: KDL chain built directly from the generated dual-arm URDF.
class KdlOracle {
public:
    explicit KdlOracle(const std::string& urdf_path) {
        if (!kdl_parser::treeFromFile(urdf_path, tree_)) {
            throw std::runtime_error("failed to parse KDL tree from " + urdf_path);
        }
        for (int i = 0; i < 2; ++i) {
            const std::string prefix = (i == 0) ? "left" : "right";
            if (!tree_.getChain(prefix + "_base_link", prefix + "_grasp_frame", base_chain_[i])) {
                throw std::runtime_error("failed to get base chain for " + prefix);
            }
            if (!tree_.getChain("world", prefix + "_grasp_frame", world_chain_[i])) {
                throw std::runtime_error("failed to get world chain for " + prefix);
            }
            buildOrder(base_chain_[i], order_[i]);
            buildOrder(world_chain_[i], world_order_[i]);
        }
    }

    Eigen::Isometry3d fkBase(int arm, const dual_arm::JointConfig& q) const {
        return fk(q, base_chain_[arm], order_[arm]);
    }
    Eigen::Isometry3d fkWorld(int arm, const dual_arm::JointConfig& q) const {
        return fk(q, world_chain_[arm], world_order_[arm]);
    }
    Eigen::Matrix<double, 6, 6> jacBase(int arm, const dual_arm::JointConfig& q) const {
        const KDL::Chain& chain = base_chain_[arm];
        KDL::JntArray qk(chain.getNrOfJoints());
        for (unsigned k = 0; k < chain.getNrOfJoints(); ++k) {
            qk(k) = q[static_cast<Eigen::Index>(order_[arm][k])];
        }
        KDL::ChainJntToJacSolver solver(chain);
        KDL::Jacobian J(chain.getNrOfJoints());
        solver.JntToJac(qk, J);
        Eigen::Matrix<double, 6, 6> out = Eigen::Matrix<double, 6, 6>::Zero();
        for (unsigned k = 0; k < chain.getNrOfJoints(); ++k) {
            out.col(static_cast<Eigen::Index>(order_[arm][k])) = J.data.col(k);
        }
        return out;
    }

private:
    static void buildOrder(const KDL::Chain& chain, std::vector<int>& order) {
        order.clear();
        for (unsigned s = 0; s < chain.getNrOfSegments(); ++s) {
            const KDL::Joint& j = chain.getSegment(s).getJoint();
            if (j.getType() == KDL::Joint::None) continue;
            const std::string name = j.getName();
            // names look like "left_j3" / "right_j5"
            const int idx = name.back() - '1';
            order.push_back(idx);
        }
    }

    Eigen::Isometry3d fk(const dual_arm::JointConfig& q, const KDL::Chain& chain,
                         const std::vector<int>& order) const {
        KDL::JntArray qk(chain.getNrOfJoints());
        for (unsigned k = 0; k < chain.getNrOfJoints(); ++k) {
            qk(k) = q[static_cast<Eigen::Index>(order[k])];
        }
        KDL::ChainFkSolverPos_recursive solver(chain);
        KDL::Frame F;
        solver.JntToCart(qk, F);
        return kdlFrameToIso(F);
    }

    KDL::Tree tree_;
    KDL::Chain base_chain_[2];
    KDL::Chain world_chain_[2];
    std::vector<int> order_[2];
    std::vector<int> world_order_[2];
};

dual_arm::JointConfig makeQ(int arm) {
    // A deterministic pose used for the tool-chain consistency check.
    dual_arm::JointConfig q;
    q << 0.3, -0.8, 0.7, 0.2, -0.5, 0.4;
    (void)arm;
    return q;
}

}  // namespace

TEST(Step0FkOracle, FkBaseMatchesKdl) {
    KdlOracle oracle(DUAL_ARM_FIXTURE);
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20260924u);

    double max_pos = 0.0;
    double max_rot = 0.0;
    for (int n = 0; n < kNumSamples; ++n) {
        const dual_arm::JointConfig q = randomQ(rng);
        for (int arm = 0; arm < 2; ++arm) {
            const dual_arm::Arm a = (arm == 0) ? dual_arm::Arm::Left : dual_arm::Arm::Right;
            const Eigen::Isometry3d dh = kin.baseTgrasp(a, q);
            const Eigen::Isometry3d kd = oracle.fkBase(arm, q);
            max_pos = std::max(max_pos, (dh.translation() - kd.translation()).norm());
            max_rot = std::max(max_rot, rotationAngleDeg(dh.linear(), kd.linear()));
        }
    }
    std::printf("[S0.1] FK(base) vs KDL: max pos = %.3e m, max rot = %.3e deg\n", max_pos, max_rot);
    EXPECT_LT(max_pos, kDhVsUrdfTol);
    EXPECT_LT(max_rot, 1e-3);  // deg
}

TEST(Step0FkOracle, FkWorldMatchesKdl) {
    KdlOracle oracle(DUAL_ARM_FIXTURE);
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20260925u);

    double max_pos = 0.0;
    double max_rot = 0.0;
    for (int n = 0; n < kNumSamples; ++n) {
        const dual_arm::JointConfig q = randomQ(rng);
        for (int arm = 0; arm < 2; ++arm) {
            const dual_arm::Arm a = (arm == 0) ? dual_arm::Arm::Left : dual_arm::Arm::Right;
            const Eigen::Isometry3d dh = kin.worldTgrasp(a, q);
            const Eigen::Isometry3d kd = oracle.fkWorld(arm, q);
            max_pos = std::max(max_pos, (dh.translation() - kd.translation()).norm());
            max_rot = std::max(max_rot, rotationAngleDeg(dh.linear(), kd.linear()));
        }
    }
    std::printf("[S0.1] FK(world) vs KDL: max pos = %.3e m, max rot = %.3e deg\n", max_pos, max_rot);
    EXPECT_LT(max_pos, kDhVsUrdfTol);
    EXPECT_LT(max_rot, 1e-3);  // deg
}

TEST(Step0FkOracle, JacobianMatchesKdl) {
    KdlOracle oracle(DUAL_ARM_FIXTURE);
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20260926u);

    double max_err = 0.0;
    for (int n = 0; n < kNumSamples; ++n) {
        const dual_arm::JointConfig q = randomQ(rng);
        for (int arm = 0; arm < 2; ++arm) {
            const dual_arm::Arm a = (arm == 0) ? dual_arm::Arm::Left : dual_arm::Arm::Right;
            const Eigen::Matrix<double, 6, 6> J_dh = kin.jacobianBase(a, q);
            const Eigen::Matrix<double, 6, 6> J_kdl = oracle.jacBase(arm, q);
            max_err = std::max(max_err, (J_dh - J_kdl).cwiseAbs().maxCoeff());
        }
    }
    std::printf("[S0.1] J(base) vs KDL: max abs diff = %.3e\n", max_err);
    EXPECT_LT(max_err, kDhVsUrdfTol);
}

TEST(Step0FkOracle, JacobianMatchesFiniteDifference) {
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20260927u);

    double max_err = 0.0;
    for (int n = 0; n < 100; ++n) {
        const dual_arm::JointConfig q = randomQ(rng);
        for (int arm = 0; arm < 2; ++arm) {
            const dual_arm::Arm a = (arm == 0) ? dual_arm::Arm::Left : dual_arm::Arm::Right;
            const Eigen::Matrix<double, 6, 6> J = kin.jacobianBase(a, q);
            Eigen::Matrix<double, 6, 6> J_fd = Eigen::Matrix<double, 6, 6>::Zero();
            for (int j = 0; j < 6; ++j) {
                dual_arm::JointConfig qp = q;
                dual_arm::JointConfig qm = q;
                qp[j] += kH;
                qm[j] -= kH;
                const Eigen::Isometry3d Tp = kin.baseTgrasp(a, qp);
                const Eigen::Isometry3d Tm = kin.baseTgrasp(a, qm);
                const Eigen::Vector3d dp = (Tp.translation() - Tm.translation()) / (2.0 * kH);
                const Eigen::Matrix3d dR = (Tp.linear() - Tm.linear()) / (2.0 * kH);
                const Eigen::Matrix3d W = dR * Tp.linear().transpose();
                const Eigen::Vector3d w(W(2, 1), W(0, 2), W(1, 0));
                J_fd.block<3, 1>(0, j) = dp;
                J_fd.block<3, 1>(3, j) = w;
            }
            max_err = std::max(max_err, (J - J_fd).cwiseAbs().maxCoeff());
        }
    }
    std::printf("[S0.1] J(base) vs finite difference: max abs diff = %.3e\n", max_err);
    EXPECT_LT(max_err, 1e-5);
}

TEST(Step0ToolChain, Consistency) {
    // URDF: wrist3_link -> grasp_frame is a fixed translation of 0.2168 m on Z.
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.LoadFile(DUAL_ARM_FIXTURE), tinyxml2::XML_SUCCESS);
    int found = 0;
    for (auto* joint = doc.FirstChildElement("robot")->FirstChildElement("joint"); joint != nullptr;
         joint = joint->NextSiblingElement("joint")) {
        const char* child = joint->FirstChildElement("child")->Attribute("link");
        if (child == nullptr) continue;
        const std::string c(child);
        if (c == "left_grasp_frame" || c == "right_grasp_frame") {
            const char* xyz = joint->FirstChildElement("origin")->Attribute("xyz");
            double x = 0.0, y = 0.0, z = 0.0;
            ASSERT_EQ(std::sscanf(xyz, "%lf %lf %lf", &x, &y, &z), 3);
            EXPECT_NEAR(z, 0.2168, 1e-12);
            ++found;
        }
    }
    EXPECT_EQ(found, 2);

    // DH: frame6 (flange) + gripper tool (0.1168) == grasp frame.
    dual_arm::DualArmKinematics kin;
    const dual_arm::JointConfig q = makeQ(0);
    Eigen::Isometry3d flange = Eigen::Isometry3d::Identity();
    flange.matrix() = kin.dh(dual_arm::Arm::Left).fkineFlange(q);
    const Eigen::Isometry3d grasp = kin.baseTgrasp(dual_arm::Arm::Left, q);
    const Eigen::Isometry3d tool = flange.inverse() * grasp;
    EXPECT_NEAR(tool.translation().norm(), 0.1168, 1e-12);
    EXPECT_NEAR(tool.translation().z(), 0.1168, 1e-12);
    std::printf("[S0.2] tool offset = %.6f m, flange->grasp = %.6f m\n",
                tool.translation().norm(), tool.translation().norm() + 0.1);
}

TEST(Step0Performance, TimingReport) {
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(1u);
    const dual_arm::JointConfig q = randomQ(rng);

    constexpr int kN = 20000;
    auto t0 = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (int i = 0; i < kN; ++i) {
        sink += kin.worldTgrasp(dual_arm::Arm::Left, q).translation().x();
    }
    auto t1 = std::chrono::steady_clock::now();
    for (int i = 0; i < kN; ++i) {
        sink += kin.jacobianBase(dual_arm::Arm::Right, q).norm();
    }
    auto t2 = std::chrono::steady_clock::now();
    const double us_fk = std::chrono::duration<double, std::micro>(t1 - t0).count() / kN;
    const double us_j = std::chrono::duration<double, std::micro>(t2 - t1).count() / kN;
    std::printf("[S0.5] DH FK = %.2f us/call, DH Jacobian = %.2f us/call (sink=%.3f)\n",
                us_fk, us_j, sink);
    EXPECT_LT(us_fk, 1000.0);  // loose sanity bound only
    EXPECT_LT(us_j, 1000.0);
}

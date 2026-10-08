// include/dual_arm_collision/dual_arm_collision.hpp
// C2.7a: dual-arm distance monitoring (FCL).
//
// Collision geometry: one CAPSULE per DH link segment (frames 0-1, 1-2, ...,
// 5-6) with a configurable radius.  This is a deliberate simplification: the
// monitor only needs a conservative, smooth-ish distance proxy; the safety
// layer (C2.7c) keeps an independent hard threshold regardless.

#pragma once

#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"

namespace dual_arm_collision {

using dual_arm::DualArmKinematics;
using dual_arm::JointConfig;
using dual_arm::Q12;
using Vec12 = Eigen::Matrix<double, 12, 1>;

/// Minimum arm-arm distance and the associated geometry.
struct DistanceInfo {
    double distance = std::numeric_limits<double>::max();
    Eigen::Vector3d p1 = Eigen::Vector3d::Zero();  ///< nearest point on the left arm
    Eigen::Vector3d p2 = Eigen::Vector3d::Zero();  ///< nearest point on the right arm
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();  ///< unit, p1 -> p2
    int link1 = -1;  ///< capsule index on the left arm (0..5)
    int link2 = -1;  ///< capsule index on the right arm (0..5)
    bool valid = false;
};

class DualArmCollisionModel {
public:
    static constexpr int kCapsulesPerArm = 6;

    explicit DualArmCollisionModel(const DualArmKinematics& kinematics,
                                   double link_radius = 0.06);

    /// Minimum distance between the two arms (all capsule pairs).
    DistanceInfo distance(const Q12& q) const;

    /// Gradient of the minimum distance w.r.t. the 12 joints [m/rad].
    /// Computed by central differences on the CURRENTLY closest capsule pair
    /// (cheap: 12 single-pair queries).  If the closest pair changes within the
    /// step this is only approximate -- see the test that compares it against a
    /// full all-pairs finite difference.
    Vec12 distanceGradient(const Q12& q, double h = 1e-4) const;

    double linkRadius() const { return radius_; }

    /// One collision capsule (world frame + cylindrical length).
    struct Capsule {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        double length = 0.0;
    };

private:
    /// Capsule for DH segment `index` (frames index -> index+1) of one arm.
    Capsule capsuleAt(int arm, const JointConfig& q, int index) const;

    /// Distance for one specific capsule pair.
    double pairDistance(const Capsule& a, const Capsule& b,
                        Eigen::Vector3d* p_a = nullptr, Eigen::Vector3d* p_b = nullptr) const;

    const DualArmKinematics& kin_;
    double radius_;
};

}  // namespace dual_arm_collision

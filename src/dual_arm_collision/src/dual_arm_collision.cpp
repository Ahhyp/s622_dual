// src/dual_arm_collision.cpp

#include "dual_arm_collision/dual_arm_collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <fcl/fcl.h>
#include <fcl/narrowphase/distance.h>

namespace dual_arm_collision {

namespace {
dual_arm::Arm toArm(int arm) {
    return (arm == 0) ? dual_arm::Arm::Left : dual_arm::Arm::Right;
}
}  // namespace

DualArmCollisionModel::DualArmCollisionModel(const DualArmKinematics& kinematics,
                                             double link_radius)
    : kin_(kinematics), radius_(link_radius > 0.0 ? link_radius : 0.06) {}

DualArmCollisionModel::Capsule DualArmCollisionModel::capsuleAt(int arm, const JointConfig& q,
                                                                int index) const {
    Capsule c;
    const dual_arm::Arm a = toArm(arm);
    const Eigen::Vector3d p0 = kin_.worldTframe(a, q, index).translation();
    const Eigen::Vector3d p1 = kin_.worldTframe(a, q, index + 1).translation();
    const Eigen::Vector3d d = p1 - p0;
    c.length = d.norm();
    c.transform.translation() = 0.5 * (p0 + p1);
    if (c.length > 1e-9) {
        c.transform.linear() =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), d / c.length)
                .toRotationMatrix();
    }
    return c;
}

double DualArmCollisionModel::pairDistance(const Capsule& a, const Capsule& b,
                                           Eigen::Vector3d* p_a, Eigen::Vector3d* p_b) const {
    const fcl::Capsule<double> ca(radius_, std::max(1e-6, a.length));
    const fcl::Capsule<double> cb(radius_, std::max(1e-6, b.length));
    fcl::DistanceRequest<double> request;
    request.enable_nearest_points = true;
    fcl::DistanceResult<double> result;
    const double d = fcl::distance(&ca, a.transform, &cb, b.transform, request, result);
    if (p_a != nullptr) *p_a = result.nearest_points[0];
    if (p_b != nullptr) *p_b = result.nearest_points[1];
    return d;
}

DistanceInfo DualArmCollisionModel::distance(const Q12& q) const {
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);

    Capsule left[kCapsulesPerArm];
    Capsule right[kCapsulesPerArm];
    for (int i = 0; i < kCapsulesPerArm; ++i) {
        left[i] = capsuleAt(0, ql, i);
        right[i] = capsuleAt(1, qr, i);
    }

    DistanceInfo best;
    for (int i = 0; i < kCapsulesPerArm; ++i) {
        for (int j = 0; j < kCapsulesPerArm; ++j) {
            Eigen::Vector3d pa, pb;
            const double d = pairDistance(left[i], right[j], &pa, &pb);
            if (d < best.distance) {
                best.distance = d;
                best.p1 = pa;
                best.p2 = pb;
                best.link1 = i;
                best.link2 = j;
                best.valid = true;
            }
        }
    }
    if (best.valid) {
        const Eigen::Vector3d dir = best.p2 - best.p1;
        const double n = dir.norm();
        best.normal = (n > 1e-9) ? (dir / n).eval() : Eigen::Vector3d::Zero();
    }
    return best;
}

Vec12 DualArmCollisionModel::distanceGradient(const Q12& q, double h) const {
    Vec12 gradient = Vec12::Zero();
    const DistanceInfo info = distance(q);
    if (!info.valid) return gradient;

    auto pairDistanceAt = [&](const Q12& qq) -> double {
        JointConfig ql, qr;
        DualArmKinematics::split(qq, &ql, &qr);
        const Capsule a = capsuleAt(0, ql, info.link1);
        const Capsule b = capsuleAt(1, qr, info.link2);
        return pairDistance(a, b, nullptr, nullptr);
    };

    for (int k = 0; k < 12; ++k) {
        Q12 qp = q, qm = q;
        qp[k] += h;
        qm[k] -= h;
        gradient[k] = (pairDistanceAt(qp) - pairDistanceAt(qm)) / (2.0 * h);
    }
    return gradient;
}

}  // namespace dual_arm_collision

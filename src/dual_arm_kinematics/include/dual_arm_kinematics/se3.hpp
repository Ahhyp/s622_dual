// include/dual_arm_kinematics/se3.hpp
// Minimal SE(3)/SO(3) utilities for the dual-arm kinematics layer.
//
// Twist convention used throughout this package:
//   xi = [v; w]  in R^6,  linear part first, angular part last.
//   adjoint(T) with T = [R p; 0 1]:
//       Ad(T) = [ R   skew(p) * R ]
//               [ 0        R      ]
//   so that  xi_a = Ad(T_ab) * xi_b  maps a twist expressed in frame b
//   (axes and reference point of b) into frame a.
//
// All "body" twists below follow  xi_body = T^{-1} * dT/dt  (axes + reference
// point of the moving frame). All "spatial" twists follow dT/dt * T^{-1}.

#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace dual_arm {

using Iso3 = Eigen::Isometry3d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;

inline Mat3 skew(const Vec3& v) {
    Mat3 S;
    S << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return S;
}

inline Vec3 vee(const Mat3& S) {
    return Vec3(S(2, 1), S(0, 2), S(1, 0));
}

inline Mat3 expSO3(const Vec3& w) {
    const double th2 = w.squaredNorm();
    const Mat3 W = skew(w);
    if (th2 < 1e-12) {
        return Mat3::Identity() + W + 0.5 * W * W;
    }
    const double th = std::sqrt(th2);
    return Mat3::Identity() + (std::sin(th) / th) * W +
           ((1.0 - std::cos(th)) / th2) * W * W;
}

inline Vec3 logSO3(const Mat3& R) {
    const double cos_th = std::min(1.0, std::max(-1.0, 0.5 * (R.trace() - 1.0)));
    const double th = std::acos(cos_th);
    if (th < 1e-10) {
        return 0.5 * vee(R - R.transpose());
    }
    if (M_PI - th < 1e-6) {
        // Near pi: R = I + 2*W^2 (W = [axis*pi]) => (R + I)/2 = axis*axis^T.
        const Mat3 A = 0.5 * (R + Mat3::Identity());
        int best = 0;
        for (int i = 1; i < 3; ++i) {
            if (A(i, i) > A(best, best)) best = i;
        }
        Vec3 axis = A.col(best) / std::sqrt(std::max(1e-12, A(best, best)));
        return M_PI * axis.normalized();
    }
    return vee((th / (2.0 * std::sin(th))) * (R - R.transpose()));
}

inline Mat3 leftJacobianSO3(const Vec3& w) {
    const double th2 = w.squaredNorm();
    const Mat3 W = skew(w);
    if (th2 < 1e-12) {
        return Mat3::Identity() + 0.5 * W + (1.0 / 6.0) * W * W;
    }
    const double th = std::sqrt(th2);
    return Mat3::Identity() + ((1.0 - std::cos(th)) / th2) * W +
           ((th - std::sin(th)) / (th2 * th)) * W * W;
}

/// J_l(w)^-1 = I - 0.5 [w] + (1/theta^2)(1 - (theta/2) cot(theta/2)) [w]^2
inline Mat3 leftJacobianInverseSO3(const Vec3& w) {
    const double th2 = w.squaredNorm();
    const Mat3 W = skew(w);
    if (th2 < 1e-12) {
        return Mat3::Identity() - 0.5 * W + (1.0 / 12.0) * W * W;
    }
    const double th = std::sqrt(th2);
    const double half = 0.5 * th;
    const double c = (1.0 - half * std::cos(half) / std::sin(half)) / th2;
    return Mat3::Identity() - 0.5 * W + c * W * W;
}

/// J_r(w)^-1 = J_l(-w)^-1 = I + 0.5 [w] + (same coefficient) [w]^2
inline Mat3 rightJacobianInverseSO3(const Vec3& w) {
    const double th2 = w.squaredNorm();
    const Mat3 W = skew(w);
    if (th2 < 1e-12) {
        return Mat3::Identity() + 0.5 * W + (1.0 / 12.0) * W * W;
    }
    const double th = std::sqrt(th2);
    const double half = 0.5 * th;
    const double c = (1.0 - half * std::cos(half) / std::sin(half)) / th2;
    return Mat3::Identity() + 0.5 * W + c * W * W;
}

inline Iso3 expSE3(const Vec6& xi) {
    Iso3 T = Iso3::Identity();
    const Vec3 v = xi.head<3>();
    const Vec3 w = xi.tail<3>();
    T.linear() = expSO3(w);
    T.translation() = leftJacobianSO3(w) * v;
    return T;
}

inline Vec6 logSE3(const Iso3& T) {
    Vec6 xi;
    const Vec3 w = logSO3(T.linear());
    xi.tail<3>() = w;
    xi.head<3>() = leftJacobianInverseSO3(w) * T.translation();
    return xi;
}

inline Mat6 adjoint(const Iso3& T) {
    Mat6 Ad = Mat6::Zero();
    const Mat3 R = T.linear();
    Ad.topLeftCorner<3, 3>() = R;
    Ad.topRightCorner<3, 3>() = skew(T.translation()) * R;
    Ad.bottomRightCorner<3, 3>() = R;
    return Ad;
}

}  // namespace dual_arm

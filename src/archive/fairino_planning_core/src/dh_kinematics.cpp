// dh_kinematics.cpp
#include "fairino_planning_core/dh_kinematics.h"

#include <cmath>

namespace fairino_planning
{

    DHKinematics::DHKinematics(const std::array<DHParam, DOF> &dh_params)
        : dh_params_(dh_params)
    {
    }

    // ---------------------------------------------------------------------------
    // 标准 DH (Classical / Denavit-Hartenberg)
    //
    //   T_i^{i-1} = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    //
    //     | cosθ   -sinθ·cosα    sinθ·sinα    a·cosθ |
    // T = | sinθ    cosθ·cosα   -cosθ·sinα    a·sinθ |
    //     |   0       sinα         cosα         d    |
    //     |   0        0            0           1    |
    //
    // 如果使用改进 DH (Modified / Craig)，矩阵形式不同：
    //   T_i^{i-1} = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
    // ---------------------------------------------------------------------------
    Eigen::Isometry3d DHKinematics::dhTransform(
        double a,
        double alpha,
        double d,
        double theta)
    {
        const double ct = std::cos(theta);
        const double st = std::sin(theta);
        const double ca = std::cos(alpha);
        const double sa = std::sin(alpha);

        Eigen::Matrix4d m;
        m << ct, -st * ca, st * sa, a * ct,
            st, ct * ca, -ct * sa, a * st,
            0.0, sa, ca, d,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Isometry3d T;
        T.matrix() = m;
        return T;
    }

    // ---------------------------------------------------------------------------
    // 正运动学：T_base^tcp = ∏ T_i^{i-1}
    // ---------------------------------------------------------------------------
    Pose DHKinematics::forward(const JointArray &q) const
    {
        Pose T = Pose::Identity();
        for (int i = 0; i < DOF; ++i)
        {
            const auto &p = dh_params_[i];
            T = T * dhTransform(p.a, p.alpha, p.d, p.theta_offset + q[i]);
        }
        return T;
    }

    // ---------------------------------------------------------------------------
    // 所有连杆变换：transforms[0] = I（基坐标系）
    //               transforms[i] = T_base^link_i  (i = 1..DOF)
    // 末端就是 transforms[DOF]。
    // ---------------------------------------------------------------------------
    std::array<Pose, DOF + 1> DHKinematics::linkTransforms(const JointArray &q) const
    {
        std::array<Pose, DOF + 1> transforms;
        transforms[0] = Pose::Identity();
        for (int i = 0; i < DOF; ++i)
        {
            const auto &p = dh_params_[i];
            transforms[i + 1] = transforms[i] * dhTransform(p.a, p.alpha, p.d, p.theta_offset + q[i]);
        }
        return transforms;
    }

} // namespace fairino_planning

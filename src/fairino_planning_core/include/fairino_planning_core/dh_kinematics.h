#pragma once

#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fairino_planning_core/types.h"

namespace fairino_planning
{

    struct DHParam
    {
        double a;
        double alpha;
        double d;
        double theta_offset;
    };

    class DHKinematics
    {
    public:
        DHKinematics(const std::array<DHParam, DOF> &dh_params);

        Pose forward(const JointArray &q) const;

        std::array<Pose, DOF + 1> linkTransforms(const JointArray &q) const;
        
        // 雅可比矩阵 (6x6)，用于奇异值检测
        // Eigen::Matrix<double, 6, DOF> jacobian(const JointArray& q) const;
        
        // 返回 DOF+1 个坐标系的原点在基座标下的 z 值（用于肘部高度代价）
        // 其实就是 linkTransforms 的 wrapper
    private:
        std::array<DHParam, DOF> dh_params_;

        static Eigen::Isometry3d dhTransform(
            double a,
            double alpha,
            double d,
            double theta);
    };

} // namespace fairino_planning